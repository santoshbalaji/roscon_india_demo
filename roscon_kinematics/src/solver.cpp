#include "roscon_kinematics/solver.hpp"
#include <Eigen/Eigenvalues>
#include <Eigen/SVD>
#include <IpIpoptApplication.hpp>
#include <IpTNLP.hpp>
#include <algorithm>
#include <cmath>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <stdexcept>
#include <urdf/model.h>

namespace roscon_kinematics {
Robot::Robot(const std::string &xml, const std::string &base, const std::string &tip,
             double margin) {
  urdf::Model model;
  KDL::Tree tree;
  if (!std::isfinite(margin) || margin < 0.0 || !model.initString(xml) ||
      !kdl_parser::treeFromUrdfModel(model, tree) || !tree.getChain(base, tip, chain_)) {
    throw std::runtime_error("Invalid URDF, chain, or joint margin");
  }
  for (const auto &segment : chain_.segments) {
    if (segment.getJoint().getType() == KDL::Joint::Fixed) {
      continue;
    }
    const auto joint = model.getJoint(segment.getJoint().getName());
    if (!joint || joint->mimic || joint->type != urdf::Joint::REVOLUTE || !joint->limits) {
      throw std::runtime_error("This example supports independent bounded revolute joints only");
    }
    names.push_back(joint->name);
  }
  if (names.size() < 6) {
    throw std::runtime_error("Full-pose singularity constraint requires at least six joints");
  }
  lower.resize(names.size());
  upper.resize(names.size());
  for (size_t i = 0; i < names.size(); ++i) {
    auto limits = model.getJoint(names[i])->limits;
    lower[i] = limits->lower + margin;
    upper[i] = limits->upper - margin;
    if (!std::isfinite(lower[i]) || !std::isfinite(upper[i]) || lower[i] >= upper[i]) {
      throw std::runtime_error("Invalid or empty joint interval");
    }
  }
}
Target Robot::pose(const Eigen::VectorXd &q) const {
  if (q.size() != lower.size() || !q.allFinite()) {
    throw std::runtime_error("Invalid joint vector");
  }
  KDL::JntArray joints(q.size());
  joints.data = q;
  KDL::Frame frame;
  KDL::ChainFkSolverPos_recursive fk(chain_);
  if (fk.JntToCart(joints, frame) < 0) {
    throw std::runtime_error("Forward kinematics failed");
  }
  Target output;
  for (int row = 0; row < 3; ++row) {
    output.position[row] = frame.p[row];
    for (int col = 0; col < 3; ++col) {
      output.rotation(row, col) = frame.M(row, col);
    }
  }
  return output;
}
Eigen::MatrixXd Robot::jacobian(const Eigen::VectorXd &q, double length) const {
  if (q.size() != lower.size() || !q.allFinite() || !std::isfinite(length) || length <= 0) {
    throw std::runtime_error("Invalid Jacobian input");
  }
  KDL::JntArray joints(q.size());
  joints.data = q;
  KDL::Jacobian jac(q.size());
  KDL::ChainJntToJacSolver solver(chain_);
  if (solver.JntToJac(joints, jac) < 0) {
    throw std::runtime_error("Jacobian failed");
  }
  Eigen::MatrixXd scaled = jac.data;
  scaled.topRows(3) /= length;
  return scaled;
}
Eigen::Matrix<double, 6, 6> unpack_factor(const Eigen::VectorXd &x, int dof) {
  Eigen::Matrix<double, 6, 6> factor = Eigen::Matrix<double, 6, 6>::Zero();
  int index = dof;
  for (int row = 0; row < 6; ++row) {
    for (int col = 0; col <= row; ++col) {
      factor(row, col) = x[index++];
    }
  }
  return factor;
}
Eigen::VectorXd factor_residual(const Eigen::MatrixXd &jac,
                                const Eigen::Matrix<double, 6, 6> &factor, double sigma) {
  Eigen::Matrix<double, 6, 6> residual = jac * jac.transpose() - factor * factor.transpose();
  residual.diagonal().array() -= sigma * sigma;
  Eigen::VectorXd values(21);
  int index = 0;
  for (int row = 0; row < 6; ++row) {
    for (int col = 0; col <= row; ++col) {
      values[index++] = residual(row, col);
    }
  }
  return values;
}

class Problem final : public Ipopt::TNLP {
public:
  Problem(const Robot &robot, const Target &target, const Eigen::VectorXd &seed,
          const Settings &settings)
      : robot_(robot), target_(target), seed_(seed), s_(settings) {}
  Eigen::VectorXd solution;
  bool get_nlp_info(Ipopt::Index &n, Ipopt::Index &m, Ipopt::Index &nnz_jac, Ipopt::Index &nnz_h,
                    IndexStyleEnum &style) override {
    n = seed_.size() + 21;
    m = 21;
    nnz_jac = n * m;
    nnz_h = 0;
    style = C_STYLE;
    return true;
  }
  bool get_bounds_info(Ipopt::Index n, double *lower, double *upper, Ipopt::Index m, double *gl,
                       double *gu) override {
    for (int i = 0; i < n; ++i) {
      lower[i] = i < seed_.size() ? robot_.lower[i] : -2e19;
      upper[i] = i < seed_.size() ? robot_.upper[i] : 2e19;
    }
    for (int i = 0; i < m; ++i) {
      gl[i] = gu[i] = 0.0;
    }
    return true;
  }
  bool get_starting_point(Ipopt::Index n, bool init_x, double *x, bool init_z, double *, double *,
                          Ipopt::Index, bool init_lambda, double *) override {
    if (!init_x || init_z || init_lambda) {
      return false;
    }
    Eigen::Map<Eigen::VectorXd> start(x, n);
    start.head(seed_.size()) = seed_;
    const Eigen::MatrixXd jac = robot_.jacobian(seed_, s_.characteristic_length);
    Eigen::Matrix<double, 6, 6> gram = jac * jac.transpose();
    gram.diagonal().array() -= s_.minimum_singular_value * s_.minimum_singular_value;
    Eigen::LLT<Eigen::Matrix<double, 6, 6>> decomposition(gram);
    Eigen::Matrix<double, 6, 6> factor = Eigen::Matrix<double, 6, 6>::Identity();
    if (decomposition.info() == Eigen::Success) {
      factor = decomposition.matrixL();
    }
    int index = seed_.size();
    for (int row = 0; row < 6; ++row) {
      for (int col = 0; col <= row; ++col) {
        start[index++] = factor(row, col);
      }
    }
    return true;
  }
  double objective(const Eigen::VectorXd &x) const {
    const Eigen::VectorXd q = x.head(seed_.size());
    const Target actual = robot_.pose(q);
    return s_.position_weight *
               ((actual.position - target_.position) / s_.position_scale).squaredNorm() +
           0.5 * s_.orientation_weight *
               (target_.rotation.transpose() * actual.rotation - Eigen::Matrix3d::Identity())
                   .squaredNorm() +
           s_.posture_weight * (q - seed_).squaredNorm();
  }
  Eigen::VectorXd constraints(const Eigen::VectorXd &x) const {
    return factor_residual(robot_.jacobian(x.head(seed_.size()), s_.characteristic_length),
                           unpack_factor(x, seed_.size()), s_.minimum_singular_value);
  }
  bool eval_f(Ipopt::Index n, const double *x, bool, double &value) override {
    value = objective(Eigen::Map<const Eigen::VectorXd>(x, n));
    return std::isfinite(value);
  }
  // Explicit central differences keep this educational implementation independent
  // of automatic differentiation libraries. IPOPT approximates only the Hessian.
  bool eval_grad_f(Ipopt::Index n, const double *x, bool, double *gradient) override {
    Eigen::VectorXd point = Eigen::Map<const Eigen::VectorXd>(x, n);
    for (int col = 0; col < n; ++col) {
      if (col >= seed_.size()) {
        gradient[col] = 0.0;
        continue;
      }
      Eigen::VectorXd plus = point, minus = point;
      plus[col] += s_.difference_step;
      minus[col] -= s_.difference_step;
      gradient[col] = (objective(plus) - objective(minus)) / (2 * s_.difference_step);
    }
    return true;
  }
  bool eval_g(Ipopt::Index n, const double *x, bool, Ipopt::Index m, double *g) override {
    Eigen::Map<Eigen::VectorXd> output(g, m);
    output = constraints(Eigen::Map<const Eigen::VectorXd>(x, n));
    return output.allFinite();
  }
  bool eval_jac_g(Ipopt::Index n, const double *x, bool, Ipopt::Index m, Ipopt::Index,
                  Ipopt::Index *rows, Ipopt::Index *cols, double *values) override {
    if (!values) {
      int index = 0;
      for (int col = 0; col < n; ++col) {
        for (int row = 0; row < m; ++row) {
          rows[index] = row;
          cols[index++] = col;
        }
      }
      return true;
    }
    Eigen::VectorXd point = Eigen::Map<const Eigen::VectorXd>(x, n);
    int index = 0;
    for (int col = 0; col < n; ++col) {
      Eigen::VectorXd plus = point, minus = point;
      plus[col] += s_.difference_step;
      minus[col] -= s_.difference_step;
      const Eigen::VectorXd derivative =
          (constraints(plus) - constraints(minus)) / (2 * s_.difference_step);
      for (int row = 0; row < m; ++row) {
        values[index++] = derivative[row];
      }
    }
    return true;
  }
  bool eval_h(Ipopt::Index, const double *, bool, double, Ipopt::Index, const double *, bool,
              Ipopt::Index, Ipopt::Index *, Ipopt::Index *, double *) override {
    return false;
  }
  void finalize_solution(Ipopt::SolverReturn, Ipopt::Index n, const double *x, const double *,
                         const double *, Ipopt::Index, const double *, const double *, double,
                         const Ipopt::IpoptData *, Ipopt::IpoptCalculatedQuantities *) override {
    solution = Eigen::Map<const Eigen::VectorXd>(x, n).head(seed_.size());
  }

private:
  const Robot &robot_;
  Target target_;
  Eigen::VectorXd seed_;
  Settings s_;
};

Result solve(const Robot &robot, const Target &target, const Eigen::VectorXd &seed,
             const Settings &s) {
  const std::vector<double> positive = {s.position_scale,         s.position_weight,
                                        s.orientation_weight,     s.characteristic_length,
                                        s.minimum_singular_value, s.position_tolerance,
                                        s.orientation_tolerance,  s.difference_step};
  for (double value : positive) {
    if (!std::isfinite(value) || value <= 0) {
      throw std::runtime_error("Solver settings must be finite and positive");
    }
  }
  if (!std::isfinite(s.posture_weight) || s.posture_weight < 0 || s.max_iterations < 1 ||
      seed.size() != robot.lower.size() || !seed.allFinite() ||
      (seed.array() < robot.lower.array()).any() || (seed.array() > robot.upper.array()).any() ||
      !target.position.allFinite() || !target.rotation.allFinite() ||
      (target.rotation.transpose() * target.rotation - Eigen::Matrix3d::Identity()).norm() > 1e-6 ||
      std::abs(target.rotation.determinant() - 1.0) > 1e-6) {
    throw std::runtime_error("Invalid seed, target, or settings");
  }
  Ipopt::SmartPtr<Problem> problem = new Problem(robot, target, seed, s);
  Ipopt::SmartPtr<Ipopt::IpoptApplication> application = IpoptApplicationFactory();
  application->Options()->SetStringValue("hessian_approximation", "limited-memory");
  application->Options()->SetNumericValue("tol", 1e-7);
  application->Options()->SetNumericValue("constr_viol_tol", 1e-9);
  application->Options()->SetNumericValue("bound_relax_factor", 0.0);
  application->Options()->SetIntegerValue("max_iter", s.max_iterations);
  application->Options()->SetIntegerValue("print_level", 0);
  application->Options()->SetStringValue("sb", "yes");
  Result result;
  result.joints = seed;
  auto status = application->Initialize();
  if (status == Ipopt::Solve_Succeeded) {
    status = application->OptimizeTNLP(problem);
  }
  result.solver_status = static_cast<int>(status);
  if (problem->solution.size() != seed.size() || !problem->solution.allFinite()) {
    return result;
  }
  result.joints = problem->solution;
  const Target actual = robot.pose(result.joints);
  result.position_error = (actual.position - target.position).norm();
  result.orientation_error =
      Eigen::AngleAxisd(target.rotation.transpose() * actual.rotation).angle();
  const Eigen::MatrixXd jac = robot.jacobian(result.joints, s.characteristic_length);
  result.minimum_singular_value = jac.jacobiSvd().singularValues().minCoeff();
  const bool bounds_ok = (result.joints.array() >= robot.lower.array()).all() &&
                         (result.joints.array() <= robot.upper.array()).all();
  result.accepted =
      (status == Ipopt::Solve_Succeeded || status == Ipopt::Solved_To_Acceptable_Level) &&
      bounds_ok && result.minimum_singular_value >= s.minimum_singular_value;
  result.target_reached = result.accepted && result.position_error <= s.position_tolerance &&
                          result.orientation_error <= s.orientation_tolerance;
  return result;
}
} // namespace roscon_kinematics
