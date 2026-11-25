#include "roscon_planner/planner.hpp"
#include <Eigen/Cholesky>
#include <algorithm>
#include <cmath>

namespace roscon_planner {
// Educational CHOMP-style covariant descent. Unlike full CHOMP, this uses a
// discrete joint-path smoothness objective and scalar minimum-clearance cost;
// it does not implement workspace arc-length integration or momentum/HMC.
Path refineWithChomp(const Path &initial, const StateCheck &check,
                     const std::function<double(const Eigen::VectorXd &)> &clearance,
                     const PlanSettings &settings) {
  const Eigen::Index points = initial.size();
  const Eigen::Index joints = initial.front().size();
  if (points < 3) {
    return initial;
  }
  Eigen::MatrixXd positions(points, joints);
  for (Eigen::Index i = 0; i < points; ++i) {
    positions.row(i) = initial[i].transpose();
  }
  Eigen::MatrixXd difference = Eigen::MatrixXd::Zero(points - 2, points);
  for (Eigen::Index i = 0; i < points - 2; ++i) {
    difference(i, i) = 1.0;
    difference(i, i + 1) = -2.0;
    difference(i, i + 2) = 1.0;
  }
  const Eigen::MatrixXd interior = difference.middleCols(1, points - 2);
  Eigen::MatrixXd metric = interior.transpose() * interior;
  metric.diagonal().array() += 1e-4;
  const Eigen::LDLT<Eigen::MatrixXd> factor(metric);
  if (factor.info() != Eigen::Success) {
    throw std::runtime_error("Cannot construct covariant metric");
  }
  const double smoothness_weight = 0.05;
  const double gradient_step = 0.001;
  auto obstacle_cost = [&](const Eigen::VectorXd &q) {
    const double distance = clearance(q);
    if (std::isnan(distance)) {
      throw std::runtime_error("Invalid clearance value");
    }
    const double deficit =
        std::max(0.0, settings.clearance_margin - distance) / settings.clearance_margin;
    return 0.5 * deficit * deficit;
  };
  auto cost = [&](const Eigen::MatrixXd &path) {
    double value = smoothness_weight * (difference * path).squaredNorm();
    for (Eigen::Index i = 1; i + 1 < points; ++i) {
      value += obstacle_cost(path.row(i).transpose());
    }
    return value;
  };
  double previous_cost = cost(positions);
  for (int iteration = 0; iteration < settings.chomp_iterations; ++iteration) {
    Eigen::MatrixXd gradient =
        2.0 * smoothness_weight * difference.transpose() * difference * positions;
    for (Eigen::Index i = 1; i + 1 < points; ++i) {
      const Eigen::VectorXd q = positions.row(i).transpose();
      // Outside the clearance penalty its derivative is zero.
      if (obstacle_cost(q) == 0.0) {
        continue;
      }
      for (Eigen::Index joint = 0; joint < joints; ++joint) {
        Eigen::VectorXd plus = q, minus = q;
        plus[joint] += gradient_step;
        minus[joint] -= gradient_step;
        gradient(i, joint) += (obstacle_cost(plus) - obstacle_cost(minus)) / (2.0 * gradient_step);
      }
    }
    Eigen::MatrixXd direction = factor.solve(gradient.middleRows(1, points - 2));
    if (!direction.allFinite()) {
      throw std::runtime_error("Invalid covariant update");
    }
    const double largest = direction.cwiseAbs().maxCoeff();
    if (largest < 1e-8) {
      break;
    }
    if (largest > 0.1) {
      direction *= 0.1 / largest;
    }
    bool improved = false;
    for (double step = 1.0; step >= 0.03125; step *= 0.5) {
      Eigen::MatrixXd candidate = positions;
      candidate.middleRows(1, points - 2) -= step * direction;
      Path path;
      for (Eigen::Index i = 0; i < points; ++i) {
        path.push_back(candidate.row(i).transpose());
      }
      if (!validatePath(path, settings.joint_step, check).valid) {
        continue;
      }
      const double candidate_cost = cost(candidate);
      if (candidate_cost < previous_cost - 1e-10) {
        positions = candidate;
        previous_cost = candidate_cost;
        improved = true;
        break;
      }
    }
    if (!improved) {
      break;
    }
  }
  Path result;
  for (Eigen::Index i = 0; i < points; ++i) {
    result.push_back(positions.row(i).transpose());
  }
  return result;
}
} // namespace roscon_planner
