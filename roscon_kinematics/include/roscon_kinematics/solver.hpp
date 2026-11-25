#pragma once
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <kdl/chain.hpp>
#include <string>
#include <vector>

namespace roscon_kinematics {
struct Settings {
  double position_scale = 0.1;
  double position_weight = 1.0;
  double orientation_weight = 1.0;
  double posture_weight = 0.0001;
  double characteristic_length = 1.0;
  double minimum_singular_value = 0.05;
  double joint_margin = 0.01;
  double position_tolerance = 0.005;
  double orientation_tolerance = 0.02;
  double difference_step = 1e-6;
  int max_iterations = 250;
};
struct Target {
  Eigen::Vector3d position;
  Eigen::Matrix3d rotation;
};
struct Result {
  Eigen::VectorXd joints;
  bool accepted = false;
  bool target_reached = false;
  int solver_status = -999;
  double position_error = 0.0;
  double orientation_error = 0.0;
  double minimum_singular_value = 0.0;
};
class Robot {
public:
  Robot(const std::string &xml, const std::string &base, const std::string &tip,
        double joint_margin);
  Target pose(const Eigen::VectorXd &q) const;
  Eigen::MatrixXd jacobian(const Eigen::VectorXd &q, double length) const;
  std::vector<std::string> names;
  Eigen::VectorXd lower, upper;

private:
  KDL::Chain chain_;
};
// This smooth equality is equivalent to a lower bound on all singular values:
// J_scaled J_scaled^T - sigma_safe^2 I = L L^T.
// L is lower triangular, adding 21 variables and 21 equality constraints.
Eigen::Matrix<double, 6, 6> unpack_factor(const Eigen::VectorXd &x, int dof);
Eigen::VectorXd factor_residual(const Eigen::MatrixXd &jacobian,
                                const Eigen::Matrix<double, 6, 6> &factor,
                                double minimum_singular_value);
Result solve(const Robot &robot, const Target &target, const Eigen::VectorXd &seed,
             const Settings &settings);
} // namespace roscon_kinematics
