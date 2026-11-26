#include "roscon_kinematics/solver.hpp"
#include <Eigen/Cholesky>
#include <Eigen/SVD>
#include <gtest/gtest.h>
#include <sstream>

// A mesh-free six-joint arm makes the solver tests independent of ROS and assets.
std::string test_robot() {
  std::ostringstream xml;
  xml << "<robot name='test'><link name='base'/>";
  const char *axes[] = {"0 0 1", "0 1 0", "0 1 0", "1 0 0", "0 1 0", "1 0 0"};
  const char *offsets[] = {"0 0 0.2", "0 0 0.1", "0.4 0 0", "0.3 0 0", "0.1 0 0", "0.1 0 0"};
  for (int i = 0; i < 6; ++i) {
    xml << "<link name='link" << i << "'/><joint name='joint" << i << "' type='revolute'>"
        << "<parent link='" << (i == 0 ? "base" : "link" + std::to_string(i - 1)) << "'/>"
        << "<child link='link" << i << "'/><origin xyz='" << offsets[i] << "'/>"
        << "<axis xyz='" << axes[i]
        << "'/><limit lower='-2.9' upper='2.9' effort='100' velocity='1'/></joint>";
  }
  return xml.str() + "</robot>";
}
Eigen::VectorXd seed() {
  Eigen::VectorXd q(6);
  q << 0.2, -0.8, 1.2, 0.4, 0.9, -0.3;
  return q;
}
TEST(Singularity, MatrixFactorMatchesSingularValueBound) {
  Eigen::MatrixXd jac = Eigen::MatrixXd::Identity(6, 6);
  Eigen::Matrix<double, 6, 6> factor =
      std::sqrt(1.0 - 0.04) * Eigen::Matrix<double, 6, 6>::Identity();
  EXPECT_LT(roscon_kinematics::factor_residual(jac, factor, 0.2).norm(), 1e-12);
  jac(5, 5) = 0.0;
  // A negative diagonal cannot equal a diagonal of L L^T.
  EXPECT_LT(roscon_kinematics::factor_residual(jac, Eigen::Matrix<double, 6, 6>::Zero(), 0.2)[20],
            0.0);
}
TEST(Solver, ReachesNearbyPoseAndHonorsConstraints) {
  roscon_kinematics::Settings settings;
  roscon_kinematics::Robot robot(test_robot(), "base", "link5", settings.joint_margin);
  Eigen::VectorXd desired = seed();
  desired[0] += 0.15;
  desired[2] -= 0.1;
  auto result = roscon_kinematics::solve(robot, robot.pose(desired), seed(), settings);
  ASSERT_TRUE(result.accepted) << result.solver_status;
  EXPECT_TRUE(result.target_reached);
  EXPECT_LT(result.position_error, settings.position_tolerance);
  EXPECT_LT(result.orientation_error, settings.orientation_tolerance);
  EXPECT_GE(result.minimum_singular_value, settings.minimum_singular_value);
  EXPECT_TRUE((result.joints.array() >= robot.lower.array()).all());
  EXPECT_TRUE((result.joints.array() <= robot.upper.array()).all());
}
TEST(Solver, UnreachableTargetIsNotReportedReached) {
  roscon_kinematics::Settings settings;
  roscon_kinematics::Robot robot(test_robot(), "base", "link5", settings.joint_margin);
  auto target = robot.pose(seed());
  target.position.x() = 10.0;
  auto result = roscon_kinematics::solve(robot, target, seed(), settings);
  EXPECT_FALSE(result.target_reached);
  if (result.accepted) {
    EXPECT_GE(result.minimum_singular_value, settings.minimum_singular_value);
  }
}
TEST(Solver, ImpossibleSingularityMarginIsRejected) {
  roscon_kinematics::Settings settings;
  settings.minimum_singular_value = 10.0;
  settings.max_iterations = 30;
  roscon_kinematics::Robot robot(test_robot(), "base", "link5", settings.joint_margin);
  auto result = roscon_kinematics::solve(robot, robot.pose(seed()), seed(), settings);
  EXPECT_FALSE(result.accepted);
}
TEST(Solver, InvalidInputIsRejected) {
  roscon_kinematics::Settings settings;
  roscon_kinematics::Robot robot(test_robot(), "base", "link5", settings.joint_margin);
  auto target = robot.pose(seed());
  auto bad = seed();
  bad[0] = 4;
  EXPECT_THROW(roscon_kinematics::solve(robot, target, bad, settings), std::runtime_error);
  target.rotation.setZero();
  EXPECT_THROW(roscon_kinematics::solve(robot, target, seed(), settings), std::runtime_error);
  EXPECT_THROW(roscon_kinematics::Robot(test_robot(), "missing", "link5", 0.01),
               std::runtime_error);
}
