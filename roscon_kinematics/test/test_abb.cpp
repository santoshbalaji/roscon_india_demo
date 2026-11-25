#include "roscon_kinematics/multi_seed.hpp"
#include "roscon_kinematics/solver.hpp"
#include <fstream>
#include <gtest/gtest.h>
#include <iterator>

TEST(AbbIntegration, ReachesTargetWithInstalledDescription) {
  std::ifstream input(ABB_TEST_URDF);
  ASSERT_TRUE(input.good()) << ABB_TEST_URDF;
  const std::string xml((std::istreambuf_iterator<char>(input)), std::istreambuf_iterator<char>());
  roscon_kinematics::Settings settings;
  roscon_kinematics::Robot robot(xml, "base_link", "flange", settings.joint_margin);
  ASSERT_EQ(robot.names.size(), 6u);
  EXPECT_EQ(robot.names.front(), "joint_1");
  EXPECT_EQ(robot.names.back(), "joint_6");
  Eigen::VectorXd seed(6);
  seed << 0.0, -1.0, 1.2, -1.0, 1.0, 0.0;
  const auto initial = roscon_kinematics::solve(robot, robot.pose(seed), seed, settings);
  ASSERT_TRUE(initial.accepted) << initial.solver_status;
  EXPECT_TRUE(initial.target_reached);
  Eigen::VectorXd desired = seed;
  desired[0] += 0.1;
  desired[2] -= 0.1;
  const auto result =
      roscon_kinematics::solve(robot, robot.pose(desired), initial.joints, settings);
  ASSERT_TRUE(result.accepted) << result.solver_status;
  EXPECT_TRUE(result.target_reached);
  EXPECT_GE(result.minimum_singular_value, settings.minimum_singular_value);
  EXPECT_TRUE((result.joints.array() >= robot.lower.array()).all());
  EXPECT_TRUE((result.joints.array() <= robot.upper.array()).all());
}

TEST(AbbIntegration, PredefinedSeedsFindDistinctValidSolutions) {
  std::ifstream input(ABB_TEST_URDF);
  ASSERT_TRUE(input.good());
  const std::string xml((std::istreambuf_iterator<char>(input)), std::istreambuf_iterator<char>());
  roscon_kinematics::Settings settings;
  roscon_kinematics::Robot robot(xml, "base_link", "flange", settings.joint_margin);
  Eigen::VectorXd current(6);
  current << 0.0, -1.0, 1.2, -1.0, 1.0, 0.0;
  const auto target = robot.pose(current);
  const auto seeds = roscon_kinematics::predefined_seeds(robot, current);
  ASSERT_EQ(seeds.size(), 17u);
  std::vector<Eigen::VectorXd> unique;
  for (const auto &seed : seeds) {
    EXPECT_TRUE((seed.array() >= robot.lower.array()).all());
    EXPECT_TRUE((seed.array() <= robot.upper.array()).all());
    const auto result = roscon_kinematics::solve(robot, target, seed, settings);
    if (!result.accepted || !result.target_reached) {
      continue;
    }
    bool duplicate = false;
    for (const auto &previous : unique) {
      if (roscon_kinematics::same_configuration(result.joints, previous)) {
        duplicate = true;
      }
    }
    if (!duplicate) {
      unique.push_back(result.joints);
    }
    EXPECT_LE(result.position_error, settings.position_tolerance);
    EXPECT_LE(result.orientation_error, settings.orientation_tolerance);
    EXPECT_GE(result.minimum_singular_value, settings.minimum_singular_value);
  }
  EXPECT_GE(unique.size(), 2u);
  auto wrapped = current;
  wrapped[3] += 6.283185307179586;
  EXPECT_TRUE(roscon_kinematics::same_configuration(current, wrapped));
  wrapped[3] += 0.2;
  EXPECT_FALSE(roscon_kinematics::same_configuration(current, wrapped));
}
