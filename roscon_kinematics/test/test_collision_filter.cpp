#include "roscon_collision/scene_checker.hpp"
#include "roscon_kinematics/multi_seed.hpp"
#include "roscon_kinematics/solver.hpp"
#include <fstream>
#include <gtest/gtest.h>

const std::string simple_robot = R"(
<robot name="collision_test">
  <link name="base"><collision><geometry><sphere radius="0.1"/></geometry></collision></link>
  <link name="pivot"/>
  <joint name="mount" type="fixed"><parent link="base"/><child link="pivot"/>
    <origin xyz="0.5 0 0"/></joint>
  <link name="tip"><collision><origin xyz="0.5 0 0"/>
    <geometry><sphere radius="0.1"/></geometry></collision></link>
  <joint name="angle" type="revolute"><parent link="pivot"/><child link="tip"/>
    <axis xyz="0 0 1"/><limit lower="-3.15" upper="3.15" effort="1" velocity="1"/></joint>
</robot>)";

TEST(CollisionFilter, SelfCollisionAndInvalidInput) {
  roscon_collision::SceneChecker checker(simple_robot, EMPTY_SCENE, 0.06);
  EXPECT_TRUE(checker.check({"angle"}, {0.0}).collision_free());
  const auto folded = checker.check({"angle"}, {3.141592653589793});
  EXPECT_TRUE(folded.self_collision);
  EXPECT_FALSE(folded.environment_collision);
  EXPECT_FALSE(folded.pairs.empty());
  EXPECT_THROW(checker.check({}, {}), std::runtime_error);
  EXPECT_THROW(checker.check({"wrong"}, {0.0}), std::runtime_error);
}

TEST(CollisionFilter, EnvironmentCollisionAndVisibleGeometry) {
  roscon_collision::SceneChecker checker(simple_robot, BLOCKED_SCENE, 0.06);
  const auto blocked = checker.check({"angle"}, {0.0});
  EXPECT_TRUE(blocked.environment_collision);
  EXPECT_FALSE(blocked.self_collision);
  EXPECT_TRUE(checker.check({"angle"}, {1.5707963267948966}).collision_free());
  const auto markers = checker.environmentMarkers();
  ASSERT_EQ(markers.markers.size(), 1u);
  EXPECT_EQ(markers.markers[0].header.frame_id, "base");
  EXPECT_DOUBLE_EQ(markers.markers[0].pose.position.x, 1.0);
}

TEST(CollisionFilter, AbbStartupHasCollisionFreeCandidates) {
  std::ifstream input(ABB_TEST_URDF);
  ASSERT_TRUE(input.good());
  const std::string xml((std::istreambuf_iterator<char>(input)), std::istreambuf_iterator<char>());
  roscon_kinematics::Settings settings;
  roscon_kinematics::Robot robot(xml, "base_link", "flange", settings.joint_margin);
  roscon_collision::SceneChecker checker(xml, ABB_SCENE, 0.06);
  Eigen::VectorXd current(6);
  current << 0.0, -1.0, 1.2, -1.0, 1.0, 0.0;
  const auto target = robot.pose(current);
  size_t free = 0;
  size_t rejected = 0;
  for (const auto &seed : roscon_kinematics::predefined_seeds(robot, current)) {
    const auto result = roscon_kinematics::solve(robot, target, seed, settings);
    if (!result.accepted || !result.target_reached) {
      continue;
    }
    const std::vector<double> positions(result.joints.data(),
                                        result.joints.data() + result.joints.size());
    const auto report = checker.check(robot.names, positions);
    if (report.collision_free()) {
      ++free;
    } else {
      ++rejected;
      for (const auto &pair : report.pairs) {
        std::cout << pair << "\n";
      }
    }
  }
  std::cout << "ABB candidates: " << free << " free, " << rejected << " rejected\n";
  EXPECT_GT(free, 0u);
}
