#include "roscon_rviz_plugins/kinematics_panel.hpp"
#include <QApplication>
#include <QCheckBox>
#include <QScrollArea>
#include <cmath>
#include <functional>
#include <gtest/gtest.h>
#include <thread>

bool waitFor(rclcpp::executors::SingleThreadedExecutor &executor,
             const std::function<bool()> &condition, double seconds = 5.0) {
  auto deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(seconds);
  while (std::chrono::steady_clock::now() < deadline) {
    QApplication::processEvents();
    executor.spin_some();
    if (condition()) {
      return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  return false;
}
TEST(KinematicsPanel, ReadoutsExplicitCommandsResetAndStaleState) {
  auto node = std::make_shared<rclcpp::Node>("kinematics_panel_test");
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  auto description = node->create_publisher<std_msgs::msg::String>(
      "/robot_description", rclcpp::QoS(1).transient_local());
  auto joints = node->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
  auto pose = node->create_publisher<geometry_msgs::msg::PoseStamped>("/ik_current_pose", 10);
  auto status = node->create_publisher<std_msgs::msg::String>("/ik_status", 10);
  auto requested_pose =
      node->create_publisher<geometry_msgs::msg::PoseStamped>("/ik_target_pose", 10);
  auto candidates = node->create_publisher<roscon_msgs::msg::IKSolutions>(
      "/ik_solutions", rclcpp::QoS(1).transient_local());
  int searches = 0, previews = 0;
  geometry_msgs::msg::PoseStamped search_target;
  roscon_msgs::msg::IKSelection preview;
  auto search_sub = node->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/ik_find_solutions", 10, [&](geometry_msgs::msg::PoseStamped::ConstSharedPtr message) {
        ++searches;
        search_target = *message;
      });
  auto preview_sub = node->create_subscription<roscon_msgs::msg::IKSelection>(
      "/ik_select_solution", 10, [&](roscon_msgs::msg::IKSelection::ConstSharedPtr message) {
        ++previews;
        preview = *message;
      });
  int joint_commands = 0, pose_commands = 0, resets = 0;
  sensor_msgs::msg::JointState joint_command;
  geometry_msgs::msg::PoseStamped pose_command;
  auto joint_sub = node->create_subscription<sensor_msgs::msg::JointState>(
      "/ik_joint_command", 10, [&](sensor_msgs::msg::JointState::ConstSharedPtr message) {
        joint_command = *message;
        ++joint_commands;
      });
  auto pose_sub = node->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/target_pose", 10, [&](geometry_msgs::msg::PoseStamped::ConstSharedPtr message) {
        pose_command = *message;
        ++pose_commands;
      });
  auto reset_service = node->create_service<std_srvs::srv::Trigger>(
      "/reset_ik", [&](std_srvs::srv::Trigger::Request::ConstSharedPtr,
                       std_srvs::srv::Trigger::Response::SharedPtr response) {
        ++resets;
        response->success = true;
        response->message = "Reset accepted";
      });
  roscon_rviz_plugins::KinematicsPanel panel;
  panel.onInitialize();
  panel.resize(500, 720);
  panel.show();
  std_msgs::msg::String model;
  model.data = R"(<robot name="test"><link name="base"/><link name="tip"/>
    <joint name="joint_1" type="revolute"><parent link="base"/><child link="tip"/>
    <axis xyz="0 0 1"/><limit lower="-2" upper="2" effort="1" velocity="1"/></joint></robot>)";
  description->publish(model);
  sensor_msgs::msg::JointState state;
  state.name = {"joint_1"};
  state.position = {0.25};
  geometry_msgs::msg::PoseStamped actual;
  actual.header.frame_id = "base_link";
  actual.pose.position.x = 0.6;
  actual.pose.position.y = 0.2;
  actual.pose.position.z = 0.8;
  actual.pose.orientation.w = 1;
  std_msgs::msg::String report;
  report.data = "TARGET REACHED\nIPOPT status: 0\nPosition error: 0.0000 m\nOrientation error: "
                "0.0000 deg\nMinimum singular value: 0.2252 (limit 0.0500)";
  auto heartbeat = node->create_wall_timer(std::chrono::milliseconds(50), [&]() {
    joints->publish(state);
    pose->publish(actual);
    requested_pose->publish(actual);
    status->publish(report);
  });
  auto *apply = panel.findChild<QPushButton *>("apply_joints");
  ASSERT_TRUE(waitFor(executor, [&]() { return apply->isEnabled(); }));
  auto *joint = panel.findChild<QDoubleSpinBox *>("joint_1_target");
  ASSERT_NE(joint, nullptr);
  EXPECT_DOUBLE_EQ(joint->value(), 0.25);
  joint->setValue(0.5);
  EXPECT_EQ(joint_commands, 0); // Editing never publishes robot states or commands.
  apply->click();
  ASSERT_TRUE(waitFor(executor, [&]() { return joint_commands == 1; }));
  EXPECT_DOUBLE_EQ(joint_command.position[0], 0.5);
  state.position[0] = 0.4;
  auto *table = panel.findChild<QTableWidget *>("joint_positions");
  ASSERT_TRUE(waitFor(executor, [&]() { return table->item(0, 1)->text() == "0.40000"; }));
  EXPECT_DOUBLE_EQ(joint->value(), 0.5); // Feedback must not overwrite a draft target.
  panel.findChild<QPushButton *>("copy_joints")->click();
  EXPECT_DOUBLE_EQ(joint->value(), 0.4);
  auto *x = panel.findChild<QDoubleSpinBox *>("pose_target_0");
  auto *yaw = panel.findChild<QDoubleSpinBox *>("pose_target_5");
  x->setValue(0.7);
  yaw->setValue(90);
  panel.findChild<QPushButton *>("solve_pose")->click();
  ASSERT_TRUE(waitFor(executor, [&]() { return pose_commands == 1; }));
  EXPECT_DOUBLE_EQ(pose_command.pose.position.x, 0.7);
  EXPECT_EQ(pose_command.header.frame_id, "base_link");
  EXPECT_NEAR(pose_command.pose.orientation.z, std::sqrt(0.5), 1e-6);
  panel.findChild<QPushButton *>("copy_pose")->click();
  EXPECT_DOUBLE_EQ(x->value(), 0.6);
  EXPECT_DOUBLE_EQ(yaw->value(), 0);
  auto *reset = panel.findChild<QPushButton *>("reset_ik");
  ASSERT_TRUE(waitFor(executor, [&]() { return reset->isEnabled(); }));
  reset->click();
  ASSERT_TRUE(waitFor(executor, [&]() { return resets == 1; }));
  EXPECT_EQ(panel.findChild<QCheckBox *>("interactive_target"), nullptr);
  auto *find = panel.findChild<QPushButton *>("find_solutions");
  auto *list = panel.findChild<QComboBox *>("ik_solutions");
  auto *show = panel.findChild<QPushButton *>("preview_solution");
  ASSERT_TRUE(waitFor(executor, [&]() { return find->isEnabled(); }));
  find->click();
  ASSERT_TRUE(waitFor(executor, [&]() { return searches == 1; }));
  EXPECT_DOUBLE_EQ(search_target.pose.position.x, actual.pose.position.x);
  roscon_msgs::msg::IKSolutions results;
  results.generation = 1;
  results.target = actual.pose;
  results.seeds_total = 17;
  results.seeds_tried = 17;
  results.complete = true;
  results.self_collision_rejections = 3;
  results.environment_collision_rejections = 8;
  results.last_rejection = "ENVIRONMENT: link_6 <-> ball";
  roscon_msgs::msg::IKSolution first;
  first.joints.name = {"joint_1"};
  first.joints.position = {0.2};
  results.solutions.push_back(first);
  first.seed_index = 3;
  first.joints.position = {-0.8};
  results.solutions.push_back(first);
  candidates->publish(results);
  ASSERT_TRUE(waitFor(executor, [&]() { return list->count() == 2 && show->isEnabled(); }));
  auto *summary = panel.findChild<QLabel *>("solution_summary");
  ASSERT_NE(summary, nullptr);
  EXPECT_TRUE(summary->text().contains("3 self / 8 environment"));
  EXPECT_TRUE(summary->text().contains("link_6 <-> ball"));
  list->setCurrentIndex(1);
  show->click();
  ASSERT_TRUE(waitFor(executor, [&]() { return previews == 1; }));
  EXPECT_EQ(preview.generation, 1u);
  EXPECT_EQ(preview.index, 1u);
  // A target-change notification removes the old choices.
  results.generation = 2;
  results.seeds_total = 0;
  results.solutions.clear();
  candidates->publish(results);
  ASSERT_TRUE(waitFor(executor, [&]() { return list->count() == 0 && !show->isEnabled(); }));
  EXPECT_EQ(node->count_publishers("/joint_states"), 1u);
  // A short RViz dock must keep the remaining controls scrollable.
  panel.resize(340, 280);
  QApplication::processEvents();
  EXPECT_NE(panel.findChild<QScrollArea *>("kinematics_scroll"), nullptr);
  panel.grab().save("/tmp/kinematics-panel-compact.png");
  heartbeat->cancel();
  ASSERT_TRUE(waitFor(executor, [&]() { return !apply->isEnabled(); }));
  apply->click();
  EXPECT_EQ(joint_commands, 1);
}
int main(int argc, char **argv) {
  QApplication application(argc, argv);
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
