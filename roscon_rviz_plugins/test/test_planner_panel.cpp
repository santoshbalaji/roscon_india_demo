#include "roscon_rviz_plugins/planner_panel.hpp"
#include <QApplication>
#include <QScrollArea>
#include <gtest/gtest.h>
#include <thread>

TEST(PlannerPanel, PoseBackendPlanningValidationAndPlaybackCommands) {
  auto node = std::make_shared<rclcpp::Node>("planner_panel_test");
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  auto pose_pub =
      node->create_publisher<geometry_msgs::msg::PoseStamped>("/planner/current_pose", 10);
  auto demo_pub = node->create_publisher<geometry_msgs::msg::PoseStamped>(
      "/planner/demo_target", rclcpp::QoS(1).transient_local());
  auto status_pub = node->create_publisher<std_msgs::msg::String>("/planner/status", 10);
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = "base_link";
  pose.pose.position.x = 0.4;
  pose.pose.orientation.w = 1.0;
  auto demo = pose;
  demo.pose.position.y = 0.2;
  demo_pub->publish(demo);
  int plans = 0, validations = 0, plays = 0, stops = 0, resets = 0;
  std::string backend;
  geometry_msgs::msg::PoseStamped received;
  auto plan = node->create_service<roscon_msgs::srv::PlanMotion>(
      "/planner/plan", [&](roscon_msgs::srv::PlanMotion::Request::ConstSharedPtr req,
                           roscon_msgs::srv::PlanMotion::Response::SharedPtr res) {
        ++plans;
        received = req->target;
        EXPECT_EQ(req->ompl_planner, "prm");
        EXPECT_EQ(req->optimizer, "chomp");
        res->success = true;
        res->message = "validated";
      });
  auto select = node->create_service<roscon_msgs::srv::SetCollisionBackend>(
      "/planner/set_backend",
      [&](roscon_msgs::srv::SetCollisionBackend::Request::ConstSharedPtr req,
          roscon_msgs::srv::SetCollisionBackend::Response::SharedPtr res) {
        backend = req->backend;
        res->success = true;
      });
  auto validate = node->create_service<roscon_msgs::srv::ValidateTrajectory>(
      "/planner/validate", [&](roscon_msgs::srv::ValidateTrajectory::Request::ConstSharedPtr req,
                               roscon_msgs::srv::ValidateTrajectory::Response::SharedPtr res) {
        ++validations;
        EXPECT_TRUE(req->trajectory.points.empty());
        res->valid = true;
      });
  auto play = node->create_service<std_srvs::srv::Trigger>(
      "/planner/play", [&](std_srvs::srv::Trigger::Request::ConstSharedPtr,
                           std_srvs::srv::Trigger::Response::SharedPtr res) {
        ++plays;
        res->success = true;
      });
  auto stop = node->create_service<std_srvs::srv::Trigger>(
      "/planner/stop", [&](std_srvs::srv::Trigger::Request::ConstSharedPtr,
                           std_srvs::srv::Trigger::Response::SharedPtr res) {
        ++stops;
        res->success = true;
      });
  auto reset = node->create_service<std_srvs::srv::Trigger>(
      "/planner/reset", [&](std_srvs::srv::Trigger::Request::ConstSharedPtr,
                            std_srvs::srv::Trigger::Response::SharedPtr res) {
        ++resets;
        res->success = true;
      });
  roscon_rviz_plugins::PlannerPanel panel;
  panel.onInitialize();
  panel.show();
  auto wait = [&](const std::function<bool()> &condition) {
    const auto end = std::chrono::steady_clock::now() + std::chrono::seconds(8);
    while (std::chrono::steady_clock::now() < end) {
      pose_pub->publish(pose);
      std_msgs::msg::String status;
      status.data = "Ready";
      status_pub->publish(status);
      QApplication::processEvents();
      executor.spin_some();
      if (condition()) {
        return true;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    return false;
  };
  auto *plan_button = panel.findChild<QPushButton *>("plan_motion");
  auto *demo_button = panel.findChild<QPushButton *>("demo_target");
  ASSERT_TRUE(wait([&]() { return plan_button->isEnabled() && demo_button->isEnabled(); }));
  demo_button->click();
  auto *algorithms = panel.findChild<QComboBox *>("planning_algorithm");
  auto *optimizers = panel.findChild<QComboBox *>("optimization_algorithm");
  ASSERT_NE(algorithms, nullptr);
  ASSERT_NE(optimizers, nullptr);
  EXPECT_EQ(algorithms->count(), 3);
  EXPECT_EQ(optimizers->count(), 3);
  algorithms->setCurrentIndex(algorithms->findData("prm"));
  optimizers->setCurrentIndex(optimizers->findData("chomp"));
  panel.findChild<QComboBox *>("collision_backend")->setCurrentText("sdf");
  ASSERT_TRUE(wait([&]() { return backend == "sdf" && plan_button->isEnabled(); }));
  plan_button->click();
  ASSERT_TRUE(wait([&]() { return plans == 1 && plan_button->isEnabled(); }));
  EXPECT_EQ(received.header.frame_id, "base_link");
  EXPECT_DOUBLE_EQ(received.pose.position.y, 0.2);
  panel.findChild<QPushButton *>("validate_path")->click();
  ASSERT_TRUE(wait([&]() { return validations == 1 && plan_button->isEnabled(); }));
  panel.findChild<QPushButton *>("play_path")->click();
  ASSERT_TRUE(wait([&]() { return plays == 1 && plan_button->isEnabled(); }));
  panel.findChild<QPushButton *>("stop_path")->click();
  ASSERT_TRUE(wait([&]() { return stops == 1 && plan_button->isEnabled(); }));
  panel.findChild<QPushButton *>("reset_planner")->click();
  ASSERT_TRUE(wait([&]() { return resets == 1 && plan_button->isEnabled(); }));
  EXPECT_EQ(node->count_publishers("/joint_states"), 0u);
  EXPECT_NE(panel.findChild<QScrollArea *>("planner_scroll"), nullptr);
}
int main(int argc, char **argv) {
  QApplication application(argc, argv);
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
