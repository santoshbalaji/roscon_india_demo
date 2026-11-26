#include "roscon_rviz_plugins/collision_panel.hpp"
#include <QApplication>
#include <chrono>
#include <functional>
#include <gtest/gtest.h>
#include <thread>

// Process Qt events and ROS messages together, just as the panel does in RViz.
static bool waitFor(rclcpp::executors::SingleThreadedExecutor &executor,
                    const std::function<bool()> &condition, double timeout_seconds = 5.0) {
  const auto deadline =
      std::chrono::steady_clock::now() + std::chrono::duration<double>(timeout_seconds);
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

TEST(CollisionPanel, JointControlsAndCollisionStatus) {
  auto node = std::make_shared<rclcpp::Node>("panel_test");
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  auto description_publisher = node->create_publisher<std_msgs::msg::String>(
      "/robot_description", rclcpp::QoS(1).transient_local());
  auto status_publisher = node->create_publisher<std_msgs::msg::String>(
      "/collision_status", rclcpp::QoS(1).transient_local());
  sensor_msgs::msg::JointState latest_state;
  auto subscription = node->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10, [&latest_state](sensor_msgs::msg::JointState::SharedPtr message) {
        latest_state = *message;
      });
  roscon_rviz_plugins::CollisionPanel panel;
  panel.onInitialize();
  std_msgs::msg::String description;
  description.data = R"(
    <robot name="panel_test_robot">
      <link name="base"/><link name="arm"/>
      <joint name="joint_1" type="revolute">
        <parent link="base"/><child link="arm"/><axis xyz="0 0 1"/>
        <limit lower="-1.0" upper="2.0" effort="1.0" velocity="1.0"/>
      </joint>
    </robot>)";
  description_publisher->publish(description);
  ASSERT_TRUE(waitFor(executor, [&latest_state]() { return !latest_state.position.empty(); }));
  auto *position = panel.findChild<QDoubleSpinBox *>("joint_1_position");
  auto *slider = panel.findChild<QSlider *>("joint_1_slider");
  ASSERT_NE(position, nullptr);
  ASSERT_NE(slider, nullptr);
  EXPECT_DOUBLE_EQ(position->minimum(), -1.0);
  EXPECT_DOUBLE_EQ(position->maximum(), 2.0);
  slider->setValue(1000);
  ASSERT_TRUE(waitFor(executor, [&latest_state]() { return latest_state.position[0] == 2.0; }));
  position->setValue(-0.5);
  ASSERT_TRUE(waitFor(executor, [&latest_state]() { return latest_state.position[0] == -0.5; }));
  panel.findChild<QPushButton *>("reset_joints")->click();
  ASSERT_TRUE(waitFor(executor, [&latest_state]() { return latest_state.position[0] == 0.0; }));

  auto *table = panel.findChild<QTableWidget *>("collision_pairs");
  auto *summary = panel.findChild<QLabel *>("collision_summary");
  ASSERT_NE(table, nullptr);
  ASSERT_NE(summary, nullptr);
  std_msgs::msg::String status;
  status.data = "SELF: arm <-> base\nENVIRONMENT: arm <-> table\n";
  status_publisher->publish(status);
  ASSERT_TRUE(waitFor(executor, [table]() { return table->rowCount() == 2; }));
  EXPECT_EQ(table->item(0, 0)->text(), "Self");
  EXPECT_EQ(table->item(1, 0)->text(), "Environment");
  EXPECT_EQ(table->item(1, 2)->text(), "table");
  status.data = "Collision free";
  status_publisher->publish(status);
  ASSERT_TRUE(waitFor(executor, [summary]() { return summary->text() == "Collision free"; }));
  EXPECT_EQ(table->rowCount(), 0);
  ASSERT_TRUE(waitFor(executor, [summary]() { return summary->text().contains("unavailable"); }));
  status_publisher->publish(status);
  ASSERT_TRUE(waitFor(executor, [summary]() { return summary->text() == "Collision free"; }));

  // The panel's new controls use services; confirmed backend state comes from the node.
  bool gpu_requested = false;
  bool sdf_requested = false;
  auto backend_service = node->create_service<std_srvs::srv::SetBool>(
      "/roscon_collision/use_gpu_spheres",
      [&gpu_requested](const std_srvs::srv::SetBool::Request::SharedPtr request,
                       std_srvs::srv::SetBool::Response::SharedPtr response) {
        gpu_requested = request->data;
        response->success = true;
        response->message = "GPU selected";
      });
  auto sdf_service = node->create_service<std_srvs::srv::SetBool>(
      "/roscon_collision/use_sdf",
      [&sdf_requested](const std_srvs::srv::SetBool::Request::SharedPtr request,
                       std_srvs::srv::SetBool::Response::SharedPtr response) {
        sdf_requested = request->data;
        response->success = true;
        response->message = "SDF selected";
      });
  auto backend_publisher = node->create_publisher<std_msgs::msg::String>(
      "/collision_backend", rclcpp::QoS(1).transient_local());
  auto *backend = panel.findChild<QComboBox *>("collision_backend");
  ASSERT_NE(backend, nullptr);
  auto *tabs = panel.findChild<QTabWidget *>("panel_tabs");
  ASSERT_NE(tabs, nullptr);
  EXPECT_EQ(tabs->count(), 2);
  EXPECT_EQ(backend->count(), 3);
  EXPECT_EQ(panel.findChild<QCheckBox *>("show_spheres"), nullptr);
  // Wait for discovery before clicking the buttons.
  auto discovery_client =
      node->create_client<std_srvs::srv::SetBool>("/roscon_collision/use_gpu_spheres");
  ASSERT_TRUE(
      waitFor(executor, [&discovery_client]() { return discovery_client->service_is_ready(); }));
  backend->setCurrentIndex(1);
  ASSERT_TRUE(waitFor(executor, [&gpu_requested]() { return gpu_requested; }));
  std_msgs::msg::String backend_status;
  backend_status.data = "gpu_spheres | view=spheres";
  backend_publisher->publish(backend_status);
  ASSERT_TRUE(waitFor(executor, [backend]() { return backend->currentIndex() == 1; }));
  backend_status.data = "fcl | view=geometry";
  backend_publisher->publish(backend_status);
  ASSERT_TRUE(waitFor(executor, [backend]() { return backend->currentIndex() == 0; }));

  auto sdf_discovery = node->create_client<std_srvs::srv::SetBool>("/roscon_collision/use_sdf");
  ASSERT_TRUE(waitFor(executor, [&sdf_discovery]() { return sdf_discovery->service_is_ready(); }));
  backend->setCurrentIndex(2);
  ASSERT_TRUE(waitFor(executor, [&sdf_requested]() { return sdf_requested; }));
  backend_status.data = "gpu_sdf | view=robot_spheres_environment_sdf";
  backend_publisher->publish(backend_status);
  ASSERT_TRUE(waitFor(executor, [backend]() { return backend->currentIndex() == 2; }));

  // A second source must pause the panel and update its displayed positions.
  auto external_publisher =
      node->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
  ASSERT_TRUE(waitFor(executor, [position]() { return !position->isEnabled(); }));
  sensor_msgs::msg::JointState external_state;
  external_state.name = {"joint_1"};
  external_state.position = {0.75};
  external_publisher->publish(external_state);
  ASSERT_TRUE(waitFor(executor, [position]() { return position->value() == 0.75; }));
}

int main(int argc, char **argv) {
  QApplication application(argc, argv);
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
