#pragma once
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QLabel>
#include <QPushButton>
#include <QTimer>
#include <array>
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <roscon_msgs/srv/plan_motion.hpp>
#include <roscon_msgs/srv/set_collision_backend.hpp>
#include <roscon_msgs/srv/validate_trajectory.hpp>
#include <rviz_common/panel.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>

namespace roscon_rviz_plugins {
class PlannerPanel : public rviz_common::Panel {
  Q_OBJECT
public:
  explicit PlannerPanel(QWidget *parent = nullptr);
  ~PlannerPanel() override;
  void onInitialize() override;

private:
  void update();
  void setBusy(bool busy);
  void updateControls();
  void fillTarget(const geometry_msgs::msg::PoseStamped &pose);
  geometry_msgs::msg::PoseStamped targetMessage() const;
  void plan();
  void validate();
  void changeBackend();
  void trigger(const rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr &client);
  std::array<QDoubleSpinBox *, 6> target_;
  QComboBox *backend_, *algorithm_, *optimizer_;
  QLabel *status_, *feedback_, *current_label_, *joints_label_;
  QPushButton *plan_, *validate_, *play_, *stop_, *reset_, *copy_, *demo_;
  QTimer *timer_;
  bool busy_ = false, current_ready_ = false, demo_ready_ = false;
  std::chrono::steady_clock::time_point last_status_;
  geometry_msgs::msg::PoseStamped current_pose_, demo_pose_;
  rclcpp::Node::SharedPtr node_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  rclcpp::Client<roscon_msgs::srv::PlanMotion>::SharedPtr plan_client_;
  rclcpp::Client<roscon_msgs::srv::SetCollisionBackend>::SharedPtr backend_client_;
  rclcpp::Client<roscon_msgs::srv::ValidateTrajectory>::SharedPtr validate_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr play_client_, stop_client_, reset_client_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr current_sub_, target_sub_,
      demo_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr status_sub_, backend_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joints_sub_;
};
} // namespace roscon_rviz_plugins
