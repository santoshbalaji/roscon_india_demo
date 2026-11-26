#ifndef ROSCON_RVIZ_PLUGINS_COLLISION_PANEL_HPP
#define ROSCON_RVIZ_PLUGINS_COLLISION_PANEL_HPP

#include <QCheckBox>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QLabel>
#include <QPushButton>
#include <QSlider>
#include <QTabWidget>
#include <QTableWidget>
#include <QTimer>
#include <QVBoxLayout>
#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <string>
#include <vector>

namespace roscon_rviz_plugins {
struct JointControl {
  std::string name;
  double minimum;
  double maximum;
  QSlider *slider;
  QDoubleSpinBox *position;
};

class CollisionPanel : public rviz_common::Panel {
  Q_OBJECT

public:
  explicit CollisionPanel(QWidget *parent = nullptr);
  ~CollisionPanel() override;
  void onInitialize() override;

private:
  void update();
  void receiveBackend(const std_msgs::msg::String::SharedPtr message);
  void requestBackend(int backend);
  void updateVisualization();
  void loadRobot(const std_msgs::msg::String::SharedPtr message);
  void receiveStatus(const std_msgs::msg::String::SharedPtr message);
  void receiveJointState(const sensor_msgs::msg::JointState::SharedPtr message);
  void addJoint(const std::string &name, double minimum, double maximum, bool prismatic);
  void setJointPosition(JointControl &control, double position);
  void resetJoints();
  void publishJointStates();

  QTabWidget *tabs_;
  QComboBox *backend_selector_;
  QLabel *backend_error_;
  int confirmed_backend_ = 0;
  int visualization_backend_ = -1;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr backend_client_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr sdf_client_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr backend_subscription_;
  QLabel *status_label_;
  QTableWidget *collision_table_;
  QLabel *robot_label_;
  QLabel *publishing_label_;
  QCheckBox *publish_checkbox_;
  QPushButton *reset_button_;
  QVBoxLayout *joint_layout_;
  QTimer *timer_;
  std::vector<JointControl> joint_controls_;
  std::string loaded_description_;
  std::string previous_status_;
  bool robot_loaded_ = false;
  bool publishing_ = false;
  bool status_received_ = false;
  std::chrono::steady_clock::time_point last_status_time_;
  rclcpp::Node::SharedPtr node_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_publisher_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr description_subscription_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr status_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_subscription_;
};
} // namespace roscon_rviz_plugins
#endif
