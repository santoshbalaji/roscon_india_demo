#pragma once

#include <QComboBox>
#include <QDoubleSpinBox>
#include <QLabel>
#include <QPushButton>
#include <QTableWidget>
#include <QTimer>
#include <array>
#include <chrono>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <roscon_msgs/msg/ik_selection.hpp>
#include <roscon_msgs/msg/ik_solutions.hpp>
#include <rviz_common/panel.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <urdf/model.h>

namespace roscon_rviz_plugins {
class KinematicsPanel : public rviz_common::Panel {
  Q_OBJECT
public:
  explicit KinematicsPanel(QWidget *parent = nullptr);
  ~KinematicsPanel() override;
  void onInitialize() override;

private:
  void update();
  void receiveJoints(const sensor_msgs::msg::JointState &message);
  void receivePose(const geometry_msgs::msg::PoseStamped &message);
  void copyJoints();
  void copyPose();
  void applyJoints();
  void solvePose();
  void reset();
  void findSolutions();
  void receiveSolutions(const roscon_msgs::msg::IKSolutions &message);
  void previewSolution();
  bool connected() const;

  QPushButton *find_solutions_;
  QPushButton *preview_solution_;
  QComboBox *solution_list_;
  QLabel *solution_summary_;
  bool target_ready_ = false;
  bool search_pending_ = false;
  uint64_t minimum_generation_ = 0;
  roscon_msgs::msg::IKSolutions solutions_;
  geometry_msgs::msg::PoseStamped requested_pose_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr search_commands_;
  rclcpp::Publisher<roscon_msgs::msg::IKSelection>::SharedPtr selection_commands_;
  rclcpp::Subscription<roscon_msgs::msg::IKSolutions>::SharedPtr solutions_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_sub_;
  QLabel *status_;
  QLabel *command_status_;
  QLabel *frame_;
  QTableWidget *joint_table_;
  std::vector<std::string> joint_names_;
  std::vector<QDoubleSpinBox *> joint_targets_;
  std::array<QDoubleSpinBox *, 6> pose_targets_;
  std::array<QTableWidgetItem *, 6> pose_values_;
  QPushButton *apply_joints_;
  QPushButton *solve_pose_;
  QPushButton *copy_joints_;
  QPushButton *copy_pose_;
  QPushButton *reset_;
  QTimer *timer_;
  urdf::Model model_;
  bool model_ready_ = false;
  bool pose_ready_ = false;
  bool joints_ready_ = false;
  bool status_ready_ = false;
  bool reset_pending_ = false;
  sensor_msgs::msg::JointState current_joints_;
  geometry_msgs::msg::PoseStamped current_pose_;
  std::array<double, 6> current_cartesian_{};
  std::chrono::steady_clock::time_point last_status_, last_joints_, last_pose_;
  rclcpp::Node::SharedPtr node_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_commands_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_commands_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joints_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr status_sub_, description_sub_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr reset_client_;
};
} // namespace roscon_rviz_plugins
