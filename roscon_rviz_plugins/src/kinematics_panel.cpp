#include "roscon_rviz_plugins/kinematics_panel.hpp"
#include <QHBoxLayout>
#include <QHeaderView>
#include <QScrollArea>
#include <QSignalBlocker>
#include <QTabWidget>
#include <QVBoxLayout>
#include <algorithm>
#include <cmath>
#include <pluginlib/class_list_macros.hpp>
#include <tf2/LinearMath/Matrix3x3.hpp>
#include <tf2/LinearMath/Quaternion.hpp>

namespace roscon_rviz_plugins {
namespace {
constexpr double radians_to_degrees = 180.0 / 3.141592653589793;
QTableWidget *makeTable(QWidget *parent, const QStringList &headers) {
  auto *table = new QTableWidget(0, headers.size(), parent);
  table->setHorizontalHeaderLabels(headers);
  table->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
  table->verticalHeader()->hide();
  table->setEditTriggers(QAbstractItemView::NoEditTriggers);
  table->setMinimumHeight(210);
  return table;
}
} // namespace

KinematicsPanel::KinematicsPanel(QWidget *parent) : rviz_common::Panel(parent) {
  auto *outer_layout = new QVBoxLayout(this);
  auto *hint =
      new QLabel("Use RViz's Interact tool to drag or rotate the target. Release to solve.", this);
  hint->setWordWrap(true);
  outer_layout->addWidget(hint);

  auto *scroll = new QScrollArea(this);
  scroll->setObjectName("kinematics_scroll");
  scroll->setWidgetResizable(true);
  scroll->setFrameShape(QFrame::NoFrame);
  auto *content = new QWidget(scroll);
  auto *layout = new QVBoxLayout(content);
  layout->setSizeConstraint(QLayout::SetMinAndMaxSize);
  layout->setContentsMargins(0, 0, 0, 0);
  scroll->setWidget(content);
  outer_layout->addWidget(scroll, 1);
  status_ = new QLabel("Waiting for IK backend", this);
  status_->setObjectName("ik_status");
  status_->setWordWrap(true);
  status_->setTextFormat(Qt::PlainText);
  layout->addWidget(status_);
  command_status_ = new QLabel("Edit a target, then press Apply or Solve.", this);
  command_status_->setWordWrap(true);
  command_status_->setTextFormat(Qt::PlainText);
  command_status_->setObjectName("command_status");
  layout->addWidget(command_status_);
  auto *tabs = new QTabWidget(this);
  layout->addWidget(tabs, 1);

  auto *joints_page = new QWidget(tabs);
  auto *joints_layout = new QVBoxLayout(joints_page);
  joints_layout->addWidget(new QLabel("Joint positions in radians", joints_page));
  joint_table_ = makeTable(joints_page, {"Joint", "Current", "Target"});
  joint_table_->setObjectName("joint_positions");
  joints_layout->addWidget(joint_table_);
  auto *joint_buttons = new QHBoxLayout;
  copy_joints_ = new QPushButton("Copy current joints", joints_page);
  copy_joints_->setObjectName("copy_joints");
  apply_joints_ = new QPushButton("Apply joints", joints_page);
  apply_joints_->setObjectName("apply_joints");
  joint_buttons->addWidget(copy_joints_);
  joint_buttons->addWidget(apply_joints_);
  joints_layout->addLayout(joint_buttons);
  auto *limits_note = new QLabel("Targets use URDF ranges. The backend also checks its joint-limit "
                                 "margin and singularity constraint before accepting a pose.",
                                 joints_page);
  limits_note->setWordWrap(true);
  joints_layout->addWidget(limits_note);
  tabs->addTab(joints_page, "Joints");

  auto *pose_page = new QWidget(tabs);
  auto *pose_layout = new QVBoxLayout(pose_page);
  frame_ = new QLabel("Waiting for end-effector pose", pose_page);
  frame_->setObjectName("cartesian_frame");
  pose_layout->addWidget(frame_);
  auto *pose_table = makeTable(pose_page, {"Coordinate", "Current", "Target"});
  pose_table->setObjectName("cartesian_positions");
  pose_table->setRowCount(6);
  const QStringList names = {"X (m)", "Y (m)", "Z (m)", "Roll (deg)", "Pitch (deg)", "Yaw (deg)"};
  for (int row = 0; row < 6; ++row) {
    pose_table->setItem(row, 0, new QTableWidgetItem(names[row]));
    pose_values_[row] = new QTableWidgetItem("—");
    pose_table->setItem(row, 1, pose_values_[row]);
    auto *target = new QDoubleSpinBox(pose_table);
    target->setObjectName("pose_target_" + QString::number(row));
    target->setDecimals(row < 3 ? 4 : 2);
    target->setRange(row < 3 ? -100.0 : -180.0, row < 3 ? 100.0 : 180.0);
    target->setSingleStep(row < 3 ? 0.01 : 1.0);
    pose_targets_[row] = target;
    pose_table->setCellWidget(row, 2, target);
  }
  pose_layout->addWidget(pose_table);
  auto *pose_buttons = new QHBoxLayout;
  copy_pose_ = new QPushButton("Copy current pose", pose_page);
  copy_pose_->setObjectName("copy_pose");
  solve_pose_ = new QPushButton("Solve pose", pose_page);
  solve_pose_->setObjectName("solve_pose");
  pose_buttons->addWidget(copy_pose_);
  pose_buttons->addWidget(solve_pose_);
  pose_layout->addLayout(pose_buttons);
  auto *orientation_note = new QLabel("Orientation: R = Rz(yaw) Ry(pitch) Rx(roll). Targets are in "
                                      "the displayed base frame. RPY is ambiguous at pitch ±90°.",
                                      pose_page);
  orientation_note->setWordWrap(true);
  pose_layout->addWidget(orientation_note);
  find_solutions_ = new QPushButton("Find alternatives for current target", pose_page);
  find_solutions_->setObjectName("find_solutions");
  find_solutions_->setEnabled(false);
  pose_layout->addWidget(find_solutions_);
  auto *search_note = new QLabel("Apply a pose or drag the target first. Searches use predefined "
                                 "seeds; completeness is not guaranteed.",
                                 pose_page);
  search_note->setWordWrap(true);
  pose_layout->addWidget(search_note);
  solution_summary_ = new QLabel("No solutions searched yet", pose_page);
  solution_summary_->setObjectName("solution_summary");
  solution_summary_->setWordWrap(true);
  pose_layout->addWidget(solution_summary_);
  solution_list_ = new QComboBox(pose_page);
  solution_list_->setObjectName("ik_solutions");
  pose_layout->addWidget(solution_list_);
  preview_solution_ = new QPushButton("Preview selected solution", pose_page);
  preview_solution_->setObjectName("preview_solution");
  preview_solution_->setEnabled(false);
  pose_layout->addWidget(preview_solution_);
  connect(find_solutions_, &QPushButton::clicked, this, &KinematicsPanel::findSolutions);
  connect(preview_solution_, &QPushButton::clicked, this, &KinematicsPanel::previewSolution);
  tabs->addTab(pose_page, "Cartesian");

  reset_ = new QPushButton("Reset to initial configuration", this);
  reset_->setObjectName("reset_ik");
  layout->addWidget(reset_);
  connect(copy_joints_, &QPushButton::clicked, this, &KinematicsPanel::copyJoints);
  connect(copy_pose_, &QPushButton::clicked, this, &KinematicsPanel::copyPose);
  connect(apply_joints_, &QPushButton::clicked, this, &KinematicsPanel::applyJoints);
  connect(solve_pose_, &QPushButton::clicked, this, &KinematicsPanel::solvePose);
  connect(reset_, &QPushButton::clicked, this, &KinematicsPanel::reset);
  for (auto *button : {copy_joints_, copy_pose_, apply_joints_, solve_pose_, reset_}) {
    button->setEnabled(false);
  }
  timer_ = new QTimer(this);
  connect(timer_, &QTimer::timeout, this, [this]() {
    try {
      update();
    } catch (const std::exception &error) {
      status_->setText(QString("Panel error: ") + error.what());
      timer_->stop();
      for (auto *button : {copy_joints_, copy_pose_, apply_joints_, solve_pose_, reset_}) {
        button->setEnabled(false);
      }
    }
  });
}
KinematicsPanel::~KinematicsPanel() {
  timer_->stop();
  if (node_) {
    executor_.remove_node(node_);
  }
}
void KinematicsPanel::onInitialize() {
  if (node_) {
    return;
  }
  static unsigned int count = 0;
  node_ = std::make_shared<rclcpp::Node>("roscon_kinematics_panel_" + std::to_string(++count));
  joint_commands_ = node_->create_publisher<sensor_msgs::msg::JointState>("/ik_joint_command", 10);
  pose_commands_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>("/target_pose", 10);
  search_commands_ =
      node_->create_publisher<geometry_msgs::msg::PoseStamped>("/ik_find_solutions", 10);
  selection_commands_ =
      node_->create_publisher<roscon_msgs::msg::IKSelection>("/ik_select_solution", 10);
  solutions_sub_ = node_->create_subscription<roscon_msgs::msg::IKSolutions>(
      "/ik_solutions", rclcpp::QoS(1).transient_local(),
      [this](roscon_msgs::msg::IKSolutions::ConstSharedPtr message) {
        receiveSolutions(*message);
      });
  target_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/ik_target_pose", 10, [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr message) {
        requested_pose_ = *message;
        target_ready_ = true;
      });
  reset_client_ = node_->create_client<std_srvs::srv::Trigger>("/reset_ik");
  description_sub_ = node_->create_subscription<std_msgs::msg::String>(
      "/robot_description", rclcpp::QoS(1).transient_local(),
      [this](std_msgs::msg::String::ConstSharedPtr message) {
        model_ready_ = model_.initString(message->data);
      });
  joints_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10,
      [this](sensor_msgs::msg::JointState::ConstSharedPtr message) { receiveJoints(*message); });
  pose_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/ik_current_pose", 10,
      [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr message) { receivePose(*message); });
  status_sub_ = node_->create_subscription<std_msgs::msg::String>(
      "/ik_status", 10, [this](std_msgs::msg::String::ConstSharedPtr message) {
        status_->setText(QString::fromStdString(message->data));
        status_ready_ = true;
        last_status_ = std::chrono::steady_clock::now();
      });
  executor_.add_node(node_);
  timer_->start(50);
}
bool KinematicsPanel::connected() const {
  const auto now = std::chrono::steady_clock::now();
  return status_ready_ && joints_ready_ && pose_ready_ &&
         now - last_status_ < std::chrono::seconds(2) &&
         now - last_joints_ < std::chrono::seconds(2) && now - last_pose_ < std::chrono::seconds(2);
}
void KinematicsPanel::update() {
  if (!rclcpp::ok()) {
    timer_->stop();
    return;
  }
  executor_.spin_some();
  const bool ready = connected();
  find_solutions_->setEnabled(ready && target_ready_ && !search_pending_ &&
                              search_commands_->get_subscription_count() > 0);
  preview_solution_->setEnabled(ready && solution_list_->count() > 0 &&
                                selection_commands_->get_subscription_count() > 0);
  if (status_ready_ && !ready) {
    status_->setText("IK backend unavailable or state stale; controls disabled.");
  }
  copy_joints_->setEnabled(ready);
  copy_pose_->setEnabled(ready);
  apply_joints_->setEnabled(ready && joint_commands_->get_subscription_count() > 0);
  solve_pose_->setEnabled(ready && pose_commands_->get_subscription_count() > 0);
  reset_->setEnabled(ready && !reset_pending_ && reset_client_->service_is_ready());
}
void KinematicsPanel::receiveJoints(const sensor_msgs::msg::JointState &message) {
  if (!model_ready_ || message.name.empty() || message.name.size() != message.position.size()) {
    return;
  }
  for (double value : message.position) {
    if (!std::isfinite(value)) {
      return;
    }
  }
  for (size_t i = 0; i < message.name.size(); ++i) {
    auto joint = model_.getJoint(message.name[i]);
    if (!joint || joint->type != urdf::Joint::REVOLUTE || !joint->limits) {
      return;
    }
    if (std::find(message.name.begin(), message.name.begin() + i, message.name[i]) !=
        message.name.begin() + i) {
      return;
    }
  }
  if (!joint_names_.empty() && joint_names_ != message.name) {
    return;
  }
  const bool first = joint_names_.empty();
  if (first) {
    joint_names_ = message.name;
    joint_table_->setRowCount(message.name.size());
    for (size_t row = 0; row < message.name.size(); ++row) {
      auto joint = model_.getJoint(message.name[row]);
      joint_table_->setItem(row, 0,
                            new QTableWidgetItem(QString::fromStdString(message.name[row])));
      joint_table_->setItem(row, 1, new QTableWidgetItem);
      auto *target = new QDoubleSpinBox(joint_table_);
      target->setObjectName(QString::fromStdString(message.name[row]) + "_target");
      target->setDecimals(5);
      target->setRange(joint->limits->lower, joint->limits->upper);
      target->setSingleStep(0.01);
      joint_table_->setCellWidget(row, 2, target);
      joint_targets_.push_back(target);
    }
  }
  current_joints_ = message;
  for (size_t row = 0; row < message.position.size(); ++row) {
    joint_table_->item(row, 1)->setText(QString::number(message.position[row], 'f', 5));
  }
  joints_ready_ = true;
  last_joints_ = std::chrono::steady_clock::now();
  if (first) {
    copyJoints();
  }
}
void KinematicsPanel::receivePose(const geometry_msgs::msg::PoseStamped &message) {
  const auto &p = message.pose.position;
  const auto &q = message.pose.orientation;
  if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z) ||
      message.header.frame_id.empty()) {
    return;
  }
  tf2::Quaternion rotation(q.x, q.y, q.z, q.w);
  if (!std::isfinite(rotation.length2()) || rotation.length2() < 1e-12) {
    return;
  }
  rotation.normalize();
  double roll, pitch, yaw;
  tf2::Matrix3x3(rotation).getRPY(roll, pitch, yaw);
  current_cartesian_ = {p.x,
                        p.y,
                        p.z,
                        roll * radians_to_degrees,
                        pitch * radians_to_degrees,
                        yaw * radians_to_degrees};
  for (int row = 0; row < 6; ++row) {
    pose_values_[row]->setText(QString::number(current_cartesian_[row], 'f', row < 3 ? 4 : 2));
  }
  frame_->setText("End-effector pose in " + QString::fromStdString(message.header.frame_id));
  current_pose_ = message;
  last_pose_ = std::chrono::steady_clock::now();
  const bool first = !pose_ready_;
  pose_ready_ = true;
  if (first) {
    copyPose();
  }
}
void KinematicsPanel::copyJoints() {
  if (!joints_ready_) {
    return;
  }
  for (size_t row = 0; row < joint_targets_.size(); ++row) {
    joint_targets_[row]->setValue(current_joints_.position[row]);
  }
}
void KinematicsPanel::copyPose() {
  if (!pose_ready_) {
    return;
  }
  for (int row = 0; row < 6; ++row) {
    pose_targets_[row]->setValue(current_cartesian_[row]);
  }
}
void KinematicsPanel::applyJoints() {
  if (!connected() || joint_commands_->get_subscription_count() == 0) {
    return;
  }
  sensor_msgs::msg::JointState command;
  command.header.stamp = node_->now();
  command.name = joint_names_;
  for (auto *target : joint_targets_) {
    command.position.push_back(target->value());
  }
  joint_commands_->publish(command);
  command_status_->setText("Joint target sent. See backend status above.");
}
void KinematicsPanel::solvePose() {
  if (!connected() || pose_commands_->get_subscription_count() == 0) {
    return;
  }
  geometry_msgs::msg::PoseStamped command;
  command.header.frame_id = current_pose_.header.frame_id;
  command.header.stamp = node_->now();
  command.pose.position.x = pose_targets_[0]->value();
  command.pose.position.y = pose_targets_[1]->value();
  command.pose.position.z = pose_targets_[2]->value();
  tf2::Quaternion q;
  q.setRPY(pose_targets_[3]->value() / radians_to_degrees,
           pose_targets_[4]->value() / radians_to_degrees,
           pose_targets_[5]->value() / radians_to_degrees);
  command.pose.orientation.x = q.x();
  command.pose.orientation.y = q.y();
  command.pose.orientation.z = q.z();
  command.pose.orientation.w = q.w();
  pose_commands_->publish(command);
  command_status_->setText("Cartesian target sent. See backend status above.");
}
void KinematicsPanel::findSolutions() {
  if (!connected() || !target_ready_ || search_pending_ ||
      search_commands_->get_subscription_count() == 0) {
    return;
  }
  minimum_generation_ = solutions_.generation + 1;
  search_pending_ = true;
  solution_list_->clear();
  find_solutions_->setEnabled(false);
  preview_solution_->setEnabled(false);
  solution_summary_->setText("Starting predefined-seed search…");
  search_commands_->publish(requested_pose_);
}
void KinematicsPanel::receiveSolutions(const roscon_msgs::msg::IKSolutions &message) {
  if (message.generation < minimum_generation_) {
    return;
  }
  const bool same_search = message.generation == solutions_.generation;
  const int previous = solution_list_->currentIndex();
  solutions_ = message;
  minimum_generation_ = message.generation;
  search_pending_ = !message.complete;
  const QSignalBlocker blocker(solution_list_);
  solution_list_->clear();
  for (size_t index = 0; index < message.solutions.size(); ++index) {
    const auto &candidate = message.solutions[index];
    solution_list_->addItem(QString("Solution %1 · seed %2 · %3 mm / %4 deg")
                                .arg(index + 1)
                                .arg(candidate.seed_index)
                                .arg(candidate.position_error * 1000.0, 0, 'f', 2)
                                .arg(candidate.orientation_error * radians_to_degrees, 0, 'f', 2));
  }
  if (same_search && previous >= 0 && previous < solution_list_->count()) {
    solution_list_->setCurrentIndex(previous);
  }
  if (message.seeds_total == 0) {
    solution_summary_->setText("Target changed. Find alternatives for the new target.");
  } else {
    solution_summary_->setText(
        QString("%1: %2 / %3 seeds; %4 collision-free solutions.\nTarget XYZ: %5, %6, %7 m")
            .arg(message.complete ? "Finished" : "Searching")
            .arg(message.seeds_tried)
            .arg(message.seeds_total)
            .arg(message.solutions.size())
            .arg(message.target.position.x, 0, 'f', 3)
            .arg(message.target.position.y, 0, 'f', 3)
            .arg(message.target.position.z, 0, 'f', 3));
    solution_summary_->setText(solution_summary_->text() +
                               QString("\nRejected candidates: %1 self / %2 environment; %3 errors")
                                   .arg(message.self_collision_rejections)
                                   .arg(message.environment_collision_rejections)
                                   .arg(message.check_errors));
    if (!message.last_rejection.empty()) {
      solution_summary_->setText(solution_summary_->text() + "\n" +
                                 QString::fromStdString(message.last_rejection));
    }
  }
}
void KinematicsPanel::previewSolution() {
  const int index = solution_list_->currentIndex();
  if (!connected() || index < 0 || static_cast<size_t>(index) >= solutions_.solutions.size() ||
      selection_commands_->get_subscription_count() == 0) {
    return;
  }
  roscon_msgs::msg::IKSelection selection;
  selection.generation = solutions_.generation;
  selection.index = index;
  selection_commands_->publish(selection);
  command_status_->setText("Preview requested. The target pose stays unchanged.");
}
void KinematicsPanel::reset() {
  if (!connected() || reset_pending_ || !reset_client_->service_is_ready()) {
    return;
  }
  reset_pending_ = true;
  reset_->setEnabled(false);
  command_status_->setText("Reset requested…");
  reset_client_->async_send_request(
      std::make_shared<std_srvs::srv::Trigger::Request>(),
      [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
        const auto reply = future.get();
        reset_pending_ = false;
        command_status_->setText(QString::fromStdString(reply->message));
      });
}
} // namespace roscon_rviz_plugins
PLUGINLIB_EXPORT_CLASS(roscon_rviz_plugins::KinematicsPanel, rviz_common::Panel)
