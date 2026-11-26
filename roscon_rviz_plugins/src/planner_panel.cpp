#include "roscon_rviz_plugins/planner_panel.hpp"
#include <QFormLayout>
#include <QHBoxLayout>
#include <QScrollArea>
#include <QSignalBlocker>
#include <QVBoxLayout>
#include <cmath>
#include <pluginlib/class_list_macros.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

namespace roscon_rviz_plugins {
static constexpr double degrees = 180.0 / 3.141592653589793;
PlannerPanel::PlannerPanel(QWidget *parent) : rviz_common::Panel(parent) {
  auto *outer = new QVBoxLayout(this);
  auto *scroll = new QScrollArea(this);
  scroll->setWidgetResizable(true);
  scroll->setObjectName("planner_scroll");
  auto *content = new QWidget(scroll);
  auto *layout = new QVBoxLayout(content);
  status_ = new QLabel("Waiting for planner...", content);
  status_->setWordWrap(true);
  status_->setObjectName("planner_status");
  layout->addWidget(status_);
  feedback_ = new QLabel("Set a target or use RViz Interact to drag the handles.", content);
  feedback_->setWordWrap(true);
  layout->addWidget(feedback_);
  backend_ = new QComboBox(content);
  backend_->setObjectName("collision_backend");
  backend_->addItems({"fcl", "spheres", "sdf"});
  auto *form = new QFormLayout();
  form->addRow("Collision checker", backend_);
  const std::array<QString, 6> names = {"X (m)",      "Y (m)",       "Z (m)",
                                        "Roll (deg)", "Pitch (deg)", "Yaw (deg)"};
  for (size_t i = 0; i < target_.size(); ++i) {
    target_[i] = new QDoubleSpinBox(content);
    target_[i]->setObjectName("target_" + QString::number(i));
    target_[i]->setRange(i < 3 ? -10.0 : -180.0, i < 3 ? 10.0 : 180.0);
    target_[i]->setDecimals(i < 3 ? 4 : 2);
    target_[i]->setSingleStep(i < 3 ? 0.01 : 1.0);
    form->addRow(names[i], target_[i]);
  }
  layout->addLayout(form);
  copy_ = new QPushButton("Copy current pose", content);
  copy_->setObjectName("copy_current");
  demo_ = new QPushButton("Load demo target", content);
  demo_->setObjectName("demo_target");
  auto *row = new QHBoxLayout();
  row->addWidget(copy_);
  row->addWidget(demo_);
  layout->addLayout(row);
  auto *algorithms = new QFormLayout();
  algorithm_ = new QComboBox(content);
  algorithm_->setObjectName("planning_algorithm");
  algorithm_->addItem("RRTConnect", "rrt_connect");
  algorithm_->addItem("RRTstar", "rrt_star");
  algorithm_->addItem("PRM", "prm");
  optimizer_ = new QComboBox(content);
  optimizer_->setObjectName("optimization_algorithm");
  optimizer_->addItem("STOMP", "stomp");
  optimizer_->addItem("CHOMP-style covariant", "chomp");
  optimizer_->addItem("None (OMPL only)", "none");
  algorithms->addRow("OMPL planner", algorithm_);
  algorithms->addRow("Optimizer", optimizer_);
  layout->addLayout(algorithms);
  plan_ = new QPushButton("Plan", content);
  plan_->setObjectName("plan_motion");
  validate_ = new QPushButton("Validate stored path", content);
  validate_->setObjectName("validate_path");
  row = new QHBoxLayout();
  row->addWidget(plan_);
  row->addWidget(validate_);
  layout->addLayout(row);
  play_ = new QPushButton("Play / resume", content);
  play_->setObjectName("play_path");
  stop_ = new QPushButton("Pause", content);
  stop_->setObjectName("stop_path");
  row = new QHBoxLayout();
  row->addWidget(play_);
  row->addWidget(stop_);
  layout->addLayout(row);
  reset_ = new QPushButton("Reset to demo start", content);
  reset_->setObjectName("reset_planner");
  layout->addWidget(reset_);
  current_label_ = new QLabel(content);
  current_label_->setWordWrap(true);
  layout->addWidget(current_label_);
  joints_label_ = new QLabel(content);
  joints_label_->setWordWrap(true);
  layout->addWidget(joints_label_);
  auto *note = new QLabel("Green line: validated tool path. Red line: needs validation.\n"
                          "Backend changes pause playback. Validation samples every path segment.\n"
                          "Playback and reset are visualization only; no hardware commands.",
                          content);
  note->setWordWrap(true);
  layout->addWidget(note);
  layout->addStretch();
  scroll->setWidget(content);
  outer->addWidget(scroll);
  connect(copy_, &QPushButton::clicked, this, [this]() { fillTarget(current_pose_); });
  connect(demo_, &QPushButton::clicked, this, [this]() { fillTarget(demo_pose_); });
  connect(plan_, &QPushButton::clicked, this, &PlannerPanel::plan);
  connect(validate_, &QPushButton::clicked, this, &PlannerPanel::validate);
  connect(backend_, qOverload<int>(&QComboBox::currentIndexChanged), this,
          &PlannerPanel::changeBackend);
  connect(play_, &QPushButton::clicked, this, [this]() { trigger(play_client_); });
  connect(stop_, &QPushButton::clicked, this, [this]() { trigger(stop_client_); });
  connect(reset_, &QPushButton::clicked, this, [this]() { trigger(reset_client_); });
  timer_ = new QTimer(this);
  connect(timer_, &QTimer::timeout, this, &PlannerPanel::update);
  for (auto *button : {plan_, validate_, play_, stop_, reset_, copy_, demo_}) {
    button->setEnabled(false);
  }
  backend_->setEnabled(false);
}
PlannerPanel::~PlannerPanel() {
  timer_->stop();
  if (node_) {
    executor_.remove_node(node_);
  }
}
void PlannerPanel::onInitialize() {
  if (node_) {
    return;
  }
  static unsigned int count = 0;
  node_ = std::make_shared<rclcpp::Node>("roscon_planner_panel_" + std::to_string(++count));
  plan_client_ = node_->create_client<roscon_msgs::srv::PlanMotion>("/planner/plan");
  backend_client_ =
      node_->create_client<roscon_msgs::srv::SetCollisionBackend>("/planner/set_backend");
  validate_client_ =
      node_->create_client<roscon_msgs::srv::ValidateTrajectory>("/planner/validate");
  play_client_ = node_->create_client<std_srvs::srv::Trigger>("/planner/play");
  stop_client_ = node_->create_client<std_srvs::srv::Trigger>("/planner/stop");
  reset_client_ = node_->create_client<std_srvs::srv::Trigger>("/planner/reset");
  status_sub_ = node_->create_subscription<std_msgs::msg::String>(
      "/planner/status", 10, [this](std_msgs::msg::String::ConstSharedPtr message) {
        status_->setText(QString::fromStdString(message->data));
        last_status_ = std::chrono::steady_clock::now();
      });
  backend_sub_ = node_->create_subscription<std_msgs::msg::String>(
      "/planner/backend", rclcpp::QoS(1).transient_local(),
      [this](std_msgs::msg::String::ConstSharedPtr message) {
        const QSignalBlocker blocker(backend_);
        backend_->setCurrentText(QString::fromStdString(message->data));
      });
  target_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/planner/target_pose", rclcpp::QoS(1).transient_local(),
      [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr message) { fillTarget(*message); });
  demo_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/planner/demo_target", rclcpp::QoS(1).transient_local(),
      [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr message) {
        demo_pose_ = *message;
        demo_ready_ = true;
      });
  current_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/planner/current_pose", 10, [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr message) {
        current_pose_ = *message;
        current_ready_ = true;
        current_label_->setText(QString("Current XYZ (%1): %2, %3, %4 m")
                                    .arg(QString::fromStdString(message->header.frame_id))
                                    .arg(message->pose.position.x, 0, 'f', 4)
                                    .arg(message->pose.position.y, 0, 'f', 4)
                                    .arg(message->pose.position.z, 0, 'f', 4));
      });
  joints_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10, [this](sensor_msgs::msg::JointState::ConstSharedPtr message) {
        if (message->name.size() != message->position.size()) {
          return;
        }
        QString text = "Current joints (rad):";
        for (size_t i = 0; i < message->name.size(); ++i) {
          text += QString("\n%1: %2")
                      .arg(QString::fromStdString(message->name[i]))
                      .arg(message->position[i], 0, 'f', 4);
        }
        joints_label_->setText(text);
      });
  executor_.add_node(node_);
  timer_->start(50);
}
void PlannerPanel::setBusy(bool busy) {
  busy_ = busy;
  updateControls();
}
void PlannerPanel::update() {
  executor_.spin_some();
  updateControls();
}
void PlannerPanel::updateControls() {
  const bool connected =
      current_ready_ &&
      std::chrono::duration<double>(std::chrono::steady_clock::now() - last_status_).count() < 2.0;
  plan_->setEnabled(connected && !busy_ && plan_client_->service_is_ready());
  validate_->setEnabled(connected && !busy_ && validate_client_->service_is_ready());
  backend_->setEnabled(connected && !busy_ && backend_client_->service_is_ready());
  play_->setEnabled(connected && !busy_ && play_client_->service_is_ready());
  stop_->setEnabled(connected && !busy_ && stop_client_->service_is_ready());
  reset_->setEnabled(connected && !busy_ && reset_client_->service_is_ready());
  copy_->setEnabled(connected && !busy_);
  demo_->setEnabled(connected && !busy_ && demo_ready_);
  for (auto *field : target_) {
    field->setEnabled(!busy_);
  }
  algorithm_->setEnabled(!busy_);
  optimizer_->setEnabled(!busy_);
}
void PlannerPanel::fillTarget(const geometry_msgs::msg::PoseStamped &message) {
  const auto &pose = message.pose;
  tf2::Quaternion q(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
  if (q.length2() < 1e-12) {
    return;
  }
  double roll, pitch, yaw;
  tf2::Matrix3x3(q.normalized()).getRPY(roll, pitch, yaw);
  const std::array<double, 6> values = {pose.position.x, pose.position.y, pose.position.z,
                                        roll * degrees,  pitch * degrees, yaw * degrees};
  for (size_t i = 0; i < values.size(); ++i) {
    target_[i]->setValue(values[i]);
  }
}
geometry_msgs::msg::PoseStamped PlannerPanel::targetMessage() const {
  geometry_msgs::msg::PoseStamped result;
  result.header.frame_id = current_pose_.header.frame_id;
  result.pose.position.x = target_[0]->value();
  result.pose.position.y = target_[1]->value();
  result.pose.position.z = target_[2]->value();
  tf2::Quaternion q;
  q.setRPY(target_[3]->value() / degrees, target_[4]->value() / degrees,
           target_[5]->value() / degrees);
  result.pose.orientation.x = q.x();
  result.pose.orientation.y = q.y();
  result.pose.orientation.z = q.z();
  result.pose.orientation.w = q.w();
  return result;
}
void PlannerPanel::plan() {
  if (busy_ || !plan_client_->service_is_ready()) {
    return;
  }
  auto request = std::make_shared<roscon_msgs::srv::PlanMotion::Request>();
  request->target = targetMessage();
  request->ompl_planner = algorithm_->currentData().toString().toStdString();
  request->optimizer = optimizer_->currentData().toString().toStdString();
  setBusy(true);
  feedback_->setText("Planning and validating... The robot stays at its current pose.");
  plan_client_->async_send_request(
      request, [this](rclcpp::Client<roscon_msgs::srv::PlanMotion>::SharedFuture future) {
        const auto response = future.get();
        setBusy(false);
        feedback_->setText(QString::fromStdString(
            (response->success ? "Plan ready: " : "Plan rejected: ") + response->message));
      });
}
void PlannerPanel::validate() {
  if (busy_ || !validate_client_->service_is_ready()) {
    return;
  }
  setBusy(true);
  feedback_->setText("Validating every path segment...");
  auto request = std::make_shared<roscon_msgs::srv::ValidateTrajectory::Request>();
  validate_client_->async_send_request(
      request, [this](rclcpp::Client<roscon_msgs::srv::ValidateTrajectory>::SharedFuture future) {
        const auto response = future.get();
        setBusy(false);
        feedback_->setText(QString::fromStdString(response->message));
      });
}
void PlannerPanel::changeBackend() {
  if (!node_ || busy_ || !backend_client_->service_is_ready()) {
    return;
  }
  setBusy(true);
  auto request = std::make_shared<roscon_msgs::srv::SetCollisionBackend::Request>();
  request->backend = backend_->currentText().toStdString();
  backend_client_->async_send_request(
      request, [this](rclcpp::Client<roscon_msgs::srv::SetCollisionBackend>::SharedFuture future) {
        const auto response = future.get();
        setBusy(false);
        feedback_->setText(QString::fromStdString(response->message));
      });
}
void PlannerPanel::trigger(const rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr &client) {
  if (busy_ || !client || !client->service_is_ready()) {
    return;
  }
  setBusy(true);
  client->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>(),
                             [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
                               const auto response = future.get();
                               setBusy(false);
                               feedback_->setText(QString::fromStdString(response->message));
                             });
}
} // namespace roscon_rviz_plugins
PLUGINLIB_EXPORT_CLASS(roscon_rviz_plugins::PlannerPanel, rviz_common::Panel)
