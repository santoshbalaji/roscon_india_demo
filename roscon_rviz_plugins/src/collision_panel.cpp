#include "roscon_rviz_plugins/collision_panel.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <QHeaderView>
#include <QHBoxLayout>
#include <QScrollArea>
#include <QSignalBlocker>
#include <QStringList>
#include <pluginlib/class_list_macros.hpp>
#include <urdf/model.h>
#include <rviz_common/display_context.hpp>
#include <rviz_common/display_group.hpp>

namespace roscon_rviz_plugins
{
CollisionPanel::CollisionPanel(QWidget * parent) : rviz_common::Panel(parent)
{
  auto * layout = new QVBoxLayout(this);
  layout->setContentsMargins(8, 8, 8, 8);
  layout->setSpacing(8);

  // Keep the current status and the backend selector visible on every tab.
  status_label_ = new QLabel("Waiting for collision status", this);
  status_label_->setObjectName("collision_summary");
  status_label_->setWordWrap(true);
  layout->addWidget(status_label_);
  backend_selector_ = new QComboBox(this);
  backend_selector_->setObjectName("collision_backend");
  backend_selector_->addItem("FCL geometry (CPU)");
  backend_selector_->addItem("Sphere pairs (CUDA GPU)");
  backend_selector_->addItem("Sphere / SDF (CUDA GPU)");
  layout->addWidget(backend_selector_);
  backend_error_ = new QLabel(this);
  backend_error_->setObjectName("backend_error");
  backend_error_->setWordWrap(true);
  backend_error_->setStyleSheet("color: #c0392b;");
  backend_error_->hide();
  layout->addWidget(backend_error_);

  tabs_ = new QTabWidget(this);
  tabs_->setObjectName("panel_tabs");
  layout->addWidget(tabs_, 1);

  // Joint controls get the full available height instead of sharing it with a table.
  auto * joints_page = new QWidget(tabs_);
  auto * joints_layout = new QVBoxLayout(joints_page);
  tabs_->addTab(joints_page, "Joints");
  robot_label_ = new QLabel("Waiting for /robot_description", joints_page);
  robot_label_->setWordWrap(true);
  joints_layout->addWidget(robot_label_);
  publish_checkbox_ = new QCheckBox("Publish joint states", joints_page);
  publish_checkbox_->setObjectName("publish_joint_states");
  publish_checkbox_->setChecked(true);
  joints_layout->addWidget(publish_checkbox_);
  publishing_label_ = new QLabel("Waiting for robot", joints_page);
  publishing_label_->setWordWrap(true);
  joints_layout->addWidget(publishing_label_);
  auto * scroll_area = new QScrollArea(joints_page);
  scroll_area->setWidgetResizable(true);
  scroll_area->setFrameShape(QFrame::NoFrame);
  auto * joint_widget = new QWidget(scroll_area);
  joint_layout_ = new QVBoxLayout(joint_widget);
  joint_layout_->setContentsMargins(0, 0, 0, 0);
  joint_layout_->setSpacing(10);
  joint_layout_->setAlignment(Qt::AlignTop);
  scroll_area->setWidget(joint_widget);
  joints_layout->addWidget(scroll_area, 1);
  reset_button_ = new QPushButton("Reset joints", joints_page);
  reset_button_->setObjectName("reset_joints");
  reset_button_->setEnabled(false);
  connect(reset_button_, &QPushButton::clicked, this, &CollisionPanel::resetJoints);
  joints_layout->addWidget(reset_button_);

  auto * collisions_page = new QWidget(tabs_);
  auto * collisions_layout = new QVBoxLayout(collisions_page);
  tabs_->addTab(collisions_page, "Collisions");
  collision_table_ = new QTableWidget(0, 3, collisions_page);
  collision_table_->setObjectName("collision_pairs");
  collision_table_->setHorizontalHeaderLabels({"Type", "Robot link", "Other"});
  collision_table_->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
  collision_table_->verticalHeader()->hide();
  collision_table_->setEditTriggers(QAbstractItemView::NoEditTriggers);
  collision_table_->setSelectionBehavior(QAbstractItemView::SelectRows);
  collisions_layout->addWidget(collision_table_, 1);
  auto * legend = new QLabel(
    "Magenta: self collision\nRed: environment collision\nYellow: both", collisions_page);
  collisions_layout->addWidget(legend);

  connect(backend_selector_, QOverload<int>::of(&QComboBox::currentIndexChanged), this,
    [this](int index) {requestBackend(index);});

  // ROS callbacks and widget updates run on the Qt thread. No worker thread is needed.
  timer_ = new QTimer(this);
  connect(timer_, &QTimer::timeout, this, [this]() {
    if (!rclcpp::ok()) {
      timer_->stop();
      return;
    }
    try {
      update();
    } catch (const std::exception & exception) {
      // ROS can shut down between the check above and a publisher operation.
      // Never let an exception escape a Qt event callback.
      timer_->stop();
      if (rclcpp::ok()) {
        status_label_->setText("Panel stopped: " + QString::fromUtf8(exception.what()));
        status_label_->setStyleSheet("");
        RCLCPP_ERROR(node_->get_logger(), "%s", exception.what());
      }
    }
  });
}

CollisionPanel::~CollisionPanel()
{
  timer_->stop();
  if (node_) {
    executor_.remove_node(node_);
  }
}

void CollisionPanel::onInitialize()
{
  if (node_) {
    return;
  }
  static unsigned int panel_number = 0;
  ++panel_number;
  node_ = std::make_shared<rclcpp::Node>("roscon_collision_panel_" + std::to_string(panel_number));
  joint_publisher_ = node_->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
  description_subscription_ = node_->create_subscription<std_msgs::msg::String>(
    "/robot_description", rclcpp::QoS(1).transient_local(),
    std::bind(&CollisionPanel::loadRobot, this, std::placeholders::_1));
  status_subscription_ = node_->create_subscription<std_msgs::msg::String>(
    "/collision_status", rclcpp::QoS(1).transient_local(),
    std::bind(&CollisionPanel::receiveStatus, this, std::placeholders::_1));
  joint_subscription_ = node_->create_subscription<sensor_msgs::msg::JointState>(
    "/joint_states", rclcpp::SensorDataQoS(),
    std::bind(&CollisionPanel::receiveJointState, this, std::placeholders::_1));
  backend_client_ = node_->create_client<std_srvs::srv::SetBool>("/roscon_collision/use_gpu_spheres");
  sdf_client_ = node_->create_client<std_srvs::srv::SetBool>("/roscon_collision/use_sdf");
  backend_subscription_ = node_->create_subscription<std_msgs::msg::String>(
    "/collision_backend", rclcpp::QoS(1).transient_local(),
    std::bind(&CollisionPanel::receiveBackend, this, std::placeholders::_1));
  executor_.add_node(node_);
  timer_->start(50);
}

void CollisionPanel::receiveBackend(const std_msgs::msg::String::SharedPtr message)
{
  confirmed_backend_ = 0;
  if (message->data.find("gpu_spheres") == 0) {
    confirmed_backend_ = 1;
  } else if (message->data.find("gpu_sdf") == 0) {
    confirmed_backend_ = 2;
  }
  const QSignalBlocker blocker(backend_selector_);
  backend_selector_->setCurrentIndex(confirmed_backend_);
  updateVisualization();
}

void CollisionPanel::updateVisualization()
{
  if (getDisplayContext() == nullptr || visualization_backend_ == confirmed_backend_) {
    return;
  }
  // These are the display names in collision.rviz. Sphere mode hides the robot mesh too.
  rviz_common::DisplayGroup * displays = getDisplayContext()->getRootDisplayGroup();
  for (int index = 0; index < displays->numDisplays(); ++index) {
    rviz_common::Display * display = displays->getDisplayAt(index);
    if (display->getName() == "Robot description") {
      display->setEnabled(confirmed_backend_ == 0);
    } else if (display->getName() == "FCL collision geometry") {
      // SDF mode uses this topic for environment surfaces only.
      display->setEnabled(confirmed_backend_ != 1);
    } else if (display->getName() == "GPU sphere geometry") {
      display->setEnabled(confirmed_backend_ != 0);
    } else if (display->getName() == "SDF distance slices") {
      display->setEnabled(confirmed_backend_ == 2);
    }
  }
  visualization_backend_ = confirmed_backend_;
}

void CollisionPanel::requestBackend(int backend)
{
  // Keep the selector on the confirmed mode until the node accepts the change.
  const QSignalBlocker blocker(backend_selector_);
  backend_selector_->setCurrentIndex(confirmed_backend_);
  backend_error_->hide();
  auto client = backend_client_;
  if (backend == 2) {
    client = sdf_client_;
  }
  if (!client || !client->service_is_ready()) {
    backend_error_->setText("Collision service unavailable. Start the collision node.");
    backend_error_->show();
    return;
  }
  backend_selector_->setEnabled(false);
  auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
  request->data = backend != 0;
  const auto pending = client->async_send_request(request,
    [this](rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture future) {
      backend_selector_->setEnabled(true);
      const auto response = future.get();
      if (!response->success) {
        backend_error_->setText(QString::fromStdString(response->message));
        backend_error_->show();
      }
    });
  const int64_t request_id = pending.request_id;
  QTimer::singleShot(5000, this, [this, client, request_id]() {
    if (client->remove_pending_request(request_id)) {
      backend_selector_->setEnabled(true);
      backend_error_->setText("Backend change timed out. Showing the last confirmed mode.");
      backend_error_->show();
    }
  });
}

void CollisionPanel::loadRobot(const std_msgs::msg::String::SharedPtr message)
{
  if (message->data == loaded_description_) {
    return;
  }
  robot_loaded_ = false;
  publishing_ = false;
  loaded_description_.clear();
  joint_controls_.clear();
  while (joint_layout_->count() > 0) {
    QLayoutItem * item = joint_layout_->takeAt(0);
    delete item->widget();
    delete item;
  }
  urdf::Model robot;
  if (!robot.initString(message->data)) {
    robot_label_->setText("Invalid robot description. Joint publishing stopped.");
    return;
  }
  // Validate all joints before creating controls, so unsupported robots cannot publish partial states.
  for (const auto & entry : robot.joints_) {
    const urdf::Joint & joint = *entry.second;
    if (joint.type == urdf::Joint::FIXED) {
      continue;
    }
    if (joint.mimic || (joint.type != urdf::Joint::REVOLUTE &&
      joint.type != urdf::Joint::CONTINUOUS && joint.type != urdf::Joint::PRISMATIC))
    {
      robot_label_->setText("Unsupported joint: " + QString::fromStdString(joint.name));
      return;
    }
    if (joint.type != urdf::Joint::CONTINUOUS &&
      (!joint.limits || !std::isfinite(joint.limits->lower) ||
      !std::isfinite(joint.limits->upper) || joint.limits->lower > joint.limits->upper))
    {
      robot_label_->setText("Invalid limits for " + QString::fromStdString(joint.name));
      return;
    }
  }
  for (const auto & entry : robot.joints_) {
    const urdf::Joint & joint = *entry.second;
    if (joint.type == urdf::Joint::FIXED) {
      continue;
    }
    double minimum = -3.141592653589793;
    double maximum = 3.141592653589793;
    if (joint.type != urdf::Joint::CONTINUOUS) {
      minimum = joint.limits->lower;
      maximum = joint.limits->upper;
    }
    addJoint(joint.name, minimum, maximum, joint.type == urdf::Joint::PRISMATIC);
  }
  loaded_description_ = message->data;
  robot_loaded_ = true;
  robot_label_->setText(QString::fromStdString(robot.getName()) + " — joint positions");
}

void CollisionPanel::addJoint(
  const std::string & name, double minimum, double maximum, bool prismatic)
{
  auto * row = new QWidget(this);
  auto * row_layout = new QHBoxLayout(row);
  row_layout->setContentsMargins(0, 0, 0, 0);
  auto * name_label = new QLabel(QString::fromStdString(name), row);
  name_label->setMinimumWidth(55);
  row_layout->addWidget(name_label);
  auto * slider = new QSlider(Qt::Horizontal, row);
  slider->setObjectName(QString::fromStdString(name) + "_slider");
  slider->setRange(0, 1000);
  auto * position = new QDoubleSpinBox(row);
  position->setObjectName(QString::fromStdString(name) + "_position");
  position->setDecimals(4);
  position->setRange(minimum, maximum);
  position->setSingleStep(0.01);
  if (prismatic) {
    position->setSuffix(" m");
  } else {
    position->setSuffix(" rad");
  }
  row_layout->addWidget(slider, 1);
  row_layout->addWidget(position);
  joint_layout_->addWidget(row);
  JointControl control{name, minimum, maximum, slider, position};
  joint_controls_.push_back(control);
  setJointPosition(joint_controls_.back(), 0.0);

  connect(slider, &QSlider::valueChanged, this,
    [position, minimum, maximum](int value) {
      const QSignalBlocker blocker(position);
      position->setValue(minimum + (maximum - minimum) * value / 1000.0);
    });
  connect(position, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
    [slider, minimum, maximum](double value) {
      const QSignalBlocker blocker(slider);
      if (maximum > minimum) {
        slider->setValue(static_cast<int>(std::round(1000.0 * (value - minimum) / (maximum - minimum))));
      }
    });
}

void CollisionPanel::setJointPosition(JointControl & control, double position)
{
  const QSignalBlocker slider_blocker(control.slider);
  const QSignalBlocker position_blocker(control.position);
  const double bounded = std::clamp(position, control.minimum, control.maximum);
  control.position->setValue(bounded);
  int slider_position = 0;
  if (control.maximum > control.minimum) {
    slider_position = static_cast<int>(std::round(
      1000.0 * (bounded - control.minimum) / (control.maximum - control.minimum)));
  }
  control.slider->setValue(slider_position);
}

void CollisionPanel::resetJoints()
{
  for (JointControl & control : joint_controls_) {
    setJointPosition(control, 0.0);
  }
}

void CollisionPanel::receiveStatus(const std_msgs::msg::String::SharedPtr message)
{
  last_status_time_ = std::chrono::steady_clock::now();
  status_received_ = true;
  if (message->data == previous_status_) {
    return;
  }
  previous_status_ = message->data;
  collision_table_->setRowCount(0);
  const QString text = QString::fromStdString(message->data);
  const QStringList lines = text.split('\n', Qt::SkipEmptyParts);
  for (const QString & line : lines) {
    QString type;
    QString pair;
    if (line.startsWith("SELF: ")) {
      type = "Self";
      pair = line.mid(6);
    } else if (line.startsWith("ENVIRONMENT: ")) {
      type = "Environment";
      pair = line.mid(13);
    } else {
      continue;
    }
    const QStringList names = pair.split(" <-> ");
    if (names.size() != 2) {
      continue;
    }
    const int row = collision_table_->rowCount();
    collision_table_->insertRow(row);
    collision_table_->setItem(row, 0, new QTableWidgetItem(type));
    collision_table_->setItem(row, 1, new QTableWidgetItem(names[0]));
    collision_table_->setItem(row, 2, new QTableWidgetItem(names[1]));
  }
  if (collision_table_->rowCount() > 0) {
    status_label_->setText(QString::number(collision_table_->rowCount()) + " colliding pairs");
    status_label_->setStyleSheet("color: #c0392b; font-weight: bold;");
  } else if (text == "Collision free") {
    status_label_->setText("Collision free");
    status_label_->setStyleSheet("color: #218c45; font-weight: bold;");
  } else {
    status_label_->setText(text);
    status_label_->setStyleSheet("");
  }
}

void CollisionPanel::receiveJointState(const sensor_msgs::msg::JointState::SharedPtr message)
{
  if (publishing_ || message->name.size() != message->position.size()) {
    return;
  }
  for (std::size_t index = 0; index < message->name.size(); ++index) {
    if (!std::isfinite(message->position[index])) {
      continue;
    }
    for (JointControl & control : joint_controls_) {
      if (control.name == message->name[index]) {
        setJointPosition(control, message->position[index]);
      }
    }
  }
}

void CollisionPanel::publishJointStates()
{
  sensor_msgs::msg::JointState message;
  message.header.stamp = node_->now();
  for (const JointControl & control : joint_controls_) {
    message.name.push_back(control.name);
    message.position.push_back(control.position->value());
  }
  joint_publisher_->publish(message);
}

void CollisionPanel::update()
{
  // Pause if another publisher is present, avoiding two sources fighting over joint positions.
  const bool external_publisher = node_->count_publishers("/joint_states") > 1;
  publishing_ = robot_loaded_ && publish_checkbox_->isChecked() && !external_publisher;
  executor_.spin_some();
  publishing_ = robot_loaded_ && publish_checkbox_->isChecked() && !external_publisher;
  for (JointControl & control : joint_controls_) {
    control.slider->setEnabled(publishing_);
    control.position->setEnabled(publishing_);
  }
  reset_button_->setEnabled(publishing_);
  if (!robot_loaded_) {
    publishing_label_->setText("Waiting for a supported robot description");
  } else if (external_publisher) {
    publishing_label_->setText("External joint publisher detected. Panel controls are paused.");
  } else if (!publish_checkbox_->isChecked()) {
    publishing_label_->setText("Joint publishing paused");
  } else {
    publishing_label_->setText("Joint controls active");
    publishJointStates();
  }
  if (status_received_ &&
    std::chrono::steady_clock::now() - last_status_time_ > std::chrono::seconds(2))
  {
    status_label_->setText("Collision status unavailable (no update for 2 seconds)");
    status_label_->setStyleSheet("");
    collision_table_->setRowCount(0);
    previous_status_.clear();
    status_received_ = false;
  }
}
}  // namespace roscon_rviz_plugins

PLUGINLIB_EXPORT_CLASS(roscon_rviz_plugins::CollisionPanel, rviz_common::Panel)
