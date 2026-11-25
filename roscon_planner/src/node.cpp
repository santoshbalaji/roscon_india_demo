#include "roscon_planner/planner.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <interactive_markers/interactive_marker_server.hpp>
#include <iterator>
#include <rclcpp/rclcpp.hpp>
#include <roscon_msgs/srv/plan_motion.hpp>
#include <roscon_msgs/srv/set_collision_backend.hpp>
#include <roscon_msgs/srv/validate_trajectory.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>

namespace roscon_planner {
class PlannerNode : public rclcpp::Node {
public:
  PlannerNode() : Node("roscon_planner") {
    const auto file = declare_parameter<std::string>("urdf_file", "");
    const auto environment = declare_parameter<std::string>("environment_file", "");
    base_ = declare_parameter<std::string>("base_link", "base_link");
    const auto tip = declare_parameter<std::string>("tip_link", "flange");
    backend_ = declare_parameter<std::string>("collision_backend", "fcl");
    if (!Planner::knownBackend(backend_)) {
      throw std::runtime_error("Invalid collision_backend");
    }
    PlanSettings settings;
    settings.planning_seconds = declare_parameter("planning_seconds", settings.planning_seconds);
    settings.joint_step = declare_parameter("validation_joint_step", settings.joint_step);
    settings.stomp_iterations = declare_parameter("stomp_iterations", settings.stomp_iterations);
    settings.stomp_rollouts = declare_parameter("stomp_rollouts", settings.stomp_rollouts);
    settings.chomp_iterations = declare_parameter("chomp_iterations", settings.chomp_iterations);
    settings.clearance_margin = declare_parameter("clearance_margin", settings.clearance_margin);
    settings.noise_stddev = declare_parameter("noise_stddev", settings.noise_stddev);
    roscon_kinematics::Settings ik;
    ik.minimum_singular_value =
        declare_parameter("minimum_singular_value", ik.minimum_singular_value);
    if (!std::isfinite(ik.minimum_singular_value) || ik.minimum_singular_value < 0.0) {
      throw std::runtime_error("Invalid minimum_singular_value");
    }
    std::ifstream input(file);
    if (!input) {
      throw std::runtime_error("Cannot open robot URDF: " + file);
    }
    const std::string xml((std::istreambuf_iterator<char>(input)),
                          std::istreambuf_iterator<char>());
    planner_ = std::make_unique<Planner>(xml, environment, base_, tip, ik, settings);
    const auto initial =
        declare_parameter<std::vector<double>>("initial_joints", {0.0, -1.0, 1.2, -1.0, 1.0, 0.0});
    home_ = Eigen::Map<const Eigen::VectorXd>(initial.data(), initial.size());
    std::string reason;
    if (!planner_->validState(home_, backend_, reason)) {
      throw std::runtime_error("Invalid initial state: " + reason);
    }
    current_ = home_;
    target_ = poseMessage(planner_->robot().pose(home_));
    joint_pub_ = create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
    pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("planner/current_pose", 10);
    target_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
        "planner/target_pose", rclcpp::QoS(1).transient_local());
    demo_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("planner/demo_target",
                                                                  rclcpp::QoS(1).transient_local());
    status_pub_ = create_publisher<std_msgs::msg::String>("planner/status", 10);
    backend_pub_ = create_publisher<std_msgs::msg::String>("planner/backend",
                                                           rclcpp::QoS(1).transient_local());
    trajectory_pub_ = create_publisher<trajectory_msgs::msg::JointTrajectory>(
        "planner/trajectory", rclcpp::QoS(1).transient_local());
    markers_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
        "planner/path_markers", rclcpp::QoS(1).transient_local());
    environment_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
        "planner/environment", rclcpp::QoS(1).transient_local());
    environment_pub_->publish(planner_->environmentMarkers());
    server_ =
        std::make_unique<interactive_markers::InteractiveMarkerServer>("planner_target", this);
    createMarker();
    publishTarget();
    auto demo = home_;
    demo[0] += 0.9;
    demo[1] += 0.10;
    const auto demo_joints = declare_parameter<std::vector<double>>(
        "demo_joints", std::vector<double>(demo.data(), demo.data() + demo.size()));
    if (demo_joints.size() != static_cast<size_t>(home_.size())) {
      throw std::runtime_error("demo_joints must have the same size as initial_joints");
    }
    demo = Eigen::Map<const Eigen::VectorXd>(demo_joints.data(), demo_joints.size());
    if (!demo.allFinite() || (demo.array() < planner_->robot().lower.array()).any() ||
        (demo.array() > planner_->robot().upper.array()).any()) {
      throw std::runtime_error("demo_joints must be finite and inside the robot joint limits");
    }
    geometry_msgs::msg::PoseStamped demo_pose;
    demo_pose.header.frame_id = base_;
    demo_pose.pose = poseMessage(planner_->robot().pose(demo));
    demo_pub_->publish(demo_pose);
    plan_service_ = create_service<roscon_msgs::srv::PlanMotion>(
        "planner/plan", [this](roscon_msgs::srv::PlanMotion::Request::ConstSharedPtr request,
                               roscon_msgs::srv::PlanMotion::Response::SharedPtr response) {
          invalidatePath();
          try {
            setTarget(request->target);
            status_ = "Planning with " + backend_ + "...";
            publishState();
            const Eigen::Quaterniond orientation(target_.orientation.w, target_.orientation.x,
                                                 target_.orientation.y, target_.orientation.z);
            roscon_kinematics::Target goal;
            goal.position =
                Eigen::Vector3d(target_.position.x, target_.position.y, target_.position.z);
            goal.rotation = orientation.toRotationMatrix();
            const auto result = planner_->plan(
                current_, goal, backend_,
                request->ompl_planner.empty() ? "rrt_connect" : request->ompl_planner,
                request->optimizer.empty() ? "stomp" : request->optimizer);
            response->success = result.success;
            response->message = result.message;
            if (result.success) {
              path_ = result.path;
              valid_ = true;
              elapsed_ = 0.0;
              trajectory_ = toTrajectory(path_);
              response->trajectory = trajectory_;
            }
            status_ = result.message;
          } catch (const std::exception &error) {
            response->success = false;
            response->message = error.what();
            status_ = "Planning failed: " + response->message;
          }
          publishPath();
          publishState();
        });
    backend_service_ = create_service<roscon_msgs::srv::SetCollisionBackend>(
        "planner/set_backend",
        [this](roscon_msgs::srv::SetCollisionBackend::Request::ConstSharedPtr request,
               roscon_msgs::srv::SetCollisionBackend::Response::SharedPtr response) {
          if (!Planner::knownBackend(request->backend)) {
            response->success = false;
            response->message = "Choose fcl, spheres, or sdf";
            return;
          }
          playing_ = false;
          valid_ = false;
          backend_ = request->backend;
          response->success = true;
          response->message = "Backend: " + backend_ + ". Validate the stored path or plan again.";
          status_ = response->message;
          publishPath();
          publishState();
        });
    validate_service_ = create_service<roscon_msgs::srv::ValidateTrajectory>(
        "planner/validate",
        [this](roscon_msgs::srv::ValidateTrajectory::Request::ConstSharedPtr request,
               roscon_msgs::srv::ValidateTrajectory::Response::SharedPtr response) {
          const bool stored = request->trajectory.points.empty();
          const std::string backend = request->backend.empty() ? backend_ : request->backend;
          playing_ = false;
          try {
            const auto path = stored ? path_ : fromTrajectory(request->trajectory);
            const auto result = planner_->validate(path, backend);
            response->valid = result.valid;
            response->checked_states = result.checked_states;
            response->message = backend + ": " + result.message;
            if (stored && backend == backend_) {
              valid_ = result.valid;
            }
          } catch (const std::exception &error) {
            response->valid = false;
            response->message = error.what();
            if (stored && backend == backend_) {
              valid_ = false;
            }
          }
          status_ = response->message;
          publishPath();
          publishState();
        });
    play_service_ = create_service<std_srvs::srv::Trigger>(
        "planner/play", [this](std_srvs::srv::Trigger::Request::ConstSharedPtr,
                               std_srvs::srv::Trigger::Response::SharedPtr response) {
          try {
            if (!valid_ || path_.empty()) {
              throw std::runtime_error("Plan or validate with the active backend first");
            }
            const auto validation = planner_->validate(path_, backend_);
            if (!validation.valid) {
              valid_ = false;
              throw std::runtime_error(validation.message);
            }
            if (elapsed_ >= trajectorySeconds()) {
              throw std::runtime_error("Playback finished. Plan from current pose or reset.");
            }
            if (!interpolate(elapsed_).isApprox(current_, 1e-6)) {
              throw std::runtime_error("Current pose differs from playback state; plan again");
            }
            playing_ = true;
            last_tick_ = std::chrono::steady_clock::now();
            response->success = true;
            response->message = "Playing validated trajectory in RViz";
          } catch (const std::exception &error) {
            response->success = false;
            response->message = error.what();
            playing_ = false;
          }
          status_ = response->message;
          publishState();
        });
    stop_service_ = create_service<std_srvs::srv::Trigger>(
        "planner/stop", [this](std_srvs::srv::Trigger::Request::ConstSharedPtr,
                               std_srvs::srv::Trigger::Response::SharedPtr response) {
          playing_ = false;
          response->success = true;
          response->message = "Playback paused";
          status_ = response->message;
        });
    reset_service_ = create_service<std_srvs::srv::Trigger>(
        "planner/reset", [this](std_srvs::srv::Trigger::Request::ConstSharedPtr,
                                std_srvs::srv::Trigger::Response::SharedPtr response) {
          std::string reason;
          if (!planner_->validState(home_, backend_, reason)) {
            response->success = false;
            response->message = reason;
            return;
          }
          invalidatePath();
          current_ = home_;
          target_ = poseMessage(planner_->robot().pose(home_));
          publishTarget();
          response->success = true;
          response->message = "Reset to demo start (visualization only)";
          status_ = response->message;
        });
    status_ = "Ready. Set a pose or drag the target, then Plan.";
    last_tick_ = std::chrono::steady_clock::now();
    timer_ = create_wall_timer(std::chrono::milliseconds(50), [this]() {
      const auto now = std::chrono::steady_clock::now();
      if (playing_) {
        elapsed_ = std::min(trajectorySeconds(),
                            elapsed_ + std::chrono::duration<double>(now - last_tick_).count());
        current_ = interpolate(elapsed_);
        if (elapsed_ >= trajectorySeconds()) {
          playing_ = false;
          status_ = "Playback finished";
        }
      }
      last_tick_ = now;
      publishState();
    });
  }

private:
  static geometry_msgs::msg::Pose poseMessage(const roscon_kinematics::Target &pose) {
    geometry_msgs::msg::Pose result;
    result.position.x = pose.position.x();
    result.position.y = pose.position.y();
    result.position.z = pose.position.z();
    const Eigen::Quaterniond q(pose.rotation);
    result.orientation.x = q.x();
    result.orientation.y = q.y();
    result.orientation.z = q.z();
    result.orientation.w = q.w();
    return result;
  }
  void setTarget(const geometry_msgs::msg::PoseStamped &pose) {
    if (pose.header.frame_id != base_) {
      throw std::runtime_error("Target frame must be " + base_);
    }
    const auto &p = pose.pose;
    const Eigen::Vector3d position(p.position.x, p.position.y, p.position.z);
    Eigen::Quaterniond q(p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z);
    if (!position.allFinite() || !q.coeffs().allFinite() || q.norm() < 1e-9 ||
        !std::isfinite(q.norm())) {
      throw std::runtime_error("Invalid target pose");
    }
    q.normalize();
    target_ = p;
    target_.orientation.x = q.x();
    target_.orientation.y = q.y();
    target_.orientation.z = q.z();
    target_.orientation.w = q.w();
    publishTarget();
  }
  void invalidatePath() {
    playing_ = false;
    valid_ = false;
    elapsed_ = 0.0;
    path_.clear();
    trajectory_ = trajectory_msgs::msg::JointTrajectory();
    publishPath();
  }
  void publishTarget() {
    geometry_msgs::msg::PoseStamped message;
    message.header.frame_id = base_;
    message.header.stamp = now();
    message.pose = target_;
    target_pub_->publish(message);
    server_->setPose("target", target_);
    server_->applyChanges();
  }
  void createMarker() {
    visualization_msgs::msg::InteractiveMarker marker;
    marker.header.frame_id = base_;
    marker.name = "target";
    marker.pose = target_;
    marker.scale = 0.22;
    visualization_msgs::msg::InteractiveMarkerControl center;
    center.always_visible = true;
    visualization_msgs::msg::Marker sphere;
    sphere.type = sphere.SPHERE;
    sphere.pose.orientation.w = 1.0;
    sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.04;
    sphere.color.r = 1.0;
    sphere.color.g = 0.6;
    sphere.color.a = 1.0;
    center.markers.push_back(sphere);
    marker.controls.push_back(center);
    const std::vector<Eigen::Quaterniond> axes = {
        Eigen::Quaterniond::Identity(),
        Eigen::Quaterniond(Eigen::AngleAxisd(1.5707963267948966, Eigen::Vector3d::UnitZ())),
        Eigen::Quaterniond(Eigen::AngleAxisd(-1.5707963267948966, Eigen::Vector3d::UnitY()))};
    for (size_t axis = 0; axis < axes.size(); ++axis) {
      visualization_msgs::msg::InteractiveMarkerControl control;
      control.orientation.x = axes[axis].x();
      control.orientation.y = axes[axis].y();
      control.orientation.z = axes[axis].z();
      control.orientation.w = axes[axis].w();
      control.orientation_mode = control.FIXED;
      control.name = "move_" + std::to_string(axis);
      control.interaction_mode = control.MOVE_AXIS;
      marker.controls.push_back(control);
      control.name = "rotate_" + std::to_string(axis);
      control.interaction_mode = control.ROTATE_AXIS;
      marker.controls.push_back(control);
    }
    server_->insert(
        marker,
        [this](visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr feedback) {
          if (feedback->event_type != feedback->MOUSE_UP) {
            return;
          }
          try {
            geometry_msgs::msg::PoseStamped pose;
            pose.header.frame_id = base_;
            pose.pose = feedback->pose;
            invalidatePath();
            setTarget(pose);
            status_ = "Target updated. Press Plan.";
          } catch (const std::exception &error) {
            status_ = error.what();
            publishTarget();
          }
        });
    server_->applyChanges();
  }
  trajectory_msgs::msg::JointTrajectory toTrajectory(const Path &path) {
    trajectory_msgs::msg::JointTrajectory result;
    result.header.frame_id = base_;
    result.header.stamp = now();
    result.joint_names = planner_->robot().names;
    double seconds = 0.0;
    for (size_t i = 0; i < path.size(); ++i) {
      if (i > 0) {
        seconds += std::max(0.02, (path[i] - path[i - 1]).cwiseAbs().maxCoeff() / 0.4);
      }
      trajectory_msgs::msg::JointTrajectoryPoint point;
      point.positions.assign(path[i].data(), path[i].data() + path[i].size());
      point.time_from_start = rclcpp::Duration::from_seconds(seconds);
      result.points.push_back(point);
    }
    return result;
  }
  Path fromTrajectory(const trajectory_msgs::msg::JointTrajectory &trajectory) {
    if (trajectory.joint_names != planner_->robot().names) {
      throw std::runtime_error("Trajectory joint names/order must match the robot");
    }
    Path result;
    for (const auto &point : trajectory.points) {
      result.emplace_back(
          Eigen::Map<const Eigen::VectorXd>(point.positions.data(), point.positions.size()));
    }
    return result;
  }
  double trajectorySeconds() const {
    return rclcpp::Duration(trajectory_.points.back().time_from_start).seconds();
  }
  Eigen::VectorXd interpolate(double time) const {
    for (size_t i = 1; i < path_.size(); ++i) {
      const double first = rclcpp::Duration(trajectory_.points[i - 1].time_from_start).seconds();
      const double last = rclcpp::Duration(trajectory_.points[i].time_from_start).seconds();
      if (time <= last) {
        const double fraction = std::clamp((time - first) / (last - first), 0.0, 1.0);
        return (1.0 - fraction) * path_[i - 1] + fraction * path_[i];
      }
    }
    return path_.back();
  }
  void publishPath() {
    trajectory_pub_->publish(trajectory_);
    visualization_msgs::msg::MarkerArray markers;
    visualization_msgs::msg::Marker line;
    line.header.frame_id = base_;
    line.header.stamp = now();
    line.ns = "planned_tool_path";
    line.id = 0;
    line.action = path_.empty() ? line.DELETE : line.ADD;
    line.type = line.LINE_STRIP;
    line.pose.orientation.w = 1.0;
    line.scale.x = 0.018;
    line.color.r = valid_ ? 0.1 : 1.0;
    line.color.g = valid_ ? 1.0 : 0.2;
    line.color.b = 0.2;
    line.color.a = 1.0;
    for (const auto &q : path_) {
      line.points.push_back(poseMessage(planner_->robot().pose(q)).position);
    }
    markers.markers.push_back(line);
    markers_pub_->publish(markers);
  }
  void publishState() {
    sensor_msgs::msg::JointState joints;
    joints.header.stamp = now();
    joints.name = planner_->robot().names;
    joints.position.assign(current_.data(), current_.data() + current_.size());
    joint_pub_->publish(joints);
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = base_;
    pose.header.stamp = now();
    pose.pose = poseMessage(planner_->robot().pose(current_));
    pose_pub_->publish(pose);
    std_msgs::msg::String status;
    status.data = status_ + "\nBackend: " + backend_ +
                  "; stored path: " + (valid_ ? "validated" : "not validated");
    status_pub_->publish(status);
    std_msgs::msg::String backend;
    backend.data = backend_;
    backend_pub_->publish(backend);
  }
  std::unique_ptr<Planner> planner_;
  std::unique_ptr<interactive_markers::InteractiveMarkerServer> server_;
  std::string base_, backend_, status_;
  Eigen::VectorXd home_, current_;
  geometry_msgs::msg::Pose target_;
  Path path_;
  trajectory_msgs::msg::JointTrajectory trajectory_;
  bool valid_ = false, playing_ = false;
  double elapsed_ = 0.0;
  std::chrono::steady_clock::time_point last_tick_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_, target_pub_, demo_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_, backend_pub_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_pub_, environment_pub_;
  rclcpp::Service<roscon_msgs::srv::PlanMotion>::SharedPtr plan_service_;
  rclcpp::Service<roscon_msgs::srv::SetCollisionBackend>::SharedPtr backend_service_;
  rclcpp::Service<roscon_msgs::srv::ValidateTrajectory>::SharedPtr validate_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr play_service_, stop_service_, reset_service_;
  rclcpp::TimerBase::SharedPtr timer_;
};
} // namespace roscon_planner
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  int exit_code = 0;
  try {
    rclcpp::spin(std::make_shared<roscon_planner::PlannerNode>());
  } catch (const std::exception &error) {
    RCLCPP_FATAL(rclcpp::get_logger("roscon_planner"), "%s", error.what());
    exit_code = 1;
  }
  rclcpp::shutdown();
  return exit_code;
}
