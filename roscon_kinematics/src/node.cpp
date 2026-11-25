#include "roscon_collision/scene_checker.hpp"
#include "roscon_kinematics/multi_seed.hpp"
#include "roscon_kinematics/solver.hpp"
#include <Eigen/SVD>
#include <algorithm>
#include <fstream>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <interactive_markers/interactive_marker_server.hpp>
#include <iomanip>
#include <rclcpp/rclcpp.hpp>
#include <roscon_msgs/msg/ik_selection.hpp>
#include <roscon_msgs/msg/ik_solutions.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sstream>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace roscon_kinematics {
class IkNode : public rclcpp::Node {
public:
  IkNode() : Node("roscon_kinematics") {
    const auto file = declare_parameter<std::string>("urdf_file", "");
    base_ = declare_parameter<std::string>("base_link", "base_link");
    const auto tip = declare_parameter<std::string>("tip_link", "flange");
    settings_.position_scale = declare_parameter("position_scale", settings_.position_scale);
    settings_.position_weight = declare_parameter("position_weight", settings_.position_weight);
    settings_.orientation_weight =
        declare_parameter("orientation_weight", settings_.orientation_weight);
    settings_.posture_weight = declare_parameter("posture_weight", settings_.posture_weight);
    settings_.characteristic_length =
        declare_parameter("characteristic_length", settings_.characteristic_length);
    settings_.minimum_singular_value =
        declare_parameter("minimum_singular_value", settings_.minimum_singular_value);
    settings_.joint_margin = declare_parameter("joint_margin", settings_.joint_margin);
    settings_.position_tolerance =
        declare_parameter("position_tolerance", settings_.position_tolerance);
    settings_.orientation_tolerance =
        declare_parameter("orientation_tolerance", settings_.orientation_tolerance);
    settings_.difference_step = declare_parameter("difference_step", settings_.difference_step);
    settings_.max_iterations = declare_parameter("max_iterations", settings_.max_iterations);
    std::ifstream input(file);
    if (!input) {
      throw std::runtime_error("Cannot read urdf_file: " + file);
    }
    const std::string xml((std::istreambuf_iterator<char>(input)),
                          std::istreambuf_iterator<char>());
    robot_ = std::make_unique<Robot>(xml, base_, tip, settings_.joint_margin);
    collision_checker_ = std::make_unique<roscon_collision::SceneChecker>(
        xml, declare_parameter<std::string>("environment_file", ""),
        declare_parameter<double>("collision_sphere_radius", 0.06));
    if (collision_checker_->frame() != base_) {
      throw std::runtime_error("Collision environment frame must match the URDF root and IK base");
    }
    auto initial =
        declare_parameter<std::vector<double>>("initial_joints", {0.0, -1.0, 1.2, -1.0, 1.0, 0.0});
    joints_ = Eigen::Map<Eigen::VectorXd>(initial.data(), initial.size());
    if (joints_.size() != robot_->lower.size()) {
      throw std::runtime_error("initial_joints size does not match chain");
    }
    target_ = robot_->pose(joints_);
    // Startup goes through exactly the same validation as subsequent targets.
    const auto initial_result = solve(*robot_, target_, joints_, settings_);
    if (!initial_result.accepted) {
      throw std::runtime_error("Initial configuration cannot satisfy solver constraints; change "
                               "initial_joints or singularity margin");
    }
    joints_ = initial_result.joints;
    initial_joints_ = joints_;
    joint_pub_ = create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("ik_markers", 10);
    environment_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
        "ik_environment", rclcpp::QoS(1).transient_local());
    environment_pub_->publish(collision_checker_->environmentMarkers());
    status_pub_ = create_publisher<std_msgs::msg::String>("ik_status", 10);
    pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("ik_current_pose", 10);
    target_pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("ik_target_pose", 10);
    solutions_pub_ = create_publisher<roscon_msgs::msg::IKSolutions>(
        "ik_solutions", rclcpp::QoS(1).transient_local());
    search_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        "ik_find_solutions", 10, [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr message) {
          if (message->header.frame_id != base_) {
            status_ = "REJECTED: target frame must be " + base_;
            publish();
            return;
          }
          if (!set_target(message->pose)) {
            return;
          }
          seeds_ = predefined_seeds(*robot_, joints_);
          next_seed_ = 0;
          search_active_ = true;
          solutions_.seeds_total = seeds_.size();
          solutions_.complete = false;
          status_ = "SEARCHING: 0 / " + std::to_string(seeds_.size()) + " seeds";
          solutions_pub_->publish(solutions_);
          publish();
        });
    selection_sub_ = create_subscription<roscon_msgs::msg::IKSelection>(
        "ik_select_solution", 10, [this](roscon_msgs::msg::IKSelection::ConstSharedPtr selection) {
          if (selection->generation != solutions_.generation ||
              selection->index >= solutions_.solutions.size()) {
            status_ = "REJECTED: solution list has changed; search again";
            publish();
            return;
          }
          const auto &candidate = solutions_.solutions[selection->index];
          const auto collision =
              collision_checker_->check(candidate.joints.name, candidate.joints.position);
          if (!collision.collision_free()) {
            status_ = "REJECTED: selected solution is in collision";
            publish();
            return;
          }
          joints_ = Eigen::Map<const Eigen::VectorXd>(candidate.joints.position.data(),
                                                      candidate.joints.position.size());
          target_reached_ = true;
          status_ = "PREVIEW: solution " + std::to_string(selection->index + 1) + " from seed " +
                    std::to_string(candidate.seed_index);
          // The requested pose and marker stay unchanged while the joint branch changes.
          publish();
        });
    joint_sub_ = create_subscription<sensor_msgs::msg::JointState>(
        "ik_joint_command", 10, [this](sensor_msgs::msg::JointState::ConstSharedPtr command) {
          if (command->name.size() != robot_->names.size() ||
              command->position.size() != robot_->names.size()) {
            status_ = "REJECTED: supply every joint exactly once";
            publish();
            return;
          }
          Eigen::VectorXd requested(joints_.size());
          for (size_t i = 0; i < robot_->names.size(); ++i) {
            if (std::count(command->name.begin(), command->name.end(), robot_->names[i]) != 1) {
              status_ = "REJECTED: missing, duplicate, or unknown joint name";
              publish();
              return;
            }
            const auto found =
                std::find(command->name.begin(), command->name.end(), robot_->names[i]);
            requested[i] = command->position[std::distance(command->name.begin(), found)];
          }
          apply_joints(requested);
        });
    reset_service_ = create_service<std_srvs::srv::Trigger>(
        "reset_ik", [this](std_srvs::srv::Trigger::Request::ConstSharedPtr,
                           std_srvs::srv::Trigger::Response::SharedPtr response) {
          response->success = apply_joints(initial_joints_);
          response->message =
              response->success
                  ? "Initial configuration restored. Use Copy current to update target fields."
                  : status_;
        });
    target_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        "target_pose", 10, [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr message) {
          if (message->header.frame_id != base_) {
            status_ = "REJECTED: target frame must be " + base_;
            publish();
            return;
          }
          update_target(message->pose);
        });
    server_ = std::make_unique<interactive_markers::InteractiveMarkerServer>("ik_target", this);
    create_target_marker();
    status_ = "Ready: use the Kinematics panel";
    clear_solutions();
    timer_ = create_wall_timer(std::chrono::milliseconds(50), [this]() {
      if (search_active_) {
        search_next_seed();
      }
      publish();
    });
    RCLCPP_INFO(get_logger(), "IK ready. Use the Kinematics panel; target frame is %s",
                base_.c_str());
  }

private:
  geometry_msgs::msg::Pose pose_message(const Target &pose) const {
    geometry_msgs::msg::Pose message;
    message.position.x = pose.position.x();
    message.position.y = pose.position.y();
    message.position.z = pose.position.z();
    const Eigen::Quaterniond orientation(pose.rotation);
    message.orientation.x = orientation.x();
    message.orientation.y = orientation.y();
    message.orientation.z = orientation.z();
    message.orientation.w = orientation.w();
    return message;
  }
  bool apply_joints(const Eigen::VectorXd &requested) {
    if (!requested.allFinite() || requested.size() != joints_.size() ||
        (requested.array() < robot_->lower.array()).any() ||
        (requested.array() > robot_->upper.array()).any()) {
      status_ = "REJECTED: joint target violates limits or is nonfinite";
      publish();
      return false;
    }
    const double sigma = robot_->jacobian(requested, settings_.characteristic_length)
                             .jacobiSvd()
                             .singularValues()
                             .minCoeff();
    if (!std::isfinite(sigma) || sigma < settings_.minimum_singular_value) {
      status_ = "REJECTED: joint target violates singularity margin";
      publish();
      return false;
    }
    joints_ = requested;
    target_ = robot_->pose(joints_);
    clear_solutions();
    target_reached_ = true;
    server_->setPose("target", pose_message(target_));
    server_->applyChanges();
    status_ = "JOINT TARGET ACCEPTED";
    publish();
    return true;
  }
  void create_target_marker() {
    visualization_msgs::msg::InteractiveMarker marker;
    marker.header.frame_id = base_;
    marker.name = "target";
    marker.description = ""; // No text is drawn in the 3D scene.
    marker.scale = 0.3;
    marker.pose = pose_message(target_);
    visualization_msgs::msg::InteractiveMarkerControl centre;
    centre.always_visible = true;
    visualization_msgs::msg::Marker sphere;
    sphere.type = sphere.SPHERE;
    sphere.pose.orientation.w = 1;
    sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.04;
    sphere.color.r = 1;
    sphere.color.g = 0.5;
    sphere.color.a = 1;
    centre.markers.push_back(sphere);
    marker.controls.push_back(centre);
    const double half_pi = 1.5707963267948966;
    const std::vector<Eigen::Quaterniond> orientations = {
        Eigen::Quaterniond::Identity(),
        Eigen::Quaterniond(Eigen::AngleAxisd(half_pi, Eigen::Vector3d::UnitZ())),
        Eigen::Quaterniond(Eigen::AngleAxisd(-half_pi, Eigen::Vector3d::UnitY()))};
    for (size_t axis = 0; axis < orientations.size(); ++axis) {
      const Eigen::Quaterniond rotation = orientations[axis];
      visualization_msgs::msg::InteractiveMarkerControl control;
      control.orientation.x = rotation.x();
      control.orientation.y = rotation.y();
      control.orientation.z = rotation.z();
      control.orientation.w = rotation.w();
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
          if (feedback->event_type == feedback->MOUSE_UP) {
            update_target(feedback->pose);
          }
        });
    server_->applyChanges();
  }
  bool set_target(const geometry_msgs::msg::Pose &pose) {
    Eigen::Quaterniond rotation(pose.orientation.w, pose.orientation.x, pose.orientation.y,
                                pose.orientation.z);
    const Eigen::Vector3d position(pose.position.x, pose.position.y, pose.position.z);
    if (!position.allFinite() || !rotation.coeffs().allFinite() || rotation.norm() < 1e-9) {
      status_ = "REJECTED: nonfinite target or zero quaternion";
      publish();
      return false;
    }
    target_ = {position, rotation.normalized().toRotationMatrix()};
    clear_solutions();
    server_->setPose("target", pose_message(target_));
    server_->applyChanges();
    return true;
  }
  void update_target(const geometry_msgs::msg::Pose &pose) {
    if (!set_target(pose)) {
      return;
    }
    status_ = "SOLVING";
    publish();
    try {
      const Result result = solve(*robot_, target_, joints_, settings_);
      if (result.accepted) {
        joints_ = result.joints;
      }
      target_reached_ = result.target_reached;
      std::ostringstream report;
      report << (result.accepted
                     ? (result.target_reached ? "TARGET REACHED" : "FEASIBLE APPROXIMATION")
                     : "REJECTED: retaining previous robot pose")
             << "\nIPOPT status: " << result.solver_status;
      status_ = report.str();
      RCLCPP_INFO(get_logger(), "%s", status_.c_str());
    } catch (const std::exception &error) {
      target_reached_ = false;
      status_ = std::string("ERROR: ") + error.what();
      RCLCPP_ERROR(get_logger(), "%s", status_.c_str());
    }
    publish();
  }
  void clear_solutions() {
    search_active_ = false;
    const uint64_t generation = solutions_.generation + 1;
    solutions_ = roscon_msgs::msg::IKSolutions();
    solutions_.generation = generation;
    solutions_.header.frame_id = base_;
    solutions_.header.stamp = now();
    solutions_.target = pose_message(target_);
    solutions_.complete = true;
    solutions_pub_->publish(solutions_);
  }
  void search_next_seed() {
    // One seed per timer callback keeps ROS callbacks and progress updates alive
    // between optimizer runs. A new target, joint command, or reset cancels this search.
    try {
      const Result result = solve(*robot_, target_, seeds_.at(next_seed_), settings_);
      bool duplicate = false;
      for (const auto &existing : solutions_.solutions) {
        const Eigen::Map<const Eigen::VectorXd> q(existing.joints.position.data(),
                                                  existing.joints.position.size());
        if (same_configuration(result.joints, q)) {
          duplicate = true;
          break;
        }
      }
      if (result.accepted && result.target_reached && !duplicate) {
        roscon_msgs::msg::IKSolution candidate;
        candidate.joints.name = robot_->names;
        candidate.joints.position.assign(result.joints.data(),
                                         result.joints.data() + result.joints.size());
        candidate.seed_index = next_seed_;
        candidate.position_error = result.position_error;
        candidate.orientation_error = result.orientation_error;
        candidate.minimum_singular_value = result.minimum_singular_value;
        const auto collision =
            collision_checker_->check(candidate.joints.name, candidate.joints.position);
        if (collision.self_collision) {
          ++solutions_.self_collision_rejections;
        }
        if (collision.environment_collision) {
          ++solutions_.environment_collision_rejections;
        }
        if (collision.collision_free()) {
          solutions_.solutions.push_back(candidate);
        } else {
          solutions_.last_rejection.clear();
          for (const auto &pair : collision.pairs) {
            if (!solutions_.last_rejection.empty()) {
              solutions_.last_rejection += "; ";
            }
            solutions_.last_rejection += pair;
          }
        }
      }
    } catch (const std::exception &error) {
      ++solutions_.check_errors;
      solutions_.last_rejection = std::string("Seed check failed: ") + error.what();
      RCLCPP_WARN(get_logger(), "Seed %zu failed: %s", next_seed_, error.what());
    }
    ++next_seed_;
    solutions_.seeds_tried = next_seed_;
    search_active_ = next_seed_ < seeds_.size();
    solutions_.complete = !search_active_;
    status_ = std::string(search_active_ ? "SEARCHING: " : "SEARCH COMPLETE: ") +
              std::to_string(next_seed_) + " / " + std::to_string(seeds_.size()) + " seeds; " +
              std::to_string(solutions_.solutions.size()) + " collision-free solutions; " +
              std::to_string(solutions_.self_collision_rejections) + " self / " +
              std::to_string(solutions_.environment_collision_rejections) +
              " environment rejections";
    solutions_.header.stamp = now();
    solutions_pub_->publish(solutions_);
  }
  void publish() {
    sensor_msgs::msg::JointState state;
    state.header.stamp = now();
    state.name = robot_->names;
    state.position.assign(joints_.data(), joints_.data() + joints_.size());
    joint_pub_->publish(state);
    const Target actual = robot_->pose(joints_);
    geometry_msgs::msg::PoseStamped current_pose;
    current_pose.header.frame_id = base_;
    current_pose.header.stamp = now();
    current_pose.pose = pose_message(actual);
    pose_pub_->publish(current_pose);
    auto requested_pose = current_pose;
    requested_pose.pose = pose_message(target_);
    target_pose_pub_->publish(requested_pose);
    const double sigma = robot_->jacobian(joints_, settings_.characteristic_length)
                             .jacobiSvd()
                             .singularValues()
                             .minCoeff();
    std::ostringstream report;
    report << status_ << std::fixed << std::setprecision(4)
           << "\nDisplayed pose error: " << (actual.position - target_.position).norm() << " m"
           << "\nOrientation error: "
           << Eigen::AngleAxisd(target_.rotation.transpose() * actual.rotation).angle() * 180.0 /
                  3.141592653589793
           << " deg"
           << "\nMinimum singular value: " << sigma << " (limit "
           << settings_.minimum_singular_value << ")";
    std_msgs::msg::String status;
    status.data = report.str();
    status_pub_->publish(status);
    visualization_msgs::msg::MarkerArray markers;
    visualization_msgs::msg::Marker line;
    line.header.frame_id = base_;
    line.header.stamp = now();
    line.ns = "ik";
    line.id = 0;
    line.type = line.LINE_LIST;
    line.action = line.ADD;
    line.pose.orientation.w = 1;
    line.scale.x = 0.006;
    line.color.r = 1;
    line.color.g = 0.5;
    line.color.a = 1;
    line.points = {pose_message(target_).position, pose_message(actual).position};
    markers.markers.push_back(line);
    for (int axis = 0; axis < 3; ++axis) {
      auto arrow = line;
      arrow.id = 1 + axis;
      arrow.type = arrow.ARROW;
      arrow.scale.x = 0.008;
      arrow.scale.y = 0.016;
      arrow.scale.z = 0.025;
      arrow.color.r = axis == 0 ? 1 : 0;
      arrow.color.g = axis == 1 ? 1 : 0;
      arrow.color.b = axis == 2 ? 1 : 0;
      Target endpoint = actual;
      endpoint.position += 0.15 * actual.rotation.col(axis);
      arrow.points = {pose_message(actual).position, pose_message(endpoint).position};
      markers.markers.push_back(arrow);
    }
    marker_pub_->publish(markers);
  }
  bool search_active_ = false;
  size_t next_seed_ = 0;
  std::vector<Eigen::VectorXd> seeds_;
  roscon_msgs::msg::IKSolutions solutions_;
  rclcpp::Publisher<roscon_msgs::msg::IKSolutions>::SharedPtr solutions_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr search_sub_;
  rclcpp::Subscription<roscon_msgs::msg::IKSelection>::SharedPtr selection_sub_;
  Settings settings_;
  std::unique_ptr<roscon_collision::SceneChecker> collision_checker_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr environment_pub_;
  std::unique_ptr<Robot> robot_;
  Eigen::VectorXd joints_;
  Target target_;
  Eigen::VectorXd initial_joints_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_, target_pose_pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_service_;
  std::string base_, status_;
  bool target_reached_ = true;
  std::unique_ptr<interactive_markers::InteractiveMarkerServer> server_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};
} // namespace roscon_kinematics
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<roscon_kinematics::IkNode>());
  } catch (const std::exception &error) {
    RCLCPP_FATAL(rclcpp::get_logger("roscon_kinematics"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
