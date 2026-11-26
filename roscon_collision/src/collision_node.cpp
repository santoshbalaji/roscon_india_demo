#include "roscon_collision/geometry.hpp"
#include "roscon_collision/gpu_spheres.hpp"
#include "roscon_collision/sdf_model.hpp"
#include "roscon_collision/sdf_visualization.hpp"
#include "roscon_collision/sphere_model.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <map>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <set>
#include <sstream>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <stdexcept>
#include <string>
#include <urdf/model.h>
#include <utility>
#include <vector>
#include <visualization_msgs/msg/marker_array.hpp>
#include <yaml-cpp/yaml.h>

namespace roscon_collision {
struct Body {
  std::string name;
  std::string link_name;
  Shape shape;
  fcl::Transform3d local_transform = fcl::Transform3d::Identity();
  std::shared_ptr<fcl::CollisionObjectd> object;
  bool self_collision = false;
  bool environment_collision = false;
};

class CollisionNode : public rclcpp::Node {
public:
  CollisionNode() : Node("roscon_collision") {
    const std::string robot_description = declare_parameter<std::string>("robot_description", "");
    const std::string environment_file = declare_parameter<std::string>("environment_file", "");
    const double update_rate = declare_parameter<double>("update_rate", 20.0);
    if (!std::isfinite(update_rate) || update_rate <= 0.0 || update_rate > 1000.0) {
      throw std::runtime_error("update_rate must be in (0, 1000] Hz");
    }
    if (!robot_.initString(robot_description)) {
      throw std::runtime_error("robot_description must contain a valid expanded URDF");
    }
    fixed_frame_ = robot_.getRoot()->name;
    loadRobot();
    loadEnvironment(resolveAssetPath(environment_file));
    backend_ = declare_parameter<std::string>("collision_backend", "fcl");
    if (backend_ != "fcl" && backend_ != "gpu_spheres" && backend_ != "gpu_sdf") {
      throw std::runtime_error("collision_backend must be fcl, gpu_spheres, or gpu_sdf");
    }
    sdf_cell_size_ = declare_parameter<double>("sdf_cell_size", 0.08);
    sdf_padding_ = declare_parameter<double>("sdf_padding", 0.25);
    buildSphereModel(declare_parameter<double>("sphere_radius", 0.10));
    if (backend_ == "gpu_spheres") {
      prepareGpu();
    } else if (backend_ == "gpu_sdf") {
      prepareSdf();
    }

    marker_publisher_ = create_publisher<visualization_msgs::msg::MarkerArray>(
        "collision_markers", rclcpp::QoS(1).transient_local());
    status_publisher_ = create_publisher<std_msgs::msg::String>("collision_status",
                                                                rclcpp::QoS(1).transient_local());
    sphere_marker_publisher_ = create_publisher<visualization_msgs::msg::MarkerArray>(
        "sphere_markers", rclcpp::QoS(1).transient_local());
    sdf_marker_publisher_ = create_publisher<visualization_msgs::msg::MarkerArray>(
        "sdf_markers", rclcpp::QoS(1).transient_local());
    backend_publisher_ = create_publisher<std_msgs::msg::String>("collision_backend",
                                                                 rclcpp::QoS(1).transient_local());
    backend_service_ = create_service<std_srvs::srv::SetBool>(
        "~/use_gpu_spheres", [this](const std_srvs::srv::SetBool::Request::SharedPtr request,
                                    std_srvs::srv::SetBool::Response::SharedPtr response) {
          try {
            if (request->data) {
              prepareGpu();
              backend_ = "gpu_spheres";
            } else {
              backend_ = "fcl";
            }
            response->success = true;
            response->message = "Active backend: " + backend_;
          } catch (const std::exception &exception) {
            response->success = false;
            response->message = exception.what();
          }
          publishBackend();
        });
    sdf_backend_service_ = create_service<std_srvs::srv::SetBool>(
        "~/use_sdf", [this](const std_srvs::srv::SetBool::Request::SharedPtr request,
                            std_srvs::srv::SetBool::Response::SharedPtr response) {
          try {
            if (request->data) {
              prepareSdf();
              backend_ = "gpu_sdf";
            } else {
              backend_ = "fcl";
            }
            response->success = true;
            response->message = "Active backend: " + backend_;
          } catch (const std::exception &exception) {
            response->success = false;
            response->message = exception.what();
          }
          publishBackend();
        });
    sdf_validation_service_ = create_service<std_srvs::srv::Trigger>(
        "~/validate_sdf",
        [this](const std_srvs::srv::Trigger::Request::SharedPtr,
               std_srvs::srv::Trigger::Response::SharedPtr response) { validateSdf(*response); });
    validation_service_ = create_service<std_srvs::srv::Trigger>(
        "~/validate_spheres", [this](const std_srvs::srv::Trigger::Request::SharedPtr,
                                     std_srvs::srv::Trigger::Response::SharedPtr response) {
          validateSpheres(*response);
        });
    publishBackend();
    joint_subscription_ = create_subscription<sensor_msgs::msg::JointState>(
        "joint_states", rclcpp::SensorDataQoS(),
        std::bind(&CollisionNode::receiveJointState, this, std::placeholders::_1));
    timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / update_rate),
                               std::bind(&CollisionNode::update, this));
    RCLCPP_INFO(get_logger(), "Loaded %zu robot collision shapes and %zu environment objects",
                robot_bodies_.size(), environment_bodies_.size());
  }

private:
  // Adjacent links share a mechanical joint and are excluded from self checks.
  // Additional exclusions must be explicitly listed in the environment file.
  void loadRobot() {
    for (const auto &joint_entry : robot_.joints_) {
      const urdf::Joint &joint = *joint_entry.second;
      if (joint.mimic ||
          (joint.type != urdf::Joint::FIXED && joint.type != urdf::Joint::REVOLUTE &&
           joint.type != urdf::Joint::CONTINUOUS && joint.type != urdf::Joint::PRISMATIC)) {
        throw std::runtime_error("Unsupported or mimic joint: " + joint.name);
      }
      if (joint.type != urdf::Joint::FIXED) {
        const Eigen::Vector3d axis(joint.axis.x, joint.axis.y, joint.axis.z);
        if (!axis.allFinite() || axis.norm() < 1e-12) {
          throw std::runtime_error("Invalid joint axis: " + joint.name);
        }
        joint_positions_[joint.name] = 0.0;
      }
      ignored_pairs_.insert(orderedPair(joint.parent_link_name, joint.child_link_name));
    }
    for (const auto &link_entry : robot_.links_) {
      const urdf::Link &link = *link_entry.second;
      for (std::size_t index = 0; index < link.collision_array.size(); ++index) {
        const urdf::Collision &collision = *link.collision_array[index];
        if (!collision.geometry) {
          throw std::runtime_error("Missing collision geometry on " + link.name);
        }
        Body body;
        body.name = link.name + "/" + std::to_string(index);
        body.link_name = link.name;
        body.shape = loadShape(*collision.geometry);
        body.local_transform = poseToTransform(collision.origin);
        body.object = std::make_shared<fcl::CollisionObjectd>(body.shape.geometry);
        robot_bodies_.push_back(body);
      }
    }
    if (robot_bodies_.empty()) {
      throw std::runtime_error("Robot has no collision geometry");
    }
  }

  static std::pair<std::string, std::string> orderedPair(const std::string &first,
                                                         const std::string &second) {
    if (first < second) {
      return {first, second};
    }
    return {second, first};
  }

  static urdf::Vector3 readVector(const YAML::Node &value, const std::string &field) {
    if (!value.IsSequence() || value.size() != 3) {
      throw std::runtime_error(field + " must contain three numbers");
    }
    urdf::Vector3 vector(value[0].as<double>(), value[1].as<double>(), value[2].as<double>());
    if (!std::isfinite(vector.x) || !std::isfinite(vector.y) || !std::isfinite(vector.z)) {
      throw std::runtime_error(field + " must contain finite numbers");
    }
    return vector;
  }

  void loadEnvironment(const std::string &path) {
    const YAML::Node configuration = YAML::LoadFile(path);
    const YAML::Node exclusions = configuration["disabled_self_collisions"];
    if (exclusions && !exclusions.IsSequence()) {
      throw std::runtime_error("disabled_self_collisions must be a list");
    }
    for (const YAML::Node &pair : exclusions) {
      if (!pair.IsSequence() || pair.size() != 2) {
        throw std::runtime_error("Each disabled self-collision pair needs two link names");
      }
      const std::string first = pair[0].as<std::string>();
      const std::string second = pair[1].as<std::string>();
      if (!robot_.getLink(first) || !robot_.getLink(second)) {
        throw std::runtime_error("Unknown link in disabled self-collision pair");
      }
      ignored_pairs_.insert(orderedPair(first, second));
    }
    const YAML::Node objects = configuration["objects"];
    if (!objects.IsSequence()) {
      throw std::runtime_error(
          "Environment YAML needs an objects list (use [] for an empty scene)");
    }
    std::set<std::string> names;
    for (const YAML::Node &entry : objects) {
      Body body;
      body.name = entry["name"].as<std::string>();
      if (body.name.empty() || !names.insert(body.name).second) {
        throw std::runtime_error("Environment object names must be nonempty and unique");
      }
      const std::string type = entry["type"].as<std::string>();
      if (type == "box") {
        urdf::Box box;
        box.dim = readVector(entry["size"], "size");
        body.shape = loadShape(box);
      } else if (type == "sphere") {
        urdf::Sphere sphere;
        sphere.radius = entry["radius"].as<double>();
        body.shape = loadShape(sphere);
      } else if (type == "cylinder") {
        urdf::Cylinder cylinder;
        cylinder.radius = entry["radius"].as<double>();
        cylinder.length = entry["length"].as<double>();
        body.shape = loadShape(cylinder);
      } else if (type == "mesh") {
        urdf::Mesh mesh;
        mesh.filename = entry["resource"].as<std::string>();
        mesh.scale = urdf::Vector3(1.0, 1.0, 1.0);
        if (entry["scale"]) {
          mesh.scale = readVector(entry["scale"], "scale");
        }
        body.shape = loadShape(mesh);
      } else {
        throw std::runtime_error("Unknown environment shape type: " + type);
      }
      urdf::Pose pose;
      pose.position = readVector(entry["position"], "position");
      if (entry["rpy"]) {
        const urdf::Vector3 angles = readVector(entry["rpy"], "rpy");
        pose.rotation.setFromRPY(angles.x, angles.y, angles.z);
      }
      body.local_transform = poseToTransform(pose);
      body.object =
          std::make_shared<fcl::CollisionObjectd>(body.shape.geometry, body.local_transform);
      environment_bodies_.push_back(body);
    }
  }

  void receiveJointState(const sensor_msgs::msg::JointState::SharedPtr message) {
    if (message->name.size() != message->position.size()) {
      RCLCPP_WARN(get_logger(), "Ignoring JointState with mismatched name and position arrays");
      return;
    }
    for (double position : message->position) {
      if (!std::isfinite(position)) {
        RCLCPP_WARN(get_logger(), "Ignoring JointState containing a non-finite position");
        return;
      }
    }
    for (std::size_t index = 0; index < message->name.size(); ++index) {
      auto joint = joint_positions_.find(message->name[index]);
      if (joint != joint_positions_.end()) {
        joint->second = message->position[index];
        received_joints_.insert(message->name[index]);
      }
    }
  }

  // URDF forward kinematics keeps every collision object in one coherent snapshot.
  void computeLinkTransforms(const urdf::LinkConstSharedPtr &link,
                             const fcl::Transform3d &parent_transform) {
    link_transforms_[link->name] = parent_transform;
    for (const urdf::JointSharedPtr &joint : link->child_joints) {
      fcl::Transform3d motion = fcl::Transform3d::Identity();
      if (joint->type != urdf::Joint::FIXED) {
        const double position = joint_positions_.at(joint->name);
        const Eigen::Vector3d axis(joint->axis.x, joint->axis.y, joint->axis.z);
        if (joint->type == urdf::Joint::PRISMATIC) {
          motion.translation() = axis.normalized() * position;
        } else {
          motion.linear() = Eigen::AngleAxisd(position, axis.normalized()).toRotationMatrix();
        }
      }
      const fcl::Transform3d child_transform =
          parent_transform * poseToTransform(joint->parent_to_joint_origin_transform) * motion;
      computeLinkTransforms(robot_.getLink(joint->child_link_name), child_transform);
    }
  }

  static bool collides(const Body &first, const Body &second) {
    fcl::CollisionRequestd request;
    fcl::CollisionResultd result;
    fcl::collide(first.object.get(), second.object.get(), request, result);
    return result.isCollision();
  }

  void update() {
    if (received_joints_.size() != joint_positions_.size()) {
      std_msgs::msg::String status;
      status.data = "Waiting for all movable joints on /joint_states";
      status_publisher_->publish(status);
      return;
    }
    computeLinkTransforms(robot_.getRoot(), fcl::Transform3d::Identity());
    for (Body &body : robot_bodies_) {
      body.self_collision = false;
      body.environment_collision = false;
      body.object->setTransform(link_transforms_.at(body.link_name) * body.local_transform);
      body.object->computeAABB();
    }
    for (Body &body : environment_bodies_) {
      body.environment_collision = false;
    }
    std::set<std::pair<int, int>> selected_pairs;
    try {
      if (backend_ == "gpu_spheres") {
        last_sphere_results_ = gpu_.check(currentTransforms());
        selected_pairs = sphereBodyPairs(last_sphere_results_, sphere_pairs_);
      } else if (backend_ == "gpu_sdf") {
        last_sphere_results_ = gpu_self_.check(currentTransforms());
        selected_pairs = sphereBodyPairs(last_sphere_results_, self_sphere_pairs_);
        const SdfResults sdf = gpu_sdf_.check(last_sphere_results_.world_spheres);
        for (std::size_t sphere = 0; sphere < robot_spheres_.size(); ++sphere) {
          for (std::size_t object = 0; object < environment_bodies_.size(); ++object) {
            if (sdf.collisions[sphere * environment_bodies_.size() + object]) {
              selected_pairs.emplace(robot_spheres_[sphere].body, robot_bodies_.size() + object);
            }
          }
        }
      } else {
        selected_pairs = checkFclPairs();
      }
    } catch (const std::exception &exception) {
      std_msgs::msg::String error;
      error.data = "Collision check unavailable: " + std::string(exception.what());
      status_publisher_->publish(error);
      clearMarkers();
      return;
    }
    const std::set<std::string> collision_pairs = describePairs(selected_pairs);
    applyCollisions(selected_pairs);
    std::ostringstream summary;
    if (collision_pairs.empty()) {
      summary << "Collision free";
    } else {
      for (const std::string &pair : collision_pairs) {
        summary << pair << '\n';
      }
    }
    std_msgs::msg::String status;
    status.data = summary.str();
    status_publisher_->publish(status);
    if (status.data != previous_status_) {
      RCLCPP_INFO(get_logger(), "%s", status.data.c_str());
      previous_status_ = status.data;
    }
    publishMarkers();
    publishSphereMarkers();
    publishSdfMarkers();
    publishBackend();
  }

  void appendMarkers(const std::vector<Body> &bodies, bool robot,
                     visualization_msgs::msg::MarkerArray &markers) {
    for (std::size_t index = 0; index < bodies.size(); ++index) {
      const Body &body = bodies[index];
      visualization_msgs::msg::Marker marker = body.shape.marker;
      marker.header.frame_id = fixed_frame_;
      marker.header.stamp = now();
      marker.ns = robot ? "robot_collision" : "environment";
      marker.id = static_cast<int>(index);
      marker.pose = transformToPose(body.object->getTransform());
      marker.color.a = 1.0;
      if (backend_ == "gpu_sdf" && !robot) {
        marker.color.a = 0.18; // Show the distance slice inside the environment surface.
      }
      if (body.self_collision && body.environment_collision) {
        marker.color.r = 1.0;
        marker.color.g = 0.9;
        marker.color.b = 0.0;
      } else if (body.self_collision) {
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 1.0;
      } else if (body.environment_collision) {
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
      } else if (robot) {
        marker.color.r = 1.0;
        marker.color.g = 0.45;
        marker.color.b = 0.08;
      } else {
        marker.color.r = 0.25;
        marker.color.g = 0.65;
        marker.color.b = 0.85;
      }
      markers.markers.push_back(marker);
    }
  }

  void publishMarkers() {
    visualization_msgs::msg::MarkerArray markers;
    visualization_msgs::msg::Marker clear;
    clear.action = visualization_msgs::msg::Marker::DELETEALL;
    markers.markers.push_back(clear);
    if (backend_ == "fcl") {
      appendMarkers(robot_bodies_, true, markers);
      appendMarkers(environment_bodies_, false, markers);
    } else if (backend_ == "gpu_sdf") {
      // The displayed surfaces are the zero level sets of the environment SDFs.
      appendMarkers(environment_bodies_, false, markers);
    }
    // Clear the floating text left by older versions of the demo.
    visualization_msgs::msg::Marker old_label;
    old_label.header.frame_id = fixed_frame_;
    old_label.ns = "status";
    old_label.id = 0;
    old_label.action = visualization_msgs::msg::Marker::DELETE;
    markers.markers.push_back(old_label);
    marker_publisher_->publish(markers);
  }

  Body &bodyAt(int index) {
    if (index < static_cast<int>(robot_bodies_.size())) {
      return robot_bodies_.at(index);
    }
    return environment_bodies_.at(index - robot_bodies_.size());
  }

  void buildSphereModel(double radius) {
    const int body_count = robot_bodies_.size() + environment_bodies_.size();
    std::vector<int> starts;
    for (int index = 0; index < body_count; ++index) {
      starts.push_back(local_spheres_.size());
      const std::vector<Sphere> spheres =
          makeSphereModel(*bodyAt(index).shape.geometry, radius, index);
      local_spheres_.insert(local_spheres_.end(), spheres.begin(), spheres.end());
      if (local_spheres_.size() > 10000) {
        throw std::runtime_error("Scene exceeds 10000 spheres; increase sphere_radius");
      }
    }
    starts.push_back(local_spheres_.size());
    for (int first = 0; first < static_cast<int>(robot_bodies_.size()); ++first) {
      for (int second = first + 1; second < body_count; ++second) {
        if (second < static_cast<int>(robot_bodies_.size()) &&
            (bodyAt(first).link_name == bodyAt(second).link_name ||
             ignored_pairs_.count(orderedPair(bodyAt(first).link_name, bodyAt(second).link_name)) >
                 0)) {
          continue;
        }
        allowed_body_pairs_.emplace_back(first, second);
        for (int first_sphere = starts[first]; first_sphere < starts[first + 1]; ++first_sphere) {
          for (int second_sphere = starts[second]; second_sphere < starts[second + 1];
               ++second_sphere) {
            if (sphere_pairs_.size() >= 5000000) {
              throw std::runtime_error(
                  "Scene exceeds 5000000 sphere pairs; increase sphere_radius");
            }
            sphere_pairs_.push_back({first_sphere, second_sphere});
          }
        }
      }
    }
  }

  void prepareGpu() {
    if (!gpu_ready_) {
      gpu_device_ = GpuSpheres::deviceName();
      gpu_.initialize(local_spheres_, sphere_pairs_,
                      robot_bodies_.size() + environment_bodies_.size());
      gpu_ready_ = true;
    }
  }

  void prepareSdf() {
    if (sdf_ready_) {
      return;
    }
    gpu_device_ = GpuSpheres::deviceName();
    robot_spheres_.clear();
    self_sphere_pairs_.clear();
    sdf_objects_.clear();
    sdf_triangles_.clear();
    sdf_slices_.markers.clear();
    for (const Sphere &sphere : local_spheres_) {
      if (sphere.body < static_cast<int>(robot_bodies_.size())) {
        robot_spheres_.push_back(sphere);
      }
    }
    // Robot spheres are the prefix of local_spheres_, so their original indices remain valid.
    for (const SpherePair &pair : sphere_pairs_) {
      if (pair.second < static_cast<int>(robot_spheres_.size())) {
        self_sphere_pairs_.push_back(pair);
      }
    }
    for (const Body &body : environment_bodies_) {
      try {
        sdf_objects_.push_back(
            makeSdfObject(*body.shape.geometry, body.local_transform, sdf_triangles_));
      } catch (const std::exception &exception) {
        throw std::runtime_error("Environment object " + body.name + ": " + exception.what());
      }
    }
    gpu_self_.initialize(robot_spheres_, self_sphere_pairs_,
                         robot_bodies_.size() + environment_bodies_.size());
    gpu_sdf_.initialize(sdf_objects_, sdf_triangles_, robot_spheres_.size());
    for (std::size_t index = 0; index < environment_bodies_.size(); ++index) {
      const Body &body = environment_bodies_[index];
      auto slice =
          makeSdfSlice(sdf_objects_[index], sdf_triangles_, body.shape.geometry->aabb_local,
                       body.local_transform, sdf_cell_size_, sdf_padding_);
      slice.id = index;
      slice.header.frame_id = fixed_frame_;
      sdf_slices_.markers.push_back(slice);
    }
    sdf_ready_ = true;
  }

  std::vector<SphereTransform> currentTransforms() {
    std::vector<SphereTransform> transforms;
    const int body_count = robot_bodies_.size() + environment_bodies_.size();
    for (int index = 0; index < body_count; ++index) {
      transforms.push_back(makeSphereTransform(bodyAt(index).object->getTransform()));
    }
    return transforms;
  }

  std::set<std::pair<int, int>> checkFclPairs() {
    std::set<std::pair<int, int>> pairs;
    for (const auto &pair : allowed_body_pairs_) {
      if (collides(bodyAt(pair.first), bodyAt(pair.second))) {
        pairs.insert(pair);
      }
    }
    return pairs;
  }

  std::set<std::pair<int, int>> sphereBodyPairs(const SphereResults &result,
                                                const std::vector<SpherePair> &candidate_pairs) {
    std::set<std::pair<int, int>> pairs;
    for (std::size_t index = 0; index < candidate_pairs.size(); ++index) {
      if (result.collisions.at(index)) {
        const SpherePair &pair = candidate_pairs[index];
        pairs.emplace(local_spheres_[pair.first].body, local_spheres_[pair.second].body);
      }
    }
    return pairs;
  }

  std::set<std::string> describePairs(const std::set<std::pair<int, int>> &pairs) {
    std::set<std::string> descriptions;
    for (const auto &pair : pairs) {
      const Body &first = bodyAt(pair.first);
      const Body &second = bodyAt(pair.second);
      if (pair.second < static_cast<int>(robot_bodies_.size())) {
        descriptions.insert("SELF: " + first.link_name + " <-> " + second.link_name);
      } else {
        descriptions.insert("ENVIRONMENT: " + first.link_name + " <-> " + second.name);
      }
    }
    return descriptions;
  }

  void applyCollisions(const std::set<std::pair<int, int>> &pairs) {
    std::set<std::string> self_links;
    std::set<std::string> environment_links;
    for (const auto &pair : pairs) {
      if (pair.second < static_cast<int>(robot_bodies_.size())) {
        self_links.insert(bodyAt(pair.first).link_name);
        self_links.insert(bodyAt(pair.second).link_name);
      } else {
        environment_links.insert(bodyAt(pair.first).link_name);
        bodyAt(pair.second).environment_collision = true;
      }
    }
    for (Body &body : robot_bodies_) {
      body.self_collision = self_links.count(body.link_name) > 0;
      body.environment_collision = environment_links.count(body.link_name) > 0;
    }
  }

  void publishBackend() {
    std_msgs::msg::String status;
    if (backend_ == "gpu_sdf") {
      status.data = backend_ + " | robot_spheres=" + std::to_string(robot_spheres_.size()) +
                    " | sdf_objects=" + std::to_string(sdf_objects_.size()) +
                    " | self_pairs=" + std::to_string(self_sphere_pairs_.size());
    } else {
      status.data = backend_ + " | spheres=" + std::to_string(local_spheres_.size()) +
                    " | pairs=" + std::to_string(sphere_pairs_.size());
    }
    if (backend_ == "gpu_spheres") {
      status.data += " | view=spheres";
    } else if (backend_ == "gpu_sdf") {
      status.data += " | view=robot_spheres_environment_sdf";
    } else {
      status.data += " | view=geometry";
    }
    if (gpu_ready_ || sdf_ready_) {
      status.data += " | GPU: " + gpu_device_;
    }
    backend_publisher_->publish(status);
  }

  void clearMarkers() {
    visualization_msgs::msg::MarkerArray markers;
    visualization_msgs::msg::Marker clear;
    clear.action = visualization_msgs::msg::Marker::DELETEALL;
    markers.markers.push_back(clear);
    marker_publisher_->publish(markers);
    sphere_marker_publisher_->publish(markers);
    sdf_marker_publisher_->publish(markers);
  }

  void publishSphereMarkers() {
    visualization_msgs::msg::MarkerArray markers;
    visualization_msgs::msg::Marker clear;
    clear.action = visualization_msgs::msg::Marker::DELETEALL;
    markers.markers.push_back(clear);
    if (backend_ != "fcl") {
      for (std::size_t index = 0; index < last_sphere_results_.world_spheres.size(); ++index) {
        const Sphere &sphere = last_sphere_results_.world_spheres[index];
        const Body &body = bodyAt(sphere.body);
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = fixed_frame_;
        marker.header.stamp = now();
        marker.ns = "gpu_spheres";
        marker.id = index;
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.position.x = sphere.x;
        marker.pose.position.y = sphere.y;
        marker.pose.position.z = sphere.z;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 2.0 * sphere.radius;
        marker.scale.y = 2.0 * sphere.radius;
        marker.scale.z = 2.0 * sphere.radius;
        marker.color.a = 0.65;
        if (body.self_collision && body.environment_collision) {
          marker.color.r = 1.0;
          marker.color.g = 0.9;
        } else if (body.self_collision) {
          marker.color.r = 1.0;
          marker.color.b = 1.0;
        } else if (body.environment_collision) {
          marker.color.r = 1.0;
        } else if (sphere.body < static_cast<int>(robot_bodies_.size())) {
          marker.color.r = 1.0;
          marker.color.g = 0.45;
          marker.color.b = 0.08;
        } else {
          marker.color.g = 0.65;
          marker.color.b = 0.85;
        }
        markers.markers.push_back(marker);
      }
    }
    sphere_marker_publisher_->publish(markers);
  }

  void publishSdfMarkers() {
    if (backend_ == "gpu_sdf") {
      sdf_marker_publisher_->publish(sdf_slices_);
    } else {
      visualization_msgs::msg::MarkerArray markers;
      visualization_msgs::msg::Marker clear;
      clear.action = visualization_msgs::msg::Marker::DELETEALL;
      markers.markers.push_back(clear);
      sdf_marker_publisher_->publish(markers);
    }
  }

  void validateSpheres(std_srvs::srv::Trigger::Response &response) {
    if (received_joints_.size() != joint_positions_.size()) {
      response.success = false;
      response.message = "Waiting for all joint positions";
      return;
    }
    try {
      prepareGpu();
      // Recompute from the current joint snapshot, even if the timer has not run yet.
      computeLinkTransforms(robot_.getRoot(), fcl::Transform3d::Identity());
      for (Body &body : robot_bodies_) {
        body.object->setTransform(link_transforms_.at(body.link_name) * body.local_transform);
        body.object->computeAABB();
      }
      const std::vector<SphereTransform> transforms = currentTransforms();
      const SphereResults gpu = gpu_.check(transforms);
      const SphereResults cpu = checkSpheresOnCpu(local_spheres_, sphere_pairs_, transforms);
      std::size_t mismatches = 0;
      for (std::size_t index = 0; index < sphere_pairs_.size(); ++index) {
        if (cpu.collisions[index] != gpu.collisions[index]) {
          ++mismatches;
        }
      }
      double largest_center_error = 0.0;
      for (std::size_t index = 0; index < local_spheres_.size(); ++index) {
        const Sphere &first = cpu.world_spheres[index];
        const Sphere &second = gpu.world_spheres[index];
        const double error = std::hypot(first.x - second.x, first.y - second.y, first.z - second.z);
        largest_center_error = std::max(largest_center_error, error);
      }
      const auto gpu_pairs = describePairs(sphereBodyPairs(gpu, sphere_pairs_));
      const auto fcl_pairs = describePairs(checkFclPairs());
      std::size_t sphere_only = 0;
      std::size_t fcl_only = 0;
      for (const std::string &pair : gpu_pairs) {
        if (fcl_pairs.count(pair) == 0) {
          ++sphere_only;
        }
      }
      for (const std::string &pair : fcl_pairs) {
        if (gpu_pairs.count(pair) == 0) {
          ++fcl_only;
        }
      }
      response.success = mismatches == 0 && largest_center_error < 1e-9;
      std::ostringstream report;
      report << "GPU / CPU sphere validation: " << mismatches << " mismatches in "
             << sphere_pairs_.size() << " pairs; max center error " << largest_center_error
             << " m.\n"
             << "Sphere approximation vs FCL: " << sphere_only << " sphere-only link/object pairs, "
             << fcl_only << " FCL-only pairs. Bounding-box spheres can over-report collision.";
      response.message = report.str();
    } catch (const std::exception &exception) {
      response.success = false;
      response.message = exception.what();
    }
  }

  void validateSdf(std_srvs::srv::Trigger::Response &response) {
    if (received_joints_.size() != joint_positions_.size()) {
      response.success = false;
      response.message = "Waiting for all joint positions";
      return;
    }
    try {
      prepareSdf();
      computeLinkTransforms(robot_.getRoot(), fcl::Transform3d::Identity());
      for (Body &body : robot_bodies_) {
        body.object->setTransform(link_transforms_.at(body.link_name) * body.local_transform);
        body.object->computeAABB();
      }
      const auto transforms = currentTransforms();
      const SphereResults gpu_self = gpu_self_.check(transforms);
      const SphereResults cpu_self =
          checkSpheresOnCpu(robot_spheres_, self_sphere_pairs_, transforms);
      const SdfResults gpu = gpu_sdf_.check(gpu_self.world_spheres);
      const SdfResults cpu = checkSdfOnCpu(cpu_self.world_spheres, sdf_objects_, sdf_triangles_);
      std::size_t mismatches = 0;
      double maximum_error = 0.0;
      for (std::size_t index = 0; index < gpu_self.collisions.size(); ++index) {
        if (gpu_self.collisions[index] != cpu_self.collisions[index]) {
          ++mismatches;
        }
      }
      for (std::size_t index = 0; index < gpu.collisions.size(); ++index) {
        if (gpu.collisions[index] != cpu.collisions[index]) {
          ++mismatches;
        }
        maximum_error =
            std::max(maximum_error, std::abs(gpu.distances[index] - cpu.distances[index]));
      }
      response.success = mismatches == 0 && maximum_error < 1e-9;
      std::ostringstream report;
      report << "GPU / CPU SDF validation: " << mismatches << " mismatches; "
             << gpu.collisions.size() << " sphere/object checks; " << gpu_self.collisions.size()
             << " self checks; max signed-distance error " << maximum_error << " m.";
      response.message = report.str();
    } catch (const std::exception &exception) {
      response.success = false;
      response.message = exception.what();
    }
  }

  double sdf_cell_size_ = 0.08;
  double sdf_padding_ = 0.25;
  visualization_msgs::msg::MarkerArray sdf_slices_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr sdf_marker_publisher_;
  bool sdf_ready_ = false;
  GpuSpheres gpu_self_;
  GpuSdf gpu_sdf_;
  std::vector<Sphere> robot_spheres_;
  std::vector<SpherePair> self_sphere_pairs_;
  std::vector<SdfObject> sdf_objects_;
  std::vector<SdfTriangle> sdf_triangles_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr sdf_backend_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr sdf_validation_service_;

  std::string backend_;
  bool gpu_ready_ = false;
  std::string gpu_device_;
  GpuSpheres gpu_;
  std::vector<Sphere> local_spheres_;
  std::vector<SpherePair> sphere_pairs_;
  std::vector<std::pair<int, int>> allowed_body_pairs_;
  SphereResults last_sphere_results_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr sphere_marker_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr backend_publisher_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr backend_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr validation_service_;

  urdf::Model robot_;
  std::string fixed_frame_;
  std::string previous_status_;
  std::map<std::string, double> joint_positions_;
  std::set<std::string> received_joints_;
  std::map<std::string, fcl::Transform3d> link_transforms_;
  std::set<std::pair<std::string, std::string>> ignored_pairs_;
  std::vector<Body> robot_bodies_;
  std::vector<Body> environment_bodies_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
};
} // namespace roscon_collision

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  int exit_code = 0;
  try {
    rclcpp::spin(std::make_shared<roscon_collision::CollisionNode>());
  } catch (const std::exception &exception) {
    RCLCPP_FATAL(rclcpp::get_logger("roscon_collision"), "%s", exception.what());
    exit_code = 1;
  }
  rclcpp::shutdown();
  return exit_code;
}
