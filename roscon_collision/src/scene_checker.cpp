#include "roscon_collision/scene_checker.hpp"
#include <cmath>
#include <limits>
#include <stdexcept>
#include <yaml-cpp/yaml.h>

namespace roscon_collision {
// Adjacent links share a mechanical joint and are excluded from self checks.
// Additional exclusions must be explicitly listed in the environment file.
void SceneChecker::loadRobot() {
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

std::pair<std::string, std::string> SceneChecker::orderedPair(const std::string &first,
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

void SceneChecker::loadEnvironment(const std::string &path) {
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
    throw std::runtime_error("Environment YAML needs an objects list (use [] for an empty scene)");
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

// URDF forward kinematics keeps every collision object in one coherent snapshot.
void SceneChecker::computeLinkTransforms(const urdf::LinkConstSharedPtr &link,
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

SceneChecker::SceneChecker(const std::string &xml, const std::string &environment_file,
                           double sphere_radius) {
  if (!robot_.initString(xml)) {
    throw std::runtime_error("Collision checker needs a valid URDF");
  }
  loadRobot();
  loadEnvironment(resolveAssetPath(environment_file));
  std::vector<size_t> starts;
  for (size_t body = 0; body < robot_bodies_.size(); ++body) {
    starts.push_back(spheres_.size());
    const auto cover = makeSphereModel(*robot_bodies_[body].shape.geometry, sphere_radius, body);
    spheres_.insert(spheres_.end(), cover.begin(), cover.end());
    if (spheres_.size() > 10000) {
      throw std::runtime_error("Collision model exceeds 10000 spheres; increase sphere radius");
    }
  }
  starts.push_back(spheres_.size());
  for (size_t first = 0; first < robot_bodies_.size(); ++first) {
    for (size_t second = first + 1; second < robot_bodies_.size(); ++second) {
      const auto &a = robot_bodies_[first];
      const auto &b = robot_bodies_[second];
      if (a.link_name == b.link_name ||
          ignored_pairs_.count(orderedPair(a.link_name, b.link_name))) {
        continue;
      }
      for (size_t i = starts[first]; i < starts[first + 1]; ++i) {
        for (size_t j = starts[second]; j < starts[second + 1]; ++j) {
          if (self_pairs_.size() >= 5000000) {
            throw std::runtime_error("Collision model exceeds 5000000 sphere pairs");
          }
          self_pairs_.push_back({static_cast<int>(i), static_cast<int>(j)});
        }
      }
    }
  }
  if (environment_bodies_.size() > 10000 ||
      environment_bodies_.size() * spheres_.size() > 5000000) {
    throw std::runtime_error("Collision scene exceeds SDF object or pair limits");
  }
  for (size_t index = 0; index < environment_bodies_.size(); ++index) {
    const auto &body = environment_bodies_[index];
    const auto cover = makeSphereModel(*body.shape.geometry, sphere_radius, index);
    for (const auto &local : cover) {
      const Eigen::Vector3d center =
          body.local_transform * Eigen::Vector3d(local.x, local.y, local.z);
      environment_spheres_.push_back(
          {center.x(), center.y(), center.z(), local.radius, static_cast<int>(index)});
    }
    if (environment_spheres_.size() > 10000) {
      throw std::runtime_error("Environment exceeds 10000 spheres");
    }
    sdf_objects_.push_back(
        makeSdfObject(*body.shape.geometry, body.local_transform, sdf_triangles_));
  }
}

std::vector<SphereTransform> SceneChecker::updateTransforms(const std::vector<std::string> &names,
                                                            const std::vector<double> &positions) {
  if (names.size() != positions.size() || names.size() != joint_positions_.size()) {
    throw std::runtime_error("Collision check requires all movable joints exactly once");
  }
  std::set<std::string> supplied;
  for (size_t i = 0; i < names.size(); ++i) {
    if (!joint_positions_.count(names[i]) || !supplied.insert(names[i]).second ||
        !std::isfinite(positions[i])) {
      throw std::runtime_error("Collision check received unknown, duplicate, or nonfinite joints");
    }
    joint_positions_.at(names[i]) = positions[i];
  }
  computeLinkTransforms(robot_.getRoot(), fcl::Transform3d::Identity());
  std::vector<SphereTransform> transforms;
  for (auto &body : robot_bodies_) {
    body.object->setTransform(link_transforms_.at(body.link_name) * body.local_transform);
    body.object->computeAABB();
    transforms.push_back(
        makeSphereTransform(link_transforms_.at(body.link_name) * body.local_transform));
  }
  return transforms;
}

CollisionReport SceneChecker::check(const std::vector<std::string> &names,
                                    const std::vector<double> &positions,
                                    const std::string &backend) {
  if (backend != "fcl" && backend != "spheres" && backend != "sdf") {
    throw std::runtime_error("Collision backend must be fcl, spheres, or sdf");
  }
  const auto transforms = updateTransforms(names, positions);
  CollisionReport report;
  std::set<std::string> pairs;
  auto add_self = [&](size_t first, size_t second) {
    report.self_collision = true;
    pairs.insert("SELF: " + robot_bodies_[first].link_name + " <-> " +
                 robot_bodies_[second].link_name);
  };
  auto add_environment = [&](size_t first, size_t second) {
    report.environment_collision = true;
    pairs.insert("ENVIRONMENT: " + robot_bodies_[first].link_name + " <-> " +
                 environment_bodies_[second].name);
  };
  if (backend == "fcl") {
    auto collides = [](const Body &first, const Body &second) {
      fcl::CollisionRequestd request;
      fcl::CollisionResultd result;
      fcl::collide(first.object.get(), second.object.get(), request, result);
      return result.isCollision();
    };
    for (size_t first = 0; first < robot_bodies_.size(); ++first) {
      for (size_t second = first + 1; second < robot_bodies_.size(); ++second) {
        const auto &a = robot_bodies_[first];
        const auto &b = robot_bodies_[second];
        if (a.link_name == b.link_name ||
            ignored_pairs_.count(orderedPair(a.link_name, b.link_name))) {
          continue;
        }
        if (collides(a, b)) {
          add_self(first, second);
        }
      }
      for (size_t object = 0; object < environment_bodies_.size(); ++object) {
        if (collides(robot_bodies_[first], environment_bodies_[object])) {
          add_environment(first, object);
        }
      }
    }
  } else {
    const auto self = checkSpheresOnCpu(spheres_, self_pairs_, transforms);
    for (size_t i = 0; i < self_pairs_.size(); ++i) {
      if (self.collisions[i]) {
        const auto &pair = self_pairs_[i];
        add_self(spheres_[pair.first].body, spheres_[pair.second].body);
      }
    }
    if (backend == "spheres") {
      for (const auto &robot : self.world_spheres) {
        for (const auto &obstacle : environment_spheres_) {
          const double distance =
              std::hypot(robot.x - obstacle.x, robot.y - obstacle.y, robot.z - obstacle.z);
          if (distance <= robot.radius + obstacle.radius) {
            add_environment(robot.body, obstacle.body);
          }
        }
      }
    } else {
      const auto environment = checkSdfOnCpu(self.world_spheres, sdf_objects_, sdf_triangles_);
      for (size_t sphere = 0; sphere < spheres_.size(); ++sphere) {
        for (size_t object = 0; object < sdf_objects_.size(); ++object) {
          const size_t index = sphere * sdf_objects_.size() + object;
          if (!std::isfinite(environment.distances[index])) {
            throw std::runtime_error("SDF check returned a nonfinite distance");
          }
          if (environment.collisions[index]) {
            add_environment(spheres_[sphere].body, object);
          }
        }
      }
    }
  }
  report.pairs.assign(pairs.begin(), pairs.end());
  return report;
}

double SceneChecker::clearance(const std::vector<std::string> &names,
                               const std::vector<double> &positions, const std::string &backend) {
  if (backend != "fcl" && backend != "spheres" && backend != "sdf") {
    throw std::runtime_error("Unknown collision backend");
  }
  const auto transforms = updateTransforms(names, positions);
  double minimum = std::numeric_limits<double>::infinity();
  if (backend == "fcl") {
    auto distance = [](const Body &first, const Body &second) {
      fcl::DistanceRequestd request;
      fcl::DistanceResultd result;
      const double value = fcl::distance(first.object.get(), second.object.get(), request, result);
      if (!std::isfinite(value)) {
        throw std::runtime_error("FCL distance evaluation failed");
      }
      return value;
    };
    for (size_t first = 0; first < robot_bodies_.size(); ++first) {
      for (size_t second = first + 1; second < robot_bodies_.size(); ++second) {
        const auto &a = robot_bodies_[first];
        const auto &b = robot_bodies_[second];
        if (a.link_name == b.link_name ||
            ignored_pairs_.count(orderedPair(a.link_name, b.link_name))) {
          continue;
        }
        minimum = std::min(minimum, distance(a, b));
      }
      for (const auto &obstacle : environment_bodies_) {
        minimum = std::min(minimum, distance(robot_bodies_[first], obstacle));
      }
    }
  } else {
    const auto world = checkSpheresOnCpu(spheres_, {}, transforms).world_spheres;
    auto separation = [](const Sphere &a, const Sphere &b) {
      return std::hypot(a.x - b.x, a.y - b.y, a.z - b.z) - a.radius - b.radius;
    };
    for (const auto &pair : self_pairs_) {
      minimum = std::min(minimum, separation(world[pair.first], world[pair.second]));
    }
    if (backend == "spheres") {
      for (const auto &sphere : world) {
        for (const auto &obstacle : environment_spheres_) {
          minimum = std::min(minimum, separation(sphere, obstacle));
        }
      }
    } else {
      const auto distances = checkSdfOnCpu(world, sdf_objects_, sdf_triangles_).distances;
      for (size_t sphere = 0; sphere < world.size(); ++sphere) {
        for (size_t object = 0; object < sdf_objects_.size(); ++object) {
          const double value =
              distances[sphere * sdf_objects_.size() + object] - world[sphere].radius;
          if (!std::isfinite(value)) {
            throw std::runtime_error("SDF distance evaluation failed");
          }
          minimum = std::min(minimum, value);
        }
      }
    }
  }
  return minimum;
}

visualization_msgs::msg::MarkerArray SceneChecker::environmentMarkers() const {
  visualization_msgs::msg::MarkerArray result;
  for (size_t i = 0; i < environment_bodies_.size(); ++i) {
    const auto &body = environment_bodies_[i];
    auto marker = body.shape.marker;
    marker.header.frame_id = frame();
    marker.ns = "ik_environment";
    marker.id = i;
    marker.pose = transformToPose(body.local_transform);
    marker.color.r = 0.25;
    marker.color.g = 0.65;
    marker.color.b = 0.85;
    marker.color.a = 0.85;
    result.markers.push_back(marker);
  }
  return result;
}
} // namespace roscon_collision
