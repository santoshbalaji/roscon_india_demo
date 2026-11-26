#ifndef ROSCON_COLLISION_SCENE_CHECKER_HPP
#define ROSCON_COLLISION_SCENE_CHECKER_HPP

#include "roscon_collision/geometry.hpp"
#include "roscon_collision/sdf_model.hpp"
#include "roscon_collision/sphere_model.hpp"
#include <map>
#include <set>
#include <urdf/model.h>
#include <visualization_msgs/msg/marker_array.hpp>

namespace roscon_collision {
struct CollisionReport {
  bool self_collision = false;
  bool environment_collision = false;
  std::vector<std::string> pairs;
  bool collision_free() const {
    return !self_collision && !environment_collision;
  }
};

// Synchronous candidate checking. CPU evaluation is intentional: no CUDA or
// /joint_states dependency, and no mutation of the displayed robot.
class SceneChecker {
public:
  SceneChecker(const std::string &xml, const std::string &environment_file, double sphere_radius);
  CollisionReport check(const std::vector<std::string> &names, const std::vector<double> &positions,
                        const std::string &backend = "sdf");
  // Minimum surface separation over the same enabled pairs as check().
  // FCL penetration distances are backend-dependent; final acceptance uses check().
  double clearance(const std::vector<std::string> &names, const std::vector<double> &positions,
                   const std::string &backend);
  visualization_msgs::msg::MarkerArray environmentMarkers() const;
  std::string frame() const {
    return robot_.getRoot()->name;
  }

private:
  struct Body {
    std::string name;
    std::string link_name;
    Shape shape;
    fcl::Transform3d local_transform = fcl::Transform3d::Identity();
    std::shared_ptr<fcl::CollisionObjectd> object;
  };
  std::vector<SphereTransform> updateTransforms(const std::vector<std::string> &names,
                                                const std::vector<double> &positions);
  bool ignoreSelfPair(const Body &first, const Body &second) const;
  void loadRobot();
  void loadEnvironment(const std::string &path);
  void computeLinkTransforms(const urdf::LinkConstSharedPtr &link,
                             const fcl::Transform3d &parent_transform);
  static std::pair<std::string, std::string> orderedPair(const std::string &first,
                                                         const std::string &second);
  urdf::Model robot_;
  std::map<std::string, double> joint_positions_;
  std::map<std::string, fcl::Transform3d> link_transforms_;
  std::set<std::pair<std::string, std::string>> ignored_pairs_;
  std::vector<Body> robot_bodies_;
  std::vector<Body> environment_bodies_;
  std::vector<Sphere> spheres_;
  std::vector<SpherePair> self_pairs_;
  std::vector<Sphere> environment_spheres_;
  std::vector<SdfObject> sdf_objects_;
  std::vector<SdfTriangle> sdf_triangles_;
};
} // namespace roscon_collision
#endif
