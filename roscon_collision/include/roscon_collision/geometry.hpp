#ifndef ROSCON_COLLISION_GEOMETRY_HPP
#define ROSCON_COLLISION_GEOMETRY_HPP

#include <fcl/fcl.h>
#include <memory>
#include <string>
#include <urdf_model/link.h>
#include <urdf_model/pose.h>
#include <visualization_msgs/msg/marker.hpp>

namespace roscon_collision {
// The marker and FCL geometry describe the same shape in its local frame.
struct Shape {
  std::shared_ptr<fcl::CollisionGeometryd> geometry;
  visualization_msgs::msg::Marker marker;
};

std::string resolveAssetPath(const std::string &resource);
fcl::Transform3d poseToTransform(const urdf::Pose &pose);
geometry_msgs::msg::Pose transformToPose(const fcl::Transform3d &transform);
Shape loadShape(const urdf::Geometry &geometry);
} // namespace roscon_collision
#endif
