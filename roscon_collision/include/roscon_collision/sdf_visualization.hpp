#ifndef ROSCON_COLLISION_SDF_VISUALIZATION_HPP
#define ROSCON_COLLISION_SDF_VISUALIZATION_HPP

#include <fcl/fcl.h>
#include <visualization_msgs/msg/marker.hpp>
#include "roscon_collision/gpu_sdf.hpp"

namespace roscon_collision
{
// A colored XZ slice through the center of one object's local bounds.
visualization_msgs::msg::Marker makeSdfSlice(
  const SdfObject & object, const std::vector<SdfTriangle> & triangles,
  const fcl::AABBd & bounds, const fcl::Transform3d & pose,
  double cell_size, double padding);
}
#endif
