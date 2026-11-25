#ifndef ROSCON_COLLISION_SDF_MODEL_HPP
#define ROSCON_COLLISION_SDF_MODEL_HPP
#include <fcl/fcl.h>
#include "roscon_collision/gpu_sdf.hpp"
namespace roscon_collision
{
// Append mesh triangles when needed. Meshes must be closed and consistently oriented.
SdfObject makeSdfObject(
  const fcl::CollisionGeometryd & geometry, const fcl::Transform3d & world_pose,
  std::vector<SdfTriangle> & triangles);
}
#endif
