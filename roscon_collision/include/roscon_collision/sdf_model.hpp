#ifndef ROSCON_COLLISION_SDF_MODEL_HPP
#define ROSCON_COLLISION_SDF_MODEL_HPP
#include "roscon_collision/gpu_sdf.hpp"
#include <fcl/fcl.h>
namespace roscon_collision {
// Append mesh triangles when needed. Meshes must be closed and consistently oriented.
SdfObject makeSdfObject(const fcl::CollisionGeometryd &geometry, const fcl::Transform3d &world_pose,
                        std::vector<SdfTriangle> &triangles);
} // namespace roscon_collision
#endif
