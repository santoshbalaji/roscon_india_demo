#ifndef ROSCON_COLLISION_SPHERE_MODEL_HPP
#define ROSCON_COLLISION_SPHERE_MODEL_HPP
#include "roscon_collision/sphere_types.hpp"
#include <fcl/fcl.h>
namespace roscon_collision
{
// Conservative cover of the local geometry bounds, generated once at startup.
std::vector<Sphere> makeSphereModel(fcl::CollisionGeometryd & geometry, double radius, int body);
SphereTransform makeSphereTransform(const fcl::Transform3d & transform);
}
#endif
