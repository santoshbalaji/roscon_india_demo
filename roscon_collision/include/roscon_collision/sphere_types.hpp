#ifndef ROSCON_COLLISION_SPHERE_TYPES_HPP
#define ROSCON_COLLISION_SPHERE_TYPES_HPP

#include <vector>

namespace roscon_collision
{
struct Sphere
{
  double x;
  double y;
  double z;
  double radius;
  int body;
};

struct SpherePair
{
  int first;
  int second;
};

// Row-major 3 x 4 rigid transform: rotation and translation.
struct SphereTransform
{
  double values[12];
};

struct SphereResults
{
  std::vector<Sphere> world_spheres;
  std::vector<unsigned char> collisions;
};

// Independent CPU reference, used only by explicit validation and tests.
SphereResults checkSpheresOnCpu(
  const std::vector<Sphere> & spheres,
  const std::vector<SpherePair> & pairs,
  const std::vector<SphereTransform> & transforms);
}  // namespace roscon_collision
#endif
