#include "roscon_collision/sphere_types.hpp"

namespace roscon_collision
{
SphereResults checkSpheresOnCpu(
  const std::vector<Sphere> & spheres,
  const std::vector<SpherePair> & pairs,
  const std::vector<SphereTransform> & transforms)
{
  SphereResults result;
  for (const Sphere & local : spheres) {
    const SphereTransform & transform = transforms.at(local.body);
    Sphere world = local;
    world.x = transform.values[0] * local.x + transform.values[1] * local.y +
      transform.values[2] * local.z + transform.values[3];
    world.y = transform.values[4] * local.x + transform.values[5] * local.y +
      transform.values[6] * local.z + transform.values[7];
    world.z = transform.values[8] * local.x + transform.values[9] * local.y +
      transform.values[10] * local.z + transform.values[11];
    result.world_spheres.push_back(world);
  }
  for (const SpherePair & pair : pairs) {
    const Sphere & first = result.world_spheres.at(pair.first);
    const Sphere & second = result.world_spheres.at(pair.second);
    const double difference_x = first.x - second.x;
    const double difference_y = first.y - second.y;
    const double difference_z = first.z - second.z;
    const double distance_squared = difference_x * difference_x + difference_y * difference_y +
      difference_z * difference_z;
    const double radius_sum = first.radius + second.radius;
    result.collisions.push_back(distance_squared <= radius_sum * radius_sum);
  }
  return result;
}
}  // namespace roscon_collision
