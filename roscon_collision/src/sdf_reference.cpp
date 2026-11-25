#include "roscon_collision/sdf_distance.hpp"
namespace roscon_collision {
SdfResults checkSdfOnCpu(
  const std::vector<Sphere> & spheres, const std::vector<SdfObject> & objects,
  const std::vector<SdfTriangle> & triangles)
{
  SdfResults results;
  for (const Sphere & sphere : spheres) {
    for (const SdfObject & object : objects) {
      const double distance = sphereCenterDistance(sphere, object, triangles.data());
      results.distances.push_back(distance);
      results.collisions.push_back(distance <= sphere.radius);
    }
  }
  return results;
}

}
