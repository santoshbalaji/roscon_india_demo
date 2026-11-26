#ifndef ROSCON_COLLISION_SDF_DISTANCE_HPP
#define ROSCON_COLLISION_SDF_DISTANCE_HPP
#include "roscon_collision/gpu_sdf.hpp"
#include <cmath>
// The CPU and CUDA evaluators share exactly the same distance formulas.
#ifdef __CUDACC__
#define ROSCON_SDF_HOST_DEVICE __host__ __device__
#else
#define ROSCON_SDF_HOST_DEVICE
#endif
namespace roscon_collision {
// These small math functions can run on the GPU or the host validation path.
ROSCON_SDF_HOST_DEVICE inline SdfPoint subtract(SdfPoint first, SdfPoint second) {
  return {first.x - second.x, first.y - second.y, first.z - second.z};
}

ROSCON_SDF_HOST_DEVICE inline double dot(SdfPoint first, SdfPoint second) {
  return first.x * second.x + first.y * second.y + first.z * second.z;
}

ROSCON_SDF_HOST_DEVICE inline SdfPoint cross(SdfPoint first, SdfPoint second) {
  return {first.y * second.z - first.z * second.y, first.z * second.x - first.x * second.z,
          first.x * second.y - first.y * second.x};
}

ROSCON_SDF_HOST_DEVICE inline double length(SdfPoint vector) {
  return sqrt(dot(vector, vector));
}

ROSCON_SDF_HOST_DEVICE inline double segmentDistance(SdfPoint point, SdfPoint start, SdfPoint end) {
  const SdfPoint edge = subtract(end, start);
  const double fraction = fmax(0.0, fmin(1.0, dot(subtract(point, start), edge) / dot(edge, edge)));
  const SdfPoint closest = {start.x + fraction * edge.x, start.y + fraction * edge.y,
                            start.z + fraction * edge.z};
  return length(subtract(point, closest));
}

ROSCON_SDF_HOST_DEVICE inline double triangleDistance(SdfPoint point, const SdfTriangle &triangle) {
  const SdfPoint normal = cross(subtract(triangle.b, triangle.a), subtract(triangle.c, triangle.a));
  const double projection_scale = dot(subtract(point, triangle.a), normal) / dot(normal, normal);
  const SdfPoint projection = {point.x - projection_scale * normal.x,
                               point.y - projection_scale * normal.y,
                               point.z - projection_scale * normal.z};
  const double side_a =
      dot(cross(subtract(triangle.b, triangle.a), subtract(projection, triangle.a)), normal);
  const double side_b =
      dot(cross(subtract(triangle.c, triangle.b), subtract(projection, triangle.b)), normal);
  const double side_c =
      dot(cross(subtract(triangle.a, triangle.c), subtract(projection, triangle.c)), normal);
  if (side_a >= 0.0 && side_b >= 0.0 && side_c >= 0.0) {
    return fabs(projection_scale) * length(normal);
  }
  return fmin(segmentDistance(point, triangle.a, triangle.b),
              fmin(segmentDistance(point, triangle.b, triangle.c),
                   segmentDistance(point, triangle.c, triangle.a)));
}

ROSCON_SDF_HOST_DEVICE inline double signedDistance(SdfPoint point, const SdfObject &object,
                                                    const SdfTriangle *triangles) {
  if (object.shape == SdfShape::Sphere) {
    return length(point) - object.radius;
  }
  if (object.shape == SdfShape::Box) {
    const double x = fabs(point.x) - object.half_x;
    const double y = fabs(point.y) - object.half_y;
    const double z = fabs(point.z) - object.half_z;
    const SdfPoint outside = {fmax(x, 0.0), fmax(y, 0.0), fmax(z, 0.0)};
    return length(outside) + fmin(fmax(x, fmax(y, z)), 0.0);
  }
  if (object.shape == SdfShape::Cylinder) {
    const double radial = sqrt(point.x * point.x + point.y * point.y) - object.radius;
    const double axial = fabs(point.z) - object.half_z;
    const double outside_radial = fmax(radial, 0.0);
    const double outside_axial = fmax(axial, 0.0);
    return sqrt(outside_radial * outside_radial + outside_axial * outside_axial) +
           fmin(fmax(radial, axial), 0.0);
  }
  double minimum_distance = INFINITY;
  double solid_angle = 0.0;
  for (int index = 0; index < object.triangle_count; ++index) {
    const SdfTriangle &triangle = triangles[object.triangle_start + index];
    minimum_distance = fmin(minimum_distance, triangleDistance(point, triangle));
    if (minimum_distance < 1e-12) {
      return 0.0;
    }
    const SdfPoint a = subtract(triangle.a, point);
    const SdfPoint b = subtract(triangle.b, point);
    const SdfPoint c = subtract(triangle.c, point);
    const double denominator = length(a) * length(b) * length(c) + dot(a, b) * length(c) +
                               dot(b, c) * length(a) + dot(c, a) * length(b);
    solid_angle += 2.0 * atan2(dot(a, cross(b, c)), denominator);
  }
  // For a closed oriented surface, the angle sum is +/-4*pi inside and zero outside.
  if (fabs(solid_angle) > 6.283185307179586) {
    return -minimum_distance;
  }
  return minimum_distance;
}

ROSCON_SDF_HOST_DEVICE inline double
sphereCenterDistance(const Sphere &sphere, const SdfObject &object, const SdfTriangle *triangles) {
  const SphereTransform &transform = object.world_to_local;
  const SdfPoint local = {transform.values[0] * sphere.x + transform.values[1] * sphere.y +
                              transform.values[2] * sphere.z + transform.values[3],
                          transform.values[4] * sphere.x + transform.values[5] * sphere.y +
                              transform.values[6] * sphere.z + transform.values[7],
                          transform.values[8] * sphere.x + transform.values[9] * sphere.y +
                              transform.values[10] * sphere.z + transform.values[11]};
  return signedDistance(local, object, triangles);
}

} // namespace roscon_collision
#undef ROSCON_SDF_HOST_DEVICE
#endif
