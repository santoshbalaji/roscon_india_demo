#ifndef ROSCON_COLLISION_GPU_SDF_HPP
#define ROSCON_COLLISION_GPU_SDF_HPP

#include <vector>
#include "roscon_collision/sphere_types.hpp"

namespace roscon_collision
{
enum class SdfShape {Box, Sphere, Cylinder, Mesh};

struct SdfPoint
{
  double x;
  double y;
  double z;
};

struct SdfTriangle
{
  SdfPoint a;
  SdfPoint b;
  SdfPoint c;
};

struct SdfObject
{
  SdfShape shape;
  SphereTransform world_to_local;
  double half_x = 0.0;
  double half_y = 0.0;
  double half_z = 0.0;
  double radius = 0.0;
  int triangle_start = 0;
  int triangle_count = 0;
};

// One result per robot sphere/environment object pair, in sphere-major order.
struct SdfResults
{
  std::vector<double> distances;
  std::vector<unsigned char> collisions;
};

// Explicit CPU evaluation, shared by validation and the synchronous IK checker.
// The GPU backend never switches to CPU implicitly.
SdfResults checkSdfOnCpu(
  const std::vector<Sphere> & world_spheres,
  const std::vector<SdfObject> & objects,
  const std::vector<SdfTriangle> & triangles);

class GpuSdf
{
public:
  GpuSdf() = default;
  ~GpuSdf();
  GpuSdf(const GpuSdf &) = delete;
  GpuSdf & operator=(const GpuSdf &) = delete;
  void initialize(
    const std::vector<SdfObject> & objects,
    const std::vector<SdfTriangle> & triangles, int sphere_count);
  SdfResults check(const std::vector<Sphere> & world_spheres);

private:
  void release();
  SdfObject * objects_ = nullptr;
  SdfTriangle * triangles_ = nullptr;
  Sphere * spheres_ = nullptr;
  double * distances_ = nullptr;
  unsigned char * collisions_ = nullptr;
  int object_count_ = 0;
  int sphere_count_ = 0;
};
}  // namespace roscon_collision
#endif
