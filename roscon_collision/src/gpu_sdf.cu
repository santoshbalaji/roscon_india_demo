#include "roscon_collision/gpu_sdf.hpp"
#include "roscon_collision/gpu_spheres.hpp"
#include <cuda_runtime.h>
#include <cmath>
#include <stdexcept>
#include <string>

namespace roscon_collision
{
// These small math functions can run on the GPU or the host validation path.
__host__ __device__ static SdfPoint subtract(SdfPoint first, SdfPoint second)
{
  return {first.x - second.x, first.y - second.y, first.z - second.z};
}

__host__ __device__ static double dot(SdfPoint first, SdfPoint second)
{
  return first.x * second.x + first.y * second.y + first.z * second.z;
}

__host__ __device__ static SdfPoint cross(SdfPoint first, SdfPoint second)
{
  return {first.y * second.z - first.z * second.y,
    first.z * second.x - first.x * second.z, first.x * second.y - first.y * second.x};
}

__host__ __device__ static double length(SdfPoint vector)
{
  return sqrt(dot(vector, vector));
}

__host__ __device__ static double segmentDistance(SdfPoint point, SdfPoint start, SdfPoint end)
{
  const SdfPoint edge = subtract(end, start);
  const double fraction = fmax(0.0, fmin(1.0, dot(subtract(point, start), edge) / dot(edge, edge)));
  const SdfPoint closest = {start.x + fraction * edge.x, start.y + fraction * edge.y,
    start.z + fraction * edge.z};
  return length(subtract(point, closest));
}

__host__ __device__ static double triangleDistance(SdfPoint point, const SdfTriangle & triangle)
{
  const SdfPoint normal = cross(subtract(triangle.b, triangle.a), subtract(triangle.c, triangle.a));
  const double projection_scale = dot(subtract(point, triangle.a), normal) / dot(normal, normal);
  const SdfPoint projection = {point.x - projection_scale * normal.x,
    point.y - projection_scale * normal.y, point.z - projection_scale * normal.z};
  const double side_a = dot(cross(subtract(triangle.b, triangle.a), subtract(projection, triangle.a)), normal);
  const double side_b = dot(cross(subtract(triangle.c, triangle.b), subtract(projection, triangle.b)), normal);
  const double side_c = dot(cross(subtract(triangle.a, triangle.c), subtract(projection, triangle.c)), normal);
  if (side_a >= 0.0 && side_b >= 0.0 && side_c >= 0.0) {
    return fabs(projection_scale) * length(normal);
  }
  return fmin(segmentDistance(point, triangle.a, triangle.b),
    fmin(segmentDistance(point, triangle.b, triangle.c), segmentDistance(point, triangle.c, triangle.a)));
}

__host__ __device__ static double signedDistance(
  SdfPoint point, const SdfObject & object, const SdfTriangle * triangles)
{
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
    const SdfTriangle & triangle = triangles[object.triangle_start + index];
    minimum_distance = fmin(minimum_distance, triangleDistance(point, triangle));
    if (minimum_distance < 1e-12) {
      return 0.0;
    }
    const SdfPoint a = subtract(triangle.a, point);
    const SdfPoint b = subtract(triangle.b, point);
    const SdfPoint c = subtract(triangle.c, point);
    const double denominator = length(a) * length(b) * length(c) +
      dot(a, b) * length(c) + dot(b, c) * length(a) + dot(c, a) * length(b);
    solid_angle += 2.0 * atan2(dot(a, cross(b, c)), denominator);
  }
  // For a closed oriented surface, the angle sum is +/-4*pi inside and zero outside.
  if (fabs(solid_angle) > 6.283185307179586) {
    return -minimum_distance;
  }
  return minimum_distance;
}

__host__ __device__ static double sphereCenterDistance(
  const Sphere & sphere, const SdfObject & object, const SdfTriangle * triangles)
{
  const SphereTransform & transform = object.world_to_local;
  const SdfPoint local = {
    transform.values[0] * sphere.x + transform.values[1] * sphere.y + transform.values[2] * sphere.z + transform.values[3],
    transform.values[4] * sphere.x + transform.values[5] * sphere.y + transform.values[6] * sphere.z + transform.values[7],
    transform.values[8] * sphere.x + transform.values[9] * sphere.y + transform.values[10] * sphere.z + transform.values[11]};
  return signedDistance(local, object, triangles);
}

// One thread checks one robot sphere against one environment SDF.
__global__ void collideSdf(
  const Sphere * spheres, const SdfObject * objects, const SdfTriangle * triangles,
  double * distances, unsigned char * collisions, int sphere_count, int object_count)
{
  const int index = blockIdx.x * blockDim.x + threadIdx.x;
  if (index >= sphere_count * object_count) {
    return;
  }
  const Sphere & sphere = spheres[index / object_count];
  const SdfObject & object = objects[index % object_count];
  const double distance = sphereCenterDistance(sphere, object, triangles);
  distances[index] = distance;
  collisions[index] = distance <= sphere.radius;
}

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

static void requireCuda(cudaError_t result, const std::string & operation)
{
  if (result != cudaSuccess) {
    throw std::runtime_error(operation + ": " + cudaGetErrorString(result));
  }
}

GpuSdf::~GpuSdf()
{
  release();
}

void GpuSdf::release()
{
  cudaFree(objects_);
  cudaFree(triangles_);
  cudaFree(spheres_);
  cudaFree(distances_);
  cudaFree(collisions_);
  objects_ = nullptr;
  triangles_ = nullptr;
  spheres_ = nullptr;
  distances_ = nullptr;
  collisions_ = nullptr;
  sphere_count_ = 0;
  object_count_ = 0;
}

void GpuSdf::initialize(
  const std::vector<SdfObject> & objects, const std::vector<SdfTriangle> & triangles, int sphere_count)
{
  GpuSpheres::deviceName();
  release();
  if (sphere_count <= 0 || sphere_count > 10000 || objects.size() > 10000 ||
    objects.size() * sphere_count > 5000000 || triangles.size() > 100000)
  {
    throw std::runtime_error("SDF scene exceeds sphere, object, triangle, or pair limits");
  }
  for (const SdfObject & object : objects) {
    if (object.triangle_start < 0 || object.triangle_count < 0 ||
      static_cast<std::size_t>(object.triangle_start) + object.triangle_count > triangles.size())
    {
      throw std::runtime_error("Invalid SDF mesh triangle range");
    }
  }
  try {
    sphere_count_ = sphere_count;
    object_count_ = objects.size();
    const std::size_t pair_count = objects.size() * sphere_count;
    if (objects.empty()) {
      return;
    }
    requireCuda(cudaMalloc(&objects_, objects.size() * sizeof(SdfObject)), "Allocate SDF objects");
    requireCuda(cudaMemcpy(objects_, objects.data(), objects.size() * sizeof(SdfObject), cudaMemcpyHostToDevice), "Upload SDF objects");
    if (!triangles.empty()) {
      requireCuda(cudaMalloc(&triangles_, triangles.size() * sizeof(SdfTriangle)), "Allocate SDF triangles");
      requireCuda(cudaMemcpy(triangles_, triangles.data(), triangles.size() * sizeof(SdfTriangle), cudaMemcpyHostToDevice), "Upload SDF triangles");
    }
    requireCuda(cudaMalloc(&spheres_, sphere_count * sizeof(Sphere)), "Allocate SDF sphere input");
    requireCuda(cudaMalloc(&distances_, pair_count * sizeof(double)), "Allocate SDF distances");
    requireCuda(cudaMalloc(&collisions_, pair_count), "Allocate SDF collisions");
  } catch (...) {
    release();
    throw;
  }
}

SdfResults GpuSdf::check(const std::vector<Sphere> & spheres)
{
  if (sphere_count_ == 0 || static_cast<int>(spheres.size()) != sphere_count_) {
    throw std::runtime_error("SDF backend is not initialized or sphere count changed");
  }
  SdfResults results;
  const int pair_count = sphere_count_ * object_count_;
  if (pair_count == 0) {
    return results;
  }
  results.distances.resize(pair_count);
  results.collisions.resize(pair_count);
  requireCuda(cudaMemcpy(spheres_, spheres.data(), spheres.size() * sizeof(Sphere), cudaMemcpyHostToDevice), "Upload robot spheres for SDF");
  const int threads = 256;
  collideSdf<<<(pair_count + threads - 1) / threads, threads>>>(
    spheres_, objects_, triangles_, distances_, collisions_, sphere_count_, object_count_);
  requireCuda(cudaGetLastError(), "Launch sphere-SDF kernel");
  requireCuda(cudaDeviceSynchronize(), "Complete sphere-SDF checks");
  requireCuda(cudaMemcpy(results.distances.data(), distances_, pair_count * sizeof(double), cudaMemcpyDeviceToHost), "Download SDF distances");
  requireCuda(cudaMemcpy(results.collisions.data(), collisions_, pair_count, cudaMemcpyDeviceToHost), "Download SDF collisions");
  return results;
}
}  // namespace roscon_collision
