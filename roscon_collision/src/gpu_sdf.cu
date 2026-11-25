#include "roscon_collision/gpu_sdf.hpp"
#include "roscon_collision/sdf_distance.hpp"
#include "roscon_collision/gpu_spheres.hpp"
#include <cuda_runtime.h>
#include <cmath>
#include <stdexcept>
#include <string>

namespace roscon_collision
{
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
