#include "roscon_collision/gpu_spheres.hpp"

#include <cuda_runtime.h>
#include <stdexcept>
#include <cmath>

namespace roscon_collision
{
static void requireCuda(cudaError_t result, const std::string & operation)
{
  if (result != cudaSuccess) {
    throw std::runtime_error(operation + ": " + cudaGetErrorString(result));
  }
}

// One thread moves one sphere from its body's local frame into the robot root frame.
__global__ void transformSpheres(
  const Sphere * local_spheres, Sphere * world_spheres,
  const SphereTransform * transforms, int sphere_count)
{
  const int index = blockIdx.x * blockDim.x + threadIdx.x;
  if (index >= sphere_count) {
    return;
  }
  const Sphere sphere = local_spheres[index];
  const SphereTransform transform = transforms[sphere.body];
  Sphere world = sphere;
  world.x = transform.values[0] * sphere.x + transform.values[1] * sphere.y +
    transform.values[2] * sphere.z + transform.values[3];
  world.y = transform.values[4] * sphere.x + transform.values[5] * sphere.y +
    transform.values[6] * sphere.z + transform.values[7];
  world.z = transform.values[8] * sphere.x + transform.values[9] * sphere.y +
    transform.values[10] * sphere.z + transform.values[11];
  world_spheres[index] = world;
}

// One thread checks one allowed pair. Touching spheres count as collision.
__global__ void collideSpheres(
  const Sphere * spheres, const SpherePair * pairs,
  unsigned char * collisions, int pair_count)
{
  const int index = blockIdx.x * blockDim.x + threadIdx.x;
  if (index >= pair_count) {
    return;
  }
  const Sphere first = spheres[pairs[index].first];
  const Sphere second = spheres[pairs[index].second];
  const double difference_x = first.x - second.x;
  const double difference_y = first.y - second.y;
  const double difference_z = first.z - second.z;
  const double radius_sum = first.radius + second.radius;
  const double distance_squared = difference_x * difference_x +
    difference_y * difference_y + difference_z * difference_z;
  collisions[index] = distance_squared <= radius_sum * radius_sum;
}

std::string GpuSpheres::deviceName()
{
  int device_count = 0;
  requireCuda(cudaGetDeviceCount(&device_count), "CUDA device discovery");
  if (device_count == 0) {
    throw std::runtime_error("No CUDA GPU is available");
  }
  requireCuda(cudaSetDevice(0), "Select CUDA device 0");
  cudaDeviceProp properties;
  requireCuda(cudaGetDeviceProperties(&properties, 0), "Read CUDA device properties");
  return properties.name;
}

GpuSpheres::~GpuSpheres()
{
  release();
}

void GpuSpheres::release()
{
  cudaFree(local_spheres_);
  cudaFree(world_spheres_);
  cudaFree(pairs_);
  cudaFree(transforms_);
  cudaFree(collisions_);
  local_spheres_ = nullptr;
  world_spheres_ = nullptr;
  pairs_ = nullptr;
  transforms_ = nullptr;
  collisions_ = nullptr;
  sphere_count_ = 0;
  pair_count_ = 0;
  body_count_ = 0;
}

void GpuSpheres::initialize(
  const std::vector<Sphere> & spheres,
  const std::vector<SpherePair> & pairs, int body_count)
{
  deviceName();
  release();
  if (spheres.empty() || spheres.size() > 10000 || pairs.size() > 5000000 || body_count <= 0) {
    throw std::runtime_error("Sphere scene must have 1..10000 spheres and at most 5000000 pairs");
  }
  for (const Sphere & sphere : spheres) {
    if (sphere.body < 0 || sphere.body >= body_count || !std::isfinite(sphere.x) ||
      !std::isfinite(sphere.y) || !std::isfinite(sphere.z) ||
      !std::isfinite(sphere.radius) || sphere.radius <= 0.0)
    {
      throw std::runtime_error("Invalid sphere center, radius, or body index");
    }
  }
  for (const SpherePair & pair : pairs) {
    if (pair.first < 0 || pair.second < 0 ||
      pair.first >= static_cast<int>(spheres.size()) || pair.second >= static_cast<int>(spheres.size()))
    {
      throw std::runtime_error("Invalid sphere pair index");
    }
  }
  try {
    sphere_count_ = static_cast<int>(spheres.size());
    pair_count_ = static_cast<int>(pairs.size());
    body_count_ = body_count;
    requireCuda(cudaMalloc(&local_spheres_, spheres.size() * sizeof(Sphere)), "Allocate local spheres");
    requireCuda(cudaMalloc(&world_spheres_, spheres.size() * sizeof(Sphere)), "Allocate world spheres");
    requireCuda(cudaMalloc(&transforms_, body_count * sizeof(SphereTransform)), "Allocate transforms");
    requireCuda(cudaMemcpy(local_spheres_, spheres.data(), spheres.size() * sizeof(Sphere),
      cudaMemcpyHostToDevice), "Upload spheres");
    if (!pairs.empty()) {
      requireCuda(cudaMalloc(&pairs_, pairs.size() * sizeof(SpherePair)), "Allocate sphere pairs");
      requireCuda(cudaMalloc(&collisions_, pairs.size()), "Allocate collision results");
      requireCuda(cudaMemcpy(pairs_, pairs.data(), pairs.size() * sizeof(SpherePair),
        cudaMemcpyHostToDevice), "Upload sphere pairs");
    }
  } catch (...) {
    release();
    throw;
  }
}

SphereResults GpuSpheres::check(const std::vector<SphereTransform> & transforms)
{
  if (sphere_count_ == 0 || static_cast<int>(transforms.size()) != body_count_) {
    throw std::runtime_error("GPU sphere scene is not initialized or transform count is incorrect");
  }
  requireCuda(cudaMemcpy(transforms_, transforms.data(), transforms.size() * sizeof(SphereTransform),
    cudaMemcpyHostToDevice), "Upload body transforms");
  const int threads_per_block = 256;
  const int sphere_blocks = (sphere_count_ + threads_per_block - 1) / threads_per_block;
  transformSpheres<<<sphere_blocks, threads_per_block>>>(
    local_spheres_, world_spheres_, transforms_, sphere_count_);
  requireCuda(cudaGetLastError(), "Launch sphere transform kernel");
  if (pair_count_ > 0) {
    const int pair_blocks = (pair_count_ + threads_per_block - 1) / threads_per_block;
    collideSpheres<<<pair_blocks, threads_per_block>>>(
      world_spheres_, pairs_, collisions_, pair_count_);
    requireCuda(cudaGetLastError(), "Launch sphere collision kernel");
  }
  requireCuda(cudaDeviceSynchronize(), "Complete GPU sphere checks");
  SphereResults results;
  results.world_spheres.resize(sphere_count_);
  results.collisions.resize(pair_count_);
  requireCuda(cudaMemcpy(results.world_spheres.data(), world_spheres_, sphere_count_ * sizeof(Sphere),
    cudaMemcpyDeviceToHost), "Download world spheres");
  if (pair_count_ > 0) {
    requireCuda(cudaMemcpy(results.collisions.data(), collisions_, pair_count_,
      cudaMemcpyDeviceToHost), "Download collision results");
  }
  return results;
}
}  // namespace roscon_collision
