#ifndef ROSCON_COLLISION_GPU_SPHERES_HPP
#define ROSCON_COLLISION_GPU_SPHERES_HPP

#include "roscon_collision/sphere_types.hpp"
#include <string>

namespace roscon_collision {
// Owns the device buffers. Geometry and candidate pairs are uploaded once.
class GpuSpheres {
public:
  GpuSpheres() = default;
  ~GpuSpheres();
  GpuSpheres(const GpuSpheres &) = delete;
  GpuSpheres &operator=(const GpuSpheres &) = delete;

  static std::string deviceName();
  void initialize(const std::vector<Sphere> &spheres, const std::vector<SpherePair> &pairs,
                  int body_count);
  SphereResults check(const std::vector<SphereTransform> &transforms);

private:
  void release();
  Sphere *local_spheres_ = nullptr;
  Sphere *world_spheres_ = nullptr;
  SpherePair *pairs_ = nullptr;
  SphereTransform *transforms_ = nullptr;
  unsigned char *collisions_ = nullptr;
  int sphere_count_ = 0;
  int pair_count_ = 0;
  int body_count_ = 0;
};
} // namespace roscon_collision
#endif
