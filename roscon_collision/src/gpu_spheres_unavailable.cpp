#include "roscon_collision/gpu_spheres.hpp"
#include <stdexcept>

namespace roscon_collision
{
GpuSpheres::~GpuSpheres() = default;
std::string GpuSpheres::deviceName()
{
  throw std::runtime_error("CUDA support was not built. Rebuild with ROSCON_ENABLE_CUDA=ON.");
}
void GpuSpheres::initialize(const std::vector<Sphere> &, const std::vector<SpherePair> &, int)
{
  deviceName();
}
SphereResults GpuSpheres::check(const std::vector<SphereTransform> &)
{
  deviceName();
  return {};
}
}  // namespace roscon_collision
