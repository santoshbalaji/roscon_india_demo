#include "roscon_collision/gpu_sdf.hpp"
#include <stdexcept>
namespace roscon_collision
{
GpuSdf::~GpuSdf() = default;
void GpuSdf::initialize(const std::vector<SdfObject> &, const std::vector<SdfTriangle> &, int)
{
  throw std::runtime_error("CUDA SDF support was not built. Rebuild with ROSCON_ENABLE_CUDA=ON.");
}
SdfResults GpuSdf::check(const std::vector<Sphere> &)
{
  throw std::runtime_error("CUDA SDF support was not built.");
}
SdfResults checkSdfOnCpu(const std::vector<Sphere> &, const std::vector<SdfObject> &, const std::vector<SdfTriangle> &)
{
  throw std::runtime_error("The SDF evaluator requires a CUDA-enabled build.");
}
}
