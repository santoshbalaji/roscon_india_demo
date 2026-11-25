#include "roscon_collision/sphere_model.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace roscon_collision
{
std::vector<Sphere> makeSphereModel(fcl::CollisionGeometryd & geometry, double radius, int body)
{
  if (!std::isfinite(radius) || radius <= 0.0) {
    throw std::runtime_error("sphere_radius must be finite and positive");
  }
  const auto * sphere_geometry = dynamic_cast<const fcl::Sphered *>(&geometry);
  if (sphere_geometry != nullptr) {
    return {{0.0, 0.0, 0.0, sphere_geometry->radius, body}};
  }
  geometry.computeLocalAABB();
  const Eigen::Vector3d minimum = geometry.aabb_local.min_;
  const Eigen::Vector3d size = geometry.aabb_local.max_ - minimum;
  if (!minimum.allFinite() || !size.allFinite() || (size.array() < 0.0).any()) {
    throw std::runtime_error("Cannot create spheres for invalid geometry bounds");
  }
  // Each cell fits completely inside its sphere. The union covers the full bounding box.
  const double cell_limit = 2.0 * radius / std::sqrt(3.0);
  int counts[3];
  for (int axis = 0; axis < 3; ++axis) {
    const double count = std::max(1.0, std::ceil(size[axis] / cell_limit));
    if (count > 10000.0) {
      throw std::runtime_error("Too many spheres. Increase sphere_radius.");
    }
    counts[axis] = static_cast<int>(count);
  }
  const std::size_t total = static_cast<std::size_t>(counts[0]) * counts[1] * counts[2];
  if (total > 10000) {
    throw std::runtime_error("Too many spheres for one shape. Increase sphere_radius.");
  }
  const Eigen::Vector3d cell(size.x() / counts[0], size.y() / counts[1], size.z() / counts[2]);
  const double cell_radius = std::max(1e-9, 0.5 * cell.norm());
  std::vector<Sphere> spheres;
  for (int x = 0; x < counts[0]; ++x) {
    for (int y = 0; y < counts[1]; ++y) {
      for (int z = 0; z < counts[2]; ++z) {
        spheres.push_back({minimum.x() + (x + 0.5) * cell.x(),
          minimum.y() + (y + 0.5) * cell.y(), minimum.z() + (z + 0.5) * cell.z(),
          cell_radius, body});
      }
    }
  }
  return spheres;
}

SphereTransform makeSphereTransform(const fcl::Transform3d & transform)
{
  SphereTransform result;
  for (int row = 0; row < 3; ++row) {
    for (int column = 0; column < 4; ++column) {
      result.values[row * 4 + column] = transform.matrix()(row, column);
    }
  }
  return result;
}
}  // namespace roscon_collision
