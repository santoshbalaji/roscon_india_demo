#include "roscon_collision/sdf_visualization.hpp"
#include "roscon_collision/geometry.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace roscon_collision {
visualization_msgs::msg::Marker makeSdfSlice(const SdfObject &object,
                                             const std::vector<SdfTriangle> &triangles,
                                             const fcl::AABBd &bounds, const fcl::Transform3d &pose,
                                             double cell_size, double padding) {
  if (!std::isfinite(cell_size) || cell_size <= 0.0 || !std::isfinite(padding) || padding <= 0.0) {
    throw std::runtime_error("SDF slice cell size and padding must be finite and positive");
  }
  const double width = bounds.max_.x() - bounds.min_.x() + 2.0 * padding;
  const double height = bounds.max_.z() - bounds.min_.z() + 2.0 * padding;
  const double columns = std::ceil(width / cell_size);
  const double rows = std::ceil(height / cell_size);
  if (columns * rows > 10000.0) {
    throw std::runtime_error("SDF slice exceeds 10000 samples. Increase sdf_cell_size.");
  }
  visualization_msgs::msg::Marker marker;
  marker.ns = "sdf_slice";
  marker.type = visualization_msgs::msg::Marker::CUBE_LIST;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose = transformToPose(pose);
  marker.scale.x = cell_size * 0.95;
  marker.scale.y = cell_size * 0.1;
  marker.scale.z = cell_size * 0.95;
  marker.color.a = 0.8;
  std::vector<Sphere> samples;
  for (int column = 0; column < static_cast<int>(columns); ++column) {
    for (int row = 0; row < static_cast<int>(rows); ++row) {
      geometry_msgs::msg::Point point;
      point.x = bounds.min_.x() - padding + (column + 0.5) * cell_size;
      point.y = (bounds.min_.y() + bounds.max_.y()) / 2.0;
      point.z = bounds.min_.z() - padding + (row + 0.5) * cell_size;
      marker.points.push_back(point);
      const Eigen::Vector3d world = pose * Eigen::Vector3d(point.x, point.y, point.z);
      samples.push_back({world.x(), world.y(), world.z(), 1e-9, 0});
    }
  }
  // Use the same CUDA distance evaluator as collision checking. Cache this static slice.
  GpuSdf sampler;
  sampler.initialize({object}, triangles, samples.size());
  const SdfResults result = sampler.check(samples);
  for (double distance : result.distances) {
    const double strength = std::min(std::abs(distance) / padding, 1.0);
    std_msgs::msg::ColorRGBA color;
    color.r = 1.0 - strength;
    color.g = 1.0 - strength;
    color.b = 1.0 - strength;
    color.a = 0.8;
    if (distance < 0.0) {
      color.r = 1.0; // Inside: red, fading to white at the surface.
    } else {
      color.b = 1.0; // Outside: blue, fading to white at the surface.
    }
    marker.colors.push_back(color);
  }
  return marker;
}
} // namespace roscon_collision
