#include <gtest/gtest.h>
#include <cmath>
#include <random>
#include "roscon_collision/gpu_spheres.hpp"
#include "roscon_collision/sphere_model.hpp"

using roscon_collision::Sphere;
using roscon_collision::SpherePair;
using roscon_collision::SphereResults;
using roscon_collision::SphereTransform;

TEST(SphereModel, CoversBoxAndPreservesSphere)
{
  fcl::Boxd box(0.6, 0.8, 0.5);
  const auto spheres = roscon_collision::makeSphereModel(box, 0.1, 0);
  for (double x : {-0.3, 0.0, 0.3}) {
    for (double y : {-0.4, 0.0, 0.4}) {
      for (double z : {-0.25, 0.0, 0.25}) {
        bool covered = false;
        for (const Sphere & sphere : spheres) {
          const double distance = std::hypot(x - sphere.x, y - sphere.y, z - sphere.z);
          if (distance <= sphere.radius + 1e-12) {
            covered = true;
          }
          EXPECT_LE(sphere.radius, 0.1 + 1e-12);
        }
        EXPECT_TRUE(covered);
      }
    }
  }
  fcl::Sphered sphere(0.25);
  const auto exact = roscon_collision::makeSphereModel(sphere, 0.1, 4);
  ASSERT_EQ(exact.size(), 1u);
  EXPECT_DOUBLE_EQ(exact[0].radius, 0.25);
  EXPECT_EQ(exact[0].body, 4);
  EXPECT_THROW(roscon_collision::makeSphereModel(box, 0.0, 0), std::runtime_error);
}

TEST(GpuSpheres, MatchesReferenceAcrossPosesAndBoundaries)
{
  try {
    roscon_collision::GpuSpheres::deviceName();
  } catch (const std::exception & exception) {
    GTEST_SKIP() << exception.what();
  }
  // Tangency, overlap, separation, and containment, followed by random pairs.
  std::vector<Sphere> spheres = {
    {0, 0, 0, 1, 0}, {2, 0, 0, 1, 0}, {1.9, 0, 0, 1, 0},
    {2.1, 0, 0, 1, 0}, {0, 0, 0, 0.2, 0}};
  std::mt19937 generator(42);
  std::uniform_real_distribution<double> position(-3.0, 3.0);
  for (int index = 0; index < 80; ++index) {
    spheres.push_back({position(generator), position(generator), position(generator), 0.25, index % 2});
  }
  std::vector<SpherePair> pairs;
  for (int first = 0; first < static_cast<int>(spheres.size()); ++first) {
    for (int second = first + 1; second < static_cast<int>(spheres.size()); ++second) {
      pairs.push_back({first, second});
    }
  }
  roscon_collision::GpuSpheres gpu;
  gpu.initialize(spheres, pairs, 2);
  for (int pose = 0; pose < 10; ++pose) {
    fcl::Transform3d first = fcl::Transform3d::Identity();
    fcl::Transform3d second = fcl::Transform3d::Identity();
    second.translation() = Eigen::Vector3d(pose * 0.1, -0.5, 0.25);
    second.linear() = Eigen::AngleAxisd(pose * 0.2, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    const std::vector<SphereTransform> transforms = {
      roscon_collision::makeSphereTransform(first), roscon_collision::makeSphereTransform(second)};
    const SphereResults actual = gpu.check(transforms);
    const SphereResults expected = roscon_collision::checkSpheresOnCpu(spheres, pairs, transforms);
    EXPECT_EQ(actual.collisions, expected.collisions);
    EXPECT_EQ(actual.collisions[0], 1);  // Tangency.
    EXPECT_EQ(actual.collisions[1], 1);  // Overlap.
    EXPECT_EQ(actual.collisions[2], 0);  // Separation.
    EXPECT_EQ(actual.collisions[3], 1);  // Containment.
    for (std::size_t index = 0; index < spheres.size(); ++index) {
      EXPECT_NEAR(actual.world_spheres[index].x, expected.world_spheres[index].x, 1e-12);
      EXPECT_NEAR(actual.world_spheres[index].y, expected.world_spheres[index].y, 1e-12);
      EXPECT_NEAR(actual.world_spheres[index].z, expected.world_spheres[index].z, 1e-12);
    }
  }
  // Zero candidate pairs must still transform spheres without an invalid kernel launch.
  gpu.initialize({spheres[0]}, {}, 1);
  EXPECT_TRUE(gpu.check({roscon_collision::makeSphereTransform(fcl::Transform3d::Identity())}).collisions.empty());
}
