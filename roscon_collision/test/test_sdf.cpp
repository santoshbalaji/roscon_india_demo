#include <gtest/gtest.h>
#include <cmath>
#include "roscon_collision/gpu_sdf.hpp"
#include "roscon_collision/gpu_spheres.hpp"
#include "roscon_collision/sdf_model.hpp"

static std::shared_ptr<fcl::BVHModel<fcl::OBBRSSd>> makeCube(bool open)
{
  const std::vector<fcl::Vector3d> vertices = {
    {-1, -1, -1}, {1, -1, -1}, {1, 1, -1}, {-1, 1, -1},
    {-1, -1, 1}, {1, -1, 1}, {1, 1, 1}, {-1, 1, 1}};
  std::vector<fcl::Triangle> faces = {
    {0, 2, 1}, {0, 3, 2}, {4, 5, 6}, {4, 6, 7}, {0, 1, 5}, {0, 5, 4},
    {1, 2, 6}, {1, 6, 5}, {2, 3, 7}, {2, 7, 6}, {3, 0, 4}, {3, 4, 7}};
  if (open) {
    faces.pop_back();
  }
  auto mesh = std::make_shared<fcl::BVHModel<fcl::OBBRSSd>>();
  mesh->beginModel();
  mesh->addSubModel(vertices, faces);
  mesh->endModel();
  return mesh;
}

TEST(SdfModel, RejectsOpenMeshes)
{
  std::vector<roscon_collision::SdfTriangle> triangles;
  EXPECT_THROW(roscon_collision::makeSdfObject(
    *makeCube(true), fcl::Transform3d::Identity(), triangles), std::runtime_error);
  EXPECT_TRUE(triangles.empty());
  const auto object = roscon_collision::makeSdfObject(
    *makeCube(false), fcl::Transform3d::Identity(), triangles);
  EXPECT_EQ(object.triangle_count, 12);
  EXPECT_EQ(triangles.size(), 12u);
}

TEST(GpuSdf, SignedDistancesTangencyContainmentAndMesh)
{
  try {
    roscon_collision::GpuSpheres::deviceName();
  } catch (const std::exception & exception) {
    GTEST_SKIP() << exception.what();
  }
  std::vector<roscon_collision::SdfTriangle> triangles;
  fcl::Boxd box(2, 2, 2);
  fcl::Sphered sphere(1);
  fcl::Cylinderd cylinder(1, 2);
  const auto pose = fcl::Transform3d::Identity();
  const std::vector<roscon_collision::SdfObject> objects = {
    roscon_collision::makeSdfObject(box, pose, triangles),
    roscon_collision::makeSdfObject(sphere, pose, triangles),
    roscon_collision::makeSdfObject(cylinder, pose, triangles),
    roscon_collision::makeSdfObject(*makeCube(false), pose, triangles)};
  const std::vector<roscon_collision::Sphere> probes = {
    {0, 0, 0, 0.25, 0}, {1.25, 0, 0, 0.25, 0}, {1.26, 0, 0, 0.25, 0},
    {1.2, 1.2, 0, 0.25, 0}, {0, 0, 1.25, 0.25, 0}, {1.2, 0, 1.2, 0.25, 0},
    {1, 0, 0, 0.1, 0}, {1, 1, 1, 0.1, 0}};
  roscon_collision::GpuSdf gpu;
  gpu.initialize(objects, triangles, probes.size());
  const auto actual = gpu.check(probes);
  const auto reference = roscon_collision::checkSdfOnCpu(probes, objects, triangles);
  EXPECT_EQ(actual.collisions, reference.collisions);
  for (std::size_t index = 0; index < actual.distances.size(); ++index) {
    EXPECT_NEAR(actual.distances[index], reference.distances[index], 1e-10);
  }
  for (int object = 0; object < 4; ++object) {
    EXPECT_NEAR(actual.distances[object], -1.0, 1e-12);
    EXPECT_EQ(actual.collisions[object], 1);  // Fully inside the environment solid.
    EXPECT_NEAR(actual.distances[4 + object], 0.25, 1e-12);
    EXPECT_EQ(actual.collisions[4 + object], 1);  // Tangency.
    EXPECT_EQ(actual.collisions[8 + object], 0);  // Separated.
  }
  EXPECT_NEAR(actual.distances[12], std::sqrt(0.08), 1e-12);  // Box corner.
  EXPECT_NEAR(actual.distances[22], std::sqrt(0.08), 1e-12);  // Cylinder rim.
  for (std::size_t probe = 0; probe < probes.size(); ++probe) {
    EXPECT_NEAR(actual.distances[probe * 4], actual.distances[probe * 4 + 3], 1e-12);
  }
  gpu.initialize({}, {}, probes.size());
  EXPECT_TRUE(gpu.check(probes).collisions.empty());
}

TEST(GpuSdf, RotatedTranslatedEnvironment)
{
  try {
    roscon_collision::GpuSpheres::deviceName();
  } catch (const std::exception & exception) {
    GTEST_SKIP() << exception.what();
  }
  fcl::Boxd box(2, 4, 6);
  fcl::Transform3d pose = fcl::Transform3d::Identity();
  pose.translation() = Eigen::Vector3d(3, -2, 1);
  pose.linear() = Eigen::AngleAxisd(1.5707963267948966, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  std::vector<roscon_collision::SdfTriangle> triangles;
  const auto object = roscon_collision::makeSdfObject(box, pose, triangles);
  roscon_collision::GpuSdf gpu;
  gpu.initialize({object}, {}, 2);
  const auto result = gpu.check({{3, -0.75, 1, 0.25, 0}, {3, -0.5, 1, 0.25, 0}});
  EXPECT_NEAR(result.distances[0], 0.25, 1e-12);
  EXPECT_EQ(result.collisions[0], 1);
  EXPECT_NEAR(result.distances[1], 0.5, 1e-12);
  EXPECT_EQ(result.collisions[1], 0);
}

TEST(SdfCpu, SignedDistanceWorksWithoutCuda) {
  std::vector<roscon_collision::SdfTriangle> triangles;
  fcl::Sphered shape(1.0);
  const auto object = roscon_collision::makeSdfObject(
      shape, fcl::Transform3d::Identity(), triangles);
  const std::vector<roscon_collision::Sphere> probes = {
      {0.0, 0.0, 0.0, 0.25, 0},
      {1.25, 0.0, 0.0, 0.25, 0},
      {1.30, 0.0, 0.0, 0.25, 0}};
  const auto result = roscon_collision::checkSdfOnCpu(probes, {object}, triangles);
  ASSERT_EQ(result.distances.size(), 3u);
  EXPECT_DOUBLE_EQ(result.distances[0], -1.0);
  EXPECT_DOUBLE_EQ(result.distances[1], 0.25);
  EXPECT_NEAR(result.distances[2], 0.30, 1e-12);
  EXPECT_TRUE(result.collisions[0]);
  EXPECT_TRUE(result.collisions[1]);
  EXPECT_FALSE(result.collisions[2]);
}
