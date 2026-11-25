#include "roscon_planner/planner.hpp"
#include <fstream>
#include <gtest/gtest.h>
#include <limits>

static std::string read(const std::string &file) {
  std::ifstream input(file);
  if (!input) {
    throw std::runtime_error("Cannot read test fixture " + file);
  }
  return std::string(std::istreambuf_iterator<char>(input), std::istreambuf_iterator<char>());
}
TEST(PathValidation, RejectsAnInteriorCollisionForEveryBackend) {
  roscon_collision::SceneChecker scene(read(SWEEP_URDF), SWEEP_ENVIRONMENT, 0.06);
  roscon_planner::Path path = {Eigen::VectorXd::Constant(1, -1.0),
                               Eigen::VectorXd::Constant(1, 1.0)};
  for (const std::string backend : {"fcl", "spheres", "sdf"}) {
    ASSERT_TRUE(scene.check({"angle"}, {-1.0}, backend).collision_free());
    ASSERT_TRUE(scene.check({"angle"}, {1.0}, backend).collision_free());
    EXPECT_GT(scene.clearance({"angle"}, {1.0}, backend), 0.0);
    EXPECT_LE(scene.clearance({"angle"}, {0.0}, backend), 0.0);
    const auto result = roscon_planner::validatePath(
        path, 0.03, [&](const Eigen::VectorXd &q, std::string &reason) {
          const auto report = scene.check({"angle"}, {q[0]}, backend);
          if (!report.collision_free()) {
            reason = report.pairs.front();
          }
          return report.collision_free();
        });
    EXPECT_FALSE(result.valid) << backend;
    EXPECT_GT(result.checked_states, 2u);
    EXPECT_NE(result.message.find("ENVIRONMENT"), std::string::npos);
  }
  EXPECT_THROW(scene.check({"angle"}, {0.0}, "unsupported"), std::runtime_error);
}
TEST(PathValidation, RejectsMalformedPathsAndSamplesAllEdges) {
  const auto valid = [](const Eigen::VectorXd &, std::string &) { return true; };
  EXPECT_FALSE(roscon_planner::validatePath({}, 0.03, valid).valid);
  roscon_planner::Path path = {Eigen::VectorXd::Zero(2), Eigen::VectorXd::Ones(2),
                               Eigen::VectorXd::Zero(2)};
  EXPECT_FALSE(roscon_planner::validatePath(path, 0.0, valid).valid);
  EXPECT_GT(roscon_planner::validatePath(path, 0.03, valid).checked_states, 60u);
  path.back()[0] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(roscon_planner::validatePath(path, 0.03, valid).valid);
}
TEST(AbbPlanner, PlansRefinesAndIndependentlyValidates) {
  roscon_kinematics::Settings ik;
  roscon_planner::PlanSettings settings;
  settings.stomp_iterations = 5;
  settings.stomp_rollouts = 4;
  roscon_planner::Planner planner(read(ABB_URDF), TEST_ENVIRONMENT, "base_link", "flange", ik,
                                  settings);
  Eigen::VectorXd start(6);
  start << 0.0, -1.0, 1.2, -1.0, 1.0, 0.0;
  auto goal = start;
  goal[0] += 0.9;
  goal[1] += 0.1;
  const auto target = planner.robot().pose(goal);
  roscon_collision::SceneChecker distances(read(ABB_URDF), TEST_ENVIRONMENT, 0.06);
  for (const std::string backend : {"fcl", "spheres", "sdf"}) {
    ASSERT_TRUE(planner.validate({start, goal}, backend).valid);
    const auto result = planner.plan(start, target, backend, "rrt_connect", "stomp");
    ASSERT_TRUE(result.success) << backend << ": " << result.message;
    ASSERT_GE(result.path.size(), 2u);
    EXPECT_TRUE(result.path.front().isApprox(start, 1e-8));
    const auto end = planner.robot().pose(result.path.back());
    EXPECT_LT((end.position - target.position).norm(), ik.position_tolerance);
    EXPECT_LT(Eigen::AngleAxisd(target.rotation.transpose() * end.rotation).angle(),
              ik.orientation_tolerance);
    EXPECT_TRUE(planner.validate(result.path, backend).valid);
    auto refinement_settings = settings;
    refinement_settings.chomp_iterations = 1;
    const auto covariant = roscon_planner::refineWithChomp(
        result.path,
        [&](const Eigen::VectorXd &q, std::string &why) {
          return planner.validState(q, backend, why);
        },
        [&](const Eigen::VectorXd &q) {
          return distances.clearance(planner.robot().names,
                                     std::vector<double>(q.data(), q.data() + q.size()), backend);
        },
        refinement_settings);
    EXPECT_TRUE(planner.validate(covariant, backend).valid);
    EXPECT_TRUE(covariant.front().isApprox(result.path.front()));
    EXPECT_TRUE(covariant.back().isApprox(result.path.back()));
    std::cout << backend << ": " << result.message << '\n';
  }
  auto colliding_target = target;
  colliding_target.position = Eigen::Vector3d(0.65, -0.65, 1.0);
  const auto rejected = planner.plan(start, colliding_target, "sdf", "rrt_connect", "stomp");
  EXPECT_FALSE(rejected.success);
  EXPECT_TRUE(rejected.path.empty());
  auto invalid = start;
  invalid[0] = planner.robot().upper[0] + 1.0;
  EXPECT_FALSE(planner.validate({start, invalid}, "fcl").valid);
}

TEST(PlannerVariants, EveryOmplAndOptimizerCombination) {
  roscon_kinematics::Settings ik;
  roscon_planner::PlanSettings settings;
  settings.planning_seconds = 1.0;
  settings.stomp_iterations = 2;
  settings.stomp_rollouts = 2;
  settings.chomp_iterations = 1;
  roscon_planner::Planner planner(read(ABB_URDF), TEST_ENVIRONMENT, "base_link", "flange", ik,
                                  settings);
  Eigen::VectorXd start(6);
  start << 0.0, -1.0, 1.2, -1.0, 1.0, 0.0;
  auto goal = start;
  goal[0] += 0.35;
  goal[1] += 0.1;
  const auto target = planner.robot().pose(goal);
  for (const std::string algorithm : {"rrt_connect", "rrt_star", "prm"}) {
    for (const std::string optimizer : {"none", "stomp", "chomp"}) {
      const auto result = planner.plan(start, target, "fcl", algorithm, optimizer);
      ASSERT_TRUE(result.success) << algorithm << " / " << optimizer << ": " << result.message;
      EXPECT_TRUE(planner.validate(result.path, "fcl").valid);
      EXPECT_TRUE(result.path.front().isApprox(start, 1e-8));
    }
  }
  EXPECT_FALSE(planner.plan(start, target, "fcl", "unknown", "stomp").success);
  EXPECT_FALSE(planner.plan(start, target, "fcl", "rrt_connect", "unknown").success);
}

TEST(PlannerVariants, CovariantRefinementSmoothsWithoutMovingEndpoints) {
  roscon_planner::Path path;
  for (int i = 0; i < 12; ++i) {
    Eigen::VectorXd q(1);
    q[0] = 0.1 * i + (i > 0 && i < 11 ? (i % 2 ? 0.15 : -0.15) : 0.0);
    path.push_back(q);
  }
  const auto valid = [](const Eigen::VectorXd &, std::string &) { return true; };
  const auto clearance = [](const Eigen::VectorXd &) { return 1.0; };
  roscon_planner::PlanSettings settings;
  const auto refined = roscon_planner::refineWithChomp(path, valid, clearance, settings);
  auto roughness = [](const roscon_planner::Path &p) {
    double value = 0.0;
    for (size_t i = 1; i + 1 < p.size(); ++i) {
      value += (p[i - 1] - 2 * p[i] + p[i + 1]).squaredNorm();
    }
    return value;
  };
  EXPECT_LT(roughness(refined), roughness(path));
  EXPECT_TRUE(refined.front().isApprox(path.front()));
  EXPECT_TRUE(refined.back().isApprox(path.back()));
}
