#include "roscon_planner/planner.hpp"
#include <Eigen/SVD>
#include <algorithm>
#include <cmath>
#include <ompl/base/goals/GoalStates.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/prm/PRM.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <roscon_kinematics/multi_seed.hpp>
#include <sstream>
#include <stdexcept>

namespace roscon_planner {
Validation validatePath(const Path &path, double joint_step, const StateCheck &check) {
  Validation result;
  if (path.size() < 2 || path.size() > 10000 || !std::isfinite(joint_step) || joint_step <= 0.0) {
    result.message = "Need 2..10000 path points and a positive finite validation step";
    return result;
  }
  const auto dimensions = path.front().size();
  for (const auto &q : path) {
    if (dimensions == 0 || q.size() != dimensions || !q.allFinite()) {
      result.message = "Path contains inconsistent or nonfinite joint positions";
      return result;
    }
  }
  for (size_t edge = 1; edge < path.size(); ++edge) {
    const double delta = (path[edge] - path[edge - 1]).cwiseAbs().maxCoeff();
    const double required_steps = std::max(1.0, std::ceil(delta / joint_step));
    if (required_steps > 100000 || result.checked_states + required_steps > 1000000) {
      result.message = "Path exceeds validation sample limit";
      return result;
    }
    const size_t steps = static_cast<size_t>(required_steps);
    for (size_t i = (edge == 1 ? 0 : 1); i <= steps; ++i) {
      const double fraction = static_cast<double>(i) / steps;
      const Eigen::VectorXd q = (1.0 - fraction) * path[edge - 1] + fraction * path[edge];
      std::string reason;
      ++result.checked_states;
      if (!check(q, reason)) {
        result.message =
            "Edge " + std::to_string(edge - 1) + ", sample " + std::to_string(i) + ": " + reason;
        return result;
      }
    }
  }
  result.valid = true;
  result.message = "Valid: " + std::to_string(result.checked_states) + " sampled states";
  return result;
}

Planner::Planner(const std::string &xml, const std::string &environment, const std::string &base,
                 const std::string &tip, const roscon_kinematics::Settings &ik_settings,
                 const PlanSettings &settings)
    : ik_settings_(ik_settings), settings_(settings),
      robot_(xml, base, tip, ik_settings.joint_margin), scene_(xml, environment, 0.06) {
  if (base != scene_.frame()) {
    throw std::runtime_error("Planner base must equal the URDF root/environment frame");
  }
  if (settings_.chomp_iterations < 1 || settings_.chomp_iterations > 30 ||
      !std::isfinite(settings_.clearance_margin) || settings_.clearance_margin <= 0.0 ||
      settings_.clearance_margin > 0.3 || !std::isfinite(settings_.joint_step) ||
      settings_.joint_step < 0.001 || settings_.joint_step > 0.1 ||
      !std::isfinite(settings_.planning_seconds) || settings_.planning_seconds <= 0.0 ||
      settings_.planning_seconds > 30.0 || settings_.stomp_iterations < 1 ||
      settings_.stomp_iterations > 100 || settings_.stomp_rollouts < 2 ||
      settings_.stomp_rollouts > 30 || !std::isfinite(settings_.noise_stddev) ||
      settings_.noise_stddev <= 0.0 || settings_.noise_stddev > 0.5) {
    throw std::runtime_error("Invalid planner settings");
  }
}
bool Planner::knownAlgorithm(const std::string &algorithm) {
  return algorithm == "rrt_connect" || algorithm == "rrt_star" || algorithm == "prm";
}
bool Planner::knownOptimizer(const std::string &optimizer) {
  return optimizer == "none" || optimizer == "stomp" || optimizer == "chomp";
}
bool Planner::knownBackend(const std::string &backend) {
  return backend == "fcl" || backend == "spheres" || backend == "sdf";
}
bool Planner::validState(const Eigen::VectorXd &q, const std::string &backend,
                         std::string &reason) {
  if (!knownBackend(backend)) {
    reason = "Unknown collision backend";
    return false;
  }
  if (q.size() != robot_.lower.size() || !q.allFinite()) {
    reason = "Invalid joint vector";
    return false;
  }
  if ((q.array() < robot_.lower.array()).any() || (q.array() > robot_.upper.array()).any()) {
    reason = "Joint limit exceeded";
    return false;
  }
  const double sigma = robot_.jacobian(q, ik_settings_.characteristic_length)
                           .jacobiSvd()
                           .singularValues()
                           .minCoeff();
  if (!std::isfinite(sigma) || sigma < ik_settings_.minimum_singular_value) {
    reason = "Singularity margin violated";
    return false;
  }
  const std::vector<double> positions(q.data(), q.data() + q.size());
  // OMPL validity callbacks must be thread-safe. The scene keeps temporary FK
  // transforms, so serialize access even when a planner uses internal threads.
  const std::lock_guard<std::mutex> lock(collision_mutex_);
  const auto collision = scene_.check(robot_.names, positions, backend);
  if (!collision.collision_free()) {
    reason = collision.pairs.empty() ? "Collision" : collision.pairs.front();
    return false;
  }
  return true;
}
Validation Planner::validate(const Path &path, const std::string &backend) {
  return validatePath(path, settings_.joint_step,
                      [&](const Eigen::VectorXd &q, std::string &reason) {
                        return validState(q, backend, reason);
                      });
}
PlanResult Planner::plan(const Eigen::VectorXd &start, const roscon_kinematics::Target &target,
                         const std::string &backend, const std::string &algorithm,
                         const std::string &optimizer) {
  PlanResult result;
  if (!knownAlgorithm(algorithm) || !knownOptimizer(optimizer)) {
    result.message = "Unknown OMPL planner or optimizer";
    return result;
  }
  const bool refine = optimizer != "none";
  std::string reason;
  if (!validState(start, backend, reason)) {
    result.message = "Invalid start: " + reason;
    return result;
  }
  if (!target.position.allFinite() || !target.rotation.allFinite() ||
      !(target.rotation.transpose() * target.rotation)
           .isApprox(Eigen::Matrix3d::Identity(), 1e-6) ||
      std::abs(target.rotation.determinant() - 1.0) > 1e-6) {
    result.message = "Invalid target pose";
    return result;
  }
  // Nearby seeds help retain the current wrist/elbow branch. Global seeds
  // alone can find only a disconnected branch even for a reachable target.
  Path seeds{start};
  for (double offset : {-0.5, 0.5, -1.0, 1.0}) {
    for (Eigen::Index joint = 0; joint < start.size(); ++joint) {
      Eigen::VectorXd seed = start;
      seed[joint] += offset;
      if (seed[joint] >= robot_.lower[joint] && seed[joint] <= robot_.upper[joint]) {
        seeds.push_back(seed);
      }
    }
  }
  const auto global_seeds = roscon_kinematics::predefined_seeds(robot_, start);
  seeds.insert(seeds.end(), global_seeds.begin() + 1, global_seeds.end());
  Path goals;
  for (const auto &seed : seeds) {
    const auto solution = roscon_kinematics::solve(robot_, target, seed, ik_settings_);
    if (!solution.accepted || !solution.target_reached ||
        !validState(solution.joints, backend, reason)) {
      continue;
    }
    bool duplicate = false;
    for (const auto &existing : goals) {
      if (roscon_kinematics::same_configuration(existing, solution.joints)) {
        duplicate = true;
        break;
      }
    }
    if (!duplicate) {
      goals.push_back(solution.joints);
    }
  }
  if (goals.empty()) {
    result.message = "No collision-free IK goal found for this pose";
    return result;
  }
  // Try nearby IK branches first. A wrist-flipped goal may be in a different
  // connected component when the singularity margin is enforced.
  std::sort(goals.begin(), goals.end(), [&](const Eigen::VectorXd &a, const Eigen::VectorXd &b) {
    return (a - start).squaredNorm() < (b - start).squaredNorm();
  });
  namespace ob = ompl::base;
  namespace og = ompl::geometric;
  auto space = std::make_shared<ob::RealVectorStateSpace>(start.size());
  ob::RealVectorBounds bounds(start.size());
  for (Eigen::Index i = 0; i < start.size(); ++i) {
    bounds.setLow(i, robot_.lower[i]);
    bounds.setHigh(i, robot_.upper[i]);
  }
  space->setBounds(bounds);
  og::SimpleSetup setup(space);
  setup.setStateValidityChecker([&](const ob::State *state) {
    const Eigen::Map<const Eigen::VectorXd> q(
        state->as<ob::RealVectorStateSpace::StateType>()->values, start.size());
    std::string state_reason;
    return validState(q, backend, state_reason);
  });
  // OMPL uses Euclidean edge length; this bound is at least as fine as the
  // per-joint bound used by our independent final validator.
  setup.getSpaceInformation()->setStateValidityCheckingResolution(settings_.joint_step /
                                                                  space->getMaximumExtent());
  ob::ScopedState<> start_state(space);
  for (Eigen::Index i = 0; i < start.size(); ++i) {
    start_state[i] = start[i];
  }
  setup.setStartState(start_state);
  auto goal_states = std::make_shared<ob::GoalStates>(setup.getSpaceInformation());
  goal_states->setThreshold(1e-8);
  for (const auto &q : goals) {
    ob::ScopedState<> goal(space);
    for (Eigen::Index i = 0; i < q.size(); ++i) {
      goal[i] = q[i];
    }
    goal_states->addState(goal);
  }
  setup.setGoal(goal_states);
  setup.getProblemDefinition()->setOptimizationObjective(
      std::make_shared<ob::PathLengthOptimizationObjective>(setup.getSpaceInformation()));
  if (algorithm == "rrt_connect") {
    auto search = std::make_shared<og::RRTConnect>(setup.getSpaceInformation());
    search->setRange(0.4);
    setup.setPlanner(search);
  } else if (algorithm == "rrt_star") {
    auto search = std::make_shared<og::RRTstar>(setup.getSpaceInformation());
    search->setRange(0.4);
    setup.setPlanner(search);
  } else {
    setup.setPlanner(std::make_shared<og::PRM>(setup.getSpaceInformation()));
  }
  setup.solve(settings_.planning_seconds);
  if (!setup.haveExactSolutionPath()) {
    result.message = "OMPL did not find an exact path within its time budget";
    return result;
  }
  setup.simplifySolution(0.15);
  auto &geometric = setup.getSolutionPath();
  geometric.interpolate(std::max<size_t>(40, geometric.getStateCount()));
  for (const auto *state : geometric.getStates()) {
    result.path.emplace_back(Eigen::Map<const Eigen::VectorXd>(
        state->as<ob::RealVectorStateSpace::StateType>()->values, start.size()));
  }
  auto validation = validate(result.path, backend);
  if (!validation.valid) {
    result.path.clear();
    result.message = "OMPL final validation failed: " + validation.message;
    return result;
  }
  std::string refinement = "OMPL only";
  if (refine && result.path.size() <= 200) {
    Path refined;
    std::string refinement_error;
    try {
      const StateCheck check = [&](const Eigen::VectorXd &q, std::string &why) {
        return validState(q, backend, why);
      };
      if (optimizer == "stomp") {
        refined = refineWithStomp(result.path, check, settings_);
      } else {
        refined = refineWithChomp(
            result.path, check,
            [&](const Eigen::VectorXd &q) {
              const std::lock_guard<std::mutex> lock(collision_mutex_);
              return scene_.clearance(robot_.names,
                                      std::vector<double>(q.data(), q.data() + q.size()), backend);
            },
            settings_);
      }
    } catch (const std::exception &error) {
      refinement_error = error.what();
    }
    if (!refined.empty() && refined.front().isApprox(start, 1e-8) &&
        refined.back().isApprox(result.path.back(), 1e-8)) {
      const auto refined_validation = validate(refined, backend);
      if (refined_validation.valid) {
        result.path = refined;
        validation = refined_validation;
        result.refined = true;
      }
    }
    refinement = result.refined
                     ? "OMPL + " + optimizer
                     : "OMPL retained: " + optimizer + " did not produce a valid refinement";
    if (!refinement_error.empty()) {
      refinement += ": " + refinement_error;
    }
  } else if (refine) {
    refinement = "OMPL retained: path exceeds optimizer point limit";
  }
  const auto achieved = robot_.pose(result.path.back());
  if ((achieved.position - target.position).norm() > ik_settings_.position_tolerance ||
      Eigen::AngleAxisd(target.rotation.transpose() * achieved.rotation).angle() >
          ik_settings_.orientation_tolerance) {
    result.path.clear();
    result.message = "Final endpoint failed pose tolerance";
    return result;
  }
  result.success = true;
  result.checked_states = validation.checked_states;
  result.message = algorithm + "; " + refinement + "; " + backend + "; " + validation.message;
  return result;
}
} // namespace roscon_planner
