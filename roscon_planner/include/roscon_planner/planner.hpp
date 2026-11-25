#pragma once
#include <functional>
#include <mutex>
#include <roscon_collision/scene_checker.hpp>
#include <roscon_kinematics/solver.hpp>

namespace roscon_planner {
using Path = std::vector<Eigen::VectorXd>;
using StateCheck = std::function<bool(const Eigen::VectorXd &, std::string &)>;
struct Validation {
  bool valid = false;
  size_t checked_states = 0;
  std::string message;
};
// Subdivide every edge, including its endpoints. No joint-angle wrapping: these
// are bounded physical joints, so a full turn is a real motion.
Validation validatePath(const Path &path, double joint_step, const StateCheck &check);
struct PlanSettings {
  double planning_seconds = 3.0;
  double joint_step = 0.03;
  int stomp_iterations = 12;
  int stomp_rollouts = 6;
  double noise_stddev = 0.04;
  int chomp_iterations = 5;
  double clearance_margin = 0.03;
};
struct PlanResult {
  bool success = false;
  bool refined = false;
  Path path;
  std::string message;
  size_t checked_states = 0;
};
class Planner {
public:
  Planner(const std::string &xml, const std::string &environment, const std::string &base,
          const std::string &tip, const roscon_kinematics::Settings &ik_settings,
          const PlanSettings &settings);
  bool validState(const Eigen::VectorXd &q, const std::string &backend, std::string &reason);
  Validation validate(const Path &path, const std::string &backend);
  PlanResult plan(const Eigen::VectorXd &start, const roscon_kinematics::Target &target,
                  const std::string &backend, const std::string &algorithm = "rrt_connect",
                  const std::string &optimizer = "stomp");
  const roscon_kinematics::Robot &robot() const {
    return robot_;
  }
  visualization_msgs::msg::MarkerArray environmentMarkers() const {
    return scene_.environmentMarkers();
  }
  static bool knownAlgorithm(const std::string &algorithm);
  static bool knownOptimizer(const std::string &optimizer);
  static bool knownBackend(const std::string &backend);

private:
  roscon_kinematics::Settings ik_settings_;
  PlanSettings settings_;
  roscon_kinematics::Robot robot_;
  roscon_collision::SceneChecker scene_;
  std::mutex collision_mutex_;
};
Path refineWithChomp(const Path &initial, const StateCheck &check,
                     const std::function<double(const Eigen::VectorXd &)> &clearance,
                     const PlanSettings &settings);
Path refineWithStomp(const Path &initial, const StateCheck &check, const PlanSettings &settings);
} // namespace roscon_planner
