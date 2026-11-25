#include <ament_index_cpp/get_package_share_directory.hpp>
#include <cmath>
#include <fstream>
#include <iostream>
#include <roscon_planner/planner.hpp>
#include <yaml-cpp/yaml.h>

Eigen::VectorXd readJoints(const YAML::Node &entry) {
  const auto values = entry.as<std::vector<double>>();
  if (values.size() != 6) {
    throw std::runtime_error("Each ABB preset must contain six joint positions");
  }
  Eigen::VectorXd joints(6);
  for (size_t index = 0; index < values.size(); ++index) {
    joints[index] = values[index];
    if (!std::isfinite(values[index])) {
      throw std::runtime_error("Joint positions must be finite");
    }
  }
  return joints;
}

int main(int argc, char **argv) {
  try {
    const std::string config =
        argc > 1 ? argv[1]
                 : ament_index_cpp::get_package_share_directory("roscon_gearbox_demo") + "/config";
    const auto robot_file =
        ament_index_cpp::get_package_share_directory("roscon_description") + "/urdf/irb2600.urdf";
    std::ifstream input(robot_file);
    if (!input) {
      throw std::runtime_error("Cannot read " + robot_file);
    }
    const std::string xml((std::istreambuf_iterator<char>(input)),
                          std::istreambuf_iterator<char>());
    const auto presets = YAML::LoadFile(config + "/targets.yaml");
    const Eigen::VectorXd home = readJoints(presets["home"]);
    roscon_kinematics::Settings ik;
    roscon_planner::PlanSettings settings;
    roscon_planner::Planner planner(xml, config + "/environment.yaml", "base_link", "flange", ik,
                                    settings);
    roscon_collision::SceneChecker scene(xml, config + "/environment.yaml", 0.06);
    for (const std::string backend : {"fcl", "spheres", "sdf"}) {
      Eigen::VectorXd current = home;
      for (const auto &entry : presets["targets"]) {
        const std::string name = entry.first.as<std::string>();
        const auto joints = readJoints(entry.second);
        const auto target = planner.robot().pose(joints);
        std::string reason;
        if (!planner.validState(joints, backend, reason)) {
          throw std::runtime_error(backend + " preset " + name + ": " + reason);
        }
        const auto plan = planner.plan(current, target, backend, "rrt_connect", "stomp");
        if (!plan.success) {
          throw std::runtime_error(backend + " plan " + name + ": " + plan.message);
        }
        const auto validation = planner.validate(plan.path, backend);
        if (!validation.valid) {
          throw std::runtime_error(backend + " path " + name + ": " + validation.message);
        }
        const auto reached = planner.robot().pose(plan.path.back());
        if ((reached.position - target.position).norm() > ik.position_tolerance ||
            Eigen::AngleAxisd(target.rotation.transpose() * reached.rotation).angle() >
                ik.orientation_tolerance) {
          throw std::runtime_error("Cartesian endpoint tolerance failed");
        }
        current = plan.path.back();
        std::cout << backend << " / " << name << ": PASS (" << validation.checked_states
                  << " checked states), target " << target.position.transpose() << std::endl;
      }
      // A deliberately lowered arm enters the workbench/housing. Check the
      // collision report itself, so an IK failure cannot masquerade as this test.
      Eigen::VectorXd blocked(6);
      blocked << 0.8, 0.3, 0.5, -1.0, 1.0, 0.0;
      const auto collision = scene.check(
          planner.robot().names,
          std::vector<double>(blocked.data(), blocked.data() + blocked.size()), backend);
      if (!collision.environment_collision) {
        throw std::runtime_error(backend + " expected fixture collision was not detected");
      }
      const auto invalid = planner.validate({home, blocked}, backend);
      if (invalid.valid) {
        throw std::runtime_error(backend + " accepted a path into the fixture");
      }
      std::cout << backend << " / blocked path: PASS " << invalid.message << std::endl;
    }
    return 0;
  } catch (const std::exception &error) {
    std::cerr << "Gearbox validation failed: " << error.what() << std::endl;
    return 1;
  }
}
