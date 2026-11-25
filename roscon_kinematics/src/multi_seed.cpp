#include "roscon_kinematics/multi_seed.hpp"
#include <array>
#include <cmath>
#include <stdexcept>

namespace roscon_kinematics {
std::vector<Eigen::VectorXd> predefined_seeds(const Robot &robot, const Eigen::VectorXd &current) {
  if (current.size() != robot.lower.size() || !current.allFinite()) {
    throw std::invalid_argument("Seed dimension does not match robot");
  }
  // Shoulder, elbow, and wrist combinations spread across the joint ranges.
  // Fractions avoid starting exactly at a joint bound.
  const std::array<std::array<double, 6>, 16> fractions = {{{{.50, .35, .70, .50, .70, .50}},
                                                            {{.50, .65, .30, .50, .30, .50}},
                                                            {{.25, .35, .70, .25, .70, .75}},
                                                            {{.75, .35, .70, .75, .30, .25}},
                                                            {{.25, .65, .30, .75, .30, .75}},
                                                            {{.75, .65, .30, .25, .70, .25}},
                                                            {{.50, .20, .85, .75, .25, .50}},
                                                            {{.50, .80, .15, .25, .75, .50}},
                                                            {{.15, .45, .60, .60, .80, .35}},
                                                            {{.85, .55, .40, .40, .20, .65}},
                                                            {{.35, .25, .45, .85, .60, .15}},
                                                            {{.65, .75, .55, .15, .40, .85}},
                                                            {{.20, .70, .80, .35, .35, .60}},
                                                            {{.80, .30, .20, .65, .65, .40}},
                                                            {{.40, .60, .25, .20, .85, .70}},
                                                            {{.60, .40, .75, .80, .15, .30}}}};
  std::vector<Eigen::VectorXd> seeds{current};
  for (const auto &fraction : fractions) {
    Eigen::VectorXd seed(current.size());
    for (Eigen::Index joint = 0; joint < seed.size(); ++joint) {
      seed[joint] =
          robot.lower[joint] + fraction[joint % 6] * (robot.upper[joint] - robot.lower[joint]);
    }
    seeds.push_back(seed);
  }
  return seeds;
}
bool same_configuration(const Eigen::VectorXd &left, const Eigen::VectorXd &right,
                        double tolerance) {
  if (left.size() != right.size() || !left.allFinite() || !right.allFinite()) {
    return false;
  }
  constexpr double full_turn = 6.283185307179586;
  for (Eigen::Index joint = 0; joint < left.size(); ++joint) {
    if (std::abs(std::remainder(left[joint] - right[joint], full_turn)) > tolerance) {
      return false;
    }
  }
  return true;
}
} // namespace roscon_kinematics
