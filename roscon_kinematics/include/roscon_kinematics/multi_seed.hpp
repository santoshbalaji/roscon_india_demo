#pragma once
#include "roscon_kinematics/solver.hpp"

namespace roscon_kinematics {
// Seed 0 is the current configuration; the remaining 16 are fixed interior
// fractions of each joint's allowed range. No random generator or user input.
std::vector<Eigen::VectorXd> predefined_seeds(const Robot &robot, const Eigen::VectorXd &current);
// Revolute angles separated by full turns represent the same geometric posture.
bool same_configuration(const Eigen::VectorXd &left, const Eigen::VectorXd &right,
                        double tolerance = 0.05);
} // namespace roscon_kinematics
