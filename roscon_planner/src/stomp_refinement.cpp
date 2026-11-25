#include "roscon_planner/planner.hpp"
#include <Eigen/Cholesky>
#include <random>
#include <stomp/stomp.h>

namespace roscon_planner {
// The upstream STOMP library computes rollout probabilities, control costs,
// and updates. This task supplies correlated noise and state validity costs.
class CollisionTask : public stomp::Task {
public:
  CollisionTask(const StateCheck &check, int points, double noise_stddev)
      : check_(check), noise_stddev_(noise_stddev), random_(42) {
    Eigen::MatrixXd acceleration = Eigen::MatrixXd::Zero(points - 2, points - 2);
    for (int i = 0; i < points - 2; ++i) {
      acceleration(i, i) = 2.0;
      if (i > 0) {
        acceleration(i, i - 1) = -1.0;
      }
      if (i + 1 < points - 2) {
        acceleration(i, i + 1) = -1.0;
      }
    }
    Eigen::MatrixXd precision = acceleration.transpose() * acceleration;
    precision.diagonal().array() += 1e-6;
    Eigen::MatrixXd covariance =
        precision.ldlt().solve(Eigen::MatrixXd::Identity(points - 2, points - 2));
    covariance /= covariance.diagonal().maxCoeff();
    Eigen::LLT<Eigen::MatrixXd> factor(covariance);
    if (factor.info() != Eigen::Success) {
      throw std::runtime_error("Cannot construct STOMP noise covariance");
    }
    noise_factor_ = factor.matrixL();
  }
  bool generateNoisyParameters(const Eigen::MatrixXd &parameters, std::size_t, std::size_t, int,
                               int, Eigen::MatrixXd &parameters_noise,
                               Eigen::MatrixXd &noise) override {
    noise = Eigen::MatrixXd::Zero(parameters.rows(), parameters.cols());
    std::normal_distribution<double> normal(0.0, 1.0);
    for (Eigen::Index joint = 0; joint < parameters.rows(); ++joint) {
      Eigen::VectorXd white(parameters.cols() - 2);
      for (Eigen::Index i = 0; i < white.size(); ++i) {
        white[i] = normal(random_);
      }
      noise.row(joint).segment(1, white.size()) =
          (noise_stddev_ * noise_factor_ * white).transpose();
    }
    parameters_noise = parameters + noise;
    return true;
  }
  bool computeNoisyCosts(const Eigen::MatrixXd &parameters, std::size_t start, std::size_t count,
                         int iteration, int, Eigen::VectorXd &costs, bool &valid) override {
    return computeCosts(parameters, start, count, iteration, costs, valid);
  }
  bool computeCosts(const Eigen::MatrixXd &parameters, std::size_t, std::size_t, int,
                    Eigen::VectorXd &costs, bool &valid) override {
    costs = Eigen::VectorXd::Zero(parameters.cols());
    valid = true;
    for (Eigen::Index i = 0; i < parameters.cols(); ++i) {
      std::string reason;
      if (!check_(parameters.col(i), reason)) {
        costs[i] = 1000.0;
        valid = false;
      }
    }
    return true;
  }
  bool filterParameterUpdates(std::size_t, std::size_t, int, const Eigen::MatrixXd &,
                              Eigen::MatrixXd &updates) override {
    updates.col(0).setZero();
    updates.col(updates.cols() - 1).setZero();
    return true;
  }

private:
  StateCheck check_;
  double noise_stddev_;
  Eigen::MatrixXd noise_factor_;
  std::mt19937 random_;
};
Path refineWithStomp(const Path &initial, const StateCheck &check, const PlanSettings &settings) {
  stomp::StompConfiguration config{};
  config.num_iterations = settings.stomp_iterations;
  config.num_iterations_after_valid = settings.stomp_iterations;
  config.num_timesteps = initial.size();
  config.num_dimensions = initial.front().size();
  config.delta_t = 0.1;
  config.initialization_method = stomp::TrajectoryInitializations::LINEAR_INTERPOLATION;
  config.exponentiated_cost_sensitivity = 10.0;
  config.num_rollouts = settings.stomp_rollouts;
  config.max_rollouts = settings.stomp_rollouts;
  config.control_cost_weight = 0.1;
  auto task = std::make_shared<CollisionTask>(check, config.num_timesteps, settings.noise_stddev);
  stomp::Stomp optimizer(config, task);
  Eigen::MatrixXd seed(config.num_dimensions, config.num_timesteps);
  for (size_t i = 0; i < initial.size(); ++i) {
    seed.col(i) = initial[i];
  }
  Eigen::MatrixXd optimized;
  if (!optimizer.solve(seed, optimized) || !optimized.allFinite()) {
    return {};
  }
  Path result;
  for (Eigen::Index i = 0; i < optimized.cols(); ++i) {
    result.push_back(optimized.col(i));
  }
  return result;
}
} // namespace roscon_planner
