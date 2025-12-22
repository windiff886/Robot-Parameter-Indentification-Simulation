/**
 * @file trajectory_optimizer.cpp
 * @brief Trajectory optimization implementation
 */

#include "trajectory/trajectory_optimizer.hpp"
#include <Eigen/SVD>
#include <iostream>
#include <random>

namespace trajectory {

TrajectoryOptimizer::TrajectoryOptimizer(const robot::RobotModel &model,
                                         FourierTrajectory &trajectory,
                                         double t_start, double t_end)
    : model_(model), trajectory_(trajectory), t_start_(t_start), t_end_(t_end),
      regressor_(model) {}

TrajectoryOptimizer::MatrixXd
TrajectoryOptimizer::buildObservationMatrix(std::size_t num_samples) const {
  const std::size_t n = model_.numDOF();
  const std::size_t num_params = regressor_.numParameters(false);

  MatrixXd W = MatrixXd::Zero(n * num_samples, num_params);

  const double dt = (t_end_ - t_start_) / static_cast<double>(num_samples - 1);

  for (std::size_t i = 0; i < num_samples; ++i) {
    const double t = t_start_ + i * dt;
    TrajectoryPoint pt = trajectory_.evaluate(t);

    MatrixXd Y = regressor_.computeRegressorMatrix(pt.q, pt.qd, pt.qdd, false);
    W.block(i * n, 0, n, num_params) = Y;
  }

  return W;
}

double TrajectoryOptimizer::computeCost(const VectorXd &params) const {
  // Temporarily set parameters
  FourierTrajectory &traj = const_cast<FourierTrajectory &>(trajectory_);
  VectorXd old_params = traj.getParameterVector();
  traj.setParameterVector(params);

  double cost = computeCost();

  // Restore old parameters
  traj.setParameterVector(old_params);

  return cost;
}

double TrajectoryOptimizer::computeCost() const {
  const std::size_t num_samples = 100;

  // Build observation matrix
  MatrixXd W = buildObservationMatrix(num_samples);

  // Compute SVD
  Eigen::JacobiSVD<MatrixXd> svd(W, Eigen::ComputeThinU | Eigen::ComputeThinV);
  VectorXd singular_values = svd.singularValues();

  if (singular_values.size() == 0) {
    return std::numeric_limits<double>::infinity();
  }

  const double sigma_max = singular_values(0);
  const double sigma_min = singular_values(singular_values.size() - 1);

  if (sigma_min < 1e-10) {
    return std::numeric_limits<double>::infinity();
  }

  // Cost: J = k1 * cond(W) + k2 / sigma_min
  const double k1 = 1.0;
  const double k2 = 100.0;
  const double cond = sigma_max / sigma_min;

  return k1 * cond + k2 / sigma_min;
}

TrajectoryLimits TrajectoryOptimizer::getLimits() const {
  const std::size_t n = model_.numDOF();
  TrajectoryLimits limits;

  limits.q_min.resize(n);
  limits.q_max.resize(n);
  limits.qd_max.resize(n);
  limits.qdd_max.resize(n);

  for (std::size_t i = 0; i < n; ++i) {
    const auto &jl = model_.limits(i);
    limits.q_min(i) = jl.q_min;
    limits.q_max(i) = jl.q_max;
    limits.qd_max(i) = jl.qd_max;
    limits.qdd_max(i) = jl.qdd_max;
  }

  return limits;
}

bool TrajectoryOptimizer::checkLimits(std::size_t num_samples) const {
  auto limits = getLimits();
  const double dt = (t_end_ - t_start_) / static_cast<double>(num_samples - 1);

  for (std::size_t i = 0; i < num_samples; ++i) {
    const double t = t_start_ + i * dt;
    TrajectoryPoint pt = trajectory_.evaluate(t);

    for (std::size_t j = 0; j < model_.numDOF(); ++j) {
      if (pt.q(j) < limits.q_min(j) || pt.q(j) > limits.q_max(j)) {
        return false;
      }
      if (std::abs(pt.qd(j)) > limits.qd_max(j)) {
        return false;
      }
      if (std::abs(pt.qdd(j)) > limits.qdd_max(j)) {
        return false;
      }
    }
  }

  return true;
}

OptimizationResult
TrajectoryOptimizer::optimize(const OptimizationOptions &options) {
  return coordinateDescentOptimize(options);
}

OptimizationResult TrajectoryOptimizer::coordinateDescentOptimize(
    const OptimizationOptions &options) {
  OptimizationResult result;
  result.success = false;
  result.iterations = 0;

  const std::size_t n_params = trajectory_.numParameters();
  VectorXd x = trajectory_.getParameterVector();
  double best_cost = computeCost();

  if (options.verbose) {
    std::cout << "Initial cost: " << best_cost << std::endl;
  }

  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<> step_dis(0.01, 0.1);

  for (std::size_t iter = 0; iter < options.max_iterations; ++iter) {
    bool improved = false;

    // Coordinate descent: optimize each parameter
    for (std::size_t i = 0; i < n_params; ++i) {
      const double step = step_dis(gen);

      // Try positive step
      VectorXd x_plus = x;
      x_plus(i) += step;
      trajectory_.setParameterVector(x_plus);
      trajectory_.applyInitialConditionConstraints();

      double cost_plus = std::numeric_limits<double>::infinity();
      if (checkLimits(options.num_samples)) {
        cost_plus = computeCost();
      }

      // Try negative step
      VectorXd x_minus = x;
      x_minus(i) -= step;
      trajectory_.setParameterVector(x_minus);
      trajectory_.applyInitialConditionConstraints();

      double cost_minus = std::numeric_limits<double>::infinity();
      if (checkLimits(options.num_samples)) {
        cost_minus = computeCost();
      }

      // Choose best
      if (cost_plus < best_cost && cost_plus <= cost_minus) {
        x = x_plus;
        trajectory_.setParameterVector(x);
        trajectory_.applyInitialConditionConstraints();
        best_cost = cost_plus;
        improved = true;
      } else if (cost_minus < best_cost) {
        x = x_minus;
        trajectory_.setParameterVector(x);
        trajectory_.applyInitialConditionConstraints();
        best_cost = cost_minus;
        improved = true;
      } else {
        // Restore original
        trajectory_.setParameterVector(x);
        trajectory_.applyInitialConditionConstraints();
      }
    }

    result.iterations = iter + 1;

    if (options.verbose && (iter % 10 == 0)) {
      std::cout << "Iteration " << iter << ", cost: " << best_cost << std::endl;
    }

    if (!improved) {
      break;
    }
  }

  // Compute final metrics
  MatrixXd W = buildObservationMatrix(options.num_samples);
  Eigen::JacobiSVD<MatrixXd> svd(W, Eigen::ComputeThinU | Eigen::ComputeThinV);
  VectorXd sv = svd.singularValues();

  result.success = true;
  result.cost = best_cost;
  result.optimal_params = trajectory_.getParameterVector();
  result.sigma_min = sv(sv.size() - 1);
  result.condition_number = sv(0) / result.sigma_min;

  if (options.verbose) {
    std::cout << "Optimization complete. Iterations: " << result.iterations
              << ", Cost: " << result.cost
              << ", Cond: " << result.condition_number << std::endl;
  }

  return result;
}

} // namespace trajectory
