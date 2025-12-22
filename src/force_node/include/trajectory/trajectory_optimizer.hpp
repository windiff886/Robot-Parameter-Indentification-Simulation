/**
 * @file trajectory_optimizer.hpp
 * @brief Trajectory optimization for parameter identification
 *
 * Optimizes Fourier coefficients to minimize observation matrix condition
 * number.
 */

#ifndef TRAJECTORY__TRAJECTORY_OPTIMIZER_HPP_
#define TRAJECTORY__TRAJECTORY_OPTIMIZER_HPP_

#include "robot/regressor.hpp"
#include "robot/robot_model.hpp"
#include "trajectory/fourier_trajectory.hpp"
#include <functional>

namespace trajectory {

/**
 * @brief Optimization result
 */
struct OptimizationResult {
  bool success;
  double cost;
  double condition_number;
  double sigma_min;
  std::size_t iterations;
  Eigen::VectorXd optimal_params;
};

/**
 * @brief Optimization options
 */
struct OptimizationOptions {
  std::size_t max_iterations = 100;
  std::size_t num_samples = 100; ///< Samples for W matrix
  double tolerance = 1e-6;
  double k1 = 1.0;   ///< Weight for cond(W)
  double k2 = 100.0; ///< Weight for 1/sigma_min
  bool verbose = false;
};

/**
 * @brief Trajectory optimizer for excitation trajectory generation
 *
 * Optimizes trajectory to minimize:
 *   J = k1 * cond(W^T W) + k2 / sigma_min(W)
 *
 * Where W is the observation matrix (stacked regressor matrices).
 *
 * This ensures good identifiability of the robot parameters.
 */
class TrajectoryOptimizer {
public:
  using VectorXd = Eigen::VectorXd;
  using MatrixXd = Eigen::MatrixXd;
  using CostFunction = std::function<double(const VectorXd &)>;

  /**
   * @brief Construct optimizer
   * @param model Robot model for regressor computation
   * @param trajectory Fourier trajectory to optimize
   * @param t_start Start time
   * @param t_end End time
   */
  TrajectoryOptimizer(const robot::RobotModel &model,
                      FourierTrajectory &trajectory, double t_start,
                      double t_end);

  /**
   * @brief Compute cost function: J = k1 * cond(W) + k2 / sigma_min
   * @param params Fourier coefficient vector
   * @return Cost value
   */
  double computeCost(const VectorXd &params) const;

  /**
   * @brief Compute cost with current trajectory parameters
   */
  double computeCost() const;

  /**
   * @brief Run optimization using gradient-free method
   * @param options Optimization options
   * @return Optimization result
   */
  OptimizationResult optimize(const OptimizationOptions &options = {});

  /**
   * @brief Build observation matrix W for current trajectory
   * @param num_samples Number of trajectory samples
   * @return Observation matrix W (n_dof * num_samples, n_params)
   */
  MatrixXd buildObservationMatrix(std::size_t num_samples) const;

  /**
   * @brief Get trajectory limits from robot model
   */
  TrajectoryLimits getLimits() const;

  /**
   * @brief Check if trajectory satisfies joint limits
   * @param num_samples Number of samples to check
   * @return true if within limits
   */
  bool checkLimits(std::size_t num_samples) const;

  // Accessors
  const robot::RobotModel &model() const { return model_; }
  FourierTrajectory &trajectory() { return trajectory_; }
  const FourierTrajectory &trajectory() const { return trajectory_; }

private:
  const robot::RobotModel &model_;
  FourierTrajectory &trajectory_;
  double t_start_;
  double t_end_;
  robot::Regressor regressor_;

  /**
   * @brief Simple gradient-free optimizer (coordinate descent)
   */
  OptimizationResult
  coordinateDescentOptimize(const OptimizationOptions &options);
};

} // namespace trajectory

#endif // TRAJECTORY__TRAJECTORY_OPTIMIZER_HPP_
