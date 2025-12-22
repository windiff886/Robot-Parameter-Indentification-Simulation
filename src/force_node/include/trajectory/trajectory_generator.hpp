/**
 * @file trajectory_generator.hpp
 * @brief Base class for trajectory generation
 *
 * Generates joint space trajectories for robot parameter identification.
 */

#ifndef TRAJECTORY__TRAJECTORY_GENERATOR_HPP_
#define TRAJECTORY__TRAJECTORY_GENERATOR_HPP_

#include <Eigen/Dense>
#include <vector>
#include <memory>

namespace trajectory
{

/**
 * @brief Trajectory point containing position, velocity, and acceleration
 */
struct TrajectoryPoint
{
  double time;
  Eigen::VectorXd q;    ///< Joint positions [rad]
  Eigen::VectorXd qd;   ///< Joint velocities [rad/s]
  Eigen::VectorXd qdd;  ///< Joint accelerations [rad/s²]
};

/**
 * @brief Complete trajectory data
 */
struct TrajectoryData
{
  std::vector<double> time;
  Eigen::MatrixXd Q;    ///< Joint positions (n_dof x n_samples)
  Eigen::MatrixXd Qd;   ///< Joint velocities (n_dof x n_samples)
  Eigen::MatrixXd Qdd;  ///< Joint accelerations (n_dof x n_samples)

  std::size_t numSamples() const { return time.size(); }
  std::size_t numDOF() const { return Q.rows(); }
};

/**
 * @brief Joint limits for trajectory constraints
 */
struct TrajectoryLimits
{
  Eigen::VectorXd q_min;     ///< Min position [rad]
  Eigen::VectorXd q_max;     ///< Max position [rad]
  Eigen::VectorXd qd_max;    ///< Max velocity [rad/s]
  Eigen::VectorXd qdd_max;   ///< Max acceleration [rad/s²]
};

/**
 * @brief Abstract base class for trajectory generators
 */
class TrajectoryGenerator
{
public:
  using VectorXd = Eigen::VectorXd;
  using MatrixXd = Eigen::MatrixXd;

  virtual ~TrajectoryGenerator() = default;

  /**
   * @brief Evaluate trajectory at a specific time
   * @param t Time [s]
   * @return Trajectory point (q, qd, qdd)
   */
  virtual TrajectoryPoint evaluate(double t) const = 0;

  /**
   * @brief Generate complete trajectory data
   * @param t_start Start time [s]
   * @param t_end End time [s]
   * @param num_samples Number of samples
   * @return TrajectoryData containing Q, Qd, Qdd matrices
   */
  virtual TrajectoryData generate(
    double t_start,
    double t_end,
    std::size_t num_samples) const;

  /**
   * @brief Get number of DOF
   */
  virtual std::size_t numDOF() const = 0;

  /**
   * @brief Get initial position q0
   */
  virtual VectorXd initialPosition() const = 0;
};

}  // namespace trajectory

#endif  // TRAJECTORY__TRAJECTORY_GENERATOR_HPP_
