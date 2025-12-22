/**
 * @file fourier_trajectory.hpp
 * @brief Fourier series trajectory generator for parameter identification
 *
 * Generates excitation trajectories using Fourier series parameterization.
 */

#ifndef TRAJECTORY__FOURIER_TRAJECTORY_HPP_
#define TRAJECTORY__FOURIER_TRAJECTORY_HPP_

#include "trajectory/trajectory_generator.hpp"
#include <cmath>

namespace trajectory {

/**
 * @brief Fourier series trajectory parameters
 */
struct FourierParameters {
  std::size_t n_dof;       ///< Number of DOF
  std::size_t n_harmonics; ///< Number of Fourier harmonics
  double omega;            ///< Fundamental frequency [rad/s]
  Eigen::VectorXd q0;      ///< Initial position
  Eigen::MatrixXd A;       ///< Sine coefficients (n_dof x n_harmonics)
  Eigen::MatrixXd B;       ///< Cosine coefficients (n_dof x n_harmonics)
};

/**
 * @brief Fourier series trajectory generator
 *
 * Trajectory parameterization:
 *   q_i(t) = q0_i + Σ_{j=1}^{n_f} [ A_ij/(ωj) * sin(ωjt) - B_ij/(ωj) * cos(ωjt)
 * ] q̇_i(t) = Σ_{j=1}^{n_f} [ A_ij * cos(ωjt) + B_ij * sin(ωjt) ] q̈_i(t) = ω *
 * Σ_{j=1}^{n_f} [ B_ij * j * cos(ωjt) - A_ij * j * sin(ωjt) ]
 *
 * This parameterization ensures:
 *   - q(0) = q0 when B coefficients satisfy constraint
 *   - q̇(0) = 0 when A coefficients satisfy constraint
 *   - Smooth, periodic motion suitable for identification
 */
class FourierTrajectory : public TrajectoryGenerator {
public:
  /**
   * @brief Construct from Fourier parameters
   */
  explicit FourierTrajectory(const FourierParameters &params);

  /**
   * @brief Construct with default parameters
   * @param n_dof Number of degrees of freedom
   * @param n_harmonics Number of Fourier harmonics (default 5)
   * @param period Trajectory period [s] (default 10.0)
   * @param q0 Initial position (default zeros)
   */
  FourierTrajectory(std::size_t n_dof, std::size_t n_harmonics = 5,
                    double period = 10.0,
                    const Eigen::VectorXd &q0 = Eigen::VectorXd());

  // TrajectoryGenerator interface
  TrajectoryPoint evaluate(double t) const override;
  std::size_t numDOF() const override { return params_.n_dof; }
  VectorXd initialPosition() const override { return params_.q0; }

  // Accessors
  const FourierParameters &parameters() const { return params_; }
  FourierParameters &parameters() { return params_; }

  /**
   * @brief Set Fourier coefficients
   * @param A Sine coefficients (n_dof x n_harmonics)
   * @param B Cosine coefficients (n_dof x n_harmonics)
   */
  void setCoefficients(const Eigen::MatrixXd &A, const Eigen::MatrixXd &B);

  /**
   * @brief Set coefficients from flat parameter vector
   * @param x Parameter vector [A_11, A_12, ..., A_1n, A_21, ..., B_11, ...]
   */
  void setParameterVector(const Eigen::VectorXd &x);

  /**
   * @brief Get flat parameter vector
   * @return Parameter vector [A_11, A_12, ..., B_n1, B_n2, ...]
   */
  Eigen::VectorXd getParameterVector() const;

  /**
   * @brief Get number of optimization parameters
   */
  std::size_t numParameters() const {
    return 2 * params_.n_dof * params_.n_harmonics;
  }

  /**
   * @brief Set random coefficients (for optimization initialization)
   * @param amplitude Max amplitude for coefficients
   */
  void setRandomCoefficients(double amplitude = 0.1);

  /**
   * @brief Apply initial condition constraints
   *
   * Ensures:
   *   - q(0) = q0
   *   - q̇(0) = 0
   *   - q̈(0) = 0
   */
  void applyInitialConditionConstraints();

private:
  FourierParameters params_;
};

} // namespace trajectory

#endif // TRAJECTORY__FOURIER_TRAJECTORY_HPP_
