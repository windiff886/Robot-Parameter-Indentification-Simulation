/**
 * @file robot_dynamics.hpp
 * @brief Robot dynamics computation (M, C, G matrices)
 *
 * Computes inverse and forward dynamics using the Euler-Lagrange formulation.
 */

#ifndef ROBOT__ROBOT_DYNAMICS_HPP_
#define ROBOT__ROBOT_DYNAMICS_HPP_

#include "robot/robot_kinematics.hpp"
#include "robot/robot_model.hpp"
#include <Eigen/Dense>
#include <memory>

namespace robot {

/**
 * @brief Result structure for dynamics computation
 */
struct DynamicsMatrices {
  Eigen::MatrixXd M; ///< Inertia matrix (NxN)
  Eigen::MatrixXd C; ///< Coriolis matrix (NxN)
  Eigen::VectorXd G; ///< Gravity vector (Nx1)
  Eigen::VectorXd F; ///< Friction torque (Nx1)
};

/**
 * @brief Robot dynamics computation class
 *
 * Computes the dynamics equation:
 *   τ = M(q)q̈ + C(q,q̇)q̇ + G(q) + F(q̇)
 *
 * Where:
 *   M(q)   : Inertia matrix
 *   C(q,q̇) : Coriolis/centrifugal matrix
 *   G(q)   : Gravity vector
 *   F(q̇)  : Friction torque
 */
class RobotDynamics {
public:
  using VectorXd = Eigen::VectorXd;
  using MatrixXd = Eigen::MatrixXd;
  using Matrix3d = Eigen::Matrix3d;
  using Vector3d = Eigen::Vector3d;

  explicit RobotDynamics(const RobotModel &model);

  // ========== Dynamics Matrices ==========

  /**
   * @brief Compute inertia matrix M(q)
   * @param q Joint angles
   * @return NxN symmetric positive definite inertia matrix
   */
  MatrixXd computeInertiaMatrix(const VectorXd &q) const;

  /**
   * @brief Compute Coriolis/centrifugal matrix C(q, q̇)
   * @param q Joint angles
   * @param qd Joint velocities
   * @return NxN Coriolis matrix (using Christoffel symbols)
   */
  MatrixXd computeCoriolisMatrix(const VectorXd &q, const VectorXd &qd) const;

  /**
   * @brief Compute gravity vector G(q)
   * @param q Joint angles
   * @return Nx1 gravity torque vector
   */
  VectorXd computeGravityVector(const VectorXd &q) const;

  /**
   * @brief Compute friction torque F(q̇)
   * @param qd Joint velocities
   * @return Nx1 friction torque vector
   */
  VectorXd computeFrictionTorque(const VectorXd &qd) const;

  /**
   * @brief Compute all dynamics matrices at once
   * @param q Joint angles
   * @param qd Joint velocities
   * @return DynamicsMatrices structure containing M, C, G, F
   */
  DynamicsMatrices computeDynamicsMatrices(const VectorXd &q,
                                           const VectorXd &qd) const;

  // ========== Inverse Dynamics ==========

  /**
   * @brief Compute inverse dynamics: τ = M(q)q̈ + C(q,q̇)q̇ + G(q) + F(q̇)
   * @param q Joint angles
   * @param qd Joint velocities
   * @param qdd Joint accelerations
   * @return Required joint torques
   */
  VectorXd computeInverseDynamics(const VectorXd &q, const VectorXd &qd,
                                  const VectorXd &qdd) const;

  /**
   * @brief Compute inverse dynamics without friction
   * @param q Joint angles
   * @param qd Joint velocities
   * @param qdd Joint accelerations
   * @return Required joint torques (no friction)
   */
  VectorXd computeInverseDynamicsNoFriction(const VectorXd &q,
                                            const VectorXd &qd,
                                            const VectorXd &qdd) const;

  // ========== Forward Dynamics ==========

  /**
   * @brief Compute forward dynamics: q̈ = M(q)^{-1}(τ - C(q,q̇)q̇ - G(q) - F(q̇))
   * @param q Joint angles
   * @param qd Joint velocities
   * @param tau Applied joint torques
   * @return Resulting joint accelerations
   */
  VectorXd computeForwardDynamics(const VectorXd &q, const VectorXd &qd,
                                  const VectorXd &tau) const;

  // ========== Accessors ==========

  const RobotModel &model() const { return model_; }
  const RobotKinematics &kinematics() const { return kinematics_; }
  std::size_t numDOF() const { return model_.numDOF(); }

private:
  const RobotModel &model_;
  RobotKinematics kinematics_;

  /**
   * @brief Convert link inertia to 6x6 spatial inertia matrix
   */
  Eigen::Matrix<double, 6, 6> computeSpatialInertia(std::size_t link_idx) const;
};

} // namespace robot

#endif // ROBOT__ROBOT_DYNAMICS_HPP_
