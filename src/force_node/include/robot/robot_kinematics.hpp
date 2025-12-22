/**
 * @file robot_kinematics.hpp
 * @brief Forward kinematics computation for serial manipulators
 *
 * Computes homogeneous transformations and Jacobians using Modified DH convention.
 */

#ifndef ROBOT__ROBOT_KINEMATICS_HPP_
#define ROBOT__ROBOT_KINEMATICS_HPP_

#include "robot/robot_model.hpp"
#include <Eigen/Dense>
#include <vector>

namespace robot
{

/**
 * @brief Forward kinematics computation class
 *
 * Uses Modified DH (Proximal/Craig) convention:
 *   T_i^{i-1} = Rot_x(alpha_{i-1}) * Trans_x(a_{i-1}) * Rot_z(theta_i) * Trans_z(d_i)
 */
class RobotKinematics
{
public:
  using Matrix4d = Eigen::Matrix4d;
  using Matrix3d = Eigen::Matrix3d;
  using Vector3d = Eigen::Vector3d;
  using VectorXd = Eigen::VectorXd;
  using MatrixXd = Eigen::MatrixXd;

  explicit RobotKinematics(const RobotModel& model);

  // ========== Forward Kinematics ==========

  /**
   * @brief Compute homogeneous transform from frame i-1 to frame i
   * @param joint_idx Joint index (0 to n-1)
   * @param q Joint angle [rad]
   * @return 4x4 homogeneous transformation matrix
   */
  Matrix4d computeDHTransform(std::size_t joint_idx, double q) const;

  /**
   * @brief Compute transform from base to joint i
   * @param joint_idx Joint index (0 to n-1, use n for end-effector)
   * @param q Joint angles vector
   * @return 4x4 homogeneous transformation matrix T_0^i
   */
  Matrix4d computeForwardKinematics(std::size_t joint_idx, const VectorXd& q) const;

  /**
   * @brief Compute end-effector pose
   * @param q Joint angles vector
   * @return 4x4 homogeneous transformation matrix T_0^n
   */
  Matrix4d computeEndEffectorPose(const VectorXd& q) const;

  /**
   * @brief Compute all joint transforms at once
   * @param q Joint angles vector
   * @return Vector of 4x4 matrices [T_0^1, T_0^2, ..., T_0^n]
   */
  std::vector<Matrix4d> computeAllTransforms(const VectorXd& q) const;

  // ========== Jacobian Computation ==========

  /**
   * @brief Compute geometric Jacobian at end-effector
   * @param q Joint angles vector
   * @return 6xN Jacobian matrix [J_v; J_omega]
   */
  MatrixXd computeJacobian(const VectorXd& q) const;

  /**
   * @brief Compute geometric Jacobian for a specific link's COM
   * @param link_idx Link index
   * @param q Joint angles vector
   * @return 6xN Jacobian matrix for the link's center of mass
   */
  MatrixXd computeLinkJacobian(std::size_t link_idx, const VectorXd& q) const;

  /**
   * @brief Compute time derivative of Jacobian
   * @param q Joint angles vector
   * @param qd Joint velocities vector
   * @return 6xN Jacobian derivative matrix
   */
  MatrixXd computeJacobianDerivative(const VectorXd& q, const VectorXd& qd) const;

  // ========== Utility Functions ==========

  /**
   * @brief Extract rotation matrix from homogeneous transform
   */
  static Matrix3d getRotation(const Matrix4d& T);

  /**
   * @brief Extract position vector from homogeneous transform
   */
  static Vector3d getPosition(const Matrix4d& T);

  /**
   * @brief Compute skew-symmetric matrix from vector
   */
  static Matrix3d skew(const Vector3d& v);

  /**
   * @brief Get number of DOF
   */
  std::size_t numDOF() const { return model_.numDOF(); }

private:
  const RobotModel& model_;
};

}  // namespace robot

#endif  // ROBOT__ROBOT_KINEMATICS_HPP_
