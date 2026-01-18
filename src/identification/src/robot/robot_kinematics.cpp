/**
 * @file robot_kinematics.cpp
 * @brief Forward kinematics implementation
 */

#include "robot/robot_kinematics.hpp"
#include <cmath>

namespace robot {

RobotKinematics::RobotKinematics(const RobotModel &model) : model_(model) {}

// ============================================================================
// Forward Kinematics
// ============================================================================

RobotKinematics::Matrix4d
RobotKinematics::computeDHTransform(std::size_t joint_idx, double q) const {
  const auto &dh = model_.dh(joint_idx);

  // Modified DH convention (Craig):
  // T = Rot_x(alpha_{i-1}) * Trans_x(a_{i-1}) * Rot_z(theta_i) * Trans_z(d_i)
  const double alpha = dh.alpha;
  const double a = dh.a;
  const double d = dh.d;
  const double theta = q + dh.theta; // Add offset

  const double ct = std::cos(theta);
  const double st = std::sin(theta);
  const double ca = std::cos(alpha);
  const double sa = std::sin(alpha);

  Matrix4d T;
  T << ct, -st, 0, a, st * ca, ct * ca, -sa, -sa * d, st * sa, ct * sa, ca,
      ca * d, 0, 0, 0, 1;

  return T;
}

RobotKinematics::Matrix4d
RobotKinematics::computeForwardKinematics(std::size_t joint_idx,
                                          const VectorXd &q) const {
  Matrix4d T = Matrix4d::Identity();

  const std::size_t end_idx = std::min(joint_idx + 1, model_.numDOF());
  for (std::size_t i = 0; i < end_idx; ++i) {
    T = T * computeDHTransform(i, q(i));
  }

  return T;
}

RobotKinematics::Matrix4d
RobotKinematics::computeEndEffectorPose(const VectorXd &q) const {
  return computeForwardKinematics(model_.numDOF() - 1, q);
}

std::vector<RobotKinematics::Matrix4d>
RobotKinematics::computeAllTransforms(const VectorXd &q) const {
  std::vector<Matrix4d> transforms(model_.numDOF());
  Matrix4d T = Matrix4d::Identity();

  for (std::size_t i = 0; i < model_.numDOF(); ++i) {
    T = T * computeDHTransform(i, q(i));
    transforms[i] = T;
  }

  return transforms;
}

// ============================================================================
// Jacobian Computation
// ============================================================================

RobotKinematics::MatrixXd
RobotKinematics::computeJacobian(const VectorXd &q) const {
  const std::size_t n = model_.numDOF();
  MatrixXd J = MatrixXd::Zero(6, n);

  // Get all transforms
  auto transforms = computeAllTransforms(q);

  // End-effector position
  Vector3d p_n = getPosition(transforms[n - 1]);

  // Compute Jacobian columns
  Vector3d z_prev = Vector3d::UnitZ(); // z-axis of base frame
  Vector3d p_prev = Vector3d::Zero();  // origin of base frame

  for (std::size_t i = 0; i < n; ++i) {
    // Linear velocity part: z_{i-1} × (p_n - p_{i-1})
    J.block<3, 1>(0, i) = z_prev.cross(p_n - p_prev);

    // Angular velocity part: z_{i-1}
    J.block<3, 1>(3, i) = z_prev;

    // Update for next iteration
    z_prev = getRotation(transforms[i]).col(2); // z-axis of frame i
    p_prev = getPosition(transforms[i]);
  }

  return J;
}

RobotKinematics::MatrixXd
RobotKinematics::computeLinkJacobian(std::size_t link_idx,
                                     const VectorXd &q) const {
  const std::size_t n = model_.numDOF();
  MatrixXd J = MatrixXd::Zero(6, n);

  // Get all transforms
  auto transforms = computeAllTransforms(q);

  // Get link COM position in world frame
  // Link frame is at joint i, COM is offset from link frame
  const auto &link = model_.link(link_idx + 1); // link_idx 0 = link 1
  Vector3d com_local(link.com[0], link.com[1], link.com[2]);

  // Transform COM to world frame
  Matrix4d T_link = (link_idx < transforms.size()) ? transforms[link_idx]
                                                   : Matrix4d::Identity();
  Vector3d p_com = getPosition(T_link) + getRotation(T_link) * com_local;

  // Compute Jacobian columns up to link_idx
  Vector3d z_prev = Vector3d::UnitZ();
  Vector3d p_prev = Vector3d::Zero();

  for (std::size_t i = 0; i <= link_idx && i < n; ++i) {
    J.block<3, 1>(0, i) = z_prev.cross(p_com - p_prev);
    J.block<3, 1>(3, i) = z_prev;

    z_prev = getRotation(transforms[i]).col(2);
    p_prev = getPosition(transforms[i]);
  }

  return J;
}

RobotKinematics::MatrixXd
RobotKinematics::computeLinkOriginJacobian(std::size_t link_idx,
                                           const VectorXd &q) const {
  const std::size_t n = model_.numDOF();
  MatrixXd J = MatrixXd::Zero(6, n);

  // Get all transforms
  auto transforms = computeAllTransforms(q);

  // Link frame origin position in world frame
  Matrix4d T_link = (link_idx < transforms.size()) ? transforms[link_idx]
                                                   : Matrix4d::Identity();
  Vector3d p_link = getPosition(T_link);

  // Compute Jacobian columns up to link_idx
  Vector3d z_prev = Vector3d::UnitZ();
  Vector3d p_prev = Vector3d::Zero();

  for (std::size_t i = 0; i <= link_idx && i < n; ++i) {
    J.block<3, 1>(0, i) = z_prev.cross(p_link - p_prev);
    J.block<3, 1>(3, i) = z_prev;

    z_prev = getRotation(transforms[i]).col(2);
    p_prev = getPosition(transforms[i]);
  }

  return J;
}

RobotKinematics::MatrixXd
RobotKinematics::computeJacobianDerivative(const VectorXd &q,
                                           const VectorXd &qd) const {
  // Numerical differentiation using finite differences
  const double eps = 1e-8;
  const std::size_t n = model_.numDOF();
  MatrixXd Jdot = MatrixXd::Zero(6, n);

  VectorXd q_plus = q + eps * qd;
  VectorXd q_minus = q - eps * qd;

  MatrixXd J_plus = computeJacobian(q_plus);
  MatrixXd J_minus = computeJacobian(q_minus);

  Jdot = (J_plus - J_minus) / (2.0 * eps);

  return Jdot;
}

RobotKinematics::MatrixXd RobotKinematics::computeLinkOriginJacobianDerivative(
    std::size_t link_idx, const VectorXd &q, const VectorXd &qd) const {
  const double eps = 1e-8;

  VectorXd q_plus = q + eps * qd;
  VectorXd q_minus = q - eps * qd;

  MatrixXd J_plus = computeLinkOriginJacobian(link_idx, q_plus);
  MatrixXd J_minus = computeLinkOriginJacobian(link_idx, q_minus);

  return (J_plus - J_minus) / (2.0 * eps);
}

RobotKinematics::MatrixXd RobotKinematics::computeLinkJacobianDerivative(
    std::size_t link_idx, const VectorXd &q, const VectorXd &qd) const {
  const double eps = 1e-8;

  VectorXd q_plus = q + eps * qd;
  VectorXd q_minus = q - eps * qd;

  MatrixXd J_plus = computeLinkJacobian(link_idx, q_plus);
  MatrixXd J_minus = computeLinkJacobian(link_idx, q_minus);

  return (J_plus - J_minus) / (2.0 * eps);
}

RobotKinematics::MatrixXd
RobotKinematics::computeFixedBodyJacobian(std::size_t fixed_body_idx,
                                          const VectorXd &q) const {
  const auto &fixed_bodies = model_.fixedBodies();
  if (fixed_body_idx >= fixed_bodies.size()) {
    throw std::out_of_range("Fixed body index out of range");
  }

  const auto &fb = fixed_bodies[fixed_body_idx];
  const std::size_t parent_idx = fb.parent_link_idx;

  const std::size_t n = model_.numDOF();
  MatrixXd J = MatrixXd::Zero(6, n);

  // Get all transforms
  auto transforms = computeAllTransforms(q);

  // Get parent link transform
  // Get parent link transform
  Matrix4d T_parent = Matrix4d::Identity();
  if (parent_idx > 0 && parent_idx - 1 < transforms.size()) {
    T_parent = transforms[parent_idx - 1];
  }

  // Fixed body transform relative to parent
  Matrix4d T_fb_local = Matrix4d::Identity();
  // Position offset
  T_fb_local.block<3, 1>(0, 3) =
      Vector3d(fb.position[0], fb.position[1], fb.position[2]);
  // Orientation (quaternion w,x,y,z)
  Eigen::Quaterniond quat(fb.quaternion[0], fb.quaternion[1], fb.quaternion[2],
                          fb.quaternion[3]);
  T_fb_local.block<3, 3>(0, 0) = quat.toRotationMatrix();

  // Fixed body frame in world
  Matrix4d T_fb = T_parent * T_fb_local;

  // Fixed body COM in fixed body frame
  Vector3d com_fb_local(fb.params.com[0], fb.params.com[1], fb.params.com[2]);

  // Transform COM to world frame
  Vector3d p_com = getPosition(T_fb) + getRotation(T_fb) * com_fb_local;

  // Compute Jacobian (same as parent link, but with different point)
  Vector3d z_prev = Vector3d::UnitZ();
  Vector3d p_prev = Vector3d::Zero();

  for (std::size_t i = 0; i <= parent_idx && i < n; ++i) {
    J.block<3, 1>(0, i) = z_prev.cross(p_com - p_prev);
    J.block<3, 1>(3, i) = z_prev;

    z_prev = getRotation(transforms[i]).col(2);
    p_prev = getPosition(transforms[i]);
  }

  return J;
}

RobotKinematics::MatrixXd RobotKinematics::computeFixedBodyJacobianDerivative(
    std::size_t fixed_body_idx, const VectorXd &q, const VectorXd &qd) const {
  const double eps = 1e-8;

  VectorXd q_plus = q + eps * qd;
  VectorXd q_minus = q - eps * qd;

  MatrixXd J_plus = computeFixedBodyJacobian(fixed_body_idx, q_plus);
  MatrixXd J_minus = computeFixedBodyJacobian(fixed_body_idx, q_minus);

  return (J_plus - J_minus) / (2.0 * eps);
}

// ============================================================================
// Utility Functions
// ============================================================================

RobotKinematics::Matrix3d RobotKinematics::getRotation(const Matrix4d &T) {
  return T.block<3, 3>(0, 0);
}

RobotKinematics::Vector3d RobotKinematics::getPosition(const Matrix4d &T) {
  return T.block<3, 1>(0, 3);
}

RobotKinematics::Matrix3d RobotKinematics::skew(const Vector3d &v) {
  Matrix3d S;
  S << 0, -v(2), v(1), v(2), 0, -v(0), -v(1), v(0), 0;
  return S;
}

} // namespace robot
