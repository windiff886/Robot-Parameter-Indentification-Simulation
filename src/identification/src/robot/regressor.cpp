/**
 * @file regressor.cpp
 * @brief Regressor matrix computation implementation
 */

#include "robot/regressor.hpp"
#include <cmath>

namespace robot {

// ============================================================================
// Standard Inertial Parameters
// ============================================================================

StandardInertialParameters
StandardInertialParameters::fromLinkParameters(const LinkParameters &link) {
  StandardInertialParameters sip;

  sip.m = link.mass;

  // First moments of mass
  sip.mx = link.mass * link.com[0];
  sip.my = link.mass * link.com[1];
  sip.mz = link.mass * link.com[2];

  // Transform inertia from COM to link frame origin using parallel axis theorem
  // I_origin = I_com + m * (c^T * c * I - c * c^T)
  const double cx = link.com[0];
  const double cy = link.com[1];
  const double cz = link.com[2];
  const double m = link.mass;

  // Parallel axis theorem terms

  sip.Ixx = link.inertia.Ixx + m * (cy * cy + cz * cz);
  sip.Iyy = link.inertia.Iyy + m * (cx * cx + cz * cz);
  sip.Izz = link.inertia.Izz + m * (cx * cx + cy * cy);
  sip.Ixy = link.inertia.Ixy - m * cx * cy;
  sip.Ixz = link.inertia.Ixz - m * cx * cz;
  sip.Iyz = link.inertia.Iyz - m * cy * cz;

  return sip;
}

// ============================================================================
// Regressor Class
// ============================================================================

Regressor::Regressor(const RobotModel &model)
    : model_(model), kinematics_(model) {}

std::size_t Regressor::numParameters(DynamicsParamFlags flags) const {
  const std::size_t n = numDOF();

  // Standard inertial parameters for each link
  std::size_t params = n * StandardInertialParameters::PARAMS_PER_LINK;

  // Add parameters for fixed bodies
  const auto &fixed_bodies = model_.fixedBodies();
  params += fixed_bodies.size() * StandardInertialParameters::PARAMS_PER_LINK;

  if (hasFlag(flags, DynamicsParamFlags::ARMATURE)) {
    params += n;
  }

  if (hasFlag(flags, DynamicsParamFlags::DAMPING)) {
    params += n;
  }

  if (hasFlag(flags, DynamicsParamFlags::FRICTION)) {
    params += 2 * n;
  }

  return params;
}

// ============================================================================
// Parameter Vector
// ============================================================================

Regressor::VectorXd
Regressor::computeParameterVector(DynamicsParamFlags flags) const {
  const std::size_t n = numDOF();
  const std::size_t num_params = numParameters(flags);
  VectorXd beta = VectorXd::Zero(num_params);

  // Fill in standard inertial parameters for each link
  for (std::size_t i = 0; i < n; ++i) {
    const auto &link = model_.link(i + 1); // Skip base link
    auto sip = StandardInertialParameters::fromLinkParameters(link);
    auto sip_vec = sip.toVector();

    const std::size_t offset = i * StandardInertialParameters::PARAMS_PER_LINK;
    beta.segment(offset, StandardInertialParameters::PARAMS_PER_LINK) = sip_vec;
  }

  std::size_t current_offset = n * StandardInertialParameters::PARAMS_PER_LINK;

  // Fill in standard inertial parameters for fixed bodies
  const auto &fixed_bodies = model_.fixedBodies();
  for (std::size_t i = 0; i < fixed_bodies.size(); ++i) {
    const auto &fb = fixed_bodies[i];
    auto sip = StandardInertialParameters::fromLinkParameters(fb.params);
    auto sip_vec = sip.toVector();

    beta.segment(current_offset, StandardInertialParameters::PARAMS_PER_LINK) =
        sip_vec;
    current_offset += StandardInertialParameters::PARAMS_PER_LINK;
  }

  // Add armature parameters if requested (MuJoCo default: 0.1)
  if (hasFlag(flags, DynamicsParamFlags::ARMATURE)) {
    for (std::size_t i = 0; i < n; ++i) {
      beta(current_offset + i) = 0.1;
    }
    current_offset += n;
  }

  // Add damping parameters if requested (MuJoCo default: 1.0)
  if (hasFlag(flags, DynamicsParamFlags::DAMPING)) {
    for (std::size_t i = 0; i < n; ++i) {
      beta(current_offset + i) = 1.0;
    }
    current_offset += n;
  }

  // Add friction parameters if requested
  if (hasFlag(flags, DynamicsParamFlags::FRICTION)) {
    const auto &friction = model_.frictionParameters();
    for (std::size_t i = 0; i < n; ++i) {
      beta(current_offset + 2 * i) = friction[i].Fv;
      beta(current_offset + 2 * i + 1) = friction[i].Fc;
    }
  }

  return beta;
}

// ============================================================================
// Regressor Matrix
// ============================================================================

Regressor::MatrixXd
Regressor::computeLinkRegressorBlock(std::size_t link_idx, const VectorXd &q,
                                     const VectorXd &qd, const VectorXd &qdd,
                                     const Eigen::Matrix3d &R) const {

  // Get Jacobian and its derivative for this link's COM
  MatrixXd J = kinematics_.computeLinkJacobian(link_idx, q);
  MatrixXd Jdot = kinematics_.computeLinkJacobianDerivative(link_idx, q, qd);

  return computeBlockFromJacobians(J, Jdot, qd, qdd, R);
}

Regressor::MatrixXd Regressor::computeFixedBodyRegressorBlock(
    std::size_t fixed_body_idx, const VectorXd &q, const VectorXd &qd,
    const VectorXd &qdd, const Eigen::Matrix3d &R) const {
  // Get Jacobian and its derivative for fixed body COM
  MatrixXd J = kinematics_.computeFixedBodyJacobian(fixed_body_idx, q);
  MatrixXd Jdot =
      kinematics_.computeFixedBodyJacobianDerivative(fixed_body_idx, q, qd);

  return computeBlockFromJacobians(J, Jdot, qd, qdd, R);
}

Regressor::MatrixXd
Regressor::computeBlockFromJacobians(const MatrixXd &J, const MatrixXd &Jdot,
                                     const VectorXd &qd, const VectorXd &qdd,
                                     const Eigen::Matrix3d &R) const {
  static_cast<void>(R);
  const std::size_t n = numDOF();

  // Linear and angular parts of Jacobian (World Frame)
  Eigen::Matrix<double, 3, Eigen::Dynamic> Jv = J.topRows(3);
  Eigen::Matrix<double, 3, Eigen::Dynamic> Jw = J.bottomRows(3);
  Eigen::Matrix<double, 3, Eigen::Dynamic> Jv_dot = Jdot.topRows(3);
  Eigen::Matrix<double, 3, Eigen::Dynamic> Jw_dot = Jdot.bottomRows(3);

  // Linear and angular accelerations (World Frame)
  Eigen::Vector3d a_linear = Jv * qdd + Jv_dot * qd;
  Eigen::Vector3d a_angular = Jw * qdd + Jw_dot * qd;
  Eigen::Vector3d omega = Jw * qd;

  // Gravity contribution (World Frame)
  const auto &g_world = model_.gravity();
  Eigen::Vector3d g(g_world[0], g_world[1], g_world[2]);
  // Ensure gravity vector points DOWN (Regessor formulation: a - g where g is
  // gravity vector)
  if (g[2] > 0) {
    g = -g;
  }

  // ROTATE TO LOCAL FRAME
  // HYPOTHESIS TEST: mj_dyn uses World Kinematics with Local Inertia.
  // To match this, we Force Rotation to Identity (World Frame Kinematics).
  // This effectively treats Local Inertia Params as World Inertia Params.
  Eigen::Matrix3d R_loc = Eigen::Matrix3d::Identity(); // WAS: R.transpose();

  Eigen::Vector3d a_linear_local = R_loc * (a_linear - g); // Includes gravity
  Eigen::Vector3d a_angular_local = R_loc * a_angular;
  Eigen::Vector3d omega_local = R_loc * omega;

  // Project Jacobian rows to Local Frame
  Eigen::MatrixXd Jv_local = R_loc * Jv;
  Eigen::MatrixXd Jw_local = R_loc * Jw;

  // Build regressor block (n x 10 matrix)
  MatrixXd Y_block = MatrixXd::Zero(n, 10);

  // 1. Mass contribution: τ_m = Jv^T * m * (a - g)
  // In local frame: Jv_local^T * m * a_linear_local
  Y_block.col(0) = Jv_local.transpose() * a_linear_local;

  // 2. First moment (m*c) contribution
  Eigen::Matrix3d omega_skew = RobotKinematics::skew(omega_local);
  Eigen::Matrix3d alpha_skew = RobotKinematics::skew(a_angular_local);

  // Term 1: α × ... + ω × (ω × ...)
  Eigen::Matrix3d kin_term = alpha_skew + omega_skew * omega_skew;

  for (int i = 0; i < 3; ++i) {
    Eigen::Vector3d e = Eigen::Vector3d::Zero();
    e(i) = 1.0;

    // Coefficient for mc_i is vector contribution to Force and Moment
    // Force: kin_term * e_i
    // Moment: e_i x a_linear_local
    Y_block.col(1 + i) = Jv_local.transpose() * (kin_term * e) +
                         Jw_local.transpose() * e.cross(a_linear_local);
  }

  // 3. Inertia contribution
  // Coefficients matrix L(ω, α) using Local vectors
  double ox = omega_local(0), oy = omega_local(1), oz = omega_local(2);
  double ax = a_angular_local(0), ay = a_angular_local(1),
         az = a_angular_local(2);

  // Ixx (Param 4)
  Y_block.col(4) =
      Jw_local.transpose() * Eigen::Vector3d(ax, ox * oz, -ox * oy);

  // Ixy (Param 5)
  Y_block.col(5) =
      Jw_local.transpose() *
      Eigen::Vector3d(ay - ox * oz, ax + oy * oz, oy * oy - ox * ox);

  // Ixz (Param 6)
  Y_block.col(6) =
      Jw_local.transpose() *
      Eigen::Vector3d(az + ox * oy, oz * oz - ox * ox, ax - oy * oz);

  // Iyy (Param 7)
  Y_block.col(7) =
      Jw_local.transpose() * Eigen::Vector3d(-oy * oz, ay, ox * oy);

  // Iyz (Param 8)
  Y_block.col(8) =
      Jw_local.transpose() *
      Eigen::Vector3d(oy * oy - oz * oz, az - ox * oy, ay + ox * oz);

  // Izz (Param 9)
  Y_block.col(9) =
      Jw_local.transpose() * Eigen::Vector3d(oy * oz, -ox * oz, az);

  return Y_block;
}

Regressor::MatrixXd
Regressor::computeRegressorMatrix(const VectorXd &q, const VectorXd &qd,
                                  const VectorXd &qdd,
                                  DynamicsParamFlags flags) const {
  const std::size_t n = numDOF();
  const std::size_t num_params = numParameters(flags);
  MatrixXd Y = MatrixXd::Zero(n, num_params);

  // Precompute link transforms to get orientations
  auto transforms = kinematics_.computeAllTransforms(q);

  // Compute regressor contribution from each link
  for (std::size_t i = 0; i < n; ++i) {
    // Get orientation of link i+1 (index i in transforms)
    Eigen::Matrix3d R = RobotKinematics::getRotation(transforms[i]);
    MatrixXd Y_link = computeLinkRegressorBlock(i, q, qd, qdd, R);

    const std::size_t offset = i * StandardInertialParameters::PARAMS_PER_LINK;
    Y.block(0, offset, n, StandardInertialParameters::PARAMS_PER_LINK) +=
        Y_link;
  }

  std::size_t current_offset = n * StandardInertialParameters::PARAMS_PER_LINK;

  // Add regressor blocks for fixed bodies
  const auto &fixed_bodies = model_.fixedBodies();
  for (std::size_t fb_idx = 0; fb_idx < fixed_bodies.size(); ++fb_idx) {
    const auto &fb = fixed_bodies[fb_idx];

    // Compute R_fb = R_parent * R_local
    std::size_t parent_idx = fb.parent_link_idx;
    Eigen::Matrix3d R_parent = Eigen::Matrix3d::Identity();
    if (parent_idx > 0 && parent_idx - 1 < transforms.size()) {
      R_parent = RobotKinematics::getRotation(transforms[parent_idx - 1]);
    }
    Eigen::Quaterniond quat(fb.quaternion[0], fb.quaternion[1],
                            fb.quaternion[2], fb.quaternion[3]);
    Eigen::Matrix3d R_fb = R_parent * quat.toRotationMatrix();

    MatrixXd Y_fb = computeFixedBodyRegressorBlock(fb_idx, q, qd, qdd, R_fb);

    Y.block(0, current_offset, n,
            StandardInertialParameters::PARAMS_PER_LINK) += Y_fb;
    current_offset += StandardInertialParameters::PARAMS_PER_LINK;
  }

  // Add armature regressor if requested
  // armature term: τ_armature = armature * q̈
  if (hasFlag(flags, DynamicsParamFlags::ARMATURE)) {
    for (std::size_t i = 0; i < n; ++i) {
      Y(i, current_offset + i) = qdd(i);
    }
    current_offset += n;
  }

  // Add damping regressor if requested
  // MuJoCo damping is a passive term: τ_ctrl = M*q̈ + C*q̇ + G - damping*q̇
  // So the regressor term is -q̇ (negative sign)
  if (hasFlag(flags, DynamicsParamFlags::DAMPING)) {
    for (std::size_t i = 0; i < n; ++i) {
      Y(i, current_offset + i) = -qd(i);
    }
    current_offset += n;
  }

  // Add friction regressor if requested
  if (hasFlag(flags, DynamicsParamFlags::FRICTION)) {
    for (std::size_t i = 0; i < n; ++i) {
      // Viscous friction: Fv * qd
      Y(i, current_offset + 2 * i) = qd(i);

      // Coulomb friction: Fc * sign(qd)
      if (std::abs(qd(i)) > 1e-6) {
        Y(i, current_offset + 2 * i + 1) = (qd(i) > 0) ? 1.0 : -1.0;
      }
    }
  }

  return Y;
}

Regressor::MatrixXd
Regressor::computeObservationMatrix(const MatrixXd &Q, const MatrixXd &Qd,
                                    const MatrixXd &Qdd,
                                    DynamicsParamFlags flags) const {
  const std::size_t n = numDOF();
  const std::size_t K = Q.cols(); // Number of samples
  const std::size_t num_params = numParameters(flags);

  MatrixXd W = MatrixXd::Zero(n * K, num_params);

  for (std::size_t k = 0; k < K; ++k) {
    VectorXd q = Q.col(k);
    VectorXd qd = Qd.col(k);
    VectorXd qdd = Qdd.col(k);

    MatrixXd Y_k = computeRegressorMatrix(q, qd, qdd, flags);
    W.block(k * n, 0, n, num_params) = Y_k;
  }

  return W;
}

} // namespace robot
