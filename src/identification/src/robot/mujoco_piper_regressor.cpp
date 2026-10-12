/**
 * @file mujoco_piper_regressor.cpp
 * @brief Regressor matrix computation implementation based on MuJoCo kinematics for Piper
 */

#include "mujoco_piper_regressor.hpp"
#include <iostream>

namespace mujoco_dynamics {

// ============================================================================
// MuJoCoPiperRegressor Constructor
// ============================================================================

MuJoCoPiperRegressor::MuJoCoPiperRegressor() { initBodies(); }

void MuJoCoPiperRegressor::initBodies() {
  bodies_[0].name = "base_link";
  bodies_[0].mass = 1.02;
  bodies_[0].com = Vector3d(-0.00473641, 2.56829e-05, 0.041452);
  bodies_[0].Ixx = 0.00267433;
  bodies_[0].Iyy = 0.00282612;
  bodies_[0].Izz = 0.00089624;
  bodies_[0].Ixy = -0.00000073;
  bodies_[0].Ixz = -0.00017389;
  bodies_[0].Iyz = 0.0000004;
  bodies_[0].has_joint = false;

  bodies_[1].name = "link1";
  bodies_[1].pos = Vector3d(0.0, 0.0, 0.123);
  bodies_[1].quat = Quaterniond(1.0, 0.0, 0.0, 0.0);
  bodies_[1].mass = 0.71;
  bodies_[1].com = Vector3d(0.000121505, 0.000104632, -0.00438597);
  bodies_[1].Ixx = 0.000489262;
  bodies_[1].Iyy = 0.000439887;
  bodies_[1].Izz = 0.000404551;
  bodies_[1].has_joint = true;

  bodies_[2].name = "link2";
  bodies_[2].quat = Quaterniond(0.0356735, -0.0356786, -0.706207, -0.706205);
  bodies_[2].mass = 1.17;
  bodies_[2].com = Vector3d(0.198666, -0.0109269, 0.00142122);
  bodies_[2].Ixx = 0.0679032;
  bodies_[2].Iyy = 0.067745;
  bodies_[2].Izz = 0.00111966;
  bodies_[2].has_joint = true;

  bodies_[3].name = "link3";
  bodies_[3].pos = Vector3d(0.28503, 0.0, 0.0);
  bodies_[3].quat = Quaterniond(0.637536, 0.0, 0.0, -0.77042);
  bodies_[3].mass = 0.5;
  bodies_[3].com = Vector3d(-0.0202738, -0.133915, -0.000458683);
  bodies_[3].Ixx = 0.0138227;
  bodies_[3].Iyy = 0.0138032;
  bodies_[3].Izz = 0.000244685;
  bodies_[3].has_joint = true;

  bodies_[4].name = "link4";
  bodies_[4].pos = Vector3d(-0.021984, -0.25075, 0.0);
  bodies_[4].quat = Quaterniond(0.707105, 0.707108, 0.0, 0.0);
  bodies_[4].mass = 0.38;
  bodies_[4].com = Vector3d(-9.66636e-05, 0.000876064, -0.00496881);
  bodies_[4].Ixx = 0.000191586;
  bodies_[4].Iyy = 0.000185052;
  bodies_[4].Izz = 0.000152863;
  bodies_[4].has_joint = true;

  bodies_[5].name = "link5";
  bodies_[5].quat = Quaterniond(0.707105, -0.707108, 0.0, 0.0);
  bodies_[5].mass = 0.383;
  bodies_[5].com = Vector3d(-4.10554e-05, -0.0566487, -0.00372058);
  bodies_[5].Ixx = 0.00166169;
  bodies_[5].Iyy = 0.00164328;
  bodies_[5].Izz = 0.000185028;
  bodies_[5].has_joint = true;

  bodies_[6].name = "link6";
  bodies_[6].pos = Vector3d(8.8259e-05, -0.091, 0.0);
  bodies_[6].quat = Quaterniond(0.707105, 0.707108, 0.0, 0.0);
  bodies_[6].mass = 0.456991;
  bodies_[6].com = Vector3d(-0.000182345, 7.94104e-05, 0.0316214);
  bodies_[6].Ixx = 0.000938039;
  bodies_[6].Iyy = 0.000723068;
  bodies_[6].Izz = 0.000395388;
  bodies_[6].has_joint = true;

  for (std::size_t i = 1; i <= N_BODIES; ++i) {
    bodies_[i].joint_axis = Vector3d(0.0, 0.0, 1.0);
    bodies_[i].armature = 0.1;
    bodies_[i].damping = 1.0;
  }
}

// ============================================================================
// Helper functions
// ============================================================================

MuJoCoPiperRegressor::Matrix3d MuJoCoPiperRegressor::skew(const Vector3d &v) {
  Matrix3d S;
  S << 0, -v(2), v(1), v(2), 0, -v(0), -v(1), v(0), 0;
  return S;
}

MuJoCoPiperRegressor::Matrix4d
MuJoCoPiperRegressor::poseToTransform(const Vector3d &pos, const Quaterniond &quat) {
  Matrix4d T = Matrix4d::Identity();
  T.block<3, 3>(0, 0) = quat.toRotationMatrix();
  T.block<3, 1>(0, 3) = pos;
  return T;
}

// ============================================================================
// Kinematics
// ============================================================================

std::vector<MuJoCoPiperRegressor::Matrix4d>
MuJoCoPiperRegressor::computeBodyTransforms(const VectorXd &q) const {
  std::vector<Matrix4d> transforms(N_BODIES + 1);

  // link0 in world coordinate frame
  transforms[0] = poseToTransform(bodies_[0].pos, bodies_[0].quat);

  std::size_t joint_idx = 0;

  for (std::size_t i = 1; i <= N_BODIES; ++i) {
    const auto &body = bodies_[i];

    // Base transform relative to parent body
    Matrix4d T_parent_body = poseToTransform(body.pos, body.quat);

    // If joint exists, add joint rotation
    if (body.has_joint && joint_idx < N_DOF) {
      double angle = q(joint_idx);
      Quaterniond joint_rot =
          Quaterniond(Eigen::AngleAxisd(angle, body.joint_axis));

      Matrix4d T_joint = Matrix4d::Identity();
      T_joint.block<3, 3>(0, 0) = joint_rot.toRotationMatrix();

      transforms[i] = transforms[i - 1] * T_parent_body * T_joint;
      ++joint_idx;
    } else {
      transforms[i] = transforms[i - 1] * T_parent_body;
    }
  }

  return transforms;
}

MuJoCoPiperRegressor::Vector3d
MuJoCoPiperRegressor::computeBodyCOM(std::size_t body_idx, const VectorXd &q) const {
  auto transforms = computeBodyTransforms(q);
  Vector3d com_local = bodies_[body_idx].com;
  Matrix4d T = transforms[body_idx];
  return T.block<3, 3>(0, 0) * com_local + T.block<3, 1>(0, 3);
}

/**
 * @brief Compute body origin Jacobian (for regressor matrix)
 *
 * Note: this is different from computeBodyJacobian!
 * - computeBodyJacobian: computes COM Jacobian (for dynamics)
 * - computeBodyOriginJacobian: computes Body origin Jacobian (for regressor)
 *
 * Regressor uses standard inertial parameters (m, mc, I_origin) defined at body origin
 */
MuJoCoPiperRegressor::MatrixXd
MuJoCoPiperRegressor::computeBodyOriginJacobian(std::size_t body_idx,
                                           const VectorXd &q) const {
  MatrixXd J = MatrixXd::Zero(6, N_DOF);

  auto transforms = computeBodyTransforms(q);

  // Body origin position (not COM!)
  Vector3d p_origin = transforms[body_idx].block<3, 1>(0, 3);

  // For each joint
  std::size_t joint_count = 0;
  for (std::size_t i = 1; i <= body_idx && i <= N_BODIES; ++i) {
    if (bodies_[i].has_joint) {
      // Joint axis direction in world frame
      Vector3d z_axis = transforms[i].block<3, 3>(0, 0) * bodies_[i].joint_axis;

      // Joint origin position
      Vector3d p_joint = transforms[i].block<3, 1>(0, 3);

      // Linear velocity Jacobian: z x (p_origin - p_joint)
      J.block<3, 1>(0, joint_count) = z_axis.cross(p_origin - p_joint);

      // Angular velocity Jacobian: z
      J.block<3, 1>(3, joint_count) = z_axis;

      ++joint_count;
    }
  }

  return J;
}

MuJoCoPiperRegressor::MatrixXd
MuJoCoPiperRegressor::computeBodyJacobian(std::size_t body_idx,
                                      const VectorXd &q) const {
  MatrixXd J = MatrixXd::Zero(6, N_DOF);

  auto transforms = computeBodyTransforms(q);
  Vector3d p_body = computeBodyCOM(body_idx, q);

  // For each joint
  std::size_t joint_count = 0;
  for (std::size_t i = 1; i <= body_idx && i <= N_BODIES; ++i) {
    if (bodies_[i].has_joint) {
      // Joint axis direction in world frame
      Vector3d z_axis = transforms[i].block<3, 3>(0, 0) * bodies_[i].joint_axis;

      // Joint origin position
      Vector3d p_joint = transforms[i].block<3, 1>(0, 3);

      // Linear velocity Jacobian: z x (p_body - p_joint)
      J.block<3, 1>(0, joint_count) = z_axis.cross(p_body - p_joint);

      // Angular velocity Jacobian: z
      J.block<3, 1>(3, joint_count) = z_axis;

      ++joint_count;
    }
  }

  return J;
}

MuJoCoPiperRegressor::MatrixXd MuJoCoPiperRegressor::computeBodyOriginJacobianDerivative(
    std::size_t body_idx, const VectorXd &q, const VectorXd &qd) const {
  // Numerical differentiation
  const double eps = 1e-7;
  VectorXd q_plus = q + eps * qd;
  VectorXd q_minus = q - eps * qd;

  MatrixXd J_plus = computeBodyOriginJacobian(body_idx, q_plus);
  MatrixXd J_minus = computeBodyOriginJacobian(body_idx, q_minus);

  return (J_plus - J_minus) / (2.0 * eps);
}

MuJoCoPiperRegressor::MatrixXd MuJoCoPiperRegressor::computeBodyJacobianDerivative(
    std::size_t body_idx, const VectorXd &q, const VectorXd &qd) const {
  // Numerical differentiation
  const double eps = 1e-7;
  VectorXd q_plus = q + eps * qd;
  VectorXd q_minus = q - eps * qd;

  MatrixXd J_plus = computeBodyJacobian(body_idx, q_plus);
  MatrixXd J_minus = computeBodyJacobian(body_idx, q_minus);

  return (J_plus - J_minus) / (2.0 * eps);
}

// ============================================================================
// Parameter vector
// ============================================================================

std::size_t MuJoCoPiperRegressor::numParameters(MuJoCoParamFlags flags) const {
  std::size_t params = N_BODIES * MuJoCoInertialParams::PARAMS_PER_BODY;

  if (hasFlag(flags, MuJoCoParamFlags::ARMATURE)) {
    params += N_DOF;
  }

  if (hasFlag(flags, MuJoCoParamFlags::DAMPING)) {
    params += N_DOF;
  }

  return params;
}

MuJoCoPiperRegressor::VectorXd
MuJoCoPiperRegressor::computeParameterVector(MuJoCoParamFlags flags) const {
  const std::size_t num_params = numParameters(flags);
  VectorXd theta = VectorXd::Zero(num_params);

  // Fill inertial parameters (link1-6)
  for (std::size_t i = 0; i < N_BODIES; ++i) {
    std::size_t body_idx = i + 1;
    auto sip = MuJoCoInertialParams::fromMuJoCoBody(bodies_[body_idx]);
    auto sip_vec = sip.toVector();

    std::size_t offset = i * MuJoCoInertialParams::PARAMS_PER_BODY;
    theta.segment(offset, MuJoCoInertialParams::PARAMS_PER_BODY) = sip_vec;
  }

  std::size_t current_offset = N_BODIES * MuJoCoInertialParams::PARAMS_PER_BODY;

  // Armature parameters
  if (hasFlag(flags, MuJoCoParamFlags::ARMATURE)) {
    for (std::size_t i = 0; i < N_DOF; ++i) {
      theta(current_offset + i) = bodies_[i + 1].armature;
    }
    current_offset += N_DOF;
  }

  // Damping parameters
  if (hasFlag(flags, MuJoCoParamFlags::DAMPING)) {
    for (std::size_t i = 0; i < N_DOF; ++i) {
      theta(current_offset + i) = bodies_[i + 1].damping;
    }
  }

  return theta;
}

std::vector<std::string>
MuJoCoPiperRegressor::getParameterNames(MuJoCoParamFlags flags) const {
  std::vector<std::string> names;

  const char *body_names[] = {"link1", "link2", "link3",
                              "link4", "link5", "link6"};
  const char *param_names[] = {"m",   "mx",  "my",  "mz",  "Ixx",
                               "Ixy", "Ixz", "Iyy", "Iyz", "Izz"};

  for (std::size_t i = 0; i < N_BODIES; ++i) {
    for (int j = 0; j < 10; ++j) {
      names.push_back(std::string(body_names[i]) + "_" + param_names[j]);
    }
  }

  if (hasFlag(flags, MuJoCoParamFlags::ARMATURE)) {
    for (std::size_t i = 0; i < N_DOF; ++i) {
      names.push_back("armature_" + std::to_string(i + 1));
    }
  }

  if (hasFlag(flags, MuJoCoParamFlags::DAMPING)) {
    for (std::size_t i = 0; i < N_DOF; ++i) {
      names.push_back("damping_" + std::to_string(i + 1));
    }
  }

  return names;
}

// ============================================================================
// Regressor matrix
// ============================================================================

MuJoCoPiperRegressor::MatrixXd MuJoCoPiperRegressor::computeBodyRegressorBlock(
    std::size_t body_idx, const VectorXd &q, const VectorXd &qd,
    const VectorXd &qdd) const {

  /**
   * Standard inertial parameters regressor matrix derivation (Body local frame)
   *
   * Parameter vector: theta = [m, mx, my, mz, Ixx, Ixy, Ixz, Iyy, Iyz, Izz]
   * - (mx, my, mz) = m * (cx, cy, cz) are first moments (local frame)
   * - I_origin is inertia tensor at Body origin (local frame)
   *
   * Dynamics equations (in Body local frame):
   *   f_local = m * (a_local - g_local) + K * mc_local
   *   n_local = [mc_local]x * (a_local - g_local) + I_origin * alpha_local +
   * [omega_local]x * I_origin * omega_local
   *
   * where K = [alpha_local]x + [omega_local]x * [omega_local]x
   *
   * Joint torque: τ = J_v_local^T * f_local + J_w_local^T * n_local
   */

  auto transforms = computeBodyTransforms(q);
  Matrix3d R = transforms[body_idx].block<3, 3>(0, 0); // Body to World rotation
  Vector3d p_origin = transforms[body_idx].block<3, 1>(0, 3); // Body origin position

  // ========== Compute Body origin Jacobian (World Frame) ==========
  MatrixXd J_world = MatrixXd::Zero(6, N_DOF);
  std::size_t joint_count = 0;
  for (std::size_t i = 1; i <= body_idx && i <= N_BODIES; ++i) {
    if (bodies_[i].has_joint) {
      Vector3d z_axis = transforms[i].block<3, 3>(0, 0) * bodies_[i].joint_axis;
      Vector3d p_joint = transforms[i].block<3, 1>(0, 3);
      J_world.block<3, 1>(0, joint_count) = z_axis.cross(p_origin - p_joint);
      J_world.block<3, 1>(3, joint_count) = z_axis;
      ++joint_count;
    }
  }

  // Jacobian derivative (numerical differentiation)
  const double dt = 1e-7;
  VectorXd q_plus = q + dt * qd;
  auto transforms_plus = computeBodyTransforms(q_plus);
  Vector3d p_origin_plus = transforms_plus[body_idx].block<3, 1>(0, 3);

  MatrixXd J_world_plus = MatrixXd::Zero(6, N_DOF);
  joint_count = 0;
  for (std::size_t i = 1; i <= body_idx && i <= N_BODIES; ++i) {
    if (bodies_[i].has_joint) {
      Vector3d z_axis =
          transforms_plus[i].block<3, 3>(0, 0) * bodies_[i].joint_axis;
      Vector3d p_joint = transforms_plus[i].block<3, 1>(0, 3);
      J_world_plus.block<3, 1>(0, joint_count) =
          z_axis.cross(p_origin_plus - p_joint);
      J_world_plus.block<3, 1>(3, joint_count) = z_axis;
      ++joint_count;
    }
  }
  MatrixXd J_world_dot = (J_world_plus - J_world) / dt;

  Eigen::Matrix<double, 3, Eigen::Dynamic> Jv_world = J_world.topRows(3);
  Eigen::Matrix<double, 3, Eigen::Dynamic> Jw_world = J_world.bottomRows(3);
  Eigen::Matrix<double, 3, Eigen::Dynamic> Jv_dot_world =
      J_world_dot.topRows(3);
  Eigen::Matrix<double, 3, Eigen::Dynamic> Jw_dot_world =
      J_world_dot.bottomRows(3);

  // ========== Kinematic quantities in World Frame ==========
  Vector3d a_origin_world = Jv_world * qdd + Jv_dot_world * qd;
  Vector3d omega_world = Jw_world * qd;
  Vector3d alpha_world = Jw_world * qdd + Jw_dot_world * qd;

  // ========== Transform to Body Local Frame ==========
  Matrix3d Rt = R.transpose();
  Vector3d a_local = Rt * a_origin_world;
  Vector3d omega_local = Rt * omega_world;
  Vector3d alpha_local = Rt * alpha_world;
  Vector3d g_local = Rt * gravity_;
  Vector3d b_local = a_local - g_local;

  // Jacobians in local frame
  Eigen::Matrix<double, 3, Eigen::Dynamic> Jv_local = Rt * Jv_world;
  Eigen::Matrix<double, 3, Eigen::Dynamic> Jw_local = Rt * Jw_world;

  // ========== Construct regressor matrix block ==========
  MatrixXd Y_block = MatrixXd::Zero(N_DOF, 10);

  // K matrix: K = [α]× + [ω]× * [ω]×
  Matrix3d alpha_skew = skew(alpha_local);
  Matrix3d omega_skew = skew(omega_local);
  Matrix3d K = alpha_skew + omega_skew * omega_skew;

  double ox = omega_local(0), oy = omega_local(1), oz = omega_local(2);
  double ax = alpha_local(0), ay = alpha_local(1), az = alpha_local(2);

  // 1. Mass m: f = m * b, n = 0
  Y_block.col(0) = Jv_local.transpose() * b_local;

  // 2. First moments (mx, my, mz): f = K * e_i, n = e_i × b
  for (int i = 0; i < 3; ++i) {
    Vector3d e_i = Vector3d::Zero();
    e_i(i) = 1.0;
    Y_block.col(1 + i) = Jv_local.transpose() * K.col(i) +
                         Jw_local.transpose() * e_i.cross(b_local);
  }

  // 3. Inertia tensor: n = E * α + [ω]× * E * ω
  Y_block.col(4) = Jw_local.transpose() * Vector3d(ax, ox * oz, -ox * oy);
  Y_block.col(5) = Jw_local.transpose() *
                   Vector3d(ay - ox * oz, ax + oy * oz, ox * ox - oy * oy);
  Y_block.col(6) = Jw_local.transpose() *
                   Vector3d(az + ox * oy, oz * oz - ox * ox, ax - oy * oz);
  Y_block.col(7) = Jw_local.transpose() * Vector3d(-oy * oz, ay, ox * oy);
  Y_block.col(8) = Jw_local.transpose() *
                   Vector3d(oy * oy - oz * oz, az - ox * oy, ay + ox * oz);
  Y_block.col(9) = Jw_local.transpose() * Vector3d(oy * oz, -ox * oz, az);

  return Y_block;
}

MuJoCoPiperRegressor::MatrixXd
MuJoCoPiperRegressor::computeRegressorMatrix(const VectorXd &q, const VectorXd &qd,
                                        const VectorXd &qdd,
                                        MuJoCoParamFlags flags) const {

  const std::size_t num_params = numParameters(flags);
  MatrixXd Y = MatrixXd::Zero(N_DOF, num_params);

  // Compute regressor matrix block for each body
  for (std::size_t i = 0; i < N_BODIES; ++i) {
    std::size_t body_idx = i + 1;
    MatrixXd Y_body = computeBodyRegressorBlock(body_idx, q, qd, qdd);

    std::size_t offset = i * MuJoCoInertialParams::PARAMS_PER_BODY;
    Y.block(0, offset, N_DOF, MuJoCoInertialParams::PARAMS_PER_BODY) = Y_body;
  }

  std::size_t current_offset = N_BODIES * MuJoCoInertialParams::PARAMS_PER_BODY;

  // Armature regression: τ_armature = armature * q̈
  if (hasFlag(flags, MuJoCoParamFlags::ARMATURE)) {
    for (std::size_t i = 0; i < N_DOF; ++i) {
      Y(i, current_offset + i) = qdd(i);
    }
    current_offset += N_DOF;
  }

  // Damping regression: in MuJoCo, damping is a passive term
  // τ_ctrl = M*q̈ + C*q̇ + G - damping*q̇
  // Therefore the regression term is -q̇
  if (hasFlag(flags, MuJoCoParamFlags::DAMPING)) {
    for (std::size_t i = 0; i < N_DOF; ++i) {
      Y(i, current_offset + i) = -qd(i);
    }
  }

  return Y;
}

MuJoCoPiperRegressor::MatrixXd
MuJoCoPiperRegressor::computeObservationMatrix(const MatrixXd &Q, const MatrixXd &Qd,
                                          const MatrixXd &Qdd,
                                          MuJoCoParamFlags flags) const {

  const std::size_t K = Q.cols(); // Number of samples
  const std::size_t num_params = numParameters(flags);

  MatrixXd W = MatrixXd::Zero(N_DOF * K, num_params);

#ifdef IDENTIFICATION_USE_OPENMP
#pragma omp parallel for schedule(static)
#endif
  for (Eigen::Index k = 0; k < static_cast<Eigen::Index>(K); ++k) {
    VectorXd q = Q.col(k);
    VectorXd qd = Qd.col(k);
    VectorXd qdd = Qdd.col(k);

    MatrixXd Y_k = computeRegressorMatrix(q, qd, qdd, flags);
    W.block(k * static_cast<Eigen::Index>(N_DOF), 0,
            static_cast<Eigen::Index>(N_DOF),
            static_cast<Eigen::Index>(num_params)) = Y_k;
  }

  return W;
}

} // namespace mujoco_dynamics
