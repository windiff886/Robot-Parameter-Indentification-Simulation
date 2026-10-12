/**
 * @file mujoco_regressor.cpp
 * @brief Implementation of regressor matrix computation based on MuJoCo kinematics
 */

#include "mujoco_regressor.hpp"
#include <iostream>

namespace mujoco_dynamics {

// ============================================================================
// MuJoCoInertialParams
// ============================================================================

MuJoCoInertialParams
MuJoCoInertialParams::fromMuJoCoBody(const MuJoCoBody &body) {
  MuJoCoInertialParams params;

  params.m = body.mass;

  // First moments: m * c
  params.mx = body.mass * body.com.x();
  params.my = body.mass * body.com.y();
  params.mz = body.mass * body.com.z();

  // Inertia tensor transformed from COM to Body origin (parallel axis theorem)
  // I_origin = I_com + m * (c^T * c * I - c * c^T)
  double cx = body.com.x();
  double cy = body.com.y();
  double cz = body.com.z();
  double m = body.mass;

  params.Ixx = body.Ixx + m * (cy * cy + cz * cz);
  params.Iyy = body.Iyy + m * (cx * cx + cz * cz);
  params.Izz = body.Izz + m * (cx * cx + cy * cy);
  params.Ixy = body.Ixy - m * cx * cy;
  params.Ixz = body.Ixz - m * cx * cz;
  params.Iyz = body.Iyz - m * cy * cz;

  return params;
}

// ============================================================================
// MuJoCoRegressor constructor
// ============================================================================

MuJoCoRegressor::MuJoCoRegressor() { initBodies(); }

void MuJoCoRegressor::initBodies() {
  // Copy body parameters from MuJoCoPandaDynamics (from panda.xml)

  // Link 0 (base, no joint)
  bodies_[0].name = "link0";
  bodies_[0].pos = Vector3d(0, 0, 0);
  bodies_[0].quat = Quaterniond(1, 0, 0, 0);
  bodies_[0].mass = 0.629769;
  bodies_[0].com = Vector3d(-0.041018, -0.00014, 0.049974);
  bodies_[0].Ixx = 0.00315;
  bodies_[0].Iyy = 0.00388;
  bodies_[0].Izz = 0.004285;
  bodies_[0].Ixy = 8.2904e-7;
  bodies_[0].Ixz = 0.00015;
  bodies_[0].Iyz = 8.2299e-6;
  bodies_[0].has_joint = false;

  // Link 1
  bodies_[1].name = "link1";
  bodies_[1].pos = Vector3d(0, 0, 0.333);
  bodies_[1].quat = Quaterniond(1, 0, 0, 0);
  bodies_[1].mass = 4.970684;
  bodies_[1].com = Vector3d(0.003875, 0.002081, -0.04762);
  bodies_[1].Ixx = 0.70337;
  bodies_[1].Iyy = 0.70661;
  bodies_[1].Izz = 0.0091170;
  bodies_[1].Ixy = -0.00013900;
  bodies_[1].Ixz = 0.0067720;
  bodies_[1].Iyz = 0.019169;
  bodies_[1].joint_axis = Vector3d(0, 0, 1);
  bodies_[1].armature = 0.1;
  bodies_[1].damping = 1.0;
  bodies_[1].has_joint = true;

  // Link 2
  bodies_[2].name = "link2";
  bodies_[2].pos = Vector3d(0, 0, 0);
  bodies_[2].quat = Quaterniond(1, -1, 0, 0).normalized();
  bodies_[2].mass = 0.646926;
  bodies_[2].com = Vector3d(-0.003141, -0.02872, 0.003495);
  bodies_[2].Ixx = 0.0079620;
  bodies_[2].Iyy = 2.8110e-2;
  bodies_[2].Izz = 2.5995e-2;
  bodies_[2].Ixy = -3.925e-3;
  bodies_[2].Ixz = 1.0254e-2;
  bodies_[2].Iyz = 7.04e-4;
  bodies_[2].joint_axis = Vector3d(0, 0, 1);
  bodies_[2].armature = 0.1;
  bodies_[2].damping = 1.0;
  bodies_[2].has_joint = true;

  // Link 3
  bodies_[3].name = "link3";
  bodies_[3].pos = Vector3d(0, -0.316, 0);
  bodies_[3].quat = Quaterniond(1, 1, 0, 0).normalized();
  bodies_[3].mass = 3.228604;
  bodies_[3].com = Vector3d(2.7518e-2, 3.9252e-2, -6.6502e-2);
  bodies_[3].Ixx = 3.7242e-2;
  bodies_[3].Iyy = 3.6155e-2;
  bodies_[3].Izz = 1.083e-2;
  bodies_[3].Ixy = -4.761e-3;
  bodies_[3].Ixz = -1.1396e-2;
  bodies_[3].Iyz = -1.2805e-2;
  bodies_[3].joint_axis = Vector3d(0, 0, 1);
  bodies_[3].armature = 0.1;
  bodies_[3].damping = 1.0;
  bodies_[3].has_joint = true;

  // Link 4
  bodies_[4].name = "link4";
  bodies_[4].pos = Vector3d(0.0825, 0, 0);
  bodies_[4].quat = Quaterniond(1, 1, 0, 0).normalized();
  bodies_[4].mass = 3.587895;
  bodies_[4].com = Vector3d(-5.317e-2, 1.04419e-1, 2.7454e-2);
  bodies_[4].Ixx = 2.5853e-2;
  bodies_[4].Iyy = 1.9552e-2;
  bodies_[4].Izz = 2.8323e-2;
  bodies_[4].Ixy = 7.796e-3;
  bodies_[4].Ixz = -1.332e-3;
  bodies_[4].Iyz = 8.641e-3;
  bodies_[4].joint_axis = Vector3d(0, 0, 1);
  bodies_[4].armature = 0.1;
  bodies_[4].damping = 1.0;
  bodies_[4].has_joint = true;

  // Link 5
  bodies_[5].name = "link5";
  bodies_[5].pos = Vector3d(-0.0825, 0.384, 0);
  bodies_[5].quat = Quaterniond(1, -1, 0, 0).normalized();
  bodies_[5].mass = 1.225946;
  bodies_[5].com = Vector3d(-1.1953e-2, 4.1065e-2, -3.8437e-2);
  bodies_[5].Ixx = 3.5549e-2;
  bodies_[5].Iyy = 2.9474e-2;
  bodies_[5].Izz = 8.627e-3;
  bodies_[5].Ixy = -2.117e-3;
  bodies_[5].Ixz = -4.037e-3;
  bodies_[5].Iyz = 2.29e-4;
  bodies_[5].joint_axis = Vector3d(0, 0, 1);
  bodies_[5].armature = 0.1;
  bodies_[5].damping = 1.0;
  bodies_[5].has_joint = true;

  // Link 6
  bodies_[6].name = "link6";
  bodies_[6].pos = Vector3d(0, 0, 0);
  bodies_[6].quat = Quaterniond(1, 1, 0, 0).normalized();
  bodies_[6].mass = 1.666555;
  bodies_[6].com = Vector3d(6.0149e-2, -1.4117e-2, -1.0517e-2);
  bodies_[6].Ixx = 1.964e-3;
  bodies_[6].Iyy = 4.354e-3;
  bodies_[6].Izz = 5.433e-3;
  bodies_[6].Ixy = 1.09e-4;
  bodies_[6].Ixz = -1.158e-3;
  bodies_[6].Iyz = 3.41e-4;
  bodies_[6].joint_axis = Vector3d(0, 0, 1);
  bodies_[6].armature = 0.1;
  bodies_[6].damping = 1.0;
  bodies_[6].has_joint = true;

  // Link 7
  bodies_[7].name = "link7";
  bodies_[7].pos = Vector3d(0.088, 0, 0);
  bodies_[7].quat = Quaterniond(1, 1, 0, 0).normalized();
  bodies_[7].mass = 7.35522e-01;
  bodies_[7].com = Vector3d(1.0517e-2, -4.252e-3, 6.1597e-2);
  bodies_[7].Ixx = 1.2516e-2;
  bodies_[7].Iyy = 1.0027e-2;
  bodies_[7].Izz = 4.815e-3;
  bodies_[7].Ixy = -4.28e-4;
  bodies_[7].Ixz = -1.196e-3;
  bodies_[7].Iyz = -7.41e-4;
  bodies_[7].joint_axis = Vector3d(0, 0, 1);
  bodies_[7].armature = 0.1;
  bodies_[7].damping = 1.0;
  bodies_[7].has_joint = true;

  // Hand (fixed connection to link7)
  bodies_[8].name = "hand";
  bodies_[8].pos = Vector3d(0, 0, 0.107);
  bodies_[8].quat = Quaterniond(0.9238795, 0, 0, -0.3826834);
  bodies_[8].mass = 0.73;
  bodies_[8].com = Vector3d(-0.01, 0, 0.03);
  bodies_[8].Ixx = 0.001;
  bodies_[8].Iyy = 0.0025;
  bodies_[8].Izz = 0.0017;
  bodies_[8].Ixy = 0;
  bodies_[8].Ixz = 0;
  bodies_[8].Iyz = 0;
  bodies_[8].has_joint = false;

  // Fingers (not currently included)
  bodies_[9].name = "left_finger";
  bodies_[9].has_joint = false;
  bodies_[10].name = "right_finger";
  bodies_[10].has_joint = false;
}

// ============================================================================
// Helper functions
// ============================================================================

MuJoCoRegressor::Matrix3d MuJoCoRegressor::skew(const Vector3d &v) {
  Matrix3d S;
  S << 0, -v(2), v(1), v(2), 0, -v(0), -v(1), v(0), 0;
  return S;
}

MuJoCoRegressor::Matrix4d
MuJoCoRegressor::poseToTransform(const Vector3d &pos, const Quaterniond &quat) {
  Matrix4d T = Matrix4d::Identity();
  T.block<3, 3>(0, 0) = quat.toRotationMatrix();
  T.block<3, 1>(0, 3) = pos;
  return T;
}

// ============================================================================
// Kinematics
// ============================================================================

std::vector<MuJoCoRegressor::Matrix4d>
MuJoCoRegressor::computeBodyTransforms(const VectorXd &q) const {
  std::vector<Matrix4d> transforms(11); // N_BODIES

  // link0 in world coordinates
  transforms[0] = poseToTransform(bodies_[0].pos, bodies_[0].quat);

  std::size_t joint_idx = 0;

  for (std::size_t i = 1; i < 8; ++i) { // link1-7
    const auto &body = bodies_[i];

    // Base transform relative to parent body
    Matrix4d T_parent_body = poseToTransform(body.pos, body.quat);

    // If has joint, add joint rotation
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

  // hand fixed connection to link7
  transforms[8] =
      transforms[7] * poseToTransform(bodies_[8].pos, bodies_[8].quat);

  return transforms;
}

MuJoCoRegressor::Vector3d
MuJoCoRegressor::computeBodyCOM(std::size_t body_idx, const VectorXd &q) const {
  auto transforms = computeBodyTransforms(q);
  Vector3d com_local = bodies_[body_idx].com;
  Matrix4d T = transforms[body_idx];
  return T.block<3, 3>(0, 0) * com_local + T.block<3, 1>(0, 3);
}

/**
 * @brief Compute Jacobian matrix at Body origin (for regressor matrix)
 *
 * Note: This is different from computeBodyJacobian!
 * - computeBodyJacobian: Computes COM Jacobian (for dynamics)
 * - computeBodyOriginJacobian: Computes Body origin Jacobian (for regressor matrix)
 *
 * Regressor matrix uses standard inertial parameters (m, mc, I_origin), with inertia defined at Body origin
 */
MuJoCoRegressor::MatrixXd
MuJoCoRegressor::computeBodyOriginJacobian(std::size_t body_idx,
                                           const VectorXd &q) const {
  MatrixXd J = MatrixXd::Zero(6, N_DOF);

  auto transforms = computeBodyTransforms(q);

  // Body origin position (not COM!)
  Vector3d p_origin = transforms[body_idx].block<3, 1>(0, 3);

  // For each joint
  std::size_t joint_count = 0;
  for (std::size_t i = 1; i <= body_idx && i < 8; ++i) {
    if (bodies_[i].has_joint) {
      // Joint axis direction in world frame
      Vector3d z_axis = transforms[i].block<3, 3>(0, 0) * bodies_[i].joint_axis;

      // Joint origin position
      Vector3d p_joint = transforms[i].block<3, 1>(0, 3);

      // Linear velocity Jacobian: z × (p_origin - p_joint)
      J.block<3, 1>(0, joint_count) = z_axis.cross(p_origin - p_joint);

      // Angular velocity Jacobian: z
      J.block<3, 1>(3, joint_count) = z_axis;

      ++joint_count;
    }
  }

  return J;
}

MuJoCoRegressor::MatrixXd
MuJoCoRegressor::computeBodyJacobian(std::size_t body_idx,
                                     const VectorXd &q) const {
  MatrixXd J = MatrixXd::Zero(6, N_DOF);

  auto transforms = computeBodyTransforms(q);
  Vector3d p_body = computeBodyCOM(body_idx, q);

  // For each joint
  std::size_t joint_count = 0;
  for (std::size_t i = 1; i <= body_idx && i < 8; ++i) {
    if (bodies_[i].has_joint) {
      // Joint axis direction in world frame
      Vector3d z_axis = transforms[i].block<3, 3>(0, 0) * bodies_[i].joint_axis;

      // Joint origin position
      Vector3d p_joint = transforms[i].block<3, 1>(0, 3);

      // Linear velocity Jacobian: z × (p_body - p_joint)
      J.block<3, 1>(0, joint_count) = z_axis.cross(p_body - p_joint);

      // Angular velocity Jacobian: z
      J.block<3, 1>(3, joint_count) = z_axis;

      ++joint_count;
    }
  }

  return J;
}

MuJoCoRegressor::MatrixXd MuJoCoRegressor::computeBodyOriginJacobianDerivative(
    std::size_t body_idx, const VectorXd &q, const VectorXd &qd) const {
  // Numerical differentiation
  const double eps = 1e-7;
  VectorXd q_plus = q + eps * qd;
  VectorXd q_minus = q - eps * qd;

  MatrixXd J_plus = computeBodyOriginJacobian(body_idx, q_plus);
  MatrixXd J_minus = computeBodyOriginJacobian(body_idx, q_minus);

  return (J_plus - J_minus) / (2.0 * eps);
}

MuJoCoRegressor::MatrixXd MuJoCoRegressor::computeBodyJacobianDerivative(
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

std::size_t MuJoCoRegressor::numParameters(MuJoCoParamFlags flags) const {
  // 8 bodies (link1-7 + hand), each with 10 inertial parameters
  std::size_t params = N_BODIES * MuJoCoInertialParams::PARAMS_PER_BODY;

  if (hasFlag(flags, MuJoCoParamFlags::ARMATURE)) {
    params += N_DOF;
  }

  if (hasFlag(flags, MuJoCoParamFlags::DAMPING)) {
    params += N_DOF;
  }

  return params;
}

MuJoCoRegressor::VectorXd
MuJoCoRegressor::computeParameterVector(MuJoCoParamFlags flags) const {
  const std::size_t num_params = numParameters(flags);
  VectorXd theta = VectorXd::Zero(num_params);

  // Fill inertial parameters (link1-7 + hand)
  for (std::size_t i = 0; i < N_BODIES; ++i) {
    std::size_t body_idx = i + 1; // bodies_[1] to bodies_[8]
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
MuJoCoRegressor::getParameterNames(MuJoCoParamFlags flags) const {
  std::vector<std::string> names;

  const char *body_names[] = {"link1", "link2", "link3", "link4",
                              "link5", "link6", "link7", "hand"};
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

MuJoCoRegressor::MatrixXd MuJoCoRegressor::computeBodyRegressorBlock(
    std::size_t body_idx, const VectorXd &q, const VectorXd &qd,
    const VectorXd &qdd) const {

  /**
   * Standard inertial parameter regressor matrix derivation (Body local frame)
   *
   * Parameter vector: θ = [m, mx, my, mz, Ixx, Ixy, Ixz, Iyy, Iyz, Izz]
   * - (mx, my, mz) = m * (cx, cy, cz) are first moments (local frame)
   * - I_origin is inertia tensor at Body origin (local frame)
   *
   * Dynamics equations (in Body local frame):
   *   f_local = m * (a_local - g_local) + K * mc_local
   *   n_local = [mc_local]× * (a_local - g_local) + I_origin * α_local +
   * [ω_local]× * I_origin * ω_local
   *
   * where K = [α_local]× + [ω_local]× * [ω_local]×
   *
   * Joint torque: τ = J_v_local^T * f_local + J_w_local^T * n_local
   */

  auto transforms = computeBodyTransforms(q);
  Matrix3d R = transforms[body_idx].block<3, 3>(0, 0); // Body to World rotation
  Vector3d p_origin = transforms[body_idx].block<3, 1>(0, 3); // Body origin position

  // ========== Compute Body origin Jacobian (World Frame) ==========
  MatrixXd J_world = MatrixXd::Zero(6, N_DOF);
  std::size_t joint_count = 0;
  for (std::size_t i = 1; i <= body_idx && i < 8; ++i) {
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
  for (std::size_t i = 1; i <= body_idx && i < 8; ++i) {
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

MuJoCoRegressor::MatrixXd
MuJoCoRegressor::computeRegressorMatrix(const VectorXd &q, const VectorXd &qd,
                                        const VectorXd &qdd,
                                        MuJoCoParamFlags flags) const {

  const std::size_t num_params = numParameters(flags);
  MatrixXd Y = MatrixXd::Zero(N_DOF, num_params);

  // Compute regressor matrix block for each body
  for (std::size_t i = 0; i < N_BODIES; ++i) {
    std::size_t body_idx = i + 1; // bodies_[1] to bodies_[8]
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

MuJoCoRegressor::MatrixXd
MuJoCoRegressor::computeObservationMatrix(const MatrixXd &Q, const MatrixXd &Qd,
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
