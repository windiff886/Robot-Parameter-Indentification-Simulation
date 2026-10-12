/**
 * @file mujoco_panda_dynamics.cpp
 * @brief MuJoCo Panda dynamics implementation
 */

#include "mujoco_panda_dynamics.hpp"
#include <iostream>

namespace mujoco_dynamics {

MuJoCoPandaDynamics::MuJoCoPandaDynamics() { initBodies(); }

void MuJoCoPandaDynamics::initBodies() {
  // Parameters extracted from panda.xml

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

  // Link 2 (note quat = (1, -1, 0, 0) after normalization)
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

  // Hand (fixed connection to link7, no joint)
  bodies_[8].name = "hand";
  bodies_[8].pos = Vector3d(0, 0, 0.107);
  bodies_[8].quat = Quaterniond(0.9238795, 0, 0, -0.3826834);
  bodies_[8].mass = 0.73;
  bodies_[8].com = Vector3d(-0.01, 0, 0.03);
  // Convert diaginertia to fullinertia
  bodies_[8].Ixx = 0.001;
  bodies_[8].Iyy = 0.0025;
  bodies_[8].Izz = 0.0017;
  bodies_[8].Ixy = 0;
  bodies_[8].Ixz = 0;
  bodies_[8].Iyz = 0;
  bodies_[8].has_joint = false;

  // Fingers temporarily excluded (negligible impact on joint torques)
  bodies_[9].name = "left_finger";
  bodies_[9].has_joint = false;
  bodies_[10].name = "right_finger";
  bodies_[10].has_joint = false;
}

// ============================================================
// Helper functions
// ============================================================

Matrix3d MuJoCoPandaDynamics::skew(const Vector3d &v) {
  Matrix3d S;
  S << 0, -v(2), v(1), v(2), 0, -v(0), -v(1), v(0), 0;
  return S;
}

Matrix4d MuJoCoPandaDynamics::poseToTransform(const Vector3d &pos,
                                              const Quaterniond &quat) {
  Matrix4d T = Matrix4d::Identity();
  T.block<3, 3>(0, 0) = quat.toRotationMatrix();
  T.block<3, 1>(0, 3) = pos;
  return T;
}

// ============================================================
// Forward Kinematics
// ============================================================

std::vector<Matrix4d>
MuJoCoPandaDynamics::computeBodyTransforms(const VectorXd &q) const {
  std::vector<Matrix4d> transforms(N_BODIES);

  // link0 in world coordinate frame
  transforms[0] = poseToTransform(bodies_[0].pos, bodies_[0].quat);

  std::size_t joint_idx = 0;

  for (std::size_t i = 1; i < 8; ++i) { // link1-7
    const auto &body = bodies_[i];

    // Base transform relative to parent body
    Matrix4d T_parent_body = poseToTransform(body.pos, body.quat);

    // If joint exists, add joint rotation
    if (body.has_joint && joint_idx < N_DOF) {
      // Default joint axis in MuJoCo is (0, 0, 1)
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

  // hand is fixed to link7
  transforms[8] =
      transforms[7] * poseToTransform(bodies_[8].pos, bodies_[8].quat);

  return transforms;
}

Vector3d MuJoCoPandaDynamics::computeBodyCOM(std::size_t body_idx,
                                             const VectorXd &q) const {
  auto transforms = computeBodyTransforms(q);

  // COM position in body coordinate frame
  Vector3d com_local = bodies_[body_idx].com;

  // Transform to world coordinate frame
  Matrix4d T = transforms[body_idx];
  Vector3d com_world = T.block<3, 3>(0, 0) * com_local + T.block<3, 1>(0, 3);

  return com_world;
}

// ============================================================
// Jacobian matrices
// ============================================================

MatrixXd MuJoCoPandaDynamics::computeBodyJacobian(std::size_t body_idx,
                                                  const VectorXd &q) const {
  MatrixXd J = MatrixXd::Zero(6, N_DOF);

  auto transforms = computeBodyTransforms(q);
  Vector3d p_body = computeBodyCOM(body_idx, q);

  // For each joint
  std::size_t joint_count = 0;
  for (std::size_t i = 1; i <= body_idx && i < 8; ++i) {
    if (bodies_[i].has_joint) {
      // Joint i axis direction in world frame
      Vector3d z_axis = transforms[i].block<3, 3>(0, 0) * bodies_[i].joint_axis;

      // Joint i origin position
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

void MuJoCoPandaDynamics::printBodyParams() const {
  std::cout << "MuJoCo Panda Bodies:\n";
  for (std::size_t i = 0; i < 8; ++i) {
    const auto &b = bodies_[i];
    std::cout << "  " << b.name << ": mass=" << b.mass
              << ", has_joint=" << b.has_joint << "\n";
  }
}

// ============================================================
// Spatial inertia matrix
// ============================================================

Eigen::Matrix<double, 6, 6>
MuJoCoPandaDynamics::computeSpatialInertia(std::size_t body_idx) const {
  const auto &body = bodies_[body_idx];

  // COM position
  Vector3d c = body.com;
  double m = body.mass;

  // Inertia tensor (at COM, fullinertia format)
  Matrix3d I_com;
  I_com << body.Ixx, body.Ixy, body.Ixz, body.Ixy, body.Iyy, body.Iyz, body.Ixz,
      body.Iyz, body.Izz;

  // Use parallel axis theorem to transform to body frame origin
  // I_origin = I_com - m * [c]x * [c]x
  Matrix3d c_skew = skew(c);
  Matrix3d I_origin = I_com - m * c_skew * c_skew;

  // Construct 6x6 spatial inertia matrix
  Eigen::Matrix<double, 6, 6> M_spatial;
  M_spatial.setZero();
  M_spatial.block<3, 3>(0, 0) = m * Matrix3d::Identity();
  M_spatial.block<3, 3>(0, 3) = m * c_skew.transpose();
  M_spatial.block<3, 3>(3, 0) = m * c_skew;
  M_spatial.block<3, 3>(3, 3) = I_origin;

  return M_spatial;
}

// ============================================================
// Inertia matrix
// ============================================================

MatrixXd MuJoCoPandaDynamics::computeInertiaMatrix(const VectorXd &q) const {
  MatrixXd M = MatrixXd::Zero(N_DOF, N_DOF);

  // M = sum_{i=1}^{7} J_i^T * M_i * J_i
  // Includes link1-7 + hand (hand fixed to link7)

  // link1-7
  for (std::size_t i = 1; i < 8; ++i) {
    MatrixXd J_i = computeBodyJacobian(i, q);
    auto M_i = computeSpatialInertia(i);
    M += J_i.transpose() * M_i * J_i;
  }

  // hand (body 8) is fixed to link7, uses Jacobian of link7
  MatrixXd J_hand = computeBodyJacobian(8, q);
  auto M_hand = computeSpatialInertia(8);
  M += J_hand.transpose() * M_hand * J_hand;

  // Add armature
  for (std::size_t i = 0; i < N_DOF; ++i) {
    M(i, i) += bodies_[i + 1].armature;
  }

  // Ensure symmetry
  M = (M + M.transpose()) / 2.0;

  return M;
}

// ============================================================
// Coriolis matrix
// ============================================================

MatrixXd MuJoCoPandaDynamics::computeCoriolisMatrix(const VectorXd &q,
                                                    const VectorXd &qd) const {
  const double eps = 1e-7;
  MatrixXd C = MatrixXd::Zero(N_DOF, N_DOF);

  // C_ij = sum_k Gamma_ijk * qd_k
  // Gamma_ijk = 0.5 * (dM_ij/dq_k + dM_ik/dq_j - dM_jk/dq_i)

  MatrixXd M = computeInertiaMatrix(q);

  // Numerical differentiation to compute dM/dq
  for (std::size_t k = 0; k < N_DOF; ++k) {
    VectorXd q_plus = q;
    VectorXd q_minus = q;
    q_plus(k) += eps;
    q_minus(k) -= eps;

    MatrixXd dM_dqk =
        (computeInertiaMatrix(q_plus) - computeInertiaMatrix(q_minus)) /
        (2.0 * eps);

    for (std::size_t i = 0; i < N_DOF; ++i) {
      for (std::size_t j = 0; j < N_DOF; ++j) {
        C(i, j) += 0.5 * dM_dqk(i, j) * qd(k);
      }
    }
  }

  return C;
}

// ============================================================
// Gravity vector
// ============================================================

VectorXd MuJoCoPandaDynamics::computeGravityVector(const VectorXd &q) const {
  VectorXd G = VectorXd::Zero(N_DOF);

  // g = sum_{i=1}^{7} J_v,i^T * m_i * gravity

  // link1-7
  for (std::size_t i = 1; i < 8; ++i) {
    MatrixXd J_i = computeBodyJacobian(i, q);
    MatrixXd J_v = J_i.topRows(3); // Linear velocity part

    G -= J_v.transpose() * (bodies_[i].mass * gravity_);
  }

  // hand
  MatrixXd J_hand = computeBodyJacobian(8, q);
  MatrixXd J_v_hand = J_hand.topRows(3);
  G -= J_v_hand.transpose() * (bodies_[8].mass * gravity_);

  return G;
}

// ============================================================
// Inverse dynamics
// ============================================================

VectorXd MuJoCoPandaDynamics::computeInverseDynamics(
    const VectorXd &q, const VectorXd &qd, const VectorXd &qdd) const {
  // MuJoCo dynamics equation:
  // tau = (M + diag(I_a)) * qdd + C * qd + g - damping * qd
  //
  // Note:
  // - M already includes armature
  // - damping is a passive term, subtracted (helps motion)

  MatrixXd M = computeInertiaMatrix(q); // Already includes armature
  MatrixXd C = computeCoriolisMatrix(q, qd);
  VectorXd g = computeGravityVector(q);

  // Damping term
  VectorXd damping_force = VectorXd::Zero(N_DOF);
  for (std::size_t i = 0; i < N_DOF; ++i) {
    damping_force(i) = bodies_[i + 1].damping * qd(i);
  }

  VectorXd tau = M * qdd + C * qd + g - damping_force;

  return tau;
}

} // namespace mujoco_dynamics
