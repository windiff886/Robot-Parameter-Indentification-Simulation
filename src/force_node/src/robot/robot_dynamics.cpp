/**
 * @file robot_dynamics.cpp
 * @brief Robot dynamics computation implementation
 */

#include "robot/robot_dynamics.hpp"
#include <cmath>

namespace robot {

RobotDynamics::RobotDynamics(const RobotModel &model)
    : model_(model), kinematics_(model) {}

// ============================================================================
// Spatial Inertia
// ============================================================================

Eigen::Matrix<double, 6, 6>
RobotDynamics::computeSpatialInertia(std::size_t link_idx) const {
  const auto &link = model_.link(link_idx);

  const double m = link.mass;
  const Vector3d c(link.com[0], link.com[1], link.com[2]);

  // Inertia tensor at COM
  Matrix3d I_com;
  I_com << link.inertia.Ixx, link.inertia.Ixy, link.inertia.Ixz,
      link.inertia.Ixy, link.inertia.Iyy, link.inertia.Iyz, link.inertia.Ixz,
      link.inertia.Iyz, link.inertia.Izz;

  // Transform inertia to link origin using parallel axis theorem
  // I_origin = I_com + m * (c^T c * I - c * c^T)
  Matrix3d c_skew = RobotKinematics::skew(c);
  Matrix3d I_origin = I_com - m * c_skew * c_skew;

  // 6x6 spatial inertia matrix
  Eigen::Matrix<double, 6, 6> M_spatial;
  M_spatial.setZero();
  M_spatial.block<3, 3>(0, 0) = m * Matrix3d::Identity();
  M_spatial.block<3, 3>(0, 3) = m * c_skew.transpose();
  M_spatial.block<3, 3>(3, 0) = m * c_skew;
  M_spatial.block<3, 3>(3, 3) = I_origin;

  return M_spatial;
}

// ============================================================================
// Inertia Matrix
// ============================================================================

RobotDynamics::MatrixXd
RobotDynamics::computeInertiaMatrix(const VectorXd &q) const {
  const std::size_t n = numDOF();
  MatrixXd M = MatrixXd::Zero(n, n);

  // M = sum_{i=1}^{n} J_i^T * M_i * J_i
  for (std::size_t i = 0; i < n; ++i) {
    // Get Jacobian for link i's COM
    MatrixXd J_i = kinematics_.computeLinkJacobian(i, q);

    // Get spatial inertia for link i+1 (skip base link)
    auto M_i = computeSpatialInertia(i + 1);

    // Add contribution to inertia matrix
    M += J_i.transpose() * M_i * J_i;
  }

  // Ensure symmetry
  M = (M + M.transpose()) / 2.0;

  return M;
}

// ============================================================================
// Coriolis Matrix
// ============================================================================

RobotDynamics::MatrixXd
RobotDynamics::computeCoriolisMatrix(const VectorXd &q,
                                     const VectorXd &qd) const {
  const std::size_t n = numDOF();
  MatrixXd C = MatrixXd::Zero(n, n);

  // Compute Coriolis matrix using Christoffel symbols
  // C_ij = sum_k Gamma_ijk * qd_k
  // Gamma_ijk = 0.5 * (dM_ij/dq_k + dM_ik/dq_j - dM_jk/dq_i)

  const double eps = 1e-7;

  // Compute M(q)
  MatrixXd M = computeInertiaMatrix(q);

  // Compute partial derivatives of M
  for (std::size_t k = 0; k < n; ++k) {
    VectorXd q_plus = q;
    VectorXd q_minus = q;
    q_plus(k) += eps;
    q_minus(k) -= eps;

    MatrixXd dM_dqk =
        (computeInertiaMatrix(q_plus) - computeInertiaMatrix(q_minus)) /
        (2.0 * eps);

    for (std::size_t i = 0; i < n; ++i) {
      for (std::size_t j = 0; j < n; ++j) {
        // Christoffel symbol of the first kind
        // This is a simplified computation; full version needs all partial
        // derivatives
        C(i, j) += 0.5 * dM_dqk(i, j) * qd(k);
      }
    }
  }

  return C;
}

// ============================================================================
// Gravity Vector
// ============================================================================

RobotDynamics::VectorXd
RobotDynamics::computeGravityVector(const VectorXd &q) const {
  const std::size_t n = numDOF();
  VectorXd G = VectorXd::Zero(n);

  // Gravity vector in world frame
  const auto &g_world = model_.gravity();
  Eigen::Vector3d g(g_world[0], g_world[1], g_world[2]);

  // G = sum_{i=1}^{n} J_{v,i}^T * m_i * g
  for (std::size_t i = 0; i < n; ++i) {
    const auto &link = model_.link(i + 1); // Skip base link

    // Get linear Jacobian for link i's COM
    MatrixXd J_i = kinematics_.computeLinkJacobian(i, q);
    Eigen::Matrix<double, 3, Eigen::Dynamic> J_v = J_i.topRows(3);

    // Add gravity contribution
    G -= J_v.transpose() * (link.mass * g);
  }

  return G;
}

// ============================================================================
// Friction Torque
// ============================================================================

RobotDynamics::VectorXd
RobotDynamics::computeFrictionTorque(const VectorXd &qd) const {
  const std::size_t n = numDOF();
  VectorXd F = VectorXd::Zero(n);

  const auto &friction = model_.frictionParameters();

  for (std::size_t i = 0; i < n; ++i) {
    // Viscous + Coulomb friction: F = Fv * qd + Fc * sign(qd)
    const double Fv = friction[i].Fv;
    const double Fc = friction[i].Fc;

    F(i) = Fv * qd(i);
    if (std::abs(qd(i)) > 1e-6) {
      F(i) += Fc * ((qd(i) > 0) ? 1.0 : -1.0);
    }
  }

  return F;
}

// ============================================================================
// Combined Dynamics
// ============================================================================

DynamicsMatrices
RobotDynamics::computeDynamicsMatrices(const VectorXd &q,
                                       const VectorXd &qd) const {
  DynamicsMatrices result;
  result.M = computeInertiaMatrix(q);
  result.C = computeCoriolisMatrix(q, qd);
  result.G = computeGravityVector(q);
  result.F = computeFrictionTorque(qd);
  return result;
}

// ============================================================================
// Inverse Dynamics
// ============================================================================

RobotDynamics::VectorXd
RobotDynamics::computeInverseDynamics(const VectorXd &q, const VectorXd &qd,
                                      const VectorXd &qdd) const {
  auto dyn = computeDynamicsMatrices(q, qd);
  return dyn.M * qdd + dyn.C * qd + dyn.G + dyn.F;
}

RobotDynamics::VectorXd RobotDynamics::computeInverseDynamicsNoFriction(
    const VectorXd &q, const VectorXd &qd, const VectorXd &qdd) const {
  MatrixXd M = computeInertiaMatrix(q);
  MatrixXd C = computeCoriolisMatrix(q, qd);
  VectorXd G = computeGravityVector(q);
  return M * qdd + C * qd + G;
}

// ============================================================================
// Forward Dynamics
// ============================================================================

RobotDynamics::VectorXd
RobotDynamics::computeForwardDynamics(const VectorXd &q, const VectorXd &qd,
                                      const VectorXd &tau) const {
  auto dyn = computeDynamicsMatrices(q, qd);

  // q̈ = M^{-1} * (τ - C*q̇ - G - F)
  VectorXd rhs = tau - dyn.C * qd - dyn.G - dyn.F;

  // Use Cholesky decomposition for numerical stability
  Eigen::LLT<MatrixXd> llt(dyn.M);
  if (llt.info() != Eigen::Success) {
    // Fallback to general solver if Cholesky fails
    return dyn.M.ldlt().solve(rhs);
  }

  return llt.solve(rhs);
}

} // namespace robot
