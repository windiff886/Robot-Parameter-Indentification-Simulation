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
  const double c_dot_c = cx * cx + cy * cy + cz * cz;

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

std::size_t Regressor::numParameters(bool include_friction) const {
  const std::size_t n = numDOF();
  std::size_t params = n * StandardInertialParameters::PARAMS_PER_LINK;

  if (include_friction) {
    params += 2 * n; // Fv and Fc for each joint
  }

  return params;
}

// ============================================================================
// Parameter Vector
// ============================================================================

Regressor::VectorXd
Regressor::computeParameterVector(bool include_friction) const {
  const std::size_t n = numDOF();
  const std::size_t num_params = numParameters(include_friction);
  VectorXd beta = VectorXd::Zero(num_params);

  // Fill in standard inertial parameters for each link
  for (std::size_t i = 0; i < n; ++i) {
    const auto &link = model_.link(i + 1); // Skip base link
    auto sip = StandardInertialParameters::fromLinkParameters(link);
    auto sip_vec = sip.toVector();

    const std::size_t offset = i * StandardInertialParameters::PARAMS_PER_LINK;
    beta.segment(offset, StandardInertialParameters::PARAMS_PER_LINK) = sip_vec;
  }

  // Add friction parameters if requested
  if (include_friction) {
    const auto &friction = model_.frictionParameters();
    const std::size_t friction_offset =
        n * StandardInertialParameters::PARAMS_PER_LINK;

    for (std::size_t i = 0; i < n; ++i) {
      beta(friction_offset + 2 * i) = friction[i].Fv;
      beta(friction_offset + 2 * i + 1) = friction[i].Fc;
    }
  }

  return beta;
}

// ============================================================================
// Regressor Matrix
// ============================================================================

Regressor::MatrixXd
Regressor::computeLinkRegressorBlock(std::size_t link_idx, const VectorXd &q,
                                     const VectorXd &qd,
                                     const VectorXd &qdd) const {
  const std::size_t n = numDOF();

  // Get Jacobian and its derivative for this link's COM
  MatrixXd J = kinematics_.computeLinkJacobian(link_idx, q);
  MatrixXd Jdot = kinematics_.computeJacobianDerivative(q, qd);

  // Linear and angular parts of Jacobian
  Eigen::Matrix<double, 3, Eigen::Dynamic> Jv = J.topRows(3);
  Eigen::Matrix<double, 3, Eigen::Dynamic> Jw = J.bottomRows(3);
  Eigen::Matrix<double, 3, Eigen::Dynamic> Jv_dot = Jdot.topRows(3);
  Eigen::Matrix<double, 3, Eigen::Dynamic> Jw_dot = Jdot.bottomRows(3);

  // Linear and angular accelerations
  Eigen::Vector3d a_linear = Jv * qdd + Jv_dot * qd;
  Eigen::Vector3d a_angular = Jw * qdd + Jw_dot * qd;
  Eigen::Vector3d omega = Jw * qd;

  // Gravity contribution
  const auto &g_world = model_.gravity();
  Eigen::Vector3d g(g_world[0], g_world[1], g_world[2]);

  // Build regressor block for this link (n x 10 matrix)
  // Parameters: [m, mx, my, mz, Ixx, Ixy, Ixz, Iyy, Iyz, Izz]
  MatrixXd Y_link = MatrixXd::Zero(n, 10);

  // Linear acceleration - gravity terms (for mass and first moments)
  Eigen::Vector3d linear_term = a_linear - g;

  // Mass contribution: τ_m = Jv^T * m * (a - g)
  Y_link.col(0) = Jv.transpose() * linear_term;

  // First moment contributions involve cross products
  // τ_mx = Jv^T * omega × (omega × [1,0,0]) + Jv^T * alpha × [1,0,0]
  // Simplified: we use the fact that m*c terms appear in acceleration equations
  Eigen::Matrix3d omega_skew = RobotKinematics::skew(omega);
  Eigen::Matrix3d alpha_skew = RobotKinematics::skew(a_angular);

  // For first moments: contribution from centripetal and angular acceleration
  for (int j = 0; j < 3; ++j) {
    Eigen::Vector3d e_j = Eigen::Vector3d::Zero();
    e_j(j) = 1.0;

    Eigen::Vector3d term = omega_skew * omega_skew * e_j + alpha_skew * e_j;
    Y_link.col(1 + j) = Jv.transpose() * term;
  }

  // Inertia contributions
  // τ_I = Jw^T * (I * alpha + omega × (I * omega))
  // This requires computing how each inertia element contributes

  // Ixx contribution
  Y_link.col(4) = Jw.transpose() *
                  (a_angular(0) * Eigen::Vector3d::UnitX() +
                   omega(1) * omega(2) *
                       (Eigen::Vector3d::UnitZ() - Eigen::Vector3d::UnitY()));

  // Ixy contribution
  Y_link.col(5) = Jw.transpose() * (a_angular(0) * Eigen::Vector3d::UnitY() +
                                    a_angular(1) * Eigen::Vector3d::UnitX());

  // Ixz contribution
  Y_link.col(6) = Jw.transpose() * (a_angular(0) * Eigen::Vector3d::UnitZ() +
                                    a_angular(2) * Eigen::Vector3d::UnitX());

  // Iyy contribution
  Y_link.col(7) = Jw.transpose() *
                  (a_angular(1) * Eigen::Vector3d::UnitY() +
                   omega(0) * omega(2) *
                       (Eigen::Vector3d::UnitX() - Eigen::Vector3d::UnitZ()));

  // Iyz contribution
  Y_link.col(8) = Jw.transpose() * (a_angular(1) * Eigen::Vector3d::UnitZ() +
                                    a_angular(2) * Eigen::Vector3d::UnitY());

  // Izz contribution
  Y_link.col(9) = Jw.transpose() *
                  (a_angular(2) * Eigen::Vector3d::UnitZ() +
                   omega(0) * omega(1) *
                       (Eigen::Vector3d::UnitY() - Eigen::Vector3d::UnitX()));

  return Y_link;
}

Regressor::MatrixXd
Regressor::computeRegressorMatrix(const VectorXd &q, const VectorXd &qd,
                                  const VectorXd &qdd,
                                  bool include_friction) const {
  const std::size_t n = numDOF();
  const std::size_t num_params = numParameters(include_friction);
  MatrixXd Y = MatrixXd::Zero(n, num_params);

  // Compute regressor contribution from each link
  for (std::size_t i = 0; i < n; ++i) {
    MatrixXd Y_link = computeLinkRegressorBlock(i, q, qd, qdd);

    const std::size_t offset = i * StandardInertialParameters::PARAMS_PER_LINK;
    Y.block(0, offset, n, StandardInertialParameters::PARAMS_PER_LINK) +=
        Y_link;
  }

  // Add friction regressor if requested
  if (include_friction) {
    const std::size_t friction_offset =
        n * StandardInertialParameters::PARAMS_PER_LINK;

    for (std::size_t i = 0; i < n; ++i) {
      // Viscous friction: Fv * qd
      Y(i, friction_offset + 2 * i) = qd(i);

      // Coulomb friction: Fc * sign(qd)
      if (std::abs(qd(i)) > 1e-6) {
        Y(i, friction_offset + 2 * i + 1) = (qd(i) > 0) ? 1.0 : -1.0;
      }
    }
  }

  return Y;
}

Regressor::MatrixXd
Regressor::computeObservationMatrix(const MatrixXd &Q, const MatrixXd &Qd,
                                    const MatrixXd &Qdd,
                                    bool include_friction) const {
  const std::size_t n = numDOF();
  const std::size_t K = Q.cols(); // Number of samples
  const std::size_t num_params = numParameters(include_friction);

  MatrixXd W = MatrixXd::Zero(n * K, num_params);

  for (std::size_t k = 0; k < K; ++k) {
    VectorXd q = Q.col(k);
    VectorXd qd = Qd.col(k);
    VectorXd qdd = Qdd.col(k);

    MatrixXd Y_k = computeRegressorMatrix(q, qd, qdd, include_friction);
    W.block(k * n, 0, n, num_params) = Y_k;
  }

  return W;
}

} // namespace robot
