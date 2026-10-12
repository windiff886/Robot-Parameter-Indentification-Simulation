/**
 * @file mujoco_regressor.hpp
 * @brief Regressor matrix computation based on MuJoCo kinematics
 *
 * Computes regressor matrix using MuJoCo Body-Local frames and transformations,
 * ensuring identification results align with MuJoCo simulation data.
 */

#pragma once

#include "mujoco_panda_dynamics.hpp"
#include <Eigen/Core>
#include <array>

namespace mujoco_dynamics {

/**
 * @brief Extra dynamics parameter type flags
 */
enum class MuJoCoParamFlags : unsigned int {
  NONE = 0,
  ARMATURE = 1 << 0, ///< Rotor reflected inertia
  DAMPING = 1 << 1,  ///< Viscous damping (negative sign)
  ALL = ARMATURE | DAMPING
};

inline MuJoCoParamFlags operator|(MuJoCoParamFlags a, MuJoCoParamFlags b) {
  return static_cast<MuJoCoParamFlags>(static_cast<unsigned>(a) |
                                       static_cast<unsigned>(b));
}

inline MuJoCoParamFlags operator&(MuJoCoParamFlags a, MuJoCoParamFlags b) {
  return static_cast<MuJoCoParamFlags>(static_cast<unsigned>(a) &
                                       static_cast<unsigned>(b));
}

inline bool hasFlag(MuJoCoParamFlags flags, MuJoCoParamFlags test) {
  return (static_cast<unsigned>(flags) & static_cast<unsigned>(test)) != 0;
}

/**
 * @brief MuJoCo standard inertial parameter structure
 *
 * Parameter vector for each body: [m, mx, my, mz, Ixx, Ixy, Ixz, Iyy, Iyz, Izz]
 * - m: mass
 * - mx, my, mz: first moments of mass (m * cx, m * cy, m * cz)
 * - Ixx, ..., Izz: inertia tensor defined at body origin
 */
struct MuJoCoInertialParams {
  static constexpr std::size_t PARAMS_PER_BODY = 10;

  double m;   ///< Mass
  double mx;  ///< First moment x
  double my;  ///< First moment y
  double mz;  ///< First moment z
  double Ixx; ///< Inertia tensor xx
  double Ixy; ///< Inertia tensor xy
  double Ixz; ///< Inertia tensor xz
  double Iyy; ///< Inertia tensor yy
  double Iyz; ///< Inertia tensor yz
  double Izz; ///< Inertia tensor zz

  Eigen::Matrix<double, 10, 1> toVector() const {
    Eigen::Matrix<double, 10, 1> v;
    v << m, mx, my, mz, Ixx, Ixy, Ixz, Iyy, Iyz, Izz;
    return v;
  }

  /**
   * @brief Create standard inertial parameters from MuJoCoBody
   *
   * Converts inertia tensor at COM to body origin via parallel axis theorem
   */
  static MuJoCoInertialParams fromMuJoCoBody(const MuJoCoBody &body);
};

/**
 * @brief Regressor matrix computation class based on MuJoCo kinematics
 *
 * Uses kinematics calculation identical to MuJoCoPandaDynamics,
 * ensuring regressor matrix Y satisfies: tau = Y * theta
 *
 * where theta is the inertial parameter vector defined in MuJoCo body frames.
 */
class MuJoCoRegressor {
public:
  static constexpr std::size_t N_DOF = 7;
  static constexpr std::size_t N_BODIES = 8; // link1-7 + hand

  using VectorXd = Eigen::VectorXd;
  using MatrixXd = Eigen::MatrixXd;
  using Matrix3d = Eigen::Matrix3d;
  using Vector3d = Eigen::Vector3d;
  using Matrix4d = Eigen::Matrix4d;
  using Quaterniond = Eigen::Quaterniond;

  MuJoCoRegressor();

  // ========== Parameter Vector ==========

  /**
   * @brief Get Ground Truth parameter vector (for validation)
   * @param flags Extra parameter flags
   */
  VectorXd
  computeParameterVector(MuJoCoParamFlags flags = MuJoCoParamFlags::ALL) const;

  /**
   * @brief Get parameter count
   */
  std::size_t
  numParameters(MuJoCoParamFlags flags = MuJoCoParamFlags::ALL) const;

  // ========== Regressor Matrix ==========

  /**
   * @brief Compute regressor matrix Y(q, qd, qdd)
   * @param q Joint angles
   * @param qd Joint velocities
   * @param qdd Joint accelerations
   * @param flags Extra parameter flags
   * @return Regressor matrix Y, satisfying tau = Y * theta
   */
  MatrixXd
  computeRegressorMatrix(const VectorXd &q, const VectorXd &qd,
                         const VectorXd &qdd,
                         MuJoCoParamFlags flags = MuJoCoParamFlags::ALL) const;

  /**
   * @brief Compute observation matrix W (stacked regressor matrix across samples)
   */
  MatrixXd computeObservationMatrix(
      const MatrixXd &Q, const MatrixXd &Qd, const MatrixXd &Qdd,
      MuJoCoParamFlags flags = MuJoCoParamFlags::ALL) const;

  /**
   * @brief Get list of parameter names (for debugging)
   */
  std::vector<std::string>
  getParameterNames(MuJoCoParamFlags flags = MuJoCoParamFlags::ALL) const;

  /**
   * @brief Compute body origin Jacobian (public helper)
   */
  MatrixXd computeBodyOriginJacobian(std::size_t body_idx,
                                     const VectorXd &q) const;

private:
  std::array<MuJoCoBody, 11> bodies_; // Same as MuJoCoPandaDynamics
  Vector3d gravity_{0, 0, -9.81};

  void initBodies();

  // ========== Kinematics ==========

  std::vector<Matrix4d> computeBodyTransforms(const VectorXd &q) const;
  Vector3d computeBodyCOM(std::size_t body_idx, const VectorXd &q) const;

  // Body origin Jacobian derivative (inertia defined at body origin)
  MatrixXd computeBodyOriginJacobianDerivative(std::size_t body_idx,
                                               const VectorXd &q,
                                               const VectorXd &qd) const;

  // Body COM Jacobian (for dynamics validation)
  MatrixXd computeBodyJacobian(std::size_t body_idx, const VectorXd &q) const;
  MatrixXd computeBodyJacobianDerivative(std::size_t body_idx,
                                         const VectorXd &q,
                                         const VectorXd &qd) const;

  // ========== Helper functions ==========

  static Matrix3d skew(const Vector3d &v);
  static Matrix4d poseToTransform(const Vector3d &pos, const Quaterniond &quat);

  /**
   * @brief Compute regressor block for a single body
   */
  MatrixXd computeBodyRegressorBlock(std::size_t body_idx, const VectorXd &q,
                                     const VectorXd &qd,
                                     const VectorXd &qdd) const;
};

} // namespace mujoco_dynamics
