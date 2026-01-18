/**
 * @file regressor.hpp
 * @brief Regressor matrix computation for parameter identification
 *
 * Computes the regressor matrix Y such that τ = Y * β
 * where β is the vector of base inertial parameters.
 */

#ifndef ROBOT__REGRESSOR_HPP_
#define ROBOT__REGRESSOR_HPP_

#include "robot/robot_kinematics.hpp"
#include "robot/robot_model.hpp"
#include <Eigen/Dense>
#include <vector>

namespace robot {

/**
 * @brief Standard Inertial Parameters for a single link
 *
 * For each link i, the SIP vector contains:
 * [m, mx, my, mz, Ixx, Ixy, Ixz, Iyy, Iyz, Izz]
 *
 * Where:
 *   m    : mass
 *   mx/y/z : first moment of mass (mass * COM position)
 *   Ixx, Ixy, ... : inertia tensor elements at link frame origin
 */
struct StandardInertialParameters {
  static constexpr std::size_t PARAMS_PER_LINK = 10;

  double m;   ///< Mass
  double mx;  ///< First moment x (m * cx)
  double my;  ///< First moment y (m * cy)
  double mz;  ///< First moment z (m * cz)
  double Ixx; ///< Inertia tensor xx
  double Ixy; ///< Inertia tensor xy
  double Ixz; ///< Inertia tensor xz
  double Iyy; ///< Inertia tensor yy
  double Iyz; ///< Inertia tensor yz
  double Izz; ///< Inertia tensor zz

  /**
   * @brief Convert to Eigen vector
   */
  Eigen::Matrix<double, 10, 1> toVector() const {
    Eigen::Matrix<double, 10, 1> v;
    v << m, mx, my, mz, Ixx, Ixy, Ixz, Iyy, Iyz, Izz;
    return v;
  }

  /**
   * @brief Create from link parameters (transform inertia from COM to origin)
   */
  static StandardInertialParameters
  fromLinkParameters(const LinkParameters &link);
};

/**
 * @brief 额外动力学参数类型标志
 *
 * 用于控制回归矩阵中包含哪些额外参数。
 * 这些标志可以组合使用。
 */
enum class DynamicsParamFlags : unsigned int {
  NONE = 0,
  ARMATURE = 1 << 0, ///< 电机反映惯量 (MuJoCo armature)
  DAMPING = 1 << 1,  ///< 粘性阻尼 (MuJoCo damping, 符号为负)
  FRICTION = 1 << 2, ///< 摩擦 (Fv, Fc)
  ALL = ARMATURE | DAMPING | FRICTION
};

inline DynamicsParamFlags operator|(DynamicsParamFlags a,
                                    DynamicsParamFlags b) {
  return static_cast<DynamicsParamFlags>(static_cast<unsigned>(a) |
                                         static_cast<unsigned>(b));
}

inline DynamicsParamFlags operator&(DynamicsParamFlags a,
                                    DynamicsParamFlags b) {
  return static_cast<DynamicsParamFlags>(static_cast<unsigned>(a) &
                                         static_cast<unsigned>(b));
}

inline bool hasFlag(DynamicsParamFlags flags, DynamicsParamFlags test) {
  return (static_cast<unsigned>(flags) & static_cast<unsigned>(test)) != 0;
}

/**
 * @brief Regressor matrix computation class
 *
 * The inverse dynamics can be written as:
 *   τ = Y(q, q̇, q̈) * β
 *
 * Where:
 *   Y : (N x M) regressor matrix depending only on kinematics
 *   β : (M x 1) vector of inertial parameters (standard or base)
 *   N : number of joints
 *   M : number of parameters
 *
 * 参数向量 β 的结构:
 *   [惯性参数 (10*N)] + [armature (N)] + [damping (N)] + [friction (2*N)]
 *
 * MuJoCo 动力学方程:
 *   (M + diag(armature)) * q̈ + C*q̇ + G = τ + damping * q̇
 *
 * 整理为回归形式:
 *   τ = M*q̈ + C*q̇ + G + armature*q̈ - damping*q̇
 *
 * 注意: damping 项符号为负，因为 MuJoCo 的阻尼是被动项（帮助驱动器）
 */
class Regressor {
public:
  using VectorXd = Eigen::VectorXd;
  using MatrixXd = Eigen::MatrixXd;

  explicit Regressor(const RobotModel &model);

  // ========== Parameter Vector ==========

  /**
   * @brief Compute full parameter vector β from robot model
   * @param flags 额外参数标志 (ARMATURE, DAMPING, FRICTION)
   * @return Parameter vector β
   */
  VectorXd computeParameterVector(
      DynamicsParamFlags flags = DynamicsParamFlags::FRICTION) const;

  // 向后兼容的重载
  VectorXd computeParameterVector(bool include_friction) const {
    return computeParameterVector(include_friction
                                      ? DynamicsParamFlags::FRICTION
                                      : DynamicsParamFlags::NONE);
  }

  /**
   * @brief Get number of parameters per link
   */
  static constexpr std::size_t paramsPerLink() {
    return StandardInertialParameters::PARAMS_PER_LINK;
  }

  /**
   * @brief Get total number of parameters
   * @param flags 额外参数标志
   */
  std::size_t numParameters(DynamicsParamFlags flags) const;

  // 向后兼容的重载
  std::size_t numParameters(bool include_friction = true) const {
    return numParameters(include_friction ? DynamicsParamFlags::FRICTION
                                          : DynamicsParamFlags::NONE);
  }

  // ========== Regressor Matrix ==========

  /**
   * @brief Compute regressor matrix Y(q, q̇, q̈)
   * @param q Joint angles
   * @param qd Joint velocities
   * @param qdd Joint accelerations
   * @param flags 额外参数标志
   * @return Regressor matrix Y such that τ = Y * β
   */
  MatrixXd computeRegressorMatrix(const VectorXd &q, const VectorXd &qd,
                                  const VectorXd &qdd,
                                  DynamicsParamFlags flags) const;

  // 向后兼容的重载
  MatrixXd computeRegressorMatrix(const VectorXd &q, const VectorXd &qd,
                                  const VectorXd &qdd,
                                  bool include_friction) const {
    return computeRegressorMatrix(q, qd, qdd,
                                  include_friction
                                      ? DynamicsParamFlags::FRICTION
                                      : DynamicsParamFlags::NONE);
  }

  /**
   * @brief Compute observation matrix W from trajectory data
   *
   * W is the stacked regressor matrix:
   *   W = [Y(q_1, q̇_1, q̈_1); Y(q_2, q̇_2, q̈_2); ...]
   *
   * @param Q Matrix of joint angles (N x K)
   * @param Qd Matrix of joint velocities (N x K)
   * @param Qdd Matrix of joint accelerations (N x K)
   * @param flags 额外参数标志
   * @return Observation matrix W (N*K x M)
   */
  MatrixXd computeObservationMatrix(const MatrixXd &Q, const MatrixXd &Qd,
                                    const MatrixXd &Qdd,
                                    DynamicsParamFlags flags) const;

  // 向后兼容的重载
  MatrixXd computeObservationMatrix(const MatrixXd &Q, const MatrixXd &Qd,
                                    const MatrixXd &Qdd,
                                    bool include_friction) const {
    return computeObservationMatrix(Q, Qd, Qdd,
                                    include_friction
                                        ? DynamicsParamFlags::FRICTION
                                        : DynamicsParamFlags::NONE);
  }

  // ========== Accessors ==========

  const RobotModel &model() const { return model_; }
  std::size_t numDOF() const { return model_.numDOF(); }

private:
  const RobotModel &model_;
  RobotKinematics kinematics_;

  /**
   * @brief Compute regressor block for a single link
   * @param R Rotation matrix from Link Frame to World Frame
   */
  MatrixXd computeLinkRegressorBlock(std::size_t link_idx, const VectorXd &q,
                                     const VectorXd &qd, const VectorXd &qdd,
                                     const Eigen::Matrix3d &R) const;

  /**
   * @brief Compute regressor block for a fixed body
   * @param R Rotation matrix from Fixed Body Frame to World Frame
   */
  MatrixXd computeFixedBodyRegressorBlock(std::size_t fixed_body_idx,
                                          const VectorXd &q, const VectorXd &qd,
                                          const VectorXd &qdd,
                                          const Eigen::Matrix3d &R) const;

  /**
   * @brief Generic method to compute regressor block from Jacobians (projected
   * to Local Frame)
   * @param J Jacobian in World Frame
   * @param Jdot Jacobian Derivative in World Frame
   * @param qd Joint velocities
   * @param qdd Joint accelerations
   * @param R Rotation matrix from Local Frame to World Frame
   */
  MatrixXd computeBlockFromJacobians(const MatrixXd &J, const MatrixXd &Jdot,
                                     const VectorXd &qd, const VectorXd &qdd,
                                     const Eigen::Matrix3d &R) const;
};

} // namespace robot

#endif // ROBOT__REGRESSOR_HPP_
