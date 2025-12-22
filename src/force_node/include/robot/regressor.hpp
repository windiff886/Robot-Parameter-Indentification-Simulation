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
 * For N-DOF robot: M = 10*N (without friction) or M = 10*N + 2*N (with
 * friction)
 */
class Regressor {
public:
  using VectorXd = Eigen::VectorXd;
  using MatrixXd = Eigen::MatrixXd;

  explicit Regressor(const RobotModel &model);

  // ========== Parameter Vector ==========

  /**
   * @brief Compute full parameter vector β from robot model
   * @param include_friction Include friction parameters (Fv, Fc)
   * @return Parameter vector β
   */
  VectorXd computeParameterVector(bool include_friction = true) const;

  /**
   * @brief Get number of parameters per link
   */
  static constexpr std::size_t paramsPerLink() {
    return StandardInertialParameters::PARAMS_PER_LINK;
  }

  /**
   * @brief Get total number of parameters
   * @param include_friction Include friction parameters
   */
  std::size_t numParameters(bool include_friction = true) const;

  // ========== Regressor Matrix ==========

  /**
   * @brief Compute regressor matrix Y(q, q̇, q̈)
   * @param q Joint angles
   * @param qd Joint velocities
   * @param qdd Joint accelerations
   * @param include_friction Include friction terms
   * @return Regressor matrix Y such that τ = Y * β
   */
  MatrixXd computeRegressorMatrix(const VectorXd &q, const VectorXd &qd,
                                  const VectorXd &qdd,
                                  bool include_friction = true) const;

  /**
   * @brief Compute observation matrix W from trajectory data
   *
   * W is the stacked regressor matrix:
   *   W = [Y(q_1, q̇_1, q̈_1); Y(q_2, q̇_2, q̈_2); ...]
   *
   * @param Q Matrix of joint angles (N x K)
   * @param Qd Matrix of joint velocities (N x K)
   * @param Qdd Matrix of joint accelerations (N x K)
   * @param include_friction Include friction terms
   * @return Observation matrix W (N*K x M)
   */
  MatrixXd computeObservationMatrix(const MatrixXd &Q, const MatrixXd &Qd,
                                    const MatrixXd &Qdd,
                                    bool include_friction = true) const;

  // ========== Accessors ==========

  const RobotModel &model() const { return model_; }
  std::size_t numDOF() const { return model_.numDOF(); }

private:
  const RobotModel &model_;
  RobotKinematics kinematics_;

  /**
   * @brief Compute regressor block for a single link
   */
  MatrixXd computeLinkRegressorBlock(std::size_t link_idx, const VectorXd &q,
                                     const VectorXd &qd,
                                     const VectorXd &qdd) const;
};

} // namespace robot

#endif // ROBOT__REGRESSOR_HPP_
