#pragma once

#include "mujoco_regressor.hpp"

namespace mujoco_dynamics {

class MuJoCoPiperRegressor {
public:
  static constexpr std::size_t N_DOF = 6;
  static constexpr std::size_t N_BODIES = 6; // link1-6

  using VectorXd = Eigen::VectorXd;
  using MatrixXd = Eigen::MatrixXd;
  using Matrix3d = Eigen::Matrix3d;
  using Vector3d = Eigen::Vector3d;
  using Matrix4d = Eigen::Matrix4d;
  using Quaterniond = Eigen::Quaterniond;

  MuJoCoPiperRegressor();

  VectorXd
  computeParameterVector(MuJoCoParamFlags flags = MuJoCoParamFlags::ALL) const;

  std::size_t
  numParameters(MuJoCoParamFlags flags = MuJoCoParamFlags::ALL) const;

  MatrixXd
  computeRegressorMatrix(const VectorXd &q, const VectorXd &qd,
                         const VectorXd &qdd,
                         MuJoCoParamFlags flags = MuJoCoParamFlags::ALL) const;

  MatrixXd computeObservationMatrix(
      const MatrixXd &Q, const MatrixXd &Qd, const MatrixXd &Qdd,
      MuJoCoParamFlags flags = MuJoCoParamFlags::ALL) const;

  std::vector<std::string>
  getParameterNames(MuJoCoParamFlags flags = MuJoCoParamFlags::ALL) const;

private:
  std::array<MuJoCoBody, N_BODIES + 1> bodies_; // base_link + link1-6
  Vector3d gravity_{0, 0, -9.81};

  void initBodies();

  std::vector<Matrix4d> computeBodyTransforms(const VectorXd &q) const;
  Vector3d computeBodyCOM(std::size_t body_idx, const VectorXd &q) const;
  MatrixXd computeBodyOriginJacobian(std::size_t body_idx,
                                     const VectorXd &q) const;
  MatrixXd computeBodyOriginJacobianDerivative(std::size_t body_idx,
                                               const VectorXd &q,
                                               const VectorXd &qd) const;
  MatrixXd computeBodyJacobian(std::size_t body_idx, const VectorXd &q) const;
  MatrixXd computeBodyJacobianDerivative(std::size_t body_idx,
                                         const VectorXd &q,
                                         const VectorXd &qd) const;

  static Matrix3d skew(const Vector3d &v);
  static Matrix4d poseToTransform(const Vector3d &pos, const Quaterniond &quat);

  MatrixXd computeBodyRegressorBlock(std::size_t body_idx, const VectorXd &q,
                                     const VectorXd &qd,
                                     const VectorXd &qdd) const;
};

} // namespace mujoco_dynamics
