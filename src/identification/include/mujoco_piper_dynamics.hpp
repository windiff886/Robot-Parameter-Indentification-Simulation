#pragma once

#include "mujoco_panda_dynamics.hpp"

namespace mujoco_dynamics {

class MuJoCoPiperDynamics {
public:
  static constexpr std::size_t N_DOF = 6;
  static constexpr std::size_t N_BODIES = 7; // base_link + link1-6

  MuJoCoPiperDynamics();

  std::vector<Matrix4d> computeBodyTransforms(const VectorXd &q) const;
  Vector3d computeBodyCOM(std::size_t body_idx, const VectorXd &q) const;
  MatrixXd computeBodyJacobian(std::size_t body_idx, const VectorXd &q) const;
  MatrixXd computeInertiaMatrix(const VectorXd &q) const;
  MatrixXd computeCoriolisMatrix(const VectorXd &q, const VectorXd &qd) const;
  VectorXd computeGravityVector(const VectorXd &q) const;
  VectorXd computeInverseDynamics(const VectorXd &q, const VectorXd &qd,
                                  const VectorXd &qdd) const;

  const MuJoCoBody &body(std::size_t idx) const { return bodies_[idx]; }

private:
  std::array<MuJoCoBody, N_BODIES> bodies_;
  Vector3d gravity_{0, 0, -9.81};

  void initBodies();
  static Matrix3d skew(const Vector3d &v);
  static Matrix4d poseToTransform(const Vector3d &pos, const Quaterniond &quat);
  Eigen::Matrix<double, 6, 6> computeSpatialInertia(std::size_t body_idx) const;
};

} // namespace mujoco_dynamics
