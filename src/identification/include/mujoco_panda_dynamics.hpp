/**
 * @file mujoco_panda_dynamics.hpp
 * @brief 基于 MuJoCo XML 参数的 Franka Panda 动力学计算
 */

#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <array>
#include <cmath>
#include <vector>

namespace mujoco_dynamics {

using Vector3d = Eigen::Vector3d;
using Matrix3d = Eigen::Matrix3d;
using Matrix4d = Eigen::Matrix4d;
using VectorXd = Eigen::VectorXd;
using MatrixXd = Eigen::MatrixXd;
using Quaterniond = Eigen::Quaterniond;

struct MuJoCoBody {
  std::string name;
  Vector3d pos{0, 0, 0};
  Quaterniond quat{1, 0, 0, 0};
  double mass{0};
  Vector3d com{0, 0, 0};
  double Ixx{0}, Iyy{0}, Izz{0};
  double Ixy{0}, Ixz{0}, Iyz{0};
  Vector3d joint_axis{0, 0, 1};
  double armature{0.1};
  double damping{1.0};
  bool has_joint{false};
};

class MuJoCoPandaDynamics {
public:
  static constexpr std::size_t N_DOF = 7;
  static constexpr std::size_t N_BODIES = 11;

  MuJoCoPandaDynamics();

  std::vector<Matrix4d> computeBodyTransforms(const VectorXd &q) const;
  Vector3d computeBodyCOM(std::size_t body_idx, const VectorXd &q) const;
  MatrixXd computeBodyJacobian(std::size_t body_idx, const VectorXd &q) const;
  MatrixXd computeInertiaMatrix(const VectorXd &q) const;
  MatrixXd computeCoriolisMatrix(const VectorXd &q, const VectorXd &qd) const;
  VectorXd computeGravityVector(const VectorXd &q) const;
  VectorXd computeInverseDynamics(const VectorXd &q, const VectorXd &qd,
                                  const VectorXd &qdd) const;

  const MuJoCoBody &body(std::size_t idx) const { return bodies_[idx]; }
  void printBodyParams() const;

private:
  std::array<MuJoCoBody, N_BODIES> bodies_;
  Vector3d gravity_{0, 0, -9.81};

  void initBodies();
  static Matrix3d skew(const Vector3d &v);
  static Matrix4d poseToTransform(const Vector3d &pos, const Quaterniond &quat);
  Eigen::Matrix<double, 6, 6> computeSpatialInertia(std::size_t body_idx) const;
};

} // namespace mujoco_dynamics
