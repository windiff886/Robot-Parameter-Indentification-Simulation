#include "mujoco_piper_dynamics.hpp"

namespace mujoco_dynamics {

MuJoCoPiperDynamics::MuJoCoPiperDynamics() { initBodies(); }

void MuJoCoPiperDynamics::initBodies() {
  bodies_[0].name = "base_link";
  bodies_[0].mass = 1.02;
  bodies_[0].com = Vector3d(-0.00473641, 2.56829e-05, 0.041452);
  bodies_[0].Ixx = 0.00267433;
  bodies_[0].Iyy = 0.00282612;
  bodies_[0].Izz = 0.00089624;
  bodies_[0].Ixy = -0.00000073;
  bodies_[0].Ixz = -0.00017389;
  bodies_[0].Iyz = 0.0000004;

  bodies_[1].name = "link1";
  bodies_[1].pos = Vector3d(0.0, 0.0, 0.123);
  bodies_[1].quat = Quaterniond(1.0, 0.0, 0.0, 0.0);
  bodies_[1].mass = 0.71;
  bodies_[1].com = Vector3d(0.000121505, 0.000104632, -0.00438597);
  bodies_[1].Ixx = 0.000489262;
  bodies_[1].Iyy = 0.000439887;
  bodies_[1].Izz = 0.000404551;
  bodies_[1].has_joint = true;

  bodies_[2].name = "link2";
  bodies_[2].quat = Quaterniond(0.0356735, -0.0356786, -0.706207, -0.706205);
  bodies_[2].mass = 1.17;
  bodies_[2].com = Vector3d(0.198666, -0.0109269, 0.00142122);
  bodies_[2].Ixx = 0.0679032;
  bodies_[2].Iyy = 0.067745;
  bodies_[2].Izz = 0.00111966;
  bodies_[2].has_joint = true;

  bodies_[3].name = "link3";
  bodies_[3].pos = Vector3d(0.28503, 0.0, 0.0);
  bodies_[3].quat = Quaterniond(0.637536, 0.0, 0.0, -0.77042);
  bodies_[3].mass = 0.5;
  bodies_[3].com = Vector3d(-0.0202738, -0.133915, -0.000458683);
  bodies_[3].Ixx = 0.0138227;
  bodies_[3].Iyy = 0.0138032;
  bodies_[3].Izz = 0.000244685;
  bodies_[3].has_joint = true;

  bodies_[4].name = "link4";
  bodies_[4].pos = Vector3d(-0.021984, -0.25075, 0.0);
  bodies_[4].quat = Quaterniond(0.707105, 0.707108, 0.0, 0.0);
  bodies_[4].mass = 0.38;
  bodies_[4].com = Vector3d(-9.66636e-05, 0.000876064, -0.00496881);
  bodies_[4].Ixx = 0.000191586;
  bodies_[4].Iyy = 0.000185052;
  bodies_[4].Izz = 0.000152863;
  bodies_[4].has_joint = true;

  bodies_[5].name = "link5";
  bodies_[5].quat = Quaterniond(0.707105, -0.707108, 0.0, 0.0);
  bodies_[5].mass = 0.383;
  bodies_[5].com = Vector3d(-4.10554e-05, -0.0566487, -0.00372058);
  bodies_[5].Ixx = 0.00166169;
  bodies_[5].Iyy = 0.00164328;
  bodies_[5].Izz = 0.000185028;
  bodies_[5].has_joint = true;

  bodies_[6].name = "link6";
  bodies_[6].pos = Vector3d(8.8259e-05, -0.091, 0.0);
  bodies_[6].quat = Quaterniond(0.707105, 0.707108, 0.0, 0.0);
  bodies_[6].mass = 0.456991;
  bodies_[6].com = Vector3d(-0.000182345, 7.94104e-05, 0.0316214);
  bodies_[6].Ixx = 0.000938039;
  bodies_[6].Iyy = 0.000723068;
  bodies_[6].Izz = 0.000395388;
  bodies_[6].Ixy = 0.0;
  bodies_[6].Ixz = 0.0;
  bodies_[6].Iyz = 0.0;
  bodies_[6].has_joint = true;

  for (std::size_t i = 1; i < N_BODIES; ++i) {
    bodies_[i].joint_axis = Vector3d(0.0, 0.0, 1.0);
    bodies_[i].armature = 0.1;
    bodies_[i].damping = 1.0;
  }
}

Matrix3d MuJoCoPiperDynamics::skew(const Vector3d &v) {
  Matrix3d S;
  S << 0.0, -v(2), v(1), v(2), 0.0, -v(0), -v(1), v(0), 0.0;
  return S;
}

Matrix4d MuJoCoPiperDynamics::poseToTransform(const Vector3d &pos,
                                              const Quaterniond &quat) {
  Matrix4d T = Matrix4d::Identity();
  T.block<3, 3>(0, 0) = quat.normalized().toRotationMatrix();
  T.block<3, 1>(0, 3) = pos;
  return T;
}

std::vector<Matrix4d>
MuJoCoPiperDynamics::computeBodyTransforms(const VectorXd &q) const {
  std::vector<Matrix4d> transforms(N_BODIES);
  transforms[0] = poseToTransform(bodies_[0].pos, bodies_[0].quat);

  for (std::size_t i = 1; i < N_BODIES; ++i) {
    Matrix4d T_parent_body = poseToTransform(bodies_[i].pos, bodies_[i].quat);
    Quaterniond joint_rot(Eigen::AngleAxisd(q(static_cast<Eigen::Index>(i - 1)),
                                            bodies_[i].joint_axis));
    Matrix4d T_joint = Matrix4d::Identity();
    T_joint.block<3, 3>(0, 0) = joint_rot.toRotationMatrix();
    transforms[i] = transforms[i - 1] * T_parent_body * T_joint;
  }

  return transforms;
}

Vector3d MuJoCoPiperDynamics::computeBodyCOM(std::size_t body_idx,
                                             const VectorXd &q) const {
  const auto transforms = computeBodyTransforms(q);
  const Matrix4d &T = transforms[body_idx];
  return T.block<3, 3>(0, 0) * bodies_[body_idx].com + T.block<3, 1>(0, 3);
}

MatrixXd MuJoCoPiperDynamics::computeBodyJacobian(std::size_t body_idx,
                                                  const VectorXd &q) const {
  MatrixXd J = MatrixXd::Zero(6, N_DOF);
  const auto transforms = computeBodyTransforms(q);
  const Vector3d p_body = computeBodyCOM(body_idx, q);

  for (std::size_t i = 1; i <= body_idx && i < N_BODIES; ++i) {
    const Vector3d z_axis =
        transforms[i].block<3, 3>(0, 0) * bodies_[i].joint_axis;
    const Vector3d p_joint = transforms[i].block<3, 1>(0, 3);
    const Eigen::Index col = static_cast<Eigen::Index>(i - 1);
    J.block<3, 1>(0, col) = z_axis.cross(p_body - p_joint);
    J.block<3, 1>(3, col) = z_axis;
  }

  return J;
}

Eigen::Matrix<double, 6, 6>
MuJoCoPiperDynamics::computeSpatialInertia(std::size_t body_idx) const {
  const auto &body = bodies_[body_idx];
  const Vector3d c = body.com;
  const double m = body.mass;

  Matrix3d I_com;
  I_com << body.Ixx, body.Ixy, body.Ixz, body.Ixy, body.Iyy, body.Iyz,
      body.Ixz, body.Iyz, body.Izz;

  const Matrix3d c_skew = skew(c);
  const Matrix3d I_origin = I_com - m * c_skew * c_skew;

  Eigen::Matrix<double, 6, 6> M_spatial;
  M_spatial.setZero();
  M_spatial.block<3, 3>(0, 0) = m * Matrix3d::Identity();
  M_spatial.block<3, 3>(0, 3) = m * c_skew.transpose();
  M_spatial.block<3, 3>(3, 0) = m * c_skew;
  M_spatial.block<3, 3>(3, 3) = I_origin;
  return M_spatial;
}

MatrixXd MuJoCoPiperDynamics::computeInertiaMatrix(const VectorXd &q) const {
  MatrixXd M = MatrixXd::Zero(N_DOF, N_DOF);

  for (std::size_t i = 1; i < N_BODIES; ++i) {
    const MatrixXd J_i = computeBodyJacobian(i, q);
    M += J_i.transpose() * computeSpatialInertia(i) * J_i;
  }

  for (std::size_t i = 0; i < N_DOF; ++i) {
    M(static_cast<Eigen::Index>(i), static_cast<Eigen::Index>(i)) +=
        bodies_[i + 1].armature;
  }

  return (M + M.transpose()) / 2.0;
}

MatrixXd MuJoCoPiperDynamics::computeCoriolisMatrix(const VectorXd &q,
                                                    const VectorXd &qd) const {
  const double eps = 1e-7;
  MatrixXd C = MatrixXd::Zero(N_DOF, N_DOF);

  for (std::size_t k = 0; k < N_DOF; ++k) {
    VectorXd q_plus = q;
    VectorXd q_minus = q;
    q_plus(static_cast<Eigen::Index>(k)) += eps;
    q_minus(static_cast<Eigen::Index>(k)) -= eps;
    const MatrixXd dM_dqk =
        (computeInertiaMatrix(q_plus) - computeInertiaMatrix(q_minus)) /
        (2.0 * eps);
    C += 0.5 * dM_dqk * qd(static_cast<Eigen::Index>(k));
  }

  return C;
}

VectorXd MuJoCoPiperDynamics::computeGravityVector(const VectorXd &q) const {
  VectorXd G = VectorXd::Zero(N_DOF);

  for (std::size_t i = 1; i < N_BODIES; ++i) {
    const MatrixXd J_i = computeBodyJacobian(i, q);
    G -= J_i.topRows(3).transpose() * (bodies_[i].mass * gravity_);
  }

  return G;
}

VectorXd MuJoCoPiperDynamics::computeInverseDynamics(const VectorXd &q,
                                                     const VectorXd &qd,
                                                     const VectorXd &qdd) const {
  const MatrixXd M = computeInertiaMatrix(q);
  const MatrixXd C = computeCoriolisMatrix(q, qd);
  const VectorXd g = computeGravityVector(q);

  VectorXd damping_force = VectorXd::Zero(N_DOF);
  for (std::size_t i = 0; i < N_DOF; ++i) {
    damping_force(static_cast<Eigen::Index>(i)) =
        bodies_[i + 1].damping * qd(static_cast<Eigen::Index>(i));
  }

  return M * qdd + C * qd + g - damping_force;
}

} // namespace mujoco_dynamics
