/**
 * @file mujoco_regressor.hpp
 * @brief 基于 MuJoCo 运动学的回归矩阵计算
 *
 * 使用 MuJoCo 的 Body-Local 坐标系和变换方式计算回归矩阵，
 * 确保辨识结果与 MuJoCo 仿真数据一致。
 */

#pragma once

#include "mujoco_panda_dynamics.hpp"
#include <Eigen/Core>
#include <array>

namespace mujoco_dynamics {

/**
 * @brief 额外动力学参数类型标志
 */
enum class MuJoCoParamFlags : unsigned int {
  NONE = 0,
  ARMATURE = 1 << 0, ///< 电机反映惯量
  DAMPING = 1 << 1,  ///< 粘性阻尼 (符号为负)
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
 * @brief MuJoCo 标准惯性参数结构
 *
 * 每个 Body 的参数向量: [m, mx, my, mz, Ixx, Ixy, Ixz, Iyy, Iyz, Izz]
 * - m: 质量
 * - mx, my, mz: 质心位置的一阶矩 (m * cx, m * cy, m * cz)
 * - Ixx, ..., Izz: 在 Body 坐标系原点的惯量张量
 */
struct MuJoCoInertialParams {
  static constexpr std::size_t PARAMS_PER_BODY = 10;

  double m;   ///< 质量
  double mx;  ///< 一阶矩 x
  double my;  ///< 一阶矩 y
  double mz;  ///< 一阶矩 z
  double Ixx; ///< 惯量张量 xx
  double Ixy; ///< 惯量张量 xy
  double Ixz; ///< 惯量张量 xz
  double Iyy; ///< 惯量张量 yy
  double Iyz; ///< 惯量张量 yz
  double Izz; ///< 惯量张量 zz

  Eigen::Matrix<double, 10, 1> toVector() const {
    Eigen::Matrix<double, 10, 1> v;
    v << m, mx, my, mz, Ixx, Ixy, Ixz, Iyy, Iyz, Izz;
    return v;
  }

  /**
   * @brief 从 MuJoCoBody 创建标准惯性参数
   *
   * 将质心处定义的惯量通过平行轴定理转换到 Body 坐标系原点
   */
  static MuJoCoInertialParams fromMuJoCoBody(const MuJoCoBody &body);
};

/**
 * @brief 基于 MuJoCo 运动学的回归矩阵计算类
 *
 * 使用与 MuJoCoPandaDynamics 完全一致的运动学计算方式，
 * 确保回归矩阵 Y 满足: τ = Y * θ
 *
 * 其中 θ 是 MuJoCo Body 坐标系中定义的惯性参数向量。
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

  // ========== 参数向量 ==========

  /**
   * @brief 获取 Ground Truth 参数向量 (用于验证)
   * @param flags 额外参数标志
   */
  VectorXd
  computeParameterVector(MuJoCoParamFlags flags = MuJoCoParamFlags::ALL) const;

  /**
   * @brief 获取参数数量
   */
  std::size_t
  numParameters(MuJoCoParamFlags flags = MuJoCoParamFlags::ALL) const;

  // ========== 回归矩阵 ==========

  /**
   * @brief 计算回归矩阵 Y(q, qd, qdd)
   * @param q 关节角度
   * @param qd 关节速度
   * @param qdd 关节加速度
   * @param flags 额外参数标志
   * @return 回归矩阵 Y，满足 τ = Y * θ
   */
  MatrixXd
  computeRegressorMatrix(const VectorXd &q, const VectorXd &qd,
                         const VectorXd &qdd,
                         MuJoCoParamFlags flags = MuJoCoParamFlags::ALL) const;

  /**
   * @brief 计算观测矩阵 W (堆叠多个样本的回归矩阵)
   */
  MatrixXd computeObservationMatrix(
      const MatrixXd &Q, const MatrixXd &Qd, const MatrixXd &Qdd,
      MuJoCoParamFlags flags = MuJoCoParamFlags::ALL) const;

  /**
   * @brief 获取参数名称列表 (用于调试)
   */
  std::vector<std::string>
  getParameterNames(MuJoCoParamFlags flags = MuJoCoParamFlags::ALL) const;

private:
  std::array<MuJoCoBody, 11> bodies_; // 与 MuJoCoPandaDynamics 相同
  Vector3d gravity_{0, 0, -9.81};

  void initBodies();

  // ========== 运动学 ==========

  std::vector<Matrix4d> computeBodyTransforms(const VectorXd &q) const;
  Vector3d computeBodyCOM(std::size_t body_idx, const VectorXd &q) const;

  // Body 原点的雅可比 (用于回归矩阵，惯量定义在 Body 原点)
  MatrixXd computeBodyOriginJacobian(std::size_t body_idx,
                                     const VectorXd &q) const;
  MatrixXd computeBodyOriginJacobianDerivative(std::size_t body_idx,
                                               const VectorXd &q,
                                               const VectorXd &qd) const;

  // Body COM 的雅可比 (用于动力学验证)
  MatrixXd computeBodyJacobian(std::size_t body_idx, const VectorXd &q) const;
  MatrixXd computeBodyJacobianDerivative(std::size_t body_idx,
                                         const VectorXd &q,
                                         const VectorXd &qd) const;

  // ========== 辅助函数 ==========

  static Matrix3d skew(const Vector3d &v);
  static Matrix4d poseToTransform(const Vector3d &pos, const Quaterniond &quat);

  /**
   * @brief 计算单个 Body 的回归矩阵块
   */
  MatrixXd computeBodyRegressorBlock(std::size_t body_idx, const VectorXd &q,
                                     const VectorXd &qd,
                                     const VectorXd &qdd) const;
};

} // namespace mujoco_dynamics
