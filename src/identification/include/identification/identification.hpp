#ifndef IDENTIFICATION_IDENTIFICATION_HPP_
#define IDENTIFICATION_IDENTIFICATION_HPP_

#include "identification/algorithms.hpp"
#include "identification/data_loader.hpp"
#include "mujoco_regressor.hpp"
#include "robot/robot_model.hpp"
#include <memory>
#include <string>

/**
 * @brief 参数辨识类（使用 MuJoCoRegressor）
 *
 * 使用与 MuJoCo 一致的坐标系计算回归矩阵，
 * 确保辨识结果可以正确预测 MuJoCo 仿真的扭矩。
 */
class Identification {
public:
  /**
   * @brief 构造函数
   *
   * 注意：RobotModel 参数保留用于兼容性，但实际使用 MuJoCoRegressor
   */
  explicit Identification(std::unique_ptr<robot::RobotModel> model = nullptr);

  /**
   * @brief Process data: calculate qdd and filter
   */
  void preprocess(ExperimentData &data);

  /**
   * @brief Solve identification problem using specified algorithm
   *
   * @param data Experiment data
   * @param algorithm_type "OLS", "WLS", "IRLS", "TLS", "EKF", "ML", "CLOE"
   * @param flags 动力学参数标志 (ARMATURE, DAMPING)
   * @return Identified parameters beta (94 parameters)
   *
   * 参数向量 β 的结构:
   *   [惯性参数 (10*8)] + [armature (7)] + [damping (7)] = 94 参数
   */
  Eigen::VectorXd solve(const ExperimentData &data,
                        const std::string &algorithm_type = "OLS",
                        mujoco_dynamics::MuJoCoParamFlags flags =
                            mujoco_dynamics::MuJoCoParamFlags::ALL);

  /**
   * @brief 获取参数数量
   */
  std::size_t numParameters(mujoco_dynamics::MuJoCoParamFlags flags =
                                mujoco_dynamics::MuJoCoParamFlags::ALL) const;

  /**
   * @brief 获取 Ground Truth 参数向量 (用于验证)
   */
  Eigen::VectorXd
  getGroundTruthParameters(mujoco_dynamics::MuJoCoParamFlags flags =
                               mujoco_dynamics::MuJoCoParamFlags::ALL) const;

  /**
   * @brief 计算观测矩阵 W
   */
  Eigen::MatrixXd
  computeObservationMatrix(const Eigen::MatrixXd &Q, const Eigen::MatrixXd &Qd,
                           const Eigen::MatrixXd &Qdd,
                           mujoco_dynamics::MuJoCoParamFlags flags =
                               mujoco_dynamics::MuJoCoParamFlags::ALL) const;

private:
  std::unique_ptr<robot::RobotModel> model_; // 保留用于兼容性
  mujoco_dynamics::MuJoCoRegressor regressor_;
};

#endif // IDENTIFICATION_IDENTIFICATION_HPP_
