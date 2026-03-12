#ifndef IDENTIFICATION_IDENTIFICATION_HPP_
#define IDENTIFICATION_IDENTIFICATION_HPP_

#include "identification/algorithms.hpp"
#include "identification/data_loader.hpp"
#include "mujoco_regressor.hpp"
#include "mujoco_piper_regressor.hpp"
#include "robot/robot_model.hpp"
#include <memory>
#include <string>

/**
 * @brief 参数辨识类（按机械臂切换对应的 MuJoCo 回归器）
 *
 * 使用与 MuJoCo 一致的坐标系计算回归矩阵，
 * 确保辨识结果可以正确预测 MuJoCo 仿真的扭矩。
 */
class Identification {
public:
  /**
   * @brief 构造函数
   *
   * @param robot_type 机械臂类型，支持 "panda" / "piper"
   * @param model 保留用于兼容性的机器人模型指针
   */
  explicit Identification(const std::string &robot_type = "panda",
                          std::unique_ptr<robot::RobotModel> model = nullptr);

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

  /**
   * @brief 使用指定算法和参数预测整段实验的堆叠力矩向量
   */
  Eigen::VectorXd
  predictTorques(const ExperimentData &data, const Eigen::VectorXd &params,
                 const std::string &algorithm_type = "OLS",
                 mujoco_dynamics::MuJoCoParamFlags flags =
                     mujoco_dynamics::MuJoCoParamFlags::ALL) const;

private:
  std::unique_ptr<robot::RobotModel> model_; // 保留用于兼容性
  std::string robot_type_;
  mujoco_dynamics::MuJoCoRegressor panda_regressor_;
  mujoco_dynamics::MuJoCoPiperRegressor piper_regressor_;
};

#endif // IDENTIFICATION_IDENTIFICATION_HPP_
