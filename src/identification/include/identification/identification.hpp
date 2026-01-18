#ifndef IDENTIFICATION_IDENTIFICATION_HPP_
#define IDENTIFICATION_IDENTIFICATION_HPP_

#include "identification/algorithms.hpp"
#include "identification/data_loader.hpp"
#include "robot/regressor.hpp"
#include "robot/robot_model.hpp"
#include <memory>
#include <string>

class Identification {
public:
  explicit Identification(std::unique_ptr<robot::RobotModel> model);

  /**
   * @brief Process data: calculate qdd and filter
   */
  void preprocess(ExperimentData &data);

  /**
   * @brief Solve identification problem using specified algorithm
   *
   * @param data Experiment data
   * @param algorithm_type "OLS", "WLS", "IRLS", "TLS"
   * @param flags 动力学参数标志 (ARMATURE, DAMPING, FRICTION)
   *              默认包含 ARMATURE 和 DAMPING 以匹配 MuJoCo 动力学模型
   * @return Identified parameters beta
   *
   * 参数向量 β 的结构:
   *   [惯性参数 (10*N)] + [armature (N)] + [damping (N)] + [friction (2*N)]
   *
   * MuJoCo 动力学方程:
   *   (M + diag(armature)) * q̈ + C*q̇ + G = τ + damping * q̇
   *
   * 整理为回归形式:
   *   τ = M*q̈ + C*q̇ + G + armature*q̈ - damping*q̇
   */
  Eigen::VectorXd solve(const ExperimentData &data,
                        const std::string &algorithm_type = "OLS",
                        robot::DynamicsParamFlags flags =
                            robot::DynamicsParamFlags::ARMATURE |
                            robot::DynamicsParamFlags::DAMPING);

  // 向后兼容的重载
  Eigen::VectorXd solve(const ExperimentData &data,
                        const std::string &algorithm_type,
                        bool include_friction) {
    return solve(data, algorithm_type,
                 include_friction ? robot::DynamicsParamFlags::FRICTION
                                  : robot::DynamicsParamFlags::NONE);
  }

private:
  std::unique_ptr<robot::RobotModel> model_;
  std::unique_ptr<robot::Regressor> regressor_;
};

#endif // IDENTIFICATION_IDENTIFICATION_HPP_
