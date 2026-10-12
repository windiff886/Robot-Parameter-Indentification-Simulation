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
 * @brief Parameter identification class (switches MuJoCo regressor by robot type)
 *
 * Computes regressor matrix using coordinate frames consistent with MuJoCo,
 * ensuring identification results accurately predict MuJoCo simulation torque.
 */
class Identification {
public:
  /**
   * @brief Constructor
   *
   * @param robot_type Robot manipulator type, supports "panda" / "piper"
   * @param model Retained robot model pointer for compatibility
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
   * @param flags Dynamics parameter flags (ARMATURE, DAMPING)
   * @return Identified parameters beta (94 parameters for Panda)
   *
   * Structure of parameter vector beta:
   *   [inertial parameters (10*8)] + [armature (7)] + [damping (7)] = 94 parameters
   */
  Eigen::VectorXd solve(const ExperimentData &data,
                        const std::string &algorithm_type = "OLS",
                        mujoco_dynamics::MuJoCoParamFlags flags =
                            mujoco_dynamics::MuJoCoParamFlags::ALL);

  /**
   * @brief Get number of parameters
   */
  std::size_t numParameters(mujoco_dynamics::MuJoCoParamFlags flags =
                                mujoco_dynamics::MuJoCoParamFlags::ALL) const;

  /**
   * @brief Get Ground Truth parameter vector (for validation)
   */
  Eigen::VectorXd
  getGroundTruthParameters(mujoco_dynamics::MuJoCoParamFlags flags =
                               mujoco_dynamics::MuJoCoParamFlags::ALL) const;

  /**
   * @brief Compute observation matrix W
   */
  Eigen::MatrixXd
  computeObservationMatrix(const Eigen::MatrixXd &Q, const Eigen::MatrixXd &Qd,
                           const Eigen::MatrixXd &Qdd,
                           mujoco_dynamics::MuJoCoParamFlags flags =
                               mujoco_dynamics::MuJoCoParamFlags::ALL) const;

  /**
   * @brief Predict stacked torque vector for entire experiment using specified algorithm and parameters
   */
  Eigen::VectorXd
  predictTorques(const ExperimentData &data, const Eigen::VectorXd &params,
                 const std::string &algorithm_type = "OLS",
                 mujoco_dynamics::MuJoCoParamFlags flags =
                     mujoco_dynamics::MuJoCoParamFlags::ALL) const;

private:
  std::unique_ptr<robot::RobotModel> model_; // Retained for compatibility
  std::string robot_type_;
  mujoco_dynamics::MuJoCoRegressor panda_regressor_;
  mujoco_dynamics::MuJoCoPiperRegressor piper_regressor_;
};

#endif // IDENTIFICATION_IDENTIFICATION_HPP_
