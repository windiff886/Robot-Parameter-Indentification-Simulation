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
   * @param include_friction whether to identify friction parameters
   * @return Identified parameters beta
   */
  Eigen::VectorXd solve(const ExperimentData &data,
                        const std::string &algorithm_type = "OLS",
                        bool include_friction = true);

private:
  std::unique_ptr<robot::RobotModel> model_;
  std::unique_ptr<robot::Regressor> regressor_;
};

#endif // IDENTIFICATION_IDENTIFICATION_HPP_
