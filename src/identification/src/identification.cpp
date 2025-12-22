#include "identification/identification.hpp"
#include "robot/robot_dynamics.hpp" // Required for CLOE
#include <iostream>

Identification::Identification(std::unique_ptr<robot::RobotModel> model)
    : model_(std::move(model)) {
  regressor_ = std::make_unique<robot::Regressor>(*model_);
}

void Identification::preprocess(ExperimentData &data) {
  // 1. Numerical Differentiation for qdd
  // Using central difference: qdd[i] = (qd[i+1] - qd[i-1]) / (2*dt)

  data.qdd.resize(data.n_samples, data.n_dof);

  for (std::size_t i = 1; i < data.n_samples - 1; ++i) {
    double dt = data.time[i + 1] - data.time[i - 1]; // spans 2 steps

    if (dt < 1e-6)
      continue;

    for (std::size_t j = 0; j < data.n_dof; ++j) {
      data.qdd(i, j) = (data.qd(i + 1, j) - data.qd(i - 1, j)) / dt;
    }
  }

  // Boundary conditions: replicate neighbors
  data.qdd.row(0) = data.qdd.row(1);
  data.qdd.row(data.n_samples - 1) = data.qdd.row(data.n_samples - 2);

  std::cout << "Preprocessing complete: computed accelerations." << std::endl;
}

Eigen::VectorXd Identification::solve(const ExperimentData &data,
                                      const std::string &algorithm_type,
                                      bool include_friction) {
  if (data.qdd.rows() == 0) {
    throw std::runtime_error("Experiment data not preprocessed: qdd is empty");
  }

  std::cout << "Building observation matrix W..." << std::endl;
  // W: (N*Samples) x (M_params)
  Eigen::MatrixXd W = regressor_->computeObservationMatrix(
      data.q.transpose(), data.qd.transpose(), data.qdd.transpose(),
      include_friction);

  std::cout << "W size: " << W.rows() << " x " << W.cols() << std::endl;

  // Y_meas (tau vector): Stack all tau measurements
  Eigen::VectorXd Tau_meas(data.n_samples * data.n_dof);
  for (std::size_t i = 0; i < data.n_samples; ++i) {
    for (std::size_t j = 0; j < data.n_dof; ++j) {
      Tau_meas(i * data.n_dof + j) = data.tau(i, j);
    }
  }

  int dof = data.n_dof;

  // Handle CLOE case which needs special construction
  if (algorithm_type == "CLOE") {
    std::cout << "Initializing CLOE..." << std::endl;
    // Estimate dt
    double dt = 0.001;
    if (data.n_samples > 1) {
      dt = (data.time.back() - data.time[0]) / (data.n_samples - 1);
    }

    // Create dynamics instance using the model
    robot::RobotDynamics dynamics(*model_);

    auto cloe = std::make_unique<identification::CLOE>(dynamics, dof, dt);
    cloe->setExperimentData(data.q, data.qd, data.tau);

    std::cout << "Solving using CLOE..." << std::endl;
    return cloe->solve(W, Tau_meas);
  }

  // Use factory for others
  auto solver = identification::createAlgorithm(algorithm_type, dof);

  std::cout << "Solving using " << algorithm_type << "..." << std::endl;
  Eigen::VectorXd beta = solver->solve(W, Tau_meas);

  return beta;
}
