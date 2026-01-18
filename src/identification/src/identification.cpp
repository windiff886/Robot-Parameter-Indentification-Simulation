#include "identification/identification.hpp"
#include "robot/robot_dynamics.hpp" // Required for CLOE
#include <iostream>

Identification::Identification(std::unique_ptr<robot::RobotModel> model)
    : model_(std::move(model)) {
  regressor_ = std::make_unique<robot::Regressor>(*model_);
}

void Identification::preprocess(ExperimentData &data) {
  // Check if qdd was already loaded from file (physics engine data)
  if (data.qdd.rows() == static_cast<Eigen::Index>(data.n_samples) &&
      data.qdd.cols() == static_cast<Eigen::Index>(data.n_dof)) {
    std::cout << "Preprocessing: using qdd from physics engine (already loaded)"
              << std::endl;
    std::cout << "DEBUG: qdd max abs: " << data.qdd.cwiseAbs().maxCoeff()
              << std::endl;
    std::cout << "DEBUG: qdd norm: " << data.qdd.norm() << std::endl;
    return;
  }

  // Fallback: Numerical Differentiation for qdd
  // Using central difference: qdd[i] = (qd[i+1] - qd[i-1]) / (2*dt)
  std::cout << "Preprocessing: computing qdd via numerical differentiation..."
            << std::endl;

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
  std::cout << "DEBUG: qdd max abs: " << data.qdd.cwiseAbs().maxCoeff()
            << std::endl;
  std::cout << "DEBUG: qdd norm: " << data.qdd.norm() << std::endl;
}

Eigen::VectorXd Identification::solve(const ExperimentData &data,
                                      const std::string &algorithm_type,
                                      robot::DynamicsParamFlags flags) {
  if (data.qdd.rows() == 0) {
    throw std::runtime_error("Experiment data not preprocessed: qdd is empty");
  }

  std::cout << "Building observation matrix W..." << std::endl;

  // Filter out samples with excessive acceleration (outliers)
  // Threshold: 10.0 rad/s^2 (Standard is ~0.001 for this trajectory)
  const double qdd_threshold = 10.0;
  std::vector<std::size_t> valid_indices;
  valid_indices.reserve(data.n_samples);

  for (std::size_t i = 0; i < data.n_samples; ++i) {
    if (data.qdd.row(i).cwiseAbs().maxCoeff() < qdd_threshold) {
      valid_indices.push_back(i);
    }
  }

  if (valid_indices.empty()) {
    throw std::runtime_error(
        "No valid samples remaining after filtering outliers!");
  }

  std::cout << "Filtered " << (data.n_samples - valid_indices.size())
            << " outlier samples. " << valid_indices.size()
            << " valid samples remain." << std::endl;

  // Build filtered W and Tau
  // Cannot verify regressor_ directly via tool, assuming API matches.
  // Regressor::computeObservationMatrix expects full matrices.
  // We need to construct subset matrices.

  Eigen::MatrixXd q_valid(valid_indices.size(), data.n_dof);
  Eigen::MatrixXd qd_valid(valid_indices.size(), data.n_dof);
  Eigen::MatrixXd qdd_valid(valid_indices.size(), data.n_dof);
  Eigen::VectorXd Tau_meas(valid_indices.size() * data.n_dof);

  for (std::size_t k = 0; k < valid_indices.size(); ++k) {
    std::size_t idx = valid_indices[k];
    q_valid.row(k) = data.q.row(idx);
    qd_valid.row(k) = data.qd.row(idx);
    qdd_valid.row(k) = data.qdd.row(idx);
    for (std::size_t j = 0; j < data.n_dof; ++j) {
      Tau_meas(k * data.n_dof + j) = data.tau(idx, j);
    }
  }

  // W: (N*Samples) x (M_params)
  Eigen::MatrixXd W = regressor_->computeObservationMatrix(
      q_valid.transpose(), qd_valid.transpose(), qdd_valid.transpose(),
      flags);

  std::cout << "W size: " << W.rows() << " x " << W.cols() << std::endl;

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
