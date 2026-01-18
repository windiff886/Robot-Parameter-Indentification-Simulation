#include "identification/identification.hpp"
#include "mujoco_panda_dynamics.hpp"
#include <iostream>

using namespace mujoco_dynamics;

Identification::Identification(std::unique_ptr<robot::RobotModel> model)
    : model_(std::move(model)) {
  // MuJoCoRegressor 在成员初始化时自动构造
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
  std::cout << "Preprocessing: computing qdd via numerical differentiation..."
            << std::endl;

  data.qdd.resize(data.n_samples, data.n_dof);

  for (std::size_t i = 1; i < data.n_samples - 1; ++i) {
    double dt = data.time[i + 1] - data.time[i - 1];

    if (dt < 1e-6)
      continue;

    for (std::size_t j = 0; j < data.n_dof; ++j) {
      data.qdd(i, j) = (data.qd(i + 1, j) - data.qd(i - 1, j)) / dt;
    }
  }

  data.qdd.row(0) = data.qdd.row(1);
  data.qdd.row(data.n_samples - 1) = data.qdd.row(data.n_samples - 2);

  std::cout << "Preprocessing complete: computed accelerations." << std::endl;
  std::cout << "DEBUG: qdd max abs: " << data.qdd.cwiseAbs().maxCoeff()
            << std::endl;
  std::cout << "DEBUG: qdd norm: " << data.qdd.norm() << std::endl;
}

std::size_t Identification::numParameters(MuJoCoParamFlags flags) const {
  return regressor_.numParameters(flags);
}

Eigen::VectorXd
Identification::getGroundTruthParameters(MuJoCoParamFlags flags) const {
  return regressor_.computeParameterVector(flags);
}

Eigen::MatrixXd Identification::computeObservationMatrix(
    const Eigen::MatrixXd &Q, const Eigen::MatrixXd &Qd,
    const Eigen::MatrixXd &Qdd, MuJoCoParamFlags flags) const {
  return regressor_.computeObservationMatrix(Q, Qd, Qdd, flags);
}

Eigen::VectorXd Identification::solve(const ExperimentData &data,
                                      const std::string &algorithm_type,
                                      MuJoCoParamFlags flags) {
  if (data.qdd.rows() == 0) {
    throw std::runtime_error("Experiment data not preprocessed: qdd is empty");
  }

  std::cout << "Building observation matrix W (using MuJoCoRegressor)..."
            << std::endl;

  // Filter out samples with excessive acceleration
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

  // Build filtered data matrices
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

  // Use MuJoCoRegressor to compute observation matrix
  Eigen::MatrixXd W = regressor_.computeObservationMatrix(
      q_valid.transpose(), qd_valid.transpose(), qdd_valid.transpose(), flags);

  std::cout << "W size: " << W.rows() << " x " << W.cols() << std::endl;

  // Use factory to create solver
  auto solver = identification::createAlgorithm(algorithm_type, data.n_dof);

  // CLOE 返回 nullptr，因为它需要 RobotDynamics 引用，目前不支持
  if (!solver) {
    std::cout << "WARNING: " << algorithm_type
              << " algorithm is not fully supported with MuJoCoRegressor. "
              << "Falling back to OLS." << std::endl;
    solver = identification::createAlgorithm("OLS", data.n_dof);
  }

  std::cout << "Solving using " << algorithm_type << "..." << std::endl;
  Eigen::VectorXd beta = solver->solve(W, Tau_meas);

  return beta;
}
