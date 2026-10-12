/**
 * @file mujoco_identify.cpp
 * @brief Parameter identification using MuJoCoRegressor
 *
 * This program uses coordinate frames and kinematics consistent with MuJoCo,
 * ensuring identification results accurately predict MuJoCo simulation torques.
 */

#include "identification/algorithms.hpp"
#include "identification/data_loader.hpp"
#include "mujoco_panda_dynamics.hpp"
#include "mujoco_regressor.hpp"
#include <Eigen/Core>
#include <filesystem>
#include <iomanip>
#include <iostream>

using namespace mujoco_dynamics;

void printIdentifiedParams(const Eigen::VectorXd &theta,
                           const std::vector<std::string> &names) {
  (void)names;
  std::cout << "\n[Identification Results]\n";

  const char *body_names[] = {"link1", "link2", "link3", "link4",
                              "link5", "link6", "link7", "hand"};

  for (std::size_t body = 0; body < 8; ++body) {
    std::cout << "\n  " << body_names[body] << ":\n";
    std::size_t offset = body * 10;
    std::cout << std::fixed << std::setprecision(6);
    std::cout << "    m     = " << std::setw(12) << theta(offset + 0) << "\n";
    std::cout << "    mx    = " << std::setw(12) << theta(offset + 1) << "\n";
    std::cout << "    my    = " << std::setw(12) << theta(offset + 2) << "\n";
    std::cout << "    mz    = " << std::setw(12) << theta(offset + 3) << "\n";
    std::cout << "    Ixx   = " << std::setw(12) << theta(offset + 4) << "\n";
    std::cout << "    Ixy   = " << std::setw(12) << theta(offset + 5) << "\n";
    std::cout << "    Ixz   = " << std::setw(12) << theta(offset + 6) << "\n";
    std::cout << "    Iyy   = " << std::setw(12) << theta(offset + 7) << "\n";
    std::cout << "    Iyz   = " << std::setw(12) << theta(offset + 8) << "\n";
    std::cout << "    Izz   = " << std::setw(12) << theta(offset + 9) << "\n";
  }

  std::cout << "\n  Armature:\n    ";
  for (int i = 0; i < 7; ++i) {
    std::cout << std::setw(10) << theta(80 + i);
  }
  std::cout << "\n";

  std::cout << "\n  Damping:\n    ";
  for (int i = 0; i < 7; ++i) {
    std::cout << std::setw(10) << theta(87 + i);
  }
  std::cout << "\n";
}

int main(int argc, char **argv) {
  std::cout << std::string(80, '=') << "\n";
  std::cout << "  MuJoCo Parameter Identification Program\n";
  std::cout << std::string(80, '=') << "\n";

  // Default data file
  std::string data_file =
      (std::filesystem::path(PROJECT_ROOT_DIR) / "data" / "benchmark_data.csv")
          .string();
  if (argc > 1) {
    data_file = argv[1];
  }

  // Load data
  DataLoader loader;
  ExperimentData data = loader.loadCSV(data_file, 7);
  std::cout << "\nLoaded sample count: " << data.n_samples << "\n";

  // Create regressor matrix computer
  MuJoCoRegressor regressor;
  MuJoCoPandaDynamics dynamics;

  auto names = regressor.getParameterNames(MuJoCoParamFlags::ALL);
  std::size_t num_params = regressor.numParameters(MuJoCoParamFlags::ALL);
  std::cout << "Number of parameters: " << num_params << "\n";

  // Filter anomalous samples
  const double qdd_threshold = 10.0;
  std::vector<std::size_t> valid_indices;
  valid_indices.reserve(data.n_samples);

  for (std::size_t i = 0; i < data.n_samples; ++i) {
    if (data.qdd.row(i).cwiseAbs().maxCoeff() < qdd_threshold) {
      valid_indices.push_back(i);
    }
  }
  std::cout << "Valid sample count: " << valid_indices.size() << " (filtered "
            << (data.n_samples - valid_indices.size()) << " anomalous samples)\n";

  // Build observation matrix W and torque vector Tau
  std::size_t K = valid_indices.size();
  Eigen::MatrixXd W(7 * K, num_params);
  Eigen::VectorXd Tau(7 * K);

  std::cout << "Building observation matrix...\n";
  for (std::size_t k = 0; k < K; ++k) {
    std::size_t idx = valid_indices[k];
    Eigen::VectorXd q = data.q.row(idx).transpose();
    Eigen::VectorXd qd = data.qd.row(idx).transpose();
    Eigen::VectorXd qdd = data.qdd.row(idx).transpose();

    Eigen::MatrixXd Y =
        regressor.computeRegressorMatrix(q, qd, qdd, MuJoCoParamFlags::ALL);
    W.block(k * 7, 0, 7, num_params) = Y;

    for (int j = 0; j < 7; ++j) {
      Tau(k * 7 + j) = data.tau(idx, j);
    }
  }
  std::cout << "W size: " << W.rows() << " x " << W.cols() << "\n";

  // Solve using OLS
  std::cout << "\nSolving with OLS...\n";
  auto solver = identification::createAlgorithm("OLS", 7);
  Eigen::VectorXd theta_identified = solver->solve(W, Tau);

  // Print results
  printIdentifiedParams(theta_identified, names);

  // Compare with Ground Truth
  std::cout << "\n" << std::string(80, '-') << "\n";
  std::cout << "  Comparison with Ground Truth\n";
  std::cout << std::string(80, '-') << "\n";

  Eigen::VectorXd theta_true =
      regressor.computeParameterVector(MuJoCoParamFlags::ALL);
  Eigen::VectorXd param_error = theta_identified - theta_true;

  double param_rmse = std::sqrt(param_error.squaredNorm() / num_params);
  std::cout << "Parameter RMSE: " << param_rmse << "\n";

  // Compute torque prediction error
  double torque_sq_error = 0.0;
  std::size_t test_count = std::min(K, (std::size_t)1000);
  for (std::size_t k = 0; k < test_count; ++k) {
    Eigen::VectorXd tau_pred =
        W.block(k * 7, 0, 7, num_params) * theta_identified;
    Eigen::VectorXd tau_meas = Tau.segment(k * 7, 7);
    torque_sq_error += (tau_pred - tau_meas).squaredNorm();
  }
  double torque_rmse = std::sqrt(torque_sq_error / (test_count * 7));
  std::cout << "Torque prediction RMSE: " << torque_rmse << " Nm\n";

  // Compare with Dynamics prediction
  double dynamics_sq_error = 0.0;
  for (std::size_t k = 0; k < test_count; ++k) {
    std::size_t idx = valid_indices[k];
    Eigen::VectorXd q = data.q.row(idx).transpose();
    Eigen::VectorXd qd = data.qd.row(idx).transpose();
    Eigen::VectorXd qdd = data.qdd.row(idx).transpose();

    Eigen::VectorXd tau_dyn = dynamics.computeInverseDynamics(q, qd, qdd);
    Eigen::VectorXd tau_meas = data.tau.row(idx).transpose();
    dynamics_sq_error += (tau_dyn - tau_meas).squaredNorm();
  }
  double dynamics_rmse = std::sqrt(dynamics_sq_error / (test_count * 7));
  std::cout << "MuJoCoPandaDynamics torque RMSE (baseline): " << dynamics_rmse
            << " Nm\n";

  std::cout << "\n" << std::string(80, '=') << "\n";
  if (torque_rmse < dynamics_rmse * 1.5) {
    std::cout << "  [OK] Identification succeeded!\n";
  } else {
    std::cout << "  [WARNING] High torque residual in identification results\n";
  }
  std::cout << std::string(80, '=') << "\n";

  return 0;
}
