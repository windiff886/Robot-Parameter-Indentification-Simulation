/**
 * @file regressor_test.cpp
 * @brief Validate the correctness of MuJoCoRegressor
 */

#include "identification/data_loader.hpp"
#include "mujoco_panda_dynamics.hpp"
#include "mujoco_regressor.hpp"
#include <Eigen/Core>
#include <iomanip>
#include <iostream>

void printVector(const std::string &name, const Eigen::VectorXd &v) {
  std::cout << "  " << std::setw(30) << std::left << name << ": [";
  for (int i = 0; i < v.size(); ++i) {
    std::cout << std::setw(10) << std::fixed << std::setprecision(4) << v(i);
    if (i < v.size() - 1)
      std::cout << ", ";
  }
  std::cout << "]\n";
}

int main() {
  std::cout << std::string(80, '=') << "\n";
  std::cout << "  MuJoCoRegressor Validation Test\n";
  std::cout << std::string(80, '=') << "\n";

  mujoco_dynamics::MuJoCoRegressor regressor;
  mujoco_dynamics::MuJoCoPandaDynamics dynamics;

  auto theta_true =
      regressor.computeParameterVector(mujoco_dynamics::MuJoCoParamFlags::ALL);
  std::cout << "\nNumber of parameters: " << theta_true.size() << "\n";

  // ====================================================================
  // Test 1: Single-point validation
  // ====================================================================
  std::cout << "\n" << std::string(80, '-') << "\n";
  std::cout
      << "  Test 1: Comparison with MuJoCoPandaDynamics::computeInverseDynamics\n";
  std::cout << std::string(80, '-') << "\n";

  Eigen::VectorXd q(7), qd(7), qdd(7);
  q << 0.0, 0.0, 0.0, -M_PI / 2, 0.0, M_PI / 2, -0.7853;
  qd << 0.1, -0.2, 0.15, -0.1, 0.25, -0.05, 0.1;
  qdd << 0.5, -0.3, 0.2, 0.4, -0.2, 0.1, -0.15;

  auto Y = regressor.computeRegressorMatrix(
      q, qd, qdd, mujoco_dynamics::MuJoCoParamFlags::ALL);
  Eigen::VectorXd tau_regressor = Y * theta_true;
  Eigen::VectorXd tau_dynamics = dynamics.computeInverseDynamics(q, qd, qdd);

  printVector("tau (Regressor: Y * theta)", tau_regressor);
  printVector("tau (Dynamics: ID)", tau_dynamics);
  printVector("Difference", tau_regressor - tau_dynamics);

  double rmse = std::sqrt((tau_regressor - tau_dynamics).squaredNorm() / 7.0);
  std::cout << "\n  RMSE (Regressor vs Dynamics): " << rmse << " Nm\n";

  if (rmse < 0.2) {
    std::cout << "  [OK] Regressor matches Dynamics results!\n";
  } else {
    std::cout << "  [ERROR] Regressor does not match Dynamics results!\n";
  }

  // ====================================================================
  // Test 2: Multi-sample validation (comparison with experimental data)
  // ====================================================================
  std::cout << "\n" << std::string(80, '-') << "\n";
  std::cout << "  Test 2: Multi-sample validation (comparison with MuJoCo recorded data)\n";
  std::string data_file = "/home/windiff/Code/Simulation/data/"
                          "benchmark_data_2026-01-18_15-54-50.csv";

  DataLoader loader;
  ExperimentData data = loader.loadCSV(data_file, 7);
  std::cout << "  Loaded sample count: " << data.n_samples << "\n";
  std::cout << std::string(80, '-') << "\n";

  double sum_sq_error_vs_dynamics = 0.0;
  double sum_sq_error_vs_recorded = 0.0;
  std::size_t valid_samples = 0;

  std::vector<std::size_t> test_indices = {100, 500, 1000, 2000, 3000, 5000};

  for (std::size_t idx : test_indices) {
    if (idx >= data.n_samples)
      continue;

    q = data.q.row(idx).transpose();
    qd = data.qd.row(idx).transpose();
    qdd = data.qdd.row(idx).transpose();
    Eigen::VectorXd tau_recorded = data.tau.row(idx).transpose();

    if (tau_recorded.norm() < 1e-6)
      continue;

    Y = regressor.computeRegressorMatrix(
        q, qd, qdd, mujoco_dynamics::MuJoCoParamFlags::ALL);
    tau_regressor = Y * theta_true;
    tau_dynamics = dynamics.computeInverseDynamics(q, qd, qdd);

    // MuJoCo computeInverseDynamics returns tau = M*qdd + C*qd + G - damping*qd
    // And recorded tau is control torque tau_ctrl = M*qdd + C*qd + G - damping*qd
    // Therefore they should match directly

    std::cout << "\n  Sample " << idx << " (t = " << data.time[idx] << " s):\n";
    printVector("tau (Regressor)", tau_regressor);
    printVector("tau (Dynamics)", tau_dynamics);
    printVector("tau (Recorded)", tau_recorded);

    Eigen::VectorXd err_vs_dynamics = tau_regressor - tau_dynamics;
    Eigen::VectorXd err_vs_recorded = tau_regressor - tau_recorded;

    double rmse_vs_dynamics = std::sqrt(err_vs_dynamics.squaredNorm() / 7.0);
    double rmse_vs_recorded = std::sqrt(err_vs_recorded.squaredNorm() / 7.0);

    std::cout << "    RMSE vs Dynamics: " << rmse_vs_dynamics << " Nm\n";
    std::cout << "    RMSE vs Recorded: " << rmse_vs_recorded << " Nm\n";

    sum_sq_error_vs_dynamics += err_vs_dynamics.squaredNorm();
    sum_sq_error_vs_recorded += err_vs_recorded.squaredNorm();
    valid_samples += 7;
  }

  // ====================================================================
  // Summary
  // ====================================================================
  std::cout << "\n" << std::string(80, '=') << "\n";
  std::cout << "  Summary\n";
  std::cout << std::string(80, '=') << "\n";

  double overall_rmse_vs_dynamics =
      std::sqrt(sum_sq_error_vs_dynamics / valid_samples);
  double overall_rmse_vs_recorded =
      std::sqrt(sum_sq_error_vs_recorded / valid_samples);

  std::cout << "  Overall RMSE vs Dynamics: " << overall_rmse_vs_dynamics
            << " Nm\n";
  std::cout << "  Overall RMSE vs Recorded: " << overall_rmse_vs_recorded
            << " Nm\n";

  if (overall_rmse_vs_dynamics < 0.2 && overall_rmse_vs_recorded < 0.5) {
    std::cout << "\n  [OK] MuJoCoRegressor validation passed! Ready for parameter identification.\n";
  } else if (overall_rmse_vs_recorded < 1.0) {
    std::cout << "\n  [WARNING] RMSE slightly elevated; minor discrepancies may exist.\n";
  } else {
    std::cout << "\n  [ERROR] MuJoCoRegressor validation failed!\n";
  }

  return 0;
}
