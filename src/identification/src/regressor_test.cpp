/**
 * @file regressor_test.cpp
 * @brief 验证 MuJoCoRegressor 的正确性
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
  std::cout << "  MuJoCoRegressor 验证测试\n";
  std::cout << std::string(80, '=') << "\n";

  mujoco_dynamics::MuJoCoRegressor regressor;
  mujoco_dynamics::MuJoCoPandaDynamics dynamics;

  auto theta_true =
      regressor.computeParameterVector(mujoco_dynamics::MuJoCoParamFlags::ALL);
  std::cout << "\n参数数量: " << theta_true.size() << "\n";

  // ====================================================================
  // 测试 1: 单点验证
  // ====================================================================
  std::cout << "\n" << std::string(80, '-') << "\n";
  std::cout
      << "  测试 1: 与 MuJoCoPandaDynamics::computeInverseDynamics 对比\n";
  std::cout << std::string(80, '-') << "\n";

  Eigen::VectorXd q(7), qd(7), qdd(7);
  q << 0.0, 0.0, 0.0, -M_PI / 2, 0.0, M_PI / 2, -0.7853;
  qd << 0.1, -0.2, 0.15, -0.1, 0.25, -0.05, 0.1;
  qdd << 0.5, -0.3, 0.2, 0.4, -0.2, 0.1, -0.15;

  auto Y = regressor.computeRegressorMatrix(
      q, qd, qdd, mujoco_dynamics::MuJoCoParamFlags::ALL);
  Eigen::VectorXd tau_regressor = Y * theta_true;
  Eigen::VectorXd tau_dynamics = dynamics.computeInverseDynamics(q, qd, qdd);

  printVector("τ (Regressor: Y * θ)", tau_regressor);
  printVector("τ (Dynamics: ID)", tau_dynamics);
  printVector("差值", tau_regressor - tau_dynamics);

  double rmse = std::sqrt((tau_regressor - tau_dynamics).squaredNorm() / 7.0);
  std::cout << "\n  RMSE (Regressor vs Dynamics): " << rmse << " Nm\n";

  if (rmse < 0.2) {
    std::cout << "  [OK] Regressor 与 Dynamics 结果一致！\n";
  } else {
    std::cout << "  [ERROR] Regressor 与 Dynamics 结果不一致！\n";
  }

  // ====================================================================
  // 测试 2: 多样本验证（与实验数据对比）
  // ====================================================================
  std::cout << "\n" << std::string(80, '-') << "\n";
  std::cout << "  测试 2: 多样本验证（与 MuJoCo 记录数据对比）\n";
  std::string data_file = "/home/windiff/Code/Simulation/data/"
                          "benchmark_data_2026-01-18_15-54-50.csv";

  DataLoader loader;
  ExperimentData data = loader.loadCSV(data_file, 7);
  std::cout << "  加载样本数: " << data.n_samples << "\n";
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

    // MuJoCo computeInverseDynamics 返回 τ = M*qdd + C*qd + G - damping*qd
    // 但记录的 tau 是控制扭矩 τ_ctrl = M*qdd + C*qd + G - damping*qd
    // 所以应该直接一致

    std::cout << "\n  Sample " << idx << " (t = " << data.time[idx] << " s):\n";
    printVector("τ (Regressor)", tau_regressor);
    printVector("τ (Dynamics)", tau_dynamics);
    printVector("τ (Recorded)", tau_recorded);

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
  // 总结
  // ====================================================================
  std::cout << "\n" << std::string(80, '=') << "\n";
  std::cout << "  总结\n";
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
    std::cout << "\n  [OK] MuJoCoRegressor 验证通过！可用于参数辨识。\n";
  } else if (overall_rmse_vs_recorded < 1.0) {
    std::cout << "\n  [WARNING] RMSE 略高，可能存在一些偏差。\n";
  } else {
    std::cout << "\n  [ERROR] MuJoCoRegressor 验证失败！\n";
  }

  return 0;
}
