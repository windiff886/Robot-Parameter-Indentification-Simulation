/**
 * @file mujoco_identify.cpp
 * @brief 使用 MuJoCoRegressor 进行参数辨识
 *
 * 该程序使用与 MuJoCo 一致的坐标系和运动学，
 * 确保辨识结果可以正确预测 MuJoCo 仿真的扭矩。
 */

#include "identification/algorithms.hpp"
#include "identification/data_loader.hpp"
#include "mujoco_panda_dynamics.hpp"
#include "mujoco_regressor.hpp"
#include <Eigen/Core>
#include <iomanip>
#include <iostream>

using namespace mujoco_dynamics;

void printIdentifiedParams(const Eigen::VectorXd &theta,
                           const std::vector<std::string> &names) {
  (void)names;
  std::cout << "\n[辨识结果]\n";

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
  std::cout << "  MuJoCo 参数辨识程序\n";
  std::cout << std::string(80, '=') << "\n";

  // 默认数据文件
  std::string data_file = "/home/windiff/Code/Simulation/data/"
                          "benchmark_data_2026-01-18_15-54-50.csv";
  if (argc > 1) {
    data_file = argv[1];
  }

  // 加载数据
  DataLoader loader;
  ExperimentData data = loader.loadCSV(data_file, 7);
  std::cout << "\n加载样本数: " << data.n_samples << "\n";

  // 创建回归矩阵计算器
  MuJoCoRegressor regressor;
  MuJoCoPandaDynamics dynamics;

  auto names = regressor.getParameterNames(MuJoCoParamFlags::ALL);
  std::size_t num_params = regressor.numParameters(MuJoCoParamFlags::ALL);
  std::cout << "参数数量: " << num_params << "\n";

  // 过滤异常样本
  const double qdd_threshold = 10.0;
  std::vector<std::size_t> valid_indices;
  valid_indices.reserve(data.n_samples);

  for (std::size_t i = 0; i < data.n_samples; ++i) {
    if (data.qdd.row(i).cwiseAbs().maxCoeff() < qdd_threshold) {
      valid_indices.push_back(i);
    }
  }
  std::cout << "有效样本数: " << valid_indices.size() << " (过滤了 "
            << (data.n_samples - valid_indices.size()) << " 个异常样本)\n";

  // 构建观测矩阵 W 和扭矩向量 Tau
  std::size_t K = valid_indices.size();
  Eigen::MatrixXd W(7 * K, num_params);
  Eigen::VectorXd Tau(7 * K);

  std::cout << "构建观测矩阵...\n";
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
  std::cout << "W 大小: " << W.rows() << " x " << W.cols() << "\n";

  // 使用 OLS 求解
  std::cout << "\n使用 OLS 求解...\n";
  auto solver = identification::createAlgorithm("OLS", 7);
  Eigen::VectorXd theta_identified = solver->solve(W, Tau);

  // 打印结果
  printIdentifiedParams(theta_identified, names);

  // 与 Ground Truth 对比
  std::cout << "\n" << std::string(80, '-') << "\n";
  std::cout << "  与 Ground Truth 对比\n";
  std::cout << std::string(80, '-') << "\n";

  Eigen::VectorXd theta_true =
      regressor.computeParameterVector(MuJoCoParamFlags::ALL);
  Eigen::VectorXd param_error = theta_identified - theta_true;

  double param_rmse = std::sqrt(param_error.squaredNorm() / num_params);
  std::cout << "参数 RMSE: " << param_rmse << "\n";

  // 计算扭矩误差
  double torque_sq_error = 0.0;
  std::size_t test_count = std::min(K, (std::size_t)1000);
  for (std::size_t k = 0; k < test_count; ++k) {
    Eigen::VectorXd tau_pred =
        W.block(k * 7, 0, 7, num_params) * theta_identified;
    Eigen::VectorXd tau_meas = Tau.segment(k * 7, 7);
    torque_sq_error += (tau_pred - tau_meas).squaredNorm();
  }
  double torque_rmse = std::sqrt(torque_sq_error / (test_count * 7));
  std::cout << "扭矩预测 RMSE: " << torque_rmse << " Nm\n";

  // 与 Dynamics 预测对比
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
  std::cout << "MuJoCoPandaDynamics 扭矩 RMSE (基准): " << dynamics_rmse
            << " Nm\n";

  std::cout << "\n" << std::string(80, '=') << "\n";
  if (torque_rmse < dynamics_rmse * 1.5) {
    std::cout << "  [OK] 辨识成功！\n";
  } else {
    std::cout << "  [WARNING] 辨识结果扭矩误差较大\n";
  }
  std::cout << std::string(80, '=') << "\n";

  return 0;
}
