#include "identification/data_loader.hpp"
#include "identification/identification.hpp"
#include "mujoco_panda_dynamics.hpp"
#include "robot/franka_panda.hpp"

#include <Eigen/Core>

#include <algorithm>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace fs = std::filesystem;

namespace {

struct IdentificationConfig {
  int algorithm = 0;
  std::string data_file =
      (fs::path(PROJECT_ROOT_DIR) / "data" / "benchmark_data.csv").string();
  std::string output_file =
      (fs::path(PROJECT_ROOT_DIR) / "results" / "identification.yaml").string();
};

struct BenchmarkResult {
  std::string name;
  Eigen::VectorXd beta;
  double torque_rmse = 0.0;
  double torque_max_error = 0.0;
  bool success = false;
};

std::string trim(const std::string &value) {
  const auto first = value.find_first_not_of(" \t\r\n");
  if (first == std::string::npos) {
    return "";
  }
  const auto last = value.find_last_not_of(" \t\r\n");
  return value.substr(first, last - first + 1);
}

std::string removeComment(const std::string &value) {
  const auto pos = value.find('#');
  return pos == std::string::npos ? value : value.substr(0, pos);
}

bool parseInt(const std::string &line, const std::string &key, int &out) {
  const std::string trimmed = trim(line);
  if (trimmed.rfind(key + ":", 0) != 0) {
    return false;
  }
  try {
    out = std::stoi(trim(removeComment(trimmed.substr(key.size() + 1))));
    return true;
  } catch (const std::exception &) {
    return false;
  }
}

bool parseString(const std::string &line, const std::string &key,
                 std::string &out) {
  const std::string trimmed = trim(line);
  if (trimmed.rfind(key + ":", 0) != 0) {
    return false;
  }

  std::string value = trim(removeComment(trimmed.substr(key.size() + 1)));
  if (value.size() >= 2 &&
      ((value.front() == '"' && value.back() == '"') ||
       (value.front() == '\'' && value.back() == '\''))) {
    value = value.substr(1, value.size() - 2);
  }
  out = value;
  return true;
}

IdentificationConfig loadConfig(const fs::path &config_path) {
  IdentificationConfig config;
  std::ifstream file(config_path);
  if (!file) {
    return config;
  }

  std::string line;
  while (std::getline(file, line)) {
    parseInt(line, "algorithm", config.algorithm);
    parseString(line, "data_file", config.data_file);
    parseString(line, "output_file", config.output_file);
  }

  const fs::path base_dir = fs::path(PROJECT_ROOT_DIR);
  if (!config.data_file.empty()) {
    fs::path data_path(config.data_file);
    if (data_path.is_relative()) {
      config.data_file = (base_dir / data_path).string();
    }
  }
  if (!config.output_file.empty()) {
    fs::path output_path(config.output_file);
    if (output_path.is_relative()) {
      config.output_file = (base_dir / output_path).string();
    }
  }

  return config;
}

std::string algorithmName(int algorithm_id) {
  switch (algorithm_id) {
  case 0:
  case 1:
    return "OLS";
  case 2:
    return "WLS";
  case 3:
    return "IRLS";
  case 4:
    return "TLS";
  case 5:
    return "EKF";
  case 6:
    return "ML";
  case 7:
    return "CLOE";
  default:
    return "OLS";
  }
}

std::tm localTime(std::time_t raw_time) {
  std::tm result{};
#ifdef _WIN32
  localtime_s(&result, &raw_time);
#else
  localtime_r(&raw_time, &result);
#endif
  return result;
}

void saveResults(const fs::path &output_path, int algorithm_id,
                 const std::vector<BenchmarkResult> &results) {
  if (output_path.has_parent_path()) {
    fs::create_directories(output_path.parent_path());
  }

  std::ofstream out(output_path);
  if (!out) {
    throw std::runtime_error("无法写入结果文件: " + output_path.string());
  }

  auto now = std::chrono::system_clock::now();
  const auto raw_time = std::chrono::system_clock::to_time_t(now);
  const auto tm_now = localTime(raw_time);

  out << "calibration_date: \"" << std::put_time(&tm_now, "%Y-%m-%d %H:%M:%S")
      << "\"\n";
  out << "evaluation_method: \"torque_residual_rmse\"\n";

  if (algorithm_id == 0) {
    out << "mode: \"BENCHMARK_ALL\"\n";
    out << "benchmark_results:\n";
    for (const auto &result : results) {
      out << "  - algorithm: \"" << result.name << "\"\n";
      if (!result.success) {
        out << "    status: \"FAILED\"\n";
        continue;
      }
      out << "    torque_rmse: " << result.torque_rmse << "\n";
      out << "    torque_max_error: " << result.torque_max_error << "\n";
      out << "    parameters:\n";
      for (int i = 0; i < result.beta.size(); ++i) {
        out << "      - " << result.beta(i) << "\n";
      }
    }
    return;
  }

  if (results.empty()) {
    return;
  }

  const auto &result = results.front();
  out << "algorithm: \"" << result.name << "\"\n";
  out << "evaluation_results:\n";
  out << "  torque_rmse: " << result.torque_rmse << "\n";
  out << "  torque_max_error: " << result.torque_max_error << "\n";
  out << "parameters:\n";
  for (int i = 0; i < result.beta.size(); ++i) {
    out << "  - " << result.beta(i) << "\n";
  }
}

} // namespace

int main(int argc, char **argv) {
  try {
    fs::path config_path = fs::path(PROJECT_ROOT_DIR) / "config" / "identification.yaml";
    for (int i = 1; i + 1 < argc; ++i) {
      if (std::string(argv[i]) == "--config") {
        config_path = argv[i + 1];
      }
    }

    IdentificationConfig config = loadConfig(config_path);
    for (int i = 1; i + 1 < argc; ++i) {
      const std::string flag = argv[i];
      const std::string value = argv[i + 1];
      if (flag == "--data-file") {
        config.data_file = value;
      } else if (flag == "--output-file") {
        config.output_file = value;
      } else if (flag == "--algorithm") {
        config.algorithm = std::stoi(value);
      }
    }

    if (config.data_file.empty()) {
      throw std::runtime_error("缺少 data_file");
    }

    std::cout << "加载数据: " << config.data_file << std::endl;
    ExperimentData data = DataLoader::loadCSV(config.data_file, 7);
    std::cout << "样本数: " << data.n_samples << std::endl;

    auto robot = robot::createFrankaPanda();
    Identification identifier(std::move(robot));
    identifier.preprocess(data);

    {
      mujoco_dynamics::MuJoCoPandaDynamics dynamics;
      double sum_sq_error = 0.0;
      double max_error = 0.0;
      const std::size_t n_dof = 7;
      for (std::size_t i = 0; i < data.n_samples; ++i) {
        const Eigen::VectorXd q = data.q.row(i).transpose();
        const Eigen::VectorXd qd = data.qd.row(i).transpose();
        const Eigen::VectorXd qdd = data.qdd.row(i).transpose();
        const Eigen::VectorXd tau = dynamics.computeInverseDynamics(q, qd, qdd);
        for (std::size_t j = 0; j < n_dof; ++j) {
          const double error = tau(static_cast<Eigen::Index>(j)) - data.tau(i, j);
          sum_sq_error += error * error;
          max_error = std::max(max_error, std::abs(error));
        }
      }
      const double rmse =
          std::sqrt(sum_sq_error / (static_cast<double>(data.n_samples) * 7.0));
      std::cout << "MuJoCo 动力学基准 RMSE: " << rmse << " Nm, Max Error: "
                << max_error << " Nm" << std::endl;
    }

    const std::size_t n_train =
        static_cast<std::size_t>(static_cast<double>(data.n_samples) * 0.8);
    const std::size_t n_val = data.n_samples - n_train;

    ExperimentData train_data;
    train_data.n_samples = n_train;
    train_data.n_dof = data.n_dof;
    train_data.time = std::vector<double>(data.time.begin(), data.time.begin() + n_train);
    train_data.q = data.q.topRows(static_cast<Eigen::Index>(n_train));
    train_data.qd = data.qd.topRows(static_cast<Eigen::Index>(n_train));
    train_data.qdd = data.qdd.topRows(static_cast<Eigen::Index>(n_train));
    train_data.tau = data.tau.topRows(static_cast<Eigen::Index>(n_train));

    ExperimentData val_data;
    val_data.n_samples = n_val;
    val_data.n_dof = data.n_dof;
    val_data.time = std::vector<double>(data.time.begin() + n_train, data.time.end());
    val_data.q = data.q.bottomRows(static_cast<Eigen::Index>(n_val));
    val_data.qd = data.qd.bottomRows(static_cast<Eigen::Index>(n_val));
    val_data.qdd = data.qdd.bottomRows(static_cast<Eigen::Index>(n_val));
    val_data.tau = data.tau.bottomRows(static_cast<Eigen::Index>(n_val));

    auto flags = mujoco_dynamics::MuJoCoParamFlags::ARMATURE |
                 mujoco_dynamics::MuJoCoParamFlags::DAMPING;
    const Eigen::MatrixXd W_val = identifier.computeObservationMatrix(
        val_data.q.transpose(), val_data.qd.transpose(), val_data.qdd.transpose(),
        flags);

    Eigen::VectorXd Tau_val(val_data.n_samples * val_data.n_dof);
    for (std::size_t i = 0; i < val_data.n_samples; ++i) {
      for (std::size_t j = 0; j < val_data.n_dof; ++j) {
        Tau_val(static_cast<Eigen::Index>(i * val_data.n_dof + j)) = val_data.tau(i, j);
      }
    }

    std::vector<std::string> algorithms;
    if (config.algorithm == 0) {
      algorithms = {"OLS", "WLS", "IRLS", "TLS", "EKF", "ML", "CLOE"};
    } else {
      algorithms.push_back(algorithmName(config.algorithm));
    }

    std::vector<BenchmarkResult> results;
    for (const auto &algorithm : algorithms) {
      BenchmarkResult result;
      result.name = algorithm;
      try {
        result.beta = identifier.solve(train_data, algorithm, flags);
        const Eigen::VectorXd Tau_pred = W_val * result.beta;
        const Eigen::VectorXd residual = Tau_val - Tau_pred;
        result.torque_rmse = std::sqrt(residual.squaredNorm() /
                                       static_cast<double>(residual.size()));
        result.torque_max_error = residual.cwiseAbs().maxCoeff();
        result.success = true;

        std::cout << algorithm << " => RMSE: " << result.torque_rmse
                  << " Nm, Max Error: " << result.torque_max_error << " Nm"
                  << std::endl;
      } catch (const std::exception &e) {
        std::cerr << algorithm << " 求解失败: " << e.what() << std::endl;
      }
      results.push_back(result);
    }

    if (config.algorithm == 0) {
      std::sort(results.begin(), results.end(),
                [](const BenchmarkResult &lhs, const BenchmarkResult &rhs) {
                  if (!lhs.success) {
                    return false;
                  }
                  if (!rhs.success) {
                    return true;
                  }
                  return lhs.torque_rmse < rhs.torque_rmse;
                });
    }

    if (!config.output_file.empty()) {
      saveResults(config.output_file, config.algorithm, results);
      std::cout << "结果已保存到: " << config.output_file << std::endl;
    }

    return 0;
  } catch (const std::exception &e) {
    std::cerr << "identify 失败: " << e.what() << std::endl;
    return 1;
  }
}
