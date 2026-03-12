#include "identification/data_loader.hpp"
#include "identification/identification.hpp"
#include "identification/algorithms.hpp"
#include "mujoco_panda_dynamics.hpp"
#include "mujoco_piper_dynamics.hpp"

#include <Eigen/Core>

#include <algorithm>
#include <cctype>
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
  std::string robot = "panda";
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

struct PreparedTrainingData {
  Eigen::MatrixXd qd_valid;
  Eigen::VectorXd tau_valid;
  Eigen::MatrixXd w_armature;
  Eigen::MatrixXd w_all;
  std::size_t filtered_outliers = 0;
};

struct PreparedEvaluationData {
  Eigen::MatrixXd qd_full;
  Eigen::VectorXd tau_full;
  Eigen::MatrixXd w_armature;
  Eigen::MatrixXd w_all;
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
    parseString(line, "robot", config.robot);
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
  case 8:
    return "NLS_FRICTION";
  default:
    return "OLS";
  }
}

std::string normalizeRobotType(std::string robot) {
  std::transform(robot.begin(), robot.end(), robot.begin(),
                 [](unsigned char ch) {
                   return static_cast<char>(std::tolower(ch));
                 });
  return robot;
}

std::size_t robotDof(const std::string &robot) {
  if (robot == "piper") {
    return 6;
  }
  return 7;
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

Eigen::VectorXd flattenTorque(const ExperimentData &data) {
  Eigen::VectorXd tau(data.n_samples * data.n_dof);
  for (std::size_t i = 0; i < data.n_samples; ++i) {
    for (std::size_t j = 0; j < data.n_dof; ++j) {
      tau(static_cast<Eigen::Index>(i * data.n_dof + j)) = data.tau(i, j);
    }
  }
  return tau;
}

PreparedTrainingData prepareTrainingData(
    const Identification &identifier, const ExperimentData &data,
    const double qdd_threshold,
    const mujoco_dynamics::MuJoCoParamFlags all_flags,
    const mujoco_dynamics::MuJoCoParamFlags armature_flags) {
  std::vector<std::size_t> valid_indices;
  valid_indices.reserve(data.n_samples);

  for (std::size_t i = 0; i < data.n_samples; ++i) {
    if (data.qdd.row(static_cast<Eigen::Index>(i)).cwiseAbs().maxCoeff() <
        qdd_threshold) {
      valid_indices.push_back(i);
    }
  }

  if (valid_indices.empty()) {
    throw std::runtime_error("No valid samples remaining after filtering outliers!");
  }

  PreparedTrainingData prepared;
  prepared.filtered_outliers = data.n_samples - valid_indices.size();

  Eigen::MatrixXd q_valid(valid_indices.size(), data.n_dof);
  Eigen::MatrixXd qd_valid(valid_indices.size(), data.n_dof);
  Eigen::MatrixXd qdd_valid(valid_indices.size(), data.n_dof);
  prepared.tau_valid.resize(static_cast<Eigen::Index>(valid_indices.size() * data.n_dof));

  for (std::size_t k = 0; k < valid_indices.size(); ++k) {
    const std::size_t idx = valid_indices[k];
    q_valid.row(static_cast<Eigen::Index>(k)) =
        data.q.row(static_cast<Eigen::Index>(idx));
    qd_valid.row(static_cast<Eigen::Index>(k)) =
        data.qd.row(static_cast<Eigen::Index>(idx));
    qdd_valid.row(static_cast<Eigen::Index>(k)) =
        data.qdd.row(static_cast<Eigen::Index>(idx));
    for (std::size_t j = 0; j < data.n_dof; ++j) {
      prepared.tau_valid(
          static_cast<Eigen::Index>(k * data.n_dof + j)) =
          data.tau(static_cast<Eigen::Index>(idx), static_cast<Eigen::Index>(j));
    }
  }

  prepared.qd_valid = std::move(qd_valid);
  prepared.w_all = identifier.computeObservationMatrix(
      q_valid.transpose(), prepared.qd_valid.transpose(), qdd_valid.transpose(),
      all_flags);
  prepared.w_armature = identifier.computeObservationMatrix(
      q_valid.transpose(), prepared.qd_valid.transpose(), qdd_valid.transpose(),
      armature_flags);
  return prepared;
}

PreparedEvaluationData prepareEvaluationData(
    const Identification &identifier, const ExperimentData &data,
    const mujoco_dynamics::MuJoCoParamFlags all_flags,
    const mujoco_dynamics::MuJoCoParamFlags armature_flags) {
  PreparedEvaluationData prepared;
  prepared.qd_full = data.qd;
  prepared.tau_full = flattenTorque(data);
  prepared.w_all = identifier.computeObservationMatrix(
      data.q.transpose(), data.qd.transpose(), data.qdd.transpose(), all_flags);
  prepared.w_armature =
      identifier.computeObservationMatrix(data.q.transpose(), data.qd.transpose(),
                                          data.qdd.transpose(), armature_flags);
  return prepared;
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
      } else if (flag == "--robot") {
        config.robot = value;
      }
    }

    config.robot = normalizeRobotType(config.robot);
    if (config.robot != "panda" && config.robot != "piper") {
      throw std::runtime_error("robot 仅支持 panda 或 piper");
    }

    if (config.data_file.empty()) {
      throw std::runtime_error("缺少 data_file");
    }

    std::cout << "加载数据: " << config.data_file << std::endl;
    std::cout << "机械臂配置: " << config.robot << std::endl;
    ExperimentData data = DataLoader::loadCSV(config.data_file, robotDof(config.robot));
    std::cout << "样本数: " << data.n_samples << std::endl;

    Identification identifier(config.robot);
    identifier.preprocess(data);

    if (config.robot == "piper") {
      mujoco_dynamics::MuJoCoPiperDynamics dynamics;
      double sum_sq_error = 0.0;
      double max_error = 0.0;
      const std::size_t n_dof = data.n_dof;
      for (std::size_t i = 0; i < data.n_samples; ++i) {
        const Eigen::VectorXd q =
            data.q.row(static_cast<Eigen::Index>(i)).transpose();
        const Eigen::VectorXd qd =
            data.qd.row(static_cast<Eigen::Index>(i)).transpose();
        const Eigen::VectorXd qdd =
            data.qdd.row(static_cast<Eigen::Index>(i)).transpose();
        const Eigen::VectorXd tau = dynamics.computeInverseDynamics(q, qd, qdd);
        for (std::size_t j = 0; j < n_dof; ++j) {
          const double error =
              tau(static_cast<Eigen::Index>(j)) -
              data.tau(static_cast<Eigen::Index>(i), static_cast<Eigen::Index>(j));
          sum_sq_error += error * error;
          max_error = std::max(max_error, std::abs(error));
        }
      }
      const double rmse = std::sqrt(
          sum_sq_error /
          (static_cast<double>(data.n_samples) * static_cast<double>(n_dof)));
      std::cout << "MuJoCo 动力学基准 RMSE: " << rmse << " Nm, Max Error: "
                << max_error << " Nm" << std::endl;
    } else {
      mujoco_dynamics::MuJoCoPandaDynamics dynamics;
      double sum_sq_error = 0.0;
      double max_error = 0.0;
      const std::size_t n_dof = data.n_dof;
      for (std::size_t i = 0; i < data.n_samples; ++i) {
        const Eigen::VectorXd q =
            data.q.row(static_cast<Eigen::Index>(i)).transpose();
        const Eigen::VectorXd qd =
            data.qd.row(static_cast<Eigen::Index>(i)).transpose();
        const Eigen::VectorXd qdd =
            data.qdd.row(static_cast<Eigen::Index>(i)).transpose();
        const Eigen::VectorXd tau = dynamics.computeInverseDynamics(q, qd, qdd);
        for (std::size_t j = 0; j < n_dof; ++j) {
          const double error =
              tau(static_cast<Eigen::Index>(j)) -
              data.tau(static_cast<Eigen::Index>(i), static_cast<Eigen::Index>(j));
          sum_sq_error += error * error;
          max_error = std::max(max_error, std::abs(error));
        }
      }
      const double rmse = std::sqrt(
          sum_sq_error /
          (static_cast<double>(data.n_samples) * static_cast<double>(n_dof)));
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

    const auto all_flags = mujoco_dynamics::MuJoCoParamFlags::ARMATURE |
                           mujoco_dynamics::MuJoCoParamFlags::DAMPING;
    const auto armature_flags = mujoco_dynamics::MuJoCoParamFlags::ARMATURE;
    const double qdd_threshold = 10.0;

    std::cout << "Preparing cached observation matrices..." << std::endl;
    PreparedTrainingData prepared_train = prepareTrainingData(
        identifier, train_data, qdd_threshold, all_flags, armature_flags);
    PreparedEvaluationData prepared_val =
        prepareEvaluationData(identifier, val_data, all_flags, armature_flags);

    std::cout << "Filtered " << prepared_train.filtered_outliers
              << " outlier samples. "
              << (prepared_train.tau_valid.size() /
                  static_cast<Eigen::Index>(train_data.n_dof))
              << " valid samples remain." << std::endl;
    std::cout << "Cached W(all) size: " << prepared_train.w_all.rows() << " x "
              << prepared_train.w_all.cols() << std::endl;
    std::cout << "Cached W(armature) size: " << prepared_train.w_armature.rows()
              << " x " << prepared_train.w_armature.cols() << std::endl;

    std::vector<std::string> algorithms;
    if (config.algorithm == 0) {
      algorithms = {"OLS", "WLS", "IRLS", "TLS", "EKF", "ML", "CLOE",
                    "NLS_FRICTION"};
    } else {
      algorithms.push_back(algorithmName(config.algorithm));
    }

    std::vector<BenchmarkResult> results;
    for (const auto &algorithm : algorithms) {
      BenchmarkResult result;
      result.name = algorithm;
      try {
        const bool use_nonlinear_friction = algorithm == "NLS_FRICTION";
        const auto &w_train =
            use_nonlinear_friction ? prepared_train.w_armature
                                   : prepared_train.w_all;
        const auto &w_val =
            use_nonlinear_friction ? prepared_val.w_armature : prepared_val.w_all;

        auto solver = identification::createAlgorithm(
            algorithm, static_cast<int>(train_data.n_dof));
        if (!solver) {
          std::cout << "WARNING: " << algorithm
                    << " algorithm is not fully supported with MuJoCoRegressor. "
                    << "Falling back to OLS." << std::endl;
          solver = identification::createAlgorithm(
              "OLS", static_cast<int>(train_data.n_dof));
        }

        if (auto *nonlinear_solver =
                dynamic_cast<identification::NonlinearFrictionLM *>(solver.get())) {
          nonlinear_solver->setVelocityData(prepared_train.qd_valid);
        }

        std::cout << "Solving using " << algorithm << "..." << std::endl;
        result.beta = solver->solve(w_train, prepared_train.tau_valid);

        Eigen::VectorXd Tau_pred;
        if (use_nonlinear_friction) {
          Tau_pred = identification::NonlinearFrictionLM::predictTorques(
              w_val, prepared_val.qd_full, result.beta,
              static_cast<int>(val_data.n_dof));
        } else {
          Tau_pred = w_val * result.beta;
        }

        const Eigen::VectorXd residual = prepared_val.tau_full - Tau_pred;
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
