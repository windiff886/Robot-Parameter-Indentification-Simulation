#include "identification/data_loader.hpp"
#include "identification/identification.hpp"
#include "mujoco_panda_dynamics.hpp"
#include "rclcpp/rclcpp.hpp"
#include "robot/franka_panda.hpp"
#include "robot/regressor.hpp"
#include "robot/robot_dynamics.hpp"
#include <Eigen/Core>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("identification_node");

  // Declare parameters
  node->declare_parameter("data_file", "");
  node->declare_parameter("output_file", ""); // Output YAML file path
  node->declare_parameter("algorithm", 1);    // Default to 1 (OLS)

  // Get parameters
  std::string filename;
  std::string algorithm;
  int algorithm_id;

  try {
    filename = node->get_parameter("data_file").as_string();
    algorithm_id = node->get_parameter("algorithm").as_int();

    // Map ID to string
    if (algorithm_id == 0) {
      algorithm = "OLS";
    } else {
      switch (algorithm_id) {
      case 1:
        algorithm = "OLS";
        break;
      case 2:
        algorithm = "WLS";
        break;
      case 3:
        algorithm = "IRLS";
        break;
      case 4:
        algorithm = "TLS";
        break;
      case 5:
        algorithm = "EKF";
        break;
      case 6:
        algorithm = "ML";
        break;
      case 7:
        algorithm = "CLOE";
        break;
      default:
        RCLCPP_WARN(node->get_logger(),
                    "Unknown algorithm ID: %d. Defaulting to OLS.",
                    algorithm_id);
        algorithm = "OLS";
        break;
      }
    }

  } catch (const rclcpp::exceptions::ParameterNotDeclaredException &e) {
    RCLCPP_ERROR(node->get_logger(), "Parameters not declared.");
    return 1;
  }

  if (filename.empty()) {
    RCLCPP_ERROR(node->get_logger(),
                 "Parameter 'data_file' is empty. Please provide path to CSV.");
    return 1;
  }

  try {
    // 1. Load Data
    RCLCPP_INFO(node->get_logger(), "Loading data from: %s", filename.c_str());
    DataLoader loader;
    ExperimentData data = loader.loadCSV(filename, 7); // Franka has 7 DOF
    RCLCPP_INFO(node->get_logger(), "Loaded %zu samples.", data.n_samples);

    // 2. Setup Identification
    RCLCPP_INFO(node->get_logger(), "Initializing robot model...");
    auto robot = robot::createFrankaPanda();
    Identification identifier(std::move(robot));

    // 3. Preprocess (Compute qdd)
    RCLCPP_INFO(node->get_logger(),
                "Preprocessing data (computing acceleration)...");
    identifier.preprocess(data);

    // ================================================================
    // MuJoCo 动力学验证 - 手动实现
    // ================================================================
    {
      std::cout << std::endl << std::string(70, '=') << std::endl;
      std::cout << "  MUJOCO DYNAMICS VERIFICATION" << std::endl;
      std::cout << std::string(70, '=') << std::endl;

      robot::RobotDynamics rob_dyn(*robot::createFrankaPanda());

      double sum_sq_error = 0.0;
      double max_error = 0.0;
      const std::size_t n_dof = 7;

      for (std::size_t i = 0; i < data.n_samples; ++i) {
        Eigen::VectorXd q_i = data.q.row(i).transpose();
        Eigen::VectorXd qd_i = data.qd.row(i).transpose();
        Eigen::VectorXd qdd_i = data.qdd.row(i).transpose();

        // Compute ID using RobotDynamics
        Eigen::MatrixXd M = rob_dyn.computeInertiaMatrix(q_i);
        Eigen::MatrixXd C = rob_dyn.computeCoriolisMatrix(q_i, qd_i);
        Eigen::VectorXd g_vec = rob_dyn.computeGravityVector(q_i);

        // Base Dynamics
        Eigen::VectorXd tau_mj = M * qdd_i + C * qd_i + g_vec;

        // Add Armature (0.1) and Damping (1.0) to match MuJoCo XML
        for (std::size_t k = 0; k < n_dof; ++k) {
          tau_mj(k) += 0.1 * qdd_i(k);
          tau_mj(k) += 1.0 * qd_i(k);
        }

        for (std::size_t j = 0; j < n_dof; ++j) {
          double err = tau_mj(j) - data.tau(i, j);
          sum_sq_error += err * err;
          max_error = std::max(max_error, std::abs(err));
        }
      }

      double rmse = std::sqrt(sum_sq_error / (data.n_samples * n_dof));

      std::cout << std::fixed << std::setprecision(6);
      std::cout << "  MuJoCo RMSE:      " << rmse << " Nm" << std::endl;
      std::cout << "  Max Error:        " << max_error << " Nm" << std::endl;

      if (rmse < 0.5) {
        std::cout << "  [OK] Data valid!" << std::endl;
      } else {
        std::cout << "  [WARNING] Large error detected!" << std::endl;
      }

      std::cout << std::string(70, '=') << std::endl;
    }

    std::vector<std::string> algorithms_to_run;
    if (algorithm_id == 0) {
      algorithms_to_run = {"OLS", "WLS", "IRLS", "TLS", "EKF", "ML", "CLOE"};
      RCLCPP_INFO(node->get_logger(),
                  "Mode 0 selected: Running BENCHMARK on all algorithms.");
    } else {
      algorithms_to_run.push_back(algorithm);
    }

    // 动力学参数标志：包含 armature 和 damping 以匹配 MuJoCo 模型
    // MuJoCo 默认: armature=0.1, damping=1.0
    auto param_flags = robot::DynamicsParamFlags::ARMATURE |
                       robot::DynamicsParamFlags::DAMPING;

    RCLCPP_INFO(node->get_logger(),
                "Dynamics model: ARMATURE=%s, DAMPING=%s, FRICTION=%s",
                robot::hasFlag(param_flags, robot::DynamicsParamFlags::ARMATURE)
                    ? "ON"
                    : "OFF",
                robot::hasFlag(param_flags, robot::DynamicsParamFlags::DAMPING)
                    ? "ON"
                    : "OFF",
                robot::hasFlag(param_flags, robot::DynamicsParamFlags::FRICTION)
                    ? "ON"
                    : "OFF");

    // ================================================================
    // Cross-Validation: Split data into training (80%) and validation (20%)
    // ================================================================
    const double train_ratio = 0.8;
    const std::size_t n_train =
        static_cast<std::size_t>(data.n_samples * train_ratio);
    const std::size_t n_val = data.n_samples - n_train;

    RCLCPP_INFO(
        node->get_logger(),
        "Cross-validation: %zu training samples, %zu validation samples",
        n_train, n_val);

    // Create training data subset
    ExperimentData train_data;
    train_data.n_samples = n_train;
    train_data.n_dof = data.n_dof;
    train_data.time =
        std::vector<double>(data.time.begin(), data.time.begin() + n_train);
    train_data.q = data.q.topRows(n_train);
    train_data.qd = data.qd.topRows(n_train);
    train_data.qdd = data.qdd.topRows(n_train);
    train_data.tau = data.tau.topRows(n_train);

    // Create validation data subset
    ExperimentData val_data;
    val_data.n_samples = n_val;
    val_data.n_dof = data.n_dof;
    val_data.time =
        std::vector<double>(data.time.begin() + n_train, data.time.end());
    val_data.q = data.q.bottomRows(n_val);
    val_data.qd = data.qd.bottomRows(n_val);
    val_data.qdd = data.qdd.bottomRows(n_val);
    val_data.tau = data.tau.bottomRows(n_val);

    // Build observation matrix for VALIDATION set (for evaluation)
    auto eval_robot = robot::createFrankaPanda();
    robot::Regressor eval_regressor(*eval_robot);
    Eigen::MatrixXd W_val = eval_regressor.computeObservationMatrix(
        val_data.q.transpose(), val_data.qd.transpose(),
        val_data.qdd.transpose(), param_flags);

    // ================================================================
    // 回归矩阵诊断：条件数和列范数分析
    // ================================================================
    {
      // 计算每列的范数
      Eigen::VectorXd col_norms(W_val.cols());
      for (int j = 0; j < W_val.cols(); ++j) {
        col_norms(j) = W_val.col(j).norm();
      }

      // 找出接近零的列（可能不可辨识的参数）
      const double norm_threshold = 1e-6; // 放宽阈值
      std::vector<int> zero_cols;
      for (int j = 0; j < W_val.cols(); ++j) {
        if (col_norms(j) < norm_threshold) {
          zero_cols.push_back(j);
        }
      }

      // 打印列范数统计
      std::cout << "Regressor column norm stats: min=" << col_norms.minCoeff()
                << ", max=" << col_norms.maxCoeff()
                << ", mean=" << col_norms.mean() << std::endl;

      if (!zero_cols.empty()) {
        std::cout << "WARNING: " << zero_cols.size()
                  << " parameters have near-zero regressor columns "
                     "(unidentifiable):\n";
        for (int idx : zero_cols) {
          int link = idx / 10;
          int param = idx % 10;
          const char *param_names[] = {"m",   "mx",  "my",  "mz",  "Ixx",
                                       "Ixy", "Ixz", "Iyy", "Iyz", "Izz"};
          if (idx < 70) {
            std::cout << "  Param " << idx << ": Link " << (link + 1) << " "
                      << param_names[param] << "\n";
          } else if (idx < 77) {
            std::cout << "  Param " << idx << ": armature[" << (idx - 70)
                      << "]\n";
          } else {
            std::cout << "  Param " << idx << ": damping[" << (idx - 77)
                      << "]\n";
          }
        }
      }

      // 计算条件数（跳过大矩阵以节省时间）
      if (W_val.rows() < 50000) {
        Eigen::JacobiSVD<Eigen::MatrixXd> svd(W_val);
        double cond = svd.singularValues()(0) /
                      svd.singularValues()(svd.singularValues().size() - 1);
        std::cout << "Regressor matrix condition number: " << std::scientific
                  << cond << std::fixed << std::endl;
        if (cond > 1e10) {
          std::cout << "WARNING: Matrix is ill-conditioned! Results may be "
                       "unstable.\n";
        }
      } else {
        std::cout
            << "Skipping condition number computation (matrix too large)\n";
      }
    }

    // Build measured torque vector for VALIDATION set (early, for theory check)
    Eigen::VectorXd Tau_meas_val(val_data.n_samples * val_data.n_dof);
    for (std::size_t i = 0; i < val_data.n_samples; ++i) {
      for (std::size_t j = 0; j < val_data.n_dof; ++j) {
        Tau_meas_val(i * val_data.n_dof + j) = val_data.tau(i, j);
      }
    }

    // NOTE: Tau_meas_val for validation evaluation

    struct BenchmarkResult {
      std::string name;
      Eigen::VectorXd beta;
      double torque_rmse = 0.0;      // Torque prediction RMSE (Nm)
      double torque_max_error = 0.0; // Max absolute torque error (Nm)
      bool success = false;
    };
    std::vector<BenchmarkResult> results;

    for (const auto &algo_name : algorithms_to_run) {
      RCLCPP_INFO(node->get_logger(),
                  "--------------------------------------------------");
      RCLCPP_INFO(node->get_logger(), "Solving using %s...", algo_name.c_str());

      BenchmarkResult res;
      res.name = algo_name;

      try {
        res.beta = identifier.solve(train_data, algo_name, param_flags);

        // Output short summary
        // Output summary stats
        RCLCPP_INFO(node->get_logger(), "Identified %s:", algo_name.c_str());
        RCLCPP_INFO(node->get_logger(), "  Beta Norm: %f", res.beta.norm());
        RCLCPP_INFO(node->get_logger(), "  Beta Max:  %f", res.beta.maxCoeff());
        RCLCPP_INFO(node->get_logger(), "  Beta Min:  %f", res.beta.minCoeff());

        std::stringstream ss;
        // Print first 20 params (spanning Link 1 and Link 2)
        int print_count = (res.beta.size() > 20) ? 20 : res.beta.size();
        for (int i = 0; i < print_count; ++i) {
          ss << res.beta(i) << " ";
        }
        RCLCPP_INFO(node->get_logger(), "  First %d params: [ %s... ]",
                    print_count, ss.str().c_str());

        // ============================================================
        // Cross-Validation Evaluation: Test on VALIDATION set
        // ============================================================
        // Compute predicted torque on validation set: tau_pred = W_val * beta
        Eigen::VectorXd Tau_pred = W_val * res.beta;

        // Compute residual on validation set
        Eigen::VectorXd residual = Tau_meas_val - Tau_pred;

        // Compute RMSE
        double mse = residual.squaredNorm() / residual.size();
        res.torque_rmse = std::sqrt(mse);

        // Compute max absolute error
        res.torque_max_error = residual.cwiseAbs().maxCoeff();

        res.success = true;

        // Print detailed results for single algorithm run
        if (algorithm_id != 0) {
          std::cout << std::endl;
          std::cout << std::string(70, '=') << std::endl;
          std::cout << "  TORQUE PREDICTION EVALUATION (" << algo_name << ")"
                    << std::endl;
          std::cout << std::string(70, '=') << std::endl;
          std::cout << std::fixed << std::setprecision(6);
          std::cout << "  Torque RMSE:        " << res.torque_rmse << " Nm"
                    << std::endl;
          std::cout << "  Max Torque Error:   " << res.torque_max_error << " Nm"
                    << std::endl;
          std::cout << "  Training Samples:   " << train_data.n_samples
                    << std::endl;
          std::cout << "  Validation Samples: " << val_data.n_samples
                    << std::endl;
          std::cout << "  Parameters:         " << res.beta.size() << std::endl;
          std::cout << std::string(70, '-') << std::endl;

          // Per-joint RMSE
          std::cout << "  Per-Joint Torque RMSE:" << std::endl;
          for (std::size_t j = 0; j < val_data.n_dof; ++j) {
            double joint_mse = 0.0;
            for (std::size_t i = 0; i < val_data.n_samples; ++i) {
              double r = residual(i * val_data.n_dof + j);
              joint_mse += r * r;
            }
            joint_mse /= val_data.n_samples;
            double joint_rmse = std::sqrt(joint_mse);
            std::cout << "    Joint " << (j + 1) << ": " << std::setw(10)
                      << joint_rmse << " Nm" << std::endl;
          }
          std::cout << std::string(70, '=') << std::endl;
        }

      } catch (const std::exception &e) {
        RCLCPP_ERROR(node->get_logger(), "Algorithm %s failed: %s",
                     algo_name.c_str(), e.what());
      }

      results.push_back(res);
    }

    // Print Benchmark Summary if multiple algorithms run
    if (algorithm_id == 0) {
      std::cout << std::endl;
      std::cout << std::string(70, '=') << std::endl;
      std::cout << "  BENCHMARK SUMMARY (Sorted by Torque RMSE)" << std::endl;
      std::cout << std::string(70, '=') << std::endl;
      std::cout << std::left << std::setw(15) << "Algorithm" << std::right
                << std::setw(20) << "Torque RMSE (Nm)" << std::setw(20)
                << "Max Error (Nm)" << std::endl;
      std::cout << std::string(70, '-') << std::endl;

      // Sort results by torque RMSE
      std::sort(results.begin(), results.end(),
                [](const BenchmarkResult &a, const BenchmarkResult &b) {
                  if (!a.success)
                    return false;
                  if (!b.success)
                    return true;
                  return a.torque_rmse < b.torque_rmse;
                });

      for (const auto &res : results) {
        if (res.success) {
          std::cout << std::left << std::setw(15) << res.name << std::right
                    << std::fixed << std::setprecision(6) << std::setw(20)
                    << res.torque_rmse << std::setw(20) << res.torque_max_error
                    << std::endl;
        } else {
          std::cout << std::left << std::setw(15) << res.name << std::right
                    << std::setw(40) << "FAILED / INVALID" << std::endl;
        }
      }
      std::cout << std::string(70, '=') << std::endl;
    }

    // 7. Save to YAML if requested
    std::string output_file;
    try {
      output_file = node->get_parameter("output_file").as_string();
    } catch (...) {
    }

    if (!output_file.empty() && !results.empty()) {
      RCLCPP_INFO(node->get_logger(), "Saving results to: %s",
                  output_file.c_str());

      // Create directories if they don't exist
      std::filesystem::path out_path(output_file);
      if (out_path.has_parent_path()) {
        try {
          std::filesystem::create_directories(out_path.parent_path());
        } catch (const std::exception &e) {
          RCLCPP_WARN(node->get_logger(), "Could not create directory: %s",
                      e.what());
        }
      }

      std::ofstream out(output_file);
      if (out.is_open()) {
        auto now = std::chrono::system_clock::now();
        std::time_t now_c = std::chrono::system_clock::to_time_t(now);

        out << "calibration_date: \""
            << std::put_time(std::localtime(&now_c), "%Y-%m-%d %H:%M:%S")
            << "\"\n";
        out << "evaluation_method: \"torque_residual_rmse\"\n";

        if (algorithm_id == 0) {
          out << "mode: \"BENCHMARK_ALL\"\n";
          out << "benchmark_results:\n";
          for (const auto &res : results) {
            out << "  - algorithm: \"" << res.name << "\"\n";
            if (res.success) {
              out << "    torque_rmse: " << res.torque_rmse << "\n";
              out << "    torque_max_error: " << res.torque_max_error << "\n";
              out << "    parameters:\n";
              for (int i = 0; i < res.beta.size(); ++i)
                out << "      - " << res.beta(i) << "\n";
            } else {
              out << "    status: \"FAILED\"\n";
            }
          }
        } else {
          // Single algorithm format
          const auto &res = results[0];
          out << "algorithm: \"" << res.name << "\"\n";
          if (res.success) {
            out << "evaluation_results:\n";
            out << "  torque_rmse: " << res.torque_rmse << "\n";
            out << "  torque_max_error: " << res.torque_max_error << "\n";
            out << "parameters:\n";
            for (int i = 0; i < res.beta.size(); ++i)
              out << "  - " << res.beta(i) << "\n";
          }
        }

        out.close();
        RCLCPP_INFO(node->get_logger(), "Successfully saved results.");
      } else {
        RCLCPP_ERROR(node->get_logger(), "Failed to open output file: %s",
                     output_file.c_str());
      }
    }
  } catch (const std::exception &e) {
    RCLCPP_ERROR(node->get_logger(), "Error: %s", e.what());
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
