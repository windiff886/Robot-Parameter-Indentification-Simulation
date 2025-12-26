#include "identification/data_loader.hpp"
#include "identification/identification.hpp"
#include "rclcpp/rclcpp.hpp"
#include "robot/franka_panda.hpp"
#include "robot/regressor.hpp"
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
                  "Unknown algorithm ID: %d. Defaulting to OLS.", algorithm_id);
      algorithm = "OLS";
      break;
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

    // 4. Determine Algorithms to run
    std::vector<std::string> algorithms_to_run;
    if (algorithm_id == 0) {
      algorithms_to_run = {"OLS", "WLS", "IRLS", "TLS", "EKF", "ML", "CLOE"};
      RCLCPP_INFO(node->get_logger(),
                  "Mode 0 selected: Running BENCHMARK on all algorithms.");
    } else {
      algorithms_to_run.push_back(algorithm);
    }

    bool include_friction = true;

    // Prepare for Evaluation (Simulation Mode)
    // Create a fresh robot model for ground truth
    auto gt_robot = robot::createFrankaPanda();
    robot::Regressor gt_regressor(*gt_robot);
    Eigen::VectorXd beta_true =
        gt_regressor.computeParameterVector(include_friction);

    struct BenchmarkResult {
      std::string name;
      Eigen::VectorXd beta;
      double rms_error = 0.0;
      double norm_error = 0.0;
      bool success = false;
      Eigen::VectorXd error;
    };
    std::vector<BenchmarkResult> results;

    for (const auto &algo_name : algorithms_to_run) {
      RCLCPP_INFO(node->get_logger(),
                  "--------------------------------------------------");
      RCLCPP_INFO(node->get_logger(), "Solving using %s...", algo_name.c_str());

      BenchmarkResult res;
      res.name = algo_name;

      try {
        res.beta = identifier.solve(data, algo_name, include_friction);

        // Output short summary
        std::stringstream ss;
        ss << res.beta.head((res.beta.size() > 5) ? 5 : res.beta.size())
                  .transpose()
           << "...";
        RCLCPP_INFO(node->get_logger(), "Identified %s: [%s]",
                    algo_name.c_str(), ss.str().c_str());

        // Evaluation
        if (res.beta.size() == beta_true.size()) {
          res.success = true;
          res.error = res.beta - beta_true;
          double mse = res.error.squaredNorm() / res.error.size();
          res.rms_error = std::sqrt(mse);
          res.norm_error = res.error.norm();

          // Detailed Print for single run or just Summary later?
          // If single run (id != 0), print detailed table as before.
          if (algorithm_id != 0) {
            std::cout << std::endl;
            std::cout << std::string(90, '=') << std::endl;
            std::cout << "  SIMULATION MODE PARAMETER EVALUATION RESULTS ("
                      << algo_name << ")" << std::endl;
            std::cout << std::string(90, '=') << std::endl;
            std::cout << std::left << std::setw(25) << "Parameter" << std::right
                      << std::setw(15) << "True Value" << std::setw(15)
                      << "Estimated" << std::setw(15) << "Abs Error"
                      << std::setw(15) << "Rel Error (%)" << std::endl;
            std::cout << std::string(90, '-') << std::endl;

            std::vector<std::string> param_names = {"m",   "mx",  "my",  "mz",
                                                    "Ixx", "Ixy", "Ixz", "Iyy",
                                                    "Iyz", "Izz"};
            int n_links = 7;
            int params_per_link = 10;

            for (int i = 0; i < n_links; ++i) {
              // Print details logic (same as before)
              // For brevity in diff, assume standard printing...
              // Actually, to keep it functional I must copy the printing loop
              // or make a helper. I will paste the loop here.
              std::cout << "Link " << (i + 1) << ":" << std::endl;
              for (int j = 0; j < params_per_link; ++j) {
                int idx = i * params_per_link + j;
                double t_val = beta_true(idx);
                double e_val = res.beta(idx);
                double abs_err = std::abs(e_val - t_val);
                double rel_err = (std::abs(t_val) > 1e-9)
                                     ? (abs_err / std::abs(t_val)) * 100.0
                                     : 0.0;
                std::cout << std::left << std::setw(25)
                          << ("L" + std::to_string(i + 1) + "_" +
                              param_names[j])
                          << std::right << std::fixed << std::setprecision(5)
                          << std::setw(15) << t_val << std::setw(15) << e_val
                          << std::setw(15) << abs_err << std::setw(14)
                          << rel_err << "%" << std::endl;
              }
            }

            if (include_friction) {
              std::cout << "Friction:" << std::endl;
              for (int i = 0; i < n_links; ++i) {
                int base_idx = n_links * params_per_link;
                double t_fv = beta_true(base_idx + 2 * i);
                double e_fv = res.beta(base_idx + 2 * i);
                double err_fv = std::abs(e_fv - t_fv);
                double rel_fv = (std::abs(t_fv) > 1e-9)
                                    ? (err_fv / std::abs(t_fv)) * 100.0
                                    : 0.0;
                std::cout << std::left << std::setw(25)
                          << ("J" + std::to_string(i + 1) + "_Fv") << std::right
                          << std::fixed << std::setprecision(5) << std::setw(15)
                          << t_fv << std::setw(15) << e_fv << std::setw(15)
                          << err_fv << std::setw(14) << rel_fv << "%"
                          << std::endl;

                double t_fc = beta_true(base_idx + 2 * i + 1);
                double e_fc = res.beta(base_idx + 2 * i + 1);
                double err_fc = std::abs(e_fc - t_fc);
                double rel_fc = (std::abs(t_fc) > 1e-9)
                                    ? (err_fc / std::abs(t_fc)) * 100.0
                                    : 0.0;
                std::cout << std::left << std::setw(25)
                          << ("J" + std::to_string(i + 1) + "_Fc") << std::right
                          << std::fixed << std::setprecision(5) << std::setw(15)
                          << t_fc << std::setw(15) << e_fc << std::setw(15)
                          << err_fc << std::setw(14) << rel_fc << "%"
                          << std::endl;
              }
            }
            std::cout << std::string(90, '=') << std::endl;
          }

        } else {
          RCLCPP_WARN(node->get_logger(), "Size mismatch for %s",
                      algo_name.c_str());
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
      std::cout << std::string(60, '=') << std::endl;
      std::cout << "  BENCHMARK SUMMARY (Sorted by RMS Error)" << std::endl;
      std::cout << std::string(60, '=') << std::endl;
      std::cout << std::left << std::setw(15) << "Algorithm" << std::right
                << std::setw(20) << "RMS Error" << std::setw(20) << "Norm Error"
                << std::endl;
      std::cout << std::string(60, '-') << std::endl;

      // Sort results
      std::sort(results.begin(), results.end(),
                [](const BenchmarkResult &a, const BenchmarkResult &b) {
                  if (!a.success)
                    return false;
                  if (!b.success)
                    return true;
                  return a.rms_error < b.rms_error;
                });

      for (const auto &res : results) {
        if (res.success) {
          std::cout << std::left << std::setw(15) << res.name << std::right
                    << std::fixed << std::setprecision(6) << std::setw(20)
                    << res.rms_error << std::setw(20) << res.norm_error
                    << std::endl;
        } else {
          std::cout << std::left << std::setw(15) << res.name << std::right
                    << std::setw(40) << "FAILED / INVALID" << std::endl;
        }
      }
      std::cout << std::string(60, '=') << std::endl;
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

        if (algorithm_id == 0) {
          out << "mode: \"BENCHMARK_ALL\"\n";
          out << "benchmark_results:\n";
          for (const auto &res : results) {
            out << "  - algorithm: \"" << res.name << "\"\n";
            if (res.success) {
              out << "    rms_error: " << res.rms_error << "\n";
              out << "    norm_error: " << res.norm_error << "\n";
              out << "    parameters:\n";
              for (int i = 0; i < res.beta.size(); ++i)
                out << "      - " << res.beta(i) << "\n";
            } else {
              out << "    status: \"FAILED\"\n";
            }
          }
        } else {
          // Legacy/Single format
          const auto &res = results[0];
          out << "algorithm: \"" << res.name << "\"\n";
          if (res.success) {
            out << "parameters:\n";
            for (int i = 0; i < res.beta.size(); ++i)
              out << "  - " << res.beta(i) << "\n";

            out << "evaluation_results:\n";
            out << "  rms_error: " << res.rms_error << "\n";
            out << "  norm_error: " << res.norm_error << "\n";
            out << "  parameter_errors:\n";
            for (int i = 0; i < res.error.size(); ++i)
              out << "    - " << res.error(i) << "\n";
            out << "  ground_truth:\n";
            for (int i = 0; i < beta_true.size(); ++i)
              out << "    - " << beta_true(i) << "\n";
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
