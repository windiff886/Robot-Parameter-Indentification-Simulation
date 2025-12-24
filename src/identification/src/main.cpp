#include "identification/data_loader.hpp"
#include "identification/identification.hpp"
#include "rclcpp/rclcpp.hpp"
#include "robot/franka_panda.hpp"
#include <chrono>
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

    // 4. Solve
    bool include_friction = true;
    RCLCPP_INFO(node->get_logger(), "Solving identification using %s...",
                algorithm.c_str());

    Eigen::VectorXd beta = identifier.solve(data, algorithm, include_friction);

    // 5. Output
    std::stringstream ss;
    ss << beta.transpose();
    RCLCPP_INFO(node->get_logger(), "Identified Parameters (beta):");
    RCLCPP_INFO_STREAM(node->get_logger(), "\n" << ss.str());

    // 6. Save to YAML if requested
    std::string output_file;
    try {
      output_file = node->get_parameter("output_file").as_string();
    } catch (...) {
      // parameter might not be declared if not passed, verify declare_parameter
    }

    if (!output_file.empty()) {
      RCLCPP_INFO(node->get_logger(), "Saving results to: %s",
                  output_file.c_str());
      std::ofstream out(output_file);
      if (out.is_open()) {
        auto now = std::chrono::system_clock::now();
        std::time_t now_c = std::chrono::system_clock::to_time_t(now);

        out << "calibration_date: \""
            << std::put_time(std::localtime(&now_c), "%Y-%m-%d %H:%M:%S")
            << "\"\n";
        out << "algorithm: \"" << algorithm << "\"\n";
        out << "parameters:\n";
        for (int i = 0; i < beta.size(); ++i) {
          out << "  - " << beta(i) << "\n";
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
