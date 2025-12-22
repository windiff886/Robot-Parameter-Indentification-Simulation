#include "identification/data_loader.hpp"
#include "identification/identification.hpp"
#include "rclcpp/rclcpp.hpp"
#include "robot/franka_panda.hpp"
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("identification_node");

  // Declare parameters
  node->declare_parameter("data_file", "");
  node->declare_parameter("algorithm", "OLS"); // Default to OLS

  // Get parameters
  std::string filename;
  std::string algorithm;
  try {
    filename = node->get_parameter("data_file").as_string();
    algorithm = node->get_parameter("algorithm").as_string();
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

  } catch (const std::exception &e) {
    RCLCPP_ERROR(node->get_logger(), "Error: %s", e.what());
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}
