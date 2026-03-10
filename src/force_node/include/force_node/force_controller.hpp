#ifndef FORCE_NODE_FORCE_CONTROLLER_HPP_
#define FORCE_NODE_FORCE_CONTROLLER_HPP_

#include "robot/franka_panda.hpp"
#include "robot/mujoco_collision_checker.hpp"
#include "trajectory/fourier_trajectory.hpp"

#include <array>
#include <cstddef>
#include <filesystem>
#include <memory>
#include <string>
#include <vector>

namespace force_node {

enum class ControllerMode {
  HOLD_POSITION,
  EXCITATION_TRAJECTORY
};

struct ForceControllerConfig {
  double control_rate_hz = 1000.0;
  double trajectory_duration = 30.0;
  std::vector<double> kp = {450.0, 450.0, 450.0, 450.0, 180.0, 100.0, 40.0};
  std::vector<double> kd = {35.0, 35.0, 35.0, 35.0, 20.0, 18.0, 10.0};
  std::vector<double> target_position = {0.0, 0.0, 0.0, -1.57079,
                                         0.0, 1.57079, -0.7853};
};

struct JointSample {
  std::vector<double> position;
  std::vector<double> velocity;
  std::vector<double> effort;
};

class ForceController {
public:
  explicit ForceController(const ForceControllerConfig &config,
                           const std::filesystem::path &collision_model_path);

  static ForceControllerConfig
  loadConfig(const std::filesystem::path &config_path);

  std::vector<double> computeTorque(const JointSample &sample,
                                    double current_time);

  double controlRateHz() const { return config_.control_rate_hz; }
  bool isTrajectoryFinished() const { return trajectory_finished_; }
  bool isTorqueSaturated() const { return torque_saturated_; }
  double trajectoryDuration() const { return trajectory_duration_; }

private:
  bool checkTrajectory(const trajectory::FourierTrajectory &traj);
  void initExcitationTrajectory();

  ForceControllerConfig config_;
  std::unique_ptr<robot::RobotModel> robot_model_;
  robot::MujocoCollisionChecker collision_checker_;
  std::unique_ptr<trajectory::FourierTrajectory> trajectory_;
  ControllerMode mode_{ControllerMode::EXCITATION_TRAJECTORY};

  double trajectory_start_time_{0.0};
  double trajectory_duration_{30.0};
  double last_print_time_{0.0};
  bool trajectory_started_{false};
  bool trajectory_finished_{false};
  bool torque_saturated_{false};
  std::size_t total_samples_{0};
  std::size_t saturated_samples_{0};

  static constexpr std::size_t NUM_ARM_JOINTS = 7;
  static constexpr std::size_t NUM_FINGER_JOINTS = 2;
  static constexpr std::size_t NUM_TOTAL_JOINTS =
      NUM_ARM_JOINTS + NUM_FINGER_JOINTS;
  static constexpr double SAFETY_PLANE_Z = 0.15;
  static constexpr double GRIPPER_LENGTH = 0.20;
  static constexpr std::array<double, NUM_ARM_JOINTS> MAX_TORQUES = {
      87.0, 87.0, 87.0, 87.0, 12.0, 12.0, 12.0};
};

} // namespace force_node

#endif
