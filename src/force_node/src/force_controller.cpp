#include "force_node/force_controller.hpp"

#include "robot/robot_kinematics.hpp"

#include <algorithm>
#include <cctype>
#include <fstream>
#include <iostream>
#include <sstream>
#include <stdexcept>

namespace fs = std::filesystem;

namespace {

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

bool parseDouble(const std::string &line, const std::string &key, double &out) {
  const std::string trimmed = trim(line);
  if (trimmed.rfind(key + ":", 0) != 0) {
    return false;
  }

  const std::string value =
      trim(removeComment(trimmed.substr(key.size() + 1)));
  try {
    out = std::stod(value);
    return true;
  } catch (const std::exception &) {
    return false;
  }
}

bool parseDoubleArray(const std::string &line, const std::string &key,
                      std::vector<double> &out) {
  const std::string trimmed = trim(line);
  if (trimmed.rfind(key + ":", 0) != 0) {
    return false;
  }

  std::string value = trim(removeComment(trimmed.substr(key.size() + 1)));
  if (value.size() >= 2 && value.front() == '[' && value.back() == ']') {
    value = value.substr(1, value.size() - 2);
  }

  std::vector<double> parsed;
  std::stringstream stream(value);
  std::string token;
  while (std::getline(stream, token, ',')) {
    parsed.push_back(std::stod(trim(token)));
  }

  if (parsed.empty()) {
    return false;
  }

  out = std::move(parsed);
  return true;
}

} // namespace

namespace force_node {

ForceControllerConfig
ForceController::loadConfig(const std::filesystem::path &config_path) {
  ForceControllerConfig config;
  std::ifstream file(config_path);
  if (!file) {
    return config;
  }

  std::string line;
  while (std::getline(file, line)) {
    parseDouble(line, "control_rate_hz", config.control_rate_hz);
    parseDouble(line, "trajectory_duration", config.trajectory_duration);
    parseDoubleArray(line, "kp", config.kp);
    parseDoubleArray(line, "kd", config.kd);
    parseDoubleArray(line, "target_position", config.target_position);
  }

  return config;
}

ForceController::ForceController(const ForceControllerConfig &config,
                                 const std::filesystem::path &collision_model)
    : config_(config), trajectory_duration_(config.trajectory_duration) {
  if (config_.kp.size() != NUM_ARM_JOINTS || config_.kd.size() != NUM_ARM_JOINTS ||
      config_.target_position.size() != NUM_ARM_JOINTS) {
    throw std::runtime_error("控制器配置必须包含 7 维 kp/kd/target_position");
  }

  robot_model_ = robot::createFrankaPanda();
  collision_checker_.initialize(collision_model.string());
  initExcitationTrajectory();
}

void ForceController::initExcitationTrajectory() {
  const std::size_t n_harmonics = 5;
  Eigen::VectorXd q0(NUM_ARM_JOINTS);
  for (std::size_t i = 0; i < NUM_ARM_JOINTS; ++i) {
    q0(static_cast<Eigen::Index>(i)) = config_.target_position[i];
  }

  trajectory_ = std::make_unique<trajectory::FourierTrajectory>(
      NUM_ARM_JOINTS, n_harmonics, trajectory_duration_, q0);

  double amplitude = 0.15;
  bool valid_trajectory = false;
  constexpr int max_attempts = 200;
  for (int attempt = 0; attempt < max_attempts; ++attempt) {
    trajectory_->setRandomCoefficients(amplitude);
    trajectory_->applyInitialConditionConstraints();
    if (checkTrajectory(*trajectory_)) {
      valid_trajectory = true;
      std::cout << "找到安全激励轨迹，尝试次数: " << (attempt + 1)
                << "，幅值: " << amplitude << std::endl;
      break;
    }
    if (attempt > 20 && amplitude > 0.01) {
      amplitude *= 0.95;
    }
  }

  if (!valid_trajectory) {
    std::cout << "警告: 未找到完全安全的轨迹，将使用最后一次生成结果"
              << std::endl;
  }
}

bool ForceController::checkTrajectory(const trajectory::FourierTrajectory &traj) {
  robot::RobotKinematics kinematics(*robot_model_);
  const auto &limits = robot_model_->jointLimits();

  for (double t = 0.0; t <= trajectory_duration_; t += 0.1) {
    const auto point = traj.evaluate(t);
    const Eigen::VectorXd &q = point.q;

    for (std::size_t i = 0; i < NUM_ARM_JOINTS; ++i) {
      if (q(static_cast<Eigen::Index>(i)) < limits[i].q_min ||
          q(static_cast<Eigen::Index>(i)) > limits[i].q_max) {
        return false;
      }
    }

    const auto transforms = kinematics.computeAllTransforms(q);
    for (const auto &transform : transforms) {
      if (transform(2, 3) < SAFETY_PLANE_Z) {
        return false;
      }
    }

    if (!transforms.empty()) {
      const auto &ee = transforms.back();
      const Eigen::Vector3d p_ee = ee.block<3, 1>(0, 3);
      const Eigen::Matrix3d r_ee = ee.block<3, 3>(0, 0);
      const Eigen::Vector3d p_tip =
          p_ee + r_ee * Eigen::Vector3d(0.0, 0.0, GRIPPER_LENGTH);
      if (p_tip.z() < SAFETY_PLANE_Z) {
        return false;
      }
    }

    std::vector<double> q_std(NUM_ARM_JOINTS);
    for (std::size_t i = 0; i < NUM_ARM_JOINTS; ++i) {
      q_std[i] = q(static_cast<Eigen::Index>(i));
    }
    if (collision_checker_.checkCollision(q_std)) {
      collision_checker_.printCollisions();
      return false;
    }
  }

  return true;
}

std::vector<double> ForceController::computeTorque(const JointSample &sample,
                                                   double current_time) {
  if (sample.position.size() < NUM_TOTAL_JOINTS ||
      sample.velocity.size() < NUM_TOTAL_JOINTS) {
    throw std::runtime_error("关节状态维度不足，至少需要 9 个关节状态");
  }

  std::vector<double> torques(NUM_ARM_JOINTS + 1, 0.0);

  if (!trajectory_started_ && mode_ == ControllerMode::EXCITATION_TRAJECTORY) {
    trajectory_start_time_ = current_time;
    trajectory_started_ = true;
    std::cout << "开始执行激励轨迹，t=" << current_time << " s" << std::endl;
  }

  if (mode_ == ControllerMode::HOLD_POSITION || trajectory_finished_) {
    for (std::size_t i = 0; i < NUM_ARM_JOINTS; ++i) {
      const double position_error = config_.target_position[i] - sample.position[i];
      torques[i] = config_.kp[i] * position_error - config_.kd[i] * sample.velocity[i];
    }
  } else {
    const double trajectory_time = current_time - trajectory_start_time_;
    if (trajectory_time >= trajectory_duration_) {
      trajectory_finished_ = true;
      std::cout << "激励轨迹执行结束，切换为定点保持模式" << std::endl;
      if (total_samples_ > 0) {
        const double ratio =
            100.0 * static_cast<double>(saturated_samples_) /
            static_cast<double>(total_samples_);
        std::cout << "力矩饱和占比: " << ratio << "%" << std::endl;
      }
      for (std::size_t i = 0; i < NUM_ARM_JOINTS; ++i) {
        const double position_error =
            config_.target_position[i] - sample.position[i];
        torques[i] =
            config_.kp[i] * position_error - config_.kd[i] * sample.velocity[i];
      }
    }

    const auto point = trajectory_->evaluate(trajectory_time);
    for (std::size_t i = 0; i < NUM_ARM_JOINTS; ++i) {
      const double q_error = point.q(static_cast<Eigen::Index>(i)) - sample.position[i];
      const double qd_error = point.qd(static_cast<Eigen::Index>(i)) - sample.velocity[i];
      torques[i] = config_.kp[i] * q_error + config_.kd[i] * qd_error;
    }

    if (trajectory_time - last_print_time_ >= 2.0) {
      std::cout << "轨迹进度: " << trajectory_time << " / "
                << trajectory_duration_ << " s" << std::endl;
      last_print_time_ = trajectory_time;
    }
  }

  torque_saturated_ = false;
  ++total_samples_;
  for (std::size_t i = 0; i < NUM_ARM_JOINTS; ++i) {
    torques[i] = std::clamp(torques[i], -MAX_TORQUES[i], MAX_TORQUES[i]);
    if (std::abs(torques[i]) >= MAX_TORQUES[i] * 0.99) {
      torque_saturated_ = true;
    }
  }
  if (torque_saturated_) {
    ++saturated_samples_;
  }

  torques.back() = 0.0;
  return torques;
}

} // namespace force_node
