/**
 * @file force_controller_node.cpp
 * @brief 力矩控制器节点实现 - 支持激励轨迹执行
 */

#include "force_node/force_controller_node.hpp"
#include "robot/robot_kinematics.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <cctype>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iomanip>
#include <sstream>

namespace fs = std::filesystem;

namespace {

// ========== 简易 YAML 解析工具 ==========

std::string trim(const std::string &s) {
  const auto first = s.find_first_not_of(" \t\r\n");
  if (first == std::string::npos) {
    return "";
  }
  const auto last = s.find_last_not_of(" \t\r\n");
  return s.substr(first, last - first + 1);
}

std::string remove_comment(const std::string &s) {
  const auto pos = s.find('#');
  return pos == std::string::npos ? s : s.substr(0, pos);
}

bool parse_double(const std::string &line, const std::string &key,
                  double &out) {
  const std::string trimmed = trim(line);
  if (trimmed.rfind(key + ":", 0) != 0) {
    return false;
  }
  const std::string value =
      trim(remove_comment(trimmed.substr(key.size() + 1)));
  try {
    out = std::stod(value);
    return true;
  } catch (...) {
    return false;
  }
}

bool parse_double_array(const std::string &line, const std::string &key,
                        std::vector<double> &out) {
  const std::string trimmed = trim(line);
  if (trimmed.rfind(key + ":", 0) != 0) {
    return false;
  }
  std::string value = trim(remove_comment(trimmed.substr(key.size() + 1)));

  if (value.size() >= 2 && value.front() == '[' && value.back() == ']') {
    value = value.substr(1, value.size() - 2);
  }

  out.clear();
  std::stringstream ss(value);
  std::string token;
  while (std::getline(ss, token, ',')) {
    try {
      out.push_back(std::stod(trim(token)));
    } catch (...) {
      return false;
    }
  }
  return !out.empty();
}

struct ForceControllerConfig {
  double control_rate_hz = 1000.0;
  std::vector<double> kp = {300.0, 300.0, 300.0, 300.0, 125.0, 75.0, 25.0};
  std::vector<double> kd = {25.0, 25.0, 25.0, 25.0, 15.0, 12.5, 7.5};
  std::vector<double> target_position = {0.0, 0.0,     0.0,    -1.57079,
                                         0.0, 1.57079, -0.7853};
};

ForceControllerConfig load_config(const fs::path &config_path) {
  ForceControllerConfig cfg;

  std::ifstream file(config_path);
  if (!file) {
    return cfg;
  }

  std::string line;
  while (std::getline(file, line)) {
    parse_double(line, "control_rate_hz", cfg.control_rate_hz);
    parse_double_array(line, "kp", cfg.kp);
    parse_double_array(line, "kd", cfg.kd);
    parse_double_array(line, "target_position", cfg.target_position);
  }

  return cfg;
}

} // namespace

namespace force_node {

ForceControllerNode::ForceControllerNode(const rclcpp::NodeOptions &options)
    : rclcpp::Node("force_controller_node", options) {
  // ========== 1. 从配置文件加载参数 ==========
  fs::path config_path;
  try {
    const auto share_dir =
        ament_index_cpp::get_package_share_directory("force_node");
    config_path = fs::path(share_dir) / "config" / "force_controller_node.yaml";
  } catch (const std::exception &) {
    config_path = fs::path("config") / "force_controller_node.yaml";
  }

  const auto cfg = load_config(config_path);

  if (cfg.kp.size() != NUM_ARM_JOINTS || cfg.kd.size() != NUM_ARM_JOINTS ||
      cfg.target_position.size() != NUM_ARM_JOINTS) {
    throw std::runtime_error(
        "kp, kd, and target_position must have exactly 7 elements");
  }

  kp_ = cfg.kp;
  kd_ = cfg.kd;
  target_position_ = cfg.target_position;

  current_position_.resize(NUM_TOTAL_JOINTS, 0.0);
  current_velocity_.resize(NUM_TOTAL_JOINTS, 0.0);
  last_torques_.resize(NUM_ARM_JOINTS + 1, 0.0);

  RCLCPP_INFO(get_logger(), "Loaded config from %s",
              fs::absolute(config_path).string().c_str());
  RCLCPP_INFO(get_logger(), "  control_rate_hz: %.1f", cfg.control_rate_hz);

  // ========== 2. 创建机器人模型 ==========
  robot_model_ = robot::createFrankaPanda();
  RCLCPP_INFO(get_logger(), "Created robot model: %s (%zu DOF)",
              robot_model_->name().c_str(), robot_model_->numDOF());

  // Initialize MuJoCo Collision Checker FIRST
  try {
    auto pkg_share =
        ament_index_cpp::get_package_share_directory("sim_com_node");
    std::string model_path = pkg_share + "/franka_emika_panda/panda.xml";
    collision_checker_.initialize(model_path);
    RCLCPP_INFO(get_logger(), "MuJoCo collision checker initialized.");
  } catch (const std::exception &e) {
    RCLCPP_ERROR(get_logger(), "Failed to init MuJoCo collision checker: %s",
                 e.what());
    // Use throw to stop node initialization if critical safety component fails
    throw;
  }

  // ========== 3. 初始化激励轨迹 ==========
  init_excitation_trajectory();

  // ========== 4. 创建订阅者和发布者 ==========
  joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "panda/joint_states", 10,
      std::bind(&ForceControllerNode::joint_state_callback, this,
                std::placeholders::_1));

  torque_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
      "panda/joint_torques", 10);

  // ========== 5. 创建控制定时器 ==========
  // Timer (1000Hz)
  const auto period = std::chrono::duration<double>(1.0 / cfg.control_rate_hz);
  control_timer_ = this->create_wall_timer(
      period, std::bind(&ForceControllerNode::control_loop, this));

  RCLCPP_INFO(get_logger(), "Controller started in %s mode. Duration: %.1f s",
              (mode_ == ControllerMode::EXCITATION_TRAJECTORY
                   ? "EXCITATION_TRAJECTORY"
                   : "HOLD_POSITION"),
              trajectory_duration_);
}

void ForceControllerNode::init_excitation_trajectory() {
  // 配置轨迹参数
  const std::size_t n_dof = NUM_ARM_JOINTS;
  const std::size_t n_harmonics = 5;          // 5 个谐波
  const double period = trajectory_duration_; // 轨迹周期 = 轨迹时长

  // 初始位置 = home 位置
  Eigen::VectorXd q0(n_dof);
  for (std::size_t i = 0; i < n_dof; ++i) {
    q0(i) = target_position_[i];
  }

  // 创建 Fourier 轨迹
  trajectory_ = std::make_unique<trajectory::FourierTrajectory>(
      n_dof, n_harmonics, period, q0);

  const int max_attempts = 200;
  bool valid_trajectory = false;
  double amplitude = 0.05; // 初始幅度较小 (Reduced from 0.2)

  robot::RobotKinematics kinematics(*robot_model_);

  for (int attempt = 0; attempt < max_attempts; ++attempt) {
    trajectory_->setRandomCoefficients(amplitude);
    trajectory_->applyInitialConditionConstraints();

    if (check_trajectory(*trajectory_)) {
      valid_trajectory = true;
      RCLCPP_INFO(get_logger(),
                  "Found valid excitation trajectory after %d attempts "
                  "(amplitude: %.2f)",
                  attempt + 1, amplitude);
      break;
    } else {
      // 如果一直找不到，可以尝试减小幅度?
      // 这里策略暂定为：前20次尝试默认幅度，之后稍微减小
      if (attempt > 20 && amplitude > 0.05) {
        amplitude *= 0.9;
      }
    }
  }

  if (!valid_trajectory) {
    RCLCPP_WARN(get_logger(),
                "Could not find a valid trajectory after %d attempts! Using "
                "the last generated one (MAY BE UNSAFE).",
                max_attempts);
  } else {
    RCLCPP_INFO(get_logger(), "Trajectory safety check passed.");
  }

  RCLCPP_INFO(get_logger(), "Generated Fourier excitation trajectory:");
  RCLCPP_INFO(get_logger(), "  Harmonics: %zu, Period: %.1f s", n_harmonics,
              period);

  // 生成带时间戳的文件名
  auto now = std::chrono::system_clock::now();
  auto time_t_now = std::chrono::system_clock::to_time_t(now);
  std::tm tm_now{};
  localtime_r(&time_t_now, &tm_now);

  std::ostringstream filename_ss;
  filename_ss << "identification_data_"
              << std::put_time(&tm_now, "%Y-%m-%d_%H-%M-%S") << ".csv";

  // 创建 data 目录（如果不存在）
  // 从包的安装目录推导工作空间根目录: install/force_node/share/force_node ->
  // 工作空间根目录
  fs::path data_dir;
  try {
    const auto share_dir =
        ament_index_cpp::get_package_share_directory("force_node");
    // share_dir = <workspace>/install/force_node/share/force_node
    // 向上 4 级到达工作空间根目录
    data_dir = fs::path(share_dir)
                   .parent_path()
                   .parent_path()
                   .parent_path()
                   .parent_path() /
               "data";
  } catch (const std::exception &) {
    // 备用方案：使用当前工作目录
    data_dir = fs::current_path() / "data";
  }
  if (!fs::exists(data_dir)) {
    fs::create_directories(data_dir);
    RCLCPP_INFO(get_logger(), "Created data directory: %s",
                data_dir.string().c_str());
  }

  fs::path data_path = data_dir / filename_ss.str();

  // 启动数据记录
  start_data_recording(data_path.string());
}

bool ForceControllerNode::check_trajectory(
    const trajectory::FourierTrajectory &traj) {
  const double dt = 0.1; // 检查步长
  robot::RobotKinematics kinematics(*robot_model_);
  const auto &limits = robot_model_->jointLimits();

  for (double t = 0.0; t <= trajectory_duration_; t += dt) {
    auto point = traj.evaluate(t);
    Eigen::VectorXd q = point.q;

    // 1. 检查关节限位
    for (std::size_t i = 0; i < NUM_ARM_JOINTS; ++i) {
      if (q(i) < limits[i].q_min || q(i) > limits[i].q_max) {
        return false;
      }
    }

    // 2. MuJoCo Collision Check (High Fidelity)
    if (collision_checker_.checkCollision(
            std::vector<double>(q.data(), q.data() + q.size()))) {
      RCLCPP_WARN(get_logger(), "Collision detected at t=%.2f", t);
      collision_checker_.printCollisions();
      return false;
    }
  }
  return true;
}

void ForceControllerNode::start_data_recording(const std::string &filename) {
  data_file_.open(filename);
  if (data_file_.is_open()) {
    // 写入 CSV 头
    data_file_ << "time";
    for (std::size_t i = 0; i < NUM_ARM_JOINTS; ++i) {
      data_file_ << ",q" << i + 1;
    }
    for (std::size_t i = 0; i < NUM_ARM_JOINTS; ++i) {
      data_file_ << ",qd" << i + 1;
    }
    for (std::size_t i = 0; i < NUM_ARM_JOINTS; ++i) {
      data_file_ << ",tau" << i + 1;
    }
    data_file_ << "\n";
    recording_ = true;
    RCLCPP_INFO(get_logger(), "Recording data to: %s", filename.c_str());
  }
}

void ForceControllerNode::record_data_point(double t,
                                            const std::vector<double> &q,
                                            const std::vector<double> &qd,
                                            const std::vector<double> &tau) {
  if (!recording_ || !data_file_.is_open()) {
    return;
  }

  if (q.size() < NUM_ARM_JOINTS || qd.size() < NUM_ARM_JOINTS ||
      tau.size() < NUM_ARM_JOINTS) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                         "Skipping record: insufficient joint data.");
    return;
  }

  const auto &limits = robot_model_->jointLimits();
  if (limits.size() < NUM_ARM_JOINTS) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                         "Skipping record: joint limits unavailable.");
    return;
  }

  for (std::size_t i = 0; i < NUM_ARM_JOINTS; ++i) {
    const double tau_max = limits[i].tau_max;
    if (std::abs(tau[i]) > tau_max || q[i] < limits[i].q_min ||
        q[i] > limits[i].q_max) {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Skipping record: torque/position exceeds joint limits.");
      return;
    }
  }

  data_file_ << std::fixed << std::setprecision(6) << t;
  for (std::size_t i = 0; i < NUM_ARM_JOINTS; ++i) {
    data_file_ << "," << q[i];
  }
  for (std::size_t i = 0; i < NUM_ARM_JOINTS; ++i) {
    data_file_ << "," << qd[i];
  }
  for (std::size_t i = 0; i < NUM_ARM_JOINTS; ++i) {
    data_file_ << "," << tau[i];
  }
  data_file_ << "\n";
}

void ForceControllerNode::joint_state_callback(
    const sensor_msgs::msg::JointState::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(state_mutex_);

  if (msg->position.size() >= NUM_TOTAL_JOINTS) {
    for (std::size_t i = 0; i < NUM_TOTAL_JOINTS; ++i) {
      current_position_[i] = msg->position[i];
    }
  }

  if (msg->velocity.size() >= NUM_TOTAL_JOINTS) {
    for (std::size_t i = 0; i < NUM_TOTAL_JOINTS; ++i) {
      current_velocity_[i] = msg->velocity[i];
    }
  }

  state_received_ = true;
}

void ForceControllerNode::control_loop() {
  if (!state_received_) {
    return;
  }

  std::vector<double> q, dq;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    q = current_position_;
    dq = current_velocity_;
  }

  // 获取当前时间
  static auto start_time = this->now();
  const double current_time = (this->now() - start_time).seconds();

  // 启动轨迹
  if (!trajectory_started_ && mode_ == ControllerMode::EXCITATION_TRAJECTORY) {
    trajectory_start_time_ = current_time;
    trajectory_started_ = true;
    RCLCPP_INFO(get_logger(), "Starting excitation trajectory at t=%.2f",
                current_time);
  }

  // Debug before compute_torque
  auto torques = compute_torque(q, dq);

  // Check for NaN
  for (double val : torques) {
    if (std::isnan(val)) {
      RCLCPP_ERROR(get_logger(), "NaN detected in torques!");
    }
  }

  // 记录数据
  if (mode_ == ControllerMode::EXCITATION_TRAJECTORY && trajectory_started_ &&
      !trajectory_finished_) {
    const double t_traj = current_time - trajectory_start_time_;
    record_data_point(t_traj, q, dq, torques);
  }

  last_torques_ = torques;

  auto msg = std_msgs::msg::Float64MultiArray();
  msg.data = torques; // Copy instead of move
  // RCLCPP_INFO(get_logger(), "Publishing torque (size %zu)...",
  // msg.data.size());
  torque_pub_->publish(msg);
  // RCLCPP_INFO(get_logger(), "Torque published.");

  // RCLCPP_INFO(get_logger(), "Loop finished (Mock).");
}

std::vector<double>
ForceControllerNode::compute_torque(const std::vector<double> &q,
                                    const std::vector<double> &dq) {
  std::vector<double> torques(8, 0.0);

  if (mode_ == ControllerMode::HOLD_POSITION || trajectory_finished_) {
    // ========== 定点 PD 控制 ==========
    for (std::size_t i = 0; i < NUM_ARM_JOINTS; ++i) {
      const double position_error = target_position_[i] - q[i];
      torques[i] = kp_[i] * position_error - kd_[i] * dq[i];
    }
  } else {
    // ========== 激励轨迹跟踪 ==========
    static auto start_time = this->now();
    const double current_time = (this->now() - start_time).seconds();
    const double t_traj = current_time - trajectory_start_time_;

    // 检查轨迹是否完成
    if (t_traj >= trajectory_duration_) {
      if (!trajectory_finished_) {
        trajectory_finished_ = true;
        RCLCPP_INFO(
            get_logger(),
            "Excitation trajectory completed! Switching to HOLD_POSITION.");
        RCLCPP_INFO(get_logger(), "Data saved successfully.");

        if (data_file_.is_open()) {
          data_file_.close();
          recording_ = false;
        }
      }
      // 回到定点控制
      for (std::size_t i = 0; i < NUM_ARM_JOINTS; ++i) {
        const double position_error = target_position_[i] - q[i];
        torques[i] = kp_[i] * position_error - kd_[i] * dq[i];
      }
      return torques;
    }

    // 获取轨迹期望状态
    auto traj_point = trajectory_->evaluate(t_traj);

    // PD 轨迹跟踪
    for (std::size_t i = 0; i < NUM_ARM_JOINTS; ++i) {
      const double q_d = traj_point.q(i);
      const double qd_d = traj_point.qd(i);

      const double position_error = q_d - q[i];
      const double velocity_error = qd_d - dq[i];

      // τ = Kp * (q_d - q) + Kd * (qd_d - qd)
      torques[i] = kp_[i] * position_error + kd_[i] * velocity_error;
    }

    // 每 2 秒打印一次状态
    static double last_print_time = 0;
    if (t_traj - last_print_time >= 2.0) {
      RCLCPP_INFO(get_logger(), "Trajectory progress: %.1f / %.1f s (%.0f%%)",
                  t_traj, trajectory_duration_,
                  100.0 * t_traj / trajectory_duration_);
      last_print_time = t_traj;
    }
  }

  // Clamp torques to match MuJoCo's forcerange limits
  // Joints 1-4: ±87 Nm, Joints 5-7: ±12 Nm
  constexpr std::array<double, 7> max_torques = {87.0, 87.0, 87.0, 87.0,
                                                 12.0, 12.0, 12.0};
  for (std::size_t i = 0; i < NUM_ARM_JOINTS; ++i) {
    torques[i] = std::clamp(torques[i], -max_torques[i], max_torques[i]);
  }

  torques[7] = 0.0; // 夹爪
  return torques;
}

} // namespace force_node

RCLCPP_COMPONENTS_REGISTER_NODE(force_node::ForceControllerNode)
