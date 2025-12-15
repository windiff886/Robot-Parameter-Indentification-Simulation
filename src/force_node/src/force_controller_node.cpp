/**
 * @file force_controller_node.cpp
 * @brief 力矩控制器节点实现
 *
 * 该节点订阅关节状态，根据 PD 控制律计算力矩，并发布力矩指令给仿真节点。
 * 所有参数从配置文件 config/force_controller_node.yaml 读取。
 */

#include "force_node/force_controller_node.hpp"

#include <rclcpp_components/register_node_macro.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <chrono>
#include <cctype>
#include <filesystem>
#include <fstream>
#include <functional>
#include <sstream>

namespace fs = std::filesystem;

namespace
{

// ========== 简易 YAML 解析工具 ==========

std::string trim(const std::string & s)
{
  const auto first = s.find_first_not_of(" \t\r\n");
  if (first == std::string::npos) {
    return "";
  }
  const auto last = s.find_last_not_of(" \t\r\n");
  return s.substr(first, last - first + 1);
}

std::string remove_comment(const std::string & s)
{
  const auto pos = s.find('#');
  return pos == std::string::npos ? s : s.substr(0, pos);
}

bool parse_double(const std::string & line, const std::string & key, double & out)
{
  const std::string trimmed = trim(line);
  if (trimmed.rfind(key + ":", 0) != 0) {
    return false;
  }
  const std::string value = trim(remove_comment(trimmed.substr(key.size() + 1)));
  try {
    out = std::stod(value);
    return true;
  } catch (...) {
    return false;
  }
}

bool parse_double_array(const std::string & line, const std::string & key, std::vector<double> & out)
{
  const std::string trimmed = trim(line);
  if (trimmed.rfind(key + ":", 0) != 0) {
    return false;
  }
  std::string value = trim(remove_comment(trimmed.substr(key.size() + 1)));

  // 移除方括号
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

struct ForceControllerConfig
{
  double control_rate_hz = 1000.0;
  std::vector<double> kp = {600.0, 600.0, 600.0, 600.0, 250.0, 150.0, 50.0};
  std::vector<double> kd = {50.0, 50.0, 50.0, 50.0, 30.0, 25.0, 15.0};
  std::vector<double> target_position = {0.0, 0.0, 0.0, -1.57079, 0.0, 1.57079, -0.7853};
};

ForceControllerConfig load_config(const fs::path & config_path)
{
  ForceControllerConfig cfg;

  std::ifstream file(config_path);
  if (!file) {
    return cfg;  // 返回默认值
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

}  // namespace

namespace force_node
{

ForceControllerNode::ForceControllerNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("force_controller_node", options)
{
  // ========== 1. 从配置文件加载参数 ==========
  fs::path config_path;
  try {
    const auto share_dir = ament_index_cpp::get_package_share_directory("force_node");
    config_path = fs::path(share_dir) / "config" / "force_controller_node.yaml";
  } catch (const std::exception &) {
    config_path = fs::path("config") / "force_controller_node.yaml";
  }

  const auto cfg = load_config(config_path);

  // 验证参数维度
  if (cfg.kp.size() != NUM_ARM_JOINTS || cfg.kd.size() != NUM_ARM_JOINTS ||
    cfg.target_position.size() != NUM_ARM_JOINTS)
  {
    throw std::runtime_error("kp, kd, and target_position must have exactly 7 elements");
  }

  kp_ = cfg.kp;
  kd_ = cfg.kd;
  target_position_ = cfg.target_position;

  // 初始化状态向量
  current_position_.resize(NUM_TOTAL_JOINTS, 0.0);
  current_velocity_.resize(NUM_TOTAL_JOINTS, 0.0);

  RCLCPP_INFO(
    get_logger(), "Loaded config from %s", fs::absolute(config_path).string().c_str());
  RCLCPP_INFO(
    get_logger(), "  control_rate_hz: %.1f", cfg.control_rate_hz);

  // ========== 2. 创建订阅者和发布者 ==========
  joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
    "panda/joint_states", 10,
    std::bind(&ForceControllerNode::joint_state_callback, this, std::placeholders::_1));

  torque_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
    "panda/joint_torques", 10);

  // ========== 3. 创建控制定时器 ==========
  const auto period = std::chrono::duration<double>(1.0 / cfg.control_rate_hz);
  control_timer_ = this->create_wall_timer(period, std::bind(&ForceControllerNode::control_loop, this));

  RCLCPP_INFO(
    get_logger(),
    "Force controller started at %.1f Hz. Waiting for joint states...",
    cfg.control_rate_hz);
}

void ForceControllerNode::joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
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

void ForceControllerNode::control_loop()
{
  if (!state_received_) {
    return;
  }

  std::vector<double> q, dq;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    q = current_position_;
    dq = current_velocity_;
  }

  auto torques = compute_torque(q, dq);

  auto msg = std_msgs::msg::Float64MultiArray();
  msg.data = std::move(torques);
  torque_pub_->publish(std::move(msg));
}

std::vector<double> ForceControllerNode::compute_torque(
  const std::vector<double> & q,
  const std::vector<double> & dq)
{
  // 力矩向量：7 个关节 + 1 个夹爪（tendon actuator）
  std::vector<double> torques(8, 0.0);

  // ========== PD 位置控制器 ==========
  // τ = Kp * (q_target - q) - Kd * dq
  for (std::size_t i = 0; i < NUM_ARM_JOINTS; ++i) {
    const double position_error = target_position_[i] - q[i];
    const double velocity = dq[i];
    torques[i] = kp_[i] * position_error - kd_[i] * velocity;
  }

  // 夹爪保持当前状态（力矩为 0）
  torques[7] = 0.0;

  return torques;
}

}  // namespace force_node

RCLCPP_COMPONENTS_REGISTER_NODE(force_node::ForceControllerNode)
