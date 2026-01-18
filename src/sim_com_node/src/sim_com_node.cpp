/**
 * @file sim_com_node.cpp
 * @brief Panda 机械臂 MuJoCo 仿真节点实现
 *
 * 该节点使用 MuJoCo 物理引擎模拟 Franka Emika Panda 机械臂，
 * 并通过 ROS2 话题发布关节状态信息，订阅力矩指令，同时提供可视化窗口。
 */

#include "sim_com_node/panda_sim_node.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <cctype>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iomanip>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <utility>

#include <GLFW/glfw3.h>

namespace fs = std::filesystem;

namespace {

std::string trim_copy(const std::string &input) {
  const auto first = input.find_first_not_of(" \t\r\n");
  if (first == std::string::npos) {
    return "";
  }
  const auto last = input.find_last_not_of(" \t\r\n");
  return input.substr(first, last - first + 1);
}

std::optional<double> parse_yaml_double(const std::string &trimmed_line,
                                        const std::string &key) {
  if (trimmed_line.rfind(key, 0) != 0) {
    return std::nullopt;
  }

  std::size_t pos = key.size();
  while (pos < trimmed_line.size() &&
         std::isspace(static_cast<unsigned char>(trimmed_line[pos]))) {
    ++pos;
  }
  if (pos >= trimmed_line.size() || trimmed_line[pos] != ':') {
    return std::nullopt;
  }

  std::string value = trim_copy(trimmed_line.substr(pos + 1));
  const auto comment = value.find('#');
  if (comment != std::string::npos) {
    value = trim_copy(value.substr(0, comment));
  }

  if (value.size() >= 2 && ((value.front() == '"' && value.back() == '"') ||
                            (value.front() == '\'' && value.back() == '\''))) {
    value = value.substr(1, value.size() - 2);
  }

  try {
    return std::stod(value);
  } catch (const std::exception &) {
    return std::nullopt;
  }
}

std::optional<double> read_publish_rate_hz(const fs::path &config_path) {
  std::ifstream file(config_path);
  if (!file) {
    return std::nullopt;
  }

  std::string line;
  while (std::getline(file, line)) {
    const std::string trimmed = trim_copy(line);
    if (trimmed.empty() || trimmed[0] == '#') {
      continue;
    }

    if (auto v = parse_yaml_double(trimmed, "publish_rate_hz")) {
      return v;
    }
    if (auto v = parse_yaml_double(trimmed, "publish_rate")) {
      return v;
    }
  }

  return std::nullopt;
}

} // namespace

namespace sim_com_node {

/**
 * @brief 构造函数 - 初始化仿真节点
 * @param options ROS2 节点选项
 */
PandaSimNode::PandaSimNode(const rclcpp::NodeOptions &options)
    : rclcpp::Node("panda_sim_node", options) {
  // ========== 1. 配置文件与模型路径解析 ==========
  fs::path share_dir;
  try {
    // 尝试从 ROS2 包共享目录获取默认模型路径
    share_dir = ament_index_cpp::get_package_share_directory("sim_com_node");
  } catch (const std::exception &) {
    // ignore
  }

  const fs::path config_path =
      share_dir.empty() ? fs::path("config") / "panda_sim_node.yaml"
                        : share_dir / "config" / "panda_sim_node.yaml";

  const double default_publish_rate_hz = 1000.0;
  const auto publish_rate_from_config = read_publish_rate_hz(config_path);
  const double publish_rate_hz =
      publish_rate_from_config.value_or(default_publish_rate_hz);
  if (!publish_rate_from_config) {
    RCLCPP_WARN(
        get_logger(), "publish_rate_hz not found in %s; using default %.1f Hz",
        fs::absolute(config_path).string().c_str(), default_publish_rate_hz);
  } else {
    RCLCPP_INFO(get_logger(), "publish_rate_hz=%.1f Hz (config: %s)",
                publish_rate_hz, fs::absolute(config_path).string().c_str());
  }
  if (publish_rate_hz <= 0.0) {
    throw std::runtime_error("Invalid publish_rate_hz in config: " +
                             config_path.string());
  }

  const fs::path model_path =
      share_dir.empty() ? fs::path("franka_emika_panda") / "scene.xml"
                        : share_dir / "franka_emika_panda" / "scene.xml";

  // 验证模型文件存在
  const fs::path resolved_model = fs::absolute(model_path);
  if (!fs::exists(resolved_model)) {
    throw std::runtime_error("Model file not found: " +
                             resolved_model.string());
  }

  // ========== 2. MuJoCo 模型加载 ==========
  char error[512];
  model_.reset(
      mj_loadXML(resolved_model.c_str(), nullptr, error, sizeof(error)));
  if (!model_) {
    throw std::runtime_error("Failed to load MuJoCo model: " +
                             std::string(error));
  }

  // 分配仿真数据结构
  data_.reset(mj_makeData(model_.get()));
  if (!data_) {
    throw std::runtime_error("Failed to allocate MuJoCo data");
  }

  // 初始化力矩指令缓冲区
  torque_command_.resize(model_->nu, 0.0);

  // ========== 3. 初始位姿设置 ==========
  // 如果模型定义了 "home" 关键帧，则重置到该位姿
  const int home_id = mj_name2id(model_.get(), mjOBJ_KEY, "home");
  if (home_id >= 0) {
    mj_resetDataKeyframe(model_.get(), data_.get(), home_id);
  } else {
    mj_resetData(model_.get(), data_.get());
  }

  // ========== 4. 关节信息提取 ==========
  // 遍历所有关节，只保留 hinge（旋转）和 slide（滑动）类型的关节
  joint_indices_.reserve(model_->njnt);
  joint_names_.reserve(model_->njnt);
  for (int j = 0; j < model_->njnt; ++j) {
    const int type = model_->jnt_type[j];
    if (type == mjJNT_FREE || type == mjJNT_BALL) {
      // 跳过自由关节和球关节，只发布简单关节
      continue;
    }
    joint_indices_.push_back(j);
    joint_names_.emplace_back(model_->names + model_->name_jntadr[j]);
  }

  if (joint_indices_.empty()) {
    throw std::runtime_error(
        "No hinge/slide joints found in the model to publish");
  }

  // ========== 5. ROS2 通信设置 ==========
  // 创建关节状态发布器
  joint_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
      "panda/joint_states", 10);

  // 创建力矩指令订阅器
  torque_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
      "panda/joint_torques", 10,
      std::bind(&PandaSimNode::torque_callback, this, std::placeholders::_1));

  // 创建饱和状态订阅器
  saturation_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "panda/saturated", 10,
      std::bind(&PandaSimNode::saturation_callback, this,
                std::placeholders::_1));

  // 创建定时器，按指定频率执行仿真步进和状态发布
  const auto period = std::chrono::duration<double>(1.0 / publish_rate_hz);
  timer_ =
      this->create_wall_timer(period, std::bind(&PandaSimNode::step, this));

  RCLCPP_INFO(get_logger(),
              "Loaded model from %s, publishing %zu joints at %.1f Hz (torque "
              "control mode)",
              resolved_model.string().c_str(), joint_indices_.size(),
              publish_rate_hz);
  RCLCPP_INFO(
      get_logger(),
      "Subscribing to torque commands on 'panda/joint_torques' (%d actuators)",
      model_->nu);

  // ========== 6. 可视化窗口启动 ==========
  start_viewer();

  // ========== 7. 自动启动数据记录 (for identification) ==========
  // 生成带时间戳的文件名
  auto now = std::chrono::system_clock::now();
  auto time_t_now = std::chrono::system_clock::to_time_t(now);
  std::tm tm_now{};
  localtime_r(&time_t_now, &tm_now);

  std::ostringstream filename_ss;
  filename_ss << "benchmark_data_"
              << std::put_time(&tm_now, "%Y-%m-%d_%H-%M-%S") << ".csv";

  // 数据目录逻辑
  fs::path data_dir;
  if (!share_dir.empty()) {
    // install/sim_com_node/share/sim_com_node -> workspace/data
    data_dir =
        share_dir.parent_path().parent_path().parent_path().parent_path() /
        "data";
  } else {
    data_dir = fs::current_path() / "data";
  }

  if (!fs::exists(data_dir)) {
    fs::create_directories(data_dir);
  }

  start_data_recording((data_dir / filename_ss.str()).string());
}

/**
 * @brief 析构函数 - 清理资源
 */
PandaSimNode::~PandaSimNode() {
  stop_viewer();
  stop_data_recording();
}

/**
 * @brief 力矩指令回调 - 接收并存储力矩指令
 * @param msg 力矩指令消息（Float64MultiArray）
 *
 * 力矩向量应包含 8 个元素（7 个关节 + 1 个夹爪 tendon）
 */
void PandaSimNode::torque_callback(
    const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(torque_mutex_);

  const std::size_t expected_size = static_cast<std::size_t>(model_->nu);
  if (msg->data.size() != expected_size) {
    RCLCPP_WARN_THROTTLE(
        get_logger(), *this->get_clock(), 1000,
        "Received torque command with %zu elements, expected %zu",
        msg->data.size(), expected_size);
    return;
  }

  // 复制力矩指令
  torque_command_ = msg->data;
}

void PandaSimNode::saturation_callback(
    const std_msgs::msg::Bool::SharedPtr msg) {
  is_saturated_.store(msg->data, std::memory_order_relaxed);
}

/**
 * @brief 仿真步进回调 - 执行一步物理仿真并发布关节状态
 *
 * 该函数由定时器周期性调用，完成以下任务：
 * 1. 应用力矩指令到 MuJoCo 控制输入
 * 2. 推进 MuJoCo 仿真一个时间步
 * 3. 读取各关节的位置和速度
 * 4. 发布 JointState 消息到 ROS2 话题
 */
void PandaSimNode::step() {
  std::vector<double> positions(joint_indices_.size());
  std::vector<double> velocities(joint_indices_.size());
  std::vector<double> efforts(joint_indices_.size());

  {
    // 使用互斥锁保护仿真数据，避免与渲染线程冲突
    std::lock_guard<std::mutex> lock(data_mutex_);

    // 应用力矩指令到 MuJoCo 控制输入
    {
      std::lock_guard<std::mutex> torque_lock(torque_mutex_);
      for (std::size_t i = 0; i < torque_command_.size() &&
                              i < static_cast<std::size_t>(model_->nu);
           ++i) {
        data_->ctrl[i] = torque_command_[i];
      }
    }

    // 推进仿真一个时间步（使用模型定义的 timestep）
    mj_step(model_.get(), data_.get());
    simulation_time_ += model_->opt.timestep;

    // 记录数据 (如果在记录模式)
    if (recording_) {
      record_current_step();
    }

    // 提取各关节的位置 (qpos) 和速度 (qvel)
    for (std::size_t i = 0; i < joint_indices_.size(); ++i) {
      const int j = joint_indices_[i];
      const int qpos_adr = model_->jnt_qposadr[j]; // 位置数组中的索引
      const int qvel_adr = model_->jnt_dofadr[j];  // 速度数组中的索引
      positions[i] = data_->qpos[qpos_adr];
      velocities[i] = data_->qvel[qvel_adr];
      efforts[i] = data_->qfrc_actuator[qvel_adr];
    }
  }

  // 构建并发布 JointState 消息
  auto msg = sensor_msgs::msg::JointState();
  msg.header.stamp = this->now();
  msg.name = joint_names_;
  msg.position = std::move(positions);
  msg.velocity = std::move(velocities);
  msg.effort = std::move(efforts);

  joint_pub_->publish(std::move(msg));
}

void PandaSimNode::start_data_recording(const std::string &filename) {
  data_file_.open(filename);
  if (data_file_.is_open()) {
    // 写入 CSV 头
    // 只记录前 7 个手臂关节，与 identification 期望的 n_dof=7 匹配
    constexpr std::size_t N_ARM_JOINTS = 7;
    data_file_ << "time";
    for (std::size_t i = 0; i < N_ARM_JOINTS; ++i) {
      data_file_ << ",q" << i;
    }
    for (std::size_t i = 0; i < N_ARM_JOINTS; ++i) {
      data_file_ << ",qd" << i;
    }
    for (std::size_t i = 0; i < N_ARM_JOINTS; ++i) {
      data_file_ << ",qdd" << i;
    }
    for (std::size_t i = 0; i < N_ARM_JOINTS; ++i) {
      data_file_ << ",tau" << i;
    }
    data_file_ << "\n";
    recording_ = true;
    RCLCPP_INFO(get_logger(), "Recording simulation data to: %s (7 DOF)",
                filename.c_str());
  } else {
    RCLCPP_ERROR(get_logger(), "Failed to open data file: %s",
                 filename.c_str());
  }
}

void PandaSimNode::stop_data_recording() {
  if (data_file_.is_open()) {
    data_file_.close();
    recording_ = false;
    RCLCPP_INFO(get_logger(), "Stopped data recording.");
  }
}

void PandaSimNode::record_current_step() {
  if (!data_file_.is_open())
    return;

  // 如果力矩饱和，则跳过记录
  if (is_saturated_.load(std::memory_order_relaxed)) {
    return;
  }

  // 只记录前 7 个手臂关节，与 identification 期望的 n_dof=7 匹配
  constexpr std::size_t N_ARM_JOINTS = 7;

  // 使用缓冲区构建行字符串，减少 I/O 碎片
  std::ostringstream ss;
  ss << std::fixed << std::setprecision(6) << simulation_time_;

  // q (使用 data_->qpos) - 前 7 个关节
  for (std::size_t i = 0; i < N_ARM_JOINTS && i < joint_indices_.size(); ++i) {
    const int j = joint_indices_[i];
    const int qpos_adr = model_->jnt_qposadr[j];
    ss << "," << data_->qpos[qpos_adr];
  }

  // qd (使用 data_->qvel) - 前 7 个关节
  for (std::size_t i = 0; i < N_ARM_JOINTS && i < joint_indices_.size(); ++i) {
    const int j = joint_indices_[i];
    const int qvel_adr = model_->jnt_dofadr[j];
    ss << "," << data_->qvel[qvel_adr];
  }

  // qdd (使用 data_->qacc) - 前 7 个关节
  for (std::size_t i = 0; i < N_ARM_JOINTS && i < joint_indices_.size(); ++i) {
    const int j = joint_indices_[i];
    const int qacc_adr = model_->jnt_dofadr[j];
    ss << "," << data_->qacc[qacc_adr];
  }

  // tau (使用 data_->ctrl) - 前 7 个 actuator
  for (std::size_t i = 0; i < N_ARM_JOINTS && i < static_cast<std::size_t>(model_->nu); ++i) {
    ss << "," << data_->ctrl[i];
  }

  ss << "\n";
  data_file_ << ss.str();

  // Periodic flush to prevent data loss on crash
  ++record_count_;
  if (record_count_ % FLUSH_INTERVAL == 0) {
    data_file_.flush();
  }
}

// ==================== GLFW 回调函数 ====================

/**
 * @brief GLFW 错误回调 - 记录 GLFW 错误信息
 */
void PandaSimNode::glfw_error_callback(int error, const char *description) {
  RCLCPP_ERROR(rclcpp::get_logger("glfw"), "GLFW error %d: %s", error,
               description);
}

/**
 * @brief 鼠标按键回调 - 处理鼠标按键事件
 *
 * 记录当前鼠标按键状态和光标位置，用于后续的相机控制
 */
void PandaSimNode::glfw_mouse_button_callback(GLFWwindow *window, int, int,
                                              int) {
  auto *self = static_cast<PandaSimNode *>(glfwGetWindowUserPointer(window));
  if (!self) {
    return;
  }

  // 更新鼠标按键状态
  self->mouse_button_left_ =
      (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS);
  self->mouse_button_middle_ =
      (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS);
  self->mouse_button_right_ =
      (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS);

  // 记录当前光标位置作为拖拽起点
  glfwGetCursorPos(window, &self->last_cursor_x_, &self->last_cursor_y_);
}

/**
 * @brief 光标移动回调 - 处理鼠标拖拽控制相机
 *
 * 相机控制方式：
 * - 左键拖拽: 旋转相机 (Shift+左键: 水平旋转)
 * - 右键拖拽: 平移相机 (Shift+右键: 水平平移)
 * - 中键拖拽: 缩放相机
 */
void PandaSimNode::glfw_cursor_pos_callback(GLFWwindow *window, double xpos,
                                            double ypos) {
  auto *self = static_cast<PandaSimNode *>(glfwGetWindowUserPointer(window));
  if (!self) {
    return;
  }

  // 没有按键按下时不处理
  if (!self->mouse_button_left_ && !self->mouse_button_middle_ &&
      !self->mouse_button_right_) {
    return;
  }

  // 计算光标移动量
  const double dx = xpos - self->last_cursor_x_;
  const double dy = ypos - self->last_cursor_y_;
  self->last_cursor_x_ = xpos;
  self->last_cursor_y_ = ypos;

  int height = 0;
  glfwGetWindowSize(window, nullptr, &height);
  if (height <= 0) {
    return;
  }

  // 检测 Shift 键状态
  const bool shift_pressed =
      (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS) ||
      (glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS);

  // 根据按键组合确定相机操作类型
  mjtMouse action = mjMOUSE_NONE;
  if (self->mouse_button_right_) {
    action = shift_pressed ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
  } else if (self->mouse_button_left_) {
    action = shift_pressed ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
  } else if (self->mouse_button_middle_) {
    action = mjMOUSE_ZOOM;
  }

  // 执行相机移动
  if (action != mjMOUSE_NONE) {
    mjv_moveCamera(self->model_.get(), action, dx / static_cast<double>(height),
                   dy / static_cast<double>(height), &self->scene_,
                   &self->camera_);
  }
}

/**
 * @brief 滚轮回调 - 处理鼠标滚轮缩放相机
 */
void PandaSimNode::glfw_scroll_callback(GLFWwindow *window, double,
                                        double yoffset) {
  auto *self = static_cast<PandaSimNode *>(glfwGetWindowUserPointer(window));
  if (!self) {
    return;
  }

  // 滚轮控制相机缩放
  mjv_moveCamera(self->model_.get(), mjMOUSE_ZOOM, 0, -0.05 * yoffset,
                 &self->scene_, &self->camera_);
}

// ==================== 可视化管理 ====================

/**
 * @brief 启动可视化线程
 *
 * 在独立线程中运行 GLFW 窗口和 MuJoCo 渲染循环
 */
void PandaSimNode::start_viewer() {
  // 避免重复启动
  if (viewer_thread_.joinable()) {
    return;
  }

  viewer_running_ = true;
  viewer_thread_ = std::thread(&PandaSimNode::render_loop, this);
}

/**
 * @brief 渲染循环 - 可视化线程主函数
 *
 * 负责：
 * 1. 初始化 GLFW 和 OpenGL 上下文
 * 2. 创建 MuJoCo 可视化对象（相机、场景、渲染上下文）
 * 3. 持续更新场景并渲染到窗口
 * 4. 清理资源
 */
void PandaSimNode::render_loop() {
  // 初始化 GLFW
  glfwSetErrorCallback(PandaSimNode::glfw_error_callback);
  if (!glfwInit()) {
    RCLCPP_WARN(get_logger(),
                "Failed to initialize GLFW; continuing without viewer.");
    viewer_running_ = false;
    return;
  }

  // 创建窗口
  window_ =
      glfwCreateWindow(1280, 720, "Panda MuJoCo Viewer", nullptr, nullptr);
  if (!window_) {
    RCLCPP_WARN(get_logger(),
                "Failed to create GLFW window; continuing without viewer.");
    glfwTerminate();
    viewer_running_ = false;
    return;
  }

  // 设置 OpenGL 上下文
  glfwMakeContextCurrent(window_);
  glfwSwapInterval(1); // 启用垂直同步

  // 注册输入回调
  glfwSetWindowUserPointer(window_, this);
  glfwSetCursorPosCallback(window_, PandaSimNode::glfw_cursor_pos_callback);
  glfwSetMouseButtonCallback(window_, PandaSimNode::glfw_mouse_button_callback);
  glfwSetScrollCallback(window_, PandaSimNode::glfw_scroll_callback);

  // 初始化 MuJoCo 可视化对象
  mjv_defaultCamera(&camera_);
  mjv_defaultOption(&visual_opt_);
  mjv_defaultPerturb(&perturb_);
  mjv_defaultScene(&scene_);
  mjr_defaultContext(&render_context_);

  // 创建场景和渲染上下文
  mjv_makeScene(model_.get(), &scene_, 1000); // 最大 1000 个几何体
  mjr_makeContext(model_.get(), &render_context_, mjFONTSCALE_150);

  RCLCPP_INFO(get_logger(), "MuJoCo viewer started (GLFW window).");

  // 主渲染循环
  while (rclcpp::ok() && viewer_running_ && window_ &&
         !glfwWindowShouldClose(window_)) {
    {
      // 在锁保护下更新场景（从仿真数据同步到可视化场景）
      std::lock_guard<std::mutex> lock(data_mutex_);
      mjv_updateScene(model_.get(), data_.get(), &visual_opt_, &perturb_,
                      &camera_, mjCAT_ALL, &scene_);
    }

    // 获取帧缓冲区大小并渲染
    mjrRect viewport = {0, 0, 0, 0};
    glfwGetFramebufferSize(window_, &viewport.width, &viewport.height);
    mjr_render(viewport, &scene_, &render_context_);

    // 交换缓冲区并处理事件
    glfwSwapBuffers(window_);
    glfwPollEvents();
  }

  // 清理 MuJoCo 可视化资源
  mjr_freeContext(&render_context_);
  mjv_freeScene(&scene_);

  // 清理 GLFW 资源
  glfwDestroyWindow(window_);
  window_ = nullptr;

  glfwTerminate();
  viewer_running_ = false;
}

/**
 * @brief 停止可视化线程
 *
 * 设置停止标志并等待渲染线程结束
 */
void PandaSimNode::stop_viewer() {
  viewer_running_ = false;

  if (viewer_thread_.joinable()) {
    viewer_thread_.join();
  }
}

} // namespace sim_com_node

RCLCPP_COMPONENTS_REGISTER_NODE(sim_com_node::PandaSimNode)
