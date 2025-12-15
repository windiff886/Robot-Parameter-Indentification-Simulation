/**
 * @file sim_com_node.cpp
 * @brief Panda 机械臂 MuJoCo 仿真节点实现
 *
 * 该节点使用 MuJoCo 物理引擎模拟 Franka Emika Panda 机械臂，
 * 并通过 ROS2 话题发布关节状态信息，同时提供可视化窗口。
 */

#include "sim_com_node/panda_sim_node.hpp"

#include <rclcpp_components/register_node_macro.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <chrono>
#include <filesystem>
#include <functional>
#include <stdexcept>
#include <utility>

#include <GLFW/glfw3.h>

namespace fs = std::filesystem;

namespace sim_com_node
{

/**
 * @brief 构造函数 - 初始化仿真节点
 * @param options ROS2 节点选项
 */
PandaSimNode::PandaSimNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("panda_sim_node", options)
{
  // ========== 1. 参数声明与模型路径解析 ==========
  fs::path default_model;
  try {
    // 尝试从 ROS2 包共享目录获取默认模型路径
    const auto share_dir = ament_index_cpp::get_package_share_directory("sim_com_node");
    default_model = fs::path(share_dir) / "franka_emika_panda" / "scene.xml";
  } catch (const std::exception &) {
    // 回退到相对路径
    default_model = fs::path("franka_emika_panda") / "scene.xml";
  }

  // 声明 ROS2 参数
  const std::string model_param =
    this->declare_parameter<std::string>("model_path", default_model.string());
  const double publish_rate_hz = this->declare_parameter<double>("publish_rate_hz", 100.0);
  viewer_enabled_ = this->declare_parameter<bool>("enable_viewer", true);

  // 验证模型文件存在
  const fs::path resolved_model = fs::absolute(model_param);
  if (!fs::exists(resolved_model)) {
    throw std::runtime_error("Model file not found: " + resolved_model.string());
  }

  // ========== 2. MuJoCo 模型加载 ==========
  char error[512];
  model_.reset(mj_loadXML(resolved_model.c_str(), nullptr, error, sizeof(error)));
  if (!model_) {
    throw std::runtime_error("Failed to load MuJoCo model: " + std::string(error));
  }

  // 分配仿真数据结构
  data_.reset(mj_makeData(model_.get()));
  if (!data_) {
    throw std::runtime_error("Failed to allocate MuJoCo data");
  }

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
    throw std::runtime_error("No hinge/slide joints found in the model to publish");
  }

  // ========== 5. ROS2 通信设置 ==========
  // 创建关节状态发布器
  joint_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("panda/joint_states", 10);

  // 创建定时器，按指定频率执行仿真步进和状态发布
  const auto period = std::chrono::duration<double>(1.0 / publish_rate_hz);
  timer_ = this->create_wall_timer(period, std::bind(&PandaSimNode::step, this));

  RCLCPP_INFO(
    get_logger(), "Loaded model from %s, publishing %zu joints at %.1f Hz",
    resolved_model.string().c_str(), joint_indices_.size(), publish_rate_hz);

  // ========== 6. 可视化窗口启动 ==========
  if (viewer_enabled_) {
    start_viewer();
  } else {
    RCLCPP_INFO(get_logger(), "MuJoCo viewer disabled (enable with enable_viewer:=true)");
  }
}

/**
 * @brief 析构函数 - 清理资源
 */
PandaSimNode::~PandaSimNode()
{
  stop_viewer();
}

/**
 * @brief 仿真步进回调 - 执行一步物理仿真并发布关节状态
 *
 * 该函数由定时器周期性调用，完成以下任务：
 * 1. 推进 MuJoCo 仿真一个时间步
 * 2. 读取各关节的位置和速度
 * 3. 发布 JointState 消息到 ROS2 话题
 */
void PandaSimNode::step()
{
  std::vector<double> positions(joint_indices_.size());
  std::vector<double> velocities(joint_indices_.size());

  {
    // 使用互斥锁保护仿真数据，避免与渲染线程冲突
    std::lock_guard<std::mutex> lock(data_mutex_);

    // 推进仿真一个时间步（使用模型定义的 timestep）
    mj_step(model_.get(), data_.get());

    // 提取各关节的位置 (qpos) 和速度 (qvel)
    for (std::size_t i = 0; i < joint_indices_.size(); ++i) {
      const int j = joint_indices_[i];
      const int qpos_adr = model_->jnt_qposadr[j];  // 位置数组中的索引
      const int qvel_adr = model_->jnt_dofadr[j];   // 速度数组中的索引
      positions[i] = data_->qpos[qpos_adr];
      velocities[i] = data_->qvel[qvel_adr];
    }
  }

  // 构建并发布 JointState 消息
  auto msg = sensor_msgs::msg::JointState();
  msg.header.stamp = this->now();
  msg.name = joint_names_;
  msg.position = std::move(positions);
  msg.velocity = std::move(velocities);

  joint_pub_->publish(std::move(msg));
}

// ==================== GLFW 回调函数 ====================

/**
 * @brief GLFW 错误回调 - 记录 GLFW 错误信息
 */
void PandaSimNode::glfw_error_callback(int error, const char * description)
{
  RCLCPP_ERROR(rclcpp::get_logger("glfw"), "GLFW error %d: %s", error, description);
}

/**
 * @brief 鼠标按键回调 - 处理鼠标按键事件
 *
 * 记录当前鼠标按键状态和光标位置，用于后续的相机控制
 */
void PandaSimNode::glfw_mouse_button_callback(GLFWwindow * window, int, int, int)
{
  auto * self = static_cast<PandaSimNode *>(glfwGetWindowUserPointer(window));
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
void PandaSimNode::glfw_cursor_pos_callback(GLFWwindow * window, double xpos, double ypos)
{
  auto * self = static_cast<PandaSimNode *>(glfwGetWindowUserPointer(window));
  if (!self) {
    return;
  }

  // 没有按键按下时不处理
  if (!self->mouse_button_left_ && !self->mouse_button_middle_ && !self->mouse_button_right_) {
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
    mjv_moveCamera(
      self->model_.get(), action, dx / static_cast<double>(height),
      dy / static_cast<double>(height), &self->scene_, &self->camera_);
  }
}

/**
 * @brief 滚轮回调 - 处理鼠标滚轮缩放相机
 */
void PandaSimNode::glfw_scroll_callback(GLFWwindow * window, double, double yoffset)
{
  auto * self = static_cast<PandaSimNode *>(glfwGetWindowUserPointer(window));
  if (!self) {
    return;
  }

  // 滚轮控制相机缩放
  mjv_moveCamera(
    self->model_.get(), mjMOUSE_ZOOM, 0, -0.05 * yoffset, &self->scene_, &self->camera_);
}

// ==================== 可视化管理 ====================

/**
 * @brief 启动可视化线程
 *
 * 在独立线程中运行 GLFW 窗口和 MuJoCo 渲染循环
 */
void PandaSimNode::start_viewer()
{
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
void PandaSimNode::render_loop()
{
  // 初始化 GLFW
  glfwSetErrorCallback(PandaSimNode::glfw_error_callback);
  if (!glfwInit()) {
    RCLCPP_WARN(get_logger(), "Failed to initialize GLFW; continuing without viewer.");
    viewer_running_ = false;
    return;
  }

  // 创建窗口
  window_ = glfwCreateWindow(1280, 720, "Panda MuJoCo Viewer", nullptr, nullptr);
  if (!window_) {
    RCLCPP_WARN(get_logger(), "Failed to create GLFW window; continuing without viewer.");
    glfwTerminate();
    viewer_running_ = false;
    return;
  }

  // 设置 OpenGL 上下文
  glfwMakeContextCurrent(window_);
  glfwSwapInterval(1);  // 启用垂直同步

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
  mjv_makeScene(model_.get(), &scene_, 1000);  // 最大 1000 个几何体
  mjr_makeContext(model_.get(), &render_context_, mjFONTSCALE_150);

  RCLCPP_INFO(get_logger(), "MuJoCo viewer started (GLFW window).");

  // 主渲染循环
  while (rclcpp::ok() && viewer_running_ && window_ && !glfwWindowShouldClose(window_)) {
    {
      // 在锁保护下更新场景（从仿真数据同步到可视化场景）
      std::lock_guard<std::mutex> lock(data_mutex_);
      mjv_updateScene(
        model_.get(), data_.get(), &visual_opt_, &perturb_, &camera_, mjCAT_ALL, &scene_);
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
void PandaSimNode::stop_viewer()
{
  viewer_running_ = false;

  if (viewer_thread_.joinable()) {
    viewer_thread_.join();
  }
}

}  // namespace sim_com_node

RCLCPP_COMPONENTS_REGISTER_NODE(sim_com_node::PandaSimNode)
