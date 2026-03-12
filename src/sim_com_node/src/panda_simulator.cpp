#include "sim_com_node/panda_simulator.hpp"

#include <GLFW/glfw3.h>

#include <cctype>
#include <chrono>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <optional>
#include <sstream>
#include <stdexcept>

namespace fs = std::filesystem;

namespace {

std::string trimCopy(const std::string &input) {
  const auto first = input.find_first_not_of(" \t\r\n");
  if (first == std::string::npos) {
    return "";
  }
  const auto last = input.find_last_not_of(" \t\r\n");
  return input.substr(first, last - first + 1);
}

std::optional<double> parseYamlDouble(const std::string &trimmed,
                                      const std::string &key) {
  if (trimmed.rfind(key, 0) != 0) {
    return std::nullopt;
  }

  std::size_t pos = key.size();
  while (pos < trimmed.size() &&
         std::isspace(static_cast<unsigned char>(trimmed[pos]))) {
    ++pos;
  }
  if (pos >= trimmed.size() || trimmed[pos] != ':') {
    return std::nullopt;
  }

  std::string value = trimCopy(trimmed.substr(pos + 1));
  const auto comment = value.find('#');
  if (comment != std::string::npos) {
    value = trimCopy(value.substr(0, comment));
  }

  try {
    return std::stod(value);
  } catch (const std::exception &) {
    return std::nullopt;
  }
}

std::optional<bool> parseYamlBool(const std::string &trimmed,
                                  const std::string &key) {
  if (trimmed.rfind(key, 0) != 0) {
    return std::nullopt;
  }

  std::size_t pos = key.size();
  while (pos < trimmed.size() &&
         std::isspace(static_cast<unsigned char>(trimmed[pos]))) {
    ++pos;
  }
  if (pos >= trimmed.size() || trimmed[pos] != ':') {
    return std::nullopt;
  }

  const std::string value = trimCopy(trimmed.substr(pos + 1));
  if (value == "true" || value == "True" || value == "1") {
    return true;
  }
  if (value == "false" || value == "False" || value == "0") {
    return false;
  }
  return std::nullopt;
}

} // namespace

namespace sim_com_node {

PandaSimConfig PandaSimulator::loadConfig(const std::filesystem::path &path) {
  PandaSimConfig config;
  std::ifstream file(path);
  if (!file) {
    return config;
  }

  std::string line;
  while (std::getline(file, line)) {
    const std::string trimmed = trimCopy(line);
    if (trimmed.empty() || trimmed.front() == '#') {
      continue;
    }
    if (const auto rate = parseYamlDouble(trimmed, "simulation_rate_hz")) {
      config.simulation_rate_hz = *rate;
    } else if (const auto rate = parseYamlDouble(trimmed, "publish_rate_hz")) {
      config.simulation_rate_hz = *rate;
    } else if (const auto enable = parseYamlBool(trimmed, "enable_viewer")) {
      config.enable_viewer = *enable;
    }
  }

  return config;
}

PandaSimulator::PandaSimulator(const PandaSimConfig &config,
                               const std::filesystem::path &scene_path,
                               const std::filesystem::path &record_file,
                               std::size_t recorded_dof)
    : config_(config), recorded_dof_(recorded_dof) {
  if (!fs::exists(scene_path)) {
    throw std::runtime_error("MuJoCo 场景文件不存在: " + scene_path.string());
  }

  char error[512];
  model_.reset(mj_loadXML(scene_path.string().c_str(), nullptr, error,
                          sizeof(error)));
  if (!model_) {
    throw std::runtime_error("MuJoCo 模型加载失败: " + std::string(error));
  }

  data_.reset(mj_makeData(model_.get()));
  if (!data_) {
    throw std::runtime_error("MuJoCo 数据结构分配失败");
  }

  if (config_.simulation_rate_hz > 0.0) {
    model_->opt.timestep = 1.0 / config_.simulation_rate_hz;
  }

  const int home_id = mj_name2id(model_.get(), mjOBJ_KEY, "home");
  if (home_id >= 0) {
    mj_resetDataKeyframe(model_.get(), data_.get(), home_id);
  } else {
    mj_resetData(model_.get(), data_.get());
  }

  joint_indices_.reserve(model_->njnt);
  joint_names_.reserve(model_->njnt);
  for (int j = 0; j < model_->njnt; ++j) {
    const int type = model_->jnt_type[j];
    if (type == mjJNT_FREE || type == mjJNT_BALL) {
      continue;
    }
    joint_indices_.push_back(j);
    joint_names_.emplace_back(model_->names + model_->name_jntadr[j]);
  }

  if (joint_indices_.empty()) {
    throw std::runtime_error("未在模型中找到可用关节");
  }
  if (recorded_dof_ == 0 || recorded_dof_ > joint_indices_.size()) {
    throw std::runtime_error("记录自由度配置非法，超出当前模型可用关节数");
  }

  startDataRecording(record_file);
  if (config_.enable_viewer) {
    startViewer();
  }
}

PandaSimulator::~PandaSimulator() {
  stopViewer();
  stopDataRecording();
}

double PandaSimulator::timeStep() const {
  return model_->opt.timestep;
}

JointState PandaSimulator::currentState() const {
  std::lock_guard<std::mutex> lock(data_mutex_);

  JointState state;
  state.names = joint_names_;
  state.position.resize(joint_indices_.size());
  state.velocity.resize(joint_indices_.size());
  state.effort.resize(joint_indices_.size());

  for (std::size_t i = 0; i < joint_indices_.size(); ++i) {
    const int joint_index = joint_indices_[i];
    const int qpos_index = model_->jnt_qposadr[joint_index];
    const int qvel_index = model_->jnt_dofadr[joint_index];
    state.position[i] = data_->qpos[qpos_index];
    state.velocity[i] = data_->qvel[qvel_index];
    state.effort[i] = data_->qfrc_actuator[qvel_index];
  }

  return state;
}

void PandaSimulator::step(const std::vector<double> &torques, bool saturated) {
  std::lock_guard<std::mutex> lock(data_mutex_);
  saturated_ = saturated;

  const std::size_t limit =
      std::min<std::size_t>(torques.size(), static_cast<std::size_t>(model_->nu));
  for (std::size_t i = 0; i < limit; ++i) {
    data_->ctrl[i] = torques[i];
  }
  for (std::size_t i = limit; i < static_cast<std::size_t>(model_->nu); ++i) {
    data_->ctrl[i] = 0.0;
  }

  mj_step(model_.get(), data_.get());
  simulation_time_ += model_->opt.timestep;

  if (recording_) {
    recordCurrentStep();
  }
}

void PandaSimulator::startDataRecording(const std::filesystem::path &record_file) {
  if (record_file.has_parent_path()) {
    fs::create_directories(record_file.parent_path());
  }

  data_file_.open(record_file);
  if (!data_file_) {
    throw std::runtime_error("无法创建数据文件: " + record_file.string());
  }

  data_file_ << "time";
  for (std::size_t i = 0; i < recorded_dof_; ++i) {
    data_file_ << ",q" << i;
  }
  for (std::size_t i = 0; i < recorded_dof_; ++i) {
    data_file_ << ",qd" << i;
  }
  for (std::size_t i = 0; i < recorded_dof_; ++i) {
    data_file_ << ",qdd" << i;
  }
  for (std::size_t i = 0; i < recorded_dof_; ++i) {
    data_file_ << ",tau" << i;
  }
  data_file_ << "\n";
  recording_ = true;
}

void PandaSimulator::stopDataRecording() {
  if (data_file_.is_open()) {
    data_file_.close();
  }
  recording_ = false;
}

void PandaSimulator::recordCurrentStep() {
  if (saturated_ || !data_file_.is_open()) {
    return;
  }

  std::ostringstream stream;
  stream << std::fixed << std::setprecision(6) << simulation_time_;
  for (std::size_t i = 0; i < recorded_dof_; ++i) {
    const int joint_index = joint_indices_[i];
    stream << "," << data_->qpos[model_->jnt_qposadr[joint_index]];
  }
  for (std::size_t i = 0; i < recorded_dof_; ++i) {
    const int joint_index = joint_indices_[i];
    stream << "," << data_->qvel[model_->jnt_dofadr[joint_index]];
  }
  for (std::size_t i = 0; i < recorded_dof_; ++i) {
    const int joint_index = joint_indices_[i];
    stream << "," << data_->qacc[model_->jnt_dofadr[joint_index]];
  }
  for (std::size_t i = 0; i < recorded_dof_; ++i) {
    stream << "," << data_->ctrl[i];
  }
  stream << "\n";
  data_file_ << stream.str();

  ++record_count_;
  if (record_count_ % FLUSH_INTERVAL == 0) {
    data_file_.flush();
  }
}

void PandaSimulator::glfwErrorCallback(int error, const char *description) {
  std::cerr << "GLFW error " << error << ": " << description << std::endl;
}

void PandaSimulator::glfwMouseButtonCallback(GLFWwindow *window, int, int, int) {
  auto *self = static_cast<PandaSimulator *>(glfwGetWindowUserPointer(window));
  if (!self) {
    return;
  }

  self->mouse_button_left_ =
      (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS);
  self->mouse_button_middle_ =
      (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS);
  self->mouse_button_right_ =
      (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS);

  glfwGetCursorPos(window, &self->last_cursor_x_, &self->last_cursor_y_);
}

void PandaSimulator::glfwCursorPosCallback(GLFWwindow *window, double xpos,
                                           double ypos) {
  auto *self = static_cast<PandaSimulator *>(glfwGetWindowUserPointer(window));
  if (!self) {
    return;
  }

  if (!self->mouse_button_left_ && !self->mouse_button_middle_ &&
      !self->mouse_button_right_) {
    return;
  }

  const double dx = xpos - self->last_cursor_x_;
  const double dy = ypos - self->last_cursor_y_;
  self->last_cursor_x_ = xpos;
  self->last_cursor_y_ = ypos;

  int height = 0;
  glfwGetWindowSize(window, nullptr, &height);
  if (height <= 0) {
    return;
  }

  const bool shift_pressed =
      (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS) ||
      (glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS);

  mjtMouse action = mjMOUSE_NONE;
  if (self->mouse_button_right_) {
    action = shift_pressed ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
  } else if (self->mouse_button_left_) {
    action = shift_pressed ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
  } else if (self->mouse_button_middle_) {
    action = mjMOUSE_ZOOM;
  }

  if (action != mjMOUSE_NONE) {
    mjv_moveCamera(self->model_.get(), action, dx / static_cast<double>(height),
                   dy / static_cast<double>(height), &self->scene_,
                   &self->camera_);
  }
}

void PandaSimulator::glfwScrollCallback(GLFWwindow *window, double, double yoffset) {
  auto *self = static_cast<PandaSimulator *>(glfwGetWindowUserPointer(window));
  if (!self) {
    return;
  }

  mjv_moveCamera(self->model_.get(), mjMOUSE_ZOOM, 0.0, -0.05 * yoffset,
                 &self->scene_, &self->camera_);
}

void PandaSimulator::startViewer() {
  if (viewer_thread_.joinable()) {
    return;
  }
  viewer_running_ = true;
  viewer_thread_ = std::thread(&PandaSimulator::renderLoop, this);
}

void PandaSimulator::stopViewer() {
  viewer_running_ = false;
  if (viewer_thread_.joinable()) {
    viewer_thread_.join();
  }
}

void PandaSimulator::renderLoop() {
  glfwSetErrorCallback(PandaSimulator::glfwErrorCallback);
  if (!glfwInit()) {
    std::cerr << "GLFW 初始化失败，跳过 viewer" << std::endl;
    viewer_running_ = false;
    return;
  }

  window_ = glfwCreateWindow(1280, 720, "Panda MuJoCo Viewer", nullptr, nullptr);
  if (!window_) {
    std::cerr << "GLFW 窗口创建失败，跳过 viewer" << std::endl;
    glfwTerminate();
    viewer_running_ = false;
    return;
  }

  glfwMakeContextCurrent(window_);
  glfwSwapInterval(1);
  glfwSetWindowUserPointer(window_, this);
  glfwSetCursorPosCallback(window_, PandaSimulator::glfwCursorPosCallback);
  glfwSetMouseButtonCallback(window_, PandaSimulator::glfwMouseButtonCallback);
  glfwSetScrollCallback(window_, PandaSimulator::glfwScrollCallback);

  mjv_defaultCamera(&camera_);
  mjv_defaultOption(&visual_opt_);
  mjv_defaultPerturb(&perturb_);
  mjv_defaultScene(&scene_);
  mjr_defaultContext(&render_context_);
  mjv_makeScene(model_.get(), &scene_, 1000);
  mjr_makeContext(model_.get(), &render_context_, mjFONTSCALE_150);

  while (viewer_running_ && window_ && !glfwWindowShouldClose(window_)) {
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      mjv_updateScene(model_.get(), data_.get(), &visual_opt_, &perturb_,
                      &camera_, mjCAT_ALL, &scene_);
    }

    mjrRect viewport{0, 0, 0, 0};
    glfwGetFramebufferSize(window_, &viewport.width, &viewport.height);
    mjr_render(viewport, &scene_, &render_context_);
    glfwSwapBuffers(window_);
    glfwPollEvents();
  }

  mjr_freeContext(&render_context_);
  mjv_freeScene(&scene_);
  glfwDestroyWindow(window_);
  window_ = nullptr;
  glfwTerminate();
  viewer_running_ = false;
}

} // namespace sim_com_node
