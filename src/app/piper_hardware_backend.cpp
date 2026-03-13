#include "app/piper_hardware_backend.hpp"

#include <array>
#include <csignal>
#include <cstdio>
#include <cstring>
#include <sstream>
#include <stdexcept>
#include <string>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>

namespace app {

PiperHardwareBackend::PiperHardwareBackend(
    const std::filesystem::path &bridge_script,
    const std::filesystem::path &bridge_config, double time_step)
    : bridge_script_(bridge_script), bridge_config_(bridge_config),
      time_step_(time_step) {}

PiperHardwareBackend::~PiperHardwareBackend() {
  stopBridge();
}

ExperimentState PiperHardwareBackend::initialize() {
  startBridge();
  sendLine("INIT");
  return parseStateLine(readLine());
}

ExperimentState
PiperHardwareBackend::step(const force_node::ControlCommand &command) {
  sendLine(formatStepCommand(command));
  simulation_time_ += time_step_;
  return parseStateLine(readLine());
}

void PiperHardwareBackend::startBridge() {
  if (child_pid_ > 0) {
    return;
  }

  int stdin_pipe[2];
  int stdout_pipe[2];
  if (pipe(stdin_pipe) != 0 || pipe(stdout_pipe) != 0) {
    throw std::runtime_error("创建 bridge 管道失败");
  }

  child_pid_ = fork();
  if (child_pid_ < 0) {
    throw std::runtime_error("fork bridge 进程失败");
  }

  if (child_pid_ == 0) {
    dup2(stdin_pipe[0], STDIN_FILENO);
    dup2(stdout_pipe[1], STDOUT_FILENO);
    close(stdin_pipe[1]);
    close(stdout_pipe[0]);
    execlp("python3", "python3", bridge_script_.string().c_str(), "--config",
           bridge_config_.string().c_str(), static_cast<char *>(nullptr));
    std::perror("execlp");
    _exit(127);
  }

  close(stdin_pipe[0]);
  close(stdout_pipe[1]);
  child_stdin_ = fdopen(stdin_pipe[1], "w");
  child_stdout_ = fdopen(stdout_pipe[0], "r");
  if (!child_stdin_ || !child_stdout_) {
    throw std::runtime_error("打开 bridge 标准流失败");
  }
}

void PiperHardwareBackend::stopBridge() {
  if (child_pid_ <= 0) {
    return;
  }

  try {
    sendLine("STOP");
    readLine();
  } catch (const std::exception &) {
  }

  if (child_stdin_) {
    fclose(child_stdin_);
    child_stdin_ = nullptr;
  }
  if (child_stdout_) {
    fclose(child_stdout_);
    child_stdout_ = nullptr;
  }

  int status = 0;
  waitpid(child_pid_, &status, 0);
  child_pid_ = -1;
}

void PiperHardwareBackend::sendLine(const std::string &line) {
  if (!child_stdin_) {
    throw std::runtime_error("bridge stdin 未初始化");
  }
  std::fprintf(child_stdin_, "%s\n", line.c_str());
  std::fflush(child_stdin_);
}

std::string PiperHardwareBackend::readLine() {
  if (!child_stdout_) {
    throw std::runtime_error("bridge stdout 未初始化");
  }
  std::array<char, 4096> buffer{};
  if (!std::fgets(buffer.data(), static_cast<int>(buffer.size()), child_stdout_)) {
    throw std::runtime_error("读取 bridge 输出失败");
  }
  std::string line(buffer.data());
  while (!line.empty() && (line.back() == '\n' || line.back() == '\r')) {
    line.pop_back();
  }
  if (line.rfind("ERROR", 0) == 0) {
    throw std::runtime_error("bridge 返回错误: " + line);
  }
  return line;
}

ExperimentState PiperHardwareBackend::parseStateLine(const std::string &line) {
  std::stringstream stream(line);
  std::string tag;
  stream >> tag;
  if (tag != "STATE") {
    throw std::runtime_error("bridge 状态格式错误: " + line);
  }

  ExperimentState state;
  state.position.resize(6);
  state.velocity.resize(6);
  state.effort.resize(6);
  for (double &value : state.position) {
    stream >> value;
  }
  for (double &value : state.velocity) {
    stream >> value;
  }
  for (double &value : state.effort) {
    stream >> value;
  }
  if (!stream) {
    throw std::runtime_error("bridge 状态字段数量不足: " + line);
  }
  return state;
}

std::string PiperHardwareBackend::formatStepCommand(
    const force_node::ControlCommand &command) {
  std::ostringstream line;
  line << "STEP";
  for (const double value : command.desired_position) {
    line << ' ' << value;
  }
  for (const double value : command.desired_velocity) {
    line << ' ' << value;
  }
  for (const double value : command.kp) {
    line << ' ' << value;
  }
  for (const double value : command.kd) {
    line << ' ' << value;
  }
  for (const double value : command.torque) {
    line << ' ' << value;
  }
  return line.str();
}

} // namespace app
