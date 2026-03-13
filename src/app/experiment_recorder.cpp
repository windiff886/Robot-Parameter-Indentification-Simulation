#include "app/experiment_recorder.hpp"

#include <filesystem>
#include <iomanip>
#include <sstream>
#include <stdexcept>

namespace fs = std::filesystem;

namespace app {

ExperimentRecorder::ExperimentRecorder(const std::filesystem::path &output_csv,
                                       std::size_t dof)
    : dof_(dof), previous_velocity_(dof, 0.0) {
  if (output_csv.has_parent_path()) {
    fs::create_directories(output_csv.parent_path());
  }
  file_.open(output_csv);
  if (!file_) {
    throw std::runtime_error("无法创建实验记录文件: " + output_csv.string());
  }

  file_ << "time";
  for (std::size_t i = 0; i < dof_; ++i) {
    file_ << ",q" << i;
  }
  for (std::size_t i = 0; i < dof_; ++i) {
    file_ << ",qd" << i;
  }
  for (std::size_t i = 0; i < dof_; ++i) {
    file_ << ",qdd" << i;
  }
  for (std::size_t i = 0; i < dof_; ++i) {
    file_ << ",tau" << i;
  }
  file_ << "\n";
}

ExperimentRecorder::~ExperimentRecorder() {
  if (file_.is_open()) {
    file_.close();
  }
}

void ExperimentRecorder::record(double time, const ExperimentState &state,
                                const force_node::ControlCommand &command) {
  if (!file_ || command.saturated) {
    return;
  }

  std::vector<double> qdd(dof_, 0.0);
  if (has_previous_ && time > previous_time_) {
    const double dt = time - previous_time_;
    for (std::size_t i = 0; i < dof_; ++i) {
      qdd[i] = (state.velocity[i] - previous_velocity_[i]) / dt;
    }
  }

  std::ostringstream line;
  line << std::fixed << std::setprecision(6) << time;
  for (std::size_t i = 0; i < dof_; ++i) {
    line << "," << state.position[i];
  }
  for (std::size_t i = 0; i < dof_; ++i) {
    line << "," << state.velocity[i];
  }
  for (std::size_t i = 0; i < dof_; ++i) {
    line << "," << qdd[i];
  }
  for (std::size_t i = 0; i < dof_; ++i) {
    line << "," << command.torque[i];
  }
  line << "\n";
  file_ << line.str();
  file_.flush();

  previous_time_ = time;
  previous_velocity_ = state.velocity;
  has_previous_ = true;
}

} // namespace app
