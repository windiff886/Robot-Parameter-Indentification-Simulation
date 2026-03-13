#ifndef APP_EXPERIMENT_RECORDER_HPP_
#define APP_EXPERIMENT_RECORDER_HPP_

#include "app/experiment_backend.hpp"

#include <filesystem>
#include <fstream>
#include <vector>

namespace app {

class ExperimentRecorder {
public:
  ExperimentRecorder(const std::filesystem::path &output_csv, std::size_t dof);
  ~ExperimentRecorder();

  void record(double time, const ExperimentState &state,
              const force_node::ControlCommand &command);

private:
  std::ofstream file_;
  std::size_t dof_{0};
  bool has_previous_{false};
  double previous_time_{0.0};
  std::vector<double> previous_velocity_;
};

} // namespace app

#endif
