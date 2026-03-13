#ifndef APP_PIPER_HARDWARE_BACKEND_HPP_
#define APP_PIPER_HARDWARE_BACKEND_HPP_

#include "app/experiment_backend.hpp"

#include <cstdio>
#include <filesystem>
#include <memory>
#include <string>

namespace app {

class PiperHardwareBackend : public ExperimentBackend {
public:
  PiperHardwareBackend(const std::filesystem::path &bridge_script,
                       const std::filesystem::path &bridge_config,
                       double time_step);
  ~PiperHardwareBackend() override;

  ExperimentState initialize() override;
  ExperimentState step(const force_node::ControlCommand &command) override;
  double simulationTime() const override { return simulation_time_; }
  double timeStep() const override { return time_step_; }

private:
  void startBridge();
  void stopBridge();
  void sendLine(const std::string &line);
  std::string readLine();
  static ExperimentState parseStateLine(const std::string &line);
  static std::string formatStepCommand(const force_node::ControlCommand &command);

  std::filesystem::path bridge_script_;
  std::filesystem::path bridge_config_;
  double time_step_{0.0};
  double simulation_time_{0.0};

  int child_pid_{-1};
  FILE *child_stdin_{nullptr};
  FILE *child_stdout_{nullptr};
};

} // namespace app

#endif
