#ifndef APP_EXPERIMENT_BACKEND_HPP_
#define APP_EXPERIMENT_BACKEND_HPP_

#include "force_node/force_controller.hpp"

#include <vector>

namespace app {

struct ExperimentState {
  std::vector<double> position;
  std::vector<double> velocity;
  std::vector<double> effort;
};

class ExperimentBackend {
public:
  virtual ~ExperimentBackend() = default;

  virtual ExperimentState initialize() = 0;
  virtual ExperimentState step(const force_node::ControlCommand &command) = 0;
  virtual double simulationTime() const = 0;
  virtual double timeStep() const = 0;
};

} // namespace app

#endif
