#ifndef APP_SIMULATION_BACKEND_HPP_
#define APP_SIMULATION_BACKEND_HPP_

#include "app/experiment_backend.hpp"
#include "sim_com_node/panda_simulator.hpp"

namespace app {

class SimulationBackend : public ExperimentBackend {
public:
  SimulationBackend(const sim_com_node::PandaSimConfig &config,
                    const std::filesystem::path &scene_path,
                    std::size_t dof);

  ExperimentState initialize() override;
  ExperimentState step(const force_node::ControlCommand &command) override;
  double simulationTime() const override;
  double timeStep() const override;

private:
  static ExperimentState convertState(const sim_com_node::JointState &state);

  sim_com_node::PandaSimulator simulator_;
};

} // namespace app

#endif
