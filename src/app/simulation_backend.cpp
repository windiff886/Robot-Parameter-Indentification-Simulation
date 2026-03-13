#include "app/simulation_backend.hpp"

namespace app {

SimulationBackend::SimulationBackend(const sim_com_node::PandaSimConfig &config,
                                     const std::filesystem::path &scene_path,
                                     std::size_t dof)
    : simulator_(config, scene_path, std::filesystem::path(), dof) {}

ExperimentState SimulationBackend::initialize() {
  return convertState(simulator_.currentState());
}

ExperimentState
SimulationBackend::step(const force_node::ControlCommand &command) {
  simulator_.step(command.torque, command.saturated);
  return convertState(simulator_.currentState());
}

double SimulationBackend::simulationTime() const {
  return simulator_.simulationTime();
}

double SimulationBackend::timeStep() const {
  return simulator_.timeStep();
}

ExperimentState
SimulationBackend::convertState(const sim_com_node::JointState &state) {
  return ExperimentState{state.position, state.velocity, state.effort};
}

} // namespace app
