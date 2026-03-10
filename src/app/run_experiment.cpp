#include "force_node/force_controller.hpp"
#include "sim_com_node/panda_simulator.hpp"

#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

namespace fs = std::filesystem;

namespace {

fs::path repoRoot() {
  return fs::path(PROJECT_ROOT_DIR);
}

fs::path readArgPath(int argc, char **argv, const std::string &flag,
                     const fs::path &default_value) {
  for (int i = 1; i + 1 < argc; ++i) {
    if (argv[i] == flag) {
      return fs::path(argv[i + 1]);
    }
  }
  return default_value;
}

bool hasFlag(int argc, char **argv, const std::string &flag) {
  for (int i = 1; i < argc; ++i) {
    if (argv[i] == flag) {
      return true;
    }
  }
  return false;
}

fs::path defaultRecordFile() {
  return repoRoot() / "data" / "benchmark_data.csv";
}

} // namespace

int main(int argc, char **argv) {
  try {
    const fs::path force_config = readArgPath(
        argc, argv, "--controller-config",
        repoRoot() / "config" / "force_controller_node.yaml");
    const fs::path sim_config = readArgPath(
        argc, argv, "--sim-config",
        repoRoot() / "config" / "panda_sim_node.yaml");
    const fs::path scene_path = readArgPath(
        argc, argv, "--scene",
        repoRoot() / "franka_emika_panda" / "scene.xml");
    const fs::path collision_model = readArgPath(
        argc, argv, "--collision-model",
        repoRoot() / "franka_emika_panda" / "panda.xml");
    const fs::path output_csv =
        readArgPath(argc, argv, "--output", defaultRecordFile());
    const bool headless = hasFlag(argc, argv, "--headless");

    auto controller_config = force_node::ForceController::loadConfig(force_config);
    auto sim_config_loaded = sim_com_node::PandaSimulator::loadConfig(sim_config);
    sim_config_loaded.enable_viewer = !headless && sim_config_loaded.enable_viewer;

    force_node::ForceController controller(controller_config, collision_model);
    sim_com_node::PandaSimulator simulator(sim_config_loaded, scene_path,
                                           output_csv);

    const double end_time = controller.trajectoryDuration() + 2.0;
    while (simulator.simulationTime() < end_time) {
      const auto state = simulator.currentState();
      force_node::JointSample sample{state.position, state.velocity, state.effort};
      const auto torque =
          controller.computeTorque(sample, simulator.simulationTime());
      simulator.step(torque, controller.isTorqueSaturated());
    }

    std::cout << "实验完成，数据已保存到: " << output_csv << std::endl;
    return 0;
  } catch (const std::exception &e) {
    std::cerr << "run_experiment 失败: " << e.what() << std::endl;
    return 1;
  }
}
