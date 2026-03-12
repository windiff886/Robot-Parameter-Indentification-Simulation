#ifndef SIM_COM_NODE_PANDA_SIMULATOR_HPP_
#define SIM_COM_NODE_PANDA_SIMULATOR_HPP_

#include <atomic>
#include <filesystem>
#include <fstream>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <mujoco/mujoco.h>

struct GLFWwindow;

namespace sim_com_node {

struct PandaSimConfig {
  double simulation_rate_hz = 1000.0;
  bool enable_viewer = true;
};

struct JointState {
  std::vector<std::string> names;
  std::vector<double> position;
  std::vector<double> velocity;
  std::vector<double> effort;
};

class PandaSimulator {
public:
  PandaSimulator(const PandaSimConfig &config,
                 const std::filesystem::path &scene_path,
                 const std::filesystem::path &record_file,
                 std::size_t recorded_dof);
  ~PandaSimulator();

  static PandaSimConfig loadConfig(const std::filesystem::path &config_path);

  double simulationTime() const { return simulation_time_; }
  double timeStep() const;
  JointState currentState() const;
  void step(const std::vector<double> &torques, bool saturated);

private:
  void startViewer();
  void stopViewer();
  void renderLoop();
  void startDataRecording(const std::filesystem::path &record_file);
  void stopDataRecording();
  void recordCurrentStep();

  static void glfwErrorCallback(int error, const char *description);
  static void glfwMouseButtonCallback(GLFWwindow *window, int button, int action,
                                      int mods);
  static void glfwCursorPosCallback(GLFWwindow *window, double xpos,
                                    double ypos);
  static void glfwScrollCallback(GLFWwindow *window, double xoffset,
                                 double yoffset);

  PandaSimConfig config_;
  std::unique_ptr<mjData, decltype(&mj_deleteData)> data_{nullptr,
                                                          mj_deleteData};
  std::unique_ptr<mjModel, decltype(&mj_deleteModel)> model_{nullptr,
                                                             mj_deleteModel};

  std::vector<int> joint_indices_;
  std::vector<std::string> joint_names_;
  mutable std::mutex data_mutex_;

  std::ofstream data_file_;
  bool recording_{false};
  bool saturated_{false};
  double simulation_time_{0.0};
  std::size_t record_count_{0};
  std::size_t recorded_dof_{0};
  static constexpr std::size_t FLUSH_INTERVAL = 1000;

  mjvCamera camera_{};
  mjvOption visual_opt_{};
  mjvScene scene_{};
  mjrContext render_context_{};
  mjvPerturb perturb_{};
  GLFWwindow *window_{nullptr};
  std::atomic_bool viewer_running_{false};
  std::thread viewer_thread_;
  bool mouse_button_left_{false};
  bool mouse_button_middle_{false};
  bool mouse_button_right_{false};
  double last_cursor_x_{0.0};
  double last_cursor_y_{0.0};
};

} // namespace sim_com_node

#endif
