#ifndef SIM_COM_NODE__PANDA_SIM_NODE_HPP_
#define SIM_COM_NODE__PANDA_SIM_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <mujoco/mujoco.h>

#include <atomic>
#include <filesystem>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

struct GLFWwindow;

namespace sim_com_node
{

class PandaSimNode : public rclcpp::Node
{
public:
  explicit PandaSimNode(const rclcpp::NodeOptions & options);
  ~PandaSimNode() override;

private:
  void step();
  void start_viewer();
  void stop_viewer();
  void render_loop();
  static void glfw_error_callback(int error, const char * description);
  static void glfw_mouse_button_callback(GLFWwindow * window, int button, int action, int mods);
  static void glfw_cursor_pos_callback(GLFWwindow * window, double xpos, double ypos);
  static void glfw_scroll_callback(GLFWwindow * window, double xoffset, double yoffset);

  std::unique_ptr<mjData, decltype(&mj_deleteData)> data_{nullptr, mj_deleteData};
  std::unique_ptr<mjModel, decltype(&mj_deleteModel)> model_{nullptr, mj_deleteModel};
  std::vector<int> joint_indices_;
  std::vector<std::string> joint_names_;
  std::mutex data_mutex_;

  mjvCamera camera_{};
  mjvOption visual_opt_{};
  mjvScene scene_{};
  mjrContext render_context_{};
  mjvPerturb perturb_{};
  GLFWwindow * window_{nullptr};
  std::atomic_bool viewer_running_{false};
  std::thread viewer_thread_;
  bool viewer_enabled_{false};
  bool mouse_button_left_{false};
  bool mouse_button_middle_{false};
  bool mouse_button_right_{false};
  double last_cursor_x_{0.0};
  double last_cursor_y_{0.0};

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace sim_com_node

#endif  // SIM_COM_NODE__PANDA_SIM_NODE_HPP_
