#ifndef SIM_COM_NODE__PANDA_SIM_NODE_HPP_
#define SIM_COM_NODE__PANDA_SIM_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

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

/**
 * @brief Panda 机械臂 MuJoCo 仿真节点
 *
 * 功能：
 * - 使用 MuJoCo 物理引擎进行机械臂仿真
 * - 发布关节状态（位置、速度、力矩）到 panda/joint_states
 * - 订阅力矩指令从 panda/joint_torques
 * - 提供 GLFW 可视化窗口
 */
class PandaSimNode : public rclcpp::Node
{
public:
  explicit PandaSimNode(const rclcpp::NodeOptions & options);
  ~PandaSimNode() override;

private:
  // ========== 核心功能函数 ==========
  void step();
  void torque_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);

  // ========== 可视化相关函数 ==========
  void start_viewer();
  void stop_viewer();
  void render_loop();
  static void glfw_error_callback(int error, const char * description);
  static void glfw_mouse_button_callback(GLFWwindow * window, int button, int action, int mods);
  static void glfw_cursor_pos_callback(GLFWwindow * window, double xpos, double ypos);
  static void glfw_scroll_callback(GLFWwindow * window, double xoffset, double yoffset);

  // ========== MuJoCo 数据 ==========
  std::unique_ptr<mjData, decltype(&mj_deleteData)> data_{nullptr, mj_deleteData};
  std::unique_ptr<mjModel, decltype(&mj_deleteModel)> model_{nullptr, mj_deleteModel};
  std::vector<int> joint_indices_;
  std::vector<std::string> joint_names_;
  std::mutex data_mutex_;

  // ========== 力矩控制 ==========
  std::vector<double> torque_command_;
  std::mutex torque_mutex_;

  // ========== 可视化数据 ==========
  mjvCamera camera_{};
  mjvOption visual_opt_{};
  mjvScene scene_{};
  mjrContext render_context_{};
  mjvPerturb perturb_{};
  GLFWwindow * window_{nullptr};
  std::atomic_bool viewer_running_{false};
  std::thread viewer_thread_;
  bool mouse_button_left_{false};
  bool mouse_button_middle_{false};
  bool mouse_button_right_{false};
  double last_cursor_x_{0.0};
  double last_cursor_y_{0.0};

  // ========== ROS2 通信 ==========
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr torque_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace sim_com_node

#endif  // SIM_COM_NODE__PANDA_SIM_NODE_HPP_
