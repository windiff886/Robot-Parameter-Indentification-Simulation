#ifndef FORCE_NODE__FORCE_CONTROLLER_NODE_HPP_
#define FORCE_NODE__FORCE_CONTROLLER_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include "robot/franka_panda.hpp"
#include "trajectory/fourier_trajectory.hpp"

#include <fstream>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace force_node {

/**
 * @brief Controller mode
 */
enum class ControllerMode {
  HOLD_POSITION,        ///< Hold at target position (original behavior)
  EXCITATION_TRAJECTORY ///< Execute excitation trajectory
};

/**
 * @brief 力矩控制器节点
 *
 * 支持两种模式：
 * 1. HOLD_POSITION: 定点 PD 控制
 * 2. EXCITATION_TRAJECTORY: 执行激励轨迹用于参数辨识
 */
class ForceControllerNode : public rclcpp::Node {
public:
  explicit ForceControllerNode(const rclcpp::NodeOptions &options);
  ~ForceControllerNode() override = default;

private:
  /**
   * @brief 关节状态回调 - 接收关节状态
   */
  void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg);

  /**
   * @brief 定时器回调 - 周期性发布力矩指令
   */
  void control_loop();

  /**
   * @brief 计算控制力矩（PD 轨迹跟踪）
   * @param q 当前关节位置
   * @param dq 当前关节速度
   * @return 计算得到的力矩向量
   */
  std::vector<double> compute_torque(const std::vector<double> &q,
                                     const std::vector<double> &dq);

  /**
   * @brief 初始化激励轨迹
   */
  void init_excitation_trajectory();

  /**
   * @brief 开始记录数据
   */
  void start_data_recording(const std::string &filename);

  /**
   * @brief 记录当前数据点
   */
  void record_data_point(double t, const std::vector<double> &q,
                         const std::vector<double> &qd,
                         const std::vector<double> &tau);

  // ROS2 通信
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr
      joint_state_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr torque_pub_;
  rclcpp::TimerBase::SharedPtr control_timer_;

  // 控制参数
  std::vector<double> kp_;              // 位置增益
  std::vector<double> kd_;              // 速度增益（阻尼）
  std::vector<double> target_position_; // 目标位置（HOLD_POSITION 模式）

  // 状态数据
  std::mutex state_mutex_;
  std::vector<double> current_position_;
  std::vector<double> current_velocity_;
  bool state_received_{false};

  // 控制模式
  ControllerMode mode_{ControllerMode::EXCITATION_TRAJECTORY};

  // 激励轨迹
  std::unique_ptr<trajectory::FourierTrajectory> trajectory_;
  double trajectory_start_time_{0.0};
  double trajectory_duration_{10.0}; // 轨迹时长 (秒)
  bool trajectory_started_{false};
  bool trajectory_finished_{false};

  // 机器人模型
  std::unique_ptr<robot::RobotModel> robot_model_;

  // 数据记录
  std::ofstream data_file_;
  bool recording_{false};
  std::vector<double> last_torques_;

  // 关节数量
  static constexpr std::size_t NUM_ARM_JOINTS = 7;
  static constexpr std::size_t NUM_FINGER_JOINTS = 2;
  static constexpr std::size_t NUM_TOTAL_JOINTS =
      NUM_ARM_JOINTS + NUM_FINGER_JOINTS;
};

} // namespace force_node

#endif // FORCE_NODE__FORCE_CONTROLLER_NODE_HPP_
