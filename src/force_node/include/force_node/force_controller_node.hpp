#ifndef FORCE_NODE__FORCE_CONTROLLER_NODE_HPP_
#define FORCE_NODE__FORCE_CONTROLLER_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <mutex>
#include <string>
#include <vector>

namespace force_node
{

/**
 * @brief 力矩控制器节点
 *
 * 该节点订阅关节状态，根据控制律计算力矩，并发布力矩指令。
 * 示例实现了简单的 PD 位置控制器，用户可以根据需要修改控制律。
 */
class ForceControllerNode : public rclcpp::Node
{
public:
  explicit ForceControllerNode(const rclcpp::NodeOptions & options);
  ~ForceControllerNode() override = default;

private:
  /**
   * @brief 关节状态回调 - 接收关节状态并计算力矩
   */
  void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg);

  /**
   * @brief 定时器回调 - 周期性发布力矩指令
   */
  void control_loop();

  /**
   * @brief 计算控制力矩（PD 控制器示例）
   * @param q 当前关节位置
   * @param dq 当前关节速度
   * @return 计算得到的力矩向量
   */
  std::vector<double> compute_torque(
    const std::vector<double> & q,
    const std::vector<double> & dq);

  // ROS2 通信
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr torque_pub_;
  rclcpp::TimerBase::SharedPtr control_timer_;

  // 控制参数
  std::vector<double> kp_;  // 位置增益
  std::vector<double> kd_;  // 速度增益（阻尼）
  std::vector<double> target_position_;  // 目标位置

  // 状态数据
  std::mutex state_mutex_;
  std::vector<double> current_position_;
  std::vector<double> current_velocity_;
  bool state_received_{false};

  // 关节数量（不包含夹爪）
  static constexpr std::size_t NUM_ARM_JOINTS = 7;
  static constexpr std::size_t NUM_FINGER_JOINTS = 2;
  static constexpr std::size_t NUM_TOTAL_JOINTS = NUM_ARM_JOINTS + NUM_FINGER_JOINTS;
};

}  // namespace force_node

#endif  // FORCE_NODE__FORCE_CONTROLLER_NODE_HPP_
