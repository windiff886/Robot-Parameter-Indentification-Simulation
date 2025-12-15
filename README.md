## 概览

- `franka_emika_panda/`：Franka Emika Panda 的 MuJoCo 模型（MJCF）。
- `src/sim_com_node/`：ROS 2 C++ 仿真节点，使用 MuJoCo 加载机器人，发布关节状态并接收力矩指令。
- `src/force_node/`：ROS 2 C++ 力矩控制器节点，订阅关节状态并发布力矩指令（内置 PD 控制器示例）。
- `config/panda_sim_node.yaml`：sim_com_node 配置文件（发布频率）。
- `config/force_controller_node.yaml`：force_node 配置文件（控制参数）。
- `src/launch/`：联合启动文件。

## 系统架构

```
┌─────────────────┐     panda/joint_states      ┌─────────────────┐
│                 │ ──────────────────────────► │                 │
│  sim_com_node   │ (position, velocity, effort)│   force_node    │
│  (MuJoCo 仿真)   │                             │  (力矩控制器)    │
│                 │ ◄────────────────────────── │                 │
└─────────────────┘     panda/joint_torques     └─────────────────┘
                          (torque command)
```

## 依赖

- MuJoCo 2.3.3+ 头文件和库（如安装在非标准路径，构建时通过 `MUJOCO_INCLUDE_DIR` 和 `MUJOCO_LIB` 指定）。
- ROS 2（已配置好 `colcon` 和环境）。
- GLFW3（用于可视化窗口）。

## 构建

在工作区根目录执行：

```bash
colcon build --packages-select sim_com_node force_node
source install/setup.bash
```

## 运行示例

### 1. 一次性启动仿真和控制器

```bash
ros2 launch src/launch/panda_sim_with_controller.launch.py
```

两个节点在同一容器中运行，通信效率更高。

### 2. 仅运行仿真节点（无控制，机械臂因重力下垂）

```bash
ros2 launch sim_com_node panda_sim_node.launch.py
```

### 3. 仅运行力矩控制器

```bash
ros2 launch force_node force_controller_node.launch.py
```

控制器会将机械臂稳定在 home 位姿。

### 4. 手动发布力矩指令

```bash
# 发布力矩指令（8 个元素：7 关节 + 1 夹爪）
ros2 topic pub /panda/joint_torques std_msgs/msg/Float64MultiArray \
  "{data: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}"
```

## 话题

| 话题名称 | 消息类型 | 方向 | 说明 |
|---------|---------|------|------|
| `panda/joint_states` | `sensor_msgs/JointState` | 仿真 → 控制器 | 关节位置、速度、力矩 |
| `panda/joint_torques` | `std_msgs/Float64MultiArray` | 控制器 → 仿真 | 力矩指令（8 维） |

## 参数

### sim_com_node

sim_com_node 从 `config/panda_sim_node.yaml` 读取配置。

| 配置项 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `publish_rate_hz` | double | 100.0 | 关节状态发布频率 (Hz) |

### force_node

force_node 从 `config/force_controller_node.yaml` 读取配置。

| 配置项 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `control_rate_hz` | double | 1000.0 | 控制频率 (Hz) |
| `kp` | double[] | [600, 600, 600, 600, 250, 150, 50] | PD 控制器位置增益 |
| `kd` | double[] | [50, 50, 50, 50, 30, 25, 15] | PD 控制器速度增益 |
| `target_position` | double[] | [0, 0, 0, -1.57, 0, 1.57, -0.79] | 目标位置 (rad) |

## 控制模式说明

模型使用 **纯力矩控制模式**：
- `data->ctrl[i]` 直接作为关节力矩 (τ = ctrl)
- 关节 1-4 力矩范围：±87 Nm
- 关节 5-7 力矩范围：±12 Nm
- 夹爪力矩范围：±20 N

## 自定义控制器

修改 `src/force_node/src/force_controller_node.cpp` 中的 `compute_torque()` 函数：

```cpp
std::vector<double> ForceControllerNode::compute_torque(
  const std::vector<double> & q,
  const std::vector<double> & dq)
{
  std::vector<double> torques(8, 0.0);

  // 在这里实现你的控制算法
  // 例如：计算动力学、阻抗控制、轨迹跟踪等

  return torques;
}
```
