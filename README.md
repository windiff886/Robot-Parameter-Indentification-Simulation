## 概览

- `franka_emika_panda/`：Franka Emika Panda 的 MuJoCo 模型（MJCF）。
- `src/sim_com_node/`：ROS 2 C++ 节点，使用 MuJoCo 加载机器人并发布关节状态。

## 依赖

- MuJoCo 2.3.3+ 头文件和库（如安装在非标准路径，构建时通过 `MUJOCO_INCLUDE_DIR` 和 `MUJOCO_LIB` 指定）。
- ROS 2（已配置好 `colcon` 和环境）。

## 构建

在工作区根目录执行：

```bash
colcon build --packages-select sim_com_node
source install/setup.bash
```

## 运行示例

```bash
ros2 run sim_com_node sim_com_node --ros-args \
  -p model_path:=/home/windiff/Code/Simulation/franka_emika_panda/scene.xml \
  -p publish_rate_hz:=200
```

- 发布话题：`panda/joint_states`（`sensor_msgs/msg/JointState`，包含铰接/滑动关节位置与速度）。
- 参数：
  - `model_path`：MJCF 文件路径，默认指向仓库内 `franka_emika_panda/scene.xml`。
  - `publish_rate_hz`：关节状态发布频率，默认 100 Hz。
