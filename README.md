## 概览

- `franka_emika_panda/`：Franka Emika Panda 的 MuJoCo 模型（MJCF）。
- `src/sim_com_node/`：ROS 2 C++ 节点，使用 MuJoCo 加载机器人并发布关节状态。
- `config/panda_sim_node.yaml`：节点参数文件（模型路径、发布频率）。

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

以组件方式运行节点

```bash
ros2 launch sim_com_node panda_sim_node.launch.py
```

- 发布话题：`panda/joint_states`（`sensor_msgs/msg/JointState`，包含铰接/滑动关节位置与速度）。
- 参数：
  - `model_path`：MJCF 文件路径，默认指向安装后的包内 `franka_emika_panda/scene.xml`，可在参数文件中覆盖。
  - `publish_rate_hz`：关节状态发布频率，默认 100 Hz，可在参数文件中调整。
  - `enable_viewer`：是否启动 GLFW 可视化窗口，默认启用；在无显示环境可设为 `false`。
