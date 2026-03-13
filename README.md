# 机器人动力学参数辨识系统

本分支是一个**纯 MuJoCo / 纯 CMake / 无 ROS** 的参数辨识实验版本，目标是在
Windows 或 Linux 上直接构建并运行。

> **声明**：本项目参考了 [BIRDy (Benchmark for Identification of Robot Dynamics)](https://github.com/TUM-ICS/BIRDY) 开源项目。

---

## 快速开始

### 1. 安装依赖

需要预先安装并配置：

- MuJoCo
- GLFW
- Eigen3
- CMake 3.21+
- C++17 编译器

如果 MuJoCo 未安装在系统默认路径，请设置环境变量 `MUJOCO_DIR`。

### 2. 配置项目

```bash
cmake -S . -B build
```

### 3. 构建

```bash
cmake --build build --parallel
```

### 4. 运行统一实验入口

```bash
./build/run_experiment
```

默认会：

- 读取 [experiment.yaml](/home/windiff/Code/Robot-Parameter-Indentification-Simulation/config/experiment.yaml)
- 读取 [force_controller_node.yaml](/home/windiff/Code/Robot-Parameter-Indentification-Simulation/config/force_controller_node.yaml)
- 读取 [panda_sim_node.yaml](/home/windiff/Code/Robot-Parameter-Indentification-Simulation/config/panda_sim_node.yaml)
- 使用 [scene.xml](/home/windiff/Code/Robot-Parameter-Indentification-Simulation/franka_emika_panda/scene.xml)
- 在 `data/benchmark_data.csv` 输出采样数据

通过修改 [experiment.yaml](/home/windiff/Code/Robot-Parameter-Indentification-Simulation/config/experiment.yaml) 中的
`robot` 和 `backend` 字段，可以在仿真后端和 Piper 真机后端之间切换。切到 `piper`
且 `backend: sim` 时，会自动改用
[piper_force_controller_node.yaml](/home/windiff/Code/Robot-Parameter-Indentification-Simulation/config/piper_force_controller_node.yaml)
和 `piper` 对应的 MuJoCo 模型。

推荐的后端切换方式：

- `backend: sim`：使用 MuJoCo 仿真后端
- `backend: piper_real`：使用 Piper 真机后端

可选参数：

```bash
./build/run_experiment --headless --output data/my_run.csv
```

也可以显式指定实验配置：

```bash
./build/run_experiment --experiment-config config/experiment.yaml
```

### 5. 运行参数辨识

```bash
./build/identify
```

默认读取 [identification.yaml](/home/windiff/Code/Robot-Parameter-Indentification-Simulation/config/identification.yaml)。
其中 `robot` 字段可以在 `panda` 和 `piper` 之间切换，`identify` 会据此选择
对应的自由度、动力学基准和回归矩阵构造方式。`panda` 使用 7 轴链路，
`piper` 使用 6 轴链路。

可选参数：

```bash
./build/identify --config config/identification.yaml
./build/identify --robot piper
./build/identify --data-file data/benchmark_data.csv --algorithm 1
./build/identify --data-file data/benchmark_data.csv --algorithm 8
./build/mujoco_identify data/benchmark_data.csv
```

### 6. 运行 Piper 真机实验

真机实验与仿真实验共用同一个入口程序，只是后端不同：

```bash
./build/run_experiment --experiment-config config/experiment.yaml --headless
```

使用前请先确认：

- 已按 [piper_sdk/README(ZH).MD](/home/windiff/Code/Robot-Parameter-Indentification-Simulation/piper_sdk/README(ZH).MD) 准备 CAN 环境
- 已安装 `python-can`
- 已在 [experiment.yaml](/home/windiff/Code/Robot-Parameter-Indentification-Simulation/config/experiment.yaml) 中设置 `backend: piper_real`
- 已检查 [config/piper_real_experiment.yaml](/home/windiff/Code/Robot-Parameter-Indentification-Simulation/config/piper_real_experiment.yaml) 中的 `control_mode`、软限位和初始位

默认配置使用 `excitation_trajectory`，会通过 MIT 接口执行 Fourier 激励轨迹，并记录与仿真相同结构的 `time / q / qd / qdd / tau`。

在真正开始激励轨迹之前，bridge 会先执行“回安全初始位”流程：

- 若 `move_to_home_before_start: true`
- 且当前关节位置不在 `home_position_tolerance` 容差窗口内
- bridge 会先用 `home_speed_rate` 把机械臂移动到 `home_position`
- 只有进入容差窗口后，统一实验主循环才会开始执行激励轨迹

这里的“回零位”在工程上等价为“回到配置中的安全初始位 `home_position`”，不是重新写入电机零点标定。

可选模式：

- `excitation_trajectory`：真机参数辨识主模式，执行 Fourier 激励轨迹
- `mit_identification`：与激励轨迹相同的 MIT 接口实验模式，便于后续扩展不同控制策略
- `position_validation`：小幅关节位置轨迹验证
- `state_only`：只连接与采样，不下发运动命令

激励轨迹相关参数位于 [config/piper_real_experiment.yaml](/home/windiff/Code/Robot-Parameter-Indentification-Simulation/config/piper_real_experiment.yaml)：

- `trajectory_period`
- `trajectory_harmonics`
- `trajectory_coefficient_scale`
- `trajectory_seed`

这些参数决定真机实验使用的 Fourier 轨迹形状。当前实现会用固定随机种子生成可复现的轨迹系数，并自动满足 `t=0` 时的初始位置/零速度约束。

真机 bridge 还会额外检查：

- `joint_soft_limits`
- `max_command_velocity`
- `max_command_torque`
- `home_position_tolerance`

任一关节命令超过这些阈值时，bridge 会拒绝该步命令并触发急停，避免上层统一主循环把明显异常的控制量直接发送到真机。

---

## 项目结构

```text
├── franka_emika_panda/   # MuJoCo Panda 模型与资源
├── src/app/              # 单进程实验入口
├── src/sim_com_node/     # 纯 C++ MuJoCo 仿真器
├── src/force_node/       # 纯 C++ 控制器、轨迹与碰撞检查
├── src/identification/   # 离线参数辨识与诊断工具
├── config/               # 运行配置
└── doc/                  # 数学推导与实验说明
```

---

## 系统架构

当前分支采用统一实验调度架构，不再依赖 ROS 中间层。实验链路分为
“配置层 -> 控制层 -> 执行后端层 -> 数据层 -> 辨识层” 五个部分。

```text
config/*.yaml
    |
    v
run_experiment
    |
    +--> ForceController --------------------+
    |                                        |
    |                                        | 统一控制命令
    |                                        v
    +--> SimulationBackend / PiperHardwareBackend
             |
             | 关节状态 q / qd
             +---------------------------> ForceController
             |
             | 统一 CSV 记录
             v
        benchmark_data.csv
             |
             v
     identify / mujoco_identify
             |
             v
   results/identification.yaml
```

- `run_experiment` 是统一实验调度入口，负责读取配置、创建控制器、选择执行后端，并驱动唯一的一套时间循环。
- `ForceController` 负责激励轨迹生成、PD 跟踪、力矩限幅，并输出统一控制命令。
- `SimulationBackend` 与 `PiperHardwareBackend` 分别负责仿真步进和真机命令执行，但不改变上层实验流程。
- `identify` 与 `mujoco_identify` 负责离线辨识，读取实验 CSV，构造观测矩阵并输出参数结果。

如果按运行时序看，单步循环可以概括为：

1. 后端返回当前关节状态。
2. `ForceController` 根据当前时刻和关节状态计算统一控制命令。
3. 后端执行一步仿真或一步真机命令下发。
4. 共享记录器将 `q / qd / qdd / tau` 记录到 CSV，供后续辨识使用。

---

## 主要可执行文件

- `run_experiment`：运行单进程 MuJoCo 闭环实验并生成 CSV
- `identify`：读取 CSV，执行单算法或完整 benchmark
  支持 `NLS_FRICTION` 非线性摩擦联合辨识（`--algorithm 8`）
- `mujoco_identify`：快速执行一次 MuJoCo 回归辨识
- `dynamics_diagnostic`：对比动力学模型与记录数据
- `model_comparison`：对比不同动力学模型
- `regressor_test`：检查回归矩阵与 MuJoCo 动力学一致性
