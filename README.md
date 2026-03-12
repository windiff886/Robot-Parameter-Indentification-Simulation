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

### 4. 运行仿真实验

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
`robot` 字段，可以在 `panda` 和 `piper` 之间切换。切到 `piper` 时会自动改用
[piper_force_controller_node.yaml](/home/windiff/Code/Robot-Parameter-Indentification-Simulation/config/piper_force_controller_node.yaml)
和 `piper` 对应的 MuJoCo 模型。

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

当前分支采用单进程闭环架构，不再依赖 ROS 中间层。实验链路分为
“配置层 -> 控制层 -> 仿真层 -> 数据层 -> 辨识层” 五个部分。

```text
config/*.yaml
    |
    v
run_experiment
    |
    +--> ForceController --------------------+
    |                                        |
    |                                        | 关节力矩
    |                                        v
    +--> PandaSimulator <--------------------+
             |
             | 关节状态 q / qd / tau
             +-------------------------> ForceController
             |
             | CSV 采样数据
             v
        data/benchmark_data.csv
             |
             v
     identify / mujoco_identify
             |
             v
   results/identification.yaml
```

- `run_experiment` 是实验调度入口，负责读取实验配置、选择机械臂、创建控制器与仿真器，并驱动整个时间循环。
- `ForceController` 负责激励轨迹生成、PD 跟踪、力矩限幅，以及基于关节限位和 MuJoCo 碰撞检测的安全检查。
- `PandaSimulator` 负责 MuJoCo 模型加载、物理步进、可视化窗口和 CSV 记录。
- `identify` 与 `mujoco_identify` 负责离线辨识，读取实验 CSV，构造观测矩阵并输出参数结果。

如果按运行时序看，单步循环可以概括为：

1. `PandaSimulator` 读取当前关节状态。
2. `ForceController` 根据当前时刻和关节状态计算控制力矩。
3. `PandaSimulator` 将力矩写入 `data->ctrl` 并执行一次 `mj_step`。
4. 仿真器将 `q / qd / qdd / tau` 记录到 CSV，供后续辨识使用。

---

## 主要可执行文件

- `run_experiment`：运行单进程 MuJoCo 闭环实验并生成 CSV
- `identify`：读取 CSV，执行单算法或完整 benchmark
  支持 `NLS_FRICTION` 非线性摩擦联合辨识（`--algorithm 8`）
- `mujoco_identify`：快速执行一次 MuJoCo 回归辨识
- `dynamics_diagnostic`：对比动力学模型与记录数据
- `model_comparison`：对比不同动力学模型
- `regressor_test`：检查回归矩阵与 MuJoCo 动力学一致性
