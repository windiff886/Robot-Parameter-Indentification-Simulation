# 机器人动力学参数辨识系统

本项目实现了基于 BIRDy 框架的机器人动力学参数辨识系统，包含 MuJoCo 仿真、ROS 2 控制节点和 C++ 动力学库。

> **声明**：本项目参考了 [BIRDy (Benchmark for Identification of Robot Dynamics)](https://github.com/TUM-ICS/BIRDY) 开源项目。

---

## 🚀 快速开始 (Quick Start)

### 1. 构建项目

```bash
colcon build
source install/setup.bash
```

### 2. 运行仿真流程

启动 MuJoCo 仿真器和控制器，开始采集数据：

```bash
ros2 launch src/launch/panda_sim_with_controller.launch.py
```

### 3. 运行参数辨识

使用采集到的数据进行离线辨识：

```bash
# 默认使用 OLS 算法
ros2 launch identification identification.launch.py data_file:=$(pwd)/data/<your_data_file.csv>

# 指定其他算法 (如抗噪声的 IRLS 或 在线估计的 EKF)
ros2 launch identification identification.launch.py data_file:=$(pwd)/data/<your_data_file.csv> algorithm:=IRLS
ros2 launch identification identification.launch.py data_file:=$(pwd)/data/<your_data_file.csv> algorithm:=EKF

```

---

## 📂 项目架构 (Project Architecture)

```
├── franka_emika_panda/          # MuJoCo 机器人模型 (MJCFxml 等)
├── BIRDy/                       # MATLAB 参考实现 (原始基准项目)
├── src/
│   ├── sim_com_node/            # MuJoCo 仿真通信节点
│   ├── force_node/              # C++ 核心动力学库 & 轨迹生成
│   │   ├── include/robot/       # 动力学计算 (RNEA, Coriolis, etc.)
│   │   └── src/                 # 实现代码
│   ├── identification/          # 参数辨识节点 (OLS)
│   └── launch/                  # ROS 2 启动脚本
├── doc/                         # 项目文档
└── config/                      # 配置文件
```

---

## 🧠 核心算法 (Core Algorithms)

本项目实现了完整的刚体动力学计算与参数辨识流程。

### 1. 动力学计算
实现了基于拉格朗日形式的动力学方程 ($M, C, G$ 矩阵计算) 与正/逆动力学求解。
原理详见 [动力学计算文档](doc/dynamics_calculation.md)。

### 2. 参数辨识
采用线性参数化方法 ($W \beta = \tau$)，支持 OLS, WLS, IRLS, EKF 等多种算法。
算法详见 [参数辨识文档](doc/parameter_identification.md)。

### 3. 激励轨迹
使用有限项 Fourier 级数生成周期性激励轨迹，并通过优化观测矩阵的条件数 (Condition Number) 来提高辨识的鲁棒性。

---

## 📚 文档索引 (Documentation)

*   [**工作模式说明 (Working Modes)**](doc/BIRDy_working_mode.md)
    *   详细介绍仿真模式与实验模式的流程与区别。
*   [**动力学计算 (Dynamics Calculation)**](doc/dynamics_calculation.md)
    *   深入解析 $M, C, G$ 矩阵的数学推导与代码实现。
*   [**参数辨识 (Parameter Identification)**](doc/parameter_identification.md)
    *   解释各种辨识算法 (OLS, EKF, ML, CLOE) 的原理与数学推导。
