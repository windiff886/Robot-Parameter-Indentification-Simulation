# 轨迹安全监测机制 (Trajectory Safety Mechanism)

## 1. 概述

为了确保参数辨识过程中生成的激励轨迹安全可靠，系统在 `ForceControllerNode` 中引入了基于 **MuJoCo 物理引擎** 的高保真碰撞检测机制。

该机制利用 MuJoCo 强大的物理仿真能力，在轨迹生成后、执行前，对整条轨迹进行全面的预检查。只有通过所有安全检查的轨迹才会被允许执行，防止机器人在实验过程中发生自碰撞或与环境（如地面）发生碰撞。

## 2. 检查原理

检查函数 `check_trajectory` 会以 **0.1秒** 为步长，遍历整个轨迹周期（默认10秒），对每个时刻的机器人状态进行以下两层检查：

### 2.1 关节限位检查 (Joint Limits Check)

首先检查每一个关节的角度是否在物理允许范围内：
$$ q_{min, i} \leq q_i(t) \leq q_{max, i} $$

- **依据**：`robot_model_->jointLimits()`
- **作用**：防止机械臂运动超出自身的机械结构硬限位。

### 2.2 高保真碰撞检查 (High-Fidelity Collision Check)

系统使用 MuJoCo 引擎直接加载机器人的 XML 模型 (`panda.xml`) 进行碰撞检测。
对于每一时刻的关节配置 $q(t)$：

1.  **状态更新**：将 $q(t)$ 设置到 MuJoCo 的 `mjData` 中。
2.  **正运动学计算**：调用 `mj_fwdPosition` 计算所有几何体的空间位置。
3.  **碰撞检测**：MuJoCo 引擎自动计算场景中所有几何体对（Geom Pairs）之间的接触。
    *   **自碰撞 (Self-Collision)**：检测机器人各连杆之间的接触。
    *   **环境碰撞 (Environment Collision)**：检测机器人与地面或其他环境物体的接触。

- **判据**：
   如果 `mjData->ncon` (接触点数量) > 0，则判定为发生碰撞。

- **优势**：
    *   相比于简化的胶囊体模型，MuJoCo 使用原始的 Mesh 或精确凸包，检测精度更高。
    *   能够自动处理复杂的几何形状和接触情况。

## 3. 系统配置

安全监测系统主要依赖以下组件：

*   **MujocoCollisionChecker**: 封装了 MuJoCo 上下文 (`mjModel`, `mjData`) 的管理和碰撞检测接口。
*   **Robot Model**: `panda.xml` (位于 `sim_com_node` 包中)，定义了机器人的运动学树、惯性参数和碰撞几何体。

## 4. 重试策略 (Retry Logic)

`init_excitation_trajectory` 函数采用“生成-验证-重试”的逻辑：

1.  **随机生成**：使用随机 Fourier 系数生成轨迹。
2.  **安全验证**：调用 `check_trajectory` 进行上述检查。
    *   **通过**：立即采用该轨迹。
    *   **失败**：丢弃轨迹，进入下一次尝试。
3.  **参数调整**：
    *   最大尝试次数：**50次**。
    *   如果连续 **20次** 失败，系统会自动缩小随机幅度（Amplitude *= 0.9），以降低轨迹剧烈程度，提高成功率。

## 5. 相关代码

*   **碰撞检测器**: `src/force_node/include/robot/mujoco_collision_checker.hpp`
*   **控制器实现**: `src/force_node/src/force_controller_node.cpp`
*   **机器人模型**: `src/sim_com_node/franka_emika_panda/panda.xml`
