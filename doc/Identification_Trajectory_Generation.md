# 辨识激励轨迹生成 (Identification Trajectory Generation)

本文档详细说明了用于机器人动力学参数辨识的激励轨迹生成逻辑、数学原理及参数配置。

## 1. 原理概述

为了充分激发机器人的动力学特性（惯性、科里奥利力、重力、摩擦力），我们需要能够同时覆盖位置、速度和加速度空间的轨迹。
本系统采用 **有限项傅里叶级数 (Finite Fourier Series)** 作为激励轨迹模型。

### 1.1 数学模型

对于每一个关节 $i$，其轨迹 $q_i(t)$ 定义为：

$$ q_i(t) = q_{i,0} + \sum_{k=1}^{N} \left( \frac{a_{i,k}}{\omega_f k} \sin(\omega_f k t) - \frac{b_{i,k}}{\omega_f k} \cos(\omega_f k t) \right) $$

其中：
*   $q_{i,0}$: 关节 $i$ 的初始位置（Home Pose）。
*   $\omega_f$: 轨迹的 **基频 (Fundamental Frequency)**。
*   $N$: 谐波次数 (Harmonics)，本系统默认为 **5**。
*   $a_{i,k}, b_{i,k}$: 随机生成的傅里叶系数。

相对应的速度 $\dot{q}_i(t)$ 和加速度 $\ddot{q}_i(t)$ 为：

$$ \dot{q}_i(t) = \sum_{k=1}^{N} \left( a_{i,k} \cos(\omega_f k t) + b_{i,k} \sin(\omega_f k t) \right) $$

$$ \ddot{q}_i(t) = \sum_{k=1}^{N} \omega_f k \left( b_{i,k} \cos(\omega_f k t) - a_{i,k} \sin(\omega_f k t) \right) $$

### 1.2 频率与周期的关系

基频 $\omega_f$ 由给定的轨迹总时长（周期）$T$ 决定：

$$ \omega_f = \frac{2\pi}{T} $$

这也是为什么 **增加轨迹时长 $T$ 会使动作变慢** 的数学原因：
*   速度幅值 $\propto \omega_f \propto 1/T$
*   加速度/力矩幅值 $\propto \omega_f^2 \propto 1/T^2$

如果将周期从 10s 增加到 20s，基频减半，所需力矩理论上会降至原来的 1/4，有效避免电机饱和。

---

## 2. 初始条件约束 (Initial Conditions)

为了保证机器人能从当前静止状态平滑开始运动，生成的轨迹必须满足 $t=0$ 时的初始条件约束：

1.  **位置约束**：$q_i(0) = q_{i,0}$
    $$ \sum_{k=1}^{N} - \frac{b_{i,k}}{\omega_f k} = 0 $$
2.  **速度约束**：$\dot{q}_i(0) = 0$
    $$ \sum_{k=1}^{N} a_{i,k} = 0 $$
3.  **加速度约束**：$\ddot{q}_i(0) = 0$
    $$ \sum_{k=1}^{N} \omega_f k \cdot b_{i,k} = 0 $$

程序在随机生成系数 $a, b$ 后，会通过修正第一项系数 ($a_{i,1}, b_{i,1}$) 来强制满足上述约束。

---

## 3. 生成策略与安全机制

由于随机生成的轨迹极易导致碰撞或超出电机能力，系统采用 **"生成-检测-重试" (Generate-Check-Retry)** 的策略。

### 3.1 振幅 (Amplitude)
振幅参数控制了随机系数 $a_{i,k}, b_{i,k}$ 的取值范围：
$$ a, b \sim Uniform(-Amplitude, +Amplitude) $$

*   **Amplitude 越大**：动作越剧烈，覆盖空间越大，但越容易碰撞/饱和。
*   **Amplitude 越小**：动作越微小，越安全，但数据信噪比可能降低。

### 3.2 自动衰减重试逻辑 (Auto-Decay Retry)

为了在“充分激励”和“安全执行”之间寻找平衡，程序实现了**阶梯式衰减**逻辑：

1.  **初始尝试**：使用原始设定的 `Amplitude` (**0.2**)，以保证充分的激励空间。
2.  **安全检查**：调用 MuJoCo 物理引擎检查轨迹是否发生碰撞。
3.  **失败处理**：
    *   **耐心重试**：如果当前幅度下可以找到安全轨迹（只是运气不好随机到了坏的），程序会继续尝试，直到试满 **20次**。
    *   **阶梯降级**：如果连续 **20次** 都失败，程序判断当前幅度过大，执行降级：`Amplitude *= 0.8`。
    *   **下限保护**：直到 Amplitude 降至 `0.001` 为止。

这种机制既能尽可能保留大动作（高信噪比），又能在确实行不通时自动退让，确保系统鲁棒性。

### 3.3 代码实现
相关逻辑位于 `src/force_node/src/force_controller_node.cpp` 中的 `init_excitation_trajectory` 函数。

```cpp
// 伪代码逻辑
double amplitude = 0.2;
for (int attempt = 0; attempt < 200; ++attempt) {
    trajectory.setRandomCoefficients(amplitude);
    
    if (check_trajectory(trajectory)) {
        return trajectory; // 成功
    }
    
    // 每 20 次失败才降级一次
    if ((attempt + 1) % 20 == 0 && amplitude > 0.001) {
        amplitude *= 0.8; 
    }
}
```
