# 动力学方程计算原理说明

本文档详细说明了仿真程序中机器人动力学方程的计算方法。相关的核心代码实现位于 `src/force_node/src/robot/robot_dynamics.cpp` 和 `src/identification/src/robot/robot_dynamics.cpp` 中。

## 1. 动力学方程形式

程序基于标准的刚体动力学方程进行计算：

$$
H(q)\ddot{q} + C(q, \dot{q})\dot{q} + g(q) + \tau_{fric}(\dot{q}) = \tau
$$

其中：
- $q, \dot{q}, \ddot{q}$ 分别为关节位置、速度和加速度向量
- $H(q)$ 为惯性矩阵 (Inertia Matrix)，代码中记为 `M`
- $C(q, \dot{q})$ 为科里奥利力与离心力矩阵 (Coriolis Matrix)，代码中记为 `C`
- $g(q)$ 为重力向量 (Gravity Vector)，代码中记为 `G`
- $\tau_{fric}(\dot{q})$ 为摩擦力矩向量 (Friction Torque)，代码中记为 `F`
- $\tau$ 为关节力矩向量

## 2. 各项计算方法

### 2.1 惯性矩阵 $H(q)$ (Inertia Matrix)

惯性矩阵是通过累加每个连杆在基座坐标系下的空间惯量计算得出的。

**计算步骤：**
1.  **计算空间惯量 (Spatial Inertia)**：
    对于每个连杆 $i$，首先根据其质量 $m$、质心位置 $c$ 和转动惯量 $I_{com}$ 计算其空间惯量 $M_{spatial}$。
    代码使用平行轴定理将质心的转动惯量变换到连杆原点：
    $$I_{origin} = I_{com} - m \cdot S(c)^2$$
    其中 $S(c)$ 是质心位置向量的反对称矩阵（skew-symmetric matrix）。

2.  **求和计算 $H(q)$**：
    遍历所有连杆，利用连杆的雅可比矩阵 $J_i$ 将空间惯量投影到关节空间并累加：
    $$H(q) = \sum_{i} J_i^T M_{spatial, i} J_i$$
    
    代码对应函数：`computeInertiaMatrix`

### 2.2 科里奥利矩阵 $C(q, \dot{q})$ (Coriolis Matrix)

程序使用**第一类克里斯托费尔符号 (Christoffel symbols of the first kind)** 来计算科里奥利矩阵。

**计算原理：**
$$C_{ij}(q, \dot{q}) = \sum_{k} \Gamma_{ijk} \dot{q}_k$$
其中克里斯托费尔符号 $\Gamma_{ijk}$ 定义为：
$$\Gamma_{ijk} = \frac{1}{2} \left( \frac{\partial H_{ij}}{\partial q_k} + \frac{\partial H_{ik}}{\partial q_j} - \frac{\partial H_{jk}}{\partial q_i} \right)$$

**数值微分实现：**
为了计算惯性矩阵 $H$ 对关节位置 $q$ 的偏导数 $\frac{\partial H}{\partial q}$，代码采用了**数值微分**的方法：
$$\frac{\partial H}{\partial q_k} \approx \frac{H(q + \epsilon e_k) - H(q - \epsilon e_k)}{2\epsilon}$$
其中 $\epsilon = 10^{-7}$ 是一个微小量。

代码对应函数：`computeCoriolisMatrix`

### 2.3 重力向量 $g(q)$ (Gravity Vector)

重力向量通过计算所有连杆重力势能产生的广义力得到。

**计算步骤：**
$$g(q) = - \sum_{i} J_{v, i}^T (m_i \cdot \mathbf{g}_{world})$$
其中：
- $J_{v, i}$ 是连杆 $i$ 质心的线速度雅可比矩阵
- $m_i$ 是连杆 $i$ 的质量
- $\mathbf{g}_{world}$ 是世界坐标系下的重力加速度向量

*注：代码中使用的是负号累减形式，即 `G -= ...`，因为通常动力学方程左边是 $+g(q)$，表示需要克服的重力矩。*

代码对应函数：`computeGravityVector`

### 2.4 摩擦力矩 $\tau_{fric}$ (Friction Torque)

程序采用**粘滞摩擦 + 库伦摩擦**模型：

$$F_i(\dot{q}_i) = F_{v, i} \dot{q}_i + F_{c, i} \cdot \text{sgn}(\dot{q}_i)$$

其中：
- $F_{v, i}$ 为粘滞摩擦系数
- $F_{c, i}$ 为库伦摩擦系数
- $\text{sgn}(\cdot)$ 为符号函数

代码对应函数：`computeFrictionTorque`

## 3. 正/逆动力学求解

### 3.1 逆动力学 (Inverse Dynamics)
即已知运动求力矩，直接代入方程计算：
$$\tau = H(q)\ddot{q} + C(q, \dot{q})\dot{q} + g(q) + \tau_{fric}$$

代码对应函数：`computeInverseDynamics`

### 3.2 正动力学 (Forward Dynamics)
即已知力矩求加速度，需要求解线性方程组：
$$H(q)\ddot{q} = \tau - C(q, \dot{q})\dot{q} - g(q) - \tau_{fric}$$

为了保证数值稳定性及效率，代码使用了 **Cholesky 分解 (LLT Decomposition)** 来求解 $H(q)x = b$，如果分解失败（例如矩阵非正定），则退化使用 LDLT 分解。

代码对应函数：`computeForwardDynamics`
