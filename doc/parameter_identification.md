# 机械臂参数辨识：数学过程与代码流程（src/identification）

本文档依据 `src/identification` 现有实现总结参数辨识的数学过程与代码流程。核心路径为 `Identification` + `MuJoCoRegressor`，目标是保证回归矩阵与 MuJoCo 动力学一致。

## 1. 范围与入口

- 主类与接口: `src/identification/include/identification/identification.hpp`
- 回归矩阵: `src/identification/include/mujoco_regressor.hpp`
- 求解算法: `src/identification/include/identification/algorithms.hpp`
- 数据加载: `src/identification/include/identification/data_loader.hpp`
- 运行入口: `src/identification/src/main.cpp` (ROS2), `src/identification/src/mujoco_identify.cpp` (CLI)
- 验证/诊断: `src/identification/src/regressor_test.cpp`, `src/identification/src/model_comparison.cpp`, `src/identification/src/dynamics_diagnostic.cpp`

## 2. 数据与符号

- 关节数: $N_{dof} = 7$
- 单样本变量: $q, \dot{q}, \ddot{q} \in \mathbb{R}^7$, $\tau \in \mathbb{R}^7$
- 样本堆叠:
  - $W \in \mathbb{R}^{(7K) \times M}$
  - $\tau_{meas} \in \mathbb{R}^{7K}$
- CSV 格式 (`DataLoader::loadCSV`):
  - `time, q0..q6, qd0..qd6, [qdd0..qdd6], tau0..tau6`
  - 若 CSV 不含 `qdd`，则在预处理阶段用数值微分补全

## 3. 参数向量与动力学模型（MuJoCo 规范）

### 3.1 参数向量结构

MuJoCoRegressor 使用 MuJoCo 的 Body-Local 坐标系参数（定义在 body 原点）：

单个 Body 的惯性参数向量:

$$\theta_{body} = [m, mx, my, mz, I_{xx}, I_{xy}, I_{xz}, I_{yy}, I_{yz}, I_{zz}]^T$$

其中:

- $m$: 质量
- $(mx, my, mz)$: 一阶矩 $m \cdot c_{local}$
- $I_{**}$: body 原点处惯量张量分量

本实现包含:

- 8 个 body: link1-7 + hand (不含 link0 与手指)
- 额外参数: armature(7), damping(7)

参数数量:

- 仅惯性: $8 \times 10 = 80$
- 加 armature: $87$
- 加 armature + damping: $94$ (Identification 默认)

### 3.2 MuJoCo 形式的线性回归

MuJoCoPandaDynamics 中采用的动力学形式:

$$\tau = M(q) \ddot{q} + C(q, \dot{q}) \dot{q} + G(q) - D \dot{q}$$

为了线性化，本项目写成:

$$\tau = Y(q, \dot{q}, \ddot{q}) \theta + A \ddot{q} - D \dot{q}$$

其中:

- $\theta$: 8 个 body 的惯性参数向量
- $A = \mathrm{diag}(armature)$, $D = \mathrm{diag}(damping)$
- $Y$ 由 `MuJoCoRegressor` 构造

### 3.3 单 body 回归块推导（`computeBodyRegressorBlock`）

1) 运动学量（body 原点）:

- $J_{world} \in \mathbb{R}^{6\times7}$, $\dot{J}$ 使用 $q \pm dt \cdot \dot{q}$ 数值微分 ($dt=1e-7$)
- $a_{world} = J_v \ddot{q} + \dot{J}_v \dot{q}$
- $\omega_{world} = J_\omega \dot{q}$
- $\alpha_{world} = J_\omega \ddot{q} + \dot{J}_\omega \dot{q}$

2) 转到 body 局部坐标系:

- $R$: body 到 world 的旋转
- $a_{local} = R^T a_{world}$
- $\omega_{local} = R^T \omega_{world}$
- $\alpha_{local} = R^T \alpha_{world}$
- $g_{local} = R^T g_{world}$
- $b_{local} = a_{local} - g_{local}$

3) 线性化关系:

$$K = [\alpha]_{\times} + [\omega]_{\times}[\omega]_{\times}$$
$$f_{local} = m \cdot b_{local} + K \cdot mc_{local}$$
$$n_{local} = [mc_{local}]_{\times} b_{local} + I_{origin} \alpha_{local} + [\omega]_{\times} I_{origin} \omega_{local}$$
$$\tau = J_{v,local}^T f_{local} + J_{\omega,local}^T n_{local}$$

4) 回归矩阵列（与实现一致）:

设 $\omega_{local}=(o_x,o_y,o_z)$, $\alpha_{local}=(a_x,a_y,a_z)$, $e_i$ 为标准基向量。

- $m$: $J_v^T b_{local}$
- $mx,my,mz$: $J_v^T K e_i + J_\omega^T (e_i \times b_{local})$
- $I_{xx}$: $J_\omega^T [a_x, o_x o_z, -o_x o_y]^T$
- $I_{xy}$: $J_\omega^T [a_y - o_x o_z, a_x + o_y o_z, o_x^2 - o_y^2]^T$
- $I_{xz}$: $J_\omega^T [a_z + o_x o_y, o_z^2 - o_x^2, a_x - o_y o_z]^T$
- $I_{yy}$: $J_\omega^T [-o_y o_z, a_y, o_x o_y]^T$
- $I_{yz}$: $J_\omega^T [o_y^2 - o_z^2, a_z - o_x o_y, a_y + o_x o_z]^T$
- $I_{zz}$: $J_\omega^T [o_y o_z, -o_x o_z, a_z]^T$

5) 关节项:

- armature: $Y(i, offset+i) = \ddot{q}(i)$
- damping: $Y(i, offset+i) = -\dot{q}(i)$  (MuJoCo 阻尼为被动项)

### 3.4 观测矩阵堆叠

$$W = [Y(q_1); Y(q_2); \ldots; Y(q_K)]$$
$$\tau_{meas} = [\tau_1; \tau_2; \ldots; \tau_K]$$

注意: `MuJoCoRegressor::computeObservationMatrix` 接受 `Q/Qd/Qdd` 的列为样本，因此 `Identification::solve` 中传入 `q_valid.transpose()`。

## 4. 目标函数与求解算法

### 4.1 目标函数

基本目标:

$$\min_{\beta} \tfrac{1}{2} \| W \beta - \tau_{meas} \|^2$$

正则化 OLS:

$$\min \| W\beta - \tau \|^2 + \lambda \|\beta\|^2, \; \lambda=1e-6$$

### 4.2 算法实现要点 (`algorithms.cpp`)

- OLS: 正则化正规方程，LDLT 求解。
- WLS: 先 OLS，按关节残差方差加权重求解。
- IRLS: Huber 权重，$\sigma$ 由 MAD 估计，迭代至 `tol=1e-4`。
- TLS: 对 $[W \; | \; \tau]$ 做 SVD，取最小奇异向量。
- EKF/RLS: 逐行更新，$P=100I$, $R=0.1$。
- ML: Levenberg-Marquardt，残差 $r=W\beta-\tau$, $J=W$。
- CLOE: 需要动力学仿真与参数回写，目前退化为 OLS。

## 5. 代码流程（主路径）

### 5.1 数据加载

文件: `src/identification/src/data_loader.cpp`

- 自动检测是否包含 `qdd` 列
- 构建 `ExperimentData` (q/qd/qdd/tau/time)

### 5.2 预处理

文件: `src/identification/src/identification.cpp`

- 若 `qdd` 已存在且尺寸匹配，则直接使用
- 否则使用中心差分:
  $$\ddot{q}(i) = \frac{\dot{q}(i+1) - \dot{q}(i-1)}{time(i+1) - time(i-1)}$$
- 首尾样本用邻近行填充

### 5.3 构建 $W$ 与求解

`Identification::solve`:

- 过滤离群样本: $\max |\ddot{q}| < 10.0$
- 按样本堆叠 $\tau_{meas}$
- 用 `MuJoCoRegressor` 构建 $W$
- `createAlgorithm` 根据字符串创建算法; 若返回 `nullptr` (CLOE) 则回退 OLS

### 5.4 ROS2 入口流程 (`main.cpp`)

- 参数读取: `data_file`, `algorithm`, `output_file`
- MuJoCo 动力学验证: `MuJoCoPandaDynamics::computeInverseDynamics` 与记录 $\tau$ 对比
- 训练/验证划分 (80%/20%)
- 构建 $W_{val}$ 并评估列范数、条件数
- 对每个算法:
  - 训练集求解 $\beta$
  - 验证集计算 Torque RMSE/Max Error
- 可选: 保存 YAML 结果

### 5.5 CLI 与测试工具

- `mujoco_identify.cpp`: OLS 单次辨识，比较 Ground Truth 与扭矩 RMSE
- `regressor_test.cpp`: 验证 $Y\theta$ 与 `MuJoCoPandaDynamics` 一致性
- `model_comparison.cpp`, `dynamics_diagnostic.cpp`: 对比 DH 模型与 MuJoCo 模型差异

## 6. 关键注意点与限制

- 坐标系: 本辨识完全基于 MuJoCo Body-Local 坐标系，DH 模型与之不一致。
- 阻尼符号: 回归中使用 $-\dot{q}$，需与 MuJoCo 定义保持一致。
- 数值微分: $\dot{J}$ 与 $\ddot{q}$ 依赖差分，噪声会影响稳定性。
- 可辨识性: $W$ 某些列可能接近 0；主程序已做列范数与条件数诊断。
- 物理约束: 当前求解未加质量/惯量正定约束，可能出现非物理解。
