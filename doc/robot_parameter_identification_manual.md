# 机器人动力学参数辨识系统统一说明

> 本文档用于统一整理 `doc/` 目录下与动力学建模、激励轨迹、数据采样、
> 参数辨识、算法评估和调试经验相关的内容。阅读本文即可理解当前仓库的
> 数学模型、工程链路和实现边界。旧文档建议保留为归档材料，后续以本文
> 作为主说明文档维护。

本文重点整合了以下主题：

- 动力学方程与 MuJoCo 力矩输入含义
- 激励轨迹生成与安全检查
- 线性回归辨识与非线性摩擦辨识
- 仿真模式 / 实验模式评估方法
- 常见数值问题、调试经验与工程限制

---

## 1. 文档目标与项目范围

### 1.1 项目目标

本项目是一个基于 **MuJoCo + 纯 C++ + CMake** 的机器人动力学参数辨识系统。
其目标不是单独做轨迹跟踪，也不是单独做仿真，而是完成如下闭环：

1. 生成安全且具有充分激励性的关节轨迹；
2. 在 MuJoCo 中执行轨迹并记录关节数据；
3. 基于记录数据构造动力学回归矩阵；
4. 采用多种辨识算法估计动力学参数；
5. 在独立验证集上评估力矩预测误差；
6. 分析参数可解释性、数值稳定性与模型局限。

### 1.2 当前支持对象

当前仓库支持两类机器人模型：

- `panda`：7 自由度 Franka Panda
- `piper`：6 自由度 Piper 机械臂

两者共用同一套实验链路，但在以下方面不同：

- 自由度数量不同；
- MuJoCo XML 模型不同；
- 刚体参数数量不同；
- 回归矩阵构造器不同。

### 1.3 系统总流程

当前分支的核心数据流如下：

```text
config/*.yaml
    |
    v
run_experiment
    |
    +--> ForceController
    |        |
    |        +--> 生成激励轨迹、执行安全检查、输出关节力矩
    |
    +--> PandaSimulator / Piper Simulator
             |
             +--> MuJoCo 步进
             +--> 记录 q, qd, qdd, tau
             v
        data/benchmark_data.csv
             |
             v
          identify
             |
             +--> 预处理
             +--> 构造观测矩阵 W
             +--> 求解参数
             +--> 验证集评估
             v
      results/identification.yaml
```

从职责划分上看，系统可以拆成五层：

1. **配置层**：实验配置、辨识配置、机器人模型路径；
2. **轨迹层**：激励轨迹的生成与安全验证；
3. **仿真层**：MuJoCo 中的状态推进与数据记录；
4. **辨识层**：观测矩阵构造、参数求解、残差评估；
5. **诊断层**：调试报告、模型比较、回归一致性检查。

---

## 2. 记号、坐标系与数据格式

### 2.1 基本记号

设机器人自由度为 $n$，样本数为 $K$。对任一时刻 $t_k$，定义：

- $q(t_k) \in \mathbb{R}^n$：关节位置
- $\dot{q}(t_k) \in \mathbb{R}^n$：关节速度
- $\ddot{q}(t_k) \in \mathbb{R}^n$：关节加速度
- $\tau(t_k) \in \mathbb{R}^n$：关节力矩

离线辨识时，将样本按时间堆叠为矩阵：

$$
Q =
\begin{bmatrix}
q(t_1) & q(t_2) & \cdots & q(t_K)
\end{bmatrix}
\in \mathbb{R}^{n \times K}
$$

$$
\dot{Q}, \ddot{Q} \in \mathbb{R}^{n \times K}
$$

并将力矩向量按关节维展开为：

$$
\tau_{meas} =
\begin{bmatrix}
\tau(t_1) \\
\tau(t_2) \\
\vdots \\
\tau(t_K)
\end{bmatrix}
\in \mathbb{R}^{nK}
$$

### 2.2 坐标系与参数定义

本项目的动力学参数不是基于传统 DH 质心坐标定义，而是基于
**MuJoCo body-local 坐标系** 定义。对每个刚体 `body`，使用如下
标准惯性参数向量：

$$
\theta_{body} =
[m, mx, my, mz, I_{xx}, I_{xy}, I_{xz}, I_{yy}, I_{yz}, I_{zz}]^T
$$

其中：

- $m$：刚体质量
- $(mx,my,mz)=m \cdot c_{local}$：一阶矩
- $c_{local}$：刚体质心在 body-local 坐标系中的位置
- $I_{**}$：在 body 原点处定义的惯量张量分量

这一定义与“以质心为参考点”的惯量写法不同，因此在做参数对比时必须先
统一参考点和坐标系，否则会出现“数值看似不同但物理上可等价”的情况。

### 2.3 数据文件格式

实验记录文件采用 CSV 格式，典型列为：

```text
time, q0..q(n-1), qd0..qd(n-1), [qdd0..qdd(n-1)], tau0..tau(n-1)
```

当前仿真器会直接记录：

- `qpos`
- `qvel`
- `qacc`
- `data->ctrl`

因此当前项目里 `tau` 的含义是**写入执行器控制输入的力矩命令**，而不是
MuJoCo 内部某个理想分解项的直接导出。

---

## 3. MuJoCo 中的动力学方程与控制输入含义

### 3.1 MuJoCo 的广义坐标动力学

MuJoCo 在广义坐标下求解动力学方程，其基本形式为：

$$
M(q)\ddot{q} + qfrc_{bias}(q,\dot{q})
= qfrc_{actuator} + qfrc_{passive} + qfrc_{applied} + qfrc_{constraint}
$$

其中：

- $M(q)$：广义质量矩阵
- $qfrc_{bias}$：偏置力，通常包含科里奥利、离心和重力项
- $qfrc_{actuator}$：执行器施加的广义力
- $qfrc_{passive}$：被动项，如阻尼、弹簧、摩擦
- $qfrc_{constraint}$：接触、关节限位和 equality 约束产生的约束力

若忽略外部扰动和约束力，并将偏置项展开，则可写成熟悉的开链形式：

$$
M(q)\ddot{q} + C(q,\dot{q})\dot{q} + g(q)
= \tau_{act} + \tau_{passive}
$$

### 3.2 控制输入 `ctrl` 与广义力的关系

仿真程序向 MuJoCo 写入的是 `data->ctrl`。对关节型 `motor` 执行器而言，
这通常通过执行器映射变为关节广义力。若 gear 为单位阵、无额外动态环节，
则可近似认为：

$$
\tau_{act} \approx \tau_{ctrl}
$$

但严格来说，两者仍可能因以下因素出现差异：

- 执行器限幅；
- 执行器 gear 或 transmission；
- tendon 映射；
- 被动项与约束项；
- 隐式积分器的内部数值处理。

因此，“模型已知”并不自动等价于“记录到的 `tau` 一定等于理想逆动力学”。

### 3.3 当前项目采用的简化动力学表达

结合当前辨识实现，系统使用如下简化模型：

$$
\tau = M(q)\ddot{q} + C(q,\dot{q})\dot{q} + g(q) - D\dot{q}
$$

其中：

- $D = \mathrm{diag}(d_1,\dots,d_n)$ 为关节阻尼矩阵；
- `armature` 直接加到惯性矩阵对角线上；
- 线性 damping 被显式写入辨识模型；
- 非线性摩擦模型仅在 `NLS_FRICTION` 中额外引入。

对当前仓库而言，这个式子是“用于辨识的工程模型”，它尽量贴近 MuJoCo，
但仍然是一个经过抽象与裁剪后的版本。

### 3.4 约束与碰撞为何重要

若轨迹运行过程中出现：

- 自碰撞；
- 与环境碰撞；
- 关节限位触发；
- equality 约束强力介入；

则 `qfrc_constraint` 会显著改变系统真实受力。此时再用

$$
\tau \approx M(q)\ddot{q} + C(q,\dot{q})\dot{q} + g(q) - D\dot{q}
$$

去拟合数据，就会把约束力误吸收到参数里，导致结果失真。这就是为什么
轨迹安全检查不是附属功能，而是辨识链路的一部分。

---

## 4. 参数向量与回归矩阵

### 4.1 线性参数向量

对每个刚体使用 10 维标准惯性参数，则总参数由三部分组成：

1. 刚体惯性参数
2. `armature` 参数
3. `damping` 参数

若有 $N_b$ 个参与辨识的刚体，则线性参数向量为：

$$
\beta_{lin} =
\begin{bmatrix}
\theta_{body,1} \\
\theta_{body,2} \\
\vdots \\
\theta_{body,N_b} \\
a_1,\dots,a_n \\
d_1,\dots,d_n
\end{bmatrix}
$$

其中：

- `piper`：$N_b = 6$，所以线性参数总数为 $6 \times 10 + 6 + 6 = 72$
- `panda`：$N_b = 8$，所以线性参数总数为 $8 \times 10 + 7 + 7 = 94$

### 4.2 线性回归模型

在 MuJoCo body-local 参数定义下，关节力矩可以线性写成：

$$
\tau =
Y(q,\dot{q},\ddot{q})\theta
+ A\ddot{q}
- D\dot{q}
$$

进一步合并可得：

$$
\tau = W(q,\dot{q},\ddot{q}) \, \beta_{lin}
$$

其中：

- $\theta$：所有刚体的惯性参数拼接
- $A=\mathrm{diag}(a_1,\dots,a_n)$：armature
- $D=\mathrm{diag}(d_1,\dots,d_n)$：damping
- $W$：完整观测矩阵

对于单个样本 $k$，有：

$$
\tau^{(k)} = W^{(k)} \beta_{lin}
$$

将全部样本堆叠后得到：

$$
\underbrace{
\begin{bmatrix}
\tau^{(1)} \\
\tau^{(2)} \\
\vdots \\
\tau^{(K)}
\end{bmatrix}
}_{\tau_{meas}}

=

\underbrace{
\begin{bmatrix}
W^{(1)} \\
W^{(2)} \\
\vdots \\
W^{(K)}
\end{bmatrix}
}_{W}
\beta_{lin}
$$

### 4.3 单刚体回归块的数学推导

对某一刚体 `body`，记：

- $R$：body-local 到 world 的旋转矩阵
- $J_v, J_\omega$：body 原点的线速度与角速度雅可比
- $\dot{J}_v, \dot{J}_\omega$：对应时间导数

先在 world 坐标系下计算：

$$
a_{world} = J_v \ddot{q} + \dot{J}_v \dot{q}
$$

$$
\omega_{world} = J_\omega \dot{q}
$$

$$
\alpha_{world} = J_\omega \ddot{q} + \dot{J}_\omega \dot{q}
$$

再变换到 body-local 坐标系：

$$
a_{local} = R^T a_{world}, \quad
\omega_{local} = R^T \omega_{world}, \quad
\alpha_{local} = R^T \alpha_{world}
$$

设重力在局部坐标系表示为：

$$
g_{local} = R^T g_{world}
$$

并定义：

$$
b_{local} = a_{local} - g_{local}
$$

刚体局部受力与力矩满足：

$$
f_{local} = m b_{local} + K \, mc_{local}
$$

$$
n_{local}
= [mc_{local}]_\times b_{local}
+ I_{origin}\alpha_{local}
+ [\omega_{local}]_\times I_{origin}\omega_{local}
$$

其中：

$$
K = [\alpha_{local}]_\times + [\omega_{local}]_\times [\omega_{local}]_\times
$$

最终该刚体对关节力矩的贡献为：

$$
\tau_{body} = J_{v,local}^T f_{local} + J_{\omega,local}^T n_{local}
$$

因为上式对

$$
[m, mx, my, mz, I_{xx}, I_{xy}, I_{xz}, I_{yy}, I_{yz}, I_{zz}]
$$

是线性的，所以可以构造单刚体回归块：

$$
Y_{body}(q,\dot{q},\ddot{q}) \in \mathbb{R}^{n \times 10}
$$

全部刚体的回归块按列拼接后即得到惯性部分回归矩阵。

### 4.4 `armature` 与 `damping` 的回归列

对每个关节 $i$：

- armature 项对应 $\ddot{q}_i$
- damping 项对应 $-\dot{q}_i$

因此附加列可直接写为：

$$
Y_{armature}(i,i) = \ddot{q}_i
$$

$$
Y_{damping}(i,i) = -\dot{q}_i
$$

### 4.5 可辨识性与秩亏

即使动力学方程本身是“正确”的，也不代表所有参数都可辨识。若存在：

- 某些刚体几何对当前轨迹不敏感；
- 某些惯量分量总是成耦合出现；
- 某些列几乎线性相关；

则观测矩阵 $W$ 将出现零空间或近零空间。此时不同参数向量可能给出几乎相同
的力矩预测，这正是“力矩拟合很好，但参数不物理”的根本原因。

---

## 5. 激励轨迹设计

### 5.1 有限项 Fourier 轨迹

为了同时激励位置、速度和加速度通道，系统对每个关节采用有限项 Fourier
级数作为激励轨迹：

$$
q_i(t) = q_{i,0} +
\sum_{k=1}^{N}
\left(
\frac{a_{i,k}}{\omega_f k}\sin(\omega_f k t)
- \frac{b_{i,k}}{\omega_f k}\cos(\omega_f k t)
\right)
$$

对应速度和加速度分别为：

$$
\dot{q}_i(t) =
\sum_{k=1}^{N}
\left(
a_{i,k}\cos(\omega_f k t)
+ b_{i,k}\sin(\omega_f k t)
\right)
$$

$$
\ddot{q}_i(t) =
\sum_{k=1}^{N}
\omega_f k
\left(
b_{i,k}\cos(\omega_f k t)
- a_{i,k}\sin(\omega_f k t)
\right)
$$

其中：

- $q_{i,0}$：关节初始位置
- $N$：谐波数
- $\omega_f = \frac{2\pi}{T}$：基频
- $T$：轨迹周期

### 5.2 周期与力矩幅值关系

基频满足：

$$
\omega_f = \frac{2\pi}{T}
$$

因此：

- 速度幅值大致与 $\omega_f$ 成正比；
- 加速度幅值大致与 $\omega_f^2$ 成正比；
- 在惯性主导阶段，所需驱动力矩也近似与 $\omega_f^2$ 成正比。

所以延长周期 $T$ 会显著降低峰值加速度和力矩需求，这既有利于避免电机饱和，
也可能降低激励强度，需要在“安全性”和“辨识可观测性”之间折中。

### 5.3 初始条件约束

为了保证机器人从静止状态平滑起动，轨迹通常要求：

$$
q_i(0) = q_{i,0}, \quad \dot{q}_i(0)=0, \quad \ddot{q}_i(0)=0
$$

将 $t=0$ 代入 Fourier 轨迹可得约束：

$$
\sum_{k=1}^{N} -\frac{b_{i,k}}{\omega_f k} = 0
$$

$$
\sum_{k=1}^{N} a_{i,k} = 0
$$

$$
\sum_{k=1}^{N} \omega_f k \, b_{i,k} = 0
$$

这保证了轨迹不会在实验开始时产生不连续的速度或加速度跳变。

### 5.4 轨迹质量与条件数

激励轨迹设计的目标不是“看起来动作大”，而是让观测矩阵 $W$ 的列尽量独立。
通常使用条件数衡量：

$$
\mathrm{cond}(W) = \frac{\sigma_{max}(W)}{\sigma_{min}(W)}
$$

条件数越小，参数估计对噪声越不敏感；条件数越大，某些参数会强耦合，
导致微小噪声放大成巨大的参数误差。

---

## 6. 轨迹安全机制

### 6.1 生成-检测-重试策略

随机 Fourier 系数往往会生成不可执行轨迹，因此系统采用：

1. 随机生成轨迹；
2. 执行安全检查；
3. 若失败则重试；
4. 若多次失败则自动缩小幅度；
5. 直到找到安全轨迹或达到上限。

这是一种典型的 **Generate-Check-Retry** 策略。

### 6.2 关节限位检查

对每个关节和每个采样时刻，要求：

$$
q_{min,i} \le q_i(t) \le q_{max,i}
$$

该条件用于避免轨迹在几何上超出关节硬限位。

### 6.3 碰撞检查

系统在 MuJoCo 中对候选轨迹进行离线碰撞预测。给定某个时刻的关节配置
$q(t)$，流程为：

1. 将 $q(t)$ 写入 `mjData`；
2. 调用 `mj_fwdPosition` 更新几何体位姿；
3. 读取碰撞结果和接触数量；
4. 若接触点数 `ncon > 0`，则判定轨迹不安全。

这种方式比手工构造简化胶囊体模型更高保真，因为它直接复用了 MuJoCo
的几何表示、接触生成和约束判定机制。

### 6.4 安全检查为何直接影响辨识质量

若一条轨迹会碰撞，但仍被拿去做数据采集，则实际记录到的扭矩中会混入：

- 约束反作用力；
- 接触冲量；
- 限位项；
- 饱和引起的控制失真。

这些量不在辨识模型中，因此会显著污染 $W\beta=\tau$ 的假设。

---

## 7. 数据采样与预处理

### 7.1 采样内容

仿真器默认记录以下信号：

- 时间 `time`
- 关节位置 `q`
- 关节速度 `qd`
- 关节加速度 `qdd`
- 控制输入 `tau`

这使得当前分支比真实机器人实验更有利于辨识，因为仿真环境能够直接给出
`qacc`，不必完全依赖数值微分。

### 7.2 若加速度缺失时的补全

若 CSV 中不含 `qdd`，则使用中心差分估计：

$$
\ddot{q}(t_i) \approx
\frac{\dot{q}(t_{i+1}) - \dot{q}(t_{i-1})}{t_{i+1} - t_{i-1}}
$$

边界点通常使用邻近点复制或单边差分近似。

### 7.3 离群样本过滤

为抑制速度或位置抖动经差分放大后的假加速度，系统对样本施加阈值过滤：

$$
\|\ddot{q}(t_i)\|_\infty < \ddot{q}_{max}
$$

当前实现中常用：

$$
\ddot{q}_{max} = 10.0 \text{ rad/s}^2
$$

过滤的目的不是“让数据更好看”，而是避免少量异常点主导最小二乘目标函数。

### 7.4 训练集与验证集

当前主程序通常按时间顺序划分：

- 训练集：前 80%
- 验证集：后 20%

训练集用于估计参数，验证集用于评估泛化误差。若直接在同一批数据上报告误差，
则很容易高估模型质量。

---

## 8. 线性参数辨识问题

### 8.1 目标函数

在线性模型下，基本辨识问题可写为：

$$
\min_{\beta_{lin}} \frac{1}{2}\|W\beta_{lin}-\tau_{meas}\|_2^2
$$

若考虑正则化，则得到：

$$
\min_{\beta_{lin}}
\|W\beta_{lin}-\tau_{meas}\|_2^2
+ \lambda \|\beta_{lin}\|_2^2
$$

其中 $\lambda > 0$ 是 Tikhonov 正则化系数。

### 8.2 为什么必须正则化

当 $W$ 接近秩亏时，正规方程

$$
(W^T W)\beta = W^T \tau
$$

会出现严重病态。此时即使残差很小，参数也可能在零空间方向上大幅漂移。
引入正则项后，求解变为：

$$
(W^T W + \lambda I)\beta = W^T \tau
$$

它不会从根本上消除不可辨识性，但能抑制数值爆炸。

### 8.3 当前实现中的主要算法

#### OLS

普通最小二乘在当前实现中使用了 ridge 正则化：

$$
\hat{\beta}_{OLS}
=
(W^T W + \lambda I)^{-1}W^T\tau
$$

优点是简单、快速；缺点是对离群点敏感。

#### WLS

加权最小二乘先用 OLS 估计残差，再按关节方差设置权重：

$$
\hat{\beta}_{WLS}
=
\arg\min_\beta
\| \Sigma^{-1/2}(W\beta-\tau)\|_2^2
$$

若噪声主要表现为“不同关节噪声水平不同”，WLS 有帮助；但若问题是少量尖峰
样本，WLS 未必优于鲁棒回归。

#### IRLS

IRLS 通过迭代重加权实现鲁棒回归。当前实现采用 Huber 权重：

$$
w_i =
\begin{cases}
1, & |r_i| \le \delta \\
\frac{\delta}{|r_i|}, & |r_i| > \delta
\end{cases}
$$

其中：

- $r_i$：当前残差
- $\delta$：由 MAD 估计的鲁棒阈值

每次迭代求解：

$$
\hat{\beta}^{(k+1)}
=
\arg\min_\beta
\|W_k^{1/2}(W\beta-\tau)\|_2^2 + \lambda\|\beta\|_2^2
$$

当数据中只存在少量异常样本时，IRLS 往往明显优于 OLS/WLS。

#### TLS

总最小二乘希望同时考虑 $W$ 和 $\tau$ 的误差，典型做法是对增广矩阵
$[W \; | \; \tau]$ 做 SVD。它适合“观测矩阵本身也有测量噪声”的场景，但在
病态问题里仍可能给出物理上不稳定的解。

#### EKF / RLS

把参数当作静态状态，逐行递推更新：

$$
\beta_{k+1} = \beta_k + K_k(y_k - H_k\beta_k)
$$

这类方法适合在线辨识，但对先验协方差、测量噪声假设和样本顺序更敏感。

#### ML

当前实现中的 `ML` 实质上仍在最小化线性残差：

$$
\min_\beta \|W\beta-\tau\|_2^2
$$

只是求解器换成了 Levenberg-Marquardt，因此它并不会凭空增加可辨识信息。

#### CLOE

闭环输出误差法理论上需要把参数写回动力学模型，在闭环仿真中直接最小化
输出误差。但当前仓库尚未完整实现该流程，因此它在实际 benchmark 中并非
完整的 CLOE。

---

## 9. 非线性摩擦辨识

### 9.1 为什么需要非线性摩擦模型

线性 damping 只能表达：

$$
\tau_f = D\dot{q}
$$

它无法刻画：

- 静摩擦
- Coulomb 摩擦平台
- Stribeck 效应
- 零速附近的平滑过渡

因此仓库额外引入了非线性摩擦扩展。

### 9.2 单关节摩擦模型

对第 $j$ 个关节，摩擦力矩定义为：

$$
\tau_{f,j}(\dot{q}_j)
=
g_{1,j}\left(\tanh(g_{2,j}\dot{q}_j)-\tanh(g_{3,j}\dot{q}_j)\right)
+ g_{4,j}\tanh(g_{5,j}\dot{q}_j)
+ g_{6,j}\dot{q}_j
$$

其中：

- $g_1$：Stribeck 项幅值
- $g_2,g_3$：Stribeck 形状参数
- $g_4$：Coulomb 平台幅值
- $g_5$：Coulomb 平滑过渡系数
- $g_6$：黏性摩擦系数

### 9.3 总参数向量

若把非线性摩擦也纳入辨识，总参数向量可写为：

$$
\Theta =
\begin{bmatrix}
\beta_{rigid+armature} \\
\theta_{f,1} \\
\theta_{f,2} \\
\vdots \\
\theta_{f,n}
\end{bmatrix}
$$

其中每个关节的摩擦子向量为：

$$
\theta_{f,j} = [g_{1,j}, g_{2,j}, g_{3,j}, g_{4,j}, g_{5,j}, g_{6,j}]^T
$$

注意：在当前实现里，`NLS_FRICTION` 使用的是
“刚体参数 + armature + 非线性摩擦”，而不是
“刚体参数 + armature + 线性 damping + 非线性摩擦”。

### 9.4 非线性最小二乘目标

此时力矩预测写为：

$$
\hat{\tau}(q,\dot{q},\ddot{q};\Theta)
=
W_{base}(q,\dot{q},\ddot{q})\beta_{base}
- \tau_f(\dot{q};\theta_f)
$$

对应优化问题为：

$$
\hat{\Theta}
=
\arg\min_{\Theta}
\sum_{k=1}^{K}
\left\|
\tau^{(k)}_{meas}
- \hat{\tau}(q^{(k)},\dot{q}^{(k)},\ddot{q}^{(k)};\Theta)
\right\|_2^2
$$

### 9.5 参数正值约束与重参数化

由于若干摩擦参数需要保持正值，当前实现采用指数映射：

$$
g_i = \exp(z_i)
$$

这样优化变量变成无约束的 $z_i$，而真实参数始终满足：

$$
g_i > 0
$$

### 9.6 初始化与 LM 求解

当前非线性辨识采用：

- 先用线性回归得到刚体参数和 armature 初值；
- 用线性残差估计摩擦平台与黏性项；
- 使用多组缩放因子构造 multi-start 初值；
- 再用 Levenberg-Marquardt 做非线性最小二乘。

这种方法的优势在于：

- 充分利用线性问题的可解性；
- 减少直接从随机初值进入局部最优的概率；
- 将复杂问题拆成“先线性、后非线性”的两阶段流程。

---

## 10. 结果评估方法

### 10.1 仿真模式评估

若系统处于理想仿真模式，且真实参数 $\beta_{true}$ 已知，则可以直接比较：

#### 绝对误差

$$
E_{abs,i} = |\beta_{est,i} - \beta_{true,i}|
$$

#### 相对误差

$$
E_{rel,i} =
\frac{|\beta_{est,i} - \beta_{true,i}|}{|\beta_{true,i}|}
\times 100\%
$$

#### 参数范数误差

$$
E_{norm} = \|\beta_{est} - \beta_{true}\|_2
$$

### 10.2 实验模式评估

当真实参数未知时，只能做交叉验证。给定验证集：

$$
\tau_{pred} = W_{val}\beta_{est}
$$

残差为：

$$
r = \tau_{meas,val} - \tau_{pred}
$$

最常用的指标是力矩 RMSE：

$$
RMSE = \sqrt{\frac{1}{N}\sum_{i=1}^{N} r_i^2}
$$

以及最大绝对误差：

$$
E_{max} = \max_i |r_i|
$$

### 10.3 为什么低 RMSE 不等于参数正确

必须强调以下事实：

$$
\text{低扭矩残差} \not \Rightarrow \text{参数真实}
$$

原因包括：

1. $W$ 可能秩亏；
2. 多组参数可在验证集上给出近似相同输出；
3. 记录的 `tau` 可能不是理想逆动力学真值；
4. 线性 damping、未建模摩擦、接触效应之间可能互相补偿；
5. 当前目标函数没有显式物理约束，如质量非负、惯量正定等。

因此辨识结果必须同时从三个角度判断：

- **预测性能**：RMSE 是否足够低；
- **数值稳定性**：参数范数是否爆炸；
- **物理合理性**：质量、惯量、阻尼符号是否合理。

---

## 11. 常见数值问题与调试经验

### 11.1 加速度尖峰与离群点

数值微分对噪声极为敏感。若速度存在微小跳变，则：

$$
\ddot{q}(t_i) \approx
\frac{\dot{q}(t_{i+1}) - \dot{q}(t_{i-1})}{2\Delta t}
$$

会将误差按 $1/\Delta t$ 放大。在 1 kHz 采样下，极小的速度抖动也可能变成
数百 `rad/s^2` 的假加速度，从而使惯性项在最小二乘中占据主导地位。

### 11.2 观测矩阵秩亏

若 $W$ 存在近零奇异值，则正规方程会在这些方向上极不稳定。其表现通常为：

- 参数值飙升到极大数量级；
- 微小噪声导致参数符号翻转；
- OLS/WLS/TLS 的结果差异很大；
- 但 RMSE 变化未必显著。

### 11.3 符号约定不一致

在当前项目里，需要特别小心阻尼与摩擦项的符号：

$$
\tau = M\ddot{q} + C\dot{q} + g - D\dot{q}
$$

如果数据记录端、逆动力学端和回归矩阵端对 damping 的符号定义不一致，
则辨识结果会出现“残差很小，但 damping 参数整体为负”的现象。

### 11.4 `ctrl` 与“真实动力学力矩”不完全等价

当前记录的是 `data->ctrl`，而不是 `qfrc_bias`、`qfrc_inverse` 或
完整广义力分解项。因此辨识结果实际上在拟合一个“控制输入下的等效动力学”，
而不一定是 MuJoCo 内部最纯粹的刚体逆动力学参数。

### 11.5 完整参数不等于最小参数

本项目使用的是全参数向量，而不是经过符号消元、线性相关约简后的
最小 base parameter 集。其优点是与 MuJoCo 模型结构对应直接，
缺点是更容易遭遇不可辨识方向。

---

## 12. 代码入口与文件对应

### 12.1 主要可执行文件

- `run_experiment`：执行激励轨迹并采样生成 CSV
- `identify`：读取 CSV，运行单算法或完整 benchmark
- `mujoco_identify`：快速执行一次 MuJoCo 回归辨识
- `dynamics_diagnostic`：诊断动力学项和记录数据的差异
- `model_comparison`：比较不同动力学模型
- `regressor_test`：验证回归矩阵与动力学模型的一致性

### 12.2 关键源码位置

- `src/app/run_experiment.cpp`：实验总入口
- `src/force_node/src/force_controller.cpp`：轨迹生成、控制与安全检查
- `src/sim_com_node/src/panda_simulator.cpp`：MuJoCo 步进与 CSV 记录
- `src/identification/src/main.cpp`：离线辨识 CLI 主入口
- `src/identification/src/identification.cpp`：预处理与求解流程
- `src/identification/src/algorithms.cpp`：各类辨识算法
- `src/identification/src/robot/*regressor.cpp`：回归矩阵构造
- `src/identification/src/robot/*dynamics.cpp`：动力学计算

### 12.3 关键配置文件

- `config/experiment.yaml`：实验配置
- `config/identification.yaml`：辨识配置
- `config/force_controller_node.yaml`：控制器与轨迹参数
- `config/panda_sim_node.yaml`：仿真器参数

---

## 13. 推荐阅读顺序

若第一次接触本仓库，建议按以下顺序阅读：

1. 本文第 1 节到第 3 节，先理解系统范围和 MuJoCo 力学含义；
2. 第 4 节，掌握参数向量和回归矩阵的定义；
3. 第 5 节到第 7 节，理解数据是如何产生并进入辨识流程的；
4. 第 8 节和第 9 节，理解线性与非线性辨识算法；
5. 第 10 节和第 11 节，理解如何解释结果以及如何排查异常；
6. 最后回到第 12 节，按源码入口对应到具体实现。

---

## 14. 结论

当前仓库已经具备一条完整的参数辨识工程链路，但需要明确区分三件事：

1. **动力学方程已知**：说明可以构造正确的回归形式；
2. **数据记录合理**：说明辨识目标量与模型输出量基本一致；
3. **参数可辨识**：说明从数据中确实能稳定恢复出物理参数。

只有这三点同时成立时，“拟合结果好”才能进一步推断为“参数可信”。
本文档的目的，就是把这三层逻辑统一到同一份说明中，避免在多个离散文档
之间来回跳转时丢失上下文。
