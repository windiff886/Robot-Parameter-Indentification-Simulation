# MuJoCo 下 Franka 力矩输入的动力学方程说明

本文档说明在本项目中，当向 MuJoCo 输入控制力矩时，Franka 机械臂（含夹爪）动力学是如何被计算的。内容与 `franka_emika_panda/panda.xml` 模型保持一致，并遵循 MuJoCo 的计算方式。

## 1. 模型自由度与执行器

- 关节自由度：
  - 7 个转动关节：`joint1` ~ `joint7`
  - 2 个滑动关节：`finger_joint1`、`finger_joint2`
- 约束：`finger_joint1` 与 `finger_joint2` 通过 `equality` 约束耦合（MuJoCo 用约束力保证一致性）。
- 执行器数量：`nu = 8`
  - 7 个关节 `motor` 执行器（对应 7 个转动关节）
  - 1 个 `tendon` 执行器（驱动夹爪，两指通过 `split` tendon 关联）
- 控制输入向量：`data->ctrl` 长度为 `model->nu`（本模型为 8 维）

## 2. MuJoCo 的动力学方程（广义坐标形式）

MuJoCo 在广义坐标空间中求解刚体动力学，基本形式为：

$$
M(q)\ddot{q} + qfrc_{bias}(q, \dot{q})
= qfrc_{actuator} + qfrc_{passive} + qfrc_{applied} + qfrc_{constraint}
$$

其中：

- $q, \dot{q}, \ddot{q}$：分别对应 `data->qpos`、`data->qvel`、`data->qacc`。
- $M(q)$：广义质量矩阵（MuJoCo 中为稀疏表示，存于 `data->qM`）。
  - **armature** 会被直接加到质量矩阵对角线上（本模型默认 `armature=0.1`）。
- $qfrc_{bias}$：科里奥利 + 离心 + 重力项（MuJoCo 的 `qfrc_bias`）。
- $qfrc_{actuator}$：由执行器产生的广义力（`qfrc_actuator`）。
- $qfrc_{passive}$：被动项，包括关节阻尼、弹性与摩擦（`qfrc_passive`）。
  - 本模型默认 `damping=1`，因此阻尼项会进入 `qfrc_passive`。
- $qfrc_{applied}$：用户施加的外部广义力（`qfrc_applied` 或 `xfrc_applied`）。
- $qfrc_{constraint}$：接触、关节限位、以及 equality 约束带来的力（`qfrc_constraint`）。

### 2.1 拉格朗日方程形式（含约束）

定义拉格朗日函数：

$$
L(q, \dot{q}) = T(q, \dot{q}) - V(q)
$$

在存在约束时，MuJoCo 对应的拉格朗日形式可写为：

$$
\frac{d}{dt}\left(\frac{\partial L}{\partial \dot{q}}\right)
- \frac{\partial L}{\partial q}
=
Q_{act} + Q_{passive} + Q_{applied} + J_c(q)^T \lambda
$$

其中：

- $Q_{act} \leftrightarrow qfrc_{actuator}$
- $Q_{passive} \leftrightarrow qfrc_{passive}$
- $Q_{applied} \leftrightarrow qfrc_{applied}$
- $J_c(q)^T \lambda \leftrightarrow qfrc_{constraint}$

与上面的 MuJoCo 表达等价地可写为：

$$
M(q)\ddot{q} + C(q, \dot{q})\dot{q} + g(q)
= qfrc_{actuator} + qfrc_{passive} + qfrc_{applied} + qfrc_{constraint}
$$

其中 `qfrc_bias` 对应 $C(q, \dot{q})\dot{q} + g(q)$，`armature` 直接体现在 $M(q)$ 中。

> 说明：MuJoCo 的积分器（本模型为 `implicitfast`）会在每个仿真步内基于上述方程求解 $\ddot{q}$ 并积分得到 $\dot{q}$ 与 $q$。

## 3. 控制力矩输入到广义力的映射

MuJoCo 将控制输入 `ctrl` 转换为执行器力，再映射为广义力：

1. **控制输入限幅**
   - 若执行器设置了 `ctrllimited="true"`，则 `ctrl` 会被裁剪到 `ctrlrange`。

2. **执行器激活与力计算**
   - 若未设置显式执行器动态，默认 `act = ctrl`。
   - 对于 `motor` 执行器，力计算可写为：
     $$
     f_{act} = gear \cdot act
     $$
     本模型未显式配置 `gear`，默认值为 1。

3. **映射到广义力**
   - MuJoCo 使用执行器矩阵（`data->actuator_moment`）将力映射到广义坐标：
     $$
     qfrc_{actuator} = J_{act}(q)^T \, f_{act}
     $$
   - 对于关节 `motor`，$J_{act}$ 在对应自由度上为 1，因此广义力等于关节力矩。
   - 对于 `tendon` 执行器，$J_{act}$ 由 tendon 运动学给出，力会按 tendon 的几何关系分配到手指滑动关节。

## 4. 夹爪/手指的 MuJoCo 表达方式

- 夹爪通过 `split` tendon 驱动：
  - `tendon` 定义中，两根手指的系数均为 0.5。
  - 夹爪输入对应的是 **tendon 方向的力**（单位为 N），而不是直接的关节力矩。
- `equality` 约束保证两指同步运动，约束力进入 `qfrc_constraint`。
- 因此夹爪的实际动力学由：
  - tendon 的广义力映射
  - equality 约束力
  - 接触力（若发生碰撞）
 共同决定。

## 5. 与本项目控制话题的对应关系

在 `sim_com_node` 中，控制输入通过 ROS2 话题写入 MuJoCo：

- 订阅话题：`panda/joint_torques`
- 写入位置：`data->ctrl`
- 输入长度必须等于 `model->nu`（本模型 8 维）

因此，当你“发送力矩”时，实际生效的是 **MuJoCo 经过限幅与执行器模型处理后的广义力**，并不是原始输入的直接叠加。

## 6. 关节限位与阻尼对动力学的影响

- 关节限位由 MuJoCo 的约束机制处理（`qfrc_constraint`）。
  - `panda.xml` 中开启了 `autolimits="true"`，并设置了各关节 `range`。
- 关节阻尼来自 `joint damping`（本模型默认 `damping=1`），进入 `qfrc_passive`。
- 这些项会改变实际加速度与输出力矩，需与控制输入一起考虑。

---

如需进一步说明（例如 `mj_step` 的计算流程或 `qfrc_*` 的具体生成函数），可在此文档基础上补充对应的 MuJoCo 内部步骤。
