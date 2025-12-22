# 参数辨识算法说明

本文档详细说明了程序中使用的机器人动力学参数辨识算法。包括已迁移的 C++ 实现算法（基于 Eigen）以及 BIRDy 框架中所有支持的算法原理。

代码位置：`src/identification/src/algorithms.cpp`

## 1. 辨识模型形式

参数辨识的基础是将动力学方程改写为关于动力学参数的在线性形式：

$$ Y(q, \dot{q}, \ddot{q}) \cdot \beta = \tau $$

将所有采样时刻堆叠得到超定方程组：
$$ W \cdot \beta = \mathcal{T}_{meas} $$

## 2. 离线辨识算法 (C++ 已实现)

本系统已移植了 BIRDy 中的核心线性辨识算法，可以通过 `identification_node` 的 `algorithm` 参数进行选择。

### 2.1 普通最小二乘法 (OLS)
**参数**: `algorithm:="OLS"`

假设测量噪声为零均值高斯白噪声且同分布。

$$ \hat{\beta}_{OLS} = (W^T W)^{-1} W^T \mathcal{T}_{meas} $$

**特性**: 计算最快，是基准算法。但对噪声分布敏感。

### 2.2 加权最小二乘法 (WLS)
**参数**: `algorithm:="WLS"`

解决不同关节噪声方差不同（异方差性）的问题。
假设噪声协方差矩阵 $\Sigma = \text{diag}(\sigma_1^2, \dots, \sigma_N^2)$。

$$ \hat{\beta}_{WLS} = (W^T \Sigma^{-1} W)^{-1} W^T \Sigma^{-1} \mathcal{T}_{meas} $$

**实现细节**:
1. 先运行 OLS 得到残差。
2. 统计每个关节的残差方差 $\hat{\sigma}_j^2$。
3. 对观测矩阵 $W$ 和 $\tau$ 进行加权：$W_{row\_i} \leftarrow W_{row\_i} / \hat{\sigma}_j$。
4. 再次求解 OLS。

### 2.3 迭代重加权最小二乘法 (IRLS)
**参数**: `algorithm:="IRLS"`

用于增强鲁棒性，抵抗数据中的离群值 (Outliers)。
采用 Huber 损失函数或 Tukey 权函数。

**流程**:
1. 初始求解 OLS 得到 $\beta^{(0)}$。
2. 计算残差 $r = \mathcal{T}_{meas} - W \beta^{(k)}$。
3. 更新权重矩阵 $S$：这取决于残差的大小，残差越大权重越小。
   $$ w_i = \begin{cases} 1 & |r_i| \le \delta \\ \delta/|r_i| & |r_i| > \delta \end{cases} $$
4. 求解加权最小二乘得到 $\beta^{(k+1)}$。
5. 迭代直到收敛。

### 2.4 总体最小二乘法 (TLS)
**参数**: `algorithm:="TLS"`

考虑到观测矩阵 $W$ (由于加速度计算引入的误差) 和 $\mathcal{T}_{meas}$ (力矩传感器) **同时存在噪声**的情况。

**原理**:
构造增广矩阵 $Z = [W, \mathcal{T}_{meas}]$，并进行奇异值分解 (SVD)：
$$ Z = U \Sigma V^T $$
取最小奇异值对应的右奇异向量 $v_{min}$，解析解为：
$$ \hat{\beta}_{TLS} = - \frac{v_{min}(1:M)}{v_{min}(M+1)} $$

---

### 2.5 扩展卡尔曼滤波 (EKF)
**参数**: `algorithm:="EKF"`

适用于在线辨识或参数随时间缓慢变化的情况。
这里通过**递推最小二乘 (RLS)** 的形式实现，将参数 $\beta$ 视为状态，假设过程噪声 $Q \approx 0$。

### 2.6 极大似然估计 (ML)
**参数**: `algorithm:="ML"`
**依赖**: `LevenbergMarquardt` Solver (Built-in)

使用内置的 Levenberg-Marquardt 非线性求解器，直接最小化观测误差 $|| W\beta - \tau ||^2$。
对于线性模型，其结果理论上与 OLS 一致，但展示了非线性优化框架的使用，可扩展用于非线性参数化模型。

### 2.7 闭环输出误差 (CLOE)
**参数**: `algorithm:="CLOE"`
**状态**: *实验性支持*

通过积分前向动力学方程 $\ddot{q} = M^{-1}(\tau - C\dot{q} - G)$ 得到位置 $\hat{q}$，并最小化位置误差 $|| q_{meas} - \hat{q} ||^2$。
**注意**: 目前 C++ 实现中，由于 `RobotModel` 类设计的常数性，无法高效在优化循环中更新动力学模型参数，该算法目前仅作为接口存根 (Stub) 或返回 OLS 初始估计，完整功能待后续重构 `RobotModel` 后开启。

---

## 4. 运行示例

```bash
# 使用 OLS (默认)
ros2 launch identification identification.launch.py data_file:=$(pwd)/data/exp1.csv algorithm:=OLS

# 使用 WRls (鲁棒性更好)
ros2 launch identification identification.launch.py data_file:=$(pwd)/data/exp1.csv algorithm:=IRLS

# 使用 EKF (在线递推风格)
ros2 launch identification identification.launch.py data_file:=$(pwd)/data/exp1.csv algorithm:=EKF

# 使用 ML (非线性求解器)
ros2 launch identification identification.launch.py data_file:=$(pwd)/data/exp1.csv algorithm:=ML
```
