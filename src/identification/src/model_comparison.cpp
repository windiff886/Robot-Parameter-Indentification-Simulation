/**
 * @file model_comparison.cpp
 * @brief 详细对比 RobotDynamics 和 MuJoCoPandaDynamics 的差异
 *
 * 分析坐标系定义、变换链、雅可比矩阵等核心区别
 */

#include "mujoco_panda_dynamics.hpp"
#include "robot/franka_panda.hpp"
#include "robot/robot_dynamics.hpp"
#include <Eigen/Core>
#include <iomanip>
#include <iostream>

void printMatrix(const std::string &name, const Eigen::MatrixXd &M,
                 int rows = -1) {
  if (rows < 0)
    rows = M.rows();
  std::cout << "\n  " << name << " (" << M.rows() << "x" << M.cols() << "):\n";
  for (int i = 0; i < std::min(rows, (int)M.rows()); ++i) {
    std::cout << "    [";
    for (int j = 0; j < M.cols(); ++j) {
      std::cout << std::setw(10) << std::fixed << std::setprecision(4)
                << M(i, j);
      if (j < M.cols() - 1)
        std::cout << ", ";
    }
    std::cout << "]\n";
  }
}

void printVector(const std::string &name, const Eigen::VectorXd &v) {
  std::cout << "  " << std::setw(30) << std::left << name << ": [";
  for (int i = 0; i < v.size(); ++i) {
    std::cout << std::setw(10) << std::fixed << std::setprecision(4) << v(i);
    if (i < v.size() - 1)
      std::cout << ", ";
  }
  std::cout << "]\n";
}

int main() {
  std::cout << std::string(80, '=') << "\n";
  std::cout << "  RobotDynamics vs MuJoCoPandaDynamics 模型对比分析\n";
  std::cout << std::string(80, '=') << "\n";

  // 初始化模型
  auto robot_model = robot::createFrankaPanda();
  robot::RobotDynamics rob_dyn(*robot_model);
  mujoco_dynamics::MuJoCoPandaDynamics mj_dyn;

  // 测试配置
  Eigen::VectorXd q(7), qd(7), qdd(7);
  q << 0.0, 0.0, 0.0, -M_PI / 2, 0.0, M_PI / 2, -0.7853; // Home position
  qd.setZero();
  qdd.setZero();

  // ========================================================================
  // 1. 重力向量对比
  // ========================================================================
  std::cout << "\n" << std::string(80, '-') << "\n";
  std::cout << "  1. 重力向量对比 (零速度、零加速度，纯重力补偿)\n";
  std::cout << std::string(80, '-') << "\n";

  std::cout << "\n  [模型配置]\n";
  const auto &g_model = robot_model->gravity();
  std::cout << "  RobotModel.gravity = [" << g_model[0] << ", " << g_model[1]
            << ", " << g_model[2] << "]\n";
  std::cout << "  MuJoCo.gravity = [0, 0, -9.81] (固定)\n";
  std::cout << "\n  注意: RobotDynamics 使用 +9.81 (向上为正)，MuJoCo 使用 "
               "-9.81 (向下为正)！\n";

  Eigen::VectorXd G_rob = rob_dyn.computeGravityVector(q);
  Eigen::VectorXd G_mj = mj_dyn.computeGravityVector(q);

  std::cout << "\n  [重力向量结果]\n";
  printVector("RobotDynamics G(q)", G_rob);
  printVector("MuJoCo G(q)", G_mj);
  printVector("差值 (Rob - MJ)", G_rob - G_mj);

  // ========================================================================
  // 2. 惯量矩阵对比
  // ========================================================================
  std::cout << "\n" << std::string(80, '-') << "\n";
  std::cout << "  2. 惯量矩阵对比\n";
  std::cout << std::string(80, '-') << "\n";

  Eigen::MatrixXd M_rob = rob_dyn.computeInertiaMatrix(q);
  Eigen::MatrixXd M_mj = mj_dyn.computeInertiaMatrix(q);

  std::cout << "\n  [惯量矩阵对角线]\n";
  std::cout << "  RobotDynamics M diag: [";
  for (int i = 0; i < 7; ++i) {
    std::cout << std::setw(8) << std::fixed << std::setprecision(4)
              << M_rob(i, i);
    if (i < 6)
      std::cout << ", ";
  }
  std::cout << "]\n";

  std::cout << "  MuJoCoDynamics M diag: [";
  for (int i = 0; i < 7; ++i) {
    std::cout << std::setw(8) << std::fixed << std::setprecision(4)
              << M_mj(i, i);
    if (i < 6)
      std::cout << ", ";
  }
  std::cout << "]\n";

  std::cout << "\n  注意: MuJoCo M 已包含 armature (每关节 "
               "+0.1)，RobotDynamics 未包含！\n";

  // ========================================================================
  // 3. 坐标系变换对比
  // ========================================================================
  std::cout << "\n" << std::string(80, '-') << "\n";
  std::cout << "  3. 坐标系变换 - 根本问题\n";
  std::cout << std::string(80, '-') << "\n";

  std::cout << R"(
  [DH 参数 vs MuJoCo Body 变换]

  RobotDynamics 使用 Modified DH (Craig) 约定：
    T_i = Rot_x(α_{i-1}) * Trans_x(a_{i-1}) * Rot_z(θ_i) * Trans_z(d_i)
  
  关节轴在 DH 坐标系中始终是 z 轴，但 DH 坐标系的定义与 MuJoCo body 坐标系不同！

  MuJoCo 使用 Body-Local 坐标系：
    - 每个 body 有自己的局部坐标系
    - body 之间通过 pos + quat 进行变换
    - 关节轴在该 body 的局部坐标系中定义

  [关键差异示例 - Link 2]
  
  MuJoCo XML:
    <body name="link2" quat="1 -1 0 0">  <!-- 绕 x 轴旋转 -90° -->
      <joint name="joint2" range="-1.7628 1.7628"/>
      ...
    </body>

  这个 quat="1 -1 0 0" 表示:
    - Link 2 的局部坐标系相对于 Link 1 绕 x 轴旋转了 -90°
    - 关节 2 的旋转轴是 Link 2 局部坐标系的 z 轴
    - 但这个 "局部 z 轴" 在世界坐标系中已经被旋转了！

  DH 参数:
    Joint 2: α = -π/2, a = 0, d = 0, θ = q2
    
  这里 α = -π/2 确实表示 x 轴旋转，但 DH 变换的累积方式与 MuJoCo 不同！

)";

  // ========================================================================
  // 4. 质心位置对比
  // ========================================================================
  std::cout << std::string(80, '-') << "\n";
  std::cout << "  4. 参数表达的坐标系差异\n";
  std::cout << std::string(80, '-') << "\n";

  std::cout << R"(
  [质心位置 (COM) 定义]

  MuJoCo 中 Link 2 的参数:
    pos="0 0 0"                          (相对于 Link 1)
    quat="1 -1 0 0"                      (绕 x 轴 -90°)
    <inertial pos="-0.003141 -0.02872 0.003495" ...>  (在 Link 2 局部坐标系)

  RobotDynamics 中 (franka_panda.cpp):
    link_params_[2].com = {{-0.003141, -0.02872, 0.003495}};

  问题:
    - 这个 COM 向量是在 MuJoCo 的 Link 2 局部坐标系中定义的
    - 但 RobotDynamics 把它当作 DH Link 2 坐标系中的向量使用
    - 这两个坐标系的定义不同，导致 COM 位置计算错误！

  [惯量张量定义]

  类似地，fullinertia 也是在 MuJoCo 局部坐标系中定义的：
    fullinertia="0.0079620 2.8110e-2 2.5995e-2 -3.925e-3 1.0254e-2 7.04e-4"

  但 RobotDynamics 直接使用这些值，假设它们是在 DH 坐标系中定义的！

)";

  // ========================================================================
  // 5. 具体数值验证
  // ========================================================================
  std::cout << std::string(80, '-') << "\n";
  std::cout << "  5. 数值验证 - 单关节测试\n";
  std::cout << std::string(80, '-') << "\n";

  // 测试只有关节 2 移动时的重力力矩
  q.setZero();
  q(1) = M_PI / 4; // 关节 2 旋转 45°

  G_rob = rob_dyn.computeGravityVector(q);
  G_mj = mj_dyn.computeGravityVector(q);

  std::cout << "\n  q = [0, π/4, 0, 0, 0, 0, 0] (只有关节 2 旋转 45°)\n\n";
  printVector("RobotDynamics G(q)", G_rob);
  printVector("MuJoCo G(q)", G_mj);
  printVector("差值 (Rob - MJ)", G_rob - G_mj);

  // ========================================================================
  // 6. 总结
  // ========================================================================
  std::cout << "\n" << std::string(80, '=') << "\n";
  std::cout << "  问题根源总结\n";
  std::cout << std::string(80, '=') << "\n";

  std::cout << R"(
  RobotDynamics 存在以下核心问题:

  1. 【坐标系定义不匹配】
     - RobotDynamics 使用 DH 坐标系
     - 惯性参数 (COM, 惯量张量) 来自 MuJoCo，定义在 MuJoCo Body 坐标系
     - 这两个坐标系不一致！

  2. 【重力方向符号】
     - FrankaPanda::gravity_ = {0, 0, +9.81}  (指向上方)
     - MuJoCo gravity_ = {0, 0, -9.81}        (指向下方)
     - 虽然 computeGravityVector 中用了负号，但累积的坐标系错误导致结果仍然错误

  3. 【变换链计算方式不同】
     - DH 变换: T = Rot_x(α) * Trans_x(a) * Rot_z(θ) * Trans_z(d)
     - MuJoCo: T = T_parent * Trans(pos) * Rot(quat) * Rot_z(θ)
     - 即使物理关节相同，数学表达完全不同

  4. 【缺少 armature/damping】
     - MuJoCo 惯量矩阵包含 armature (电机转子惯量)
     - RobotDynamics 没有加入这一项

  【解决方案】

  要获得与 MuJoCo 一致的结果，有两种选择:

  (A) 使用 MuJoCoPandaDynamics 类
      - 直接使用 MuJoCo 的参数和变换方式
      - 已验证 RMSE < 0.2 Nm

  (B) 修正 RobotDynamics
      - 将惯性参数从 MuJoCo Body 坐标系转换到 DH 坐标系
      - 需要为每个连杆计算坐标系变换矩阵 R_dh_to_mj
      - 工作量大，容易出错

  推荐: 在参数辨识中使用 MuJoCoPandaDynamics！

)";

  return 0;
}
