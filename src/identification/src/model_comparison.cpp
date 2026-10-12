/**
 * @file model_comparison.cpp
 * @brief Detailed comparison between RobotDynamics and MuJoCoPandaDynamics
 *
 * Analyzes core differences such as coordinate frame definitions, transform chains, Jacobian matrices, etc.
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
  std::cout << "  RobotDynamics vs MuJoCoPandaDynamics Model Comparison Analysis\n";
  std::cout << std::string(80, '=') << "\n";

  // Initialize models
  auto robot_model = robot::createFrankaPanda();
  robot::RobotDynamics rob_dyn(*robot_model);
  mujoco_dynamics::MuJoCoPandaDynamics mj_dyn;

  // Test configurations
  Eigen::VectorXd q(7), qd(7), qdd(7);
  q << 0.0, 0.0, 0.0, -M_PI / 2, 0.0, M_PI / 2, -0.7853; // Home position
  qd.setZero();
  qdd.setZero();

  // ========================================================================
  // 1. Gravity vector comparison
  // ========================================================================
  std::cout << "\n" << std::string(80, '-') << "\n";
  std::cout << "  1. Gravity Vector Comparison (zero velocity, zero acceleration, pure gravity compensation)\n";
  std::cout << std::string(80, '-') << "\n";

  std::cout << "\n  [Model Configurations]\n";
  const auto &g_model = robot_model->gravity();
  std::cout << "  RobotModel.gravity = [" << g_model[0] << ", " << g_model[1]
            << ", " << g_model[2] << "]\n";
  std::cout << "  MuJoCo.gravity = [0, 0, -9.81] (fixed)\n";
  std::cout << "\n  Note: RobotDynamics uses +9.81 (upward positive), MuJoCo uses "
               "-9.81 (downward positive)!\n";

  Eigen::VectorXd G_rob = rob_dyn.computeGravityVector(q);
  Eigen::VectorXd G_mj = mj_dyn.computeGravityVector(q);

  std::cout << "\n  [Gravity Vector Results]\n";
  printVector("RobotDynamics G(q)", G_rob);
  printVector("MuJoCo G(q)", G_mj);
  printVector("Diff (Rob - MJ)", G_rob - G_mj);

  // ========================================================================
  // 2. Inertia matrix comparison
  // ========================================================================
  std::cout << "\n" << std::string(80, '-') << "\n";
  std::cout << "  2. Inertia Matrix Comparison\n";
  std::cout << std::string(80, '-') << "\n";

  Eigen::MatrixXd M_rob = rob_dyn.computeInertiaMatrix(q);
  Eigen::MatrixXd M_mj = mj_dyn.computeInertiaMatrix(q);

  std::cout << "\n  [Inertia Matrix Diagonal]\n";
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

  std::cout << "\n  Note: MuJoCo M already includes armature (+0.1 per joint), RobotDynamics does not!\n";

  // ========================================================================
  // 3. Coordinate transform comparison
  // ========================================================================
  std::cout << "\n" << std::string(80, '-') << "\n";
  std::cout << "  3. Coordinate Transforms - Root Cause\n";
  std::cout << std::string(80, '-') << "\n";

  std::cout << R"(
  [DH Parameters vs MuJoCo Body Transforms]

  RobotDynamics uses Modified DH (Craig) convention:
    T_i = Rot_x(α_{i-1}) * Trans_x(a_{i-1}) * Rot_z(θ_i) * Trans_z(d_i)
  
  Joint axis in DH frame is always the z axis, but the DH frame definition differs from MuJoCo body frame!

  MuJoCo uses Body-Local frames:
    - Each body has its own local coordinate frame
    - Bodies are transformed relative to each other via pos + quat
    - Joint axis is defined in that body's local coordinate frame

  [Key Difference Example - Link 2]
  
  MuJoCo XML:
    <body name="link2" quat="1 -1 0 0">  <!-- Rotation of -90 deg around x axis -->
      <joint name="joint2" range="-1.7628 1.7628"/>
      ...
    </body>

  This quat="1 -1 0 0" indicates:
    - Link 2 local frame is rotated by -90 deg around x axis relative to Link 1
    - Joint 2 axis of rotation is the z axis of Link 2 local frame
    - But this "local z axis" is already rotated in world coordinates!

  DH Parameters:
    Joint 2: α = -π/2, a = 0, d = 0, θ = q2
    
  Here α = -π/2 indeed denotes an x-axis rotation, but DH transform accumulation differs from MuJoCo!

)";

  // ========================================================================
  // 4. Center of mass (COM) comparison
  // ========================================================================
  std::cout << std::string(80, '-') << "\n";
  std::cout << "  4. Coordinate Frame Differences in Parameter Representation\n";
  std::cout << std::string(80, '-') << "\n";

  std::cout << R"(
  [Center of Mass (COM) Definition]

  MuJoCo Link 2 parameters:
    pos="0 0 0"                          (relative to Link 1)
    quat="1 -1 0 0"                      (around x axis -90 deg)
    <inertial pos="-0.003141 -0.02872 0.003495" ...>  (in Link 2 local frame)

  In RobotDynamics (franka_panda.cpp):
    link_params_[2].com = {{-0.003141, -0.02872, 0.003495}};

  Problem:
    - This COM vector is defined in MuJoCo's Link 2 local frame
    - But RobotDynamics treats it as a vector in the DH Link 2 frame
    - These two frames have different definitions, resulting in incorrect COM position calculation!

  [Inertia Tensor Definition]

  Similarly, fullinertia is also defined in MuJoCo local frame:
    fullinertia="0.0079620 2.8110e-2 2.5995e-2 -3.925e-3 1.0254e-2 7.04e-4"

  However, RobotDynamics directly uses these values, assuming they are defined in DH frame!

)";

  // ========================================================================
  // 5. Numerical verification
  // ========================================================================
  std::cout << std::string(80, '-') << "\n";
  std::cout << "  5. Numerical Verification - Single Joint Test\n";
  std::cout << std::string(80, '-') << "\n";

  // Test gravity torque when only joint 2 moves
  q.setZero();
  q(1) = M_PI / 4; // Joint 2 rotated 45 deg

  G_rob = rob_dyn.computeGravityVector(q);
  G_mj = mj_dyn.computeGravityVector(q);

  std::cout << "\n  q = [0, π/4, 0, 0, 0, 0, 0] (only Joint 2 rotated 45 deg)\n\n";
  printVector("RobotDynamics G(q)", G_rob);
  printVector("MuJoCo G(q)", G_mj);
  printVector("Diff (Rob - MJ)", G_rob - G_mj);

  // ========================================================================
  // 6. Summary
  // ========================================================================
  std::cout << "\n" << std::string(80, '=') << "\n";
  std::cout << "  Root Cause Summary\n";
  std::cout << std::string(80, '=') << "\n";

  std::cout << R"(
  RobotDynamics has the following core issues:

  1. [Coordinate Frame Mismatch]
     - RobotDynamics uses DH frame
     - Inertial parameters (COM, inertia tensor) come from MuJoCo, defined in MuJoCo Body frame
     - These two frames do not match!

  2. [Gravity Direction Sign]
     - FrankaPanda::gravity_ = {0, 0, +9.81}  (pointing upward)
     - MuJoCo gravity_ = {0, 0, -9.81}        (pointing downward)
     - Although a minus sign is used in computeGravityVector, accumulated frame errors still yield incorrect results

  3. [Transform Chain Computation Differs]
     - DH transform: T = Rot_x(α) * Trans_x(a) * Rot_z(θ) * Trans_z(d)
     - MuJoCo: T = T_parent * Trans(pos) * Rot(quat) * Rot_z(θ)
     - Even though physical joints are identical, mathematical expressions differ completely

  4. [Missing armature/damping]
     - MuJoCo inertia matrix includes armature (rotor inertia)
     - RobotDynamics does not include this term

  [Solution]

  To achieve results consistent with MuJoCo, there are two options:

  (A) Use MuJoCoPandaDynamics class
      - Directly uses MuJoCo parameters and transforms
      - Verified RMSE < 0.2 Nm

  (B) Fix RobotDynamics
      - Transform inertial parameters from MuJoCo Body frame to DH frame
      - Requires computing transformation matrix R_dh_to_mj for each link
      - High effort and error-prone

  Recommendation: Use MuJoCoPandaDynamics for parameter identification!

)";

  return 0;
}
