/**
 * @file franka_panda.cpp
 * @brief Franka Emika Panda robot model implementation
 *
 * Parameters extracted from MuJoCo panda_nohand.xml
 */

#include "robot/franka_panda.hpp"
#include <cmath>

namespace robot {

FrankaPanda::FrankaPanda() {
  name_ = "FrankaPanda";
  num_dof_ = NUM_JOINTS;

  // ========================================================================
  // DH Parameters (Modified DH / Proximal / Craig Convention)
  // ========================================================================
  // From BIRDy Franka.m and standard Franka specifications:
  //   alpha(i-1), a(i-1), d(i), theta_offset
  //
  // Joint 1: alpha=0,     a=0,       d=0.333,  theta=q1
  // Joint 2: alpha=-π/2,  a=0,       d=0,      theta=q2
  // Joint 3: alpha=π/2,   a=0,       d=0.316,  theta=q3
  // Joint 4: alpha=π/2,   a=0.0825,  d=0,      theta=q4
  // Joint 5: alpha=-π/2,  a=-0.0825, d=0.384,  theta=q5
  // Joint 6: alpha=π/2,   a=0,       d=0,      theta=q6
  // Joint 7: alpha=π/2,   a=0.088,   d=0,      theta=q7

  dh_params_.resize(NUM_JOINTS);

  dh_params_[0] = {0.0, 0.0, 0.333, 0.0};
  dh_params_[1] = {-M_PI / 2.0, 0.0, 0.0, 0.0};
  dh_params_[2] = {M_PI / 2.0, 0.0, 0.316, 0.0};
  dh_params_[3] = {M_PI / 2.0, 0.0825, 0.0, 0.0};
  dh_params_[4] = {-M_PI / 2.0, -0.0825, 0.384, 0.0};
  dh_params_[5] = {M_PI / 2.0, 0.0, 0.0, 0.0};
  dh_params_[6] = {M_PI / 2.0, 0.088, 0.0, 0.0};

  // ========================================================================
  // Link Parameters (from panda_nohand.xml)
  // ========================================================================
  // Note: link0 is the base, link1-link7 are the arm links
  //       Parameters extracted from <inertial> tags in XML

  link_params_.resize(NUM_LINKS);

  // Link 0 (Base)
  // mass="0.629769" pos="-0.041018 -0.00014 0.049974"
  // fullinertia="0.00315 0.00388 0.004285 8.2904e-7 0.00015 8.2299e-6"
  link_params_[0].mass = 0.629769;
  link_params_[0].com = {{-0.041018, -0.00014, 0.049974}};
  link_params_[0].inertia = InertiaTensor::fromMuJoCo(
      {{0.00315, 0.00388, 0.004285, 8.2904e-7, 0.00015, 8.2299e-6}});

  // Link 1
  // mass="4.970684" pos="0.003875 0.002081 -0.04762"
  // fullinertia="0.70337 0.70661 0.0091170 -0.00013900 0.0067720 0.019169"
  link_params_[1].mass = 4.970684;
  link_params_[1].com = {{0.003875, 0.002081, -0.04762}};
  link_params_[1].inertia = InertiaTensor::fromMuJoCo(
      {{0.70337, 0.70661, 0.0091170, -0.00013900, 0.0067720, 0.019169}});

  // Link 2
  // mass="0.646926" pos="-0.003141 -0.02872 0.003495"
  // fullinertia="0.0079620 2.8110e-2 2.5995e-2 -3.925e-3 1.0254e-2 7.04e-4"
  link_params_[2].mass = 0.646926;
  link_params_[2].com = {{-0.003141, -0.02872, 0.003495}};
  link_params_[2].inertia = InertiaTensor::fromMuJoCo(
      {{0.0079620, 2.8110e-2, 2.5995e-2, -3.925e-3, 1.0254e-2, 7.04e-4}});

  // Link 3
  // mass="3.228604" pos="2.7518e-2 3.9252e-2 -6.6502e-2"
  // fullinertia="3.7242e-2 3.6155e-2 1.083e-2 -4.761e-3 -1.1396e-2 -1.2805e-2"
  link_params_[3].mass = 3.228604;
  link_params_[3].com = {{2.7518e-2, 3.9252e-2, -6.6502e-2}};
  link_params_[3].inertia = InertiaTensor::fromMuJoCo(
      {{3.7242e-2, 3.6155e-2, 1.083e-2, -4.761e-3, -1.1396e-2, -1.2805e-2}});

  // Link 4
  // mass="3.587895" pos="-5.317e-2 1.04419e-1 2.7454e-2"
  // fullinertia="2.5853e-2 1.9552e-2 2.8323e-2 7.796e-3 -1.332e-3 8.641e-3"
  link_params_[4].mass = 3.587895;
  link_params_[4].com = {{-5.317e-2, 1.04419e-1, 2.7454e-2}};
  link_params_[4].inertia = InertiaTensor::fromMuJoCo(
      {{2.5853e-2, 1.9552e-2, 2.8323e-2, 7.796e-3, -1.332e-3, 8.641e-3}});

  // Link 5
  // mass="1.225946" pos="-1.1953e-2 4.1065e-2 -3.8437e-2"
  // fullinertia="3.5549e-2 2.9474e-2 8.627e-3 -2.117e-3 -4.037e-3 2.29e-4"
  link_params_[5].mass = 1.225946;
  link_params_[5].com = {{-1.1953e-2, 4.1065e-2, -3.8437e-2}};
  link_params_[5].inertia = InertiaTensor::fromMuJoCo(
      {{3.5549e-2, 2.9474e-2, 8.627e-3, -2.117e-3, -4.037e-3, 2.29e-4}});

  // Link 6
  // mass="1.666555" pos="6.0149e-2 -1.4117e-2 -1.0517e-2"
  // fullinertia="1.964e-3 4.354e-3 5.433e-3 1.09e-4 -1.158e-3 3.41e-4"
  link_params_[6].mass = 1.666555;
  link_params_[6].com = {{6.0149e-2, -1.4117e-2, -1.0517e-2}};
  link_params_[6].inertia = InertiaTensor::fromMuJoCo(
      {{1.964e-3, 4.354e-3, 5.433e-3, 1.09e-4, -1.158e-3, 3.41e-4}});

  // Link 7 (只包含 Link 7 自身，不合并 Hand)
  // fullinertia=\"0.012516 0.010027 0.004815 -0.000428 -0.001196 -0.000741\"
  link_params_[7].mass = 7.35522e-01;
  link_params_[7].com = {{1.0517e-2, -4.252e-3, 6.1597e-2}};
  link_params_[7].inertia = InertiaTensor::fromMuJoCo(
      {{1.2516e-2, 1.0027e-2, 4.815e-3, -4.28e-4, -1.196e-3, -7.41e-4}});

  // ========================================================================
  // Fixed Body: Hand (附着在 Link 7)
  // ========================================================================
  // Hand 通过固定变换附着在 Link 7 上
  // Position: (0, 0, 0.107) - 沿 Link 7 的 z 轴向上 107mm
  // Orientation: quat=(0.9238795, 0, 0, -0.3826834) - 绕 z 轴旋转 -45°
  //
  // Hand 自身的惯性参数（在 Hand 质心处定义）:
  //   mass=0.73 kg
  //   COM (in hand frame): (-0.01, 0, 0.03)
  //   diaginertia: (0.001, 0.0025, 0.0017)
  {
    FixedBodyAttachment hand;
    hand.name = "hand";
    hand.parent_link_idx = 7;            // 附着到 Link 7
    hand.position = {{0.0, 0.0, 0.107}}; // 107mm offset along z
    hand.quaternion = {
        {0.9238795, 0.0, 0.0, -0.3826834}}; // -45° about z (w,x,y,z)

    hand.params.mass = 0.73;
    hand.params.com = {{-0.01, 0.0, 0.03}}; // COM in hand frame
    // diaginertia -> fullinertia
    hand.params.inertia =
        InertiaTensor::fromMuJoCo({{0.001, 0.0025, 0.0017, 0.0, 0.0, 0.0}});

    fixed_bodies_.push_back(hand);
  }

  // ========================================================================
  // Joint Limits (from panda_nohand.xml)
  // ========================================================================
  // Default joint range in XML: range="-2.8973 2.8973"
  // Default force range: forcerange="-87 87" (joints 1-4), forcerange="-12 12"
  // (joints 5-7)

  joint_limits_.resize(NUM_JOINTS);

  // Joint 1: default range, 87 Nm
  joint_limits_[0] = {-2.8973, 2.8973, 2.175, 15.0, 87.0};

  // Joint 2: range="-1.7628 1.7628", 87 Nm
  joint_limits_[1] = {-1.7628, 1.7628, 2.175, 7.5, 87.0};

  // Joint 3: default range, 87 Nm
  joint_limits_[2] = {-2.8973, 2.8973, 2.175, 10.0, 87.0};

  // Joint 4: range="-3.0718 -0.0698", 87 Nm
  joint_limits_[3] = {-3.0718, -0.0698, 2.175, 12.5, 87.0};

  // Joint 5: default range, 12 Nm
  joint_limits_[4] = {-2.8973, 2.8973, 2.610, 15.0, 12.0};

  // Joint 6: range="-0.0175 3.7525", 12 Nm
  joint_limits_[5] = {-0.0175, 3.7525, 2.610, 20.0, 12.0};

  // Joint 7: default range, 12 Nm
  joint_limits_[6] = {-2.8973, 2.8973, 2.610, 20.0, 12.0};

  // ========================================================================
  // Friction Parameters (initial estimates)
  // ========================================================================

  friction_params_.resize(NUM_JOINTS);
  for (std::size_t i = 0; i < NUM_JOINTS; ++i) {
    friction_params_[i] = {0.5, 0.5, 0.0, 0.01}; // Fv, Fc, Fs, Vs
  }

  // ========================================================================
  // Control Parameters
  // ========================================================================

  control_params_.control_frequency = 1000.0;
  control_params_.sampling_frequency = 1000.0;

  // PD gains (conservative values for position control)
  control_params_.kp = {600.0, 600.0, 600.0, 500.0, 150.0, 150.0, 50.0};
  control_params_.kd = {50.0, 50.0, 50.0, 30.0, 10.0, 10.0, 5.0};
  control_params_.ki.resize(NUM_JOINTS, 0.0);

  // ========================================================================
  // Gravity
  // ========================================================================

  gravity_ = {{0.0, 0.0, 9.81}};

  // ========================================================================
  // Home Position (from panda_nohand.xml keyframe "home")
  // ========================================================================
  // qpos="0 0 0 -1.57079 0 1.57079 -0.7853"

  home_position_ = {0.0, 0.0, 0.0, -M_PI / 2.0, 0.0, M_PI / 2.0, -0.7853};
}

std::unique_ptr<RobotModel> createFrankaPanda() {
  return std::make_unique<FrankaPanda>();
}

} // namespace robot
