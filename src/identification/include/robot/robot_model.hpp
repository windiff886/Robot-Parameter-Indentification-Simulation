/**
 * @file robot_model.hpp
 * @brief Robot model base class and data structures
 *
 * Defines the fundamental data structures for robot dynamics similar to BIRDy's
 * approach. Supports Modified DH (Proximal) convention.
 */

#ifndef ROBOT__ROBOT_MODEL_HPP_
#define ROBOT__ROBOT_MODEL_HPP_

#include <array>
#include <cmath>
#include <string>
#include <vector>

namespace robot {

// ============================================================================
// DH Parameters (Modified/Proximal Convention)
// ============================================================================

/**
 * @brief Denavit-Hartenberg parameters for a single joint
 *
 * Modified DH convention (Craig):
 *   alpha_{i-1}: rotation about x_{i-1} from z_{i-1} to z_i
 *   a_{i-1}:     translation along x_{i-1}
 *   d_i:         translation along z_i
 *   theta_i:     rotation about z_i (joint variable for revolute)
 */
struct DHParameter {
  double alpha{0.0}; ///< [rad] Link twist
  double a{0.0};     ///< [m] Link length
  double d{0.0};     ///< [m] Link offset
  double theta{0.0}; ///< [rad] Joint angle offset (added to joint variable)
};

// ============================================================================
// Inertia
// ============================================================================

/**
 * @brief 3x3 Symmetric inertia tensor
 *
 * Stored as upper triangular: [Ixx, Ixy, Ixz, Iyy, Iyz, Izz]
 * MuJoCo fullinertia format: [Ixx, Iyy, Izz, Ixy, Ixz, Iyz]
 */
struct InertiaTensor {
  double Ixx{0.0};
  double Ixy{0.0};
  double Ixz{0.0};
  double Iyy{0.0};
  double Iyz{0.0};
  double Izz{0.0};

  /**
   * @brief Construct from MuJoCo fullinertia format
   * @param mj_inertia Array in order [Ixx, Iyy, Izz, Ixy, Ixz, Iyz]
   */
  static InertiaTensor fromMuJoCo(const std::array<double, 6> &mj_inertia) {
    return InertiaTensor{
        mj_inertia[0], // Ixx
        mj_inertia[3], // Ixy
        mj_inertia[4], // Ixz
        mj_inertia[1], // Iyy
        mj_inertia[5], // Iyz
        mj_inertia[2]  // Izz
    };
  }
};

// ============================================================================
// Link Parameters
// ============================================================================

/**
 * @brief Physical parameters for a single link
 */
struct LinkParameters {
  double mass{0.0}; ///< [kg] Link mass
  std::array<double, 3> com{
      {0.0, 0.0, 0.0}};  ///< [m] Center of mass in link frame
  InertiaTensor inertia; ///< Inertia tensor at COM
};

// ============================================================================
// Joint Limits
// ============================================================================

/**
 * @brief Physical constraints for a single joint
 */
struct JointLimits {
  double q_min{-M_PI};   ///< [rad] Minimum position
  double q_max{M_PI};    ///< [rad] Maximum position
  double qd_max{2.0};    ///< [rad/s] Maximum velocity
  double qdd_max{10.0};  ///< [rad/s²] Maximum acceleration
  double tau_max{100.0}; ///< [Nm] Maximum torque
};

// ============================================================================
// Control Parameters
// ============================================================================

/**
 * @brief Control-related parameters
 */
struct ControlParameters {
  double control_frequency{1000.0};  ///< [Hz] Control loop frequency
  double sampling_frequency{1000.0}; ///< [Hz] Sensor sampling frequency
  std::vector<double> kp;            ///< Position gains
  std::vector<double> kd;            ///< Velocity gains (damping)
  std::vector<double> ki;            ///< Integral gains
};

// ============================================================================
// Friction Parameters
// ============================================================================

/**
 * @brief Friction model parameters
 */
struct FrictionParameters {
  double Fv{0.0};  ///< Viscous friction coefficient
  double Fc{0.0};  ///< Coulomb friction
  double Fs{0.0};  ///< Static friction
  double Vs{0.01}; ///< Stribeck velocity
};

// ============================================================================
// Fixed Body Attachment
// ============================================================================

/**
 * @brief Fixed body attached to a parent link
 *
 * Represents a body that is rigidly attached to a parent link (e.g., Hand
 * attached to Link 7). The fixed body does not add DOF but contributes to
 * the overall dynamics through its inertial parameters.
 */
struct FixedBodyAttachment {
  std::size_t parent_link_idx{0}; ///< Index of parent link (0-based)
  std::array<double, 3> position{
      {0.0, 0.0, 0.0}}; ///< Position offset from parent link origin
  std::array<double, 4> quaternion{
      {1.0, 0.0, 0.0, 0.0}}; ///< Orientation (w, x, y, z)
  LinkParameters params;     ///< Inertial parameters (mass, COM, inertia)
  std::string name;          ///< Name of the fixed body
};

// ============================================================================
// Robot Model Base Class
// ============================================================================

/**
 * @brief Base class for robot model
 *
 * Similar to BIRDy's robot description files, this class encapsulates
 * all kinematic and dynamic parameters of a robot.
 */
class RobotModel {
public:
  virtual ~RobotModel() = default;

  // ========== Getters ==========

  const std::string &name() const { return name_; }
  std::size_t numDOF() const { return num_dof_; }

  const std::vector<DHParameter> &dhParameters() const { return dh_params_; }
  const std::vector<LinkParameters> &linkParameters() const {
    return link_params_;
  }
  const std::vector<JointLimits> &jointLimits() const { return joint_limits_; }
  const std::vector<FrictionParameters> &frictionParameters() const {
    return friction_params_;
  }
  const ControlParameters &controlParameters() const { return control_params_; }

  const std::array<double, 3> &gravity() const { return gravity_; }
  const std::vector<double> &homePosition() const { return home_position_; }

  /**
   * @brief Get DH parameter for specific joint
   */
  const DHParameter &dh(std::size_t joint_idx) const {
    return dh_params_.at(joint_idx);
  }

  /**
   * @brief Get link parameter for specific link
   */
  const LinkParameters &link(std::size_t link_idx) const {
    return link_params_.at(link_idx);
  }

  /**
   * @brief Get joint limits for specific joint
   */
  const JointLimits &limits(std::size_t joint_idx) const {
    return joint_limits_.at(joint_idx);
  }

  /**
   * @brief Get all fixed body attachments
   */
  const std::vector<FixedBodyAttachment> &fixedBodies() const {
    return fixed_bodies_;
  }

  /**
   * @brief Get total number of bodies (links + fixed bodies)
   */
  std::size_t numBodies() const {
    return link_params_.size() + fixed_bodies_.size();
  }

protected:
  std::string name_;
  std::size_t num_dof_{0};

  std::vector<DHParameter> dh_params_;
  std::vector<LinkParameters> link_params_;
  std::vector<JointLimits> joint_limits_;
  std::vector<FrictionParameters> friction_params_;
  std::vector<FixedBodyAttachment>
      fixed_bodies_; // Fixed bodies attached to links
  ControlParameters control_params_;

  std::array<double, 3> gravity_{{0.0, 0.0, 9.81}};
  std::vector<double> home_position_;
};

} // namespace robot

#endif // ROBOT__ROBOT_MODEL_HPP_
