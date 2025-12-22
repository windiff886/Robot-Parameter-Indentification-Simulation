/**
 * @file franka_panda.hpp
 * @brief Franka Emika Panda robot model declaration
 *
 * 7-DOF robot arm with parameters extracted from MuJoCo XML.
 */

#ifndef ROBOT__FRANKA_PANDA_HPP_
#define ROBOT__FRANKA_PANDA_HPP_

#include "robot/robot_model.hpp"
#include <memory>

namespace robot {

/**
 * @brief Franka Emika Panda 7-DOF robot arm
 *
 * Robot parameters extracted from:
 *   - MuJoCo: franka_emika_panda/panda_nohand.xml
 *   - BIRDy: Utils/SymbolicModelData/RobotDescriptionFiles/Franka.m
 *
 * Uses Modified DH (Proximal/Craig) convention.
 */
class FrankaPanda : public RobotModel {
public:
  /// Number of arm joints (excluding gripper)
  static constexpr std::size_t NUM_JOINTS = 7;

  /// Number of links (including base link0)
  static constexpr std::size_t NUM_LINKS = 8;

  FrankaPanda();
  ~FrankaPanda() override = default;

  // No copy, allow move
  FrankaPanda(const FrankaPanda &) = delete;
  FrankaPanda &operator=(const FrankaPanda &) = delete;
  FrankaPanda(FrankaPanda &&) = default;
  FrankaPanda &operator=(FrankaPanda &&) = default;
};

/**
 * @brief Factory function to create Franka Panda model
 * @return Unique pointer to FrankaPanda model
 */
std::unique_ptr<RobotModel> createFrankaPanda();

} // namespace robot

#endif // ROBOT__FRANKA_PANDA_HPP_
