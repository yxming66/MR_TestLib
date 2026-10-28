#pragma once

// Fixed-size actuator boundary for the three arm joints. The generic arm
// controller stops at joint coordinates; this class is the only arm-module
// code that knows the concrete motor protocols.

#include <stddef.h>
#include <stdint.h>

#include "device/motor/core/motor_aliases.hpp"
#include "device/motor/core/motor_state.hpp"
#include "device/motor/motor.hpp"
#include "module/arm/arm_control_types.h"
#include "robotics/arm/core/types.hpp"

namespace mr::arm {

using ThreeAxisJointState =
    mr::robotics::arm::JointState<ARM_JOINT_COUNT>;
using ThreeAxisJointCommand =
    mr::robotics::arm::JointCommand<ARM_JOINT_COUNT>;

class ArmOutputAdapter final {
 public:
  ArmOutputAdapter();

  ArmOutputAdapter(const ArmOutputAdapter&) = delete;
  ArmOutputAdapter& operator=(const ArmOutputAdapter&) = delete;

  bool Initialize(const Arm_Params_t& parameters);
  bool IsInitialized() const { return initialized_; }

  int8_t Register();
  int8_t Enable();
  int8_t Relax();
  int8_t Relax(size_t index);
  int8_t UpdateFeedback(ThreeAxisJointState* state);
  int8_t Apply(const ThreeAxisJointCommand& command);
  int8_t Commit();
  int8_t SetZero(size_t index);

  mr::motor::MotorState MotorState(size_t index) const;
  bool IsOnline(size_t index) const;

  mr::motor::LzRso3Motor* Joint1Motor() { return joint1_motor_; }
  mr::motor::DmJ4340Motor* Joint2Motor() { return joint2_motor_; }
  mr::motor::DmJ4310PMotor* Joint3Motor() { return joint3_motor_; }

 private:
  static mr::motor::MotorTemperatureProtectionConfig
  MakeTemperatureProtection(
      const MOTOR_TemperatureProtectionConfig_t& parameters);

  int8_t FirstFailure(int8_t current, int8_t next) const;
  int8_t ApplyJoint(size_t index, const ThreeAxisJointCommand& command);

  mr::motor::LzRso3Motor* joint1_motor_;
  mr::motor::DmJ4340Motor* joint2_motor_;
  mr::motor::DmJ4310PMotor* joint3_motor_;
  float kp_[ARM_JOINT_COUNT];
  float kd_[ARM_JOINT_COUNT];
  float gravity_scale_[ARM_JOINT_COUNT];
  bool initialized_;
};

}  // namespace mr::arm
