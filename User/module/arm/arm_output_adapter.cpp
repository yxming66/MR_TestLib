#include "module/arm/arm_output_adapter.hpp"

#include "component/math/scalar.hpp"
#include "device/device.h"
#include "device/motor/factory/motor_factory.hpp"

namespace mr::arm {
namespace {

using mr::motor::MotorFactory;
using mr::motor::MotorInstanceConfig;
using mr::motor::MotorKind;
using mr::motor::MotorModel;

float FiniteOr(float value, float fallback) {
  return mr::component::math::is_finite_scalar(value) ? value : fallback;
}

}  // namespace

ArmOutputAdapter::ArmOutputAdapter()
    : joint1_motor_(nullptr),
      joint2_motor_(nullptr),
      joint3_motor_(nullptr),
      kp_{0.0f, 0.0f, 0.0f},
      kd_{0.0f, 0.0f, 0.0f},
      gravity_scale_{1.0f, 1.0f, 1.0f},
      initialized_(false) {}

mr::motor::MotorTemperatureProtectionConfig
ArmOutputAdapter::MakeTemperatureProtection(
    const MOTOR_TemperatureProtectionConfig_t& parameters) {
  mr::motor::MotorTemperatureProtectionConfig result;
  result.warning_c = parameters.warning_c;
  result.limit_c = parameters.limit_c;
  result.auto_relax_on_limit = parameters.auto_relax_on_limit;
  return result;
}

bool ArmOutputAdapter::Initialize(const Arm_Params_t& parameters) {
  if (initialized_) {
    return true;
  }

  const MotorInstanceConfig<MotorKind::LZ> joint1_config =
      MotorInstanceConfig<MotorKind::LZ>::FromVendorParam(
          parameters.joint1_motor_param);
  const MotorInstanceConfig<MotorKind::DM> joint2_config =
      MotorInstanceConfig<MotorKind::DM>::FromVendorParam(
          parameters.joint2_motor_param);
  const MotorInstanceConfig<MotorKind::DM> joint3_config =
      MotorInstanceConfig<MotorKind::DM>::FromVendorParam(
          parameters.joint3_motor_param);

  joint1_motor_ = MotorFactory::Create<MotorKind::LZ, MotorModel::RSO3>(
      joint1_config, mr::motor::kDirectDriveInstall,
      MakeTemperatureProtection(parameters.joint_temperature_protection[0]));
  joint2_motor_ = MotorFactory::Create<MotorKind::DM, MotorModel::J4340>(
      joint2_config, mr::motor::kDirectDriveInstall,
      MakeTemperatureProtection(parameters.joint_temperature_protection[1]));
  joint3_motor_ = MotorFactory::Create<MotorKind::DM, MotorModel::J4310P>(
      joint3_config, mr::motor::kDirectDriveInstall,
      MakeTemperatureProtection(parameters.joint_temperature_protection[2]));

  if (joint1_motor_ == nullptr || joint2_motor_ == nullptr ||
      joint3_motor_ == nullptr) {
    return false;
  }

  for (size_t index = 0; index < ARM_JOINT_COUNT; ++index) {
    kp_[index] = FiniteOr(parameters.joint_kp[index], 0.0f);
    kd_[index] = FiniteOr(parameters.joint_kd[index], 0.0f);
    gravity_scale_[index] = FiniteOr(parameters.gravity_comp_scale[index], 1.0f);
  }
  initialized_ = true;
  return true;
}

int8_t ArmOutputAdapter::FirstFailure(int8_t current, int8_t next) const {
  return (current != DEVICE_OK) ? current : next;
}

int8_t ArmOutputAdapter::Register() {
  if (!initialized_) {
    return DEVICE_ERR_NULL;
  }
  int8_t result = joint1_motor_->Register();
  result = FirstFailure(result, joint2_motor_->Register());
  result = FirstFailure(result, joint3_motor_->Register());
  return result;
}

int8_t ArmOutputAdapter::Enable() {
  if (!initialized_) {
    return DEVICE_ERR_NULL;
  }
  int8_t result = joint1_motor_->Enable();
  result = FirstFailure(result, joint2_motor_->Enable());
  result = FirstFailure(result, joint3_motor_->Enable());
  return result;
}

int8_t ArmOutputAdapter::Relax() {
  if (!initialized_) {
    return DEVICE_ERR_NULL;
  }
  joint1_motor_->ClearPendingCommand();
  joint2_motor_->ClearPendingCommand();
  joint3_motor_->ClearPendingCommand();
  int8_t result = joint1_motor_->Relax();
  result = FirstFailure(result, joint2_motor_->Relax());
  result = FirstFailure(result, joint3_motor_->Relax());
  return result;
}

int8_t ArmOutputAdapter::Relax(size_t index) {
  if (!initialized_) {
    return DEVICE_ERR_NULL;
  }
  switch (index) {
    case 0U:
      joint1_motor_->ClearPendingCommand();
      return joint1_motor_->Relax();
    case 1U:
      joint2_motor_->ClearPendingCommand();
      return joint2_motor_->Relax();
    case 2U:
      joint3_motor_->ClearPendingCommand();
      return joint3_motor_->Relax();
    default:
      return DEVICE_ERR_NULL;
  }
}

mr::motor::MotorState ArmOutputAdapter::MotorState(size_t index) const {
  if (index == 0U && joint1_motor_ != nullptr) {
    return joint1_motor_->GetState();
  }
  if (index == 1U && joint2_motor_ != nullptr) {
    return joint2_motor_->GetState();
  }
  if (index == 2U && joint3_motor_ != nullptr) {
    return joint3_motor_->GetState();
  }
  return mr::motor::MotorState{};
}

bool ArmOutputAdapter::IsOnline(size_t index) const {
  return MotorState(index).online;
}

int8_t ArmOutputAdapter::UpdateFeedback(ThreeAxisJointState* state) {
  if (!initialized_ || state == nullptr) {
    return DEVICE_ERR_NULL;
  }

  int8_t result = joint1_motor_->Update();
  result = FirstFailure(result, joint2_motor_->Update());
  result = FirstFailure(result, joint3_motor_->Update());

  state->valid = true;
  for (size_t index = 0; index < ARM_JOINT_COUNT; ++index) {
    const mr::motor::MotorState motor_state = MotorState(index);
    state->position[index][0] = motor_state.position_rad;
    state->velocity[index][0] = motor_state.velocity_rad_s;
    state->effort[index][0] = motor_state.torque_nm;
    state->valid = state->valid && motor_state.online &&
                   mr::component::math::is_finite_scalar(
                       motor_state.position_rad) &&
                   mr::component::math::is_finite_scalar(
                       motor_state.velocity_rad_s) &&
                   mr::component::math::is_finite_scalar(motor_state.torque_nm);
  }
  state->valid = state->valid && result == DEVICE_OK;
  return result;
}

int8_t ArmOutputAdapter::ApplyJoint(
    size_t index,
    const ThreeAxisJointCommand& command) {
  const float torque_feedforward =
      command.effort_feedforward[index][0] * gravity_scale_[index];
  if (command.mode == mr::robotics::arm::JointCommandMode::kEffort) {
    switch (index) {
      case 0U:
        return joint1_motor_->SetTorque(torque_feedforward);
      case 1U:
        return joint2_motor_->SetTorque(torque_feedforward);
      case 2U:
        return joint3_motor_->SetTorque(torque_feedforward);
      default:
        return DEVICE_ERR_NULL;
    }
  }
  switch (index) {
    case 0U:
      return joint1_motor_->SetMIT(command.position[index][0],
                                   command.velocity[index][0], kp_[index],
                                   kd_[index], torque_feedforward);
    case 1U:
      return joint2_motor_->SetMIT(command.position[index][0],
                                   command.velocity[index][0], kp_[index],
                                   kd_[index], torque_feedforward);
    case 2U:
      return joint3_motor_->SetMIT(command.position[index][0],
                                   command.velocity[index][0], kp_[index],
                                   kd_[index], torque_feedforward);
    default:
      return DEVICE_ERR_NULL;
  }
}

int8_t ArmOutputAdapter::Apply(const ThreeAxisJointCommand& command) {
  if (!initialized_ || !command.valid) {
    return DEVICE_ERR;
  }
  if (command.mode == mr::robotics::arm::JointCommandMode::kDisabled) {
    return Relax();
  }

  int8_t result = DEVICE_OK;
  for (size_t index = 0; index < ARM_JOINT_COUNT; ++index) {
    result = FirstFailure(result, ApplyJoint(index, command));
  }
  return result;
}

int8_t ArmOutputAdapter::Commit() {
  if (!initialized_) {
    return DEVICE_ERR_NULL;
  }
  int8_t result = joint1_motor_->CommitCommand();
  result = FirstFailure(result, joint2_motor_->CommitCommand());
  result = FirstFailure(result, joint3_motor_->CommitCommand());
  return result;
}

int8_t ArmOutputAdapter::SetZero(size_t index) {
  if (!initialized_) {
    return DEVICE_ERR_NULL;
  }
  switch (index) {
    case 0U:
      return joint1_motor_->SetZero();
    case 1U:
      return joint2_motor_->SetZero();
    case 2U:
      return joint3_motor_->SetZero();
    default:
      return DEVICE_ERR_NULL;
  }
}

}  // namespace mr::arm
