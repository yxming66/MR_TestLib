#pragma once

// Explicit, stateless validation for joint feedback and output commands.
// Incoming targets are rejected by callers when unsafe; ClampJointPosition is
// intended only for a sampled trajectory at the final output boundary.

#include "component/math/matrix.hpp"
#include "component/math/scalar.hpp"
#include "../core/types.hpp"
#include "../model/arm_model.hpp"

namespace mr::robotics::arm::safety {

inline bool IsJointCommandModeValid(JointCommandMode mode) {
  switch (mode) {
    case JointCommandMode::kDisabled:
    case JointCommandMode::kPosition:
    case JointCommandMode::kPositionVelocity:
    case JointCommandMode::kPositionVelocityEffort:
    case JointCommandMode::kEffort:
      return true;
    default:
      return false;
  }
}

template <int DoF>
inline bool IsJointPositionSafe(const ArmModel<DoF>& model,
                               const JointVector<DoF>& position) {
  if (!IsModelValid(model) || !IsJointVectorFinite(position)) {
    return false;
  }
  for (int index = 0; index < DoF; ++index) {
    const JointLimits& limits = model.Joint(index).limits;
    if (limits.position_enabled &&
        (position[index][0] < limits.position_min ||
         position[index][0] > limits.position_max)) {
      return false;
    }
  }
  return true;
}

template <int DoF>
inline JointVector<DoF> ClampJointPosition(
    const ArmModel<DoF>& model,
    const JointVector<DoF>& position) {
  JointVector<DoF> result =
      mr::component::math::matrix::zeros<DoF, 1>();
  if (!IsModelValid(model) || !IsJointVectorFinite(position)) {
    return result;
  }

  result = position;
  for (int index = 0; index < DoF; ++index) {
    const JointLimits& limits = model.Joint(index).limits;
    if (limits.position_enabled) {
      result[index][0] = mr::component::math::clamp_scalar(
          result[index][0], limits.position_min, limits.position_max);
    }
  }
  return result;
}

template <int DoF>
inline bool IsCommandFinite(const JointCommand<DoF>& command) {
  return IsJointCommandModeValid(command.mode) &&
         IsJointCommandFinite(command);
}

template <int DoF>
inline bool IsJointVelocitySafe(const ArmModel<DoF>& model,
                               const JointVector<DoF>& velocity) {
  if (!IsModelValid(model) || !IsJointVectorFinite(velocity)) {
    return false;
  }
  for (int index = 0; index < DoF; ++index) {
    if (mr::component::math::abs_scalar(velocity[index][0]) >
        model.Joint(index).limits.max_velocity) {
      return false;
    }
  }
  return true;
}

template <int DoF>
inline bool IsJointAccelerationSafe(const ArmModel<DoF>& model,
                                   const JointVector<DoF>& acceleration) {
  if (!IsModelValid(model) || !IsJointVectorFinite(acceleration)) {
    return false;
  }
  for (int index = 0; index < DoF; ++index) {
    if (mr::component::math::abs_scalar(acceleration[index][0]) >
        model.Joint(index).limits.max_acceleration) {
      return false;
    }
  }
  return true;
}

template <int DoF>
inline bool IsJointCommandWithinLimits(const ArmModel<DoF>& model,
                                       const JointCommand<DoF>& command) {
  return IsCommandFinite(command) &&
         IsJointPositionSafe(model, command.position) &&
         IsJointVelocitySafe(model, command.velocity) &&
         IsJointAccelerationSafe(model, command.acceleration);
}

}  // namespace mr::robotics::arm::safety
