#pragma once

// Fixed-size joint-to-actuator mapping owned by the execution layer.
// Kinematics works in joint coordinates; this type handles motor-side
// scaling, offsets, and a small number of explicit coupled sources.

#include <stdint.h>

namespace mr {

inline constexpr uint8_t kJointMaxCouplings = 4U;

enum class JointType : uint8_t {
  kRevolute = 0,
  kPrismatic,
  kFixed,
};

struct JointCouplingTerm {
  uint8_t source_joint = 0U;
  float coefficient = 0.0f;
  bool enabled = false;
};

struct JointTransmission {
  float actuator_scale = 1.0f;
  float actuator_offset = 0.0f;
  JointCouplingTerm couplings[kJointMaxCouplings]{};
  uint8_t coupling_count = 0U;

  void ClearCouplings() {
    coupling_count = 0U;
    for (uint8_t index = 0U; index < kJointMaxCouplings; ++index) {
      couplings[index] = JointCouplingTerm{};
    }
  }

  bool AddCoupling(uint8_t source_joint, float coefficient) {
    if (coupling_count >= kJointMaxCouplings) {
      return false;
    }
    couplings[coupling_count] =
        JointCouplingTerm{source_joint, coefficient, true};
    ++coupling_count;
    return true;
  }
};

inline float JointToActuatorPosition(const JointTransmission& transmission,
                                     float joint_position,
                                     float coupled_position,
                                     float calibration_offset = 0.0f) {
  return transmission.actuator_scale * joint_position +
         transmission.actuator_offset + coupled_position +
         calibration_offset;
}

inline float JointToActuatorVelocity(const JointTransmission& transmission,
                                     float joint_velocity,
                                     float coupled_velocity) {
  return transmission.actuator_scale * joint_velocity + coupled_velocity;
}

inline float ActuatorToJointPosition(const JointTransmission& transmission,
                                     float actuator_position,
                                     float coupled_position,
                                     float calibration_offset = 0.0f) {
  float joint_position = actuator_position - calibration_offset -
                         transmission.actuator_offset - coupled_position;
  if (transmission.actuator_scale > 1.0e-6f ||
      transmission.actuator_scale < -1.0e-6f) {
    joint_position /= transmission.actuator_scale;
  } else {
    joint_position = 0.0f;
  }
  return joint_position;
}

inline float ActuatorToJointVelocity(const JointTransmission& transmission,
                                     float actuator_velocity,
                                     float coupled_velocity) {
  float joint_velocity = actuator_velocity - coupled_velocity;
  if (transmission.actuator_scale > 1.0e-6f ||
      transmission.actuator_scale < -1.0e-6f) {
    joint_velocity /= transmission.actuator_scale;
  } else {
    joint_velocity = 0.0f;
  }
  return joint_velocity;
}

}  // namespace mr
