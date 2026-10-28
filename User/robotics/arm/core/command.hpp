#pragma once

#include "status.hpp"
#include "types.hpp"

namespace mr::robotics::arm {

enum class ControlMode : uint8_t {
  kDisabled = 0,
  kHold,
  kMoveJoint,
  kMoveCartesian,
};

template <int DoF>
struct ArmCommand {
  ControlMode mode;
  JointVector<DoF> joint_target;
  Transform pose_target;
  JointVector<DoF> effort_feedforward;
  bool position_only;
  // A streaming Cartesian target is updated every control cycle. It uses a
  // bounded IK step instead of restarting a time-based Cartesian plan.
  bool streaming_cartesian;

  ArmCommand()
      : mode(ControlMode::kDisabled),
        joint_target(mr::component::math::matrix::zeros<DoF, 1>()),
        pose_target(mr::component::math::matrix::identity<4>()),
        effort_feedforward(mr::component::math::matrix::zeros<DoF, 1>()),
        position_only(true),
        streaming_cartesian(false) {}
};

template <int DoF>
struct ArmOutput {
  Status status;
  JointCommand<DoF> joint_command;
  Transform current_pose;
  bool reached;
  bool position_only_fallback;
  bool near_singularity;

  ArmOutput()
      : status(Status::kNotConfigured),
        joint_command(),
        current_pose(mr::component::math::matrix::identity<4>()),
        reached(false),
        position_only_fallback(false),
        near_singularity(false) {}
};

template <int DoF>
inline bool IsArmCommandFinite(const ArmCommand<DoF>& command) {
  return IsJointVectorFinite(command.joint_target) &&
         mr::component::math::matrix::is_finite(command.pose_target) &&
         IsJointVectorFinite(command.effort_feedforward);
}

}  // namespace mr::robotics::arm
