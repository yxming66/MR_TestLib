#pragma once

// Static gravity compensation only. The returned effort is the actuator
// effort that opposes gravity: effort = -Jv^T * (mass * gravity).
// Gravity is expressed in the model world frame and joint axes are evaluated
// in that same frame. The center of mass is expressed in each link frame.

#include "component/math/matrix.hpp"
#include "component/math/scalar.hpp"
#include "robotics/core/transform.hpp"
#include "../core/status.hpp"
#include "../core/types.hpp"
#include "../kinematics/forward_kinematics.hpp"
#include "../model/arm_model.hpp"

namespace mr::robotics::arm::dynamics {

template <int DoF>
struct GravityResult {
  Status status;
  JointVector<DoF> effort;

  GravityResult()
      : status(Status::kInvalidInput),
        effort(mr::component::math::matrix::zeros<DoF, 1>()) {}
};

namespace internal {

inline Vec3 TransformPoint(const Transform& transform, const Vec3& point) {
  return mr::robotics::core::RotationOf(transform) * point +
         mr::robotics::core::TranslationOf(transform);
}

template <int DoF>
inline bool AddLinkGravity(const ArmModel<DoF>& model,
                          const kinematics::KinematicFrames<DoF>& frames,
                          int link_index,
                          const Vec3& force,
                          JointVector<DoF>* effort) {
  const JointModel& link = model.Joint(link_index);
  if (link.inertial.mass_kg <= 0.0f) {
    return true;
  }

  const Vec3 center_of_mass_world =
      TransformPoint(frames.link_frame[link_index],
                     link.inertial.center_of_mass);
  if (!mr::component::math::matrix::is_finite(center_of_mass_world)) {
    return false;
  }

  for (int joint_index = 0; joint_index <= link_index; ++joint_index) {
    const JointModel& joint = model.Joint(joint_index);
    const Vec3& axis_world = frames.joint_axis_world[joint_index];
    const Vec3& origin_world = frames.joint_origin_world[joint_index];
    Scalar generalized_gravity = 0.0f;
    if (joint.type == JointType::kRevolute) {
      const Vec3 linear_velocity =
          mr::component::math::vector3f::cross(
              axis_world, center_of_mass_world - origin_world);
      generalized_gravity =
          linear_velocity[0][0] * force[0][0] +
          linear_velocity[1][0] * force[1][0] +
          linear_velocity[2][0] * force[2][0];
    } else if (joint.type == JointType::kPrismatic) {
      generalized_gravity = axis_world[0][0] * force[0][0] +
                            axis_world[1][0] * force[1][0] +
                            axis_world[2][0] * force[2][0];
    } else {
      return false;
    }
    (*effort)[joint_index][0] -= generalized_gravity;
  }
  return true;
}

}  // namespace internal

template <int DoF>
inline GravityResult<DoF> ComputeGravityCompensation(
    const ArmModel<DoF>& model,
    const JointVector<DoF>& position) {
  GravityResult<DoF> result;
  if (!IsModelValid(model) || !IsJointVectorFinite(position)) {
    result.status = IsModelValid(model) ? Status::kInvalidInput
                                        : Status::kInvalidModel;
    return result;
  }

  kinematics::KinematicFrames<DoF> frames;
  if (!kinematics::ComputeKinematicFrames(model, position, &frames)) {
    result.status = Status::kNumericalFailure;
    return result;
  }

  const Vec3 gravity_force = model.Gravity();
  for (int link_index = 0; link_index < DoF; ++link_index) {
    const Scalar mass = model.Joint(link_index).inertial.mass_kg;
    if (mass > 0.0f) {
      Vec3 force_vector = gravity_force * mass;
      if (!internal::AddLinkGravity(model, frames, link_index,
                                    force_vector, &result.effort)) {
        result.status = Status::kNumericalFailure;
        result.effort = mr::component::math::matrix::zeros<DoF, 1>();
        return result;
      }
    }
  }

  if (!IsJointVectorFinite(result.effort)) {
    result.effort = mr::component::math::matrix::zeros<DoF, 1>();
    result.status = Status::kNumericalFailure;
    return result;
  }
  result.status = Status::kOk;
  return result;
}

}  // namespace mr::robotics::arm::dynamics
