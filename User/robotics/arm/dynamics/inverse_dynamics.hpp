#pragma once

// Fixed-size recursive Newton-Euler inverse dynamics. The result is the
// actuator effort required to produce the requested joint acceleration while
// compensating gravity, Coriolis, centrifugal, and link-inertia effects.

#include "component/math/matrix.hpp"
#include "component/math/scalar.hpp"
#include "robotics/core/rotation.hpp"
#include "robotics/core/transform.hpp"
#include "../core/status.hpp"
#include "../core/types.hpp"
#include "../kinematics/forward_kinematics.hpp"
#include "../model/arm_model.hpp"

namespace mr::robotics::arm::dynamics {

template <int DoF>
struct DynamicCompensationResult {
  Status status;
  JointVector<DoF> effort;

  DynamicCompensationResult()
      : status(Status::kInvalidInput),
        effort(mr::component::math::matrix::zeros<DoF, 1>()) {}
};

namespace inverse_dynamics_internal {

inline Scalar Dot(const Vec3& lhs, const Vec3& rhs) {
  return mr::component::math::matrix::dot(lhs, rhs);
}

inline Vec3 Cross(const Vec3& lhs, const Vec3& rhs) {
  return mr::component::math::vector3f::cross(lhs, rhs);
}

inline Vec3 TransformPoint(const Transform& transform, const Vec3& point) {
  return mr::robotics::core::RotationOf(transform) * point +
         mr::robotics::core::TranslationOf(transform);
}

}  // namespace inverse_dynamics_internal

template <int DoF>
inline DynamicCompensationResult<DoF> ComputeDynamicCompensation(
    const ArmModel<DoF>& model,
    const JointVector<DoF>& position,
    const JointVector<DoF>& velocity,
    const JointVector<DoF>& acceleration) {
  DynamicCompensationResult<DoF> result;
  if (!IsModelValid(model)) {
    result.status = Status::kInvalidModel;
    return result;
  }
  if (!IsJointVectorFinite(position) || !IsJointVectorFinite(velocity) ||
      !IsJointVectorFinite(acceleration)) {
    result.status = Status::kInvalidInput;
    return result;
  }

  kinematics::KinematicFrames<DoF> frames;
  if (!kinematics::ComputeKinematicFrames(model, position, &frames)) {
    result.status = Status::kNumericalFailure;
    return result;
  }

  Vec3 link_force[DoF];
  Vec3 link_moment[DoF];

  Vec3 parent_angular_velocity =
      mr::component::math::matrix::zeros<3, 1>();
  Vec3 parent_angular_acceleration =
      mr::component::math::matrix::zeros<3, 1>();
  Vec3 parent_origin_acceleration = -model.Gravity();
  Vec3 parent_origin =
      mr::robotics::core::TranslationOf(model.BaseTransform());

  for (int index = 0; index < DoF; ++index) {
    const JointModel& joint = model.Joint(index);
    const Vec3& axis = frames.joint_axis_world[index];
    const Vec3 joint_origin = frames.joint_origin_world[index];
    const Vec3 link_origin =
        mr::robotics::core::TranslationOf(frames.link_frame[index]);
    const Vec3 parent_to_joint = joint_origin - parent_origin;

    Vec3 link_origin_acceleration =
        parent_origin_acceleration +
        inverse_dynamics_internal::Cross(parent_angular_acceleration,
                                          parent_to_joint) +
        inverse_dynamics_internal::Cross(
            parent_angular_velocity,
            inverse_dynamics_internal::Cross(parent_angular_velocity,
                                              parent_to_joint));

    Vec3 angular_velocity = parent_angular_velocity;
    Vec3 angular_acceleration = parent_angular_acceleration;
    if (joint.type == JointType::kRevolute) {
      angular_velocity += axis * velocity[index][0];
      angular_acceleration +=
          axis * acceleration[index][0] +
          inverse_dynamics_internal::Cross(
              parent_angular_velocity, axis * velocity[index][0]);
    } else {
      const Vec3 joint_to_link = link_origin - joint_origin;
      link_origin_acceleration +=
          inverse_dynamics_internal::Cross(parent_angular_acceleration,
                                            joint_to_link) +
          inverse_dynamics_internal::Cross(
              parent_angular_velocity,
              inverse_dynamics_internal::Cross(parent_angular_velocity,
                                                joint_to_link)) +
          2.0f * inverse_dynamics_internal::Cross(
                     parent_angular_velocity, axis * velocity[index][0]) +
          axis * acceleration[index][0];
    }

    const InertialParameters& inertial = joint.inertial;
    const Vec3 center_of_mass_position =
        inverse_dynamics_internal::TransformPoint(
            frames.link_frame[index], inertial.center_of_mass);
    const Vec3 origin_to_center =
        center_of_mass_position - link_origin;
    const Vec3 center_acceleration =
        link_origin_acceleration +
        inverse_dynamics_internal::Cross(angular_acceleration,
                                          origin_to_center) +
        inverse_dynamics_internal::Cross(
            angular_velocity,
            inverse_dynamics_internal::Cross(angular_velocity,
                                              origin_to_center));

    link_force[index] = center_acceleration * inertial.mass_kg;
    const Rotation link_rotation =
        mr::robotics::core::RotationOf(frames.link_frame[index]);
    const Rotation inertia_world =
        link_rotation * inertial.inertia_at_center_of_mass *
        link_rotation.transpose();
    const Vec3 angular_momentum = inertia_world * angular_velocity;
    link_moment[index] =
        inertia_world * angular_acceleration +
        inverse_dynamics_internal::Cross(angular_velocity,
                                          angular_momentum) +
        inverse_dynamics_internal::Cross(origin_to_center,
                                          link_force[index]);

    parent_angular_velocity = angular_velocity;
    parent_angular_acceleration = angular_acceleration;
    parent_origin_acceleration = link_origin_acceleration;
    parent_origin = link_origin;
  }

  Vec3 accumulated_force = mr::component::math::matrix::zeros<3, 1>();
  Vec3 accumulated_moment = mr::component::math::matrix::zeros<3, 1>();
  Vec3 accumulated_origin =
      mr::robotics::core::TranslationOf(frames.link_frame[DoF - 1]);
  for (int index = DoF - 1; index >= 0; --index) {
    const Vec3 link_origin =
        mr::robotics::core::TranslationOf(frames.link_frame[index]);
    accumulated_moment +=
        inverse_dynamics_internal::Cross(accumulated_origin - link_origin,
                                          accumulated_force);
    accumulated_force += link_force[index];
    accumulated_moment += link_moment[index];

    const JointModel& joint = model.Joint(index);
    if (joint.type == JointType::kRevolute) {
      const Vec3 joint_to_link =
          link_origin - frames.joint_origin_world[index];
      const Vec3 moment_at_joint =
          accumulated_moment +
          inverse_dynamics_internal::Cross(joint_to_link,
                                            accumulated_force);
      result.effort[index][0] = inverse_dynamics_internal::Dot(
          frames.joint_axis_world[index], moment_at_joint);
    } else {
      result.effort[index][0] = inverse_dynamics_internal::Dot(
          frames.joint_axis_world[index], accumulated_force);
    }
    accumulated_origin = link_origin;
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
