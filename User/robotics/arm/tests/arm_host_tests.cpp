#include <cmath>
#include <cstdio>
#include <limits>

#include "robotics/arm/arm.hpp"

namespace {

using mr::component::math::Scalar;
using mr::component::math::matrix::identity;
using mr::robotics::arm::ArmModel;
using mr::robotics::arm::JointModel;
using mr::robotics::arm::JointType;
using mr::robotics::arm::JointVector;
using mr::robotics::arm::IsJointVectorFinite;
using mr::robotics::arm::Transform;
using mr::robotics::arm::kinematics::ComputeJacobian;
using mr::robotics::arm::kinematics::ComputeKinematicFrames;
using mr::robotics::arm::kinematics::ComputePoseError;
using mr::robotics::arm::kinematics::ForwardKinematics;
using mr::robotics::arm::kinematics::InverseKinematicsRequest;
using mr::robotics::arm::kinematics::MakePositionOnlyInverseKinematicsOptions;
using mr::robotics::arm::kinematics::SolveInverseKinematics;
using mr::robotics::arm::planning::CartesianPlanRequest;
using mr::robotics::arm::planning::CartesianPlanner;
using mr::robotics::arm::planning::JointPlanRequest;
using mr::robotics::arm::planning::JointPlanner;
using mr::robotics::arm::safety::ClampJointPosition;
using mr::robotics::arm::safety::IsCommandFinite;
using mr::robotics::arm::safety::IsJointCommandWithinLimits;
using mr::robotics::arm::safety::IsJointPositionSafe;
using mr::robotics::arm::dynamics::ComputeDynamicCompensation;
using mr::robotics::arm::dynamics::ComputeGravityCompensation;
using mr::robotics::arm::control::ArmController;
using mr::robotics::arm::control::ArmControllerConfig;

int failures = 0;

void Expect(bool condition, const char* message) {
  if (!condition) {
    std::printf("FAIL: %s\n", message);
    ++failures;
  }
}

bool Near(Scalar lhs, Scalar rhs, Scalar tolerance = 1.0e-4f) {
  return std::fabs(lhs - rhs) <= tolerance;
}

}  // namespace

int main() {
  JointModel joints[2];
  for (int index = 0; index < 2; ++index) {
    joints[index].type = JointType::kRevolute;
    joints[index].parent_to_joint = identity<4>();
    joints[index].parent_to_joint[0][3] = 1.0f;
    joints[index].limits.position_enabled = true;
    joints[index].limits.position_min = -3.14f;
    joints[index].limits.position_max = 3.14f;
    joints[index].limits.max_velocity = 2.0f;
    joints[index].limits.max_acceleration = 4.0f;
  }

  mr::robotics::core::Vec3 gravity =
      mr::component::math::matrix::zeros<3, 1>();
  gravity[2][0] = -9.81f;
  ArmModel<2> model(joints, identity<4>(), identity<4>(), gravity);
  Expect(model.Validate(), "planar model should be valid");

  // Public RPY vectors use the conventional [roll, pitch, yaw] layout while
  // composing Rz(yaw) * Ry(pitch) * Rx(roll).
  mr::robotics::core::Vec3 input_rpy =
      mr::component::math::matrix::zeros<3, 1>();
  input_rpy[0][0] = 0.20f;
  input_rpy[1][0] = -0.30f;
  input_rpy[2][0] = 0.40f;
  const auto rpy_rotation =
      mr::robotics::core::RotationFromRpy(input_rpy);
  const auto recovered_rpy =
      mr::robotics::core::RpyFromRotation(rpy_rotation);
  Expect(Near(recovered_rpy[0][0], input_rpy[0][0]) &&
             Near(recovered_rpy[1][0], input_rpy[1][0]) &&
             Near(recovered_rpy[2][0], input_rpy[2][0]),
         "RPY round trip uses roll pitch yaw vector order");

  JointModel invalid_type_joint = joints[0];
  invalid_type_joint.type = static_cast<JointType>(99U);
  Expect(!mr::robotics::arm::ValidateJointModel(invalid_type_joint),
         "unknown joint type should invalidate the model");

  JointModel asymmetric_inertia_joint = joints[0];
  asymmetric_inertia_joint.inertial.inertia_at_center_of_mass[0][1] = 0.1f;
  Expect(!mr::robotics::arm::ValidateJointModel(asymmetric_inertia_joint),
         "asymmetric inertia tensor should invalidate the model");

  JointModel indefinite_inertia_joint = joints[0];
  indefinite_inertia_joint.inertial.inertia_at_center_of_mass[0][0] = 1.0f;
  indefinite_inertia_joint.inertial.inertia_at_center_of_mass[1][1] = 1.0f;
  indefinite_inertia_joint.inertial.inertia_at_center_of_mass[0][1] = 2.0f;
  indefinite_inertia_joint.inertial.inertia_at_center_of_mass[1][0] = 2.0f;
  Expect(!mr::robotics::arm::ValidateJointModel(indefinite_inertia_joint),
         "indefinite inertia tensor should invalidate the model");

  JointVector<2> position =
      mr::component::math::matrix::zeros<2, 1>();
  const auto pose = ForwardKinematics(model, position);
  Expect(Near(pose[0][3], 2.0f), "zero pose x position");
  Expect(Near(pose[1][3], 0.0f), "zero pose y position");

  position[0][0] = 0.5f;
  const auto rotated_pose = ForwardKinematics(model, position);
  Expect(Near(rotated_pose[0][3], 1.87758f, 2.0e-4f),
         "rotated pose x position");
  Expect(Near(rotated_pose[1][3], 0.47943f, 2.0e-4f),
         "rotated pose y position");

  const auto jacobian = ComputeJacobian(model, position);
  Expect(Near(jacobian(0, 0), -0.47943f, 2.0e-4f),
         "jacobian linear x");
  Expect(Near(jacobian(1, 0), 0.87758f, 2.0e-4f),
         "jacobian linear y");
  Expect(Near(jacobian(5, 0), 1.0f), "jacobian angular z");

  // Compare the translational Jacobian against a central finite difference.
  // This catches frame, axis, and cross-product sign errors in one small test.
  constexpr Scalar kFiniteDifferenceStep = 1.0e-4f;
  for (int column = 0; column < 2; ++column) {
    JointVector<2> plus = position;
    JointVector<2> minus = position;
    plus[column][0] += kFiniteDifferenceStep;
    minus[column][0] -= kFiniteDifferenceStep;
    const Transform plus_pose = ForwardKinematics(model, plus);
    const Transform minus_pose = ForwardKinematics(model, minus);
    for (int row = 0; row < 3; ++row) {
      const Scalar numerical_derivative =
          (plus_pose(row, 3) - minus_pose(row, 3)) /
          (2.0f * kFiniteDifferenceStep);
      Expect(Near(numerical_derivative, jacobian(row, column), 2.0e-3f),
             "jacobian translational finite difference");
    }
  }

  mr::robotics::arm::kinematics::KinematicFrames<2> frames;
  Expect(ComputeKinematicFrames(model, position, &frames) && frames.valid,
         "kinematic frames should be valid");
  JointVector<2> invalid_position = position;
  invalid_position[0][0] = std::numeric_limits<Scalar>::quiet_NaN();
  Expect(!ComputeKinematicFrames(model, invalid_position, &frames) &&
             !frames.valid,
         "failed frame computation should clear valid flag");

  const auto error = ComputePoseError(rotated_pose, pose);
  Expect(error.valid, "pose error should be valid");
  Expect(error.position_norm > 0.1f, "pose error should detect translation");
  Expect(error.orientation_norm > 0.1f, "pose error should detect rotation");

  JointVector<2> target_position =
      mr::component::math::matrix::zeros<2, 1>();
  target_position[0][0] = 0.45f;
  target_position[1][0] = -0.30f;
  const auto target_pose = ForwardKinematics(model, target_position);

  InverseKinematicsRequest<2> request;
  request.target_pose = target_pose;
  request.initial_position =
      mr::component::math::matrix::zeros<2, 1>();
  auto ik_options = MakePositionOnlyInverseKinematicsOptions();
  ik_options.max_iterations = 64;
  ik_options.position_tolerance = 1.0e-4f;
  ik_options.max_joint_step = 0.20f;
  const auto ik_result = SolveInverseKinematics(model, request, ik_options);
  Expect(ik_result.status == mr::robotics::arm::Status::kOk,
         "position-only inverse kinematics should converge");
  const auto solved_pose = ForwardKinematics(model, ik_result.position);
  Expect(Near(solved_pose[0][3], target_pose[0][3], 2.0e-3f),
         "inverse kinematics solved x position");
  Expect(Near(solved_pose[1][3], target_pose[1][3], 2.0e-3f),
         "inverse kinematics solved y position");

  auto invalid_ik_options = ik_options;
  invalid_ik_options.max_joint_step = 0.0f;
  const auto invalid_ik_result =
      SolveInverseKinematics(model, request, invalid_ik_options);
  Expect(invalid_ik_result.status == mr::robotics::arm::Status::kInvalidInput,
         "inverse kinematics rejects zero joint step");

  invalid_ik_options = ik_options;
  invalid_ik_options.task_weights[0][0] = -1.0f;
  const auto negative_weight_result =
      SolveInverseKinematics(model, request, invalid_ik_options);
  Expect(negative_weight_result.status ==
             mr::robotics::arm::Status::kInvalidInput,
         "inverse kinematics rejects negative task weight");

  InverseKinematicsRequest<2> unreachable_request = request;
  unreachable_request.target_pose = identity<4>();
  unreachable_request.target_pose[0][3] = 10.0f;
  const auto unreachable_result =
      SolveInverseKinematics(model, unreachable_request, ik_options);
  Expect(unreachable_result.status == mr::robotics::arm::Status::kNoConvergence,
         "unreachable target should report no convergence");

  InverseKinematicsRequest<2> invalid_start_request = request;
  invalid_start_request.initial_position[0][0] = 4.0f;
  const auto invalid_start_result =
      SolveInverseKinematics(model, invalid_start_request, ik_options);
  Expect(invalid_start_result.status ==
             mr::robotics::arm::Status::kLimitViolation,
         "out-of-limit initial position should be rejected");

  InverseKinematicsRequest<2> nonfinite_start_request = request;
  nonfinite_start_request.initial_position[0][0] =
      std::numeric_limits<Scalar>::quiet_NaN();
  const auto nonfinite_start_result =
      SolveInverseKinematics(model, nonfinite_start_request, ik_options);
  Expect(nonfinite_start_result.status ==
             mr::robotics::arm::Status::kInvalidInput &&
             IsJointVectorFinite(nonfinite_start_result.position),
         "inverse kinematics failure keeps a finite result position");

  JointPlanRequest<2> joint_plan_request;
  joint_plan_request.start = mr::component::math::matrix::zeros<2, 1>();
  joint_plan_request.goal = mr::component::math::matrix::zeros<2, 1>();
  joint_plan_request.goal[0][0] = 1.0f;
  joint_plan_request.goal[1][0] = -0.25f;
  joint_plan_request.max_velocity[0][0] = 1.0f;
  joint_plan_request.max_velocity[1][0] = 0.5f;
  joint_plan_request.max_acceleration[0][0] = 2.0f;
  joint_plan_request.max_acceleration[1][0] = 1.0f;
  JointPlanner<2> joint_planner;
  Expect(joint_planner.Configure(joint_plan_request),
         "joint planner should accept a valid request");
  const auto joint_start_sample = joint_planner.Sample(0.0f);
  Expect(joint_start_sample.valid && !joint_start_sample.finished,
         "joint planner start sample");
  Expect(Near(joint_start_sample.position[0][0], 0.0f),
         "joint planner starts at first joint");
  Expect(Near(joint_start_sample.position[1][0], 0.0f),
         "joint planner starts at second joint");
  for (int sample_index = 0; sample_index <= 20; ++sample_index) {
    const Scalar elapsed =
        joint_planner.Duration() * static_cast<Scalar>(sample_index) / 20.0f;
    const auto sample = joint_planner.Sample(elapsed);
    Expect(sample.valid, "joint planner samples are valid");
    Expect(std::fabs(sample.velocity[0][0]) <=
               joint_plan_request.max_velocity[0][0] + 1.0e-4f,
           "joint planner velocity limit axis one");
    Expect(std::fabs(sample.velocity[1][0]) <=
               joint_plan_request.max_velocity[1][0] + 1.0e-4f,
           "joint planner velocity limit axis two");
    Expect(std::fabs(sample.acceleration[0][0]) <=
               joint_plan_request.max_acceleration[0][0] + 1.0e-4f,
           "joint planner acceleration limit axis one");
    Expect(std::fabs(sample.acceleration[1][0]) <=
               joint_plan_request.max_acceleration[1][0] + 1.0e-4f,
           "joint planner acceleration limit axis two");
  }
  const auto joint_end_sample =
      joint_planner.Sample(joint_planner.Duration() + 0.1f);
  Expect(joint_end_sample.finished, "joint planner end sample");
  Expect(Near(joint_end_sample.position[0][0], 1.0f),
         "joint planner reaches first goal");
  Expect(Near(joint_end_sample.position[1][0], -0.25f),
         "joint planner reaches second goal");

  JointPlanRequest<2> invalid_joint_plan = joint_plan_request;
  invalid_joint_plan.max_velocity[1][0] = 0.0f;
  JointPlanner<2> invalid_joint_planner;
  Expect(!invalid_joint_planner.Configure(invalid_joint_plan),
         "joint planner rejects zero velocity limit");
  Expect(!invalid_joint_planner.Sample(0.1f).valid,
         "invalid joint planner stays invalid");

  CartesianPlanRequest cartesian_request;
  cartesian_request.start = identity<4>();
  mr::robotics::core::Vec3 translation_vector =
      mr::component::math::matrix::zeros<3, 1>();
  translation_vector[0][0] = 0.30f;
  const auto translation =
      mr::robotics::core::TransformFromTranslation(translation_vector);
  mr::robotics::core::Vec3 rotation_axis =
      mr::component::math::matrix::zeros<3, 1>();
  rotation_axis[2][0] = 1.0f;
  const auto rotation = mr::robotics::core::TransformFromAxisAngle(
      rotation_axis, 0.4f);
  cartesian_request.goal = translation * rotation;
  CartesianPlanner cartesian_planner;
  Expect(cartesian_planner.Configure(cartesian_request),
         "Cartesian planner should accept a valid request");
  const auto cartesian_start = cartesian_planner.Sample(0.0f);
  Expect(cartesian_start.valid && !cartesian_start.finished,
         "Cartesian planner start sample");
  Expect(Near(cartesian_start.pose[0][3], cartesian_request.start[0][3]),
         "Cartesian planner starts at requested pose");
  const auto cartesian_mid =
      cartesian_planner.Sample(0.5f * cartesian_planner.Duration());
  Expect(cartesian_mid.valid, "Cartesian planner midpoint sample");
  const auto cartesian_end =
      cartesian_planner.Sample(cartesian_planner.Duration() + 0.1f);
  Expect(cartesian_end.finished, "Cartesian planner end sample");
  for (int row = 0; row < 4; ++row) {
    for (int col = 0; col < 4; ++col) {
      Expect(Near(cartesian_end.pose(row, col),
                  cartesian_request.goal(row, col), 2.0e-4f),
             "Cartesian planner reaches goal pose");
    }
  }

  CartesianPlanRequest invalid_cartesian_request = cartesian_request;
  invalid_cartesian_request.max_linear_velocity_m_s = -1.0f;
  CartesianPlanner invalid_cartesian_planner;
  Expect(!invalid_cartesian_planner.Configure(invalid_cartesian_request),
         "Cartesian planner rejects invalid velocity");

  CartesianPlanRequest zero_distance_cartesian = cartesian_request;
  zero_distance_cartesian.goal = zero_distance_cartesian.start;
  zero_distance_cartesian.max_linear_velocity_m_s = 0.0f;
  Expect(!invalid_cartesian_planner.Configure(zero_distance_cartesian),
         "Cartesian planner validates limits for zero distance");

  JointVector<2> safe_position =
      mr::component::math::matrix::zeros<2, 1>();
  safe_position[0][0] = 4.0f;
  Expect(!IsJointPositionSafe(model, safe_position),
         "unsafe joint position should be rejected");
  const JointVector<2> clamped_position = ClampJointPosition(model, safe_position);
  Expect(Near(clamped_position[0][0], 3.14f),
         "joint position clamp should use model limit");

  mr::robotics::arm::JointCommand<2> joint_command;
  joint_command.valid = true;
  joint_command.mode = mr::robotics::arm::JointCommandMode::kPositionVelocity;
  joint_command.position =
      mr::component::math::matrix::zeros<2, 1>();
  joint_command.velocity =
      mr::component::math::matrix::zeros<2, 1>();
  joint_command.acceleration =
      mr::component::math::matrix::zeros<2, 1>();
  joint_command.effort_feedforward =
      mr::component::math::matrix::zeros<2, 1>();
  Expect(IsCommandFinite(joint_command), "finite joint command");
  Expect(IsJointCommandWithinLimits(model, joint_command),
         "joint command should satisfy model limits");
  joint_command.velocity[0][0] = 3.0f;
  Expect(!IsJointCommandWithinLimits(model, joint_command),
         "joint command velocity limit");
  joint_command.position[0][0] =
      std::numeric_limits<Scalar>::quiet_NaN();
  Expect(!IsCommandFinite(joint_command), "nonfinite joint command");

  JointModel gravity_joint;
  gravity_joint.limits.position_enabled = true;
  gravity_joint.limits.position_min = -3.14f;
  gravity_joint.limits.position_max = 3.14f;
  gravity_joint.limits.max_velocity = 2.0f;
  gravity_joint.limits.max_acceleration = 4.0f;
  gravity_joint.inertial.mass_kg = 1.0f;
  gravity_joint.inertial.center_of_mass[0][0] = 0.5f;
  gravity_joint.inertial.inertia_at_center_of_mass[2][2] = 0.1f;
  mr::robotics::core::Vec3 planar_gravity =
      mr::component::math::matrix::zeros<3, 1>();
  planar_gravity[1][0] = -9.81f;
  JointModel gravity_joints[1] = {gravity_joint};
  ArmModel<1> gravity_model(gravity_joints, identity<4>(), identity<4>(),
                            planar_gravity);
  JointVector<1> gravity_position =
      mr::component::math::matrix::zeros<1, 1>();
  const auto gravity_result =
      ComputeGravityCompensation(gravity_model, gravity_position);
  Expect(gravity_result.status == mr::robotics::arm::Status::kOk,
         "gravity compensation should accept planar model");
  Expect(Near(gravity_result.effort[0][0], 4.905f, 2.0e-3f),
         "gravity compensation should oppose downward torque");
  gravity_position[0][0] = 0.5f * mr::component::math::kPi;
  const auto vertical_gravity_result =
      ComputeGravityCompensation(gravity_model, gravity_position);
  Expect(Near(vertical_gravity_result.effort[0][0], 0.0f, 2.0e-3f),
         "gravity compensation at vertical pose");

  gravity_position[0][0] = 0.0f;
  JointVector<1> gravity_velocity =
      mr::component::math::matrix::zeros<1, 1>();
  JointVector<1> gravity_acceleration =
      mr::component::math::matrix::zeros<1, 1>();
  const auto stationary_dynamic_result = ComputeDynamicCompensation(
      gravity_model, gravity_position, gravity_velocity,
      gravity_acceleration);
  Expect(stationary_dynamic_result.status ==
             mr::robotics::arm::Status::kOk &&
             Near(stationary_dynamic_result.effort[0][0],
                  gravity_result.effort[0][0], 2.0e-3f),
         "stationary inverse dynamics should equal gravity compensation");

  gravity_velocity[0][0] = 2.0f;
  gravity_acceleration[0][0] = 3.0f;
  const auto accelerating_dynamic_result = ComputeDynamicCompensation(
      gravity_model, gravity_position, gravity_velocity,
      gravity_acceleration);
  Expect(accelerating_dynamic_result.status ==
             mr::robotics::arm::Status::kOk,
         "dynamic compensation should accept finite motion");
  Expect(Near(accelerating_dynamic_result.effort[0][0], 5.955f, 2.0e-3f),
         "dynamic compensation should include rotational inertia");

  JointModel prismatic_joint;
  prismatic_joint.type = JointType::kPrismatic;
  prismatic_joint.axis_in_joint_frame[0][0] = 1.0f;
  prismatic_joint.axis_in_joint_frame[2][0] = 0.0f;
  prismatic_joint.limits.position_enabled = true;
  prismatic_joint.limits.position_min = -1.0f;
  prismatic_joint.limits.position_max = 1.0f;
  prismatic_joint.limits.max_velocity = 5.0f;
  prismatic_joint.limits.max_acceleration = 10.0f;
  prismatic_joint.inertial.mass_kg = 2.0f;
  JointModel prismatic_joints[1] = {prismatic_joint};
  const mr::robotics::core::Vec3 zero_gravity =
      mr::component::math::matrix::zeros<3, 1>();
  ArmModel<1> prismatic_model(prismatic_joints, identity<4>(), identity<4>(),
                              zero_gravity);
  JointVector<1> prismatic_position =
      mr::component::math::matrix::zeros<1, 1>();
  JointVector<1> prismatic_velocity =
      mr::component::math::matrix::zeros<1, 1>();
  JointVector<1> prismatic_acceleration =
      mr::component::math::matrix::zeros<1, 1>();
  prismatic_acceleration[0][0] = 3.0f;
  const auto prismatic_dynamic_result = ComputeDynamicCompensation(
      prismatic_model, prismatic_position, prismatic_velocity,
      prismatic_acceleration);
  Expect(prismatic_dynamic_result.status ==
             mr::robotics::arm::Status::kOk &&
             Near(prismatic_dynamic_result.effort[0][0], 6.0f),
         "prismatic dynamic compensation should include mass times acceleration");

  JointModel dynamic_joints[2];
  for (int index = 0; index < 2; ++index) {
    dynamic_joints[index].limits.position_enabled = true;
    dynamic_joints[index].limits.position_min = -3.14f;
    dynamic_joints[index].limits.position_max = 3.14f;
    dynamic_joints[index].limits.max_velocity = 5.0f;
    dynamic_joints[index].limits.max_acceleration = 10.0f;
  }
  dynamic_joints[0].inertial.mass_kg = 2.0f;
  dynamic_joints[0].inertial.center_of_mass[0][0] = 0.4f;
  dynamic_joints[0].inertial.inertia_at_center_of_mass[2][2] = 0.05f;
  dynamic_joints[1].parent_to_joint[0][3] = 1.0f;
  dynamic_joints[1].inertial.mass_kg = 1.5f;
  dynamic_joints[1].inertial.center_of_mass[0][0] = 0.3f;
  dynamic_joints[1].inertial.inertia_at_center_of_mass[2][2] = 0.02f;
  ArmModel<2> dynamic_model(dynamic_joints, identity<4>(), identity<4>(),
                            zero_gravity);
  JointVector<2> dynamic_position =
      mr::component::math::matrix::zeros<2, 1>();
  JointVector<2> dynamic_velocity =
      mr::component::math::matrix::zeros<2, 1>();
  JointVector<2> dynamic_acceleration =
      mr::component::math::matrix::zeros<2, 1>();
  dynamic_position[0][0] = 0.2f;
  dynamic_position[1][0] = 0.6f;
  dynamic_velocity[0][0] = 0.7f;
  dynamic_velocity[1][0] = -0.4f;
  dynamic_acceleration[0][0] = 1.2f;
  dynamic_acceleration[1][0] = -0.8f;
  const auto two_link_dynamic_result = ComputeDynamicCompensation(
      dynamic_model, dynamic_position, dynamic_velocity,
      dynamic_acceleration);
  Expect(two_link_dynamic_result.status == mr::robotics::arm::Status::kOk,
         "two-link inverse dynamics should accept valid motion");
  Expect(Near(two_link_dynamic_result.effort[0][0], 3.00188f, 2.0e-3f) &&
             Near(two_link_dynamic_result.effort[1][0], 0.63218f, 2.0e-3f),
         "two-link inverse dynamics should match analytical equations");

  JointModel controller_joints[2];
  for (int index = 0; index < 2; ++index) {
    controller_joints[index].type = JointType::kRevolute;
    controller_joints[index].parent_to_joint = identity<4>();
    controller_joints[index].limits.position_enabled = true;
    controller_joints[index].limits.position_min = -3.14f;
    controller_joints[index].limits.position_max = 3.14f;
    controller_joints[index].limits.max_velocity = 2.0f;
    controller_joints[index].limits.max_acceleration = 4.0f;
  }
  controller_joints[1].parent_to_joint[0][3] = 1.0f;
  Transform controller_tool = identity<4>();
  controller_tool[0][3] = 1.0f;
  ArmModel<2> controller_model(controller_joints, identity<4>(),
                                controller_tool, gravity);

  mr::robotics::arm::JointState<2> feedback;
  feedback.valid = true;
  feedback.position =
      mr::component::math::matrix::zeros<2, 1>();
  feedback.velocity =
      mr::component::math::matrix::zeros<2, 1>();
  feedback.effort =
      mr::component::math::matrix::zeros<2, 1>();
  ArmControllerConfig<2> controller_config;
  controller_config.enable_gravity_compensation = false;
  controller_config.inverse_kinematics =
      mr::robotics::arm::kinematics::MakePositionOnlyInverseKinematicsOptions();
  controller_config.inverse_kinematics.max_iterations = 48;
  controller_config.inverse_kinematics.position_tolerance = 2.0e-3f;
  ArmController<2> controller(controller_model, controller_config);
  Expect(controller.IsConfigured(), "arm controller should be configured");
  controller.Reset(feedback);

  mr::robotics::arm::ArmCommand<2> disabled_command;
  disabled_command.mode = mr::robotics::arm::ControlMode::kDisabled;
  const auto disabled_output = controller.Update(feedback, disabled_command,
                                                  0.01f);
  Expect(disabled_output.status == mr::robotics::arm::Status::kOk,
         "disabled controller update");
  Expect(disabled_output.joint_command.mode ==
             mr::robotics::arm::JointCommandMode::kDisabled,
         "disabled controller command mode");

  mr::robotics::arm::ArmCommand<2> hold_command;
  hold_command.mode = mr::robotics::arm::ControlMode::kHold;
  const auto hold_output = controller.Update(feedback, hold_command, 0.01f);
  Expect(hold_output.status == mr::robotics::arm::Status::kOk,
         "hold controller update");
  Expect(Near(hold_output.joint_command.position[0][0], 0.0f) &&
             Near(hold_output.joint_command.position[1][0], 0.0f),
         "hold controller keeps feedback position");

  mr::robotics::arm::ArmCommand<2> move_joint_command;
  move_joint_command.mode = mr::robotics::arm::ControlMode::kMoveJoint;
  move_joint_command.joint_target =
      mr::component::math::matrix::zeros<2, 1>();
  move_joint_command.joint_target[0][0] = 0.35f;
  move_joint_command.joint_target[1][0] = -0.20f;
  bool joint_reached = false;
  mr::robotics::arm::ArmOutput<2> joint_output;
  for (int iteration = 0; iteration < 120; ++iteration) {
    joint_output = controller.Update(feedback, move_joint_command, 0.01f);
    Expect(joint_output.status == mr::robotics::arm::Status::kOk,
           "move joint controller update");
    if (!joint_output.joint_command.valid) {
      break;
    }
    feedback.position = joint_output.joint_command.position;
    joint_reached = joint_output.reached;
    if (joint_reached) {
      break;
    }
  }
  Expect(joint_reached, "move joint controller reaches target");
  Expect(Near(feedback.position[0][0], 0.35f, 2.0e-3f) &&
             Near(feedback.position[1][0], -0.20f, 2.0e-3f),
         "move joint controller final position");

  mr::robotics::arm::ArmCommand<2> rejected_joint_command =
      move_joint_command;
  rejected_joint_command.joint_target[0][0] = 4.0f;
  const auto rejected_joint_output =
      controller.Update(feedback, rejected_joint_command, 0.01f);
  Expect(rejected_joint_output.status ==
             mr::robotics::arm::Status::kLimitViolation,
         "out-of-limit move joint target is rejected");
  Expect(Near(rejected_joint_output.joint_command.position[0][0],
              feedback.position[0][0], 2.0e-3f),
         "failed move joint holds last safe position");

  mr::robotics::arm::JointVector<2> cartesian_target_position =
      mr::component::math::matrix::zeros<2, 1>();
  cartesian_target_position[0][0] = 0.15f;
  cartesian_target_position[1][0] = -0.10f;
  const Transform cartesian_target_pose =
      ForwardKinematics(controller_model, cartesian_target_position);
  mr::robotics::arm::ArmCommand<2> move_cartesian_command;
  move_cartesian_command.mode =
      mr::robotics::arm::ControlMode::kMoveCartesian;
  move_cartesian_command.pose_target = cartesian_target_pose;
  move_cartesian_command.position_only = true;
  bool cartesian_reached = false;
  mr::robotics::arm::ArmOutput<2> cartesian_output;
  for (int iteration = 0; iteration < 400; ++iteration) {
    cartesian_output =
        controller.Update(feedback, move_cartesian_command, 0.01f);
    if (cartesian_output.status != mr::robotics::arm::Status::kOk) {
      break;
    }
    feedback.position = cartesian_output.joint_command.position;
    cartesian_reached = cartesian_output.reached;
    if (cartesian_reached) {
      break;
    }
  }
  Expect(cartesian_reached, "move Cartesian controller reaches target");
  const Transform cartesian_feedback_pose =
      ForwardKinematics(controller_model, feedback.position);
  Expect(Near(cartesian_feedback_pose[0][3], cartesian_target_pose[0][3],
              5.0e-3f) &&
             Near(cartesian_feedback_pose[1][3], cartesian_target_pose[1][3],
                  5.0e-3f),
         "move Cartesian controller final pose");

  // Streaming Cartesian commands must keep moving while their target changes
  // every cycle; they must not restart a time-based plan at t=0 forever.
  controller.Reset(feedback);
  mr::robotics::arm::ArmCommand<2> streaming_command;
  streaming_command.mode = mr::robotics::arm::ControlMode::kMoveCartesian;
  streaming_command.position_only = true;
  streaming_command.streaming_cartesian = true;
  JointVector<2> streaming_target = feedback.position;
  Scalar first_position = feedback.position[0][0];
  for (int iteration = 0; iteration < 80; ++iteration) {
    streaming_target[0][0] += 0.0025f;
    streaming_command.pose_target =
        ForwardKinematics(controller_model, streaming_target);
    const auto streaming_output =
        controller.Update(feedback, streaming_command, 0.01f);
    Expect(streaming_output.status == mr::robotics::arm::Status::kOk,
           "streaming Cartesian controller update");
    Expect(std::fabs(streaming_output.joint_command.velocity[0][0]) <=
               controller_joints[0].limits.max_velocity + 1.0e-4f,
           "streaming Cartesian joint velocity limit");
    feedback.position = streaming_output.joint_command.position;
  }
  Expect(feedback.position[0][0] > first_position + 0.05f,
         "streaming Cartesian target should advance the joint");

  mr::robotics::arm::ArmCommand<2> unreachable_cartesian_command =
      move_cartesian_command;
  unreachable_cartesian_command.pose_target = identity<4>();
  unreachable_cartesian_command.pose_target[0][3] = 10.0f;
  bool unreachable_failure_seen = false;
  mr::robotics::arm::ArmOutput<2> unreachable_cartesian_output;
  JointVector<2> safe_before_unreachable = feedback.position;
  for (int iteration = 0; iteration < 160; ++iteration) {
    safe_before_unreachable = feedback.position;
    unreachable_cartesian_output =
        controller.Update(feedback, unreachable_cartesian_command, 0.01f);
    if (unreachable_cartesian_output.status !=
        mr::robotics::arm::Status::kOk) {
      unreachable_failure_seen = true;
      break;
    }
    feedback.position = unreachable_cartesian_output.joint_command.position;
  }
  Expect(unreachable_failure_seen,
         "unreachable Cartesian target reports IK failure");
  Expect(Near(unreachable_cartesian_output.joint_command.position[0][0],
              safe_before_unreachable[0][0], 2.0e-3f) &&
             Near(unreachable_cartesian_output.joint_command.position[1][0],
                  safe_before_unreachable[1][0], 2.0e-3f),
         "failed Cartesian move holds last safe position");

  if (failures != 0) {
    std::printf("arm host tests failed: %d\n", failures);
    return 1;
  }
  std::printf("arm host tests passed\n");
  return 0;
}
