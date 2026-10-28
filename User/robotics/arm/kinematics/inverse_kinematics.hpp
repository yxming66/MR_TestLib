#pragma once

#include "component/math/matrix.hpp"
#include "component/math/scalar.hpp"
#include "../core/status.hpp"
#include "../model/arm_model.hpp"
#include "forward_kinematics.hpp"
#include "jacobian.hpp"
#include "pose_error.hpp"

namespace mr::robotics::arm::kinematics {

constexpr int kMaximumInverseKinematicsIterations = 64;

struct InverseKinematicsOptions {
  int max_iterations;
  Scalar position_tolerance;
  Scalar orientation_tolerance;
  Scalar damping;
  Scalar max_joint_step;
  Twist6 task_weights;
  bool use_orientation;
  bool enforce_position_limits;
  bool enable_adaptive_damping;
  Scalar singularity_sigma_safe;
  Scalar singularity_sigma_stop;
  Scalar maximum_damping;

  InverseKinematicsOptions()
      : max_iterations(32),
        position_tolerance(1.0e-3f),
        orientation_tolerance(1.0e-3f),
        damping(1.0e-3f),
        max_joint_step(0.10f),
        task_weights(mr::component::math::matrix::ones<6, 1>()),
        use_orientation(true),
        enforce_position_limits(true),
        enable_adaptive_damping(true),
        singularity_sigma_safe(5.0e-2f),
        singularity_sigma_stop(5.0e-3f),
        maximum_damping(1.0e-1f) {}
};

inline bool IsInverseKinematicsOptionsValid(
    const InverseKinematicsOptions& options) {
  if (options.max_iterations <= 0 ||
      !mr::component::math::is_positive_scalar(options.position_tolerance) ||
      !mr::component::math::is_positive_scalar(
          options.orientation_tolerance) ||
      !mr::component::math::is_positive_scalar(options.damping) ||
      !mr::component::math::is_positive_scalar(options.max_joint_step) ||
      !mr::component::math::is_positive_scalar(options.singularity_sigma_safe) ||
      !mr::component::math::is_positive_scalar(options.singularity_sigma_stop) ||
      !mr::component::math::is_positive_scalar(options.maximum_damping) ||
      options.singularity_sigma_stop > options.singularity_sigma_safe ||
      options.maximum_damping < options.damping ||
      !mr::component::math::matrix::is_finite(options.task_weights)) {
    return false;
  }

  bool has_active_task = false;
  for (int index = 0; index < 6; ++index) {
    const Scalar weight = options.task_weights[index][0];
    if (weight < 0.0f) {
      return false;
    }
    if ((options.use_orientation || index < 3) &&
        weight > mr::component::math::kDefaultEpsilon) {
      has_active_task = true;
    }
  }
  return has_active_task;
}

inline InverseKinematicsOptions MakePositionOnlyInverseKinematicsOptions() {
  InverseKinematicsOptions options;
  options.use_orientation = false;
  options.task_weights[3][0] = 0.0f;
  options.task_weights[4][0] = 0.0f;
  options.task_weights[5][0] = 0.0f;
  return options;
}

inline InverseKinematicsOptions MakePositionOnlyInverseKinematicsOptions(
    const InverseKinematicsOptions& base_options) {
  InverseKinematicsOptions options = base_options;
  options.use_orientation = false;
  options.task_weights[3][0] = 0.0f;
  options.task_weights[4][0] = 0.0f;
  options.task_weights[5][0] = 0.0f;
  return options;
}

template <int DoF>
struct InverseKinematicsRequest {
  Transform target_pose;
  JointVector<DoF> initial_position;

  InverseKinematicsRequest()
      : target_pose(mr::component::math::matrix::identity<4>()),
        initial_position(mr::component::math::matrix::zeros<DoF, 1>()) {}
};

template <int DoF>
struct InverseKinematicsResult {
  Status status;
  JointVector<DoF> position;
  int iterations;
  Scalar position_error;
  Scalar orientation_error;
  Scalar final_error;
  bool clamped_to_limits;
  Scalar minimum_singular_value;
  Scalar damping_used;
  bool near_singularity;

  InverseKinematicsResult()
      : status(Status::kInvalidInput),
        position(mr::component::math::matrix::zeros<DoF, 1>()),
        iterations(0),
        position_error(0.0f),
        orientation_error(0.0f),
        final_error(0.0f),
        clamped_to_limits(false),
        minimum_singular_value(0.0f),
        damping_used(0.0f),
        near_singularity(false) {}
};

namespace internal {

template <int DoF>
inline bool IsPositionWithinLimits(const ArmModel<DoF>& model,
                                   const JointVector<DoF>& position) {
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
inline JointVector<DoF> ClampToPositionLimits(const ArmModel<DoF>& model,
                                              const JointVector<DoF>& position,
                                              bool* changed) {
  JointVector<DoF> result = position;
  if (changed != nullptr) {
    *changed = false;
  }
  for (int index = 0; index < DoF; ++index) {
    const JointLimits& limits = model.Joint(index).limits;
    if (!limits.position_enabled) {
      continue;
    }
    const Scalar limited = mr::component::math::clamp_scalar(
        result[index][0], limits.position_min, limits.position_max);
    if (limited != result[index][0] && changed != nullptr) {
      *changed = true;
    }
    result[index][0] = limited;
  }
  return result;
}

template <int DoF>
inline bool BuildNormalEquation(const Jacobian<DoF>& jacobian,
                                const Twist6& error,
                                const InverseKinematicsOptions& options,
                                Scalar damping,
                                mr::component::math::Matrix<DoF, DoF>* normal,
                                JointVector<DoF>* right_hand_side) {
  if (normal == nullptr || right_hand_side == nullptr ||
      !IsJacobianFinite(jacobian) ||
      !mr::component::math::matrix::is_finite(error)) {
    return false;
  }

  *normal = mr::component::math::matrix::zeros<DoF, DoF>();
  *right_hand_side = mr::component::math::matrix::zeros<DoF, 1>();
  for (int task_row = 0; task_row < 6; ++task_row) {
    Scalar weight = options.task_weights[task_row][0];
    if (task_row >= 3 && !options.use_orientation) {
      weight = 0.0f;
    }
    if (!mr::component::math::is_finite_scalar(weight)) {
      return false;
    }
    const Scalar weight_squared = weight * weight;
    for (int row = 0; row < DoF; ++row) {
      (*right_hand_side)[row][0] +=
          weight_squared * jacobian(task_row, row) * error[task_row][0];
      for (int col = 0; col < DoF; ++col) {
        (*normal)(row, col) += weight_squared * jacobian(task_row, row) *
                               jacobian(task_row, col);
      }
    }
  }

  for (int index = 0; index < DoF; ++index) {
    (*normal)(index, index) += damping;
  }
  return mr::component::math::matrix::is_finite(*normal) &&
         mr::component::math::matrix::is_finite(*right_hand_side);
}

// Returns the smallest singular value in the active task subspace. A small
// fixed-size Jacobi eigensolver keeps this usable in the firmware build.
template <int DoF>
inline Scalar MinimumSingularValue(const Jacobian<DoF>& jacobian,
                                   const InverseKinematicsOptions& options) {
  mr::component::math::Matrix<6, 6> gram =
      mr::component::math::matrix::zeros<6, 6>();
  int active_rows = 0;
  for (int row = 0; row < 6; ++row) {
    const Scalar weight = (row >= 3 && !options.use_orientation)
                              ? 0.0f : options.task_weights[row][0];
    if (weight > mr::component::math::kDefaultEpsilon) ++active_rows;
  }
  // Build J*J^T explicitly; rows with zero task weight are zeroed.
  gram = mr::component::math::matrix::zeros<6, 6>();
  for (int row = 0; row < 6; ++row) {
    const Scalar wr = (row >= 3 && !options.use_orientation)
                          ? 0.0f : options.task_weights[row][0];
    for (int col = 0; col < 6; ++col) {
      const Scalar wc = (col >= 3 && !options.use_orientation)
                            ? 0.0f : options.task_weights[col][0];
      for (int joint = 0; joint < DoF; ++joint) {
        gram(row, col) += wr * wc * jacobian(row, joint) * jacobian(col, joint);
      }
    }
  }
  const int rank_target = (active_rows < DoF) ? active_rows : DoF;
  if (rank_target <= 0) return 0.0f;
  for (int sweep = 0; sweep < 12; ++sweep) {
    for (int p = 0; p < 6; ++p) {
      for (int q = p + 1; q < 6; ++q) {
        const Scalar apq = gram(p, q);
        if (mr::component::math::abs_scalar(apq) < 1.0e-7f) continue;
        const Scalar tau = (gram(q, q) - gram(p, p)) / (2.0f * apq);
        const Scalar t = (tau >= 0.0f ? 1.0f : -1.0f) /
                         (mr::component::math::abs_scalar(tau) +
                          sqrtf(1.0f + tau * tau));
        const Scalar c = 1.0f / sqrtf(1.0f + t * t);
        const Scalar s = t * c;
        const Scalar app = gram(p, p), aqq = gram(q, q);
        for (int k = 0; k < 6; ++k) {
          const Scalar gpk = gram(p, k), gqk = gram(q, k);
          gram(p, k) = c * gpk - s * gqk;
          gram(q, k) = s * gpk + c * gqk;
        }
        for (int k = 0; k < 6; ++k) {
          const Scalar gkp = gram(k, p), gkq = gram(k, q);
          gram(k, p) = c * gkp - s * gkq;
          gram(k, q) = s * gkp + c * gkq;
        }
        gram(p, p) = c * c * app - 2.0f * s * c * apq + s * s * aqq;
        gram(q, q) = s * s * app + 2.0f * s * c * apq + c * c * aqq;
      }
    }
  }
  Scalar eigenvalues[6];
  for (int i = 0; i < 6; ++i) eigenvalues[i] = mr::component::math::positive_or_zero(gram(i, i));
  for (int i = 0; i < 6; ++i) for (int j = i + 1; j < 6; ++j)
    if (eigenvalues[j] > eigenvalues[i]) { const Scalar t = eigenvalues[i]; eigenvalues[i] = eigenvalues[j]; eigenvalues[j] = t; }
  return sqrtf(eigenvalues[rank_target - 1]);
}

template <int DoF>
inline void LimitJointStep(JointVector<DoF>* step, Scalar max_joint_step) {
  if (step == nullptr || max_joint_step <= 0.0f) {
    return;
  }
  Scalar maximum = 0.0f;
  for (int index = 0; index < DoF; ++index) {
    const Scalar absolute_step =
        mr::component::math::abs_scalar((*step)[index][0]);
    if (absolute_step > maximum) {
      maximum = absolute_step;
    }
  }
  if (maximum <= max_joint_step) {
    return;
  }
  *step = *step * (max_joint_step / maximum);
}

}  // namespace internal

template <int DoF>
inline InverseKinematicsResult<DoF> SolveInverseKinematics(
    const ArmModel<DoF>& model,
    const InverseKinematicsRequest<DoF>& request,
    const InverseKinematicsOptions& options) {
  InverseKinematicsResult<DoF> result;

  if (!IsModelValid(model)) {
    result.status = Status::kInvalidModel;
    return result;
  }
  if (!IsJointVectorFinite(request.initial_position)) {
    return result;
  }
  result.position = request.initial_position;
  if (!IsTransformValid(request.target_pose) ||
      !IsInverseKinematicsOptionsValid(options)) {
    return result;
  }

  if (options.enforce_position_limits &&
      !internal::IsPositionWithinLimits(model, result.position)) {
    result.status = Status::kLimitViolation;
    return result;
  }

  const int iteration_limit =
      (options.max_iterations < kMaximumInverseKinematicsIterations)
          ? options.max_iterations
          : kMaximumInverseKinematicsIterations;

  for (int iteration = 0; iteration < iteration_limit; ++iteration) {
    const Transform current_pose = ForwardKinematics(model, result.position);
    const PoseError pose_error =
        ComputePoseError(request.target_pose, current_pose);
    if (!pose_error.valid) {
      result.status = Status::kNumericalFailure;
      result.iterations = iteration;
      return result;
    }

    result.position_error = pose_error.position_norm;
    result.orientation_error = pose_error.orientation_norm;
    result.final_error =
        pose_error.position_norm +
        (options.use_orientation ? pose_error.orientation_norm : 0.0f);
    result.iterations = iteration;

    const Jacobian<DoF> diagnostic_jacobian =
        ComputeJacobian(model, result.position);
    result.minimum_singular_value =
        internal::MinimumSingularValue(diagnostic_jacobian, options);
    result.near_singularity =
        result.minimum_singular_value <= options.singularity_sigma_safe;
    Scalar damping = options.damping;
    if (options.enable_adaptive_damping &&
        result.minimum_singular_value < options.singularity_sigma_safe) {
      const Scalar span = options.singularity_sigma_safe -
                          options.singularity_sigma_stop;
      const Scalar ratio = span > 0.0f
                               ? (options.singularity_sigma_safe -
                                  result.minimum_singular_value) / span
                               : 1.0f;
      damping = options.damping +
                (options.maximum_damping - options.damping) *
                    mr::component::math::clamp_scalar(ratio, 0.0f, 1.0f);
    }
    result.damping_used = damping;

    const bool position_reached =
        pose_error.position_norm <= options.position_tolerance;
    const bool orientation_reached =
        !options.use_orientation ||
        pose_error.orientation_norm <= options.orientation_tolerance;
    if (position_reached && orientation_reached) {
      result.status = Status::kOk;
      return result;
    }

    const Jacobian<DoF> jacobian = ComputeJacobian(model, result.position);
    mr::component::math::Matrix<DoF, DoF> normal;
    JointVector<DoF> right_hand_side;
    if (!internal::BuildNormalEquation(jacobian, pose_error.twist, options,
                                       damping,
                                       &normal, &right_hand_side)) {
      result.status = Status::kNumericalFailure;
      return result;
    }

    JointVector<DoF> step;
    if (!mr::component::math::matrix::solve(normal, right_hand_side, &step,
                                            mr::component::math::kDefaultEpsilon)) {
      result.status = Status::kSingularity;
      return result;
    }
    if (!IsJointVectorFinite(step)) {
      result.status = Status::kNumericalFailure;
      return result;
    }
    internal::LimitJointStep(&step, options.max_joint_step);
    result.position += step;

    if (options.enforce_position_limits) {
      bool clamped = false;
      result.position = internal::ClampToPositionLimits(
          model, result.position, &clamped);
      result.clamped_to_limits = result.clamped_to_limits || clamped;
    }
    if (!IsJointVectorFinite(result.position)) {
      result.status = Status::kNumericalFailure;
      return result;
    }
  }

  const Transform final_pose = ForwardKinematics(model, result.position);
  const PoseError final_error =
      ComputePoseError(request.target_pose, final_pose);
  if (final_error.valid) {
    result.position_error = final_error.position_norm;
    result.orientation_error = final_error.orientation_norm;
    result.final_error =
        final_error.position_norm +
        (options.use_orientation ? final_error.orientation_norm : 0.0f);
    const bool position_reached =
        final_error.position_norm <= options.position_tolerance;
    const bool orientation_reached =
        !options.use_orientation ||
        final_error.orientation_norm <= options.orientation_tolerance;
    if (position_reached && orientation_reached) {
      result.status = Status::kOk;
      return result;
    }
  }
  result.iterations = iteration_limit;
  result.status = Status::kNoConvergence;
  return result;
}

}  // namespace mr::robotics::arm::kinematics
