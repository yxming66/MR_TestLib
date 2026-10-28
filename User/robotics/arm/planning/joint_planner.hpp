#pragma once

// Fixed-size synchronized joint-space planning.
// Configure is intended for target changes; Sample is the periodic operation.

#include <array>

#include "component/math/matrix.hpp"
#include "component/math/scalar.hpp"
#include "component/trajectory/synchronized_profile.hpp"
#include "component/trajectory/trapezoid.hpp"
#include "../core/types.hpp"

namespace mr::robotics::arm::planning {

template <int DoF>
struct JointPlanRequest {
  JointVector<DoF> start;
  JointVector<DoF> goal;
  JointVector<DoF> max_velocity;
  JointVector<DoF> max_acceleration;

  JointPlanRequest()
      : start(mr::component::math::matrix::zeros<DoF, 1>()),
        goal(mr::component::math::matrix::zeros<DoF, 1>()),
        max_velocity(mr::component::math::matrix::zeros<DoF, 1>()),
        max_acceleration(mr::component::math::matrix::zeros<DoF, 1>()) {}
};

template <int DoF>
struct JointPlanSample {
  JointVector<DoF> position;
  JointVector<DoF> velocity;
  JointVector<DoF> acceleration;
  Scalar time_from_start_s;
  bool valid;
  bool finished;

  JointPlanSample()
      : position(mr::component::math::matrix::zeros<DoF, 1>()),
        velocity(mr::component::math::matrix::zeros<DoF, 1>()),
        acceleration(mr::component::math::matrix::zeros<DoF, 1>()),
        time_from_start_s(0.0f),
        valid(false),
        finished(false) {}
};

template <int DoF>
class JointPlanner {
  static_assert(DoF > 0, "JointPlanner requires at least one joint");

 public:
  using Profile = mr::comp::traj::SynchronizedProfile<
      DoF, mr::comp::traj::TrapezoidProfile>;

  JointPlanner()
      : request_(), profile_(), duration_s_(0.0f), valid_(false) {}

  bool Configure(const JointPlanRequest<DoF>& request) {
    valid_ = false;
    duration_s_ = 0.0f;
    request_ = request;

    if (!mr::component::math::matrix::is_finite(request.start) ||
        !mr::component::math::matrix::is_finite(request.goal) ||
        !mr::component::math::matrix::is_finite(request.max_velocity) ||
        !mr::component::math::matrix::is_finite(request.max_acceleration)) {
      return false;
    }

    std::array<mr::comp::traj::TrapezoidProfile::Config, DoF> axis_configs{};
    for (int index = 0; index < DoF; ++index) {
      const Scalar max_velocity = request.max_velocity[index][0];
      const Scalar max_acceleration = request.max_acceleration[index][0];
      if (!mr::component::math::is_positive_scalar(max_velocity) ||
          !mr::component::math::is_positive_scalar(max_acceleration)) {
        return false;
      }
      axis_configs[index].distance =
          request.goal[index][0] - request.start[index][0];
      axis_configs[index].max_velocity = max_velocity;
      axis_configs[index].max_acceleration = max_acceleration;
      axis_configs[index].epsilon = mr::component::math::kDefaultEpsilon;
    }

    if (!profile_.Configure(axis_configs)) {
      return false;
    }

    duration_s_ = profile_.Duration();
    valid_ = mr::component::math::is_finite_scalar(duration_s_);
    if (!valid_) {
      duration_s_ = 0.0f;
    }
    return valid_;
  }

  JointPlanSample<DoF> Sample(Scalar elapsed_s) const {
    JointPlanSample<DoF> sample;
    sample.time_from_start_s = elapsed_s;
    sample.position = request_.start;
    if (!valid_ || !mr::component::math::is_finite_scalar(elapsed_s)) {
      return sample;
    }

    sample.valid = true;
    for (int index = 0; index < DoF; ++index) {
      const mr::comp::traj::ScalarMotionSample axis_sample =
          profile_.SampleAxis(static_cast<uint16_t>(index), elapsed_s);
      sample.position[index][0] =
          request_.start[index][0] + axis_sample.position;
      sample.velocity[index][0] = axis_sample.velocity;
      sample.acceleration[index][0] = axis_sample.acceleration;
    }
    sample.finished = elapsed_s >= duration_s_;

    if (sample.finished) {
      sample.position = request_.goal;
      sample.velocity = mr::component::math::matrix::zeros<DoF, 1>();
      sample.acceleration = mr::component::math::matrix::zeros<DoF, 1>();
    }
    return sample;
  }

  Scalar Duration() const { return duration_s_; }
  bool IsValid() const { return valid_; }
  const JointPlanRequest<DoF>& Request() const { return request_; }

 private:
  JointPlanRequest<DoF> request_;
  Profile profile_;
  Scalar duration_s_;
  bool valid_;
};

}  // namespace mr::robotics::arm::planning
