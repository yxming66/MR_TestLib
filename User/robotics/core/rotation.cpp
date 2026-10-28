#include "rotation.hpp"

#include <cmath>

#include "component/math/scalar.hpp"

namespace mr::robotics::core {
namespace {

constexpr float kAxisEpsilon = 1.0e-6f;

float clamp_unit(float value) {
  return mr::component::math::clamp_scalar(value, -1.0f, 1.0f);
}

Rotation identity_rotation() {
  return ::mr::component::math::matrix::identity_rect<3, 3>();
}

}  // namespace

Rotation RotationFromRpy(const Vec3& rpy) {
  const float roll = rpy[0][0];
  const float pitch = rpy[1][0];
  const float yaw = rpy[2][0];
  const float cy = cosf(yaw);
  const float sy = sinf(yaw);
  const float cp = cosf(pitch);
  const float sp = sinf(pitch);
  const float cr = cosf(roll);
  const float sr = sinf(roll);
  const float data[9] = {
      cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr,
      sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr,
      -sp,     cp * sr,                cp * cr,
  };
  return Rotation(data);
}

Vec3 RpyFromRotation(const Rotation& rotation) {
  const float data[3] = {
      atan2f(rotation[2][1], rotation[2][2]),
      atan2f(-rotation[2][0],
             sqrtf(rotation[2][1] * rotation[2][1] +
                   rotation[2][2] * rotation[2][2])),
      atan2f(rotation[1][0], rotation[0][0]),
  };
  return Vec3(data);
}

Rotation RotationFromAxisAngle(const Vec3& axis, float angle) {
  const float norm = axis.norm();
  if (!mr::component::math::is_finite_scalar(norm) || norm <= kAxisEpsilon ||
      !mr::component::math::is_finite_scalar(angle)) {
    return identity_rotation();
  }

  const Vec3 unit_axis = axis / norm;
  const Matrix<3, 3> axis_hat = ::mr::component::math::vector3f::hat(unit_axis);
  return identity_rotation() + axis_hat * sinf(angle) +
         axis_hat * axis_hat * (1.0f - cosf(angle));
}

Vec4 AxisAngleFromRotation(const Rotation& rotation) {
  const float theta = acosf(clamp_unit(0.5f * (rotation.trace() - 1.0f)));
  float data[4] = {1.0f, 0.0f, 0.0f, theta};
  if (theta <= kAxisEpsilon) {
    data[3] = 0.0f;
    return Vec4(data);
  }

  const float sine = sinf(theta);
  if (fabsf(sine) > kAxisEpsilon) {
    data[0] = (rotation[2][1] - rotation[1][2]) / (2.0f * sine);
    data[1] = (rotation[0][2] - rotation[2][0]) / (2.0f * sine);
    data[2] = (rotation[1][0] - rotation[0][1]) / (2.0f * sine);
    return Vec4(data);
  }

  data[0] = sqrtf(fmaxf(0.0f, 0.5f * (rotation[0][0] + 1.0f)));
  data[1] = sqrtf(fmaxf(0.0f, 0.5f * (rotation[1][1] + 1.0f)));
  data[2] = sqrtf(fmaxf(0.0f, 0.5f * (rotation[2][2] + 1.0f)));
  if (rotation[0][1] < 0.0f) data[1] = -data[1];
  if (rotation[0][2] < 0.0f) data[2] = -data[2];
  return Vec4(data);
}

Rotation RotationFromQuaternion(const Vec4& quaternion) {
  const float norm = quaternion.norm();
  if (!mr::component::math::is_finite_scalar(norm) || norm <= kAxisEpsilon) {
    return identity_rotation();
  }
  const Vec4 q = quaternion / norm;
  const float w = q[0][0];
  const float x = q[1][0];
  const float y = q[2][0];
  const float z = q[3][0];
  const float data[9] = {
      1.0f - 2.0f * (y * y + z * z), 2.0f * (x * y - w * z),
      2.0f * (x * z + w * y), 2.0f * (x * y + w * z),
      1.0f - 2.0f * (x * x + z * z), 2.0f * (y * z - w * x),
      2.0f * (x * z - w * y), 2.0f * (y * z + w * x),
      1.0f - 2.0f * (x * x + y * y),
  };
  return Rotation(data);
}

Vec4 QuaternionFromRotation(const Rotation& rotation) {
  const float trace = rotation.trace();
  float data[4] = {};
  if (trace > 0.0f) {
    const float scale = 2.0f * sqrtf(trace + 1.0f);
    if (scale <= kAxisEpsilon) {
      data[0] = 1.0f;
      return Vec4(data);
    }
    data[0] = 0.25f * scale;
    data[1] = (rotation[2][1] - rotation[1][2]) / scale;
    data[2] = (rotation[0][2] - rotation[2][0]) / scale;
    data[3] = (rotation[1][0] - rotation[0][1]) / scale;
  } else if (rotation[0][0] > rotation[1][1] && rotation[0][0] > rotation[2][2]) {
    const float scale = 2.0f * sqrtf(
        fmaxf(0.0f, 1.0f + rotation[0][0] - rotation[1][1] - rotation[2][2]));
    if (scale <= kAxisEpsilon) {
      data[0] = 1.0f;
      return Vec4(data);
    }
    data[0] = (rotation[2][1] - rotation[1][2]) / scale;
    data[1] = 0.25f * scale;
    data[2] = (rotation[0][1] + rotation[1][0]) / scale;
    data[3] = (rotation[0][2] + rotation[2][0]) / scale;
  } else if (rotation[1][1] > rotation[2][2]) {
    const float scale = 2.0f * sqrtf(
        fmaxf(0.0f, 1.0f + rotation[1][1] - rotation[0][0] - rotation[2][2]));
    if (scale <= kAxisEpsilon) {
      data[0] = 1.0f;
      return Vec4(data);
    }
    data[0] = (rotation[0][2] - rotation[2][0]) / scale;
    data[1] = (rotation[0][1] + rotation[1][0]) / scale;
    data[2] = 0.25f * scale;
    data[3] = (rotation[1][2] + rotation[2][1]) / scale;
  } else {
    const float scale = 2.0f * sqrtf(
        fmaxf(0.0f, 1.0f + rotation[2][2] - rotation[0][0] - rotation[1][1]));
    if (scale <= kAxisEpsilon) {
      data[0] = 1.0f;
      return Vec4(data);
    }
    data[0] = (rotation[1][0] - rotation[0][1]) / scale;
    data[1] = (rotation[0][2] + rotation[2][0]) / scale;
    data[2] = (rotation[1][2] + rotation[2][1]) / scale;
    data[3] = 0.25f * scale;
  }
  return Vec4(data);
}


}  // namespace mr::robotics::core
