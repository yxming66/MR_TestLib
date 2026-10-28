#include "transform.hpp"

namespace mr::robotics::core {

Transform MakeTransform(const Rotation& rotation, const Vec3& translation) {
  const float data[16] = {
      rotation[0][0], rotation[0][1], rotation[0][2], translation[0][0],
      rotation[1][0], rotation[1][1], rotation[1][2], translation[1][0],
      rotation[2][0], rotation[2][1], rotation[2][2], translation[2][0],
      0.0f,           0.0f,           0.0f,           1.0f,
  };
  return Transform(data);
}

Transform TransformFromTranslation(const Vec3& translation) {
  return MakeTransform(::mr::component::math::matrix::identity_rect<3, 3>(), translation);
}

Transform TransformFromAxisAngle(const Vec3& axis, float angle) {
  return MakeTransform(RotationFromAxisAngle(axis, angle), matrix::zeros<3, 1>());
}

Rotation RotationOf(const Transform& transform) {
  return transform.block<3, 3>(0, 0);
}

Vec3 TranslationOf(const Transform& transform) {
  return transform.block<3, 1>(0, 3);
}

Transform InverseTransform(const Transform& transform) {
  const Rotation inverse_rotation = RotationOf(transform).transpose();
  return MakeTransform(inverse_rotation,
                       -1.0f * inverse_rotation * TranslationOf(transform));
}

Twist6 TwistFromTransform(const Transform& transform) {
  const Vec3 translation = TranslationOf(transform);
  const Vec4 axis_angle = AxisAngleFromRotation(RotationOf(transform));
  const Vec3 angular = axis_angle.block<3, 1>(0, 0) * axis_angle[3][0];
  const float data[6] = {
      translation[0][0], translation[1][0], translation[2][0],
      angular[0][0], angular[1][0], angular[2][0],
  };
  return Twist6(data);
}

Transform TransformFromTwist(const Twist6& twist) {
  const Vec3 translation = twist.block<3, 1>(0, 0);
  const Vec3 angular = twist.block<3, 1>(3, 0);
  const float angle = angular.norm();
  return MakeTransform(RotationFromAxisAngle(angular, angle), translation);
}

}  // namespace mr::robotics::core
