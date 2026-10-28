#ifndef MR_ROBOTICS_CORE_TRANSFORM_HPP
#define MR_ROBOTICS_CORE_TRANSFORM_HPP

// 齐次变换与 Twist 类型。
// 使用方式：Transform T = matrix::identity<4>();
// 先用 rotation/translation 组件构造 T，再通过 T * point 或 T * pose 使用。

#include "rotation.hpp"

namespace mr::robotics::core {

using mr::component::math::Matrix;

using Transform = Matrix<4, 4>;
using Twist6 = Matrix<6, 1>;

Transform MakeTransform(const Rotation& rotation, const Vec3& translation);
Transform TransformFromTranslation(const Vec3& translation);
Transform TransformFromAxisAngle(const Vec3& axis, float angle);

Rotation RotationOf(const Transform& transform);
Vec3 TranslationOf(const Transform& transform);
Transform InverseTransform(const Transform& transform);

Twist6 TwistFromTransform(const Transform& transform);
Transform TransformFromTwist(const Twist6& twist);

}  // namespace mr::robotics::core

#endif
