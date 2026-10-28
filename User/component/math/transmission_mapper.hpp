#pragma once

// 输入轴与输出轴之间的传动比、速度和力矩换算工具。
// 使用方式：auto mapper = TransmissionRatioMapper::FromRatio(ratio);
// 然后用 ToInput/ToOutput 或 ToInputLimit/ToOutputLimit 完成单位换算。

#include "scalar.hpp"

namespace mr::component::math {

struct TransmissionRatioMapper {
  Scalar ratio = 1.0f;

  static TransmissionRatioMapper FromRatio(Scalar ratio_value) {
    return {positive_or(ratio_value, 1.0f)};
  }

  Scalar ToInput(Scalar output_value) const {
    return output_value * EffectiveRatio();
  }

  Scalar ToInputLimit(Scalar output_limit) const {
    return abs_scalar(output_limit) * EffectiveRatio();
  }

  Scalar ToOutput(Scalar input_value) const {
    return input_value / EffectiveRatio();
  }

 private:
  Scalar EffectiveRatio() const {
    return positive_or(ratio, 1.0f);
  }
};

}  // namespace mr::component::math
