#pragma once

#include <stdint.h>

namespace mr::robotics::arm {

enum class Status : uint8_t {
  kOk = 0,                  //成功
  kNotConfigured,           //未配置 
  kInvalidInput,            //输入无效  
  kInvalidModel,            //模型无效  
  kLimitViolation,          //超限  
  kNoConvergence,           //未收敛  
  kSingularity,             //奇异  
  kNumericalFailure,        //数值失败  
  kUnsupported,             //不支持  
};

inline bool IsOk(Status status) {
  return status == Status::kOk;
}

inline bool IsFailure(Status status) {
  return !IsOk(status);
}

}  // namespace mr::robotics::arm
