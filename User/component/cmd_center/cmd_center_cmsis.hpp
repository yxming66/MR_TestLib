#pragma once

// CMSIS-RTOS 消息队列适配器。
// 使用方式：先创建 osMessageQueue，再构造 CmsisQueueSink(queue)，
// 将 sink 注册到命令中心；命令中心分发时会把最新命令写入该队列。

#include "cmsis_os2.h"

namespace cmd {

class CmsisQueueSink {
 public:
  explicit CmsisQueueSink(osMessageQueueId_t queue, bool reset_before_put = true)
      : queue_(queue), reset_before_put_(reset_before_put) {}

  template <typename Cmd>
  bool operator()(const Cmd& cmd) const {
    if (queue_ == nullptr) {
      return false;
    }
    if (reset_before_put_) {
      (void)osMessageQueueReset(queue_);
    }
    return osMessageQueuePut(queue_, &cmd, 0, 0) == osOK;
  }

 private:
  osMessageQueueId_t queue_ = nullptr;
  bool reset_before_put_ = true;
};

inline CmsisQueueSink queue(osMessageQueueId_t queue,
                            bool reset_before_put = true) {
  return CmsisQueueSink(queue, reset_before_put);
}

}  // namespace cmd
