#pragma once

// 电机协议使用的静态 CAN 传输边界。
// 使用方式：CanTransport can(BSP_CAN_1); 注册反馈 ID 后调用 ReadLatest/ReadNext，
// 发送时构造 CanFrame 并调用 SendStandard 或 SendExtended。该类只访问 BSP CAN，
// 不包含具体电机协议逻辑。

#include <stdint.h>

#include "bsp/can.h"

namespace mr::motor {

// A standard CAN data frame used by motor protocols. The BSP remains the
// sole owner of CAN hardware, IRQ handling, receive queues, and TX queues.
struct CanFrame {
  enum class Format : uint8_t { Standard, Extended };

  uint32_t id = 0U;
  Format format = Format::Standard;
  uint8_t dlc = 0U;
  uint8_t data[BSP_CAN_MAX_DLC]{};
  uint32_t timestamp_ms = 0U;
  uint32_t sequence = 0U;
};

class CanTransport final {
 public:
  explicit constexpr CanTransport(BSP_CAN_t bus) : bus_(bus) {}

  BSP_CAN_t bus() const { return bus_; }
  bool is_valid() const { return bus_ < BSP_CAN_NUM; }

  // Registers a latest-value receive slot owned by the BSP CAN manager.
  int8_t RegisterLatest(uint32_t id) const;
  int8_t RegisterQueue(uint32_t id, uint8_t queue_size) const;
  int8_t RegisterIdParser(BSP_CAN_IdParser_t parser) const;
  int8_t ReadLatest(uint32_t id, CanFrame* frame) const;
  int8_t ReadNext(uint32_t id, CanFrame* frame) const;
  int8_t SendStandard(const CanFrame& frame) const;
  int8_t SendExtended(const CanFrame& frame) const;

 private:
  BSP_CAN_t bus_;
};

}  // namespace mr::motor
