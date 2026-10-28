#include "device/motor/bus/can_transport.hpp"

#include <string.h>

namespace mr::motor {

int8_t CanTransport::RegisterLatest(uint32_t id) const {
  if (!is_valid()) {
    return BSP_ERR_NO_DEV;
  }
  return BSP_CAN_RegisterLatestId(bus_, id);
}

int8_t CanTransport::RegisterQueue(uint32_t id, uint8_t queue_size) const {
  if (!is_valid()) return BSP_ERR_NO_DEV;
  return BSP_CAN_RegisterId(bus_, id, queue_size);
}

int8_t CanTransport::RegisterIdParser(BSP_CAN_IdParser_t parser) const {
  if (!is_valid() || parser == nullptr) return BSP_ERR_NULL;
  return BSP_CAN_RegisterIdParser(parser);
}

int8_t CanTransport::ReadLatest(uint32_t id, CanFrame* frame) const {
  if (!is_valid()) {
    return BSP_ERR_NO_DEV;
  }
  if (frame == nullptr) {
    return BSP_ERR_NULL;
  }

  BSP_CAN_Message_t message{};
  const int8_t result = BSP_CAN_GetLatestMessage(bus_, id, &message);
  if (result != BSP_OK) {
    return result;
  }
  if (message.frame_type != BSP_CAN_FRAME_STD_DATA ||
      message.dlc > BSP_CAN_MAX_DLC) {
    return BSP_ERR;
  }

  frame->id = message.original_id;
  frame->format = message.frame_type == BSP_CAN_FRAME_EXT_DATA
                      ? CanFrame::Format::Extended : CanFrame::Format::Standard;
  frame->dlc = message.dlc;
  memcpy(frame->data, message.data, message.dlc);
  frame->timestamp_ms = message.timestamp;
  frame->sequence = message.sequence;
  return BSP_OK;
}

int8_t CanTransport::ReadNext(uint32_t id, CanFrame* frame) const {
  if (!is_valid()) return BSP_ERR_NO_DEV;
  if (frame == nullptr) return BSP_ERR_NULL;
  BSP_CAN_Message_t message{};
  const int8_t result = BSP_CAN_GetMessage(bus_, id, &message, BSP_CAN_TIMEOUT_IMMEDIATE);
  if (result != BSP_OK || message.dlc > BSP_CAN_MAX_DLC) return result;
  frame->id = message.original_id;
  frame->format = message.frame_type == BSP_CAN_FRAME_EXT_DATA
                      ? CanFrame::Format::Extended : CanFrame::Format::Standard;
  frame->dlc = message.dlc;
  memcpy(frame->data, message.data, message.dlc);
  frame->timestamp_ms = message.timestamp;
  frame->sequence = message.sequence;
  return BSP_OK;
}

int8_t CanTransport::SendStandard(const CanFrame& frame) const {
  if (!is_valid()) {
    return BSP_ERR_NO_DEV;
  }
  if (frame.id > 0x7ffU || frame.dlc > BSP_CAN_MAX_DLC) {
    return BSP_ERR;
  }

  BSP_CAN_StdDataFrame_t native_frame{};
  native_frame.id = frame.id;
  native_frame.dlc = frame.dlc;
  memcpy(native_frame.data, frame.data, frame.dlc);
  return BSP_CAN_TransmitStdDataFrame(bus_, &native_frame);
}

int8_t CanTransport::SendExtended(const CanFrame& frame) const {
  if (!is_valid()) return BSP_ERR_NO_DEV;
  if (frame.id > 0x1FFFFFFFU || frame.dlc > BSP_CAN_MAX_DLC) return BSP_ERR;
  BSP_CAN_ExtDataFrame_t native_frame{};
  native_frame.id = frame.id;
  native_frame.dlc = frame.dlc;
  memcpy(native_frame.data, frame.data, frame.dlc);
  return BSP_CAN_TransmitExtDataFrame(bus_, &native_frame);
}

}  // namespace mr::motor
