#pragma once

// 单个电机的总线和地址配置。使用方式：按 MotorKind 构造对应 Config，填入
// CAN、反馈/控制 ID 和 reverse，再传给 MotorFactory::Create 或 MotorT 构造函数。

// 定义按 MotorKind 分类的实例配置结构。
// 用于在通用 motor 配置与底层厂商参数结构之间做集中转换。

#include <stdint.h>

#include "bsp/can.h"
#include "device/motor/core/motor_kind.hpp"
#include "device/motor/protocol/protocol_types.hpp"

namespace mr::motor {

template <MotorKind Kind>
struct MotorInstanceConfig;

template <>
struct MotorInstanceConfig<MotorKind::RM> {
    BSP_CAN_t can = BSP_CAN_1;
    uint16_t id = 0;
    bool reverse = false;

    template <typename VendorParam>
    static constexpr MotorInstanceConfig FromVendorParam(const VendorParam& param) {
        return {param.can, param.id, param.reverse};
    }

};

template <>
struct MotorInstanceConfig<MotorKind::DM> {
    BSP_CAN_t can = BSP_CAN_1;
    uint16_t master_id = 0;
    uint16_t can_id = 0;
    bool reverse = false;

    template <typename VendorParam>
    static constexpr MotorInstanceConfig FromVendorParam(const VendorParam& param) {
        return {param.can, param.master_id, param.can_id, param.reverse};
    }

};

template <>
struct MotorInstanceConfig<MotorKind::LZ> {
    BSP_CAN_t can = BSP_CAN_1;
    uint8_t motor_id = 0;
    uint8_t host_id = 0xff;
    LzControlMode mode = LzControlMode::Motion;
    bool reverse = false;

    template <typename VendorParam>
    static constexpr MotorInstanceConfig FromVendorParam(const VendorParam& param) {
        return {param.can, param.motor_id, param.host_id,
                static_cast<LzControlMode>(param.mode), param.reverse};
    }

};

} // namespace mr::motor
