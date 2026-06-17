#pragma once

#include "bsp.hpp"

#include "os.hpp"

#include <cstddef>
#include <cstdint>

namespace bsp {

enum class CanBus : uint8_t {
    Bus1 = 0,
    Bus2 = 1,
    Bus3 = 2,
};

enum class CanType : uint8_t {
    Classic = 0, ///< RxFifo0 + 错误中断（hfdcan2/3）
    FD = 1,      ///< RxFifo1 + Tx 延迟补偿（hfdcan1）
};

using CanRxCallback = void (*)(CanBus bus, uint32_t id, const uint8_t* data, uint8_t len,
                               void* user_data);

constexpr size_t kMaxCanRxCallbacks = 8;

Status can_init(CanBus bus, CanType type);
Status can_transmit(CanBus bus, uint32_t std_id, const uint8_t* data, uint16_t len);

Status can_register_rx_callback(CanBus bus, CanRxCallback callback, void* user_data);
void can_unregister_rx_callbacks(CanBus bus);

os::Semaphore* can_error_sem(CanBus bus);

} // namespace bsp
