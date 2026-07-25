#pragma once

#include "bsp_usb.hpp"

namespace bsp::usb::detail
{

inline constexpr uint16_t common_max_transfer_size = 512U;
inline constexpr uint8_t common_tx_queue_depth = 2U;

struct backend_config
{
    uint8_t read_priority = 0U;
    uint8_t write_priority = 0U;
    uint32_t period_ticks = 0U;
    uint16_t min_rx_size = 0U;
    uint16_t max_tx_size = 0U;
};

types::status backend_init(const backend_config& config) noexcept;
bool backend_connected() noexcept;
void backend_signal_tx() noexcept;
capabilities backend_capabilities() noexcept;

// The backend owns the RTOS synchronization primitive. These functions are
// used only from ThreadX context and never from a peripheral ISR.
bool backend_lock() noexcept;
void backend_unlock() noexcept;

void connection_from_backend(bool is_connected) noexcept;
void receive_started_from_backend() noexcept;
void transmit_woken_from_backend() noexcept;
void receive_from_backend(const uint8_t* data, uint16_t length,
                          types::status status,
                          uint32_t native_status) noexcept;
bool prepare_tx_for_backend(uint8_t* output, uint16_t capacity,
                            uint16_t& length) noexcept;
void transmit_complete_from_backend(uint16_t requested,
                                    uint16_t actual,
                                    types::status status,
                                    uint32_t native_status) noexcept;

} // namespace bsp::usb::detail
