#pragma once

#include "config.hpp"
#include "usertypes.hpp"

#include <cstddef>
#include <cstdint>

namespace bsp::usart
{

enum class mode : std::uint8_t
{
    block = 0,
    dma = 1,
    it = 2,
};

struct rx_frame
{
    std::uint8_t* data = nullptr;
    std::size_t len = 0;
};

struct telemetry
{
    std::uint32_t rx_count = 0;
    std::uint32_t tx_count = 0;
    std::uint32_t error_count = 0;
    std::uint32_t busy_count = 0;
    std::uint32_t last_rx_len = 0;
};

using rx_handler = void (*)(port port, const rx_frame& frame, void* user_data);
using notify_handler = void (*)(void* user_data);

bool port_enabled(port port) noexcept;
types::status init(port port, mode mode);
types::status transmit(port port, const std::uint8_t* data, std::size_t len,
                       std::uint32_t timeout_ms);
types::status start_rx_to_idle(port port, std::uint8_t* buffer, std::size_t len,
                               rx_handler handler, void* user_data,
                               notify_handler notify = nullptr,
                               void* notify_user_data = nullptr);
types::status restart_rx(port port);
telemetry snapshot(port port) noexcept;

namespace detail
{

types::status backend_init(port port, mode mode) noexcept;
types::status backend_transmit(port port, mode mode, const std::uint8_t* data,
                               std::size_t len, std::uint32_t timeout_ms) noexcept;
types::status backend_start_rx(port port, mode mode, std::uint8_t* buffer,
                               std::size_t len) noexcept;
types::status backend_restart_rx(port port) noexcept;

void rx_from_isr(port port, std::size_t len) noexcept;
void error_from_isr(port port) noexcept;
void tx_complete_from_isr(port port) noexcept;

} // namespace detail

} // namespace bsp::usart
