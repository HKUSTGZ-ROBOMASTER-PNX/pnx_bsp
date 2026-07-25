#pragma once

#include "config.hpp"
#include "usertypes.hpp"

#include <cstddef>
#include <cstdint>

namespace bsp::can
{

struct rx_frame
{
    std::uint32_t id = 0;
    std::uint32_t tick = 0;
    std::uint8_t len = 0;
    std::uint8_t data[64]{};
};

enum class state : std::uint8_t
{
    stopped = 0,
    active,
    warning,
    passive,
    bus_off,
    recovering,
    fault,
};

struct telemetry
{
    std::uint32_t rx_count = 0;
    std::uint32_t tx_count = 0;
    std::uint32_t last_id = 0;
    std::uint32_t last_tick = 0;
    std::uint32_t error_count = 0;
    std::uint32_t bus_off_count = 0;
    std::uint32_t drop_count = 0;
    std::uint32_t fault_epoch = 0;
    state bus_state = state::stopped;
};

using rx_handler = void (*)(bus bus, const rx_frame& frame, void* user_data);

bool bus_enabled(std::size_t index) noexcept;
bus_type configured_bus_type(std::size_t index) noexcept;
id_type filter_id_type_of(std::size_t index) noexcept;

types::status init(bus bus, bus_type type);
// Thread-context recovery only. ISRs report bus-off through error_from_isr().
types::status recover(bus bus);
types::status transmit(bus bus, std::uint32_t id, const std::uint8_t* data,
                       std::uint16_t len);
types::status transmit_if_healthy(
    bus bus, std::uint32_t id, const std::uint8_t* data,
    std::uint16_t len, std::uint32_t expected_error_count,
    std::uint32_t expected_drop_count,
    std::uint32_t expected_fault_epoch);

types::status register_rx_handler(bus bus, rx_handler handler, void* user_data);
void unregister_rx_handlers(bus bus);
telemetry snapshot(bus bus) noexcept;
std::uint32_t time_now() noexcept;

namespace detail
{

types::status backend_init(bus bus, bus_type type) noexcept;
types::status backend_transmit(bus bus, std::uint32_t id,
                               const std::uint8_t* data,
                               std::uint16_t len) noexcept;
types::status backend_recover(bus bus) noexcept;
std::uint32_t backend_tick_now() noexcept;
std::uint32_t backend_enter_critical() noexcept;
void backend_exit_critical(std::uint32_t state) noexcept;

void rx_from_isr(bus bus, const rx_frame& frame, std::uint32_t tick) noexcept;
void error_from_isr(bus bus, state next_state, std::uint32_t tick) noexcept;
void drop_from_isr(bus bus) noexcept;

} // namespace detail

} // namespace bsp::can
