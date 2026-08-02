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
// Thread-context recovery only; board ISR paths must only record or schedule it.
types::status recover(bus bus);
types::status transmit(bus bus, std::uint32_t id, const std::uint8_t* data,
                       std::uint16_t len);
// DEFERRED / PROVISIONAL -- no production caller as of 2026-08.
//
// Guarded transmit: sends only if the bus counters still match what the caller
// last observed via snapshot(), so a caller can refuse to transmit onto a bus
// that has degraded since its last check. Currently exercised only by the host
// contract tests (tests/host/can_host_tests.cpp).
//
// Intended sole consumer is a control-side CAN writer (e.g. chassis runtime)
// that needs transmit and health-check to be one decision. Until such a caller
// exists, treat this entry point as unstable: do not build new APIs on top of
// it, and re-evaluate whether it should stay in the shared contract when CAN is
// first integrated into a product application. Plain transmit() plus snapshot()
// covers every present use.
types::status transmit_if_healthy(
    bus bus, std::uint32_t id, const std::uint8_t* data,
    std::uint16_t len, std::uint32_t expected_error_count,
    std::uint32_t expected_drop_count,
    std::uint32_t expected_fault_epoch);

types::status register_rx_handler(bus bus, rx_handler handler, void* user_data);
void unregister_rx_handlers(bus bus);
telemetry snapshot(bus bus) noexcept;
std::uint32_t time_now() noexcept;

} // namespace bsp::can
