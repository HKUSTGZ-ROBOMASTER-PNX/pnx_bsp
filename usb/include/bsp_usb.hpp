#pragma once

#include "config.hpp"
#include "usertypes.hpp"

#include <cstddef>
#include <cstdint>
#include <cstring>

namespace bsp::usb
{

enum class link_state : uint8_t
{
    uninitialized = 0,
    initialized,
    connected,
    fault,
};

struct capabilities
{
    bool device_mode = false;
    bool high_speed = false;
    bool supports_vbus_sense = false;
    uint16_t max_packet_size = 0;
    uint16_t max_transfer_size = 0;
    uint8_t tx_queue_depth = 0;
};

struct config
{
    uint8_t read_priority = static_cast<uint8_t>(params::usb::read_thread_priority);
    uint8_t write_priority = static_cast<uint8_t>(params::usb::write_thread_priority);
    uint32_t period_ticks = params::usb::period_ticks;
    uint16_t min_rx_size = 1;
    uint16_t max_tx_size = 64;
    void (*on_rx)(const uint8_t* data, uint16_t len, void* user) = nullptr;
    uint16_t (*fill_tx)(uint8_t* buf, uint16_t cap, void* user) = nullptr;
    void (*on_tx_done)(uint16_t requested_len, uint16_t actual_len, uint32_t status, void* user) = nullptr;
    void* user = nullptr;
};

struct runtime_state
{
    bool initialized = false;
    bool connected = false;
    link_state link = link_state::uninitialized;
    bool read_busy = false;
    bool write_busy = false;
    uint32_t last_read_status = 0;
    uint32_t last_write_status = 0;
    uint16_t last_read_len = 0;
    uint16_t last_write_requested = 0;
    uint16_t last_write_actual = 0;
    uint16_t pending_write_len = 0;
    uint32_t read_count = 0;
    uint32_t write_count = 0;
    uint32_t error_count = 0;
    uint32_t tx_wake_count = 0;
    uint32_t fill_count = 0;
    uint32_t rx_bytes = 0;
    uint32_t tx_bytes = 0;
    uint32_t tx_drop_count = 0;
    uint32_t connect_count = 0;
    uint32_t disconnect_count = 0;
    uint8_t tx_queue_size = 0;
    uint8_t tx_queue_high_water = 0;
};

struct write_result
{
    types::status status = types::status::error;
    uint16_t accepted = 0;
};

// Accepts the configuration, retains its callbacks/user context, creates the
// required resources and schedules asynchronous controller startup. An `ok`
// result does not mean the device has enumerated or that the transport is
// connected/ready. Repeating init with the identical configuration is
// idempotent; an incompatible configuration is rejected so callback ownership
// cannot be replaced silently.
//
// Callbacks execute in implementation-owned thread context, never retain their
// provided buffers beyond the call, and must remain bounded/non-blocking.
// Disconnect may suppress completion for a retired transfer.
types::status init(const config& cfg) noexcept;

// Copies a complete transfer into a bounded two-entry queue. This API is for
// thread context only; it never retains the caller's buffer. Calls made before
// connected() becomes true, after disconnect, or while the bounded queue cannot
// accept the transfer return an explicit non-ok status without waiting.
write_result write(const uint8_t* data, std::size_t len) noexcept;

// True only while the selected board's CDC transport is usable.
bool connected() noexcept;
runtime_state snapshot() noexcept;
capabilities get_capabilities() noexcept;

// Compatibility view for existing callers. New code should use snapshot() so
// it never keeps a reference to mutable telemetry.
const runtime_state& state() noexcept;

namespace detail
{

template <typename Rx, typename Tx>
struct typed_link_binding
{
    inline static void (*on_rx_fn)(const Rx&, void*) = nullptr;
    inline static bool (*fill_tx_fn)(Tx&, void*) = nullptr;
    inline static void (*on_tx_done_fn)(uint16_t, uint16_t, uint32_t, void*) = nullptr;
    inline static void* user_ptr = nullptr;

    static void on_rx_raw(const uint8_t* data, uint16_t len, void* /*user*/)
    {
        if (len < sizeof(Rx) || on_rx_fn == nullptr)
        {
            return;
        }
        on_rx_fn(*reinterpret_cast<const Rx*>(data), user_ptr);
    }

    static uint16_t fill_tx_raw(uint8_t* buf, uint16_t cap, void* /*user*/)
    {
        if (fill_tx_fn == nullptr || cap < sizeof(Tx))
        {
            return 0;
        }
        Tx packet{};
        if (!fill_tx_fn(packet, user_ptr))
        {
            return 0;
        }
        std::memcpy(buf, &packet, sizeof(Tx));
        return sizeof(Tx);
    }

    static void on_tx_done_raw(uint16_t requested_len, uint16_t actual_len, uint32_t status, void* /*user*/)
    {
        if (on_tx_done_fn != nullptr)
        {
            on_tx_done_fn(requested_len, actual_len, status, user_ptr);
        }
    }
};

} // namespace detail

template <typename Rx, typename Tx>
config make_config(void (*on_rx_typed)(const Rx&, void*), bool (*fill_tx_typed)(Tx&, void*),
                   void (*on_tx_done_typed)(uint16_t, uint16_t, uint32_t, void*) = nullptr,
                   void* user = nullptr)
{
    static_assert(sizeof(Rx) > 0, "Rx must be non-empty");
    static_assert(sizeof(Tx) > 0, "Tx must be non-empty");

    detail::typed_link_binding<Rx, Tx>::on_rx_fn = on_rx_typed;
    detail::typed_link_binding<Rx, Tx>::fill_tx_fn = fill_tx_typed;
    detail::typed_link_binding<Rx, Tx>::on_tx_done_fn = on_tx_done_typed;
    detail::typed_link_binding<Rx, Tx>::user_ptr = user;

    config cfg{};
    cfg.min_rx_size = sizeof(Rx);
    cfg.max_tx_size = sizeof(Tx);
    cfg.user = user;
    cfg.on_rx = detail::typed_link_binding<Rx, Tx>::on_rx_raw;
    cfg.fill_tx = detail::typed_link_binding<Rx, Tx>::fill_tx_raw;
    cfg.on_tx_done = detail::typed_link_binding<Rx, Tx>::on_tx_done_raw;
    return cfg;
}

} // namespace bsp::usb
