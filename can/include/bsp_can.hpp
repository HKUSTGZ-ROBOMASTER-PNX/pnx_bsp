#pragma once

#include "callback.hpp"
#include "status.hpp"
#include "tx_api.h"

#include <cstdint>

namespace bsp::can
{
enum class handle : uint8_t
{
    none,
    can1,
    can2,
    fdcan1,
    fdcan2,
    fdcan3
};
enum class frame_mode : uint8_t
{
    classic,
    fd_no_brs,
    fd_brs
};

enum class id_type : uint8_t
{
    standard,
    extended
};

struct rx_frame
{
    uint32_t id = 0;
    uint8_t len = 0;
    id_type id_kind = id_type::standard;
    frame_mode format = frame_mode::classic;
    bool remote = false;
    uint8_t data[64]{};
};

struct config
{
    handle instance = handle::none;
    id_type filter = id_type::standard;
};

struct fd_config
{
    handle instance = handle::none;
    id_type filter = id_type::standard;
    bool brs = true;
    bool tdc_enabled = false;
    uint8_t tdc_offset = 0;
    uint8_t tdc_filter = 0;
};

inline constexpr uint8_t max_rx_callbacks = 8;

// Borrowed target and frame; invoked in IRQ context, must not block.
using rx_callback = pnxtypes::callback<void(handle, const rx_frame&)>;

// Call after CubeMX initialization, from ThreadX initialization or a task.
// Serialize lifecycle and transmit calls from foreground tasks.
pnxtypes::status init(const config& resource) noexcept;
pnxtypes::status init(const fd_config& resource) noexcept;

pnxtypes::status stop(handle instance) noexcept;
pnxtypes::status restart(handle instance) noexcept;
pnxtypes::status current_status(handle instance) noexcept;
// Zero-length data frames are supported. FD payloads are padded to a legal DLC.
pnxtypes::status transmit(handle instance, uint32_t id, const uint8_t* data, uint16_t len) noexcept;

// Registration is allowed only before init or after successful stop.
// The target must outlive registration. Unregister before destroying it.
pnxtypes::status register_rx_callback(handle instance, rx_callback callback) noexcept;
pnxtypes::status unregister_rx_callbacks(handle instance) noexcept;

// Errors signal this semaphore in IRQ context. Bus-off marks the controller
// unavailable; the consuming task calls restart() to recover explicitly.
TX_SEMAPHORE* err_sem(handle instance) noexcept;
} // namespace bsp::can
