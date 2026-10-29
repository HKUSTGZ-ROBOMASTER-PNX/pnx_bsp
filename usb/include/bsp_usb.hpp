#pragma once

#include "status.hpp"
#include "callback.hpp"

#include <cstdint>

namespace bsp::usb
{
// Called once by tx_application_define, after CubeMX initializes the USB PCD.
pnxtypes::status init() noexcept;
bool connected() noexcept;

using rx_callback = pnxtypes::callback<void(const uint8_t*, uint32_t)>;

// Register before polling; {} unregisters. Returns busy during poll/callback.
// The callback target must live until successfully replaced or unregistered.
pnxtypes::status on_rx(rx_callback callback) noexcept;
// Read up to 64 bytes and dispatch in the calling thread, without creating a worker.
// Choose poll or direct read for RX. Bytes are valid only during the callback;
// CDC chunks are not message boundaries. Partial data is delivered even on error.
pnxtypes::status poll(uint32_t timeout_ms = 1000) noexcept;

// Thread context only. One reader and one writer may run concurrently.
// Buffers live until return. transferred may be nonzero on failure.
// Timeout is per USB transfer, in milliseconds; it is not a whole-call deadline.
// RX size must be a positive multiple of the full-speed OUT packet size (64 bytes).
pnxtypes::status read(uint8_t* data, uint32_t size, uint32_t& transferred,
                      uint32_t timeout_ms = 1000) noexcept;
pnxtypes::status write(const uint8_t* data, uint32_t size, uint32_t& transferred,
                       uint32_t timeout_ms = 1000) noexcept;
} // namespace bsp::usb
