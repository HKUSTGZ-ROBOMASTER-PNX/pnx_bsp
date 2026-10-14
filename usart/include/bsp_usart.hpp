#pragma once
#include "config.hpp"
#include "memory.hpp"
#include "callback.hpp"
#include "status.hpp"
#include "tx_api.h"

namespace bsp::usart
{
enum class mode : uint8_t
{
    block,
    dma
};
enum class word_length : uint8_t
{
    bits_8,
    bits_9
};
enum class stop_bits : uint8_t
{
    one,
    two
};
enum class parity : uint8_t
{
    none,
    even,
    odd
};

struct line_config
{
    uint32_t baud_rate = 115200;
    word_length data_bits = word_length::bits_8;
    stop_bits stop = stop_bits::one;
    parity parity_mode = parity::none;
    bool enable_tx = true;
    bool enable_rx = true;
};

// Borrowed frame/target, invoked in IRQ context. Copy data before returning if
// needed later; circular DMA can overwrite it. Callback must not block.
using rx_callback = pnxtypes::callback<void(handle, uint8_t*, uint32_t)>;
// Serialize lifecycle/RX ownership changes from foreground tasks.
pnxtypes::status init(const config& resource, mode method = mode::block) noexcept;
pnxtypes::status configure(handle instance, const line_config& settings) noexcept;
// Lengths are bytes. 9-bit/no-parity words occupy two aligned bytes each.
// DMA TX copies at most 256 bytes into an internal slot; overlapping submissions return busy.
pnxtypes::status transmit(handle instance, const uint8_t* data, uint32_t size,
                          uint32_t timeout_ms) noexcept;
pnxtypes::status abort_tx(handle instance) noexcept;
pnxtypes::status start_rx_to_idle(handle instance, uint8_t* data, uint32_t size, uint32_t capacity,
                                  rx_callback callback, TX_SEMAPHORE* notify = nullptr) noexcept;
pnxtypes::status restart_rx(handle instance) noexcept;
pnxtypes::status rx_status(handle instance) noexcept;
pnxtypes::status stop_rx(handle instance) noexcept;

// The buffer and callback target must live until stop_rx succeeds.
template <uint32_t N>
pnxtypes::status start_rx_to_idle(handle instance, memory::buffer<N>& data, rx_callback callback,
                                  TX_SEMAPHORE* notify = nullptr) noexcept
{
    return start_rx_to_idle(instance, data.data(), N, data.capacity(), callback, notify);
}
} // namespace bsp::usart
