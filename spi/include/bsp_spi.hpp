#pragma once
#include "config.hpp"
#include "memory.hpp"
#include "status.hpp"

namespace bsp::spi
{
struct transfer_state
{
    volatile bool busy = false;
    volatile bool complete = false;
    volatile bool receive = false;
    volatile bool dma = false;
    volatile pnxtypes::status last_status = pnxtypes::status::ok;
};

// CubeMX initializes hardware first; serialize foreground calls per instance.
pnxtypes::status wait_ready(handle instance, uint32_t timeout_ms = 1000) noexcept;
pnxtypes::status transmit(handle instance, const uint8_t* data, uint32_t size,
                          uint32_t timeout_ms) noexcept;
pnxtypes::status receive(handle instance, uint8_t* data, uint32_t size,
                         uint32_t timeout_ms) noexcept;
pnxtypes::status transmit_receive(handle instance, const uint8_t* tx, uint8_t* rx, uint32_t size,
                                  uint32_t timeout_ms) noexcept;
pnxtypes::status transmit_it(handle instance, const uint8_t* data, uint32_t size) noexcept;
pnxtypes::status receive_it(handle instance, uint8_t* data, uint32_t size) noexcept;
pnxtypes::status transmit_receive_it(handle instance, const uint8_t* tx, uint8_t* rx,
    uint32_t size) noexcept;
pnxtypes::status transmit_dma(handle instance, const uint8_t* data, uint32_t size,
                              uint32_t capacity) noexcept;
pnxtypes::status receive_dma(handle instance, uint8_t* data, uint32_t size,
                             uint32_t capacity) noexcept;
// Full-duplex DMA requires separate, non-overlapping cache-line-isolated allocations.
pnxtypes::status transmit_receive_dma(handle instance, const uint8_t* tx, uint8_t* rx,
    uint32_t size, uint32_t tx_capacity, uint32_t rx_capacity) noexcept;
// Borrow IT/DMA buffers until successful completion or abort. Error retains ownership.
pnxtypes::status abort(handle instance) noexcept;
const transfer_state& state(handle instance) noexcept;

template <uint32_t N>
pnxtypes::status transmit_dma(handle instance, const memory::buffer<N>& data) noexcept
{
    return transmit_dma(instance, data.data(), N, data.capacity());
}

template <uint32_t N>
pnxtypes::status receive_dma(handle instance, memory::buffer<N>& data) noexcept
{
    return receive_dma(instance, data.data(), N, data.capacity());
}

template <uint32_t N>
pnxtypes::status transmit_receive_dma(handle instance, const memory::buffer<N>& tx,
    memory::buffer<N>& rx) noexcept
{
    return transmit_receive_dma(instance, tx.data(), rx.data(), N, tx.capacity(), rx.capacity());
}
} // namespace bsp::spi
