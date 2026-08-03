#pragma once

#include "bsp_usart.hpp"

#include <atomic>
#include <cstddef>
#include <cstdint>

namespace board::usart
{

static_assert(std::atomic<bool>::is_always_lock_free,
              "UART ISR gate requires lock-free atomics");

class async_gate
{
  public:
    bool try_acquire() noexcept
    {
        bool expected = false;
        return busy_.compare_exchange_strong(
            expected, true, std::memory_order_acq_rel,
            std::memory_order_relaxed);
    }

    void release() noexcept
    {
        busy_.store(false, std::memory_order_release);
    }

  private:
    std::atomic<bool> busy_{false};
};

class async_gate_lease
{
  public:
    explicit async_gate_lease(async_gate& gate) noexcept
        : gate_(gate.try_acquire() ? &gate : nullptr)
    {
    }

    async_gate_lease(const async_gate_lease&) = delete;
    async_gate_lease& operator=(const async_gate_lease&) = delete;

    ~async_gate_lease()
    {
        if (gate_ != nullptr)
        {
            gate_->release();
        }
    }

    bool acquired() const noexcept
    {
        return gate_ != nullptr;
    }

    void hand_off_to_isr() noexcept
    {
        gate_ = nullptr;
    }

  private:
    async_gate* gate_;
};

// Both retained STM32 HAL versions can leave the DMAT request bit set after a
// DMA error has already ended TX and restored gState to READY. The explicit
// DMA-error input distinguishes that terminal path from an RX-only error that
// occurs while an async TX still owns the staging buffer.
constexpr bool async_tx_error_releases_gate(
    bool tx_in_flight, bool hal_tx_state_busy,
    bool dma_tx_request_active,
    bool dma_error_reported) noexcept
{
    return tx_in_flight && !hal_tx_state_busy &&
           (!dma_tx_request_active || dma_error_reported);
}

struct rx_segment
{
    std::size_t offset = 0U;
    std::size_t len = 0U;
};

struct circular_rx_update
{
    rx_segment first{};
    rx_segment second{};
    std::size_t next = 0U;
};

enum class rx_event
{
    idle,
    half_transfer,
    transfer_complete,
};

constexpr circular_rx_update
circular_rx_segments(std::size_t cursor, std::size_t position,
                     std::size_t capacity, rx_event event) noexcept
{
    if (capacity == 0U || cursor >= capacity || position > capacity)
    {
        return {};
    }

    // HAL reports Size == capacity for TC and can immediately report the same
    // position for IDLE. Normalize the latter to zero so TC -> IDLE cannot
    // dispatch the entire buffer twice.
    if (event == rx_event::transfer_complete)
    {
        if (position != capacity)
        {
            return {};
        }
        return {{cursor, capacity - cursor}, {}, 0U};
    }

    const std::size_t normalized =
        position == capacity ? 0U : position;
    if (normalized == cursor)
    {
        return {{cursor, 0U}, {}, cursor};
    }
    if (normalized > cursor)
    {
        return {{cursor, normalized - cursor}, {}, normalized};
    }
    return {{cursor, capacity - cursor}, {0U, normalized}, normalized};
}

struct rx_activation_state
{
    std::uint8_t* buffer = nullptr;
    std::size_t len = 0U;
    std::size_t cursor = 0U;
    bsp::usart::rx_delivery delivery =
        bsp::usart::rx_delivery::frame_snapshot;
};

template <typename Starter>
types::status activate_rx(
    rx_activation_state& state, std::uint8_t* buffer,
    std::size_t len, bsp::usart::rx_delivery delivery,
    Starter start) noexcept
{
    const rx_activation_state previous = state;
    state.buffer = buffer;
    state.len = len;
    state.cursor = 0U;
    state.delivery = delivery;

    const types::status result = start();
    if (result != types::status::ok)
    {
        state = previous;
    }
    return result;
}

} // namespace board::usart
