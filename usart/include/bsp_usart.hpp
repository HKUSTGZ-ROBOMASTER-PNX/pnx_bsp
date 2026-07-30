#pragma once

#include "config.hpp"
#include "usertypes.hpp"

#include <array>
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

enum class word_length : std::uint8_t
{
    bits_8 = 0,
    bits_9,
};

enum class stop_bits : std::uint8_t
{
    one = 0,
    two,
};

enum class parity : std::uint8_t
{
    none = 0,
    even,
    odd,
};

struct line_config
{
    std::uint32_t baud_rate = 115200U;
    word_length data_bits = word_length::bits_8;
    stop_bits stop = stop_bits::one;
    parity parity_mode = parity::none;
    bool enable_tx = true;
    bool enable_rx = true;
};

enum class rx_delivery : std::uint8_t
{
    // Preserve the original ReceiveToIdle contract: ignore DMA HT/TC events
    // for circular DMA and deliver one buffer-start snapshot on IDLE.
    frame_snapshot = 0,
    // Deliver newly written circular-DMA segments on HT, TC and IDLE.
    stream_segments = 1,
};

inline constexpr std::size_t dma_cache_line_size = 32U;

template <std::size_t LogicalSize>
struct alignas(dma_cache_line_size) dma_rx_storage
{
    static_assert(LogicalSize > 0U);
    static constexpr std::size_t logical_size = LogicalSize;
    static constexpr std::size_t storage_size =
        ((LogicalSize + dma_cache_line_size - 1U) /
         dma_cache_line_size) *
        dma_cache_line_size;

    std::array<std::uint8_t, storage_size> bytes{};

    std::uint8_t* data() noexcept
    {
        return bytes.data();
    }

    const std::uint8_t* data() const noexcept
    {
        return bytes.data();
    }
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

// RX handlers and notify handlers execute from the UART/DMA ISR. They must be
// constant-time and non-blocking. The RX buffer must remain valid for the
// lifetime of the active reception. On cache-equipped boards use
// dma_rx_storage so cache maintenance cannot touch unrelated objects.
bool port_enabled(port port) noexcept;
types::status init(port port, mode mode);
// DMA/IT transmit copies at most 256 bytes into board-owned staging storage;
// the caller may release or reuse data after this function returns.
types::status configure(port port, const line_config& config);
types::status transmit(port port, const std::uint8_t* data, std::size_t len,
                       std::uint32_t timeout_ms);
types::status start_rx_to_idle(port port, std::uint8_t* buffer, std::size_t len,
                               rx_handler handler, void* user_data,
                               notify_handler notify = nullptr,
                               void* notify_user_data = nullptr,
                               rx_delivery delivery =
                                   rx_delivery::frame_snapshot);
types::status restart_rx(port port);
telemetry snapshot(port port) noexcept;

} // namespace bsp::usart
