#include "bsp_usart.hpp"
#include "usart_support.hpp"

#include "stm32f4xx_hal.h"
#include "usart.h"

#include <array>
#include <atomic>
#include <cstdint>
#include <cstring>
#include <limits>

namespace
{

constexpr std::size_t tx_stage_size = 256U;
constexpr std::uintptr_t dma_sram_begin = 0x20000000UL;
constexpr std::uintptr_t dma_sram_end = 0x20020000UL;

struct hardware_state
{
    board::usart::async_gate tx_gate{};
    std::atomic<bool> tx_in_flight{false};
    bsp::usart::mode active_mode = bsp::usart::mode::block;
    board::usart::rx_activation_state rx{};
};

struct port_state
{
    bool initialized = false;
    bsp::usart::mode active_mode = bsp::usart::mode::block;
    bsp::usart::rx_handler handler = nullptr;
    void* user_data = nullptr;
    bsp::usart::notify_handler notify = nullptr;
    void* notify_user_data = nullptr;
    bsp::usart::rx_delivery delivery =
        bsp::usart::rx_delivery::frame_snapshot;
    std::uint8_t* rx_buffer = nullptr;
    std::size_t rx_buffer_len = 0U;
    std::atomic<std::uint32_t> rx_count{0U};
    std::atomic<std::uint32_t> tx_count{0U};
    std::atomic<std::uint32_t> error_count{0U};
    std::atomic<std::uint32_t> busy_count{0U};
    std::atomic<std::uint32_t> last_rx_len{0U};
};

static_assert(std::atomic<bool>::is_always_lock_free,
              "UART ISR state requires lock-free atomics");
static_assert(std::atomic<std::uint32_t>::is_always_lock_free,
              "UART ISR telemetry requires lock-free atomics");

std::array<hardware_state, bsp::usart::port_count> hardware_states{};
std::array<port_state, bsp::usart::port_count> port_states{};
alignas(4)
std::uint8_t tx_stage[bsp::usart::port_count][tx_stage_size]{};

hardware_state* hardware_state_of(
    bsp::usart::port selected) noexcept
{
    return selected < bsp::usart::port_count
               ? &hardware_states[selected]
               : nullptr;
}

port_state* port_state_of(bsp::usart::port selected) noexcept
{
    return selected < bsp::usart::port_count
               ? &port_states[selected]
               : nullptr;
}

const bsp::usart::port_config* config_of(
    bsp::usart::port selected) noexcept
{
    return selected < bsp::usart::port_count
               ? &bsp::usart::configs[selected]
               : nullptr;
}

UART_HandleTypeDef* handle_from_id(
    bsp::usart::handle_id id) noexcept
{
    switch (id)
    {
    case bsp::usart::handle_id::usart1:
        return &huart1;
    case bsp::usart::handle_id::usart3:
        return &huart3;
    case bsp::usart::handle_id::usart6:
        return &huart6;
    default:
        return nullptr;
    }
}

UART_HandleTypeDef* handle_of(
    bsp::usart::port selected) noexcept
{
    if (selected >= bsp::usart::port_count ||
        !bsp::usart::configs[selected].enabled)
    {
        return nullptr;
    }
    return handle_from_id(
        bsp::usart::configs[selected].handle);
}

bsp::usart::port port_of(
    UART_HandleTypeDef* handle) noexcept
{
    for (bsp::usart::port selected = 0U;
         selected < bsp::usart::port_count; ++selected)
    {
        if (handle_of(selected) == handle)
        {
            return selected;
        }
    }
    return bsp::usart::port_count;
}

bool dma_accessible(
    const void* buffer, std::size_t len) noexcept
{
    if (buffer == nullptr || len == 0U)
    {
        return false;
    }
    const auto begin =
        reinterpret_cast<std::uintptr_t>(buffer);
    if (len - 1U >
        std::numeric_limits<std::uintptr_t>::max() - begin)
    {
        return false;
    }
    const auto last = begin + len - 1U;
    return begin >= dma_sram_begin && last < dma_sram_end;
}

bool circular_rx(
    UART_HandleTypeDef* handle) noexcept
{
    return handle != nullptr && handle->hdmarx != nullptr &&
           handle->hdmarx->Init.Mode == DMA_CIRCULAR;
}

types::status to_status(
    HAL_StatusTypeDef status) noexcept
{
    if (status == HAL_OK)
    {
        return types::status::ok;
    }
    return status == HAL_BUSY ? types::status::busy
                              : types::status::error;
}

types::status start_dma_rx(
    bsp::usart::port selected, std::uint8_t* buffer,
    std::size_t len,
    bsp::usart::rx_delivery delivery) noexcept
{
    UART_HandleTypeDef* handle = handle_of(selected);
    if (handle == nullptr || !dma_accessible(buffer, len) ||
        len > std::numeric_limits<std::uint16_t>::max())
    {
        return types::status::invalid_arg;
    }

    const HAL_StatusTypeDef status =
        HAL_UARTEx_ReceiveToIdle_DMA(
            handle, buffer, static_cast<std::uint16_t>(len));
    if (status != HAL_OK)
    {
        return to_status(status);
    }
    if (handle->hdmarx != nullptr)
    {
        if (circular_rx(handle) &&
            delivery ==
                bsp::usart::rx_delivery::stream_segments)
        {
            __HAL_DMA_ENABLE_IT(handle->hdmarx, DMA_IT_HT);
        }
        else
        {
            __HAL_DMA_DISABLE_IT(handle->hdmarx, DMA_IT_HT);
        }
    }
    return types::status::ok;
}

bool release_tx(bsp::usart::port selected) noexcept
{
    hardware_state* ctx = hardware_state_of(selected);
    if (ctx == nullptr ||
        !ctx->tx_in_flight.exchange(
            false, std::memory_order_acq_rel))
    {
        return false;
    }
    ctx->tx_gate.release();
    return true;
}

} // namespace

namespace bsp::usart
{
namespace
{

types::status hardware_init(
    port selected, mode selected_mode) noexcept
{
    UART_HandleTypeDef* handle = handle_of(selected);
    hardware_state* ctx = hardware_state_of(selected);
    if (handle == nullptr || ctx == nullptr)
    {
        return types::status::invalid_arg;
    }
    __HAL_UART_FLUSH_DRREGISTER(handle);
    ctx->active_mode = selected_mode;
    return types::status::ok;
}

types::status hardware_transmit(
    port selected, mode selected_mode,
    const std::uint8_t* data, std::size_t len,
    std::uint32_t timeout_ms) noexcept
{
    UART_HandleTypeDef* handle = handle_of(selected);
    hardware_state* ctx = hardware_state_of(selected);
    if (handle == nullptr || ctx == nullptr ||
        data == nullptr || len == 0U ||
        len > std::numeric_limits<std::uint16_t>::max())
    {
        return types::status::invalid_arg;
    }

    if (selected_mode == mode::block)
    {
        return to_status(HAL_UART_Transmit(
            handle, const_cast<std::uint8_t*>(data),
            static_cast<std::uint16_t>(len), timeout_ms));
    }
    if (len > tx_stage_size)
    {
        return types::status::invalid_arg;
    }
    if (selected_mode == mode::dma &&
        !configs[selected].has_tx_dma)
    {
        return types::status::not_configured;
    }

    board::usart::async_gate_lease lease(ctx->tx_gate);
    if (!lease.acquired())
    {
        return types::status::busy;
    }
    std::memcpy(tx_stage[selected], data, len);
    ctx->tx_in_flight.store(
        true, std::memory_order_release);
    const HAL_StatusTypeDef status =
        selected_mode == mode::dma
            ? HAL_UART_Transmit_DMA(
                  handle, tx_stage[selected],
                  static_cast<std::uint16_t>(len))
            : HAL_UART_Transmit_IT(
                  handle, tx_stage[selected],
                  static_cast<std::uint16_t>(len));
    if (status != HAL_OK)
    {
        ctx->tx_in_flight.store(
            false, std::memory_order_release);
        return to_status(status);
    }
    lease.hand_off_to_isr();
    return types::status::ok;
}

types::status hardware_start_rx(
    port selected, mode selected_mode,
    std::uint8_t* buffer, std::size_t len,
    rx_delivery delivery) noexcept
{
    hardware_state* ctx = hardware_state_of(selected);
    if (ctx == nullptr || selected_mode != mode::dma ||
        !configs[selected].has_rx_dma)
    {
        return types::status::invalid_arg;
    }

    const std::uint32_t interrupt_state = __get_PRIMASK();
    __disable_irq();
    const types::status status = board::usart::activate_rx(
        ctx->rx, buffer, len, delivery, [&]() noexcept {
            return start_dma_rx(
                selected, buffer, len, delivery);
        });
    __set_PRIMASK(interrupt_state);
    return status;
}

types::status hardware_restart_rx(
    port selected) noexcept
{
    hardware_state* ctx = hardware_state_of(selected);
    if (ctx == nullptr || ctx->rx.buffer == nullptr)
    {
        return types::status::invalid_arg;
    }
    const types::status status = start_dma_rx(
        selected, ctx->rx.buffer, ctx->rx.len,
        ctx->rx.delivery);
    if (status == types::status::ok)
    {
        ctx->rx.cursor = 0U;
    }
    return status;
}

} // namespace

bool port_enabled(port selected) noexcept
{
    const port_config* cfg = config_of(selected);
    return cfg != nullptr && cfg->enabled;
}

types::status init(port selected, mode selected_mode)
{
    port_state* ctx = port_state_of(selected);
    if (ctx == nullptr)
    {
        return types::status::invalid_arg;
    }
    if (!port_enabled(selected))
    {
        return types::status::not_configured;
    }
    if (ctx->initialized)
    {
        return ctx->active_mode == selected_mode ? types::status::ok
                                                 : types::status::invalid_arg;
    }

    const types::status status = hardware_init(selected, selected_mode);
    if (status != types::status::ok)
    {
        ctx->error_count.fetch_add(1U, std::memory_order_relaxed);
        return status;
    }
    ctx->active_mode = selected_mode;
    ctx->initialized = true;
    return types::status::ok;
}

types::status configure(port selected, const line_config& line)
{
    if (selected >= port_count || line.baud_rate == 0U ||
        (!line.enable_tx && !line.enable_rx))
    {
        return types::status::invalid_arg;
    }
    if (!port_enabled(selected))
    {
        return types::status::not_configured;
    }
    port_state* ctx = port_state_of(selected);
    hardware_state* hw = hardware_state_of(selected);
    UART_HandleTypeDef* handle = handle_of(selected);
    if (ctx == nullptr || hw == nullptr || handle == nullptr)
    {
        return types::status::invalid_arg;
    }
    if (ctx->rx_buffer != nullptr ||
        hw->tx_in_flight.load(std::memory_order_acquire))
    {
        return types::status::busy;
    }

    switch (line.data_bits)
    {
    case word_length::bits_8:
        handle->Init.WordLength = UART_WORDLENGTH_8B;
        break;
    case word_length::bits_9:
        handle->Init.WordLength = UART_WORDLENGTH_9B;
        break;
    default:
        return types::status::invalid_arg;
    }

    switch (line.stop)
    {
    case stop_bits::one:
        handle->Init.StopBits = UART_STOPBITS_1;
        break;
    case stop_bits::two:
        handle->Init.StopBits = UART_STOPBITS_2;
        break;
    default:
        return types::status::invalid_arg;
    }

    switch (line.parity_mode)
    {
    case parity::none:
        handle->Init.Parity = UART_PARITY_NONE;
        break;
    case parity::even:
        handle->Init.Parity = UART_PARITY_EVEN;
        break;
    case parity::odd:
        handle->Init.Parity = UART_PARITY_ODD;
        break;
    default:
        return types::status::invalid_arg;
    }

    handle->Init.BaudRate = line.baud_rate;
    handle->Init.Mode =
        (line.enable_tx ? UART_MODE_TX : 0U) |
        (line.enable_rx ? UART_MODE_RX : 0U);
    handle->Init.HwFlowCtl = UART_HWCONTROL_NONE;
    if (HAL_UART_Init(handle) != HAL_OK)
    {
        ctx->error_count.fetch_add(
            1U, std::memory_order_relaxed);
        return types::status::error;
    }

    ctx->initialized = false;
    return types::status::ok;
}

types::status transmit(port selected, const std::uint8_t* data,
                       std::size_t len, std::uint32_t timeout_ms)
{
    port_state* ctx = port_state_of(selected);
    if (ctx == nullptr || data == nullptr || len == 0U ||
        len > std::numeric_limits<std::uint16_t>::max())
    {
        return types::status::invalid_arg;
    }
    if (!port_enabled(selected))
    {
        return types::status::not_configured;
    }
    if (!ctx->initialized)
    {
        return types::status::error;
    }

    const types::status status = hardware_transmit(
        selected, ctx->active_mode, data, len, timeout_ms);
    if (status == types::status::ok)
    {
        if (ctx->active_mode == mode::block)
        {
            ctx->tx_count.fetch_add(1U, std::memory_order_relaxed);
        }
    }
    else if (status == types::status::busy)
    {
        ctx->busy_count.fetch_add(1U, std::memory_order_relaxed);
    }
    else
    {
        ctx->error_count.fetch_add(1U, std::memory_order_relaxed);
    }
    return status;
}

types::status start_rx_to_idle(port selected, std::uint8_t* buffer,
                               std::size_t len, rx_handler handler,
                               void* user_data, notify_handler notify,
                               void* notify_user_data,
                               rx_delivery delivery)
{
    port_state* ctx = port_state_of(selected);
    if (ctx == nullptr || buffer == nullptr || len == 0U ||
        len > std::numeric_limits<std::uint16_t>::max() ||
        !ctx->initialized)
    {
        return types::status::invalid_arg;
    }
    if (ctx->rx_buffer != nullptr &&
        (ctx->rx_buffer != buffer || ctx->rx_buffer_len != len ||
         ctx->handler != handler || ctx->user_data != user_data ||
         ctx->notify != notify ||
         ctx->notify_user_data != notify_user_data ||
         ctx->delivery != delivery))
    {
        return types::status::busy;
    }

    std::uint8_t* const previous_buffer = ctx->rx_buffer;
    const std::size_t previous_buffer_len = ctx->rx_buffer_len;
    const rx_handler previous_handler = ctx->handler;
    void* const previous_user_data = ctx->user_data;
    const notify_handler previous_notify = ctx->notify;
    void* const previous_notify_user_data = ctx->notify_user_data;
    const rx_delivery previous_delivery = ctx->delivery;

    ctx->rx_buffer = buffer;
    ctx->rx_buffer_len = len;
    ctx->handler = handler;
    ctx->user_data = user_data;
    ctx->notify = notify;
    ctx->notify_user_data = notify_user_data;
    ctx->delivery = delivery;

    const types::status status = hardware_start_rx(
        selected, ctx->active_mode, buffer, len, delivery);
    if (status != types::status::ok)
    {
        ctx->rx_buffer = previous_buffer;
        ctx->rx_buffer_len = previous_buffer_len;
        ctx->handler = previous_handler;
        ctx->user_data = previous_user_data;
        ctx->notify = previous_notify;
        ctx->notify_user_data = previous_notify_user_data;
        ctx->delivery = previous_delivery;
        ctx->error_count.fetch_add(1U, std::memory_order_relaxed);
    }
    return status;
}

types::status restart_rx(port selected)
{
    port_state* ctx = port_state_of(selected);
    if (ctx == nullptr || !ctx->initialized || ctx->rx_buffer == nullptr)
    {
        return types::status::invalid_arg;
    }
    const types::status status = hardware_restart_rx(selected);
    if (status == types::status::busy)
    {
        ctx->busy_count.fetch_add(1U, std::memory_order_relaxed);
    }
    else if (status != types::status::ok)
    {
        ctx->error_count.fetch_add(1U, std::memory_order_relaxed);
    }
    return status;
}

telemetry snapshot(port selected) noexcept
{
    telemetry result{};
    const port_state* ctx = port_state_of(selected);
    if (ctx == nullptr)
    {
        return result;
    }
    result.rx_count = ctx->rx_count.load(std::memory_order_relaxed);
    result.tx_count = ctx->tx_count.load(std::memory_order_relaxed);
    result.error_count =
        ctx->error_count.load(std::memory_order_relaxed);
    result.busy_count = ctx->busy_count.load(std::memory_order_relaxed);
    result.last_rx_len =
        ctx->last_rx_len.load(std::memory_order_relaxed);
    return result;
}

static void dispatch_rx_from_isr(port selected, std::size_t offset,
                                 std::size_t len) noexcept
{
    port_state* ctx = port_state_of(selected);
    if (ctx == nullptr || ctx->rx_buffer == nullptr ||
        offset > ctx->rx_buffer_len ||
        len > ctx->rx_buffer_len - offset)
    {
        return;
    }
    ctx->rx_count.fetch_add(1U, std::memory_order_relaxed);
    ctx->last_rx_len.store(
        static_cast<std::uint32_t>(len), std::memory_order_relaxed);
    const rx_frame frame{ctx->rx_buffer + offset, len};
    if (ctx->notify != nullptr)
    {
        ctx->notify(ctx->notify_user_data);
    }
    if (ctx->handler != nullptr)
    {
        ctx->handler(selected, frame, ctx->user_data);
    }
}

static void record_error_from_isr(port selected) noexcept
{
    port_state* ctx = port_state_of(selected);
    if (ctx != nullptr)
    {
        ctx->error_count.fetch_add(1U, std::memory_order_relaxed);
    }
}

static void record_tx_complete_from_isr(port selected) noexcept
{
    port_state* ctx = port_state_of(selected);
    if (ctx != nullptr)
    {
        ctx->tx_count.fetch_add(1U, std::memory_order_relaxed);
    }
}

} // namespace bsp::usart

extern "C" void HAL_UARTEx_RxEventCallback(
    UART_HandleTypeDef* handle, std::uint16_t size)
{
    const bsp::usart::port selected = port_of(handle);
    hardware_state* ctx = hardware_state_of(selected);
    if (ctx == nullptr || ctx->rx.buffer == nullptr ||
        size > ctx->rx.len)
    {
        return;
    }

    if (!circular_rx(handle))
    {
        if (HAL_UARTEx_GetRxEventType(handle) !=
            HAL_UART_RXEVENT_HT)
        {
            bsp::usart::dispatch_rx_from_isr(
                selected, 0U, size);
            (void)bsp::usart::hardware_restart_rx(selected);
        }
        return;
    }

    const auto event_type =
        HAL_UARTEx_GetRxEventType(handle);
    if (ctx->rx.delivery ==
        bsp::usart::rx_delivery::frame_snapshot)
    {
        if (event_type == HAL_UART_RXEVENT_IDLE &&
            size != 0U)
        {
            bsp::usart::dispatch_rx_from_isr(
                selected, 0U, size);
        }
        return;
    }

    const auto event =
        event_type == HAL_UART_RXEVENT_TC
            ? board::usart::rx_event::
                  transfer_complete
            : event_type == HAL_UART_RXEVENT_HT
                  ? board::usart::rx_event::
                        half_transfer
                  : board::usart::rx_event::idle;
    const auto update =
        board::usart::circular_rx_segments(
            ctx->rx.cursor, size, ctx->rx.len,
            event);
    if (update.first.len != 0U)
    {
        bsp::usart::dispatch_rx_from_isr(
            selected, update.first.offset,
            update.first.len);
    }
    if (update.second.len != 0U)
    {
        bsp::usart::dispatch_rx_from_isr(
            selected, update.second.offset,
            update.second.len);
    }
    ctx->rx.cursor = update.next;
}

extern "C" void HAL_UART_ErrorCallback(
    UART_HandleTypeDef* handle)
{
    const bsp::usart::port selected = port_of(handle);
    hardware_state* ctx = hardware_state_of(selected);
    if (ctx == nullptr)
    {
        return;
    }
    __HAL_UART_CLEAR_OREFLAG(handle);
    if (handle->gState != HAL_UART_STATE_BUSY_TX &&
        handle->gState != HAL_UART_STATE_BUSY_TX_RX)
    {
        (void)release_tx(selected);
    }
    bsp::usart::record_error_from_isr(selected);
    if (ctx->rx.buffer != nullptr)
    {
        (void)bsp::usart::hardware_restart_rx(selected);
    }
}

extern "C" void HAL_UART_TxCpltCallback(
    UART_HandleTypeDef* handle)
{
    const bsp::usart::port selected = port_of(handle);
    if (release_tx(selected))
    {
        bsp::usart::record_tx_complete_from_isr(selected);
    }
}

extern "C" void HAL_UART_AbortTransmitCpltCallback(
    UART_HandleTypeDef* handle)
{
    const bsp::usart::port selected = port_of(handle);
    (void)release_tx(selected);
    bsp::usart::record_error_from_isr(selected);
}

extern "C" void HAL_UART_AbortCpltCallback(
    UART_HandleTypeDef* handle)
{
    HAL_UART_AbortTransmitCpltCallback(handle);
}
