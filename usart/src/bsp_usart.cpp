#include "bsp_usart.hpp"

#include "memory.h"

#include <cstdint>
#include <cstring>

extern "C" {
extern DMA_HandleTypeDef hdma_uart5_rx;
extern DMA_HandleTypeDef hdma_uart7_rx;
extern DMA_HandleTypeDef hdma_uart7_tx;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart1_tx;
}

namespace
{

constexpr uint16_t dma_tx_stage_size = 256;
constexpr std::uintptr_t ram_d1_start = 0x24000000UL;
constexpr std::uintptr_t ram_d1_end = 0x24050000UL;
constexpr std::uintptr_t ram_d2_start = 0x30000000UL;
constexpr std::uintptr_t ram_d2_end = 0x30008000UL;
constexpr std::uintptr_t ram_d3_start = 0x38000000UL;
constexpr std::uintptr_t ram_d3_end = 0x38004000UL;
constexpr std::uintptr_t dcache_line_size = 32U;

struct port_state
{
    bool initialized = false;
    bool tx_busy = false;
    bsp::usart::mode port_mode = bsp::usart::mode::block;
    bsp::usart::rx_handler handler = nullptr;
    void* user_data = nullptr;
    TX_SEMAPHORE* notify_sem = nullptr;
    uint8_t* rx_buffer = nullptr;
    size_t rx_buffer_len = 0;
};

port_state ports[bsp::usart::port_count]{};
alignas(dcache_line_size)
uint8_t tx_dma_stage_buffers[bsp::usart::port_count][dma_tx_stage_size] RAM_D1_BSS{};

port_state* state_of(bsp::usart::port port)
{
    if (static_cast<std::uint8_t>(port) >= bsp::usart::port_count)
    {
        return nullptr;
    }

    return &ports[port];
}

uint8_t* tx_dma_stage_of(bsp::usart::port port)
{
    if (static_cast<std::uint8_t>(port) >= bsp::usart::port_count)
    {
        return nullptr;
    }

    return tx_dma_stage_buffers[port];
}

std::uintptr_t align_down(std::uintptr_t value, std::uintptr_t alignment)
{
    return value & ~(alignment - 1U);
}

std::uintptr_t align_up(std::uintptr_t value, std::uintptr_t alignment)
{
    return (value + alignment - 1U) & ~(alignment - 1U);
}

bool in_range(std::uintptr_t addr, std::uintptr_t start, std::uintptr_t end)
{
    return addr >= start && addr < end;
}

bool is_dma_accessible_buffer(const void* buffer, size_t len)
{
    if (buffer == nullptr || len == 0)
    {
        return false;
    }

    const auto start = reinterpret_cast<std::uintptr_t>(buffer);
    const auto last = start + len - 1U;

    return (in_range(start, ram_d1_start, ram_d1_end) && in_range(last, ram_d1_start, ram_d1_end)) ||
           (in_range(start, ram_d2_start, ram_d2_end) && in_range(last, ram_d2_start, ram_d2_end)) ||
           (in_range(start, ram_d3_start, ram_d3_end) && in_range(last, ram_d3_start, ram_d3_end));
}

void clean_dcache_region(const void* buffer, size_t len)
{
#if defined(__DCACHE_PRESENT) && (__DCACHE_PRESENT == 1U)
    if (buffer == nullptr || len == 0)
    {
        return;
    }

    const auto start = reinterpret_cast<std::uintptr_t>(buffer);
    const auto aligned_start = align_down(start, dcache_line_size);
    const auto aligned_end = align_up(start + len, dcache_line_size);
    SCB_CleanDCache_by_Addr(reinterpret_cast<uint32_t*>(aligned_start),
                            static_cast<int32_t>(aligned_end - aligned_start));
#else
    (void)buffer;
    (void)len;
#endif
}

void invalidate_dcache_region(const void* buffer, size_t len)
{
#if defined(__DCACHE_PRESENT) && (__DCACHE_PRESENT == 1U)
    if (buffer == nullptr || len == 0)
    {
        return;
    }

    const auto start = reinterpret_cast<std::uintptr_t>(buffer);
    const auto aligned_start = align_down(start, dcache_line_size);
    const auto aligned_end = align_up(start + len, dcache_line_size);
    SCB_InvalidateDCache_by_Addr(reinterpret_cast<uint32_t*>(aligned_start),
                                 static_cast<int32_t>(aligned_end - aligned_start));
#else
    (void)buffer;
    (void)len;
#endif
}

void notify_rx(bsp::usart::port port, size_t len)
{
    port_state* ctx = state_of(port);
    if (ctx == nullptr || ctx->rx_buffer == nullptr)
    {
        return;
    }

    bsp::usart::rx_frame frame{ctx->rx_buffer, len};
    if (ctx->notify_sem != nullptr)
    {
        tx_semaphore_put(ctx->notify_sem);
    }
    if (ctx->handler != nullptr)
    {
        ctx->handler(port, frame, ctx->user_data);
    }
}

bool rx_dma_is_circular(UART_HandleTypeDef* handle)
{
    return handle != nullptr && handle->hdmarx != nullptr &&
           handle->hdmarx->Init.Mode == DMA_CIRCULAR;
}

void disable_rx_half_transfer_irq(UART_HandleTypeDef* handle)
{
    if (handle != nullptr && handle->hdmarx != nullptr)
    {
        __HAL_DMA_DISABLE_IT(handle->hdmarx, DMA_IT_HT);
    }
}

bool same_rx_owner(const port_state& ctx, uint8_t* buffer, size_t len, bsp::usart::rx_handler handler,
                   void* user_data, TX_SEMAPHORE* notify_sem)
{
    return ctx.rx_buffer == buffer && ctx.rx_buffer_len == len && ctx.handler == handler &&
           ctx.user_data == user_data && ctx.notify_sem == notify_sem;
}

} // namespace

namespace bsp::usart
{

namespace
{

const port_config* config_of(std::size_t index) noexcept
{
    return index < port_count ? &configs[index] : nullptr;
}

UART_HandleTypeDef* handle_from_id(handle_id id) noexcept
{
    switch (id)
    {
    case handle_id::uart5:
        return &huart5;
    case handle_id::uart7:
        return &huart7;
    case handle_id::usart1:
        return &huart1;
    default:
        return nullptr;
    }
}

DMA_HandleTypeDef* rx_dma_from_id(handle_id id) noexcept
{
    switch (id)
    {
    case handle_id::uart5:
        return &hdma_uart5_rx;
    case handle_id::uart7:
        return &hdma_uart7_rx;
    case handle_id::usart1:
        return &hdma_usart1_rx;
    default:
        return nullptr;
    }
}

DMA_HandleTypeDef* tx_dma_from_id(handle_id id) noexcept
{
    switch (id)
    {
    case handle_id::uart7:
        return &hdma_uart7_tx;
    case handle_id::usart1:
        return &hdma_usart1_tx;
    default:
        return nullptr;
    }
}

void enable_dma_irq(DMA_HandleTypeDef* dma) noexcept
{
    if (dma == nullptr)
    {
        return;
    }

    __HAL_DMA_DISABLE_IT(dma, DMA_IT_HT);
    __HAL_DMA_ENABLE_IT(dma, DMA_IT_TC);
}

} // namespace

UART_HandleTypeDef* handle_of(std::size_t index) noexcept
{
    const port_config* cfg = config_of(index);
    return cfg != nullptr && cfg->enabled ? handle_from_id(cfg->handle) : nullptr;
}

std::size_t index_of(UART_HandleTypeDef* handle) noexcept
{
    for (std::size_t i = 0; i < port_count; ++i)
    {
        if (handle_of(i) == handle)
        {
            return i;
        }
    }
    return port_count;
}

bool port_enabled(port port) noexcept
{
    const port_config* cfg = config_of(port);
    return cfg != nullptr && cfg->enabled;
}

void setup_dma(UART_HandleTypeDef* handle) noexcept
{
    const std::size_t index = index_of(handle);
    const port_config* cfg = config_of(index);
    if (cfg == nullptr || !cfg->enabled)
    {
        return;
    }

    if (cfg->has_rx_dma)
    {
        enable_dma_irq(rx_dma_from_id(cfg->handle));
    }
    if (cfg->has_tx_dma)
    {
        enable_dma_irq(tx_dma_from_id(cfg->handle));
    }
}

types::status restart_rx(port port)
{
    port_state* ctx = state_of(port);
    UART_HandleTypeDef* handle = handle_of(port);
    if (ctx == nullptr || handle == nullptr || ctx->rx_buffer == nullptr || ctx->rx_buffer_len == 0)
    {
        return types::status::invalid_arg;
    }

    if (HAL_UARTEx_ReceiveToIdle_DMA(handle, ctx->rx_buffer,
                                     static_cast<uint16_t>(ctx->rx_buffer_len)) != HAL_OK)
    {
        return types::status::error;
    }

    disable_rx_half_transfer_irq(handle);
    invalidate_dcache_region(ctx->rx_buffer, ctx->rx_buffer_len);

    return types::status::ok;
}

void receive(UART_HandleTypeDef* handle, uint16_t size)
{
    const HAL_UART_RxEventTypeTypeDef event = HAL_UARTEx_GetRxEventType(handle);
    if (event == HAL_UART_RXEVENT_HT)
    {
        return;
    }

    const port port = index_of(handle);
    port_state* ctx = state_of(port);
    if (ctx == nullptr || ctx->rx_buffer == nullptr)
    {
        return;
    }

    if (rx_dma_is_circular(handle) && event == HAL_UART_RXEVENT_TC)
    {
        return;
    }

    invalidate_dcache_region(ctx->rx_buffer, size);
    notify_rx(port, size);
    if (!rx_dma_is_circular(handle))
    {
        restart_rx(port);
    }
}

void handle_error(UART_HandleTypeDef* handle)
{
    __HAL_UART_CLEAR_OREFLAG(handle);
    restart_rx(index_of(handle));
}

void handle_tx_complete(UART_HandleTypeDef* handle)
{
    const port current_port = index_of(handle);
    port_state* ctx = state_of(current_port);
    if (ctx == nullptr)
    {
        return;
    }

    ctx->tx_busy = false;
}

types::status init(port port, mode mode)
{
    if (port >= port_count)
    {
        return types::status::invalid_arg;
    }

    if (!port_enabled(port))
    {
        return types::status::not_configured;
    }

    UART_HandleTypeDef* handle = handle_of(port);
    port_state* ctx = state_of(port);
    if (handle == nullptr || ctx == nullptr)
    {
        return types::status::invalid_arg;
    }

    if (ctx->initialized)
    {
        return types::status::ok;
    }

    if (mode == mode::dma)
    {
        setup_dma(handle);
    }

    __HAL_UART_SEND_REQ(handle, UART_RXDATA_FLUSH_REQUEST);
    ctx->port_mode = mode;
    ctx->initialized = true;
    return types::status::ok;
}

types::status transmit(port port, const uint8_t* data, size_t len, uint32_t timeout_ms)
{
    if (data == nullptr || len == 0)
    {
        return types::status::invalid_arg;
    }

    if (port >= port_count)
    {
        return types::status::invalid_arg;
    }

    if (!port_enabled(port))
    {
        return types::status::not_configured;
    }

    UART_HandleTypeDef* handle = handle_of(port);
    port_state* ctx = state_of(port);
    if (handle == nullptr || ctx == nullptr || !ctx->initialized)
    {
        return types::status::error;
    }

    if (ctx->port_mode == mode::dma)
    {
        if (ctx->tx_busy)
        {
            return types::status::busy;
        }

        const uint8_t* dma_buffer = data;
        if (!is_dma_accessible_buffer(data, len))
        {
            uint8_t* tx_dma_stage = tx_dma_stage_of(port);
            if (tx_dma_stage == nullptr || len > dma_tx_stage_size)
            {
                return types::status::invalid_arg;
            }

            std::memcpy(tx_dma_stage, data, len);
            dma_buffer = tx_dma_stage;
        }

        ctx->tx_busy = true;
        clean_dcache_region(dma_buffer, len);
        if (HAL_UART_Transmit_DMA(handle, const_cast<uint8_t*>(dma_buffer), static_cast<uint16_t>(len)) !=
            HAL_OK)
        {
            ctx->tx_busy = false;
            return types::status::error;
        }
        return types::status::ok;
    }

    if (HAL_UART_Transmit(handle, const_cast<uint8_t*>(data), static_cast<uint16_t>(len),
                          timeout_ms) != HAL_OK)
    {
        return types::status::error;
    }

    return types::status::ok;
}

types::status start_rx_to_idle(port port, uint8_t* buffer, size_t len, rx_handler handler,
                               void* user_data, TX_SEMAPHORE* notify_sem)
{
    if (buffer == nullptr || len == 0)
    {
        return types::status::invalid_arg;
    }

    if (port >= port_count)
    {
        return types::status::invalid_arg;
    }

    if (!port_enabled(port))
    {
        return types::status::not_configured;
    }

    UART_HandleTypeDef* handle = handle_of(port);
    port_state* ctx = state_of(port);
    if (handle == nullptr || ctx == nullptr || !ctx->initialized)
    {
        return types::status::error;
    }

    if (ctx->rx_buffer != nullptr &&
        !same_rx_owner(*ctx, buffer, len, handler, user_data, notify_sem))
    {
        return types::status::busy;
    }

    if (ctx->port_mode == mode::dma && !is_dma_accessible_buffer(buffer, len))
    {
        return types::status::invalid_arg;
    }

    ctx->rx_buffer = buffer;
    ctx->rx_buffer_len = len;
    ctx->handler = handler;
    ctx->user_data = user_data;
    ctx->notify_sem = notify_sem;

    if (HAL_UARTEx_ReceiveToIdle_DMA(handle, buffer, static_cast<uint16_t>(len)) != HAL_OK)
    {
        return types::status::error;
    }

    disable_rx_half_transfer_irq(handle);
    invalidate_dcache_region(buffer, len);

    return types::status::ok;
}

} // namespace bsp::usart

extern "C" void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef* huart, uint16_t size)
{
    bsp::usart::receive(huart, size);
}

extern "C" void HAL_UART_ErrorCallback(UART_HandleTypeDef* huart)
{
    bsp::usart::handle_error(huart);
}

extern "C" void HAL_UART_TxCpltCallback(UART_HandleTypeDef* huart)
{
    bsp::usart::handle_tx_complete(huart);
}
