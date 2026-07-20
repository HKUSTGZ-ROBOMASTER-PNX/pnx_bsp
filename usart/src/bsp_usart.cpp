#include "bsp_usart.hpp"

#include <array>

namespace bsp::usart
{
namespace
{

struct port_state
{
    bool initialized = false;
    mode active_mode = mode::block;
    rx_handler handler = nullptr;
    void* user_data = nullptr;
    notify_handler notify = nullptr;
    void* notify_user_data = nullptr;
    std::uint8_t* rx_buffer = nullptr;
    std::size_t rx_buffer_len = 0;
    volatile std::uint32_t rx_count = 0;
    volatile std::uint32_t tx_count = 0;
    volatile std::uint32_t error_count = 0;
    volatile std::uint32_t busy_count = 0;
    volatile std::uint32_t last_rx_len = 0;
};

std::array<port_state, port_count> states{};

port_state* state_of(port selected) noexcept
{
    return selected < port_count ? &states[selected] : nullptr;
}

const port_config* config_of(port selected) noexcept
{
    return selected < port_count ? &configs[selected] : nullptr;
}

} // namespace

bool port_enabled(port selected) noexcept
{
    const port_config* cfg = config_of(selected);
    return cfg != nullptr && cfg->enabled;
}

types::status init(port selected, mode selected_mode)
{
    port_state* ctx = state_of(selected);
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

    const types::status status = detail::backend_init(selected, selected_mode);
    if (status != types::status::ok)
    {
        ++ctx->error_count;
        return status;
    }
    ctx->active_mode = selected_mode;
    ctx->initialized = true;
    return types::status::ok;
}

types::status transmit(port selected, const std::uint8_t* data, std::size_t len,
                       std::uint32_t timeout_ms)
{
    port_state* ctx = state_of(selected);
    if (ctx == nullptr || data == nullptr || len == 0U)
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

    const types::status status = detail::backend_transmit(
        selected, ctx->active_mode, data, len, timeout_ms);
    if (status == types::status::ok)
    {
        if (ctx->active_mode != mode::dma)
        {
            ++ctx->tx_count;
        }
    }
    else if (status == types::status::busy)
    {
        ++ctx->busy_count;
    }
    else
    {
        ++ctx->error_count;
    }
    return status;
}

types::status start_rx_to_idle(port selected, std::uint8_t* buffer,
                               std::size_t len, rx_handler handler,
                               void* user_data, notify_handler notify,
                               void* notify_user_data)
{
    port_state* ctx = state_of(selected);
    if (ctx == nullptr || buffer == nullptr || len == 0U || !ctx->initialized)
    {
        return types::status::invalid_arg;
    }
    if (ctx->rx_buffer != nullptr &&
        (ctx->rx_buffer != buffer || ctx->rx_buffer_len != len ||
         ctx->handler != handler || ctx->user_data != user_data ||
         ctx->notify != notify ||
         ctx->notify_user_data != notify_user_data))
    {
        return types::status::busy;
    }

    ctx->rx_buffer = buffer;
    ctx->rx_buffer_len = len;
    ctx->handler = handler;
    ctx->user_data = user_data;
    ctx->notify = notify;
    ctx->notify_user_data = notify_user_data;

    const types::status status = detail::backend_start_rx(
        selected, ctx->active_mode, buffer, len);
    if (status != types::status::ok)
    {
        ++ctx->error_count;
    }
    return status;
}

types::status restart_rx(port selected)
{
    port_state* ctx = state_of(selected);
    if (ctx == nullptr || !ctx->initialized || ctx->rx_buffer == nullptr)
    {
        return types::status::invalid_arg;
    }
    const types::status status = detail::backend_restart_rx(selected);
    if (status != types::status::ok)
    {
        ++ctx->error_count;
    }
    return status;
}

telemetry snapshot(port selected) noexcept
{
    telemetry result{};
    const port_state* ctx = state_of(selected);
    if (ctx == nullptr)
    {
        return result;
    }
    result.rx_count = ctx->rx_count;
    result.tx_count = ctx->tx_count;
    result.error_count = ctx->error_count;
    result.busy_count = ctx->busy_count;
    result.last_rx_len = ctx->last_rx_len;
    return result;
}

namespace detail
{

void rx_from_isr(port selected, std::size_t len) noexcept
{
    port_state* ctx = state_of(selected);
    if (ctx == nullptr || ctx->rx_buffer == nullptr ||
        len > ctx->rx_buffer_len)
    {
        return;
    }
    ++ctx->rx_count;
    ctx->last_rx_len = static_cast<std::uint32_t>(len);
    const rx_frame frame{ctx->rx_buffer, len};
    if (ctx->notify != nullptr)
    {
        ctx->notify(ctx->notify_user_data);
    }
    if (ctx->handler != nullptr)
    {
        ctx->handler(selected, frame, ctx->user_data);
    }
}

void error_from_isr(port selected) noexcept
{
    port_state* ctx = state_of(selected);
    if (ctx != nullptr)
    {
        ++ctx->error_count;
    }
}

void tx_complete_from_isr(port selected) noexcept
{
    port_state* ctx = state_of(selected);
    if (ctx != nullptr)
    {
        ++ctx->tx_count;
    }
}

} // namespace detail
} // namespace bsp::usart
