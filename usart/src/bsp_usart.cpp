#include "bsp_usart.hpp"

#include <array>
#include <atomic>
#include <limits>

namespace bsp::usart
{
namespace
{

static_assert(std::atomic<std::uint32_t>::is_always_lock_free,
              "UART ISR telemetry requires lock-free atomics");

struct port_state
{
    bool initialized = false;
    mode active_mode = mode::block;
    rx_handler handler = nullptr;
    void* user_data = nullptr;
    notify_handler notify = nullptr;
    void* notify_user_data = nullptr;
    rx_delivery delivery = rx_delivery::frame_snapshot;
    std::uint8_t* rx_buffer = nullptr;
    std::size_t rx_buffer_len = 0;
    std::atomic<std::uint32_t> rx_count{0U};
    std::atomic<std::uint32_t> tx_count{0U};
    std::atomic<std::uint32_t> error_count{0U};
    std::atomic<std::uint32_t> busy_count{0U};
    std::atomic<std::uint32_t> last_rx_len{0U};
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
        ctx->error_count.fetch_add(1U, std::memory_order_relaxed);
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

    const types::status status = detail::backend_transmit(
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
    port_state* ctx = state_of(selected);
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

    const types::status status = detail::backend_start_rx(
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
    port_state* ctx = state_of(selected);
    if (ctx == nullptr || !ctx->initialized || ctx->rx_buffer == nullptr)
    {
        return types::status::invalid_arg;
    }
    const types::status status = detail::backend_restart_rx(selected);
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
    const port_state* ctx = state_of(selected);
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

namespace detail
{

void rx_from_isr(port selected, std::size_t len) noexcept
{
    rx_from_isr(selected, 0U, len);
}

void rx_from_isr(port selected, std::size_t offset,
                 std::size_t len) noexcept
{
    port_state* ctx = state_of(selected);
    if (ctx == nullptr || ctx->rx_buffer == nullptr ||
        offset > ctx->rx_buffer_len ||
        len > ctx->rx_buffer_len - offset)
    {
        return;
    }
    ctx->rx_count.fetch_add(1U, std::memory_order_relaxed);
    ctx->last_rx_len.store(static_cast<std::uint32_t>(len),
                           std::memory_order_relaxed);
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

void error_from_isr(port selected) noexcept
{
    port_state* ctx = state_of(selected);
    if (ctx != nullptr)
    {
        ctx->error_count.fetch_add(1U, std::memory_order_relaxed);
    }
}

void tx_complete_from_isr(port selected) noexcept
{
    port_state* ctx = state_of(selected);
    if (ctx != nullptr)
    {
        ctx->tx_count.fetch_add(1U, std::memory_order_relaxed);
    }
}

} // namespace detail
} // namespace bsp::usart
