#include "bsp_usb.hpp"

#include "bsp_usb_backend.hpp"

#include <algorithm>
#include <array>
#include <cstring>

namespace bsp::usb
{
namespace
{

struct tx_slot
{
    std::array<uint8_t, detail::common_max_transfer_size> data{};
    uint16_t length = 0U;
};

config active_config{};
runtime_state active_state{};
runtime_state compatibility_snapshot{};
std::array<tx_slot, detail::common_tx_queue_depth> tx_queue{};
uint8_t tx_head = 0U;
uint8_t tx_tail = 0U;
uint8_t tx_count = 0U;
bool initialized = false;

class backend_guard
{
  public:
    backend_guard() noexcept : locked_(detail::backend_lock()) {}
    backend_guard(const backend_guard&) = delete;
    backend_guard& operator=(const backend_guard&) = delete;

    ~backend_guard()
    {
        if (locked_)
        {
            detail::backend_unlock();
        }
    }

    explicit operator bool() const noexcept { return locked_; }

  private:
    bool locked_;
};

uint16_t transfer_limit() noexcept
{
    const capabilities backend = detail::backend_capabilities();
    return std::min<uint16_t>(
        active_config.max_tx_size,
        std::min<uint16_t>(backend.max_transfer_size,
                           detail::common_max_transfer_size));
}

void update_queue_state() noexcept
{
    active_state.tx_queue_size = tx_count;
    active_state.pending_write_len =
        tx_count == 0U ? 0U : tx_queue[tx_tail].length;
    active_state.tx_queue_high_water =
        std::max(active_state.tx_queue_high_water, tx_count);
}

} // namespace

types::status init(const config& cfg) noexcept
{
    if (initialized)
    {
        return types::status::ok;
    }
    if (cfg.period_ticks == 0U || cfg.max_tx_size == 0U)
    {
        return types::status::invalid_arg;
    }

    active_config = cfg;
    active_state = {};
    tx_head = 0U;
    tx_tail = 0U;
    tx_count = 0U;

    const capabilities backend = detail::backend_capabilities();
    if (!backend.device_mode || backend.max_transfer_size == 0U ||
        transfer_limit() == 0U)
    {
        active_state.link = link_state::fault;
        ++active_state.error_count;
        return types::status::not_configured;
    }

    detail::backend_config backend_cfg{};
    backend_cfg.read_priority = active_config.read_priority;
    backend_cfg.write_priority = active_config.write_priority;
    backend_cfg.period_ticks = active_config.period_ticks;
    backend_cfg.min_rx_size = active_config.min_rx_size;
    backend_cfg.max_tx_size = transfer_limit();

    // Publish the immutable config before the backend can start worker
    // threads. A failed backend_init() is required to leave no worker running.
    initialized = true;
    active_state.initialized = true;
    active_state.link = link_state::initialized;
    const types::status result = detail::backend_init(backend_cfg);
    if (result != types::status::ok)
    {
        initialized = false;
        active_state.initialized = false;
        active_state.link = link_state::fault;
        ++active_state.error_count;
        return result;
    }

    if (detail::backend_connected())
    {
        detail::connection_from_backend(true);
    }
    return types::status::ok;
}

write_result write(const uint8_t* data, std::size_t len) noexcept
{
    if (data == nullptr || len == 0U ||
        len > detail::common_max_transfer_size)
    {
        return {types::status::invalid_arg, 0U};
    }
    if (!initialized)
    {
        return {types::status::not_configured, 0U};
    }

    backend_guard guard{};
    if (!guard)
    {
        return {types::status::busy, 0U};
    }
    if (!active_state.connected)
    {
        return {types::status::not_configured, 0U};
    }
    if (len > transfer_limit())
    {
        return {types::status::invalid_arg, 0U};
    }
    if (tx_count >= detail::common_tx_queue_depth)
    {
        ++active_state.tx_drop_count;
        return {types::status::busy, 0U};
    }

    tx_slot& slot = tx_queue[tx_head];
    std::memcpy(slot.data.data(), data, len);
    slot.length = static_cast<uint16_t>(len);
    tx_head = static_cast<uint8_t>(
        (tx_head + 1U) % detail::common_tx_queue_depth);
    ++tx_count;
    update_queue_state();

    detail::backend_signal_tx();
    return {types::status::ok, static_cast<uint16_t>(len)};
}

bool connected() noexcept
{
    return snapshot().connected;
}

runtime_state snapshot() noexcept
{
    if (!initialized)
    {
        return active_state;
    }
    backend_guard guard{};
    return guard ? active_state : runtime_state{};
}

capabilities get_capabilities() noexcept
{
    capabilities result = detail::backend_capabilities();
    result.max_transfer_size =
        std::min<uint16_t>(result.max_transfer_size,
                           detail::common_max_transfer_size);
    result.tx_queue_depth = detail::common_tx_queue_depth;
    return result;
}

const runtime_state& state() noexcept
{
    compatibility_snapshot = snapshot();
    return compatibility_snapshot;
}

namespace detail
{

void connection_from_backend(bool is_connected) noexcept
{
    if (!initialized && !active_state.initialized)
    {
        return;
    }
    backend_guard guard{};
    if (!guard)
    {
        return;
    }

    if (is_connected)
    {
        if (!active_state.connected)
        {
            ++active_state.connect_count;
        }
        active_state.connected = true;
        active_state.link = link_state::connected;
        return;
    }

    if (active_state.connected)
    {
        ++active_state.disconnect_count;
    }
    active_state.connected = false;
    active_state.link = active_state.initialized
                            ? link_state::initialized
                            : link_state::uninitialized;
    active_state.write_busy = false;
    active_state.read_busy = false;
    active_state.tx_drop_count += tx_count;
    tx_head = 0U;
    tx_tail = 0U;
    tx_count = 0U;
    update_queue_state();
}

void receive_started_from_backend() noexcept
{
    backend_guard guard{};
    if (guard && active_state.connected)
    {
        active_state.read_busy = true;
    }
}

void transmit_woken_from_backend() noexcept
{
    backend_guard guard{};
    if (guard)
    {
        ++active_state.tx_wake_count;
    }
}

void receive_from_backend(const uint8_t* data, uint16_t length,
                          types::status status,
                          uint32_t native_status) noexcept
{
    void (*callback)(const uint8_t*, uint16_t, void*) = nullptr;
    void* user = nullptr;
    {
        backend_guard guard{};
        if (!guard)
        {
            return;
        }
        active_state.read_busy = false;
        active_state.last_read_status = native_status;
        active_state.last_read_len = length;
        if (status != types::status::ok)
        {
            ++active_state.error_count;
            return;
        }
        if (!active_state.connected || data == nullptr ||
            length < active_config.min_rx_size)
        {
            return;
        }
        ++active_state.read_count;
        active_state.rx_bytes += length;
        callback = active_config.on_rx;
        user = active_config.user;
    }
    if (callback != nullptr)
    {
        callback(data, length, user);
    }
}

bool prepare_tx_for_backend(uint8_t* output, uint16_t capacity,
                            uint16_t& length) noexcept
{
    length = 0U;
    if (output == nullptr || capacity == 0U)
    {
        return false;
    }

    uint16_t (*fill)(uint8_t*, uint16_t, void*) = nullptr;
    void* user = nullptr;
    uint16_t fill_capacity = 0U;
    {
        backend_guard guard{};
        if (!guard || !active_state.connected)
        {
            return false;
        }
        if (tx_count > 0U)
        {
            tx_slot& slot = tx_queue[tx_tail];
            if (slot.length > capacity)
            {
                ++active_state.error_count;
                ++active_state.tx_drop_count;
                tx_tail = static_cast<uint8_t>(
                    (tx_tail + 1U) % common_tx_queue_depth);
                --tx_count;
                update_queue_state();
                return false;
            }
            std::memcpy(output, slot.data.data(), slot.length);
            length = slot.length;
            tx_tail = static_cast<uint8_t>(
                (tx_tail + 1U) % common_tx_queue_depth);
            --tx_count;
            update_queue_state();
            active_state.write_busy = true;
            active_state.last_write_requested = length;
            return true;
        }
        fill = active_config.fill_tx;
        user = active_config.user;
        fill_capacity =
            std::min<uint16_t>(capacity, transfer_limit());
        if (fill != nullptr)
        {
            ++active_state.fill_count;
        }
    }

    if (fill == nullptr || fill_capacity == 0U)
    {
        return false;
    }
    const uint16_t filled = fill(output, fill_capacity, user);
    if (filled == 0U || filled > fill_capacity)
    {
        return false;
    }

    backend_guard guard{};
    if (!guard || !active_state.connected)
    {
        return false;
    }
    length = filled;
    active_state.write_busy = true;
    active_state.last_write_requested = filled;
    active_state.pending_write_len = filled;
    return true;
}

void transmit_complete_from_backend(uint16_t requested,
                                    uint16_t actual,
                                    types::status status,
                                    uint32_t native_status) noexcept
{
    void (*callback)(uint16_t, uint16_t, uint32_t, void*) = nullptr;
    void* user = nullptr;
    {
        backend_guard guard{};
        if (!guard)
        {
            return;
        }
        active_state.write_busy = false;
        active_state.pending_write_len =
            tx_count == 0U ? 0U : tx_queue[tx_tail].length;
        active_state.last_write_requested = requested;
        active_state.last_write_actual = actual;
        active_state.last_write_status = native_status;
        if (status == types::status::ok && actual == requested)
        {
            ++active_state.write_count;
            active_state.tx_bytes += actual;
        }
        else
        {
            ++active_state.error_count;
        }
        callback = active_config.on_tx_done;
        user = active_config.user;
    }
    if (callback != nullptr)
    {
        callback(requested, actual, native_status, user);
    }
}

} // namespace detail
} // namespace bsp::usb
