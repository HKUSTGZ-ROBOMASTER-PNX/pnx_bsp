#include "bsp_can.hpp"

#include <array>
#include <atomic>

namespace bsp::can
{
namespace
{

struct rx_slot
{
    std::atomic<rx_handler> handler{nullptr};
    std::atomic<void*> user_data{nullptr};
};

struct telemetry_state
{
    std::atomic<std::uint32_t> rx_count{0U};
    std::atomic<std::uint32_t> tx_count{0U};
    std::atomic<std::uint32_t> last_id{0U};
    std::atomic<std::uint32_t> last_tick{0U};
    std::atomic<std::uint32_t> error_count{0U};
    std::atomic<std::uint32_t> bus_off_count{0U};
    std::atomic<std::uint32_t> drop_count{0U};
    std::atomic<std::uint32_t> bus_state{
        static_cast<std::uint32_t>(state::stopped)};
};

static_assert(std::atomic<std::uint32_t>::is_always_lock_free,
              "CAN ISR telemetry requires lock-free 32-bit atomics");
static_assert(std::atomic<rx_handler>::is_always_lock_free,
              "CAN ISR callback dispatch requires lock-free pointer atomics");
static_assert(std::atomic<void*>::is_always_lock_free,
              "CAN ISR callback context requires lock-free pointer atomics");

std::array<bus_type, bus_count> active_types{};
std::array<bool, bus_count> initialized{};
std::array<std::array<rx_slot, max_rx_callbacks>, bus_count> rx_slots{};
std::array<telemetry_state, bus_count> telemetry_states{};
std::array<std::atomic<bool>, bus_count> recovery_in_progress{};
std::atomic<std::uint32_t> fault_epoch{0U};

std::size_t index_of(bus selected) noexcept
{
    const auto index = static_cast<std::size_t>(selected);
    return index < bus_count ? index : bus_count;
}

const bus_config* config_of(std::size_t index) noexcept
{
    return index < bus_count ? &configs[index] : nullptr;
}

bool valid_transmit(std::size_t index, const std::uint8_t* data,
                    std::uint16_t len) noexcept
{
    return index < bus_count && data != nullptr && len != 0U;
}

} // namespace

bool bus_enabled(std::size_t index) noexcept
{
    const bus_config* cfg = config_of(index);
    return cfg != nullptr && cfg->enabled;
}

bus_type configured_bus_type(std::size_t index) noexcept
{
    const bus_config* cfg = config_of(index);
    return cfg != nullptr ? cfg->type : bus_type::classic;
}

id_type filter_id_type_of(std::size_t index) noexcept
{
    const bus_config* cfg = config_of(index);
    return cfg != nullptr ? cfg->filter_id_type : id_type::standard;
}

types::status init(bus selected, bus_type type)
{
    const std::size_t index = index_of(selected);
    if (index >= bus_count)
    {
        return types::status::invalid_arg;
    }
    if (!bus_enabled(index))
    {
        return types::status::not_configured;
    }
    if (initialized[index])
    {
        return active_types[index] == type ? types::status::ok
                                           : types::status::invalid_arg;
    }
    if (configured_bus_type(index) == bus_type::classic && type == bus_type::fd)
    {
        return types::status::not_configured;
    }

    const types::status status = detail::backend_init(selected, type);
    if (status != types::status::ok)
    {
        telemetry_states[index].bus_state.store(
            static_cast<std::uint32_t>(state::fault),
            std::memory_order_relaxed);
        telemetry_states[index].error_count.fetch_add(
            1U, std::memory_order_release);
        fault_epoch.fetch_add(1U, std::memory_order_release);
        return status;
    }

    active_types[index] = type;
    initialized[index] = true;
    telemetry_states[index].bus_state.store(
        static_cast<std::uint32_t>(state::active),
        std::memory_order_release);
    return types::status::ok;
}

types::status recover(bus selected)
{
    const std::size_t index = index_of(selected);
    if (index >= bus_count)
    {
        return types::status::invalid_arg;
    }
    if (!bus_enabled(index))
    {
        return types::status::not_configured;
    }
    if (!initialized[index])
    {
        return types::status::error;
    }

    telemetry_state& stats = telemetry_states[index];
    if (static_cast<state>(
            stats.bus_state.load(std::memory_order_acquire)) ==
        state::active)
    {
        return types::status::ok;
    }

    bool expected = false;
    if (!recovery_in_progress[index].compare_exchange_strong(
            expected, true, std::memory_order_acq_rel))
    {
        return types::status::busy;
    }

    const std::uint32_t irq_state = detail::backend_enter_critical();
    const state current = static_cast<state>(
        stats.bus_state.load(std::memory_order_acquire));
    if (current != state::bus_off)
    {
        detail::backend_exit_critical(irq_state);
        recovery_in_progress[index].store(
            false, std::memory_order_release);
        return current == state::active ? types::status::ok
                                        : types::status::error;
    }
    stats.bus_state.store(
        static_cast<std::uint32_t>(state::recovering),
        std::memory_order_release);
    detail::backend_exit_critical(irq_state);

    const types::status status =
        detail::backend_recover(selected);
    const std::uint32_t completion_irq_state =
        detail::backend_enter_critical();
    const bool owns_transition =
        static_cast<state>(
            stats.bus_state.load(std::memory_order_acquire)) ==
        state::recovering;
    if (status == types::status::ok && owns_transition)
    {
        stats.bus_state.store(
            static_cast<std::uint32_t>(state::active),
            std::memory_order_release);
    }
    else if (owns_transition)
    {
        stats.bus_state.store(
            static_cast<std::uint32_t>(state::fault),
            std::memory_order_release);
        stats.error_count.fetch_add(
            1U, std::memory_order_release);
        fault_epoch.fetch_add(1U, std::memory_order_release);
    }
    detail::backend_exit_critical(completion_irq_state);
    recovery_in_progress[index].store(
        false, std::memory_order_release);
    return status == types::status::ok && owns_transition
               ? types::status::ok
               : types::status::error;
}

types::status transmit(bus selected, std::uint32_t id, const std::uint8_t* data,
                       std::uint16_t len)
{
    const std::size_t index = index_of(selected);
    if (!valid_transmit(index, data, len))
    {
        return types::status::invalid_arg;
    }
    if (!bus_enabled(index))
    {
        return types::status::not_configured;
    }
    if (!initialized[index])
    {
        return types::status::error;
    }
    if (static_cast<state>(
            telemetry_states[index].bus_state.load(
                std::memory_order_acquire)) == state::bus_off)
    {
        const types::status recovery_status = recover(selected);
        if (recovery_status != types::status::ok)
        {
            return recovery_status;
        }
    }

    const types::status status =
        detail::backend_transmit(selected, id, data, len);
    if (status == types::status::ok)
    {
        telemetry_states[index].tx_count.fetch_add(
            1U, std::memory_order_relaxed);
    }
    else
    {
        telemetry_states[index].error_count.fetch_add(
            1U, std::memory_order_release);
        fault_epoch.fetch_add(1U, std::memory_order_release);
    }
    return status;
}

types::status transmit_if_healthy(
    bus selected, std::uint32_t id, const std::uint8_t* data,
    std::uint16_t len, std::uint32_t expected_error_count,
    std::uint32_t expected_drop_count,
    std::uint32_t expected_fault_epoch)
{
    const std::size_t index = index_of(selected);
    if (!valid_transmit(index, data, len))
    {
        return types::status::invalid_arg;
    }
    if (!bus_enabled(index))
    {
        return types::status::not_configured;
    }
    if (!initialized[index])
    {
        return types::status::error;
    }

    telemetry_state& stats = telemetry_states[index];
    const std::uint32_t irq_state = detail::backend_enter_critical();
    const bool healthy =
        stats.error_count.load(std::memory_order_acquire) ==
            expected_error_count &&
        stats.drop_count.load(std::memory_order_acquire) ==
            expected_drop_count &&
        fault_epoch.load(std::memory_order_acquire) ==
            expected_fault_epoch &&
        static_cast<state>(
            stats.bus_state.load(std::memory_order_acquire)) ==
            state::active;
    const types::status status =
        healthy ? detail::backend_transmit(selected, id, data, len)
                : types::status::error;
    detail::backend_exit_critical(irq_state);

    if (status == types::status::ok)
    {
        stats.tx_count.fetch_add(1U, std::memory_order_relaxed);
    }
    else if (healthy)
    {
        stats.error_count.fetch_add(1U, std::memory_order_release);
        fault_epoch.fetch_add(1U, std::memory_order_release);
    }
    return status;
}

types::status register_rx_handler(bus selected, rx_handler handler,
                                  void* user_data)
{
    const std::size_t index = index_of(selected);
    if (index >= bus_count || handler == nullptr)
    {
        return types::status::invalid_arg;
    }
    if (!bus_enabled(index))
    {
        return types::status::not_configured;
    }

    for (auto& slot : rx_slots[index])
    {
        if (slot.handler.load(std::memory_order_acquire) == nullptr)
        {
            slot.user_data.store(user_data, std::memory_order_relaxed);
            slot.handler.store(handler, std::memory_order_release);
            return types::status::ok;
        }
    }
    return types::status::busy;
}

void unregister_rx_handlers(bus selected)
{
    const std::size_t index = index_of(selected);
    if (index >= bus_count)
    {
        return;
    }
    for (auto& slot : rx_slots[index])
    {
        slot.handler.store(nullptr, std::memory_order_release);
    }
}

telemetry snapshot(bus selected) noexcept
{
    telemetry result{};
    const std::size_t index = index_of(selected);
    if (index >= bus_count)
    {
        result.bus_state = state::fault;
        return result;
    }

    const telemetry_state& source = telemetry_states[index];
    result.rx_count =
        source.rx_count.load(std::memory_order_relaxed);
    result.tx_count =
        source.tx_count.load(std::memory_order_relaxed);
    result.last_id =
        source.last_id.load(std::memory_order_relaxed);
    result.last_tick =
        source.last_tick.load(std::memory_order_relaxed);
    result.error_count =
        source.error_count.load(std::memory_order_acquire);
    result.bus_off_count =
        source.bus_off_count.load(std::memory_order_relaxed);
    result.drop_count =
        source.drop_count.load(std::memory_order_acquire);
    result.fault_epoch =
        fault_epoch.load(std::memory_order_acquire);
    result.bus_state = static_cast<state>(
        source.bus_state.load(std::memory_order_acquire));
    return result;
}

std::uint32_t time_now() noexcept
{
    return detail::backend_tick_now();
}

namespace detail
{

void rx_from_isr(bus selected, const rx_frame& frame, std::uint32_t tick) noexcept
{
    const std::size_t index = index_of(selected);
    if (index >= bus_count)
    {
        return;
    }

    telemetry_state& stats = telemetry_states[index];
    stats.rx_count.fetch_add(1U, std::memory_order_relaxed);
    stats.last_id.store(frame.id, std::memory_order_relaxed);
    stats.last_tick.store(tick, std::memory_order_relaxed);

    rx_frame stamped = frame;
    stamped.tick = tick;
    for (const auto& slot : rx_slots[index])
    {
        const rx_handler handler =
            slot.handler.load(std::memory_order_acquire);
        if (handler != nullptr)
        {
            handler(
                selected, stamped,
                slot.user_data.load(std::memory_order_relaxed));
        }
    }
}

void error_from_isr(bus selected, state next_state, std::uint32_t tick) noexcept
{
    const std::size_t index = index_of(selected);
    if (index >= bus_count)
    {
        return;
    }
    telemetry_state& stats = telemetry_states[index];
    stats.last_tick.store(tick, std::memory_order_relaxed);
    stats.bus_state.store(
        static_cast<std::uint32_t>(next_state),
        std::memory_order_relaxed);
    if (next_state == state::bus_off)
    {
        stats.bus_off_count.fetch_add(
            1U, std::memory_order_relaxed);
    }
    stats.error_count.fetch_add(1U, std::memory_order_release);
    fault_epoch.fetch_add(1U, std::memory_order_release);
}

void drop_from_isr(bus selected) noexcept
{
    const std::size_t index = index_of(selected);
    if (index < bus_count)
    {
        telemetry_states[index].drop_count.fetch_add(
            1U, std::memory_order_release);
        fault_epoch.fetch_add(1U, std::memory_order_release);
    }
}

} // namespace detail
} // namespace bsp::can
