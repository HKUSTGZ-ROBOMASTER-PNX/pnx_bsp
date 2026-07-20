#include "bsp_can.hpp"

#include <array>

namespace bsp::can
{
namespace
{

struct rx_slot
{
    rx_handler handler = nullptr;
    void* user_data = nullptr;
};

struct telemetry_state
{
    volatile std::uint32_t rx_count = 0;
    volatile std::uint32_t tx_count = 0;
    volatile std::uint32_t last_id = 0;
    volatile std::uint32_t last_tick = 0;
    volatile std::uint32_t error_count = 0;
    volatile std::uint32_t bus_off_count = 0;
    volatile std::uint32_t drop_count = 0;
    volatile state bus_state = state::stopped;
};

std::array<bus_type, bus_count> active_types{};
std::array<bool, bus_count> initialized{};
std::array<std::array<rx_slot, max_rx_callbacks>, bus_count> rx_slots{};
std::array<telemetry_state, bus_count> telemetry_states{};

std::size_t index_of(bus selected) noexcept
{
    const auto index = static_cast<std::size_t>(selected);
    return index < bus_count ? index : bus_count;
}

const bus_config* config_of(std::size_t index) noexcept
{
    return index < bus_count ? &configs[index] : nullptr;
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
        ++telemetry_states[index].error_count;
        telemetry_states[index].bus_state = state::fault;
        return status;
    }

    active_types[index] = type;
    initialized[index] = true;
    telemetry_states[index].bus_state = state::active;
    return types::status::ok;
}

types::status transmit(bus selected, std::uint32_t id, const std::uint8_t* data,
                       std::uint16_t len)
{
    const std::size_t index = index_of(selected);
    if (index >= bus_count || data == nullptr || len == 0U)
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

    const types::status status =
        detail::backend_transmit(selected, id, data, len);
    if (status == types::status::ok)
    {
        ++telemetry_states[index].tx_count;
    }
    else
    {
        ++telemetry_states[index].error_count;
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
        if (slot.handler == nullptr)
        {
            slot.user_data = user_data;
            slot.handler = handler;
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
        slot.handler = nullptr;
        slot.user_data = nullptr;
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
    result.rx_count = source.rx_count;
    result.tx_count = source.tx_count;
    result.last_id = source.last_id;
    result.last_tick = source.last_tick;
    result.error_count = source.error_count;
    result.bus_off_count = source.bus_off_count;
    result.drop_count = source.drop_count;
    result.bus_state = source.bus_state;
    return result;
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
    ++stats.rx_count;
    stats.last_id = frame.id;
    stats.last_tick = tick;
    stats.bus_state = state::active;

    rx_frame stamped = frame;
    stamped.tick = tick;
    for (const auto& slot : rx_slots[index])
    {
        if (slot.handler != nullptr)
        {
            slot.handler(selected, stamped, slot.user_data);
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
    ++stats.error_count;
    stats.last_tick = tick;
    stats.bus_state = next_state;
    if (next_state == state::bus_off)
    {
        ++stats.bus_off_count;
    }
}

void drop_from_isr(bus selected) noexcept
{
    const std::size_t index = index_of(selected);
    if (index < bus_count)
    {
        ++telemetry_states[index].drop_count;
    }
}

} // namespace detail
} // namespace bsp::can
