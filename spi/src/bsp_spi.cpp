#include "bsp_spi.hpp"

namespace bsp::spi
{

bool bus_enabled(bus selected) noexcept
{
    return detail::backend_bus_enabled(selected);
}

bool select_enabled(chip_select selected) noexcept
{
    return detail::backend_select_enabled(selected);
}

types::status init(bus selected) noexcept
{
    return bus_enabled(selected)
               ? detail::backend_init(selected)
               : types::status::not_configured;
}

types::status wait_ready(
    bus selected, std::uint32_t timeout_ms) noexcept
{
    return bus_enabled(selected)
               ? detail::backend_wait_ready(selected, timeout_ms)
               : types::status::not_configured;
}

types::status transmit(
    bus selected, const std::uint8_t* data, std::size_t len,
    std::uint32_t timeout_ms) noexcept
{
    if (data == nullptr || len == 0U)
    {
        return types::status::invalid_arg;
    }
    return bus_enabled(selected)
               ? detail::backend_transmit(
                     selected, data, len, timeout_ms)
               : types::status::not_configured;
}

types::status receive(
    bus selected, std::uint8_t* data, std::size_t len,
    std::uint32_t timeout_ms) noexcept
{
    if (data == nullptr || len == 0U)
    {
        return types::status::invalid_arg;
    }
    return bus_enabled(selected)
               ? detail::backend_receive(
                     selected, data, len, timeout_ms)
               : types::status::not_configured;
}

types::status set_select(
    chip_select selected, bool active) noexcept
{
    return select_enabled(selected)
               ? detail::backend_set_select(selected, active)
               : types::status::not_configured;
}

} // namespace bsp::spi
