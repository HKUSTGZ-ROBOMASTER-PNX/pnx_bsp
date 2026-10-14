#pragma once
#include "config.hpp"
#include "status.hpp"
#include "peripherals.hpp"

namespace bsp::gpio
{
using pnxtypes::status;

inline status read(const config& pin, bool& high) noexcept
{
    auto* port = peripherals::gpio_port(pin.instance);
    const auto mask = peripherals::gpio_pin(pin.pin);
    if (port == nullptr || mask == 0)
    {
        return status::not_configured;
    }
    high = HAL_GPIO_ReadPin(port, mask) == GPIO_PIN_SET;
    return status::ok;
}

inline status is_active(const config& pin, bool& active) noexcept
{
    bool high;
    const auto result = read(pin, high);
    if (result == status::ok)
    {
        active = high == (pin.active == active_level::high);
    }
    return result;
}

inline status write(const config& pin, bool high) noexcept
{
    auto* port = peripherals::gpio_port(pin.instance);
    const auto mask = peripherals::gpio_pin(pin.pin);
    if (port == nullptr || mask == 0)
    {
        return status::not_configured;
    }
    HAL_GPIO_WritePin(port, mask, high ? GPIO_PIN_SET : GPIO_PIN_RESET);
    return status::ok;
}

inline status set_active(const config& pin, bool active) noexcept
{
    return write(pin, active == (pin.active == active_level::high));
}

inline status toggle(const config& pin) noexcept
{
    auto* port = peripherals::gpio_port(pin.instance);
    const auto mask = peripherals::gpio_pin(pin.pin);
    if (port == nullptr || mask == 0)
    {
        return status::not_configured;
    }
    HAL_GPIO_TogglePin(port, mask);
    return status::ok;
}

} // namespace bsp::gpio
