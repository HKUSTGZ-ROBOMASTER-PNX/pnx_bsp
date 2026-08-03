#include "bsp_indicator.hpp"

#include "main.h"

namespace bsp::indicator
{
namespace
{

state current{};

std::uint16_t pin_of(channel selected) noexcept
{
    switch (selected)
    {
    case channel::red:
        return GPIO_PIN_12;
    case channel::green:
        return GPIO_PIN_11;
    case channel::blue:
        return GPIO_PIN_10;
    default:
        return 0U;
    }
}

bool* value_of(channel selected) noexcept
{
    switch (selected)
    {
    case channel::red:
        return &current.red;
    case channel::green:
        return &current.green;
    case channel::blue:
        return &current.blue;
    default:
        return nullptr;
    }
}

} // namespace

types::status init() noexcept
{
    HAL_GPIO_WritePin(GPIOH, GPIO_PIN_12 | GPIO_PIN_11 | GPIO_PIN_10,
                      GPIO_PIN_RESET);
    current = {};
    current.initialized = true;
    return types::status::ok;
}

types::status set(channel selected, bool on) noexcept
{
    bool* value = value_of(selected);
    const std::uint16_t pin = pin_of(selected);
    if (!current.initialized || value == nullptr || pin == 0U)
    {
        return types::status::invalid_arg;
    }

    HAL_GPIO_WritePin(GPIOH, pin, on ? GPIO_PIN_SET : GPIO_PIN_RESET);
    *value = on;
    return types::status::ok;
}

types::status toggle(channel selected) noexcept
{
    bool* value = value_of(selected);
    return value != nullptr ? set(selected, !*value)
                            : types::status::invalid_arg;
}

state snapshot() noexcept
{
    return current;
}

} // namespace bsp::indicator
