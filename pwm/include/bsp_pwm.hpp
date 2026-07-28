#pragma once

#include "usertypes.hpp"

#include <cstdint>

namespace bsp::pwm
{

struct channel
{
    std::uint8_t value = 0xFFU;

    constexpr explicit channel(std::uint8_t selected = 0xFFU) noexcept
        : value(selected)
    {
    }
};

constexpr bool operator==(channel lhs, channel rhs) noexcept
{
    return lhs.value == rhs.value;
}

constexpr bool operator!=(channel lhs, channel rhs) noexcept
{
    return !(lhs == rhs);
}

inline constexpr channel none{};

bool is_enabled(channel selected) noexcept;
types::status start(channel selected) noexcept;
types::status stop(channel selected) noexcept;
types::status set_period_us(channel selected,
                            std::uint32_t period_us) noexcept;
types::status set_pulse_us(channel selected,
                           std::uint32_t pulse_us) noexcept;
types::status set_duty(channel selected, float duty_ratio) noexcept;

namespace detail
{
bool backend_enabled(channel selected) noexcept;
types::status backend_start(channel selected) noexcept;
types::status backend_stop(channel selected) noexcept;
types::status backend_set_period_us(channel selected,
                                    std::uint32_t period_us) noexcept;
types::status backend_set_pulse_us(channel selected,
                                   std::uint32_t pulse_us) noexcept;
} // namespace detail

} // namespace bsp::pwm
