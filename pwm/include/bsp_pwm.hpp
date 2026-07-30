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

    // Legacy template channels remain compile-compatible. A selected board
    // implementation may leave these logical slots unsupported and fail closed.
    static const channel tim3_ch4;
    static const channel tim12_ch2;
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
inline constexpr channel channel::tim3_ch4{0xFEU};
inline constexpr channel channel::tim12_ch2{0xFDU};

bool is_enabled(channel selected) noexcept;
types::status start(channel selected) noexcept;
types::status stop(channel selected) noexcept;
types::status set_period_us(channel selected,
                            std::uint32_t period_us) noexcept;
types::status set_pulse_us(channel selected,
                           std::uint32_t pulse_us) noexcept;
types::status set_duty(channel selected, float duty_ratio) noexcept;
void set_period(channel selected, float period_s) noexcept;

} // namespace bsp::pwm
