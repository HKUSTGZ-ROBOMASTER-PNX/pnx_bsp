#pragma once

#include "config.hpp"
#include "tim.h"
#include "usertypes.hpp"

namespace bsp::pwm
{

enum class channel : uint8_t
{
    tim3_ch4 = 0,
    tim12_ch2 = 1,
};

constexpr bool is_enabled(channel ch)
{
    const auto index = static_cast<std::size_t>(ch);
    return index < channel_count && configs[index].enabled;
}

TIM_HandleTypeDef* timer_of(channel ch) noexcept;
std::uint32_t hal_channel_of(channel ch) noexcept;

types::status start(channel ch);
types::status stop(channel ch);
void set_duty(channel ch, float duty_ratio);
void set_period(channel ch, float period_s);

} // namespace bsp::pwm
