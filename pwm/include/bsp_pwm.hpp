#pragma once

#include "config.hpp"
#include "tim.h"
#include "usertypes.hpp"

namespace bsp::pwm
{

constexpr bool is_enabled(channel channel_id)
{
    return static_cast<std::size_t>(channel_id) < channel_count;
}

types::status init(channel channel_id);
types::status start(channel channel_id);
types::status stop(channel channel_id);
types::status set_duty(channel channel_id, float duty_ratio);

// The period belongs to the timer and therefore affects every channel on it.
types::status set_period_us(channel channel_id, std::uint32_t period_us);
types::status set_pulse_width_us(channel channel_id, std::uint32_t pulse_width_us);

} // namespace bsp::pwm

namespace bsp::pwm::detail
{

struct binding
{
    TIM_HandleTypeDef* timer = nullptr;
    std::uint32_t hal_channel = 0;
};

bool binding_for(channel channel_id, binding& out) noexcept;

} // namespace bsp::pwm::detail
