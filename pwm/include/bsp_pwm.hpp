#pragma once

#include "config.hpp"
#include "status.hpp"
#include "bsp.hpp"
#include "peripherals.hpp"
#include <cmath>

namespace bsp::pwm
{
using pnxtypes::status;

inline status ready(handle timer, channel output) noexcept
{
    auto* native = peripherals::tim_handle(timer);
    if (native == nullptr || peripherals::pwm_channel(timer, output) == UINT32_MAX)
    {
        return status::not_configured;
    }
    return native->Instance != nullptr && native->State != HAL_TIM_STATE_RESET
               ? status::ok
               : status::not_initialized;
}

inline uint64_t ticks(const config& timer, uint32_t us) noexcept
{
    auto* native = peripherals::tim_handle(timer.instance);
    return static_cast<uint64_t>(timer.timer_clock_hz) * us /
           ((static_cast<uint64_t>(native->Init.Prescaler) + 1U) * 1000000U);
}

inline uint64_t max_compare(TIM_HandleTypeDef* timer) noexcept
{
    return IS_TIM_32B_COUNTER_INSTANCE(timer->Instance) ? UINT32_MAX : UINT16_MAX;
}

inline status ready(const config& timer, channel output) noexcept
{
    const auto number = static_cast<uint8_t>(output);
    if (number < 1 || number > 4 || timer.timer_clock_hz == 0 ||
        (timer.channel_mask & (1U << (number - 1U))) == 0)
    {
        return status::not_configured;
    }
    return ready(timer.instance, output);
}

inline status start(handle timer, channel output) noexcept
{
    if (auto checked = ready(timer, output); checked != status::ok)
    {
        return checked;
    }
    return bsp::to_status(
        HAL_TIM_PWM_Start(peripherals::tim_handle(timer), peripherals::pwm_channel(timer, output)));
}

inline status stop(handle timer, channel output) noexcept
{
    if (auto checked = ready(timer, output); checked != status::ok)
    {
        return checked;
    }
    return bsp::to_status(
        HAL_TIM_PWM_Stop(peripherals::tim_handle(timer), peripherals::pwm_channel(timer, output)));
}

inline status set_duty(handle timer, channel output, float ratio) noexcept
{
    if (auto checked = ready(timer, output); checked != status::ok)
    {
        return checked;
    }
    if (!std::isfinite(ratio) || ratio < 0 || ratio > 1)
    {
        return status::invalid_arg;
    }
    auto* native = peripherals::tim_handle(timer);
    const uint64_t period = static_cast<uint64_t>(__HAL_TIM_GET_AUTORELOAD(native)) + 1;
    const uint64_t compare = static_cast<uint64_t>(static_cast<double>(ratio) * period);
    if (compare > max_compare(native))
    {
        return status::invalid_arg;
    }
    __HAL_TIM_SET_COMPARE(native, peripherals::pwm_channel(timer, output),
                          static_cast<uint32_t>(compare));
    return status::ok;
}

// Period is shared by every channel on the timer.
inline status set_period_us(const config& timer, uint32_t period) noexcept
{
    auto* native = peripherals::tim_handle(timer.instance);
    if (native == nullptr || timer.timer_clock_hz == 0)
    {
        return status::not_configured;
    }
    if (native->Instance == nullptr || native->State == HAL_TIM_STATE_RESET)
    {
        return status::not_initialized;
    }
    const auto count = ticks(timer, period);
    const uint64_t limit = max_compare(native) + 1U;
    if (period == 0 || count == 0 || count > limit)
    {
        return status::invalid_arg;
    }
    __HAL_TIM_SET_AUTORELOAD(native, static_cast<uint32_t>(count - 1));
    return status::ok;
}

inline status set_pulse_width_us(const config& timer, channel output, uint32_t pulse) noexcept
{
    if (auto checked = ready(timer, output); checked != status::ok)
    {
        return checked;
    }
    auto* native = peripherals::tim_handle(timer.instance);
    const auto count = ticks(timer, pulse);
    if ((pulse != 0 && count == 0) ||
        count > static_cast<uint64_t>(__HAL_TIM_GET_AUTORELOAD(native)) + 1 ||
        count > max_compare(native))
    {
        return status::invalid_arg;
    }
    __HAL_TIM_SET_COMPARE(native, peripherals::pwm_channel(timer.instance, output),
                          static_cast<uint32_t>(count));
    return status::ok;
}
} // namespace bsp::pwm
