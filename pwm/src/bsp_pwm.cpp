#include "bsp_pwm.hpp"

namespace
{

struct pwm_binding
{
    TIM_HandleTypeDef* timer = nullptr;
    uint32_t hal_channel = 0;
};

bool binding_for(bsp::pwm::channel ch, pwm_binding* out)
{
    if (out == nullptr)
    {
        return false;
    }

    if (!bsp::pwm::is_enabled(ch))
    {
        return false;
    }

    out->timer = bsp::pwm::timer_of(ch);
    out->hal_channel = bsp::pwm::hal_channel_of(ch);
    return out->timer != nullptr && out->hal_channel != 0U;
}

} // namespace

namespace bsp::pwm
{

namespace
{

const channel_config* config_of(channel ch) noexcept
{
    const auto index = static_cast<std::size_t>(ch);
    return index < channel_count ? &configs[index] : nullptr;
}

TIM_HandleTypeDef* timer_from_id(timer_id id) noexcept
{
    switch (id)
    {
    case timer_id::tim3:
        return &htim3;
    case timer_id::tim12:
        return &htim12;
    default:
        return nullptr;
    }
}

std::uint32_t hal_channel_from_id(channel_id id) noexcept
{
    switch (id)
    {
    case channel_id::ch1:
        return TIM_CHANNEL_1;
    case channel_id::ch2:
        return TIM_CHANNEL_2;
    case channel_id::ch3:
        return TIM_CHANNEL_3;
    case channel_id::ch4:
        return TIM_CHANNEL_4;
    default:
        return 0;
    }
}

} // namespace

TIM_HandleTypeDef* timer_of(channel ch) noexcept
{
    const channel_config* cfg = config_of(ch);
    return cfg != nullptr && cfg->enabled ? timer_from_id(cfg->timer) : nullptr;
}

std::uint32_t hal_channel_of(channel ch) noexcept
{
    const channel_config* cfg = config_of(ch);
    return cfg != nullptr && cfg->enabled ? hal_channel_from_id(cfg->channel) : 0;
}

types::status start(channel ch)
{
    pwm_binding binding{};
    if (!binding_for(ch, &binding))
    {
        return is_enabled(ch) ? types::status::invalid_arg : types::status::not_configured;
    }

    __HAL_TIM_SET_COMPARE(binding.timer, binding.hal_channel, 0);
    if (HAL_TIM_PWM_Start(binding.timer, binding.hal_channel) != HAL_OK)
    {
        return types::status::error;
    }

    return types::status::ok;
}

types::status stop(channel ch)
{
    pwm_binding binding{};
    if (!binding_for(ch, &binding))
    {
        return is_enabled(ch) ? types::status::invalid_arg : types::status::not_configured;
    }

    if (HAL_TIM_PWM_Stop(binding.timer, binding.hal_channel) != HAL_OK)
    {
        return types::status::error;
    }

    return types::status::ok;
}

void set_period(channel ch, float period_s)
{
    pwm_binding binding{};
    if (!binding_for(ch, &binding) || period_s <= 0.0f)
    {
        return;
    }

    const uint32_t prescaler = binding.timer->Init.Prescaler + 1U;
    const channel_config* cfg = config_of(ch);
    const uint32_t source_clock_hz = cfg != nullptr ? cfg->timer_clock_hz : 0U;
    if (source_clock_hz == 0U)
    {
        return;
    }

    __HAL_TIM_SET_AUTORELOAD(
        binding.timer,
        static_cast<uint32_t>(period_s * (static_cast<float>(source_clock_hz) / static_cast<float>(prescaler))));
}

void set_duty(channel ch, float duty_ratio)
{
    pwm_binding binding{};
    if (!binding_for(ch, &binding))
    {
        return;
    }

    if (duty_ratio < 0.0f)
    {
        duty_ratio = 0.0f;
    }
    if (duty_ratio > 1.0f)
    {
        duty_ratio = 1.0f;
    }

    const uint32_t period = __HAL_TIM_GET_AUTORELOAD(binding.timer) + 1U;
    __HAL_TIM_SET_COMPARE(binding.timer, binding.hal_channel,
                          static_cast<uint32_t>(duty_ratio * static_cast<float>(period)));
}

} // namespace bsp::pwm
