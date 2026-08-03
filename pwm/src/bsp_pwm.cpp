#include "bsp_pwm.hpp"

#include "pwm_channels.hpp"
#include "stm32f4xx_hal.h"

#include <array>
#include <cmath>

namespace bsp::pwm
{
namespace
{

TIM_HandleTypeDef timer{};
bool initialized = false;
constexpr std::uint32_t timer_channel = TIM_CHANNEL_2;
constexpr std::size_t tracked_channel_count = 8U;
std::array<std::uint32_t, tracked_channel_count> periods_us{
    20000U, 20000U, 20000U, 20000U,
    20000U, 20000U, 20000U, 20000U};

bool selected_channel(channel selected) noexcept
{
    return selected == board::pwm::servo_c2;
}

std::uint32_t* period_of(channel selected) noexcept
{
    return selected.value < periods_us.size()
               ? &periods_us[selected.value]
               : nullptr;
}

std::uint32_t timer_clock_hz() noexcept
{
    const std::uint32_t pclk = HAL_RCC_GetPCLK2Freq();
    return (RCC->CFGR & RCC_CFGR_PPRE2) == RCC_HCLK_DIV1
               ? pclk
               : pclk * 2U;
}

types::status ensure_initialized() noexcept
{
    if (initialized)
    {
        return types::status::ok;
    }

    const std::uint32_t clock_hz = timer_clock_hz();
    if (clock_hz < 1000000U || (clock_hz % 1000000U) != 0U)
    {
        return types::status::error;
    }

    __HAL_RCC_GPIOE_CLK_ENABLE();
    __HAL_RCC_TIM1_CLK_ENABLE();

    GPIO_InitTypeDef gpio{};
    gpio.Pin = GPIO_PIN_11;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    gpio.Alternate = GPIO_AF1_TIM1;
    HAL_GPIO_Init(GPIOE, &gpio);

    timer.Instance = TIM1;
    timer.Init.Prescaler = (clock_hz / 1000000U) - 1U;
    timer.Init.CounterMode = TIM_COUNTERMODE_UP;
    timer.Init.Period = 20000U - 1U;
    timer.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    timer.Init.RepetitionCounter = 0U;
    timer.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_PWM_Init(&timer) != HAL_OK)
    {
        return types::status::error;
    }

    TIM_OC_InitTypeDef output{};
    output.OCMode = TIM_OCMODE_PWM1;
    output.Pulse = 0U;
    output.OCPolarity = TIM_OCPOLARITY_HIGH;
    output.OCNPolarity = TIM_OCNPOLARITY_HIGH;
    output.OCFastMode = TIM_OCFAST_DISABLE;
    output.OCIdleState = TIM_OCIDLESTATE_RESET;
    output.OCNIdleState = TIM_OCNIDLESTATE_RESET;
    if (HAL_TIM_PWM_ConfigChannel(&timer, &output, timer_channel) != HAL_OK)
    {
        return types::status::error;
    }

    initialized = true;
    return types::status::ok;
}

} // namespace

bool is_enabled(channel selected) noexcept
{
    return selected_channel(selected);
}

types::status start(channel selected) noexcept
{
    if (!is_enabled(selected))
    {
        return types::status::not_configured;
    }
    if (ensure_initialized() != types::status::ok)
    {
        return types::status::error;
    }
    return HAL_TIM_PWM_Start(&timer, timer_channel) == HAL_OK
               ? types::status::ok
               : types::status::error;
}

types::status stop(channel selected) noexcept
{
    if (!is_enabled(selected))
    {
        return types::status::not_configured;
    }
    if (!initialized)
    {
        return types::status::ok;
    }

    __HAL_TIM_SET_COMPARE(&timer, timer_channel, 0U);
    const HAL_StatusTypeDef result =
        HAL_TIM_PWM_Stop(&timer, timer_channel);
    (void)HAL_TIM_PWM_DeInit(&timer);
    HAL_GPIO_DeInit(GPIOE, GPIO_PIN_11);
    __HAL_RCC_TIM1_CLK_DISABLE();
    initialized = false;
    return result == HAL_OK ? types::status::ok
                            : types::status::error;
}

types::status set_period_us(channel selected,
                            std::uint32_t period_us) noexcept
{
    std::uint32_t* current = period_of(selected);
    if (!is_enabled(selected))
    {
        return types::status::not_configured;
    }
    if (current == nullptr || period_us == 0U || period_us > 65536U)
    {
        return types::status::invalid_arg;
    }
    if (ensure_initialized() != types::status::ok)
    {
        return types::status::error;
    }

    __HAL_TIM_SET_AUTORELOAD(&timer, period_us - 1U);
    __HAL_TIM_SET_COUNTER(&timer, 0U);
    *current = period_us;
    return types::status::ok;
}

types::status set_pulse_us(channel selected,
                           std::uint32_t pulse_us) noexcept
{
    std::uint32_t* current = period_of(selected);
    if (!is_enabled(selected))
    {
        return types::status::not_configured;
    }
    if (current == nullptr || pulse_us > *current)
    {
        return types::status::invalid_arg;
    }
    if (ensure_initialized() != types::status::ok)
    {
        return types::status::error;
    }

    __HAL_TIM_SET_COMPARE(&timer, timer_channel, pulse_us);
    return types::status::ok;
}

types::status set_duty(channel selected, float duty_ratio) noexcept
{
    std::uint32_t* current = period_of(selected);
    if (!is_enabled(selected))
    {
        return types::status::not_configured;
    }
    if (current == nullptr || !std::isfinite(duty_ratio))
    {
        return types::status::invalid_arg;
    }
    if (duty_ratio < 0.0F)
    {
        duty_ratio = 0.0F;
    }
    else if (duty_ratio > 1.0F)
    {
        duty_ratio = 1.0F;
    }
    return set_pulse_us(
        selected,
        static_cast<std::uint32_t>(
            duty_ratio * static_cast<float>(*current)));
}

void set_period(channel selected, float period_s) noexcept
{
    if (!is_enabled(selected) || !std::isfinite(period_s) ||
        period_s <= 0.0F || period_s > 0.065536F)
    {
        return;
    }
    (void)set_period_us(
        selected,
        static_cast<std::uint32_t>(period_s * 1000000.0F));
}

} // namespace bsp::pwm
