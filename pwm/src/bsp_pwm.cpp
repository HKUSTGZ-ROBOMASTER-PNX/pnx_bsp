#include "bsp_pwm.hpp"

// TIM1 的 attended PWM 驗證閉包：CH1/CH2/CH3/CH4 → PE9/PE11/PE13/PE14。
// 四路共用同一 ARR（20ms / 50Hz）給同類舵機；MyCar 底盤不使用它作為馬達控制介面。
// 手寫 HAL 初始化，不經 CubeMX；IOC 未占用 TIM1 與這四個腳位。

#include "pwm_channels.hpp"
#include "stm32f4xx_hal.h"

#include <array>
#include <cmath>
#include <cstddef>

namespace bsp::pwm
{
namespace
{

struct channel_binding
{
    channel logical;
    std::uint32_t timer_channel;
    std::uint16_t gpio_pin;
};

// 邏輯通道 → TIM1 硬體通道 / GPIOE 腳位。
constexpr std::array<channel_binding, board::pwm::servo_channel_count>
    bindings{{
        {board::pwm::servo_c2, TIM_CHANNEL_2, GPIO_PIN_11},
        {board::pwm::servo_pe9, TIM_CHANNEL_1, GPIO_PIN_9},
        {board::pwm::servo_pe13, TIM_CHANNEL_3, GPIO_PIN_13},
        {board::pwm::servo_pe14, TIM_CHANNEL_4, GPIO_PIN_14},
    }};

constexpr std::uint32_t default_period_us = 20000U;

TIM_HandleTypeDef timer{};
bool initialized = false;

// 週期由共用時基決定；started 是每通道獨立的 capture/compare 輸出狀態。
std::uint32_t shared_period_us = default_period_us;
std::array<bool, board::pwm::servo_channel_count> channel_started{};

const channel_binding* binding_of(channel selected) noexcept
{
    for (const channel_binding& candidate : bindings)
    {
        if (candidate.logical == selected)
        {
            return &candidate;
        }
    }
    return nullptr;
}

std::size_t index_of(const channel_binding& binding) noexcept
{
    return static_cast<std::size_t>(&binding - bindings.data());
}

std::uint16_t all_pins() noexcept
{
    std::uint16_t pins = 0U;
    for (const channel_binding& candidate : bindings)
    {
        pins = static_cast<std::uint16_t>(pins | candidate.gpio_pin);
    }
    return pins;
}

bool any_channel_started() noexcept
{
    for (const bool started : channel_started)
    {
        if (started)
        {
            return true;
        }
    }
    return false;
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
    gpio.Pin = all_pins();
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    gpio.Alternate = GPIO_AF1_TIM1;
    HAL_GPIO_Init(GPIOE, &gpio);

    timer.Instance = TIM1;
    timer.Init.Prescaler = (clock_hz / 1000000U) - 1U;
    timer.Init.CounterMode = TIM_COUNTERMODE_UP;
    timer.Init.Period = shared_period_us - 1U;
    timer.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    timer.Init.RepetitionCounter = 0U;
    timer.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_PWM_Init(&timer) != HAL_OK)
    {
        return types::status::error;
    }

    // 全部通道以零脈寬配置，維持上電零輸出。
    for (const channel_binding& candidate : bindings)
    {
        TIM_OC_InitTypeDef output{};
        output.OCMode = TIM_OCMODE_PWM1;
        output.Pulse = 0U;
        output.OCPolarity = TIM_OCPOLARITY_HIGH;
        output.OCNPolarity = TIM_OCNPOLARITY_HIGH;
        output.OCFastMode = TIM_OCFAST_DISABLE;
        output.OCIdleState = TIM_OCIDLESTATE_RESET;
        output.OCNIdleState = TIM_OCNIDLESTATE_RESET;
        if (HAL_TIM_PWM_ConfigChannel(
                &timer, &output, candidate.timer_channel) != HAL_OK)
        {
            return types::status::error;
        }
    }

    initialized = true;
    return types::status::ok;
}

} // namespace

bool is_enabled(channel selected) noexcept
{
    return binding_of(selected) != nullptr;
}

types::status start(channel selected) noexcept
{
    const channel_binding* binding = binding_of(selected);
    if (binding == nullptr)
    {
        return types::status::not_configured;
    }
    if (ensure_initialized() != types::status::ok)
    {
        return types::status::error;
    }
    if (HAL_TIM_PWM_Start(&timer, binding->timer_channel) != HAL_OK)
    {
        return types::status::error;
    }
    channel_started[index_of(*binding)] = true;
    return types::status::ok;
}

types::status stop(channel selected) noexcept
{
    const channel_binding* binding = binding_of(selected);
    if (binding == nullptr)
    {
        return types::status::not_configured;
    }
    if (!initialized)
    {
        return types::status::ok;
    }

    __HAL_TIM_SET_COMPARE(&timer, binding->timer_channel, 0U);
    const HAL_StatusTypeDef result =
        HAL_TIM_PWM_Stop(&timer, binding->timer_channel);
    channel_started[index_of(*binding)] = false;

    // 共用時基：只有最後一路停止時才釋放 TIM1 與腳位。
    if (!any_channel_started())
    {
        (void)HAL_TIM_PWM_DeInit(&timer);
        HAL_GPIO_DeInit(GPIOE, all_pins());
        __HAL_RCC_TIM1_CLK_DISABLE();
        initialized = false;
        shared_period_us = default_period_us;
    }
    return result == HAL_OK ? types::status::ok
                            : types::status::error;
}

types::status set_period_us(channel selected,
                            std::uint32_t period_us) noexcept
{
    if (!is_enabled(selected))
    {
        return types::status::not_configured;
    }
    if (period_us == 0U || period_us > 65536U)
    {
        return types::status::invalid_arg;
    }
    if (ensure_initialized() != types::status::ok)
    {
        return types::status::error;
    }

    // Fail closed: shortening ARR must not leave any existing CCR above the
    // new period, which would otherwise make that PWM output continuously
    // active in PWM mode 1.
    for (const channel_binding& candidate : bindings)
    {
        if (__HAL_TIM_GET_COMPARE(&timer, candidate.timer_channel) > period_us)
        {
            return types::status::invalid_arg;
        }
    }

    // ARR 是 timer 層級資源：改一路的週期會套用到全部四路。
    __HAL_TIM_SET_AUTORELOAD(&timer, period_us - 1U);
    __HAL_TIM_SET_COUNTER(&timer, 0U);
    shared_period_us = period_us;
    return types::status::ok;
}

types::status set_pulse_us(channel selected,
                           std::uint32_t pulse_us) noexcept
{
    const channel_binding* binding = binding_of(selected);
    if (binding == nullptr)
    {
        return types::status::not_configured;
    }
    if (pulse_us > shared_period_us)
    {
        return types::status::invalid_arg;
    }
    if (ensure_initialized() != types::status::ok)
    {
        return types::status::error;
    }

    __HAL_TIM_SET_COMPARE(&timer, binding->timer_channel, pulse_us);
    return types::status::ok;
}

types::status set_duty(channel selected, float duty_ratio) noexcept
{
    if (!is_enabled(selected))
    {
        return types::status::not_configured;
    }
    if (!std::isfinite(duty_ratio))
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
            duty_ratio * static_cast<float>(shared_period_us)));
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
