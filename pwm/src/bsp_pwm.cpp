#include "bsp_pwm.hpp"

#include <cmath>
#include <cstdint>
#include <limits>

namespace bsp::pwm
{
namespace
{

constexpr std::uint32_t heater_lease_timeout_ms = 250U;

alignas(32) __attribute__((section(".ram_d2_bss"))) std::uint32_t heater_off_dma_source[8];

struct heater_lease_state
{
    bool initialized = false;
    volatile std::uint32_t fault_count = 0U;
};

heater_lease_state& heater_lease() noexcept
{
    static heater_lease_state state{};
    return state;
}

constexpr std::uint64_t compare_for_duty(std::uint64_t period_ticks, float duty_ratio) noexcept
{
    return static_cast<std::uint64_t>(duty_ratio * static_cast<double>(period_ticks));
}

static_assert(compare_for_duty(2'000U, 0.0f) == 0U);
static_assert(compare_for_duty(2'000U, 0.5f) == 1'000U);
static_assert(compare_for_duty(2'000U, 1.0f) == 2'000U);

const channel_config* config_of(channel channel_id) noexcept
{
    const auto index = static_cast<std::size_t>(channel_id);
    return index < channel_count ? &configs[index] : nullptr;
}

types::status get_binding(channel channel_id, detail::binding& binding) noexcept
{
    if (!is_enabled(channel_id))
    {
        return types::status::invalid_arg;
    }
    if (!detail::binding_for(channel_id, binding) || binding.timer == nullptr || binding.hal_channel == 0U)
    {
        return types::status::not_configured;
    }
    return types::status::ok;
}

bool is_heater_pwm(const detail::binding& binding) noexcept
{
    return binding.timer == &htim3 && binding.hal_channel == TIM_CHANNEL_4;
}

std::uint32_t apb1_timer_clock_hz() noexcept
{
    const std::uint32_t pclk1_hz = HAL_RCC_GetPCLK1Freq();
    RCC_ClkInitTypeDef clock_config{};
    std::uint32_t flash_latency = 0U;
    HAL_RCC_GetClockConfig(&clock_config, &flash_latency);

    if ((RCC->CFGR & RCC_CFGR_TIMPRE) == 0U)
    {
        return clock_config.APB1CLKDivider == RCC_APB1_DIV1 ? pclk1_hz : pclk1_hz * 2U;
    }
    return HAL_RCC_GetHCLKFreq();
}

DMA_HandleTypeDef* heater_lease_dma() noexcept
{
    return htim6.Instance == TIM6 ? htim6.hdma[TIM_DMA_ID_UPDATE] : nullptr;
}

bool heater_lease_timing_is_valid() noexcept
{
    const std::uint64_t timer_clock_hz = apb1_timer_clock_hz();
    const std::uint64_t divider = static_cast<std::uint64_t>(htim6.Init.Prescaler) + 1U;
    const std::uint64_t period = static_cast<std::uint64_t>(htim6.Init.Period) + 1U;
    return timer_clock_hz != 0U && divider != 0U && period != 0U &&
           divider * period * 1'000U == timer_clock_hz * heater_lease_timeout_ms;
}

types::status init_heater_lease() noexcept
{
    heater_lease_state& state = heater_lease();
    if (state.initialized)
    {
        return types::status::ok;
    }

    // Both timers must keep running while the Cortex-M7 is halted by a debugger.
    __HAL_DBGMCU_UnFreeze_TIM3();
    __HAL_DBGMCU_UnFreeze_TIM6();

    for (std::uint32_t& word : heater_off_dma_source)
    {
        word = 0U;
    }
    SCB_CleanDCache_by_Addr(heater_off_dma_source,
                            static_cast<std::int32_t>(sizeof(heater_off_dma_source)));
    __DSB();

    DMA_HandleTypeDef* dma = heater_lease_dma();
    if (dma == nullptr || dma->Init.Request != DMA_REQUEST_TIM6_UP ||
        dma->Init.Direction != DMA_MEMORY_TO_PERIPH || !heater_lease_timing_is_valid())
    {
        ++state.fault_count;
        return types::status::error;
    }

    CLEAR_BIT(TIM6->CR1, TIM_CR1_CEN);
    CLEAR_BIT(TIM6->DIER, TIM_DIER_UDE);
    SET_BIT(TIM6->CR1, TIM_CR1_OPM);
    state.initialized = true;
    return types::status::ok;
}

void stop_heater_lease() noexcept
{
    heater_lease_state& state = heater_lease();
    if (!state.initialized)
    {
        return;
    }

    CLEAR_BIT(TIM6->CR1, TIM_CR1_CEN);
    CLEAR_BIT(TIM6->DIER, TIM_DIER_UDE);
    DMA_HandleTypeDef* dma = heater_lease_dma();
    if (dma != nullptr && dma->State == HAL_DMA_STATE_BUSY && HAL_DMA_Abort(dma) != HAL_OK)
    {
        ++state.fault_count;
    }
}

bool arm_heater_lease() noexcept
{
    heater_lease_state& state = heater_lease();
    DMA_HandleTypeDef* dma = heater_lease_dma();
    if (!state.initialized || dma == nullptr)
    {
        ++state.fault_count;
        return false;
    }

    __HAL_TIM_SET_COUNTER(&htim6, 0U);
    CLEAR_BIT(TIM6->DIER, TIM_DIER_UDE);
    SET_BIT(TIM6->EGR, TIM_EGR_UG);
    CLEAR_BIT(TIM6->SR, TIM_SR_UIF);

    const auto source = static_cast<std::uint32_t>(reinterpret_cast<std::uintptr_t>(heater_off_dma_source));
    const auto destination = static_cast<std::uint32_t>(reinterpret_cast<std::uintptr_t>(&TIM3->CCR4));
    if (HAL_DMA_Start(dma, source, destination, 1U) != HAL_OK)
    {
        ++state.fault_count;
        return false;
    }

    CLEAR_BIT(TIM6->SR, TIM_SR_UIF);
    SET_BIT(TIM6->DIER, TIM_DIER_UDE);
    SET_BIT(TIM6->CR1, TIM_CR1_OPM | TIM_CR1_CEN);
    return true;
}

types::status start_heater_pwm(const detail::binding& binding) noexcept
{
    __HAL_TIM_SET_COMPARE(binding.timer, binding.hal_channel, 0U);
    if (init_heater_lease() != types::status::ok)
    {
        return types::status::error;
    }

    stop_heater_lease();
    if (HAL_TIM_PWM_Start(binding.timer, binding.hal_channel) != HAL_OK)
    {
        return types::status::error;
    }

    const std::uint32_t interrupt_posture = __get_PRIMASK();
    __disable_irq();
    __HAL_TIM_SET_COUNTER(binding.timer, 0U);
    if (interrupt_posture == 0U)
    {
        __enable_irq();
    }
    return types::status::ok;
}

types::status set_heater_compare_with_lease(const detail::binding& binding,
                                            std::uint32_t compare) noexcept
{
    const std::uint32_t interrupt_posture = __get_PRIMASK();
    __disable_irq(); //短暂关闭中断防止被打断

    __HAL_TIM_SET_COMPARE(binding.timer, binding.hal_channel, 0U);
    stop_heater_lease();

    types::status status = types::status::ok;
    if (compare != 0U)
    {
        if (arm_heater_lease())
        {
            __HAL_TIM_SET_COMPARE(binding.timer, binding.hal_channel, compare);
        }
        else
        {
            status = types::status::error;
        }
    }

    __DSB();
    if (interrupt_posture == 0U)
    {
        __enable_irq();
    }
    return status;
}

bool timer_ticks_for_us(const channel_config& config,
                        const TIM_HandleTypeDef& timer,
                        std::uint32_t duration_us,
                        std::uint64_t& ticks) noexcept
{
    const std::uint64_t divider = static_cast<std::uint64_t>(timer.Init.Prescaler) + 1U;
    if (config.timer_clock_hz == 0U || divider == 0U)
    {
        return false;
    }
    const std::uint64_t timer_tick_hz = static_cast<std::uint64_t>(config.timer_clock_hz) / divider;
    if (timer_tick_hz == 0U || duration_us > std::numeric_limits<std::uint64_t>::max() / timer_tick_hz)
    {
        return false;
    }
    ticks = timer_tick_hz * duration_us / 1'000'000U;
    return ticks != 0U || duration_us == 0U;
}

} // namespace

types::status init(channel channel_id)
{
    detail::binding binding{};
    const types::status status = get_binding(channel_id, binding);
    const channel_config* config = config_of(channel_id);
    if (status != types::status::ok || config == nullptr || config->timer_clock_hz == 0U)
    {
        return status == types::status::ok ? types::status::not_configured : status;
    }
    return is_heater_pwm(binding) ? init_heater_lease() : types::status::ok;
}

types::status start(channel channel_id)
{
    detail::binding binding{};
    const types::status status = get_binding(channel_id, binding);
    if (status != types::status::ok)
    {
        return status;
    }
    if (is_heater_pwm(binding))
    {
        return start_heater_pwm(binding);
    }
    return HAL_TIM_PWM_Start(binding.timer, binding.hal_channel) == HAL_OK
               ? types::status::ok
               : types::status::error;
}

types::status stop(channel channel_id)
{
    detail::binding binding{};
    const types::status status = get_binding(channel_id, binding);
    if (status != types::status::ok)
    {
        return status;
    }
    if (is_heater_pwm(binding))
    {
        const std::uint32_t interrupt_posture = __get_PRIMASK();
        __disable_irq();
        __HAL_TIM_SET_COMPARE(binding.timer, binding.hal_channel, 0U);
        stop_heater_lease();
        if (interrupt_posture == 0U)
        {
            __enable_irq();
        }
    }
    return HAL_TIM_PWM_Stop(binding.timer, binding.hal_channel) == HAL_OK
               ? types::status::ok
               : types::status::error;
}

types::status set_duty(channel channel_id, float duty_ratio)
{
    detail::binding binding{};
    const types::status status = get_binding(channel_id, binding);
    if (status != types::status::ok)
    {
        return status;
    }
    if (!std::isfinite(duty_ratio) || duty_ratio < 0.0f || duty_ratio > 1.0f)
    {
        return types::status::invalid_arg;
    }
    const std::uint64_t period_ticks = static_cast<std::uint64_t>(__HAL_TIM_GET_AUTORELOAD(binding.timer)) + 1U;
    const std::uint64_t compare = compare_for_duty(period_ticks, duty_ratio);
    if (compare > std::numeric_limits<std::uint32_t>::max())
    {
        return types::status::invalid_arg;
    }
    const auto compare_value = static_cast<std::uint32_t>(compare);
    if (is_heater_pwm(binding))
    {
        return set_heater_compare_with_lease(binding, compare_value);
    }
    __HAL_TIM_SET_COMPARE(binding.timer, binding.hal_channel, compare_value);
    return types::status::ok;
}

types::status set_period_us(channel channel_id, std::uint32_t period_us)
{
    detail::binding binding{};
    const types::status status = get_binding(channel_id, binding);
    const channel_config* config = config_of(channel_id);
    if (status != types::status::ok || config == nullptr)
    {
        return status == types::status::ok ? types::status::not_configured : status;
    }
    if (period_us == 0U)
    {
        return types::status::invalid_arg;
    }
    std::uint64_t period_ticks = 0U;
    if (!timer_ticks_for_us(*config, *binding.timer, period_us, period_ticks))
    {
        return types::status::invalid_arg;
    }
    const std::uint64_t max_period_ticks = IS_TIM_32B_COUNTER_INSTANCE(binding.timer->Instance)
                                               ? 0x1'0000'0000ULL
                                               : 0x1'0000ULL;
    if (period_ticks == 0U || period_ticks > max_period_ticks)
    {
        return types::status::invalid_arg;
    }
    __HAL_TIM_SET_AUTORELOAD(binding.timer, static_cast<std::uint32_t>(period_ticks - 1U));
    return types::status::ok;
}

types::status set_pulse_width_us(channel channel_id, std::uint32_t pulse_width_us)
{
    detail::binding binding{};
    const types::status status = get_binding(channel_id, binding);
    const channel_config* config = config_of(channel_id);
    if (status != types::status::ok || config == nullptr)
    {
        return status == types::status::ok ? types::status::not_configured : status;
    }
    std::uint64_t pulse_ticks = 0U;
    if (!timer_ticks_for_us(*config, *binding.timer, pulse_width_us, pulse_ticks))
    {
        return types::status::invalid_arg;
    }
    const std::uint64_t period_ticks = static_cast<std::uint64_t>(__HAL_TIM_GET_AUTORELOAD(binding.timer)) + 1U;
    if (pulse_ticks > period_ticks)
    {
        return types::status::invalid_arg;
    }
    const auto compare_value = static_cast<std::uint32_t>(pulse_ticks);
    if (is_heater_pwm(binding))
    {
        return set_heater_compare_with_lease(binding, compare_value);
    }
    __HAL_TIM_SET_COMPARE(binding.timer, binding.hal_channel, compare_value);
    return types::status::ok;
}

} // namespace bsp::pwm
