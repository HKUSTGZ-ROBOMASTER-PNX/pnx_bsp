#include "bsp_pwm.hpp"

#include <array>
#include <cmath>

namespace
{

constexpr std::size_t tracked_channel_count = 8U;
std::array<std::uint32_t, tracked_channel_count> periods_us{
    20000U, 20000U, 20000U, 20000U,
    20000U, 20000U, 20000U, 20000U};

std::uint32_t* period_of(bsp::pwm::channel selected) noexcept
{
    return selected.value < periods_us.size()
               ? &periods_us[selected.value]
               : nullptr;
}

} // namespace

namespace bsp::pwm
{

bool is_enabled(channel selected) noexcept
{
    return detail::backend_enabled(selected);
}

types::status start(channel selected) noexcept
{
    return is_enabled(selected)
               ? detail::backend_start(selected)
               : types::status::not_configured;
}

types::status stop(channel selected) noexcept
{
    return is_enabled(selected)
               ? detail::backend_stop(selected)
               : types::status::not_configured;
}

types::status set_period_us(channel selected,
                            std::uint32_t period_us) noexcept
{
    std::uint32_t* current = period_of(selected);
    if (!is_enabled(selected))
    {
        return types::status::not_configured;
    }
    if (current == nullptr || period_us == 0U)
    {
        return types::status::invalid_arg;
    }

    const types::status result =
        detail::backend_set_period_us(selected, period_us);
    if (result == types::status::ok)
    {
        *current = period_us;
    }
    return result;
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
    return detail::backend_set_pulse_us(selected, pulse_us);
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

} // namespace bsp::pwm
