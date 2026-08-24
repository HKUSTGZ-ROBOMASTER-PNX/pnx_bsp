#include "bsp_adc.hpp"

namespace bsp::adc
{
namespace
{

types::status get_binding(channel channel_id, detail::binding& binding) noexcept
{
    if (!is_enabled(channel_id))
    {
        return types::status::invalid_arg;
    }
    if (!detail::binding_for(channel_id, binding) || binding.adc == nullptr || binding.rank == 0U)
    {
        return types::status::not_configured;
    }
    return types::status::ok;
}

} // namespace

types::status init(channel channel_id)
{
    detail::binding binding{};
    const types::status status = get_binding(channel_id, binding);
    if (status != types::status::ok)
    {
        return status;
    }
    if (binding.adc->Init.NbrOfConversion != 1U || binding.adc->Init.ScanConvMode != ADC_SCAN_DISABLE ||
        binding.adc->Init.ExternalTrigConv != ADC_SOFTWARE_START)
    {
        return types::status::not_configured;
    }
    return types::status::ok;
}

types::status calibrate(channel channel_id)
{
    detail::binding binding{};
    const types::status status = get_binding(channel_id, binding);
    if (status != types::status::ok)
    {
        return status;
    }
    return HAL_ADCEx_Calibration_Start(binding.adc, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED) == HAL_OK
               ? types::status::ok
               : types::status::error;
}

types::status read_raw(channel channel_id, std::uint32_t& raw_value, std::uint32_t timeout_ms)
{
    detail::binding binding{};
    const types::status status = get_binding(channel_id, binding);
    if (status != types::status::ok)
    {
        return status;
    }

    ADC_ChannelConfTypeDef channel_config{};
    channel_config.Channel = binding.hal_channel;
    channel_config.Rank = binding.rank;
    channel_config.SamplingTime = binding.sampling_time;
    channel_config.SingleDiff = ADC_SINGLE_ENDED;
    channel_config.OffsetNumber = ADC_OFFSET_NONE;
    channel_config.Offset = 0;
    channel_config.OffsetSignedSaturation = DISABLE;
    if (HAL_ADC_ConfigChannel(binding.adc, &channel_config) != HAL_OK)
    {
        return types::status::error;
    }
    if (HAL_ADC_Start(binding.adc) != HAL_OK)
    {
        return types::status::error;
    }
    if (HAL_ADC_PollForConversion(binding.adc, timeout_ms) != HAL_OK)
    {
        static_cast<void>(HAL_ADC_Stop(binding.adc));
        return types::status::error;
    }

    const std::uint32_t value = HAL_ADC_GetValue(binding.adc);
    if (HAL_ADC_Stop(binding.adc) != HAL_OK)
    {
        return types::status::error;
    }
    raw_value = value;
    return types::status::ok;
}

} // namespace bsp::adc
