#pragma once

#include "adc.h"
#include "bsp_adc.hpp"

namespace bsp::adc::detail
{

struct binding
{
    ADC_HandleTypeDef* adc = nullptr;
    std::uint32_t hal_channel = 0;
    std::uint32_t rank = 0;
    std::uint32_t sampling_time = 0;
};

bool binding_for(channel channel_id, binding& out) noexcept;

} // namespace bsp::adc::detail
