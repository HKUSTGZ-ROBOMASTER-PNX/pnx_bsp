#pragma once

#include "adc.h"
#include "config.hpp"
#include "usertypes.hpp"

#include <cstdint>

namespace bsp::adc
{

constexpr bool is_enabled(channel channel_id)
{
    return static_cast<std::size_t>(channel_id) < channel_count;
}

types::status init(channel channel_id);
types::status calibrate(channel channel_id);
types::status read_raw(channel channel_id, std::uint32_t& raw_value, std::uint32_t timeout_ms);

} // namespace bsp::adc

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
