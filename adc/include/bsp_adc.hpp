#pragma once

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
