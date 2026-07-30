#pragma once

#include "usertypes.hpp"

#include <cstdint>

namespace bsp::indicator
{

enum class channel : std::uint8_t
{
    red = 0,
    green,
    blue,
    count,
};

struct state
{
    bool initialized = false;
    bool red = false;
    bool green = false;
    bool blue = false;
};

types::status init() noexcept;
types::status set(channel selected, bool on) noexcept;
types::status toggle(channel selected) noexcept;
state snapshot() noexcept;

} // namespace bsp::indicator
