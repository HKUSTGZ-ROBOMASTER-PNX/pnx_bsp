#pragma once

#include "can.h"
#include "config.hpp"

namespace bsp::can
{

CAN_HandleTypeDef* handle_of(bus selected) noexcept;
bus bus_of(CAN_HandleTypeDef* handle) noexcept;

} // namespace bsp::can

namespace bsp::can::detail
{

struct binding
{
    CAN_HandleTypeDef* can = nullptr;
};

bool binding_for(bus bus_id, binding& out) noexcept;

} // namespace bsp::can::detail
