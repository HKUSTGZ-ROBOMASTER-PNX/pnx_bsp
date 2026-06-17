#pragma once

#include "bsp_can.hpp"

namespace bsp {
namespace can_detail {

void init_classic(CanBus bus);
void transmit_classic(CanBus bus, uint32_t std_id, const uint8_t* data, uint16_t len);

} // namespace can_detail
} // namespace bsp
