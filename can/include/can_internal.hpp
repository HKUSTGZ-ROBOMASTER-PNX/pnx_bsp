#pragma once

#include "bsp_can.hpp"

#include "fdcan.h"

#include <cstdint>

namespace bsp {
namespace can_detail {

CanBus bus_from_handle(FDCAN_HandleTypeDef* handle);
CanType query_bus_type(CanBus bus);

void on_rx(CanBus bus, uint32_t id, const uint8_t* data, uint8_t len);
void on_error_status(CanBus bus, FDCAN_HandleTypeDef* handle, uint32_t error_status_its);

uint8_t rx_dlc_to_len(uint32_t dlc);

} // namespace can_detail
} // namespace bsp
