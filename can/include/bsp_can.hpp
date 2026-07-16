#pragma once

#include "config.hpp"
#include "fdcan.h"
#include "tx_api.h"
#include "usertypes.hpp"

#include <cstddef>

namespace bsp::can
{

struct rx_frame
{
    uint32_t id = 0;
    uint8_t len = 0;
    uint8_t data[64]{};
};

using rx_handler = void (*)(bus bus, const rx_frame& frame, void* user_data);

FDCAN_HandleTypeDef* handle_of(bus b) noexcept;
bus bus_of(FDCAN_HandleTypeDef* handle) noexcept;
bool bus_enabled(std::size_t index) noexcept;
bus_type configured_bus_type(std::size_t index) noexcept;
id_type filter_id_type_of(std::size_t index) noexcept;

void receive(FDCAN_HandleTypeDef* handle, uint32_t rx_fifo, bus_type mode);
void handle_error(FDCAN_HandleTypeDef* handle, uint32_t error_status_its);

types::status init(bus bus, bus_type type);
types::status transmit(bus bus, uint32_t id, const uint8_t* data, uint16_t len);

types::status register_rx_handler(bus bus, rx_handler handler, void* user_data);
void unregister_rx_handlers(bus bus);

TX_SEMAPHORE* err_sem(bus bus);

} // namespace bsp::can
