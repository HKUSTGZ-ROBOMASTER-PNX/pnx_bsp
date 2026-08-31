#pragma once

#include "config.hpp"
#include "bsp_dma.hpp"
#include "spi.h"
#include "usertypes.hpp"

#include <cstddef>
#include <cstdint>

namespace bsp::spi
{

enum class bus : uint8_t
{
    spi2 = 0,
    spi6 = 1,
};

enum class cs : uint8_t
{
    bmi088_acc = 0,
    bmi088_gyro = 1,
};

struct transfer_state
{
    volatile bool busy = false;
    volatile bool complete = false;
    volatile bool receive = false;
    volatile bool dma = false;
    volatile types::status last_status = types::status::ok;
};

types::status wait_ready(bus bus, uint32_t timeout_ms = 1000);
bool bus_enabled(bus bus) noexcept;
SPI_HandleTypeDef* handle_of(bus bus) noexcept;
types::status init(bus bus);
types::status transmit(bus bus, const uint8_t* data, size_t len, uint32_t timeout_ms);
types::status receive(bus bus, uint8_t* data, size_t len, uint32_t timeout_ms);
// IT buffers remain owned by the caller until the transfer state reports completion.
types::status transmit_it(bus bus, const uint8_t* data, size_t len);
types::status receive_it(bus bus, uint8_t* data, size_t len);
// DMA buffers must be static bsp::dma::buffer<N> objects placed with BSP_DMA_BUFFER,
// and remain owned by the caller until the transfer state reports completion.
types::status transmit_dma(bus bus, bsp::dma::buffer_view buffer);
types::status receive_dma(bus bus, bsp::dma::buffer_view buffer);
const transfer_state& state(bus bus) noexcept;
void handle_tx_complete(SPI_HandleTypeDef* handle) noexcept;
void handle_rx_complete(SPI_HandleTypeDef* handle) noexcept;
void handle_error(SPI_HandleTypeDef* handle) noexcept;
void cs_set(cs line, bool selected);

} // namespace bsp::spi
