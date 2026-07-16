#pragma once

#include "config.hpp"
#include "spi.h"
#include "usertypes.hpp"

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

types::status wait_ready(bus bus, uint32_t timeout_ms = 1000);
bool bus_enabled(bus bus) noexcept;
SPI_HandleTypeDef* handle_of(bus bus) noexcept;
types::status init(bus bus);
types::status transmit(bus bus, const uint8_t* data, size_t len, uint32_t timeout_ms);
types::status receive(bus bus, uint8_t* data, size_t len, uint32_t timeout_ms);
void cs_set(cs line, bool selected);

} // namespace bsp::spi
