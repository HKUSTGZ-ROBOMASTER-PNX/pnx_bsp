#include "bsp_ws2812.hpp"

#include "spi.h"

namespace {

constexpr uint8_t kWs2812LowLevel = 0xC0;
constexpr uint8_t kWs2812HighLevel = 0xF0;

} // namespace

namespace bsp {

#if BSP_WS2812_ENABLE

Status ws2812_set_rgb(uint8_t r, uint8_t g, uint8_t b) {
    uint8_t txbuf[24];
    uint8_t reset_byte = 0;

    for (int i = 0; i < 8; ++i) {
        txbuf[7 - i] = static_cast<uint8_t>((((g >> i) & 0x01U) ? kWs2812HighLevel : kWs2812LowLevel) >> 1);
        txbuf[15 - i] = static_cast<uint8_t>((((r >> i) & 0x01U) ? kWs2812HighLevel : kWs2812LowLevel) >> 1);
        txbuf[23 - i] = static_cast<uint8_t>((((b >> i) & 0x01U) ? kWs2812HighLevel : kWs2812LowLevel) >> 1);
    }

    HAL_SPI_Transmit(&hspi6, &reset_byte, 0, HAL_MAX_DELAY);
    while (hspi6.State != HAL_SPI_STATE_READY) {
    }
    if (HAL_SPI_Transmit(&hspi6, txbuf, 24, HAL_MAX_DELAY) != HAL_OK) {
        return Status::Error;
    }
    for (int i = 0; i < 100; ++i) {
        HAL_SPI_Transmit(&hspi6, &reset_byte, 1, HAL_MAX_DELAY);
    }
    return Status::Ok;
}

#endif

} // namespace bsp
