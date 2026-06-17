#pragma once

#include "bsp.hpp"
#include "bsp_board.hpp"

#include <cstdint>

namespace bsp {

#if BSP_WS2812_ENABLE
Status ws2812_set_rgb(uint8_t r, uint8_t g, uint8_t b);
#else
inline Status ws2812_set_rgb(uint8_t, uint8_t, uint8_t) {
    return Status::NotConfigured;
}
#endif

} // namespace bsp
