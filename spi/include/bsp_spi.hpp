#pragma once

#include "bsp.hpp"
#include "bsp_board.hpp"

#include <cstddef>
#include <cstdint>

namespace bsp {

enum class SpiPort : uint8_t {
    Spi2 = 2,
    Spi6 = 6,
};

enum class SpiMode : uint8_t {
    Block = 0,
    IT = 1,
    Dma = 2,
};

constexpr bool spi_is_enabled(SpiPort port) {
    switch (port) {
#if BSP_SPI_ENABLE_SPI2
        case SpiPort::Spi2:
            return true;
#endif
#if BSP_SPI_ENABLE_SPI6
        case SpiPort::Spi6:
            return true;
#endif
        default:
            return false;
    }
}

Status spi_transmit_impl(SpiPort port, const uint8_t* data, uint8_t len, SpiMode mode);
Status spi_receive_impl(SpiPort port, uint8_t* data, uint8_t len, SpiMode mode);

template <SpiPort Port>
Status spi_transmit(const uint8_t* data, uint8_t len, SpiMode mode = SpiMode::Block) {
    static_assert(spi_is_enabled(Port), "SPI port not enabled in pnx_configs/bsp_board.hpp");
    return spi_transmit_impl(Port, data, len, mode);
}

template <SpiPort Port>
Status spi_receive(uint8_t* data, uint8_t len, SpiMode mode = SpiMode::Block) {
    static_assert(spi_is_enabled(Port), "SPI port not enabled in pnx_configs/bsp_board.hpp");
    return spi_receive_impl(Port, data, len, mode);
}

} // namespace bsp
