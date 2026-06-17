#include "bsp_spi.hpp"

#include "spi.h"

namespace {

SPI_HandleTypeDef* handle_for(bsp::SpiPort port) {
    switch (port) {
        case bsp::SpiPort::Spi2:
            return &hspi2;
        case bsp::SpiPort::Spi6:
            return &hspi6;
        default:
            return nullptr;
    }
}

} // namespace

namespace bsp {

Status spi_transmit_impl(SpiPort port, const uint8_t* data, uint8_t len, SpiMode mode) {
    if (!spi_is_enabled(port) || data == nullptr || len == 0) {
        return Status::NotConfigured;
    }

    SPI_HandleTypeDef* hspi = handle_for(port);
    if (hspi == nullptr) {
        return Status::InvalidArg;
    }

    switch (mode) {
        case SpiMode::Block:
            for (uint8_t i = 0; i < len; ++i) {
                uint8_t byte = data[i];
                if (HAL_SPI_Transmit(hspi, &byte, 1, 1000) != HAL_OK) {
                    return Status::Error;
                }
            }
            return Status::Ok;
        case SpiMode::IT:
            return HAL_SPI_Transmit_IT(hspi, const_cast<uint8_t*>(data), len) == HAL_OK ? Status::Ok
                                                                                        : Status::Error;
        case SpiMode::Dma:
            return HAL_SPI_Transmit_DMA(hspi, const_cast<uint8_t*>(data), len) == HAL_OK ? Status::Ok
                                                                                          : Status::Error;
        default:
            return Status::InvalidArg;
    }
}

Status spi_receive_impl(SpiPort port, uint8_t* data, uint8_t len, SpiMode mode) {
    if (!spi_is_enabled(port) || data == nullptr || len == 0) {
        return Status::NotConfigured;
    }

    SPI_HandleTypeDef* hspi = handle_for(port);
    if (hspi == nullptr) {
        return Status::InvalidArg;
    }

    switch (mode) {
        case SpiMode::Block:
            for (uint8_t i = 0; i < len; ++i) {
                uint8_t byte = 0;
                if (HAL_SPI_Receive(hspi, &byte, 1, 1000) != HAL_OK) {
                    return Status::Error;
                }
                data[i] = byte;
            }
            return Status::Ok;
        case SpiMode::IT:
            return HAL_SPI_Receive_IT(hspi, data, len) == HAL_OK ? Status::Ok : Status::Error;
        case SpiMode::Dma:
            return HAL_SPI_Receive_DMA(hspi, data, len) == HAL_OK ? Status::Ok : Status::Error;
        default:
            return Status::InvalidArg;
    }
}

} // namespace bsp
