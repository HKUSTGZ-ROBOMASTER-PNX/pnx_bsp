#pragma once

#include "bsp.hpp"
#include "bsp_board.hpp"

#include <cstddef>
#include <cstdint>

namespace bsp {

enum class I2cPort : uint8_t {
    I2c1 = 1,
    I2c2 = 2,
    I2c3 = 3,
    I2c4 = 4,
};

constexpr bool i2c_is_enabled(I2cPort port) {
    switch (port) {
#if BSP_I2C_ENABLE_I2C1
        case I2cPort::I2c1:
            return true;
#endif
#if BSP_I2C_ENABLE_I2C2
        case I2cPort::I2c2:
            return true;
#endif
#if BSP_I2C_ENABLE_I2C3
        case I2cPort::I2c3:
            return true;
#endif
#if BSP_I2C_ENABLE_I2C4
        case I2cPort::I2c4:
            return true;
#endif
        default:
            return false;
    }
}

Status i2c_init_impl(I2cPort port, uint32_t clock_hz);
Status i2c_write(I2cPort port, uint16_t dev_addr_7bit, const uint8_t* data, size_t len,
                 uint32_t timeout_ms);
Status i2c_read(I2cPort port, uint16_t dev_addr_7bit, uint8_t* data, size_t len,
                uint32_t timeout_ms);
Status i2c_mem_write(I2cPort port, uint16_t dev_addr_7bit, uint16_t mem_addr, uint16_t mem_addr_size,
                     const uint8_t* data, size_t len, uint32_t timeout_ms);
Status i2c_mem_read(I2cPort port, uint16_t dev_addr_7bit, uint16_t mem_addr, uint16_t mem_addr_size,
                    uint8_t* data, size_t len, uint32_t timeout_ms);

template <I2cPort Port>
Status i2c_init(uint32_t clock_hz = 400'000) {
    static_assert(i2c_is_enabled(Port), "I2C port not enabled in pnx_configs/bsp_board.hpp");
    return i2c_init_impl(Port, clock_hz);
}

} // namespace bsp
