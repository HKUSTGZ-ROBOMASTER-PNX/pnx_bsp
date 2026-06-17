#include "bsp_iic.hpp"

namespace bsp {

Status i2c_init_impl(I2cPort port, uint32_t clock_hz) {
    (void)clock_hz;
    if (!i2c_is_enabled(port)) {
        return Status::NotConfigured;
    }
    // 当前板级未配置 I2C 外设；启用 BSP_I2C_ENABLE_* 后在此补充 HAL 映射。
    return Status::NotConfigured;
}

Status i2c_write(I2cPort port, uint16_t dev_addr_7bit, const uint8_t* data, size_t len,
                 uint32_t timeout_ms) {
    (void)dev_addr_7bit;
    (void)data;
    (void)len;
    (void)timeout_ms;
    return i2c_is_enabled(port) ? Status::Error : Status::NotConfigured;
}

Status i2c_read(I2cPort port, uint16_t dev_addr_7bit, uint8_t* data, size_t len,
                uint32_t timeout_ms) {
    (void)dev_addr_7bit;
    (void)data;
    (void)len;
    (void)timeout_ms;
    return i2c_is_enabled(port) ? Status::Error : Status::NotConfigured;
}

Status i2c_mem_write(I2cPort port, uint16_t dev_addr_7bit, uint16_t mem_addr,
                     uint16_t mem_addr_size, const uint8_t* data, size_t len,
                     uint32_t timeout_ms) {
    (void)dev_addr_7bit;
    (void)mem_addr;
    (void)mem_addr_size;
    (void)data;
    (void)len;
    (void)timeout_ms;
    return i2c_is_enabled(port) ? Status::Error : Status::NotConfigured;
}

Status i2c_mem_read(I2cPort port, uint16_t dev_addr_7bit, uint16_t mem_addr,
                    uint16_t mem_addr_size, uint8_t* data, size_t len, uint32_t timeout_ms) {
    (void)dev_addr_7bit;
    (void)mem_addr;
    (void)mem_addr_size;
    (void)data;
    (void)len;
    (void)timeout_ms;
    return i2c_is_enabled(port) ? Status::Error : Status::NotConfigured;
}

} // namespace bsp
