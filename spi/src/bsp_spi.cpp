#include "bsp_spi.hpp"

namespace bsp::spi
{

namespace
{

const bus_config* config_of(std::size_t index) noexcept
{
    return index < bus_count ? &configs[index] : nullptr;
}

SPI_HandleTypeDef* handle_from_id(handle_id id) noexcept
{
    switch (id)
    {
    case handle_id::spi2:
        return &hspi2;
    case handle_id::spi6:
        return &hspi6;
    default:
        return nullptr;
    }
}

} // namespace

void cs_set(cs line, bool selected)
{
    const GPIO_PinState state = selected ? GPIO_PIN_RESET : GPIO_PIN_SET;
    switch (line)
    {
        case cs::bmi088_acc:
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, state);
            break;
        case cs::bmi088_gyro:
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, state);
            break;
    }
}

bool bus_enabled(bus bus) noexcept
{
    const bus_config* cfg = config_of(static_cast<std::size_t>(bus));
    return cfg != nullptr && cfg->enabled;
}

SPI_HandleTypeDef* handle_of(bus bus) noexcept
{
    const bus_config* cfg = config_of(static_cast<std::size_t>(bus));
    return cfg != nullptr && cfg->enabled ? handle_from_id(cfg->handle) : nullptr;
}

types::status init(bus bus)
{
    if (!bus_enabled(bus))
    {
        return types::status::not_configured;
    }

    switch (bus)
    {
        case bus::spi2:
            cs_set(cs::bmi088_acc, false);
            cs_set(cs::bmi088_gyro, false);
            return types::status::ok;
        case bus::spi6:
            return types::status::ok;
        default:
            return types::status::invalid_arg;
    }
}

types::status wait_ready(bus bus, uint32_t timeout_ms)
{
    if (!bus_enabled(bus))
    {
        return types::status::not_configured;
    }

    SPI_HandleTypeDef* handle = handle_of(bus);
    if (handle == nullptr)
    {
        return types::status::invalid_arg;
    }

    const uint32_t start = HAL_GetTick();
    while (handle->State != HAL_SPI_STATE_READY)
    {
        if ((HAL_GetTick() - start) > timeout_ms)
        {
            return types::status::error;
        }
    }

    return types::status::ok;
}

types::status transmit(bus bus, const uint8_t* data, size_t len, uint32_t timeout_ms)
{
    if (!bus_enabled(bus))
    {
        return types::status::not_configured;
    }

    SPI_HandleTypeDef* handle = handle_of(bus);
    if (handle == nullptr || data == nullptr || len == 0)
    {
        return types::status::invalid_arg;
    }

    if (HAL_SPI_Transmit(handle, const_cast<uint8_t*>(data), static_cast<uint16_t>(len),
                         timeout_ms) != HAL_OK)
    {
        return types::status::error;
    }

    return types::status::ok;
}

types::status receive(bus bus, uint8_t* data, size_t len, uint32_t timeout_ms)
{
    if (!bus_enabled(bus))
    {
        return types::status::not_configured;
    }

    SPI_HandleTypeDef* handle = handle_of(bus);
    if (handle == nullptr || data == nullptr || len == 0)
    {
        return types::status::invalid_arg;
    }

    if (HAL_SPI_Receive(handle, data, static_cast<uint16_t>(len), timeout_ms) != HAL_OK)
    {
        return types::status::error;
    }

    return types::status::ok;
}

} // namespace bsp::spi
