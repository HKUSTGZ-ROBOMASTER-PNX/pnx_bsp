#include "bsp_spi.hpp"

#include "spi_devices.hpp"
#include "stm32f4xx_hal.h"

namespace
{

bool initialized = false;

bool selected_bus(bsp::spi::bus selected) noexcept
{
    return selected == board::spi::imu_bus;
}

GPIO_TypeDef* select_port(
    bsp::spi::chip_select selected) noexcept
{
    if (selected == board::spi::bmi088_accel)
    {
        return GPIOA;
    }
    if (selected == board::spi::bmi088_gyro)
    {
        return GPIOB;
    }
    return nullptr;
}

std::uint16_t select_pin(
    bsp::spi::chip_select selected) noexcept
{
    if (selected == board::spi::bmi088_accel)
    {
        return GPIO_PIN_4;
    }
    if (selected == board::spi::bmi088_gyro)
    {
        return GPIO_PIN_0;
    }
    return 0U;
}

types::status initialize_spi1() noexcept
{
    if (initialized)
    {
        return types::status::ok;
    }

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_SPI1_CLK_ENABLE();

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);

    GPIO_InitTypeDef select_gpio{};
    select_gpio.Mode = GPIO_MODE_OUTPUT_PP;
    select_gpio.Pull = GPIO_NOPULL;
    select_gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    select_gpio.Pin = GPIO_PIN_4;
    HAL_GPIO_Init(GPIOA, &select_gpio);
    select_gpio.Pin = GPIO_PIN_0;
    HAL_GPIO_Init(GPIOB, &select_gpio);

    GPIO_InitTypeDef spi_gpio{};
    spi_gpio.Mode = GPIO_MODE_AF_PP;
    spi_gpio.Pull = GPIO_PULLUP;
    spi_gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    spi_gpio.Alternate = GPIO_AF5_SPI1;
    spi_gpio.Pin = GPIO_PIN_7;
    HAL_GPIO_Init(GPIOA, &spi_gpio);
    spi_gpio.Pin = GPIO_PIN_3 | GPIO_PIN_4;
    HAL_GPIO_Init(GPIOB, &spi_gpio);

    SPI1->CR1 = 0U;
    SPI1->CR2 = 0U;
    SPI1->CRCPR = 7U;
    SPI1->CR1 = SPI_CR1_MSTR | SPI_CR1_SSM | SPI_CR1_SSI |
                SPI_CR1_BR_1 | SPI_CR1_CPOL | SPI_CR1_CPHA;
    SPI1->CR1 |= SPI_CR1_SPE;
    initialized = true;
    return types::status::ok;
}

bool wait_for(
    std::uint32_t mask, bool set,
    std::uint32_t started, std::uint32_t timeout_ms) noexcept
{
    while (((SPI1->SR & mask) != 0U) != set)
    {
        if ((HAL_GetTick() - started) >= timeout_ms)
        {
            return false;
        }
    }
    return true;
}

types::status transfer_byte(
    std::uint8_t transmitted, std::uint8_t& received,
    std::uint32_t started, std::uint32_t timeout_ms) noexcept
{
    if (!wait_for(SPI_SR_TXE, true, started, timeout_ms))
    {
        return types::status::error;
    }
    *reinterpret_cast<volatile std::uint8_t*>(&SPI1->DR) = transmitted;
    if (!wait_for(SPI_SR_RXNE, true, started, timeout_ms))
    {
        return types::status::error;
    }
    received = *reinterpret_cast<volatile std::uint8_t*>(&SPI1->DR);
    return types::status::ok;
}

} // namespace

namespace bsp::spi
{

bool bus_enabled(bus selected) noexcept
{
    return selected_bus(selected);
}

bool select_enabled(chip_select selected) noexcept
{
    return select_port(selected) != nullptr;
}

types::status init(bus selected) noexcept
{
    return selected_bus(selected)
               ? initialize_spi1()
               : types::status::not_configured;
}

types::status wait_ready(
    bus selected, std::uint32_t timeout_ms) noexcept
{
    if (!selected_bus(selected) || !initialized)
    {
        return types::status::not_configured;
    }
    const std::uint32_t started = HAL_GetTick();
    return wait_for(SPI_SR_BSY, false, started, timeout_ms)
               ? types::status::ok
               : types::status::error;
}

types::status transmit(
    bus selected, const std::uint8_t* data, std::size_t len,
    std::uint32_t timeout_ms) noexcept
{
    if (data == nullptr || len == 0U)
    {
        return types::status::invalid_arg;
    }
    if (!selected_bus(selected) || !initialized)
    {
        return types::status::not_configured;
    }
    const std::uint32_t started = HAL_GetTick();
    std::uint8_t discarded = 0U;
    for (std::size_t index = 0U; index < len; ++index)
    {
        const types::status status = transfer_byte(
            data[index], discarded, started, timeout_ms);
        if (status != types::status::ok)
        {
            return status;
        }
    }
    return wait_for(SPI_SR_BSY, false, started, timeout_ms)
               ? types::status::ok
               : types::status::error;
}

types::status receive(
    bus selected, std::uint8_t* data, std::size_t len,
    std::uint32_t timeout_ms) noexcept
{
    if (data == nullptr || len == 0U)
    {
        return types::status::invalid_arg;
    }
    if (!selected_bus(selected) || !initialized)
    {
        return types::status::not_configured;
    }
    const std::uint32_t started = HAL_GetTick();
    for (std::size_t index = 0U; index < len; ++index)
    {
        const types::status status = transfer_byte(
            0U, data[index], started, timeout_ms);
        if (status != types::status::ok)
        {
            return status;
        }
    }
    return wait_for(SPI_SR_BSY, false, started, timeout_ms)
               ? types::status::ok
               : types::status::error;
}

types::status set_select(
    chip_select selected, bool active) noexcept
{
    GPIO_TypeDef* const port = select_port(selected);
    const std::uint16_t pin = select_pin(selected);
    if (port == nullptr || pin == 0U)
    {
        return types::status::not_configured;
    }
    HAL_GPIO_WritePin(
        port, pin, active ? GPIO_PIN_RESET : GPIO_PIN_SET);
    return types::status::ok;
}

void cs_set(cs selected, bool active) noexcept
{
    (void)set_select(selected, active);
}

} // namespace bsp::spi
