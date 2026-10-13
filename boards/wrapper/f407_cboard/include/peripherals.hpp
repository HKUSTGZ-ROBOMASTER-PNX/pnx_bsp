#pragma once

#include "config.hpp"
#include "can.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"

// f407_cboard.ioc. These values describe CubeMX configuration; they do not init HAL.
namespace bsp::peripherals
{
// bxCAN supports classic frames. The IOC does not configure acceptance filters;
// standard is the wrapper default, not a CubeMX filter setup.
inline constexpr can::config can1{can::handle::can1, can::frame_mode::classic, can::id_type::standard};
inline constexpr can::config can2{can::handle::can2, can::frame_mode::classic, can::id_type::standard};

// DMA flags are {rx, tx}; SPI1_IRQn is not enabled in the IOC.
inline constexpr spi::config spi1{spi::handle::spi1, {true, true}, false};
inline constexpr usart::config usart1{usart::handle::usart1, {true, true}};
inline constexpr usart::config usart2{usart::handle::usart2, {true, true}};
inline constexpr usart::config usart3{usart::handle::usart3, {true, false}};
inline constexpr usart::config usart6{usart::handle::usart6, {true, true}};

// Timer kernel clocks before prescaling, not PWM output frequencies.
// APB2: RCC.APB2TimFreq_Value; APB1: RCC.APB1TimFreq_Value.
inline constexpr pwm::config tim1{168000000U, pwm::handle::tim1, 0x0FU}; // CH1/2/3/4
inline constexpr pwm::config tim4{84000000U, pwm::handle::tim4, 0x04U}; // CH3
inline constexpr pwm::config tim8{168000000U, pwm::handle::tim8, 0x07U}; // CH1/2/3
inline constexpr pwm::config tim10{168000000U, pwm::handle::tim10, 0x01U}; // CH1

// GPIO locations only. Initial levels do not determine logical active levels;
// active retains the type default (high) and must be chosen by the consumer.
inline constexpr gpio::config pa4{gpio::port::a, 4};
inline constexpr gpio::config pb0{gpio::port::b, 0};
inline constexpr gpio::config pc5{gpio::port::c, 5}; // EXTI5
inline constexpr gpio::config pc8{gpio::port::c, 8};
inline constexpr gpio::config pg6{gpio::port::g, 6};
inline constexpr gpio::config ph10{gpio::port::h, 10};
inline constexpr gpio::config ph11{gpio::port::h, 11};
inline constexpr gpio::config ph12{gpio::port::h, 12};

// Queries only resolve resources; callers retain responsibility for HAL operations.
[[nodiscard]] inline constexpr CAN_HandleTypeDef* can_handle(can::handle id) noexcept
{
    switch (id)
    {
        case can::handle::can1: return &hcan1;
        case can::handle::can2: return &hcan2;
        default: return nullptr;
    }
}

[[nodiscard]] inline constexpr SPI_HandleTypeDef* spi_handle(spi::handle id) noexcept
{
    switch (id)
    {
        case spi::handle::spi1: return &hspi1;
        default: return nullptr;
    }
}

[[nodiscard]] inline constexpr UART_HandleTypeDef* usart_handle(usart::handle id) noexcept
{
    switch (id)
    {
        case usart::handle::usart1: return &huart1;
        case usart::handle::usart2: return &huart2;
        case usart::handle::usart3: return &huart3;
        case usart::handle::usart6: return &huart6;
        default: return nullptr;
    }
}

[[nodiscard]] inline constexpr TIM_HandleTypeDef* tim_handle(pwm::handle id) noexcept
{
    switch (id)
    {
        case pwm::handle::tim1: return &htim1;
        case pwm::handle::tim4: return &htim4;
        case pwm::handle::tim8: return &htim8;
        case pwm::handle::tim10: return &htim10;
        default: return nullptr;
    }
}

// TIM_CHANNEL_1 is zero, so UINT32_MAX is the unsupported-channel sentinel.
[[nodiscard]] inline constexpr uint32_t pwm_channel(pwm::handle id, pwm::channel channel) noexcept
{
    const pwm::config* resource = nullptr;
    switch (id)
    {
        case pwm::handle::tim1: resource = &tim1; break;
        case pwm::handle::tim4: resource = &tim4; break;
        case pwm::handle::tim8: resource = &tim8; break;
        case pwm::handle::tim10: resource = &tim10; break;
        default: return UINT32_MAX;
    }
    const auto number = static_cast<uint8_t>(channel);
    if (number < 1 || number > 4 || (resource->channel_mask & (1U << (number - 1))) == 0)
        return UINT32_MAX;
    switch (channel)
    {
        case pwm::channel::ch1: return TIM_CHANNEL_1;
        case pwm::channel::ch2: return TIM_CHANNEL_2;
        case pwm::channel::ch3: return TIM_CHANNEL_3;
        case pwm::channel::ch4: return TIM_CHANNEL_4;
        default: return UINT32_MAX;
    }
}

[[nodiscard]] inline GPIO_TypeDef* gpio_port(gpio::port id) noexcept
{
    switch (id)
    {
        case gpio::port::a: return GPIOA;
        case gpio::port::b: return GPIOB;
        case gpio::port::c: return GPIOC;
        case gpio::port::d: return GPIOD;
        case gpio::port::e: return GPIOE;
        case gpio::port::f: return GPIOF;
        case gpio::port::g: return GPIOG;
        case gpio::port::h: return GPIOH;
        case gpio::port::i: return GPIOI;
        default: return nullptr;
    }
}

// Convert the configured pin number to HAL mask. Zero means invalid pin.
[[nodiscard]] inline constexpr uint16_t gpio_pin(uint8_t number) noexcept
{
    return number < 16 ? static_cast<uint16_t>(1U << number) : uint16_t{0};
}
// IOC can supply instances, DMA/IRQ, GPIO locations, PWM clocks/channels and ADC settings.
// GPIO active levels and acceptance-filter policy require manual device/driver decisions.
} // namespace bsp::peripherals

