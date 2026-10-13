#pragma once

#include "config.hpp"
#include "adc.h"
#include "fdcan.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"

// h723_mc02.ioc. These values describe CubeMX configuration; they do not init HAL.
namespace bsp::peripherals
{
// FDCAN1 has standard filters and FD+BRS; FDCAN2/3 use classic frames.
// TDC is not enabled by CubeMX. Timing and FIFO sizes remain in fdcan.c.
inline constexpr can::config fdcan1{can::handle::fdcan1, can::frame_mode::fd_brs, can::id_type::standard};
inline constexpr can::config fdcan2{can::handle::fdcan2, can::frame_mode::classic, can::id_type::standard};
inline constexpr can::config fdcan3{can::handle::fdcan3, can::frame_mode::classic, can::id_type::standard};

// DMA flags are {rx, tx}; irq is the SPI peripheral NVIC enable.
inline constexpr spi::config spi2{spi::handle::spi2, {true, true}, true};
inline constexpr spi::config spi6{spi::handle::spi6, {false, false}, false};
inline constexpr usart::config uart5{usart::handle::uart5, {true, false}};
inline constexpr usart::config uart7{usart::handle::uart7, {true, true}};
inline constexpr usart::config usart1{usart::handle::usart1, {true, true}};
inline constexpr usart::config usart10{usart::handle::usart10, {true, false}};

inline constexpr adc::config adc1{4, 1, adc::sampling_time::cycles_1_5, adc::handle::adc1};

// Timer kernel clocks before prescaling, not PWM output frequencies.
// APB1 timers: RCC.Tim2OutputFreq_Value=260000000.
inline constexpr pwm::config tim3{260000000U, pwm::handle::tim3, 0x08U}; // CH4
inline constexpr pwm::config tim12{260000000U, pwm::handle::tim12, 0x02U}; // CH2

// GPIO locations only. The IOC gives initial levels, not logical active levels;
// active retains the type default (high) and must be chosen by the consumer.
inline constexpr gpio::config ps2_cs{gpio::port::a, 0};
inline constexpr gpio::config ps2_clk{gpio::port::a, 2};
inline constexpr gpio::config acc_cs{gpio::port::c, 0};
inline constexpr gpio::config gyro_cs{gpio::port::c, 3};
inline constexpr gpio::config pc13{gpio::port::c, 13};
inline constexpr gpio::config pc14{gpio::port::c, 14};
inline constexpr gpio::config pc15{gpio::port::c, 15};
inline constexpr gpio::config ps2_data{gpio::port::e, 9};
inline constexpr gpio::config acc_int{gpio::port::e, 10};
inline constexpr gpio::config gyro_int{gpio::port::e, 12};
inline constexpr gpio::config ps2_cmd{gpio::port::e, 13};

// Queries only resolve resources; callers retain responsibility for HAL operations.
[[nodiscard]] inline constexpr FDCAN_HandleTypeDef* fdcan_handle(can::handle id) noexcept
{
    switch (id)
    {
        case can::handle::fdcan1: return &hfdcan1;
        case can::handle::fdcan2: return &hfdcan2;
        case can::handle::fdcan3: return &hfdcan3;
        default: return nullptr;
    }
}

[[nodiscard]] inline constexpr SPI_HandleTypeDef* spi_handle(spi::handle id) noexcept
{
    switch (id)
    {
        case spi::handle::spi2: return &hspi2;
        case spi::handle::spi6: return &hspi6;
        default: return nullptr;
    }
}

[[nodiscard]] inline constexpr UART_HandleTypeDef* usart_handle(usart::handle id) noexcept
{
    switch (id)
    {
        case usart::handle::uart5: return &huart5;
        case usart::handle::uart7: return &huart7;
        case usart::handle::usart1: return &huart1;
        case usart::handle::usart10: return &huart10;
        default: return nullptr;
    }
}

[[nodiscard]] inline constexpr TIM_HandleTypeDef* tim_handle(pwm::handle id) noexcept
{
    switch (id)
    {
        case pwm::handle::tim3: return &htim3;
        case pwm::handle::tim12: return &htim12;
        default: return nullptr;
    }
}

// TIM_CHANNEL_1 is zero, so UINT32_MAX is the unsupported-channel sentinel.
[[nodiscard]] inline constexpr uint32_t pwm_channel(pwm::handle id, pwm::channel channel) noexcept
{
    const pwm::config* resource = nullptr;
    switch (id)
    {
        case pwm::handle::tim3: resource = &tim3; break;
        case pwm::handle::tim12: resource = &tim12; break;
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
        case gpio::port::h: return GPIOH;
        default: return nullptr;
    }
}

// Convert the configured pin number to HAL mask. Zero means invalid pin.
[[nodiscard]] inline constexpr uint16_t gpio_pin(uint8_t number) noexcept
{
    return number < 16 ? static_cast<uint16_t>(1U << number) : uint16_t{0};
}
[[nodiscard]] inline constexpr ADC_HandleTypeDef* adc_handle(adc::handle id) noexcept
{
    return id == adc::handle::adc1 ? &hadc1 : nullptr;
}

// Only channel 4 / rank 1 is configured by this IOC. Other IDs are unsupported.
[[nodiscard]] inline constexpr uint32_t adc_channel(uint8_t number) noexcept
{
    return number == adc1.channel ? ADC_CHANNEL_4 : UINT32_MAX;
}

[[nodiscard]] inline constexpr uint32_t adc_rank(uint8_t number) noexcept
{
    return number == adc1.rank ? ADC_REGULAR_RANK_1 : UINT32_MAX;
}

// These sampling times are valid for H723 ADC1/2; do not reuse for ADC3 or F407.
[[nodiscard]] inline constexpr uint32_t adc_sampling_time(adc::sampling_time sampling) noexcept
{
    switch (sampling)
    {
        case adc::sampling_time::cycles_1_5: return ADC_SAMPLETIME_1CYCLE_5;
        case adc::sampling_time::cycles_2_5: return ADC_SAMPLETIME_2CYCLES_5;
        case adc::sampling_time::cycles_8_5: return ADC_SAMPLETIME_8CYCLES_5;
        case adc::sampling_time::cycles_16_5: return ADC_SAMPLETIME_16CYCLES_5;
        case adc::sampling_time::cycles_32_5: return ADC_SAMPLETIME_32CYCLES_5;
        case adc::sampling_time::cycles_64_5: return ADC_SAMPLETIME_64CYCLES_5;
        case adc::sampling_time::cycles_387_5: return ADC_SAMPLETIME_387CYCLES_5;
        case adc::sampling_time::cycles_810_5: return ADC_SAMPLETIME_810CYCLES_5;
        default: return UINT32_MAX;
    }
}

// IOC can supply instances, DMA/IRQ, GPIO locations, PWM clocks/channels and ADC settings.
// GPIO active levels and acceptance-filter policy require manual device/driver decisions.
} // namespace bsp::peripherals

