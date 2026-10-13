#pragma once

// Configuration types for peripherals enabled in f407_cboard.ioc; ADC is not enabled.

#include <cstdint>

namespace bsp
{

enum class active_level : uint8_t
{
    low,
    high
};

struct dma_config
{
    bool rx = false;
    bool tx = false;
};

namespace can
{
    enum class handle : uint8_t
    {
        none,
        can1,
        can2
    };

    enum class frame_mode : uint8_t
    {
        classic
    };

    enum class id_type : uint8_t
    {
        standard,
        extended
    };

    struct config
    {
        handle instance = handle::none;
        frame_mode mode = frame_mode::classic;
        id_type filter = id_type::standard;
    };
}

namespace spi
{
    enum class handle : uint8_t
    {
        none,
        spi1
    };

    struct config
    {
        handle instance = handle::none;
        dma_config dma{};
        bool irq = false;
    };
}

namespace gpio
{
    enum class port : uint8_t
    {
        none,
        a, b, c, d, e, f, g, h, i
    };

    struct config
    {
        port instance = port::none;
        uint8_t pin = 0; // Pin number 0..15, not a HAL bit mask.
        active_level active = active_level::high;
    };
}

namespace pwm
{
    enum class handle : uint8_t { none, tim1, tim4, tim8, tim10 };
    enum class channel : uint8_t { none, ch1, ch2, ch3, ch4 };

    struct config
    {
        uint32_t timer_clock_hz = 0; // Kernel clock before the timer prescaler.
        handle instance = handle::none;
        uint8_t channel_mask = 0; // Bits 0..3 describe enabled CH1..CH4.
    };
}

namespace usart
{
    enum class handle : uint8_t
    {
        none,
        usart1,
        usart2,
        usart3,
        usart6
    };

    struct config
    {
        handle instance = handle::none;
        dma_config dma{};
    };
}

} // namespace bsp
