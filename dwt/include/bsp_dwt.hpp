#pragma once

#include "main.h"
#include "stdint.h"

#include "status.hpp"

namespace bsp::dwt
{

struct dwt_state
{
    uint32_t cpu_freq_hz = 0;
    uint32_t last_cnt = 0;
    uint64_t cycles = 0;
    bool initialized = false;
};

inline dwt_state state{};

inline pnxtypes::status init() noexcept
{
    if (state.initialized)
    {
        return pnxtypes::status::ok;
    }
    SystemCoreClockUpdate();
    if (SystemCoreClock == 0)
    {
        return pnxtypes::status::invalid_arg;
    }

    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;

    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    auto& s = state;

    s.cpu_freq_hz = SystemCoreClock;
    s.last_cnt = 0;
    s.cycles = 0;
    s.initialized = true;

    return pnxtypes::status::ok;
}

[[nodiscard]] inline uint32_t cycles32() noexcept
{
    return DWT->CYCCNT;
}

[[nodiscard]] inline uint64_t cycles64() noexcept
{
    auto& s = state;

    const uint32_t mask = __get_PRIMASK();
    __disable_irq();

    const uint32_t now = DWT->CYCCNT;
    s.cycles += static_cast<uint32_t>(now - s.last_cnt);
    s.last_cnt = now;

    const uint64_t result = s.cycles;

    __set_PRIMASK(mask);

    return result;
}

[[nodiscard]] inline uint64_t timeline_us() noexcept
{
    const auto& s = state;

    if (!s.initialized)
    {
        return 0;
    }

    const uint64_t cycles = cycles64();

    return (cycles / s.cpu_freq_hz) * 1'000'000ULL +
           ((cycles % s.cpu_freq_hz) * 1'000'000ULL) / s.cpu_freq_hz;
}

inline void wait_cycles(uint64_t cycles) noexcept
{
    while (cycles != 0)
    {
        const uint32_t part = cycles > 0x7FFFFFFFU ? 0x7FFFFFFFU : static_cast<uint32_t>(cycles);
        const uint32_t start = DWT->CYCCNT;
        while (static_cast<uint32_t>(DWT->CYCCNT - start) < part)
        {
        }
        cycles -= part;
        // Helps maintain the extended clock during long busy waits.
        (void)cycles64();
    }
}

[[nodiscard]] inline float delta_s(uint32_t& last) noexcept
{
    if (!state.initialized)
    {
        return 0.0f;
    }
    const uint32_t now = static_cast<uint32_t>(cycles64());
    const uint32_t elapsed = now - last;

    last = now;

    return static_cast<float>(elapsed) / static_cast<float>(state.cpu_freq_hz);
}

inline void delay_us(uint32_t us) noexcept
{
    if (!state.initialized || us == 0)
    {
        return;
    }
    wait_cycles(static_cast<uint64_t>(us) * state.cpu_freq_hz / 1'000'000ULL);
}

inline void delay_ms(uint32_t ms) noexcept
{
    if (!state.initialized || ms == 0)
    {
        return;
    }
    wait_cycles(static_cast<uint64_t>(ms) * state.cpu_freq_hz / 1000ULL);
}

} // namespace bsp::dwt
