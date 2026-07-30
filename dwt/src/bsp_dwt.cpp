#include "bsp_dwt.hpp"

#include "pnx_cmsis_device.h"

namespace
{

struct dwt_state
{
    bsp::dwt::time sys_time{};
    std::uint32_t cpu_freq_hz = 0;
    std::uint32_t cycles_per_ms = 0;
    std::uint32_t cycles_per_us = 0;
    std::uint32_t overflow_count = 0;
    std::uint32_t last_cnt = 0;
    std::uint64_t cnt64 = 0;
    bool initialized = false;
};

dwt_state state;

void update_overflow()
{
    static volatile std::uint8_t lock = 0;
    if (lock != 0U)
    {
        return;
    }
    lock = 1U;

    const std::uint32_t cnt_now = DWT->CYCCNT;
    if (cnt_now < state.last_cnt)
    {
        ++state.overflow_count;
    }
    state.last_cnt = cnt_now;

    lock = 0U;
}

void delay_cycles(std::uint64_t cycles)
{
    while (cycles > 0U)
    {
        const std::uint32_t chunk =
            cycles > 0x7FFFFFFFUL ? 0x7FFFFFFFUL : static_cast<std::uint32_t>(cycles);
        const std::uint32_t start = DWT->CYCCNT;
        while ((DWT->CYCCNT - start) < chunk)
        {
        }
        cycles -= chunk;
        update_overflow();
    }
}

} // namespace

namespace bsp::dwt
{

types::status init()
{
    SystemCoreClockUpdate();
    if (SystemCoreClock == 0U)
    {
        return types::status::invalid_arg;
    }

    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    state.sys_time = {};
    state.cpu_freq_hz = SystemCoreClock;
    state.cycles_per_ms = SystemCoreClock / 1000U;
    state.cycles_per_us = SystemCoreClock / 1'000'000U;
    state.overflow_count = 0U;
    state.last_cnt = 0U;
    state.cnt64 = 0U;
    state.initialized = true;
    update_overflow();

    return types::status::ok;
}

bool initialized()
{
    return state.initialized;
}

void update()
{
    if (!state.initialized)
    {
        return;
    }

    update_overflow();

    const std::uint32_t cnt_now = DWT->CYCCNT;
    state.cnt64 = (static_cast<std::uint64_t>(state.overflow_count) << 32U) + cnt_now;

    const std::uint64_t seconds = state.cnt64 / state.cpu_freq_hz;
    std::uint64_t remain = state.cnt64 - seconds * state.cpu_freq_hz;
    const std::uint64_t milliseconds = remain / state.cycles_per_ms;
    remain -= milliseconds * state.cycles_per_ms;

    state.sys_time.s = static_cast<std::uint32_t>(seconds);
    state.sys_time.ms = static_cast<std::uint32_t>(milliseconds);
    state.sys_time.us = static_cast<std::uint32_t>(remain / state.cycles_per_us);
}

const time& now()
{
    update();
    return state.sys_time;
}

float timeline_s()
{
    update();
    return static_cast<float>(state.sys_time.s) + static_cast<float>(state.sys_time.ms) * 0.001f +
           static_cast<float>(state.sys_time.us) * 0.000001f;
}

float timeline_ms()
{
    update();
    return static_cast<float>(state.sys_time.s) * 1000.0f +
           static_cast<float>(state.sys_time.ms) +
           static_cast<float>(state.sys_time.us) * 0.001f;
}

std::uint64_t timeline_us()
{
    update();
    return static_cast<std::uint64_t>(state.sys_time.s) * 1'000'000ULL +
           static_cast<std::uint64_t>(state.sys_time.ms) * 1000ULL + state.sys_time.us;
}

float delta_s(std::uint32_t* last_cnt)
{
    if (last_cnt == nullptr || !state.initialized)
    {
        return 0.0f;
    }

    const std::uint32_t cnt_now = DWT->CYCCNT;
    const float dt =
        static_cast<float>(cnt_now - *last_cnt) / static_cast<float>(state.cpu_freq_hz);
    *last_cnt = cnt_now;
    update_overflow();
    return dt;
}

double delta_s64(std::uint32_t* last_cnt)
{
    if (last_cnt == nullptr || !state.initialized)
    {
        return 0.0;
    }

    const std::uint32_t cnt_now = DWT->CYCCNT;
    const double dt =
        static_cast<double>(cnt_now - *last_cnt) / static_cast<double>(state.cpu_freq_hz);
    *last_cnt = cnt_now;
    update_overflow();
    return dt;
}

void delay_s(float seconds)
{
    if (seconds <= 0.0f || !state.initialized)
    {
        return;
    }

    delay_cycles(static_cast<std::uint64_t>(seconds * static_cast<float>(state.cpu_freq_hz)));
}

void delay_ms(std::uint32_t ms)
{
    if (ms == 0U || !state.initialized)
    {
        return;
    }

    delay_cycles(static_cast<std::uint64_t>(ms) * state.cycles_per_ms);
}

void delay_us(std::uint32_t us)
{
    if (us == 0U || !state.initialized)
    {
        return;
    }

    delay_cycles(static_cast<std::uint64_t>(us) * state.cycles_per_us);
}

} // namespace bsp::dwt
