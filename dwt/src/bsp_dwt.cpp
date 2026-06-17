#include "bsp_dwt.hpp"

#include "stm32h7xx.h"

namespace {

struct DwtState {
    bsp::DwtTime sys_time{};
    uint32_t cpu_freq_hz = 0;
    uint32_t cpu_freq_hz_ms = 0;
    uint32_t cpu_freq_hz_us = 0;
    uint32_t round_count = 0;
    uint32_t last_cnt = 0;
    uint64_t cnt64 = 0;
};

DwtState g_dwt;

void dwt_cnt_update() {
    static volatile uint8_t locker = 0;
    if (locker != 0U) {
        return;
    }
    locker = 1U;
    const volatile uint32_t cnt_now = DWT->CYCCNT;
    if (cnt_now < g_dwt.last_cnt) {
        ++g_dwt.round_count;
    }
    g_dwt.last_cnt = DWT->CYCCNT;
    locker = 0U;
}

} // namespace

namespace bsp {

void dwt_init(uint32_t cpu_freq_mhz) {
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    g_dwt.cpu_freq_hz = cpu_freq_mhz * 1'000'000U;
    g_dwt.cpu_freq_hz_ms = g_dwt.cpu_freq_hz / 1000U;
    g_dwt.cpu_freq_hz_us = g_dwt.cpu_freq_hz / 1'000'000U;
    g_dwt.round_count = 0U;
    dwt_cnt_update();
}

float dwt_delta_s(uint32_t* last_cnt) {
    const volatile uint32_t cnt_now = DWT->CYCCNT;
    const float dt = static_cast<float>(cnt_now - *last_cnt) / static_cast<float>(g_dwt.cpu_freq_hz);
    *last_cnt = cnt_now;
    dwt_cnt_update();
    return dt;
}

double dwt_delta_s64(uint32_t* last_cnt) {
    const volatile uint32_t cnt_now = DWT->CYCCNT;
    const double dt = static_cast<double>(cnt_now - *last_cnt) / static_cast<double>(g_dwt.cpu_freq_hz);
    *last_cnt = cnt_now;
    dwt_cnt_update();
    return dt;
}

void dwt_update() {
    const volatile uint32_t cnt_now = DWT->CYCCNT;
    uint64_t temp_s = 0;
    uint64_t temp_remain = 0;
    uint64_t temp_us = 0;

    dwt_cnt_update();

    g_dwt.cnt64 = static_cast<uint64_t>(g_dwt.round_count) * static_cast<uint64_t>(UINT32_MAX) +
                  static_cast<uint64_t>(cnt_now);
    temp_s = g_dwt.cnt64 / g_dwt.cpu_freq_hz;
    temp_remain = g_dwt.cnt64 - temp_s * g_dwt.cpu_freq_hz;
    g_dwt.sys_time.s = static_cast<uint32_t>(temp_s);
    g_dwt.sys_time.ms = static_cast<uint16_t>(temp_remain / g_dwt.cpu_freq_hz_ms);
    temp_us = temp_remain - static_cast<uint64_t>(g_dwt.sys_time.ms) * g_dwt.cpu_freq_hz_ms;
    g_dwt.sys_time.us = static_cast<uint16_t>(temp_us / g_dwt.cpu_freq_hz_us);
}

float dwt_timeline_s() {
    dwt_update();
    return static_cast<float>(g_dwt.sys_time.s) + static_cast<float>(g_dwt.sys_time.ms) * 0.001f +
           static_cast<float>(g_dwt.sys_time.us) * 0.000001f;
}

float dwt_timeline_ms() {
    dwt_update();
    return static_cast<float>(g_dwt.sys_time.s) * 1000.0f + static_cast<float>(g_dwt.sys_time.ms) +
           static_cast<float>(g_dwt.sys_time.us) * 0.001f;
}

uint64_t dwt_timeline_us() {
    dwt_update();
    return static_cast<uint64_t>(g_dwt.sys_time.s) * 1'000'000ULL +
           static_cast<uint64_t>(g_dwt.sys_time.ms) * 1000ULL + g_dwt.sys_time.us;
}

void dwt_delay_s(float seconds) {
    const uint32_t tick_start = DWT->CYCCNT;
    const float wait_ticks = seconds * static_cast<float>(g_dwt.cpu_freq_hz);
    while ((DWT->CYCCNT - tick_start) < wait_ticks) {
    }
}

} // namespace bsp
