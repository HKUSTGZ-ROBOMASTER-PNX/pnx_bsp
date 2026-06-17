#pragma once

#include "bsp.hpp"

#include <cstdint>

namespace bsp {

struct DwtTime {
    uint32_t s;
    uint16_t ms;
    uint16_t us;
};

void dwt_init(uint32_t cpu_freq_mhz);
float dwt_delta_s(uint32_t* last_cnt);
double dwt_delta_s64(uint32_t* last_cnt);
float dwt_timeline_s();
float dwt_timeline_ms();
uint64_t dwt_timeline_us();
void dwt_delay_s(float seconds);
void dwt_update();

} // namespace bsp
