#pragma once

#include "usertypes.hpp"

#include <cstdint>

namespace bsp::dwt
{

struct time
{
    std::uint32_t s = 0;
    std::uint32_t ms = 0;
    std::uint32_t us = 0;
};

types::status init();
bool initialized();

void update();
const time& now();

float timeline_s();
float timeline_ms();
std::uint64_t timeline_us();

float delta_s(std::uint32_t* last_cnt);
double delta_s64(std::uint32_t* last_cnt);

void delay_s(float seconds);
void delay_ms(std::uint32_t ms);
void delay_us(std::uint32_t us);

} // namespace bsp::dwt
