#pragma once

#include "bsp_pwm.hpp"

namespace board::pwm
{

// DJI C-board connector C2 signal: PE11 / TIM1_CH2.
inline constexpr bsp::pwm::channel servo_c2{0U};

} // namespace board::pwm
