#pragma once

// PWM validation closure 的舵機通道映射；實體腳位仍以 IOC 與 board README 為準。
// 四個通道共用 TIM1 時基（單一 ARR）→ 週期是 timer 層級屬性，脈寬才是每通道獨立。
// C2 是 board README 已記錄的接口標籤；其餘三路的接口標籤未記錄，故以腳位命名。

#include "bsp_pwm.hpp"

#include <cstddef>

namespace board::pwm
{

// DJI C-board connector C2 signal: PE11 / TIM1_CH2.
inline constexpr bsp::pwm::channel servo_c2{0U};

// 同一 TIM1 時基上新增的舵機通道；IOC 未占用這些腳位，TIM1 保持單一擁有者。
inline constexpr bsp::pwm::channel servo_pe9{1U};  // TIM1_CH1
inline constexpr bsp::pwm::channel servo_pe13{2U}; // TIM1_CH3
inline constexpr bsp::pwm::channel servo_pe14{3U}; // TIM1_CH4

// 供 host fake 與契約測試對齊的邏輯通道數量。
inline constexpr std::size_t servo_channel_count = 4U;

} // namespace board::pwm
