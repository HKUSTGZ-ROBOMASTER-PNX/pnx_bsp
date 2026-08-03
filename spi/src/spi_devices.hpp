#pragma once

#include "bsp_spi.hpp"

namespace board::spi
{

inline constexpr bsp::spi::bus imu_bus{0U};
inline constexpr bsp::spi::chip_select bmi088_accel{0U};
inline constexpr bsp::spi::chip_select bmi088_gyro{1U};

} // namespace board::spi
