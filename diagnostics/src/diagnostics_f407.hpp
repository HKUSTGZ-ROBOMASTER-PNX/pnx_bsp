#pragma once

#include "bsp_diagnostics.h"

#include <cstddef>
#include <cstdint>

namespace pnx::board::f407::diagnostics
{

inline constexpr std::uintptr_t sram_start = 0x20000000UL;
inline constexpr std::uintptr_t sram_end = 0x20020000UL;
inline constexpr std::uintptr_t ccmram_start = 0x10000000UL;
inline constexpr std::uintptr_t ccmram_end = 0x10010000UL;
inline constexpr std::size_t basic_exception_frame_size =
    8U * sizeof(std::uint32_t);
inline constexpr std::uint32_t stacking_error_mask =
    (1UL << 3U) | (1UL << 4U) | (1UL << 5U) |
    (1UL << 11U) | (1UL << 12U) | (1UL << 13U);

constexpr bool basic_frame_pointer_valid(
    std::uintptr_t address, std::uint32_t cfsr) noexcept
{
    if (address == 0U ||
        (address & (alignof(std::uint32_t) - 1U)) != 0U ||
        (cfsr & stacking_error_mask) != 0U)
    {
        return false;
    }
    const auto in_region =
        [address](std::uintptr_t begin,
                  std::uintptr_t end) constexpr noexcept {
            return address >= begin &&
                   address <= end - basic_exception_frame_size;
        };
    return in_region(sram_start, sram_end) ||
           in_region(ccmram_start, ccmram_end);
}

constexpr std::uint32_t
decode_reset_reasons(std::uint32_t raw_flags) noexcept
{
    std::uint32_t reasons = BSP_DIAGNOSTICS_RESET_NONE;
    if ((raw_flags & (1UL << 25U)) != 0U)
    {
        reasons |= BSP_DIAGNOSTICS_RESET_BROWNOUT;
    }
    if ((raw_flags & (1UL << 26U)) != 0U)
    {
        reasons |= BSP_DIAGNOSTICS_RESET_PIN;
    }
    if ((raw_flags & (1UL << 27U)) != 0U)
    {
        reasons |= BSP_DIAGNOSTICS_RESET_POWER_ON;
    }
    if ((raw_flags & (1UL << 28U)) != 0U)
    {
        reasons |= BSP_DIAGNOSTICS_RESET_SOFTWARE;
    }
    if ((raw_flags & (1UL << 29U)) != 0U)
    {
        reasons |= BSP_DIAGNOSTICS_RESET_IWDG;
    }
    if ((raw_flags & (1UL << 30U)) != 0U)
    {
        reasons |= BSP_DIAGNOSTICS_RESET_WWDG;
    }
    if ((raw_flags & (1UL << 31U)) != 0U)
    {
        reasons |= BSP_DIAGNOSTICS_RESET_LOW_POWER;
    }
    return reasons;
}

} // namespace pnx::board::f407::diagnostics
