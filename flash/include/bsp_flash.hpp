#pragma once

#include "usertypes.hpp"

#include <cstddef>
#include <cstdint>

namespace bsp::flash
{

struct geometry
{
    std::uint32_t begin = 0U;
    std::uint32_t end = 0U;
    std::uint32_t program_alignment = 0U;
};

geometry layout() noexcept;
types::status erase_block(std::uint32_t address) noexcept;
types::status program(
    std::uint32_t address, const void* data, std::size_t len) noexcept;

} // namespace bsp::flash
