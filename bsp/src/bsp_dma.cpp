#include "bsp_dma.hpp"

#include <limits>

namespace bsp::dma
{
namespace
{

constexpr std::uintptr_t ram_d1_start = 0x24000000UL;
constexpr std::uintptr_t ram_d1_end = 0x24050000UL;
constexpr std::uintptr_t ram_d2_start = 0x30000000UL;
constexpr std::uintptr_t ram_d2_end = 0x30008000UL;

extern "C"
{
extern uint8_t _sdma_buffer;
extern uint8_t _edma_buffer;
}

bool in_range(std::uintptr_t address, std::uintptr_t start, std::uintptr_t end) noexcept
{
    return address >= start && address < end;
}

bool is_dma_accessible(std::uintptr_t start, std::uintptr_t end) noexcept
{
    return (in_range(start, ram_d1_start, ram_d1_end) && in_range(end - 1U, ram_d1_start, ram_d1_end)) ||
           (in_range(start, ram_d2_start, ram_d2_end) && in_range(end - 1U, ram_d2_start, ram_d2_end));
}

} // namespace

bool valid(buffer_view buffer) noexcept
{
    if (buffer.data() == nullptr || buffer.logical_size() == 0U ||
        buffer.logical_size() > buffer.capacity() ||
        (reinterpret_cast<std::uintptr_t>(buffer.data()) % cache_line_size) != 0U ||
        (buffer.capacity() % cache_line_size) != 0U)
    {
        return false;
    }

    const std::uintptr_t start = reinterpret_cast<std::uintptr_t>(buffer.data());
    if (start > std::numeric_limits<std::uintptr_t>::max() - buffer.capacity())
    {
        return false;
    }
    const std::uintptr_t end = start + buffer.capacity();
    if (!is_dma_accessible(start, end))
    {
        return false;
    }

    const std::uintptr_t dma_start = reinterpret_cast<std::uintptr_t>(&_sdma_buffer);
    const std::uintptr_t dma_end = reinterpret_cast<std::uintptr_t>(&_edma_buffer);
    return dma_start <= start && end <= dma_end;
}

} // namespace bsp::dma
