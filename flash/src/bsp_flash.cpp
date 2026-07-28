#include "bsp_flash.hpp"

namespace bsp::flash
{

geometry layout() noexcept
{
    return detail::backend_geometry();
}

types::status erase_block(std::uint32_t address) noexcept
{
    const geometry current = layout();
    if (current.begin >= current.end ||
        address < current.begin || address >= current.end)
    {
        return types::status::invalid_arg;
    }
    return detail::backend_erase_block(address);
}

types::status program(
    std::uint32_t address, const void* data, std::size_t len) noexcept
{
    const geometry current = layout();
    if (data == nullptr || len == 0U ||
        current.begin >= current.end ||
        current.program_alignment == 0U ||
        address < current.begin || address >= current.end ||
        (address % current.program_alignment) != 0U ||
        (len % current.program_alignment) != 0U ||
        len > static_cast<std::size_t>(current.end - address))
    {
        return types::status::invalid_arg;
    }
    return detail::backend_program(
        address, static_cast<const std::uint8_t*>(data), len);
}

} // namespace bsp::flash
