#include "bsp_flash.hpp"

#include "stm32f4xx_hal.h"

#include <array>
#include <cstring>

namespace
{

struct sector
{
    std::uint32_t address;
    std::uint32_t id;
};

constexpr std::array<sector, 12U> sectors{{
    {0x08000000U, FLASH_SECTOR_0},
    {0x08004000U, FLASH_SECTOR_1},
    {0x08008000U, FLASH_SECTOR_2},
    {0x0800C000U, FLASH_SECTOR_3},
    {0x08010000U, FLASH_SECTOR_4},
    {0x08020000U, FLASH_SECTOR_5},
    {0x08040000U, FLASH_SECTOR_6},
    {0x08060000U, FLASH_SECTOR_7},
    {0x08080000U, FLASH_SECTOR_8},
    {0x080A0000U, FLASH_SECTOR_9},
    {0x080C0000U, FLASH_SECTOR_10},
    {0x080E0000U, FLASH_SECTOR_11},
}};

bool sector_at(std::uint32_t address, std::uint32_t& id) noexcept
{
    for (const sector& entry : sectors)
    {
        if (entry.address == address)
        {
            id = entry.id;
            return true;
        }
    }
    return false;
}

types::status finish_flash(HAL_StatusTypeDef status) noexcept
{
    const HAL_StatusTypeDef lock_status = HAL_FLASH_Lock();
    if (status != HAL_OK || lock_status != HAL_OK)
    {
        return types::status::error;
    }
    return types::status::ok;
}

} // namespace

namespace bsp::flash
{

geometry layout() noexcept
{
    return {0x08000000U, 0x08100000U, 4U};
}

types::status erase_block(std::uint32_t address) noexcept
{
    const geometry current = layout();
    if (current.begin >= current.end ||
        address < current.begin || address >= current.end)
    {
        return types::status::invalid_arg;
    }

    std::uint32_t sector_id = 0U;
    if (!sector_at(address, sector_id))
    {
        return types::status::invalid_arg;
    }

    FLASH_EraseInitTypeDef erase{};
    erase.TypeErase = FLASH_TYPEERASE_SECTORS;
    erase.VoltageRange = FLASH_VOLTAGE_RANGE_3;
    erase.Sector = sector_id;
    erase.NbSectors = 1U;
    std::uint32_t failed_sector = 0U;

    if (HAL_FLASH_Unlock() != HAL_OK)
    {
        return types::status::error;
    }
    return finish_flash(
        HAL_FLASHEx_Erase(&erase, &failed_sector));
}

types::status program(
    std::uint32_t address, const void* source, std::size_t len) noexcept
{
    const geometry current = layout();
    if (source == nullptr || len == 0U ||
        current.begin >= current.end ||
        current.program_alignment == 0U ||
        address < current.begin || address >= current.end ||
        (address % current.program_alignment) != 0U ||
        (len % current.program_alignment) != 0U ||
        len > static_cast<std::size_t>(current.end - address))
    {
        return types::status::invalid_arg;
    }
    const auto* data = static_cast<const std::uint8_t*>(source);
    if (HAL_FLASH_Unlock() != HAL_OK)
    {
        return types::status::error;
    }

    HAL_StatusTypeDef status = HAL_OK;
    for (std::size_t offset = 0U; offset < len; offset += 4U)
    {
        std::uint32_t word = 0U;
        std::memcpy(&word, data + offset, sizeof(word));
        status = HAL_FLASH_Program(
            FLASH_TYPEPROGRAM_WORD, address + offset, word);
        if (status != HAL_OK)
        {
            break;
        }
    }
    return finish_flash(status);
}

} // namespace bsp::flash
