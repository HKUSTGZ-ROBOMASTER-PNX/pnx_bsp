#include "bsp_flash.hpp"

#include "stm32h7xx_hal.h"

namespace
{

constexpr uint32_t flash_bank_2 = 0x02U;

uint32_t flash_sector_of(uint32_t addr, uint32_t* bank)
{
    if (addr < 0x08100000UL)
    {
        *bank = FLASH_BANK_1;
        if (addr < 0x08020000UL) return FLASH_SECTOR_0;
        if (addr < 0x08040000UL) return FLASH_SECTOR_1;
        if (addr < 0x08060000UL) return FLASH_SECTOR_2;
        if (addr < 0x08080000UL) return FLASH_SECTOR_3;
        if (addr < 0x080A0000UL) return FLASH_SECTOR_4;
        if (addr < 0x080C0000UL) return FLASH_SECTOR_5;
        if (addr < 0x080E0000UL) return FLASH_SECTOR_6;
        return FLASH_SECTOR_7;
    }

    *bank = flash_bank_2;
    if (addr < 0x08120000UL) return FLASH_SECTOR_0;
    if (addr < 0x08140000UL) return FLASH_SECTOR_1;
    if (addr < 0x08160000UL) return FLASH_SECTOR_2;
    if (addr < 0x08180000UL) return FLASH_SECTOR_3;
    if (addr < 0x081A0000UL) return FLASH_SECTOR_4;
    if (addr < 0x081C0000UL) return FLASH_SECTOR_5;
    if (addr < 0x081E0000UL) return FLASH_SECTOR_6;
    return FLASH_SECTOR_7;
}

void invalidate_flash_cache(uint32_t addr, std::size_t size)
{
#if defined(__DCACHE_PRESENT) && (__DCACHE_PRESENT == 1U)
    SCB_InvalidateDCache_by_Addr(reinterpret_cast<uint32_t*>(addr), static_cast<int32_t>(size));
#else
    (void)addr;
    (void)size;
#endif
}

types::status from_hal(HAL_StatusTypeDef status)
{
    return status == HAL_OK ? types::status::ok : types::status::error;
}

} // namespace

namespace bsp::flash
{

types::status erase_sector(uint32_t addr)
{
    uint32_t bank = 0;
    uint32_t sector = flash_sector_of(addr, &bank);

    FLASH_EraseInitTypeDef erase_init{};
    uint32_t sector_error = 0;

    erase_init.TypeErase = FLASH_TYPEERASE_SECTORS;
    erase_init.Banks = bank;
    erase_init.Sector = sector;
    erase_init.NbSectors = 1;

    HAL_FLASH_Unlock();
    const HAL_StatusTypeDef status = HAL_FLASHEx_Erase(&erase_init, &sector_error);
    HAL_FLASH_Lock();

    if (status == HAL_OK)
    {
        invalidate_flash_cache(addr, 128UL * 1024UL);
    }

    return from_hal(status);
}

types::status write_flash_word(uint32_t addr, const void* data)
{
    if (data == nullptr || (addr % flash_word_size) != 0U)
    {
        return types::status::invalid_arg;
    }

    HAL_FLASH_Unlock();
    const HAL_StatusTypeDef status = HAL_FLASH_Program(
        FLASH_TYPEPROGRAM_FLASHWORD, addr, reinterpret_cast<uint32_t>(data));
    HAL_FLASH_Lock();

    if (status == HAL_OK)
    {
        invalidate_flash_cache(addr, flash_word_size);
    }

    return from_hal(status);
}

} // namespace bsp::flash
