#include "bsp_diagnostics.h"

#include <cstddef>
#include <cstdint>

namespace
{

constexpr std::uint32_t fnv_offset = 2166136261U;
constexpr std::uint32_t fnv_prime = 16777619U;

} // namespace

extern "C" std::uint32_t bsp_diagnostics_record_checksum(
    const bsp_diagnostics_crash_record* record)
{
    if (record == nullptr)
    {
        return 0U;
    }
    const auto* bytes =
        reinterpret_cast<const std::uint8_t*>(record);
    std::uint32_t hash = fnv_offset;
    for (std::size_t index = 0U;
         index < offsetof(bsp_diagnostics_crash_record, checksum);
         ++index)
    {
        hash ^= bytes[index];
        hash *= fnv_prime;
    }
    return hash;
}

extern "C" int bsp_diagnostics_record_valid(
    const bsp_diagnostics_crash_record* record)
{
    return record != nullptr &&
                   record->magic == BSP_DIAGNOSTICS_CRASH_MAGIC &&
                   record->version == BSP_DIAGNOSTICS_ABI_VERSION &&
                   record->size == sizeof(*record) &&
                   record->checksum ==
                       bsp_diagnostics_record_checksum(record)
               ? 1
               : 0;
}
