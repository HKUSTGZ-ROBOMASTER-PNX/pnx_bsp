#pragma once

#include "main.h"
#include "status.hpp"

namespace bsp
{
constexpr pnxtypes::status to_status(HAL_StatusTypeDef value) noexcept
{
    switch (value)
    {
        case HAL_OK:
            return pnxtypes::status::ok;
        case HAL_BUSY:
            return pnxtypes::status::busy;
        case HAL_TIMEOUT:
            return pnxtypes::status::timeout;
        default:
            return pnxtypes::status::error;
    }
}

// Call only on validated, cache-line-isolated DMA allocations.
inline void clean_cache(const void* data, uint32_t capacity) noexcept
{
#if defined(__DCACHE_PRESENT) && (__DCACHE_PRESENT == 1U)
    SCB_CleanDCache_by_Addr(reinterpret_cast<uint32_t*>(const_cast<void*>(data)),
                            static_cast<int32_t>(capacity));
#else
    (void)data;
    (void)capacity;
#endif
}

inline void prepare_rx_cache(void* data, uint32_t capacity) noexcept
{
#if defined(__DCACHE_PRESENT) && (__DCACHE_PRESENT == 1U)
    SCB_CleanInvalidateDCache_by_Addr(reinterpret_cast<uint32_t*>(data),
                                      static_cast<int32_t>(capacity));
#else
    (void)data;
    (void)capacity;
#endif
}

inline void invalidate_cache(void* data, uint32_t capacity) noexcept
{
#if defined(__DCACHE_PRESENT) && (__DCACHE_PRESENT == 1U)
    SCB_InvalidateDCache_by_Addr(reinterpret_cast<uint32_t*>(data), static_cast<int32_t>(capacity));
#else
    (void)data;
    (void)capacity;
#endif
}
} // namespace bsp
