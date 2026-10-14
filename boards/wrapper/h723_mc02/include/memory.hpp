#pragma once

#include <stdint.h>

// Static D1 storage is not startup-cleared; drivers must handle D-cache.
#define RAM_D1_BSS __attribute__((section(".ram_d1_bss")))

namespace memory
{
inline constexpr uint32_t cache_line_size = 32U;

// N is the payload size in bytes; capacity includes alignment padding.
template <uint32_t N> class buffer
{
    static_assert(N > 0 && N <= UINT32_MAX - (cache_line_size - 1U), "Invalid DMA buffer size");
    static constexpr uint32_t bytes = (N + cache_line_size - 1U) & ~(cache_line_size - 1U);
    alignas(cache_line_size) uint8_t storage_[bytes]{};

public:
    constexpr uint8_t* data() noexcept
    {
        return storage_;
    }

    constexpr const uint8_t* data() const noexcept
    {
        return storage_;
    }

    constexpr uint8_t& operator[](uint32_t i) noexcept
    {
        return storage_[i];
    }

    constexpr const uint8_t& operator[](uint32_t i) const noexcept
    {
        return storage_[i];
    }

    static constexpr uint32_t logical_size() noexcept
    {
        return N;
    }

    static constexpr uint32_t capacity() noexcept
    {
        return bytes;
    }
};

inline bool valid(const void* data, uint32_t size, uint32_t capacity) noexcept
{
    const auto start = reinterpret_cast<uintptr_t>(data);
    return size != 0 && capacity >= size && capacity % cache_line_size == 0 &&
           start >= 0x24000000U && start < 0x24050000U && start % cache_line_size == 0 &&
           capacity <= 0x24050000U - start;
}

template <uint32_t N> inline bool valid(const buffer<N>& value) noexcept
{
    return valid(value.data(), value.logical_size(), value.capacity());
}
} // namespace memory
