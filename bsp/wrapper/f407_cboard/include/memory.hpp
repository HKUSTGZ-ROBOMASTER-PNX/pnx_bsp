#pragma once

#include <stdint.h>

// Use static storage for asynchronous DMA; CCM is not DMA-accessible.
#define RAM_D1_BSS

namespace memory
{
inline constexpr uint32_t cache_line_size = 1U;

// N is the payload size in bytes; capacity includes alignment padding.
template <uint32_t N>
class buffer
{
    static_assert(N > 0 && N <= UINT32_MAX - (cache_line_size - 1U),
                  "Invalid DMA buffer size");
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

template <uint32_t N>
inline bool valid(const buffer<N>& value) noexcept
{
    const auto start = reinterpret_cast<uintptr_t>(value.data());
    return start >= 0x20000000U && start < 0x20020000U &&
           start % cache_line_size == 0U && value.capacity() <= 0x20020000U - start;
}
} // namespace memory
