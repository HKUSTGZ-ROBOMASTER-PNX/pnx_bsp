#pragma once

#include <cstddef>
#include <cstdint>

namespace bsp {

enum class Status : uint8_t {
    Ok = 0,
    Error,
    NotConfigured,
    InvalidArg,
};

void init();

} // namespace bsp
