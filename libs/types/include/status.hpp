#pragma once

#include <cstdint>

namespace pnxtypes
{
    
enum class status : uint8_t 
{
    ok = 0,
    error,
    not_configured,
    invalid_arg,
    busy,
    not_initialized,
    timeout,
};

} // namespace pnxtypes