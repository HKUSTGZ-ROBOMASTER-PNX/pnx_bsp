#include "bsp_indicator.hpp"

namespace bsp::indicator
{
namespace
{

state current{};

bool* value_of(channel selected) noexcept
{
    switch (selected)
    {
    case channel::red:
        return &current.red;
    case channel::green:
        return &current.green;
    case channel::blue:
        return &current.blue;
    default:
        return nullptr;
    }
}

} // namespace

types::status init() noexcept
{
    const types::status status = detail::backend_init();
    if (status == types::status::ok)
    {
        current = {};
        current.initialized = true;
    }
    return status;
}

types::status set(channel selected, bool on) noexcept
{
    bool* value = value_of(selected);
    if (!current.initialized || value == nullptr)
    {
        return types::status::invalid_arg;
    }
    const types::status status = detail::backend_set(selected, on);
    if (status == types::status::ok)
    {
        *value = on;
    }
    return status;
}

types::status toggle(channel selected) noexcept
{
    bool* value = value_of(selected);
    return value != nullptr ? set(selected, !*value) : types::status::invalid_arg;
}

state snapshot() noexcept
{
    return current;
}

} // namespace bsp::indicator
