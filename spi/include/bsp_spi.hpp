#pragma once

#include "usertypes.hpp"

#include <cstddef>
#include <cstdint>

namespace bsp::spi
{

struct bus
{
    std::uint8_t value = 0xFFU;

    constexpr explicit bus(std::uint8_t selected = 0xFFU) noexcept
        : value(selected)
    {
    }
};

struct chip_select
{
    std::uint8_t value = 0xFFU;

    constexpr explicit chip_select(
        std::uint8_t selected = 0xFFU) noexcept
        : value(selected)
    {
    }
};

constexpr bool operator==(bus lhs, bus rhs) noexcept
{
    return lhs.value == rhs.value;
}

constexpr bool operator!=(bus lhs, bus rhs) noexcept
{
    return !(lhs == rhs);
}

constexpr bool operator==(chip_select lhs, chip_select rhs) noexcept
{
    return lhs.value == rhs.value;
}

inline constexpr bus no_bus{};
inline constexpr chip_select no_select{};

bool bus_enabled(bus selected) noexcept;
bool select_enabled(chip_select selected) noexcept;
types::status init(bus selected) noexcept;
types::status wait_ready(
    bus selected, std::uint32_t timeout_ms = 1000U) noexcept;
types::status transmit(
    bus selected, const std::uint8_t* data, std::size_t len,
    std::uint32_t timeout_ms) noexcept;
types::status receive(
    bus selected, std::uint8_t* data, std::size_t len,
    std::uint32_t timeout_ms) noexcept;
types::status set_select(
    chip_select selected, bool active) noexcept;

namespace detail
{
bool backend_bus_enabled(bus selected) noexcept;
bool backend_select_enabled(chip_select selected) noexcept;
types::status backend_init(bus selected) noexcept;
types::status backend_wait_ready(
    bus selected, std::uint32_t timeout_ms) noexcept;
types::status backend_transmit(
    bus selected, const std::uint8_t* data, std::size_t len,
    std::uint32_t timeout_ms) noexcept;
types::status backend_receive(
    bus selected, std::uint8_t* data, std::size_t len,
    std::uint32_t timeout_ms) noexcept;
types::status backend_set_select(
    chip_select selected, bool active) noexcept;
} // namespace detail

} // namespace bsp::spi
