#pragma once

#include "main.h"
#include "config.hpp"
#include "memory.hpp"
#include "status.hpp"

#if defined(HAL_ADC_MODULE_ENABLED)
#include "adc.h"

namespace bsp::adc
{
struct conversion_state
{
    volatile bool busy = false;
    volatile bool complete = false;
    volatile bool dma = false;
    volatile pnxtypes::status last_status = pnxtypes::status::ok;
    volatile uint32_t last_value = 0;
};

// CubeMX initializes ADC first. Serialize calls from foreground tasks.
pnxtypes::status init(const config& resource) noexcept;
pnxtypes::status calibrate(handle instance) noexcept;
pnxtypes::status read_raw(handle instance, uint32_t& value, uint32_t timeout_ms) noexcept;

// Async operations retain ownership after completion/error until the matching stop.
pnxtypes::status start_it(handle instance) noexcept;
pnxtypes::status stop_it(handle instance) noexcept;

const conversion_state& state(handle instance) noexcept;

// Raw entry requires static, padded D1 storage. Prefer the buffer overload below.
pnxtypes::status start_dma(handle instance, uint8_t* data, uint32_t size,
                           uint32_t capacity) noexcept;
pnxtypes::status stop_dma(handle instance) noexcept;

// N is bytes; each ADC sample occupies four bytes. Keep storage alive and do
// not access it until successful completion or stop. Stop before restarting or reuse.
template <uint32_t N>
pnxtypes::status start_dma(handle instance, memory::buffer<N>& samples) noexcept
{
    if (!memory::valid(samples))
    {
        return pnxtypes::status::invalid_arg;
    }
    return start_dma(instance, samples.data(), samples.logical_size(), samples.capacity());
}

// HAL callbacks are forwarded by this module; ADC/DMA IRQ handlers belong to the board.
void handle_conversion_complete(ADC_HandleTypeDef* adc) noexcept;
void handle_error(ADC_HandleTypeDef* adc) noexcept;
} // namespace bsp::adc
#endif
