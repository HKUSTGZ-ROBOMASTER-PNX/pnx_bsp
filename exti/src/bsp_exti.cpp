#include "bsp_exti.hpp"
#include "bsp_exti.h"

namespace
{

struct exti_slot
{
    uint16_t pin = 0;
    bsp::exti::interrupt_callback callback{};
};

constexpr int max_exti_slots = 16;
exti_slot slots[max_exti_slots]{};

} // namespace

types::status bsp::exti::attach(uint16_t pin, interrupt_callback callback)
{
    if (!callback)
    {
        return types::status::invalid_arg;
    }
    for (auto& slot : slots)
    {
        if (slot.pin == pin || slot.pin == 0)
        {
            slot.pin = pin;
            slot.callback = callback;
            return types::status::ok;
        }
    }
    return types::status::error;
}

extern "C" void bsp_exti_dispatch(uint16_t pin)
{
    for (auto& slot : slots)
    {
        if (slot.pin == pin && slot.callback)
        {
            slot.callback();
            return;
        }
    }
}
