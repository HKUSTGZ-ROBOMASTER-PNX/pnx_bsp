#include "bsp_exti.h"

namespace
{

struct exti_slot
{
    uint16_t pin = 0;
    bsp_exti_callback_t callback = nullptr;
    void* user = nullptr;
};

constexpr int max_exti_slots = 16;
exti_slot slots[max_exti_slots]{};

} // namespace

extern "C" void bsp_exti_attach(uint16_t pin, bsp_exti_callback_t callback, void* user)
{
    for (auto& slot : slots)
    {
        if (slot.pin == pin || slot.pin == 0)
        {
            slot.pin = pin;
            slot.callback = callback;
            slot.user = user;
            return;
        }
    }
}

extern "C" void bsp_exti_dispatch(uint16_t pin)
{
    for (auto& slot : slots)
    {
        if (slot.pin == pin && slot.callback != nullptr)
        {
            slot.callback(slot.user);
            return;
        }
    }
}
