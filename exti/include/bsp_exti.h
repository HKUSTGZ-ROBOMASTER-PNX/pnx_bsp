#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef void (*bsp_exti_callback_t)(void* user);

void bsp_exti_attach(uint16_t pin, bsp_exti_callback_t callback, void* user);
void bsp_exti_dispatch(uint16_t pin);

#ifdef __cplusplus
}
#endif
