#pragma once

#include "bsp.hpp"
#include "bsp_board.hpp"

namespace bsp {

enum class PwmInstance : uint8_t {
    Tim3_Ch4,
    Tim12_Ch2,
};

constexpr bool pwm_is_enabled(PwmInstance inst) {
    switch (inst) {
#if BSP_PWM_ENABLE_TIM3_CH4
        case PwmInstance::Tim3_Ch4:
            return true;
#endif
#if BSP_PWM_ENABLE_TIM12_CH2
        case PwmInstance::Tim12_Ch2:
            return true;
#endif
        default:
            return false;
    }
}

Status pwm_init_impl(PwmInstance inst);
Status pwm_start(PwmInstance inst);
Status pwm_stop(PwmInstance inst);
void pwm_set_period(PwmInstance inst, float period_s);
void pwm_set_duty(PwmInstance inst, float duty_ratio);

template <PwmInstance Inst>
Status pwm_init() {
    static_assert(pwm_is_enabled(Inst), "PWM instance not enabled in pnx_configs/bsp_board.hpp");
    return pwm_init_impl(Inst);
}

} // namespace bsp
