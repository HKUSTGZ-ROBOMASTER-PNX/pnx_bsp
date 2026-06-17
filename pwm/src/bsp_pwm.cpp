#include "bsp_pwm.hpp"

#include "tim.h"

namespace {

constexpr uint32_t kPwmTimerClockHz = 240'000'000U;

struct PwmBinding {
    TIM_HandleTypeDef* timer;
    uint32_t channel;
};

bool binding_for(bsp::PwmInstance inst, PwmBinding* out) {
    switch (inst) {
        case bsp::PwmInstance::Tim12_Ch2:
            out->timer = &htim12;
            out->channel = TIM_CHANNEL_2;
            return true;
        default:
            return false;
    }
}

} // namespace

namespace bsp {

Status pwm_init_impl(PwmInstance inst) {
    if (!pwm_is_enabled(inst)) {
        return Status::NotConfigured;
    }
    PwmBinding binding{};
    if (!binding_for(inst, &binding)) {
        return Status::InvalidArg;
    }
    __HAL_TIM_SET_COMPARE(binding.timer, binding.channel, 0);
    return HAL_TIM_PWM_Start(binding.timer, binding.channel) == HAL_OK ? Status::Ok : Status::Error;
}

Status pwm_start(PwmInstance inst) {
    PwmBinding binding{};
    if (!binding_for(inst, &binding)) {
        return Status::InvalidArg;
    }
    __HAL_TIM_SET_COMPARE(binding.timer, binding.channel, 0);
    return HAL_TIM_PWM_Start(binding.timer, binding.channel) == HAL_OK ? Status::Ok : Status::Error;
}

Status pwm_stop(PwmInstance inst) {
    PwmBinding binding{};
    if (!binding_for(inst, &binding)) {
        return Status::InvalidArg;
    }
    return HAL_TIM_PWM_Stop(binding.timer, binding.channel) == HAL_OK ? Status::Ok : Status::Error;
}

void pwm_set_period(PwmInstance inst, float period_s) {
    PwmBinding binding{};
    if (!binding_for(inst, &binding) || period_s <= 0.0f) {
        return;
    }
    const uint32_t prescaler = binding.timer->Init.Prescaler + 1U;
    __HAL_TIM_SET_AUTORELOAD(binding.timer,
                             static_cast<uint32_t>(period_s * (static_cast<float>(kPwmTimerClockHz) /
                                                               static_cast<float>(prescaler))));
}

void pwm_set_duty(PwmInstance inst, float duty_ratio) {
    PwmBinding binding{};
    if (!binding_for(inst, &binding)) {
        return;
    }
    if (duty_ratio < 0.0f) {
        duty_ratio = 0.0f;
    }
    if (duty_ratio > 1.0f) {
        duty_ratio = 1.0f;
    }
    __HAL_TIM_SET_COMPARE(binding.timer, binding.channel,
                          static_cast<uint32_t>(duty_ratio * static_cast<float>(binding.timer->Instance->ARR)));
}

} // namespace bsp
