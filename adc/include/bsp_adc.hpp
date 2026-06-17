#pragma once

#include "bsp.hpp"
#include "bsp_board.hpp"

#include <cstddef>
#include <cstdint>

namespace bsp {

enum class AdcInstance : uint8_t {
    Adc1,
};

constexpr bool adc_is_enabled(AdcInstance inst) {
    switch (inst) {
#if BSP_ADC_ENABLE_ADC1
        case AdcInstance::Adc1:
            return true;
#endif
        default:
            return false;
    }
}

Status adc_init_impl(AdcInstance inst);
Status adc_start(AdcInstance inst, uint16_t* buffer, size_t count);

template <AdcInstance Inst>
Status adc_init() {
    static_assert(adc_is_enabled(Inst), "ADC instance not enabled in pnx_configs/bsp_board.hpp");
    return adc_init_impl(Inst);
}

} // namespace bsp
