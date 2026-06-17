#include "bsp_adc.hpp"

#include "adc.h"

namespace bsp {

Status adc_init_impl(AdcInstance inst) {
    if (!adc_is_enabled(inst)) {
        return Status::NotConfigured;
    }
    switch (inst) {
        case AdcInstance::Adc1:
            return HAL_ADCEx_Calibration_Start(&hadc1, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED) == HAL_OK
                       ? Status::Ok
                       : Status::Error;
        default:
            return Status::InvalidArg;
    }
}

Status adc_start(AdcInstance inst, uint16_t* buffer, size_t count) {
    if (!adc_is_enabled(inst) || buffer == nullptr || count == 0) {
        return Status::InvalidArg;
    }
    switch (inst) {
        case AdcInstance::Adc1:
            if (HAL_ADC_Start(&hadc1) != HAL_OK) {
                return Status::Error;
            }
            if (HAL_ADC_PollForConversion(&hadc1, 100) != HAL_OK) {
                return Status::Error;
            }
            buffer[0] = static_cast<uint16_t>(HAL_ADC_GetValue(&hadc1));
            return HAL_ADC_Stop(&hadc1) == HAL_OK ? Status::Ok : Status::Error;
        default:
            return Status::InvalidArg;
    }
}

} // namespace bsp
