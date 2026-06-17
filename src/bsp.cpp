#include "bsp.hpp"

#include "bsp_board.hpp"

#include "bsp_adc.hpp"
#include "bsp_can.hpp"
#include "bsp_dwt.hpp"
#include "bsp_pwm.hpp"
#include "bsp_usart.hpp"

namespace bsp {

void init() {
#if BSP_DWT_ENABLE
    dwt_init(BSP_CPU_FREQ_MHZ);
#endif
#if BSP_UART_ENABLE_USART1
    usart_init<UartPort::Usart1>(UartMode::Dma);
#endif
#if BSP_CAN_ENABLE_BUS1
    can_init(CanBus::Bus1, CanType::Classic);
#endif
#if BSP_CAN_ENABLE_BUS2
    can_init(CanBus::Bus2, CanType::Classic);
#endif
#if BSP_CAN_ENABLE_BUS3
    can_init(CanBus::Bus3, CanType::Classic);
#endif
#if BSP_PWM_ENABLE_TIM12_CH2
    pwm_init<PwmInstance::Tim12_Ch2>();
#endif
#if BSP_ADC_ENABLE_ADC1
    adc_init<AdcInstance::Adc1>();
#endif
}

} // namespace bsp
