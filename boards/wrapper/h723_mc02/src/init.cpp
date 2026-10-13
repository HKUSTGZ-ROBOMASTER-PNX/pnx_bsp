#include "app_threadx.h"
#include "main.h"
#include "adc.h"
#include "crc.h"
#include "dma.h"
#include "fdcan.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "usb_otg.h"
#include "gpio.h"

extern "C" void SystemClock_Config(void);
extern "C" void PeriphCommonClock_Config(void);


int main(void)
{
    SCB_EnableICache();
    SCB_EnableDCache();
    HAL_Init();
    SystemClock_Config();
    PeriphCommonClock_Config();
    MX_GPIO_Init();
    MX_DMA_Init();
    MX_FDCAN1_Init();
    MX_FDCAN2_Init();
    MX_FDCAN3_Init();
    MX_USART1_UART_Init();
    MX_ADC1_Init();
    MX_SPI6_Init();
    MX_TIM12_Init();
    MX_UART5_Init();
    MX_UART7_Init();
    MX_SPI2_Init();
    MX_TIM3_Init();
    MX_CRC_Init();
    MX_USB_OTG_HS_PCD_Init();
    MX_USART10_UART_Init();
    MX_TIM6_Init();
    tx_kernel_enter();
    return 1;
}