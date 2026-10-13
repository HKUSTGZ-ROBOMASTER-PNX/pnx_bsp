#include "app_threadx.h"
#include "main.h"
#include "crc.h"
#include "dma.h"
#include "can.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "usb_otg.h"
#include "gpio.h"

extern "C" void SystemClock_Config(void);
extern "C" void PeriphCommonClock_Config(void);

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_DMA_Init();
    MX_CAN1_Init();
    MX_CAN2_Init();
    MX_SPI1_Init();
    MX_TIM1_Init();
    MX_TIM2_Init();
    MX_TIM4_Init();
    MX_TIM5_Init();
    MX_TIM8_Init();
    MX_TIM10_Init();
    MX_USART3_UART_Init();
    MX_CRC_Init();
    MX_USART6_UART_Init();
    MX_USART1_UART_Init();
    MX_USB_OTG_FS_PCD_Init();
    MX_USART2_UART_Init();
    MX_TIM7_Init();
    tx_kernel_enter();
    return 1;
}