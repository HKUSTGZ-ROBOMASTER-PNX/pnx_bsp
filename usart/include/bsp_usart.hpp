#pragma once

#include "bsp.hpp"
#include "bsp_board.hpp"

#include "os.hpp"

#include <cstddef>
#include <cstdint>

namespace bsp {

enum class UartPort : uint8_t {
    Usart1 = 1,
    Uart5 = 5,
    Uart7 = 7,
};

enum class UartMode : uint8_t {
    Block = 0,
    Dma = 1,
    IT = 2,
};

using UartRxCallback = void (*)(UartPort port, uint8_t* data, size_t len, void* user_data);

constexpr bool usart_is_enabled(UartPort port) {
    switch (port) {
#if BSP_UART_ENABLE_USART1
        case UartPort::Usart1:
            return true;
#endif
#if BSP_UART_ENABLE_UART5
        case UartPort::Uart5:
            return true;
#endif
#if BSP_UART_ENABLE_UART7
        case UartPort::Uart7:
            return true;
#endif
        default:
            return false;
    }
}

Status usart_init_impl(UartPort port, UartMode mode);

template <UartPort Port>
Status usart_init(UartMode mode) {
    static_assert(usart_is_enabled(Port), "UART port not enabled in pnx_configs/bsp_board.hpp");
    return usart_init_impl(Port, mode);
}

Status usart_transmit(UartPort port, const uint8_t* data, size_t len, uint32_t timeout_ms);
Status usart_start_rx_to_idle(UartPort port, uint8_t* buffer, size_t len, UartRxCallback callback,
                              void* user_data, os::Semaphore* notify_sem = nullptr);
Status usart_restart_rx(UartPort port);

} // namespace bsp
