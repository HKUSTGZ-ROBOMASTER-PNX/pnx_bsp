#include "bsp_usart.hpp"

#include "bsp_board.hpp"

#include "dma.h"
#include "os.hpp"
#include "usart.h"

#include <cstring>

extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart1_tx;

extern DMA_HandleTypeDef hdma_uart5_rx;
extern DMA_HandleTypeDef hdma_uart7_rx;

namespace {

struct UartPortContext {
    bool initialized = false;
    bsp::UartMode mode = bsp::UartMode::Block;
    bsp::UartRxCallback rx_callback = nullptr;
    void* rx_user_data = nullptr;
    os::Semaphore* notify_sem = nullptr;
    uint8_t* rx_buffer = nullptr;
    size_t rx_buffer_len = 0;
};

UartPortContext g_uart_ports[3];

UART_HandleTypeDef* handle_for(bsp::UartPort port) {
    switch (port) {
        case bsp::UartPort::Usart1:
            return &huart1;
        case bsp::UartPort::Uart5:
            return &huart5;
        case bsp::UartPort::Uart7:
            return &huart7;
        default:
            return nullptr;
    }
}

UartPortContext* ctx_for(bsp::UartPort port) {
    switch (port) {
        case bsp::UartPort::Usart1:
            return &g_uart_ports[0];
        case bsp::UartPort::Uart5:
            return &g_uart_ports[1];
        case bsp::UartPort::Uart7:
            return &g_uart_ports[2];
        default:
            return nullptr;
    }
}

bsp::UartPort port_from_handle(UART_HandleTypeDef* huart) {
    if (huart == &huart1) {
        return bsp::UartPort::Usart1;
    }
    if (huart == &huart5) {
        return bsp::UartPort::Uart5;
    }
    if (huart == &huart7) {
        return bsp::UartPort::Uart7;
    }
    return bsp::UartPort::Usart1;
}

void setup_dma(UART_HandleTypeDef* huart) {
    if (huart == &huart1) {
        __HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);
        __HAL_DMA_ENABLE_IT(&hdma_usart1_rx, DMA_IT_TC);
        __HAL_DMA_DISABLE_IT(&hdma_usart1_tx, DMA_IT_HT);
        __HAL_DMA_ENABLE_IT(&hdma_usart1_tx, DMA_IT_TC);
    }
    if( huart == &huart5) {
        __HAL_DMA_DISABLE_IT(&hdma_uart5_rx, DMA_IT_HT);
        __HAL_DMA_ENABLE_IT(&hdma_uart5_rx, DMA_IT_TC);
        // __HALDMA_DISABLE_IT(&hdma_uart5_tx, DMA_IT_HT);
        // __HAL_DMA_ENABLE_IT(&hdma_uart5_tx, DMA_IT_TC);
        //NOTE: 5tx没开，7tx开了
    }
    if( huart == &huart7) {
        __HAL_DMA_DISABLE_IT(&hdma_uart7_rx, DMA_IT_HT);
        __HAL_DMA_ENABLE_IT(&hdma_uart7_rx, DMA_IT_TC);
        // __HALDMA_DISABLE_IT(&hdma_uart7_tx, DMA_IT_HT);
        // __HAL_DMA_ENABLE_IT(&hdma_uart7_tx, DMA_IT_TC);
    }
}

void invalidate_rx_cache(uint8_t* buffer, size_t len) {
#if (__DCACHE_PRESENT == 1U)
    SCB_InvalidateDCache_by_Addr(reinterpret_cast<uint32_t*>(buffer), static_cast<int32_t>(len));
#else
    (void)buffer;
    (void)len;
#endif
}

} // namespace

namespace bsp {

Status usart_init_impl(UartPort port, UartMode mode) {
    if (!usart_is_enabled(port)) {
        return Status::NotConfigured;
    }
    UART_HandleTypeDef* huart = handle_for(port);
    UartPortContext* ctx = ctx_for(port);
    if (huart == nullptr || ctx == nullptr) {
        return Status::InvalidArg;
    }
    if (mode == UartMode::Dma) {
        setup_dma(huart);
    }
    __HAL_UART_SEND_REQ(huart, UART_RXDATA_FLUSH_REQUEST);
    ctx->mode = mode;
    ctx->initialized = true;
    return Status::Ok;
}

Status usart_transmit(UartPort port, const uint8_t* data, size_t len, uint32_t timeout_ms) {
    if (!usart_is_enabled(port) || data == nullptr || len == 0) {
        return Status::InvalidArg;
    }
    UART_HandleTypeDef* huart = handle_for(port);
    UartPortContext* ctx = ctx_for(port);
    if (huart == nullptr || ctx == nullptr || !ctx->initialized) {
        return Status::Error;
    }

    if (ctx->mode == UartMode::Dma) {
        return HAL_UART_Transmit_DMA(huart, const_cast<uint8_t*>(data), static_cast<uint16_t>(len)) ==
                       HAL_OK
                   ? Status::Ok
                   : Status::Error;
    }

    return HAL_UART_Transmit(huart, const_cast<uint8_t*>(data), static_cast<uint16_t>(len),
                             timeout_ms) == HAL_OK
               ? Status::Ok
               : Status::Error;
}

Status usart_start_rx_to_idle(UartPort port, uint8_t* buffer, size_t len, UartRxCallback callback,
                              void* user_data, os::Semaphore* notify_sem) {
    if (!usart_is_enabled(port) || buffer == nullptr || len == 0) {
        return Status::InvalidArg;
    }
    UART_HandleTypeDef* huart = handle_for(port);
    UartPortContext* ctx = ctx_for(port);
    if (huart == nullptr || ctx == nullptr || !ctx->initialized) {
        return Status::Error;
    }

    ctx->rx_buffer = buffer;
    ctx->rx_buffer_len = len;
    ctx->rx_callback = callback;
    ctx->rx_user_data = user_data;
    ctx->notify_sem = notify_sem;

    if (HAL_UARTEx_ReceiveToIdle_DMA(huart, buffer, static_cast<uint16_t>(len)) != HAL_OK) {
        return Status::Error;
    }
    return Status::Ok;
}

Status usart_restart_rx(UartPort port) {
    UartPortContext* ctx = ctx_for(port);
    UART_HandleTypeDef* huart = handle_for(port);
    if (ctx == nullptr || huart == nullptr || ctx->rx_buffer == nullptr || ctx->rx_buffer_len == 0) {
        return Status::InvalidArg;
    }
    if (HAL_UARTEx_ReceiveToIdle_DMA(huart, ctx->rx_buffer,
                                       static_cast<uint16_t>(ctx->rx_buffer_len)) != HAL_OK) {
        return Status::Error;
    }
    return Status::Ok;
}

} // namespace bsp

extern "C" void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef* huart, uint16_t size) {
    const auto port = port_from_handle(huart);
    auto* ctx = ctx_for(port);
    if (ctx == nullptr || ctx->rx_buffer == nullptr) {
        return;
    }

    invalidate_rx_cache(ctx->rx_buffer, size);

    if (ctx->notify_sem != nullptr) {
        os::sem_give(ctx->notify_sem);
    }
    if (ctx->rx_callback != nullptr) {
        ctx->rx_callback(port, ctx->rx_buffer, size, ctx->rx_user_data);
    }

    HAL_UARTEx_ReceiveToIdle_DMA(huart, ctx->rx_buffer, static_cast<uint16_t>(ctx->rx_buffer_len));
}

extern "C" void HAL_UART_ErrorCallback(UART_HandleTypeDef* huart) {
    const auto port = port_from_handle(huart);
    auto* ctx = ctx_for(port);
    if (ctx == nullptr) {
        return;
    }
    __HAL_UART_CLEAR_OREFLAG(huart);
    bsp::usart_restart_rx(port);
}
