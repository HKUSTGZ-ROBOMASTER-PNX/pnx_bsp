#pragma once

#include "dma.h"
#include "callback.hpp"
#include "bsp_dma.hpp"
#include "config.hpp"
#include "tx_api.h"
#include "usart.h"
#include "usertypes.hpp"

#include <cstddef>

namespace bsp::usart
{

enum class mode : uint8_t
{
    block = 0,
    dma = 1,
    it = 2
};

enum class word_length : uint8_t
{
    bits_8 = 0,
    bits_9,
};

enum class stop_bits : uint8_t
{
    one = 0,
    two,
};

enum class parity : uint8_t
{
    none = 0,
    even,
    odd,
};

struct line_config
{
    uint32_t baud_rate = 115200;
    word_length data_bits = word_length::bits_8;
    stop_bits stop = stop_bits::one;
    parity parity_mode = parity::none;
    bool enable_tx = true;
    bool enable_rx = true;
};

struct rx_frame
{
    uint8_t* data = nullptr;
    size_t len = 0;
};

// Invoked from the UART HAL interrupt/DMA callback path; it must not block.
using rx_callback = core::callback<void(port, const rx_frame&)>;

UART_HandleTypeDef* handle_of(std::size_t index) noexcept;
std::size_t index_of(UART_HandleTypeDef* handle) noexcept;
bool port_enabled(port port) noexcept;
void setup_dma(UART_HandleTypeDef* handle) noexcept;

void receive(UART_HandleTypeDef* handle, uint16_t size);
void handle_error(UART_HandleTypeDef* handle);
void handle_tx_complete(UART_HandleTypeDef* handle);

types::status init(port port, mode mode);
types::status configure(port port, const line_config& config);
types::status transmit(port port, const uint8_t* data, size_t len, uint32_t timeout_ms);
/**
 * 启动 UART receive-to-idle DMA。
 *
 * 上下文：线程/启动阶段。接收回调运行在 UART HAL 中断/DMA 回调路径，
 * 不能阻塞。`buffer` 和 callback 的目标为借用对象，必须保持有效，直到
 * 接收停止或以相同 owner 替换。`notify_sem` 非空时，会从该回调路径发出
 * 信号。buffer 必须来自 `bsp::dma::buffer`。
 */
types::status start_rx_to_idle(port port, bsp::dma::buffer_view buffer, rx_callback callback,
                               TX_SEMAPHORE* notify_sem = nullptr);
types::status restart_rx(port port);

} // namespace bsp::usart
