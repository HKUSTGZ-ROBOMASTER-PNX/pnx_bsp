#pragma once

#include "dma.h"
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

using rx_handler = void (*)(port port, const rx_frame& frame, void* user_data);

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
types::status start_rx_to_idle(port port, uint8_t* buffer, size_t len, rx_handler handler,
                               void* user_data, TX_SEMAPHORE* notify_sem = nullptr);
types::status restart_rx(port port);

} // namespace bsp::usart
