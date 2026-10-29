#include "bsp_usart.hpp"
#include "bsp.hpp"
#include "peripherals.hpp"
#include <cstring>

namespace bsp::usart
{
using pnxtypes::status;
static constexpr uint32_t count = sizeof(peripherals::usart_configs) / sizeof(config);

struct context
{
    bool initialized = false;
    volatile bool tx_busy = false;
    mode method = mode::block;
    uint8_t* rx = nullptr;
    uint32_t size = 0, capacity = 0, position = 0;
    rx_callback callback{};
    TX_SEMAPHORE* notify = nullptr;
    volatile status received = status::not_initialized;
};

static context contexts[count]{};
alignas(memory::cache_line_size < 2
            ? 2
            : memory::cache_line_size) static memory::buffer<256> tx_slots[count] RAM_D1_BSS;

static uint32_t index(handle instance) noexcept
{
    for (uint32_t i = 0; i < count; ++i)
    {
        if (peripherals::usart_configs[i].instance == instance)
        {
            return i;
        }
    }
    return count;
}

static uint32_t index(UART_HandleTypeDef* native) noexcept
{
    for (uint32_t i = 0; i < count; ++i)
    {
        if (native != nullptr &&
            peripherals::usart_handle(peripherals::usart_configs[i].instance) == native)
        {
            return i;
        }
    }
    return count;
}

static uint32_t word_bytes(UART_HandleTypeDef* native) noexcept
{
    return native->Init.WordLength == UART_WORDLENGTH_9B && native->Init.Parity == UART_PARITY_NONE
               ? 2U
               : 1U;
}

status init(const config& resource, mode method) noexcept
{
    const auto i = index(resource.instance);
    auto* native = peripherals::usart_handle(resource.instance);
    if (i == count || native == nullptr)
    {
        return status::not_configured;
    }
    auto& item = contexts[i];
    if (item.tx_busy || item.rx != nullptr)
    {
        return status::busy;
    }
    if (static_cast<uint8_t>(method) > static_cast<uint8_t>(mode::dma))
    {
        return status::invalid_arg;
    }
    if (native->Instance == nullptr || native->gState == HAL_UART_STATE_RESET)
    {
        return status::not_initialized;
    }
    const auto& board = peripherals::usart_configs[i];
    if ((resource.dma.rx && !board.dma.rx) || (resource.dma.tx && !board.dma.tx) ||
        (method == mode::dma && (!board.dma.tx || native->hdmatx == nullptr)))
    {
        return status::not_configured;
    }
    __HAL_UART_FLUSH_DRREGISTER(native);
    item.method = method;
    item.initialized = true;
    return status::ok;
}

status configure(handle instance, const line_config& settings) noexcept
{
    const auto i = index(instance);
    auto* native = peripherals::usart_handle(instance);
    if (i == count || native == nullptr)
    {
        return status::not_configured;
    }
    auto& item = contexts[i];
    if (native->Instance == nullptr || native->gState == HAL_UART_STATE_RESET)
    {
        return status::not_initialized;
    }
    if (item.tx_busy || item.rx != nullptr || native->gState != HAL_UART_STATE_READY ||
        native->RxState != HAL_UART_STATE_READY)
    {
        return status::busy;
    }
    const auto bits = static_cast<uint8_t>(settings.data_bits);
    const auto stop = static_cast<uint8_t>(settings.stop);
    const auto parity = static_cast<uint8_t>(settings.parity_mode);
    if (settings.baud_rate == 0 || (!settings.enable_tx && !settings.enable_rx) || bits > 1 ||
        stop > 1 || parity > 2)
    {
        return status::invalid_arg;
    }
    constexpr uint32_t words[]{UART_WORDLENGTH_8B, UART_WORDLENGTH_9B};
    constexpr uint32_t stops[]{UART_STOPBITS_1, UART_STOPBITS_2};
    constexpr uint32_t parities[]{UART_PARITY_NONE, UART_PARITY_EVEN, UART_PARITY_ODD};
    const auto previous = native->Init;
    native->Init.BaudRate = settings.baud_rate;
    native->Init.WordLength = words[bits];
    native->Init.StopBits = stops[stop];
    native->Init.Parity = parities[parity];
    native->Init.Mode =
        (settings.enable_tx ? UART_MODE_TX : 0U) | (settings.enable_rx ? UART_MODE_RX : 0U);
    native->Init.HwFlowCtl = UART_HWCONTROL_NONE;
    const auto result = HAL_UART_Init(native);
    item.initialized = false; // Reinitialize the BSP mode after line configuration.
    if (result != HAL_OK)
    {
        native->Init = previous;
    }
    return bsp::to_status(result);
}

status transmit(handle instance, const uint8_t* data, uint32_t size, uint32_t timeout) noexcept
{
    const auto i = index(instance);
    auto* native = peripherals::usart_handle(instance);
    if (i == count || native == nullptr)
    {
        return status::not_configured;
    }
    auto& item = contexts[i];
    if (!item.initialized)
    {
        return status::not_initialized;
    }
    const auto width = word_bytes(native);
    if (data == nullptr || size == 0 || size % width != 0 || size / width > UINT16_MAX ||
        reinterpret_cast<uintptr_t>(data) % width != 0)
    {
        return status::invalid_arg;
    }
    if ((native->Init.Mode & UART_MODE_TX) == 0)
    {
        return status::not_configured;
    }
    if (item.method != mode::dma)
    {
        return bsp::to_status(HAL_UART_Transmit(native, const_cast<uint8_t*>(data),
                                                static_cast<uint16_t>(size / width), timeout));
    }
    auto* dma = native->hdmatx;
    if (dma == nullptr || dma->Instance == nullptr || dma->Init.Mode != DMA_NORMAL ||
        dma->Init.Direction != DMA_MEMORY_TO_PERIPH ||
        dma->Init.PeriphDataAlignment !=
            (width == 1 ? DMA_PDATAALIGN_BYTE : DMA_PDATAALIGN_HALFWORD) ||
        dma->Init.MemDataAlignment !=
            (width == 1 ? DMA_MDATAALIGN_BYTE : DMA_MDATAALIGN_HALFWORD) ||
        dma->Init.MemInc != DMA_MINC_ENABLE)
    {
        return status::not_configured;
    }
    if (size > tx_slots[i].logical_size() || !memory::valid(tx_slots[i]))
    {
        return status::invalid_arg;
    }
    const auto mask = __get_PRIMASK();
    __disable_irq();
    if (item.tx_busy)
    {
        __set_PRIMASK(mask);
        return status::busy;
    }
    item.tx_busy = true;
    __set_PRIMASK(mask);
    std::memcpy(tx_slots[i].data(), data, size);
    bsp::clean_cache(tx_slots[i].data(), tx_slots[i].capacity());
    const auto result =
        HAL_UART_Transmit_DMA(native, tx_slots[i].data(), static_cast<uint16_t>(size / width));
    if (result != HAL_OK)
    {
        // Abort a partial HAL start before allowing the staging slot to be overwritten.
        if (HAL_UART_AbortTransmit(native) == HAL_OK)
        {
            item.tx_busy = false;
        }
    }
    return bsp::to_status(result);
}

status abort_tx(handle instance) noexcept
{
    const auto i = index(instance);
    auto* native = peripherals::usart_handle(instance);
    if (i == count || native == nullptr)
    {
        return status::not_configured;
    }
    if (!contexts[i].initialized)
    {
        return status::not_initialized;
    }
    const auto result = HAL_UART_AbortTransmit(native);
    if (result == HAL_OK)
    {
        contexts[i].tx_busy = false;
    }
    return bsp::to_status(result);
}

status restart_rx(handle instance) noexcept
{
    const auto i = index(instance);
    auto* native = peripherals::usart_handle(instance);
    if (i == count || native == nullptr)
    {
        return status::not_configured;
    }
    auto& item = contexts[i];
    if (!item.initialized)
    {
        return status::not_initialized;
    }
    if (!memory::valid(item.rx, item.size, item.capacity))
    {
        return item.received = status::invalid_arg;
    }
    if (native->RxState != HAL_UART_STATE_READY)
    {
        return item.received = status::busy;
    }
    bsp::prepare_rx_cache(item.rx, item.capacity);
    item.position = 0;
    const auto result = HAL_UARTEx_ReceiveToIdle_DMA(
        native, item.rx, static_cast<uint16_t>(item.size / word_bytes(native)));
    if (result == HAL_OK)
    {
        __HAL_DMA_DISABLE_IT(native->hdmarx, DMA_IT_HT);
    }
    return item.received = bsp::to_status(result);
}

status start_rx_to_idle(handle instance, uint8_t* data, uint32_t size, uint32_t capacity,
                        rx_callback callback, TX_SEMAPHORE* notify) noexcept
{
    const auto i = index(instance);
    auto* native = peripherals::usart_handle(instance);
    if (i == count || native == nullptr)
    {
        return status::not_configured;
    }
    auto& item = contexts[i];
    if (!item.initialized)
    {
        return status::not_initialized;
    }
    const auto width = word_bytes(native);
    if (size % width != 0 || size / width > UINT16_MAX ||
        reinterpret_cast<uintptr_t>(data) % width != 0 || !memory::valid(data, size, capacity))
    {
        return status::invalid_arg;
    }
    auto* dma = native->hdmarx;
    if (!peripherals::usart_configs[i].dma.rx || dma == nullptr || dma->Instance == nullptr ||
        (native->Init.Mode & UART_MODE_RX) == 0 || dma->Init.Direction != DMA_PERIPH_TO_MEMORY ||
        dma->Init.PeriphDataAlignment !=
            (width == 1 ? DMA_PDATAALIGN_BYTE : DMA_PDATAALIGN_HALFWORD) ||
        dma->Init.MemDataAlignment !=
            (width == 1 ? DMA_MDATAALIGN_BYTE : DMA_MDATAALIGN_HALFWORD) ||
        dma->Init.MemInc != DMA_MINC_ENABLE)
    {
        return status::not_configured;
    }
    if (item.rx != nullptr)
    {
        if (item.rx != data || item.size != size || item.capacity != capacity ||
            item.callback != callback || item.notify != notify)
        {
            return status::busy;
        }
        return restart_rx(instance);
    }
    if (native->RxState != HAL_UART_STATE_READY)
    {
        return status::busy;
    }
    // Publish ownership before starting HAL, which can immediately invoke a callback.
    item.rx = data;
    item.size = size;
    item.capacity = capacity;
    item.callback = callback;
    item.notify = notify;
    const auto result = restart_rx(instance);
    if (result != status::ok && HAL_UART_AbortReceive(native) == HAL_OK)
    {
        const auto mask = __get_PRIMASK();
        __disable_irq();
        item.rx = nullptr;
        item.callback = {};
        item.notify = nullptr;
        __set_PRIMASK(mask);
    }
    return result;
}

status stop_rx(handle instance) noexcept
{
    const auto i = index(instance);
    auto* native = peripherals::usart_handle(instance);
    if (i == count || native == nullptr)
    {
        return status::not_configured;
    }
    auto& item = contexts[i];
    if (item.rx == nullptr)
    {
        return item.received = status::ok;
    }
    const auto result = HAL_UART_AbortReceive(native);
    if (result != HAL_OK)
    {
        return item.received = bsp::to_status(result);
    }
    bsp::invalidate_cache(item.rx, item.capacity);
    const auto mask = __get_PRIMASK();
    __disable_irq();
    item.rx = nullptr;
    item.size = item.capacity = item.position = 0;
    item.callback = {};
    item.notify = nullptr;
    item.received = status::ok;
    __set_PRIMASK(mask);
    return status::ok;
}

status rx_status(handle instance) noexcept
{
    const auto i = index(instance);
    return i != count ? contexts[i].received : status::not_configured;
}

static void notify(uint32_t i, uint32_t offset, uint32_t size) noexcept
{
    if (size == 0)
    {
        return;
    }
    auto& item = contexts[i];
    const auto start = reinterpret_cast<uintptr_t>(item.rx + offset);
    const auto aligned = start & ~(static_cast<uintptr_t>(memory::cache_line_size) - 1U);
    const auto end = (start + size + memory::cache_line_size - 1U) &
                     ~(static_cast<uintptr_t>(memory::cache_line_size) - 1U);
    bsp::invalidate_cache(reinterpret_cast<void*>(aligned), static_cast<uint32_t>(end - aligned));
    if (item.callback)
    {
        item.callback(peripherals::usart_configs[i].instance, item.rx + offset, size);
    }
    if (item.notify != nullptr)
    {
        (void)tx_semaphore_put(item.notify);
    }
}

static void receive(UART_HandleTypeDef* native, uint16_t words) noexcept
{
    const auto i = index(native);
    if (i == count)
    {
        return;
    }
    auto& item = contexts[i];
    const auto size = static_cast<uint32_t>(words) * word_bytes(native);
    if (item.rx == nullptr || size > item.size)
    {
        return;
    }
    const auto event = HAL_UARTEx_GetRxEventType(native);
    if (event == HAL_UART_RXEVENT_HT)
    {
        return;
    }
    if (native->hdmarx->Init.Mode != DMA_CIRCULAR)
    {
        notify(i, 0, size);
        (void)restart_rx(peripherals::usart_configs[i].instance);
        return;
    }
    const uint32_t position = size < item.size ? size : 0;
    if (event == HAL_UART_RXEVENT_TC)
    {
        notify(i, item.position, item.size - item.position);
        item.position = 0;
    }
    else
    {
        if (position < item.position)
        {
            notify(i, item.position, item.size - item.position);
            item.position = 0;
        }
        if (position > item.position)
        {
            notify(i, item.position, position - item.position);
        }
        item.position = position;
    }
}

static void error(UART_HandleTypeDef* native) noexcept
{
    const auto i = index(native);
    if (i == count || contexts[i].rx == nullptr)
    {
        return;
    }
    __HAL_UART_CLEAR_OREFLAG(native);
    const auto aborted = HAL_UART_AbortReceive(native);
    if (aborted != HAL_OK)
    {
        contexts[i].received = bsp::to_status(aborted);
        return;
    }
    (void)restart_rx(peripherals::usart_configs[i].instance);
}

static void transmitted(UART_HandleTypeDef* native) noexcept
{
    const auto i = index(native);
    if (i != count)
    {
        contexts[i].tx_busy = false;
    }
}
} // namespace bsp::usart

extern "C" void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef* native, uint16_t size)
{
    bsp::usart::receive(native, size);
}

extern "C" void HAL_UART_ErrorCallback(UART_HandleTypeDef* native)
{
    bsp::usart::error(native);
}

extern "C" void HAL_UART_TxCpltCallback(UART_HandleTypeDef* native)
{
    bsp::usart::transmitted(native);
}
