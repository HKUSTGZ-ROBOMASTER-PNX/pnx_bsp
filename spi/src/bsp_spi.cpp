#include "bsp_spi.hpp"
#include "bsp.hpp"
#include "peripherals.hpp"

namespace bsp::spi
{
using pnxtypes::status;
static constexpr uint32_t count = sizeof(peripherals::spi_configs) / sizeof(config);

struct context
{
    transfer_state state{};
    uint8_t* rx = nullptr;
    uint32_t capacity = 0;
};

static context contexts[count]{};

static uint32_t index(handle instance) noexcept
{
    for (uint32_t i = 0; i < count; ++i)
    {
        if (peripherals::spi_configs[i].instance == instance)
        {
            return i;
        }
    }
    return count;
}

static status ready(handle instance, uint32_t size, bool interrupt = false) noexcept
{
    const auto i = index(instance);
    auto* native = peripherals::spi_handle(instance);
    if (i == count || native == nullptr)
    {
        return status::not_configured;
    }
    if (native->Instance == nullptr || native->State == HAL_SPI_STATE_RESET)
    {
        return status::not_initialized;
    }
    if (contexts[i].state.busy || native->State != HAL_SPI_STATE_READY)
    {
        return status::busy;
    }
    if (size == 0 || size > UINT16_MAX)
    {
        return status::invalid_arg;
    }
    if (native->Init.DataSize != SPI_DATASIZE_8BIT ||
        (interrupt && !peripherals::spi_configs[i].irq))
    {
        return status::not_configured;
    }
    return status::ok;
}

static void begin(context& item, bool receive, bool dma, uint8_t* rx = nullptr,
                  uint32_t capacity = 0) noexcept
{
    item.rx = rx;
    item.capacity = capacity;
    item.state = {true, false, receive, dma, status::ok};
}

static status finish(context& item, HAL_StatusTypeDef result) noexcept
{
    if (item.rx != nullptr && item.state.dma)
    {
        bsp::invalidate_cache(item.rx, item.capacity);
    }
    item.rx = nullptr;
    item.capacity = 0;
    item.state.last_status = bsp::to_status(result);
    item.state.complete = true;
    item.state.busy = false;
    return item.state.last_status;
}

static status started(handle instance, HAL_StatusTypeDef result) noexcept
{
    if (result == HAL_OK)
    {
        return status::ok;
    }
    auto& item = contexts[index(instance)];
    item.state.last_status = bsp::to_status(result);
    const auto stopped = HAL_SPI_Abort(peripherals::spi_handle(instance));
    if (stopped != HAL_OK)
    {
        return item.state.last_status = bsp::to_status(stopped);
    }
    return finish(item, result);
}

static bool dma_ready(DMA_HandleTypeDef* dma, bool receive) noexcept
{
    return dma != nullptr && dma->Instance != nullptr && dma->Init.Mode == DMA_NORMAL &&
           dma->Init.Direction == (receive ? DMA_PERIPH_TO_MEMORY : DMA_MEMORY_TO_PERIPH) &&
           dma->Init.PeriphDataAlignment == DMA_PDATAALIGN_BYTE &&
           dma->Init.MemDataAlignment == DMA_MDATAALIGN_BYTE && dma->Init.MemInc == DMA_MINC_ENABLE;
}

static status prepare_dma(handle instance, bool receive, const void* data, uint32_t size,
                          uint32_t capacity) noexcept
{
    if (auto result = ready(instance, size); result != status::ok)
    {
        return result;
    }
    if (!memory::valid(data, size, capacity))
    {
        return status::invalid_arg;
    }
    auto* native = peripherals::spi_handle(instance);
    const auto& resource = peripherals::spi_configs[index(instance)];
    if (!(receive ? resource.dma.rx : resource.dma.tx) ||
        !dma_ready(receive ? native->hdmarx : native->hdmatx, receive))
    {
        return status::not_configured;
    }
#if defined(STM32F407xx)
    // F4 full-duplex master receive also needs TX DMA to generate the clocks.
    if (receive && native->Init.Mode == SPI_MODE_MASTER &&
        native->Init.Direction == SPI_DIRECTION_2LINES &&
        (!resource.dma.tx || !dma_ready(native->hdmatx, false)))
    {
        return status::not_configured;
    }
#endif
    return status::ok;
}

status transmit(handle instance, const uint8_t* data, uint32_t size, uint32_t timeout) noexcept
{
    if (data == nullptr)
    {
        return status::invalid_arg;
    }
    if (auto result = ready(instance, size); result != status::ok)
    {
        return result;
    }
    auto& item = contexts[index(instance)];
    begin(item, false, false);
    return finish(item,
                  HAL_SPI_Transmit(peripherals::spi_handle(instance), const_cast<uint8_t*>(data),
                                   static_cast<uint16_t>(size), timeout));
}

status receive(handle instance, uint8_t* data, uint32_t size, uint32_t timeout) noexcept
{
    if (data == nullptr)
    {
        return status::invalid_arg;
    }
    if (auto result = ready(instance, size); result != status::ok)
    {
        return result;
    }
    auto& item = contexts[index(instance)];
    begin(item, true, false);
    return finish(item, HAL_SPI_Receive(peripherals::spi_handle(instance), data,
                                        static_cast<uint16_t>(size), timeout));
}

status transmit_receive(handle instance, const uint8_t* tx, uint8_t* rx, uint32_t size,
                        uint32_t timeout) noexcept
{
    if (tx == nullptr || rx == nullptr)
    {
        return status::invalid_arg;
    }
    if (auto result = ready(instance, size); result != status::ok)
    {
        return result;
    }
    auto& item = contexts[index(instance)];
    begin(item, true, false);
    return finish(item, HAL_SPI_TransmitReceive(peripherals::spi_handle(instance),
                                                const_cast<uint8_t*>(tx), rx,
                                                static_cast<uint16_t>(size), timeout));
}

status transmit_it(handle instance, const uint8_t* data, uint32_t size) noexcept
{
    if (data == nullptr)
    {
        return status::invalid_arg;
    }
    if (auto result = ready(instance, size, true); result != status::ok)
    {
        return result;
    }
    begin(contexts[index(instance)], false, false);
    return started(instance,
                   HAL_SPI_Transmit_IT(peripherals::spi_handle(instance),
                                       const_cast<uint8_t*>(data), static_cast<uint16_t>(size)));
}

status receive_it(handle instance, uint8_t* data, uint32_t size) noexcept
{
    if (data == nullptr)
    {
        return status::invalid_arg;
    }
    if (auto result = ready(instance, size, true); result != status::ok)
    {
        return result;
    }
    begin(contexts[index(instance)], true, false);
    return started(instance, HAL_SPI_Receive_IT(peripherals::spi_handle(instance), data,
                                                static_cast<uint16_t>(size)));
}

status transmit_dma(handle instance, const uint8_t* data, uint32_t size, uint32_t capacity) noexcept
{
    if (auto result = prepare_dma(instance, false, data, size, capacity); result != status::ok)
    {
        return result;
    }
    bsp::clean_cache(data, capacity);
    begin(contexts[index(instance)], false, true);
    return started(instance,
                   HAL_SPI_Transmit_DMA(peripherals::spi_handle(instance),
                                        const_cast<uint8_t*>(data), static_cast<uint16_t>(size)));
}

status receive_dma(handle instance, uint8_t* data, uint32_t size, uint32_t capacity) noexcept
{
    if (auto result = prepare_dma(instance, true, data, size, capacity); result != status::ok)
    {
        return result;
    }
    bsp::prepare_rx_cache(data, capacity);
    begin(contexts[index(instance)], true, true, data, capacity);
    return started(instance, HAL_SPI_Receive_DMA(peripherals::spi_handle(instance), data,
                                                 static_cast<uint16_t>(size)));
}

status abort(handle instance) noexcept
{
    const auto i = index(instance);
    auto* native = peripherals::spi_handle(instance);
    if (i == count || native == nullptr)
    {
        return status::not_configured;
    }
    if (native->Instance == nullptr || native->State == HAL_SPI_STATE_RESET)
    {
        return status::not_initialized;
    }
    const auto result = HAL_SPI_Abort(native);
    if (result != HAL_OK)
    {
        return contexts[i].state.last_status = bsp::to_status(result);
    }
    return finish(contexts[i], result);
}

status wait_ready(handle instance, uint32_t timeout) noexcept
{
    const auto i = index(instance);
    auto* native = peripherals::spi_handle(instance);
    if (i == count || native == nullptr)
    {
        return status::not_configured;
    }
    if (native->Instance == nullptr || native->State == HAL_SPI_STATE_RESET)
    {
        return status::not_initialized;
    }
    const auto start = HAL_GetTick();
    while (contexts[i].state.busy || native->State != HAL_SPI_STATE_READY)
    {
        if (timeout != HAL_MAX_DELAY && HAL_GetTick() - start >= timeout)
        {
            return status::timeout;
        }
    }
    return status::ok;
}

const transfer_state& state(handle instance) noexcept
{
    static const transfer_state unavailable{false, false, false, false, status::not_configured};
    const auto i = index(instance);
    return i != count ? contexts[i].state : unavailable;
}

static void complete(SPI_HandleTypeDef* native, bool success) noexcept
{
    for (uint32_t i = 0; i < count; ++i)
    {
        auto& item = contexts[i];
        if (native == nullptr ||
            peripherals::spi_handle(peripherals::spi_configs[i].instance) != native ||
            !item.state.busy)
        {
            continue;
        }
        if (!success)
        {
            item.state.last_status = status::error;
            item.state.complete = true;
        }
        else if (item.state.last_status == status::ok)
        {
            finish(item, HAL_OK);
        }
        return;
    }
}
} // namespace bsp::spi

extern "C" void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef* native)
{
    bsp::spi::complete(native, true);
}

extern "C" void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef* native)
{
    bsp::spi::complete(native, true);
}

extern "C" void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef* native)
{
    bsp::spi::complete(native, true);
}

extern "C" void HAL_SPI_ErrorCallback(SPI_HandleTypeDef* native)
{
    bsp::spi::complete(native, false);
}
