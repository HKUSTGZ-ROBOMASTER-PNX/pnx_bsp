#include "bsp.hpp"
#include "bsp_adc.hpp"
#include "peripherals.hpp"

#if defined(HAL_ADC_MODULE_ENABLED)
#include <cstring>

namespace bsp::adc
{

using pnxtypes::status;
static conversion_state current{};
static bool initialized = false;
enum class operation
{
    idle,
    polling,
    interrupt,
    dma
};
static volatile operation active = operation::idle;
static uint8_t* dma_data = nullptr;
static uint32_t dma_size = 0;
static uint32_t dma_capacity = 0;

static status ready(handle instance, bool require_idle = true) noexcept
{
    if (peripherals::adc_handle(instance) == nullptr)
    {
        return status::not_configured;
    }
    if (require_idle && active != operation::idle)
    {
        return status::busy;
    }
    return initialized ? status::ok : status::not_initialized;
}

static void begin(ADC_HandleTypeDef* adc, operation mode) noexcept
{
    const uint32_t management =
        mode == operation::dma ? ADC_CONVERSIONDATA_DMA_ONESHOT : ADC_CONVERSIONDATA_DR;
    adc->Init.ConversionDataManagement = management;
    MODIFY_REG(adc->Instance->CFGR, ADC_CFGR_DMNGT_0 | ADC_CFGR_DMNGT_1, management);
    current.complete = false;
    current.dma = mode == operation::dma;
    current.last_status = status::ok;
    current.busy = true;
    active = mode;
}

static status finish(HAL_StatusTypeDef value) noexcept
{
    current.last_status = bsp::to_status(value);
    current.busy = false;
    current.dma = false;
    active = operation::idle;
    dma_data = nullptr;
    dma_size = dma_capacity = 0;
    return current.last_status;
}

static status stop(handle instance, bool dma) noexcept
{
    if (const auto checked = ready(instance, false); checked != status::ok)
    {
        return checked;
    }
    if (dma ? active != operation::dma : active == operation::dma)
    {
        return dma ? status::invalid_arg : status::busy;
    }
    auto* adc = peripherals::adc_handle(instance);
    const auto stopped = dma ? HAL_ADC_Stop_DMA(adc) : HAL_ADC_Stop_IT(adc);
    if (stopped != HAL_OK)
    {
        return current.last_status = bsp::to_status(stopped);
    }
    if (dma)
    {
        SCB_InvalidateDCache_by_Addr(reinterpret_cast<uint32_t*>(dma_data),
                                     static_cast<int32_t>(dma_capacity));
    }
    return finish(stopped);
}

status init(const config& resource) noexcept
{
    auto* adc = peripherals::adc_handle(resource.instance);
    if (adc == nullptr)
    {
        return status::not_configured;
    }
    if (active != operation::idle)
    {
        return status::busy;
    }
    ADC_ChannelConfTypeDef channel_config{};
    channel_config.Channel = peripherals::adc_channel(resource.channel);
    channel_config.Rank = peripherals::adc_rank(resource.rank);
    channel_config.SamplingTime = peripherals::adc_sampling_time(resource.sampling);
    if (channel_config.Channel == UINT32_MAX || channel_config.Rank != ADC_REGULAR_RANK_1 ||
        channel_config.SamplingTime == UINT32_MAX)
    {
        return status::invalid_arg;
    }
    initialized = false;
    if (adc->Instance == nullptr || HAL_ADC_GetState(adc) == HAL_ADC_STATE_RESET)
    {
        return current.last_status = status::not_initialized;
    }
    if (adc->Init.NbrOfConversion != 1 || adc->Init.ScanConvMode != ADC_SCAN_DISABLE ||
        adc->Init.ExternalTrigConv != ADC_SOFTWARE_START ||
        adc->Init.ExternalTrigConvEdge != ADC_EXTERNALTRIGCONVEDGE_NONE)
    {
        return current.last_status = status::not_configured;
    }
    channel_config.SingleDiff = ADC_SINGLE_ENDED;
    channel_config.OffsetNumber = ADC_OFFSET_NONE;
    const auto value = HAL_ADC_ConfigChannel(adc, &channel_config);
    initialized = value == HAL_OK;
    current.complete = false;
    return finish(value);
}

status calibrate(handle instance) noexcept
{
    if (const auto checked = ready(instance); checked != status::ok)
    {
        return checked;
    }
    return finish(HAL_ADCEx_Calibration_Start(peripherals::adc_handle(instance), ADC_CALIB_OFFSET,
                                              ADC_SINGLE_ENDED));
}

status read_raw(handle instance, uint32_t& value, uint32_t timeout_ms) noexcept
{
    if (const auto checked = ready(instance); checked != status::ok)
    {
        return checked;
    }
    auto* adc = peripherals::adc_handle(instance);
    begin(adc, operation::polling);
    auto converted = HAL_ADC_Start(adc);
    if (converted != HAL_OK)
    {
        return finish(converted);
    }
    converted = HAL_ADC_PollForConversion(adc, timeout_ms);
    const auto raw = converted == HAL_OK ? HAL_ADC_GetValue(adc) : 0U;
    const auto stopped = HAL_ADC_Stop(adc);
    if (stopped != HAL_OK)
    {
        // stop_it retries polling/interrupt cleanup without releasing ownership.
        return current.last_status = bsp::to_status(stopped);
    }
    if (converted == HAL_OK)
    {
        value = raw;
        current.last_value = raw;
        current.complete = true;
    }
    return finish(converted);
}

status start_it(handle instance) noexcept
{
    if (const auto checked = ready(instance); checked != status::ok)
    {
        return checked;
    }
    if (NVIC_GetEnableIRQ(ADC_IRQn) == 0)
    {
        return status::not_configured;
    }
    auto* adc = peripherals::adc_handle(instance);
    begin(adc, operation::interrupt);
    const auto started = HAL_ADC_Start_IT(adc);
    return started == HAL_OK ? status::ok : finish(started);
}

status stop_it(handle instance) noexcept
{
    return stop(instance, false);
}

status start_dma(handle instance, uint8_t* data, uint32_t size, uint32_t capacity) noexcept
{
    if (const auto checked = ready(instance); checked != status::ok)
    {
        return checked;
    }
    const auto address = reinterpret_cast<uintptr_t>(data);
    if (size == 0 || size % sizeof(uint32_t) != 0 || size / sizeof(uint32_t) > 0xFFFFU ||
        capacity < size || capacity % memory::cache_line_size != 0 ||
        address % memory::cache_line_size != 0 || address < 0x24000000U || address >= 0x24050000U ||
        capacity > 0x24050000U - address)
    {
        return status::invalid_arg;
    }
    auto* adc = peripherals::adc_handle(instance);
    auto* dma = adc->DMA_Handle;
    if (dma == nullptr || dma->Instance == nullptr || dma->State != HAL_DMA_STATE_READY ||
        dma->Init.Mode != DMA_NORMAL || dma->Init.Direction != DMA_PERIPH_TO_MEMORY ||
        dma->Init.PeriphDataAlignment != DMA_PDATAALIGN_WORD ||
        dma->Init.MemDataAlignment != DMA_MDATAALIGN_WORD || dma->Init.MemInc != DMA_MINC_ENABLE ||
        (size > sizeof(uint32_t) && adc->Init.ContinuousConvMode != ENABLE))
    {
        return status::not_configured;
    }
    SCB_CleanInvalidateDCache_by_Addr(reinterpret_cast<uint32_t*>(data),
                                      static_cast<int32_t>(capacity));
    dma_data = data;
    dma_size = size;
    dma_capacity = capacity;
    begin(adc, operation::dma);
    const auto started =
        HAL_ADC_Start_DMA(adc, reinterpret_cast<uint32_t*>(data), size / sizeof(uint32_t));
    if (started == HAL_OK)
    {
        return status::ok;
    }
    current.last_status = bsp::to_status(started);
    // HAL may have partially started DMA; retain ownership if cleanup fails.
    const auto stopped = stop_dma(instance);
    if (stopped != status::ok)
    {
        return stopped;
    }
    current.complete = false;
    return current.last_status = bsp::to_status(started);
}

status stop_dma(handle instance) noexcept
{
    return stop(instance, true);
}

const conversion_state& state(handle instance) noexcept
{
    static const conversion_state unavailable{false, false, false, status::not_configured, 0};
    return peripherals::adc_handle(instance) != nullptr ? current : unavailable;
}

void handle_conversion_complete(ADC_HandleTypeDef* adc) noexcept
{
    if (adc == nullptr || adc != peripherals::adc_handle(handle::adc1) ||
        (active != operation::interrupt && active != operation::dma) ||
        current.last_status != status::ok)
    {
        return;
    }
    uint32_t value;
    if (active == operation::dma)
    {
        SCB_InvalidateDCache_by_Addr(reinterpret_cast<uint32_t*>(dma_data),
                                     static_cast<int32_t>(dma_capacity));
        std::memcpy(&value, dma_data + dma_size - sizeof(value), sizeof(value));
    }
    else
    {
        value = HAL_ADC_GetValue(adc);
    }
    current.last_value = value;
    current.busy = active == operation::interrupt && adc->Init.ContinuousConvMode == ENABLE;
    // Retain ownership until stop_it/stop_dma, including after one-shot completion.
    current.complete = true;
}

void handle_error(ADC_HandleTypeDef* adc) noexcept
{
    if (adc == nullptr || adc != peripherals::adc_handle(handle::adc1) ||
        (active != operation::interrupt && active != operation::dma))
    {
        return;
    }
    current.last_status = status::error;
    current.complete = true; // Stop explicitly before reusing ADC or DMA storage.
}
} // namespace bsp::adc

extern "C" void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* adc)
{
    bsp::adc::handle_conversion_complete(adc);
}

extern "C" void HAL_ADC_ErrorCallback(ADC_HandleTypeDef* adc)
{
    bsp::adc::handle_error(adc);
}
#endif
