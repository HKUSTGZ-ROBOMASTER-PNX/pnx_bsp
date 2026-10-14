#include "bsp_usb.hpp"
#include "bsp.hpp"
#include "usb_otg.h"
#include "ux_api.h"
#include "ux_device_stack.h"
#include "ux_device_class_cdc_acm.h"
#include "ux_device_descriptors.h"

#include <atomic>

// The STM32 DCD header contains C-only inline code; this is its C entry point.
extern "C" UINT _ux_dcd_stm32_initialize(ULONG, ULONG);

namespace bsp::usb
{
using pnxtypes::status;

// Both boards currently use a full-speed embedded PHY with PCD DMA disabled.
alignas(ULONG) static uint8_t stack_memory[16 * 1024];
static UX_SLAVE_CLASS_CDC_ACM_PARAMETER parameters{};
static std::atomic<UX_SLAVE_CLASS_CDC_ACM*> cdc{nullptr};
static std::atomic_flag reading = ATOMIC_FLAG_INIT;
static std::atomic_flag writing = ATOMIC_FLAG_INIT;
static std::atomic_flag polling = ATOMIC_FLAG_INIT;
static rx_callback receiver{};
static bool attempted = false;
static status init_status = status::not_initialized;

static void activate(void* instance)
{
    cdc.store(static_cast<UX_SLAVE_CLASS_CDC_ACM*>(instance));
}

static void deactivate(void*)
{
    cdc.store(nullptr);
}

static status result(UINT value) noexcept
{
    switch (value)
    {
        case UX_SUCCESS:
            return status::ok;
        case UX_TRANSFER_TIMEOUT:
            return status::timeout;
        case UX_CONFIGURATION_HANDLE_UNKNOWN:
        case UX_TRANSFER_NO_ANSWER:
            return status::not_initialized;
        default:
            return status::error;
    }
}

static status finish(UX_SLAVE_CLASS_CDC_ACM* instance, ULONG direction, UINT value) noexcept
{
    // These STM32 DCD versions return ThreadX's semaphore timeout directly.
    if (value != TX_NO_INSTANCE && value != UX_TRANSFER_TIMEOUT)
    {
        return result(value);
    }
    const auto mask = __get_PRIMASK();
    __disable_irq();
    if (cdc.load() == instance)
    {
        auto* endpoint =
            instance->ux_slave_class_cdc_acm_interface->ux_slave_interface_first_endpoint;
        while (endpoint != nullptr)
        {
            if ((endpoint->ux_slave_endpoint_descriptor.bEndpointAddress & UX_ENDPOINT_DIRECTION) ==
                direction)
            {
                auto& transfer = endpoint->ux_slave_endpoint_transfer_request;
                ux_device_stack_transfer_abort(&transfer, UX_TRANSFER_TIMEOUT);
                // Abort/completion may have posted a token after the timed wait ended.
                while (tx_semaphore_get(&transfer.ux_slave_transfer_request_semaphore,
                                        TX_NO_WAIT) == TX_SUCCESS)
                {
                }
                break;
            }
            endpoint = endpoint->ux_slave_endpoint_next_endpoint;
        }
    }
    __set_PRIMASK(mask);
    return status::timeout;
}

status init() noexcept
{
    if (attempted)
    {
        return init_status;
    }
#if defined(STM32H723xx)
    auto* pcd = &hpcd_USB_OTG_HS;
#elif defined(STM32F407xx)
    auto* pcd = &hpcd_USB_OTG_FS;
#else
#error Unsupported USB controller
#endif
    if (pcd->Instance == nullptr || pcd->State != HAL_PCD_STATE_READY)
    {
        return status::not_initialized;
    }
    if (pcd->Init.speed != PCD_SPEED_FULL || pcd->Init.dma_enable != DISABLE)
    {
        return status::not_configured;
    }

    // A partial stack initialization cannot safely be repeated; startup failure is fatal.
    attempted = true;
    init_status = status::error;
    if (ux_system_initialize(stack_memory, sizeof(stack_memory), nullptr, 0) != UX_SUCCESS)
    {
        return init_status;
    }

    ULONG framework_length, string_length, language_length;
    auto* framework = USBD_Get_Device_Framework_Speed(USBD_FULL_SPEED, &framework_length);
    auto* strings = USBD_Get_String_Framework(&string_length);
    auto* languages = USBD_Get_Language_Id_Framework(&language_length);
    if (ux_device_stack_initialize(nullptr, 0, framework, framework_length, strings, string_length,
                                   languages, language_length, nullptr) != UX_SUCCESS)
    {
        return init_status;
    }

    parameters.ux_slave_class_cdc_acm_instance_activate = activate;
    parameters.ux_slave_class_cdc_acm_instance_deactivate = deactivate;
    // The generated descriptors on both boards contain one CDC class: configuration 1, interface 0.
    if (ux_device_stack_class_register(_ux_system_slave_class_cdc_acm_name,
                                       ux_device_class_cdc_acm_entry, 1, 0,
                                       &parameters) != UX_SUCCESS)
    {
        return init_status;
    }

    // FIFO sizes are 32-bit words. Program in ascending order: HAL sums earlier FIFOs.
    // F407 uses IN endpoints 2/3, H723 uses 1/2. Reserve 16 words for gaps.
    constexpr auto data_ep = USBD_CDCACM_EPIN_ADDR & 0x7FU;
    constexpr auto command_ep = USBD_CDCACM_EPINCMD_ADDR & 0x7FU;
    constexpr auto last_ep = data_ep > command_ep ? data_ep : command_ep;
    if (last_ep >= pcd->Init.dev_endpoints || HAL_PCDEx_SetRxFiFo(pcd, 128) != HAL_OK)
    {
        return init_status;
    }
    for (uint16_t ep = 0; ep <= last_ep; ++ep)
    {
        if (HAL_PCDEx_SetTxFiFo(pcd, ep, ep == 0 || ep == data_ep ? 64 : 16) != HAL_OK)
        {
            return init_status;
        }
    }
    if (_ux_dcd_stm32_initialize(reinterpret_cast<ULONG>(pcd->Instance),
                                 reinterpret_cast<ULONG>(pcd)) != UX_SUCCESS)
    {
        return init_status;
    }
    init_status = bsp::to_status(HAL_PCD_Start(pcd));
    return init_status;
}

bool connected() noexcept
{
    return cdc.load() != nullptr;
}

status on_rx(rx_callback callback) noexcept
{
    if (polling.test_and_set())
    {
        return status::busy;
    }
    receiver = callback;
    polling.clear();
    return status::ok;
}

status poll(uint32_t timeout_ms) noexcept
{
    if (polling.test_and_set())
    {
        return status::busy;
    }
    if (!receiver)
    {
        polling.clear();
        return status::not_configured;
    }
    uint8_t data[USBD_CDCACM_EPOUT_FS_MPS];
    uint32_t received = 0;
    const auto completed = read(data, sizeof(data), received, timeout_ms);
    if (received != 0)
    {
        receiver(data, received);
    }
    polling.clear();
    return completed;
}

status read(uint8_t* data, uint32_t size, uint32_t& transferred, uint32_t timeout_ms) noexcept
{
    transferred = 0;
    if (data == nullptr || size == 0 || size % USBD_CDCACM_EPOUT_FS_MPS != 0)
    {
        return status::invalid_arg;
    }
    if (reading.test_and_set())
    {
        return status::busy;
    }
    auto* instance = cdc.load();
    if (instance == nullptr)
    {
        reading.clear();
        return status::not_initialized;
    }
    const ULONG ticks =
        (static_cast<uint64_t>(timeout_ms) * TX_TIMER_TICKS_PER_SECOND + 999) / 1000;
    auto value = ux_device_class_cdc_acm_ioctl(
        instance, UX_SLAVE_CLASS_CDC_ACM_IOCTL_SET_READ_TIMEOUT, reinterpret_cast<void*>(ticks));
    ULONG actual = 0;
    if (value == UX_SUCCESS)
    {
        value = ux_device_class_cdc_acm_read(instance, data, size, &actual);
    }
    transferred = actual;
    const auto completed = finish(instance, UX_ENDPOINT_OUT, value);
    reading.clear();
    return completed;
}

status write(const uint8_t* data, uint32_t size, uint32_t& transferred,
             uint32_t timeout_ms) noexcept
{
    transferred = 0;
    if (data == nullptr && size != 0)
    {
        return status::invalid_arg;
    }
    if (writing.test_and_set())
    {
        return status::busy;
    }
    auto* instance = cdc.load();
    if (instance == nullptr)
    {
        writing.clear();
        return status::not_initialized;
    }
    const ULONG ticks =
        (static_cast<uint64_t>(timeout_ms) * TX_TIMER_TICKS_PER_SECOND + 999) / 1000;
    auto value = ux_device_class_cdc_acm_ioctl(
        instance, UX_SLAVE_CLASS_CDC_ACM_IOCTL_SET_WRITE_TIMEOUT, reinterpret_cast<void*>(ticks));
    ULONG actual = 0;
    if (value == UX_SUCCESS)
    {
        value = ux_device_class_cdc_acm_write(instance, const_cast<uint8_t*>(data), size, &actual);
    }
    transferred = actual;
    const auto completed = finish(instance, UX_ENDPOINT_IN, value);
    writing.clear();
    return completed;
}
} // namespace bsp::usb
