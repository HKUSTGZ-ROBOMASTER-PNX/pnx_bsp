#include "bsp_can.hpp"
#include "bsp.hpp"
#include "can.h"
#include <cstring>

namespace bsp::can
{
namespace
{
using pnxtypes::status;
using native_handle = CAN_HandleTypeDef;

constexpr uint32_t notifications = CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_RX_FIFO0_OVERRUN |
                                   CAN_IT_ERROR_WARNING | CAN_IT_ERROR_PASSIVE | CAN_IT_BUSOFF |
                                   CAN_IT_LAST_ERROR_CODE | CAN_IT_ERROR;

enum class phase : uint8_t
{
    unprepared,
    stopped,
    started
};

struct controller
{
    native_handle* native = nullptr;
    config resource{};
    volatile phase lifecycle = phase::unprepared;
    volatile status last_status = status::not_initialized;
    bool semaphore_ready = false;
    TX_SEMAPHORE errors{};
    rx_callback callbacks[max_rx_callbacks]{};
};

controller controllers[]{{&hcan1, {handle::can1}}, {&hcan2, {handle::can2}}};

controller* find(handle instance) noexcept
{
    for (auto& item : controllers)
    {
        if (item.resource.instance == instance)
        {
            return &item;
        }
    }
    return nullptr;
}

controller* find(native_handle* native) noexcept
{
    for (auto& item : controllers)
    {
        if (item.native == native)
        {
            return &item;
        }
    }
    return nullptr;
}

status halt(controller& item) noexcept
{
    item.last_status = status::not_initialized; // Suppress IRQ dispatch before stopping.
    if (item.lifecycle != phase::started)
    {
        return status::ok;
    }
    auto result = HAL_CAN_DeactivateNotification(item.native, notifications);
    if (result == HAL_OK)
    {
        result = HAL_CAN_Stop(item.native);
    }
    const auto value = to_status(result);
    if (value != status::ok)
    {
        return item.last_status = value;
    }
    item.lifecycle = phase::stopped;
    return status::ok;
}

status start(controller& item) noexcept
{
    auto value = to_status(HAL_CAN_Start(item.native));
    if (value != status::ok)
    {
        return item.last_status = value;
    }
    // Publish readiness before notifications can deliver pending RX.
    item.lifecycle = phase::started;
    item.last_status = status::ok;
    value = to_status(HAL_CAN_ActivateNotification(item.native, notifications));
    if (value != status::ok)
    {
        (void)halt(item); // Failed stop retains ownership for the next retry.
        item.last_status = value;
    }
    return value;
}

void notify_error(native_handle* native, bool bus_off = false) noexcept
{
    auto* item = find(native);
    if (item == nullptr || item->lifecycle != phase::started)
    {
        return;
    }
    if (bus_off)
    {
        item->last_status = status::error;
    }
    if (item->semaphore_ready)
    {
        (void)tx_semaphore_put(&item->errors);
    }
}

void receive(native_handle* native) noexcept
{
    auto* item = find(native);
    if (item == nullptr || item->lifecycle != phase::started || item->last_status != status::ok)
    {
        return;
    }
    const auto pending = HAL_CAN_GetRxFifoFillLevel(native, CAN_RX_FIFO0);
    for (uint32_t i = 0; i < pending && i < 64U; ++i)
    {
        if (item->last_status != status::ok)
        {
            return;
        }
        rx_frame frame{};
        CAN_RxHeaderTypeDef header{};
        const auto value = HAL_CAN_GetRxMessage(native, CAN_RX_FIFO0, &header, frame.data);
        if (value == HAL_OK)
        {
            frame.id_kind = header.IDE == CAN_ID_EXT ? id_type::extended : id_type::standard;
            frame.id = frame.id_kind == id_type::extended ? header.ExtId : header.StdId;
            frame.len = header.DLC <= 8 ? static_cast<uint8_t>(header.DLC) : UINT8_MAX;
            frame.format = frame_mode::classic;
            frame.remote = header.RTR != CAN_RTR_DATA;
        }

        if (value != HAL_OK)
        {
            const bool bus_off = (native->ErrorCode & HAL_CAN_ERROR_BOF) != 0;
            notify_error(native, bus_off);
            return;
        }
        if (frame.remote || frame.id_kind != item->resource.filter || frame.len > 8)
        {
            continue;
        }
        for (auto callback : item->callbacks)
        {
            if (callback)
            {
                callback(item->resource.instance, frame);
            }
        }
    }
}
} // namespace

static status initialize(const config& resource, const fd_config* fd = nullptr) noexcept
{
    auto* item = find(resource.instance);
    if (item == nullptr)
    {
        return status::invalid_arg;
    }
    if (item->lifecycle == phase::started)
    {
        return status::busy;
    }
    if (resource.filter != id_type::standard && resource.filter != id_type::extended)
    {
        return status::invalid_arg;
    }
    item->lifecycle = phase::unprepared; // Failed preparation must be retried with init.
    auto* native = item->native;
    if (native->Instance == nullptr || HAL_CAN_GetState(native) != HAL_CAN_STATE_READY)
    {
        return item->last_status = status::not_initialized;
    }
    if (fd != nullptr &&
        (fd->tdc_offset > 127 || fd->tdc_filter > 127 || (fd->tdc_enabled && !fd->brs)))
    {
        return item->last_status = status::invalid_arg;
    }
    if (fd != nullptr)
    {
        return item->last_status = status::not_configured;
    }
    CAN_FilterTypeDef filter{};
    filter.FilterBank = native->Instance == CAN1 ? 0U : 14U;
    filter.SlaveStartFilterBank = 14U;
    filter.FilterMode = CAN_FILTERMODE_IDMASK;
    filter.FilterScale = CAN_FILTERSCALE_32BIT;
    filter.FilterIdLow = resource.filter == id_type::extended ? CAN_ID_EXT : CAN_ID_STD;
    filter.FilterMaskIdLow = CAN_ID_EXT | CAN_RTR_REMOTE;
    filter.FilterFIFOAssignment = CAN_RX_FIFO0;
    filter.FilterActivation = ENABLE;
    auto value = HAL_CAN_ConfigFilter(native, &filter);
    if (value != HAL_OK)
    {
        return item->last_status = to_status(value);
    }
    item->resource = resource;
    if (!item->semaphore_ready)
    {
        if (tx_semaphore_create(&item->errors, const_cast<CHAR*>("can_err"), 0) != TX_SUCCESS)
        {
            return item->last_status = status::error;
        }
        item->semaphore_ready = true;
    }
    item->lifecycle = phase::stopped;
    return start(*item);
}

status init(const config& resource) noexcept
{
    return initialize(resource);
}

status init(const fd_config& resource) noexcept
{
    return initialize({resource.instance, resource.filter}, &resource);
}

status stop(handle instance) noexcept
{
    auto* item = find(instance);
    if (item == nullptr)
    {
        return status::invalid_arg;
    }
    if (item->lifecycle == phase::unprepared)
    {
        return status::not_initialized;
    }
    return halt(*item);
}

status restart(handle instance) noexcept
{
    const auto value = stop(instance);
    return value == status::ok ? start(*find(instance)) : value;
}

status current_status(handle instance) noexcept
{
    auto* item = find(instance);
    if (item == nullptr)
    {
        return status::invalid_arg;
    }
    return item->last_status;
}

status transmit(handle instance, uint32_t id, const uint8_t* data, uint16_t len) noexcept
{
    auto* item = find(instance);
    if (item == nullptr || (data == nullptr && len != 0))
    {
        return status::invalid_arg;
    }
    if (item->last_status != status::ok)
    {
        return item->last_status;
    }
    const auto& resource = item->resource;
    if (id > (resource.filter == id_type::extended ? 0x1FFFFFFFU : 0x7FFU) || len > 8)
    {
        return status::invalid_arg;
    }
    auto* native = item->native;
    uint8_t payload[8]{};
    if (len != 0)
    {
        std::memcpy(payload, data, len);
    }
    if (HAL_CAN_GetTxMailboxesFreeLevel(native) == 0)
    {
        return status::busy;
    }
    CAN_TxHeaderTypeDef header{};
    header.StdId = resource.filter == id_type::standard ? id : 0;
    header.ExtId = resource.filter == id_type::extended ? id : 0;
    header.IDE = resource.filter == id_type::extended ? CAN_ID_EXT : CAN_ID_STD;
    header.RTR = CAN_RTR_DATA;
    header.DLC = len;
    header.TransmitGlobalTime = DISABLE;
    uint32_t mailbox;
    return to_status(HAL_CAN_AddTxMessage(native, &header, payload, &mailbox));
}

status register_rx_callback(handle instance, rx_callback callback) noexcept
{
    auto* item = find(instance);
    if (item == nullptr || !callback)
    {
        return status::invalid_arg;
    }
    if (item->lifecycle == phase::started)
    {
        return status::busy;
    }
    for (auto& slot : item->callbacks)
    {
        if (!slot)
        {
            slot = callback;
            return status::ok;
        }
    }
    return status::busy;
}

status unregister_rx_callbacks(handle instance) noexcept
{
    auto* item = find(instance);
    if (item == nullptr)
    {
        return status::invalid_arg;
    }
    if (item->lifecycle == phase::started)
    {
        return status::busy;
    }
    for (auto& slot : item->callbacks)
    {
        slot = {};
    }
    return status::ok;
}

TX_SEMAPHORE* err_sem(handle instance) noexcept
{
    auto* item = find(instance);
    return item != nullptr && item->semaphore_ready ? &item->errors : nullptr;
}
} // namespace bsp::can

extern "C" void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef* native)
{
    bsp::can::receive(native);
}

extern "C" void HAL_CAN_ErrorCallback(CAN_HandleTypeDef* native)
{
    if (native != nullptr)
    {
        bsp::can::notify_error(native, (native->ErrorCode & HAL_CAN_ERROR_BOF) != 0);
    }
}
