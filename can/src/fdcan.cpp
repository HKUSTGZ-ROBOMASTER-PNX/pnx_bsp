#include "bsp_can.hpp"
#include "bsp.hpp"
#include "fdcan.h"
#include <cstring>

namespace bsp::can
{
namespace
{
using pnxtypes::status;
using native_handle = FDCAN_HandleTypeDef;

constexpr uint32_t error_events = FDCAN_IT_ERROR_WARNING | FDCAN_IT_ERROR_PASSIVE |
                                  FDCAN_IT_ARB_PROTOCOL_ERROR | FDCAN_IT_DATA_PROTOCOL_ERROR |
                                  FDCAN_IT_BUS_OFF;
constexpr uint32_t notifications = error_events | FDCAN_IT_RX_FIFO0_NEW_MESSAGE |
                                   FDCAN_IT_RX_FIFO1_NEW_MESSAGE | FDCAN_IT_RX_FIFO0_MESSAGE_LOST |
                                   FDCAN_IT_RX_FIFO1_MESSAGE_LOST;
constexpr uint8_t wire_lengths[]{0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64};

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
    frame_mode mode = frame_mode::classic;
    uint32_t fifo = FDCAN_RX_FIFO0;
    uint32_t rx_capacity = 0;
    uint32_t tx_capacity = 0;
    volatile phase lifecycle = phase::unprepared;
    volatile status last_status = status::not_initialized;
    bool semaphore_ready = false;
    TX_SEMAPHORE errors{};
    rx_callback callbacks[max_rx_callbacks]{};
};

controller controllers[]{
    {&hfdcan1, {handle::fdcan1}}, {&hfdcan2, {handle::fdcan2}}, {&hfdcan3, {handle::fdcan3}}};

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
    auto result = HAL_FDCAN_DeactivateNotification(item.native, notifications);
    if (result == HAL_OK)
    {
        result = HAL_FDCAN_Stop(item.native);
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
    auto value = to_status(HAL_FDCAN_Start(item.native));
    if (value != status::ok)
    {
        return item.last_status = value;
    }
    // Publish readiness before notifications can deliver pending RX.
    item.lifecycle = phase::started;
    item.last_status = status::ok;
    const auto rx_events = item.fifo == FDCAN_RX_FIFO0
                               ? FDCAN_IT_RX_FIFO0_NEW_MESSAGE | FDCAN_IT_RX_FIFO0_MESSAGE_LOST
                               : FDCAN_IT_RX_FIFO1_NEW_MESSAGE | FDCAN_IT_RX_FIFO1_MESSAGE_LOST;
    value = to_status(HAL_FDCAN_ActivateNotification(item.native, rx_events | error_events, 0));
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
    const auto pending = HAL_FDCAN_GetRxFifoFillLevel(native, item->fifo);
    for (uint32_t i = 0; i < pending && i < 64U; ++i)
    {
        if (item->last_status != status::ok)
        {
            return;
        }
        rx_frame frame{};
        FDCAN_RxHeaderTypeDef header{};
        const auto value = HAL_FDCAN_GetRxMessage(native, item->fifo, &header, frame.data);
        if (value == HAL_OK)
        {
            frame.id = header.Identifier;
            frame.len = header.DataLength <= FDCAN_DLC_BYTES_64 ? wire_lengths[header.DataLength]
                                                                : UINT8_MAX;
            frame.id_kind =
                header.IdType == FDCAN_EXTENDED_ID ? id_type::extended : id_type::standard;
            frame.format = header.FDFormat != FDCAN_FD_CAN        ? frame_mode::classic
                           : header.BitRateSwitch == FDCAN_BRS_ON ? frame_mode::fd_brs
                                                                  : frame_mode::fd_no_brs;
            frame.remote = header.RxFrameType != FDCAN_DATA_FRAME;
        }

        if (value != HAL_OK)
        {
            FDCAN_ProtocolStatusTypeDef protocol{};
            const bool bus_off =
                HAL_FDCAN_GetProtocolStatus(native, &protocol) == HAL_OK && protocol.BusOff != 0;
            notify_error(native, bus_off);
            return;
        }
        if (frame.remote || frame.id_kind != item->resource.filter ||
            (frame.format == frame_mode::classic && frame.len > 8) ||
            (item->mode == frame_mode::classic && frame.format != frame_mode::classic))
        {
            continue;
        }
        if (frame.len > item->rx_capacity)
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
    if (native->Instance == nullptr || native->State != HAL_FDCAN_STATE_READY)
    {
        return item->last_status = status::not_initialized;
    }
    if (fd != nullptr &&
        (fd->tdc_offset > 127 || fd->tdc_filter > 127 || (fd->tdc_enabled && !fd->brs)))
    {
        return item->last_status = status::invalid_arg;
    }
    const auto mode = fd == nullptr ? frame_mode::classic
                      : fd->brs     ? frame_mode::fd_brs
                                    : frame_mode::fd_no_brs;
    const auto& init = native->Init;
    if ((mode != frame_mode::classic && init.FrameFormat == FDCAN_FRAME_CLASSIC) ||
        (mode == frame_mode::fd_brs && init.FrameFormat != FDCAN_FRAME_FD_BRS) ||
        init.TxFifoQueueElmtsNbr == 0 ||
        (resource.filter == id_type::standard ? init.StdFiltersNbr : init.ExtFiltersNbr) == 0 ||
        ((init.RxFifo0ElmtsNbr > 0) == (init.RxFifo1ElmtsNbr > 0)))
    {
        return item->last_status = status::not_configured;
    }
    item->fifo = init.RxFifo0ElmtsNbr > 0 ? FDCAN_RX_FIFO0 : FDCAN_RX_FIFO1;
    // HAL message RAM sizes include two header words.
    const auto rx_words =
        item->fifo == FDCAN_RX_FIFO0 ? init.RxFifo0ElmtSize : init.RxFifo1ElmtSize;
    item->rx_capacity = rx_words >= 2 ? (rx_words - 2U) * 4U : 0;
    item->tx_capacity = init.TxElmtSize >= 2 ? (init.TxElmtSize - 2U) * 4U : 0;
    FDCAN_FilterTypeDef filter{};
    filter.IdType = resource.filter == id_type::extended ? FDCAN_EXTENDED_ID : FDCAN_STANDARD_ID;
    filter.FilterType = FDCAN_FILTER_MASK;
    filter.FilterConfig =
        item->fifo == FDCAN_RX_FIFO0 ? FDCAN_FILTER_TO_RXFIFO0 : FDCAN_FILTER_TO_RXFIFO1;
    auto value = HAL_FDCAN_ConfigFilter(native, &filter);
    if (value == HAL_OK)
    {
        value = HAL_FDCAN_ConfigGlobalFilter(native, FDCAN_REJECT, FDCAN_REJECT,
                                             FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE);
    }
    if (value == HAL_OK)
    {
        if (fd != nullptr && fd->tdc_enabled)
        {
            value = HAL_FDCAN_ConfigTxDelayCompensation(native, fd->tdc_offset, fd->tdc_filter);
            if (value == HAL_OK)
            {
                value = HAL_FDCAN_EnableTxDelayCompensation(native);
            }
        }
        else
        {
            value = HAL_FDCAN_DisableTxDelayCompensation(native);
        }
    }
    if (value != HAL_OK)
    {
        return item->last_status = to_status(value);
    }
    item->resource = resource;
    item->mode = mode;
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
    if (id > (resource.filter == id_type::extended ? 0x1FFFFFFFU : 0x7FFU) ||
        len > (item->mode == frame_mode::classic ? 8U : 64U))
    {
        return status::invalid_arg;
    }
    auto* native = item->native;
    uint8_t payload[64]{};
    if (len != 0)
    {
        std::memcpy(payload, data, len);
    }
    uint32_t dlc = 0;
    while (wire_lengths[dlc] < len)
    {
        ++dlc;
    }
    if (wire_lengths[dlc] > item->tx_capacity)
    {
        return status::invalid_arg;
    }
    if (HAL_FDCAN_GetTxFifoFreeLevel(native) == 0)
    {
        return status::busy;
    }
    FDCAN_TxHeaderTypeDef header{};
    header.Identifier = id;
    header.IdType = resource.filter == id_type::extended ? FDCAN_EXTENDED_ID : FDCAN_STANDARD_ID;
    header.TxFrameType = FDCAN_DATA_FRAME;
    header.DataLength = dlc;
    header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    header.BitRateSwitch = item->mode == frame_mode::fd_brs ? FDCAN_BRS_ON : FDCAN_BRS_OFF;
    header.FDFormat = item->mode == frame_mode::classic ? FDCAN_CLASSIC_CAN : FDCAN_FD_CAN;
    header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    return to_status(HAL_FDCAN_AddMessageToTxFifoQ(native, &header, payload));
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

extern "C" void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef* native, uint32_t events)
{
    if (events & FDCAN_IT_RX_FIFO0_MESSAGE_LOST)
    {
        bsp::can::notify_error(native);
    }
    auto* item = bsp::can::find(native);
    if (item != nullptr && item->fifo == FDCAN_RX_FIFO0 && (events & FDCAN_IT_RX_FIFO0_NEW_MESSAGE))
    {
        bsp::can::receive(native);
    }
}

extern "C" void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef* native, uint32_t events)
{
    if (events & FDCAN_IT_RX_FIFO1_MESSAGE_LOST)
    {
        bsp::can::notify_error(native);
    }
    auto* item = bsp::can::find(native);
    if (item != nullptr && item->fifo == FDCAN_RX_FIFO1 && (events & FDCAN_IT_RX_FIFO1_NEW_MESSAGE))
    {
        bsp::can::receive(native);
    }
}

extern "C" void HAL_FDCAN_ErrorCallback(FDCAN_HandleTypeDef* native)
{
    bsp::can::notify_error(native);
}

extern "C" void HAL_FDCAN_ErrorStatusCallback(FDCAN_HandleTypeDef* native, uint32_t events)
{
    if (events & bsp::can::error_events)
    {
        bsp::can::notify_error(native, (events & FDCAN_IT_BUS_OFF) != 0);
    }
}
