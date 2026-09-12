#include "bsp_can.hpp"
#include "bsp_can_binding.hpp"

#include <array>

namespace bsp::can
{
namespace
{

struct rx_slot
{
    rx_callback callback{};
};

std::array<bool, bus_count> initialized{};
std::array<std::array<rx_slot, max_rx_callbacks>, bus_count> rx_slots{};
std::array<TX_SEMAPHORE, bus_count> err_sems{};
std::array<bool, bus_count> err_sem_ready{};

const bus_config* config_of(std::size_t index) noexcept
{
    return index < bus_count ? &configs[index] : nullptr;
}

types::status ensure_err_sem(bus selected)
{
    const std::size_t index = static_cast<std::size_t>(selected);
    if (index >= bus_count)
    {
        return types::status::invalid_arg;
    }
    if (err_sem_ready[index])
    {
        return types::status::ok;
    }
    if (tx_semaphore_create(&err_sems[index], const_cast<CHAR*>("can_err"), 0U) != TX_SUCCESS)
    {
        return types::status::error;
    }
    err_sem_ready[index] = true;
    return types::status::ok;
}

void notify_rx(bus selected, const rx_frame& frame)
{
    const std::size_t index = static_cast<std::size_t>(selected);
    if (index >= bus_count)
    {
        return;
    }
    for (const auto& slot : rx_slots[index])
    {
        if (slot.callback)
        {
            slot.callback(selected, frame);
        }
    }
}

constexpr std::uint32_t notifications =
    CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_RX_FIFO0_OVERRUN |
    CAN_IT_ERROR_WARNING | CAN_IT_ERROR_PASSIVE | CAN_IT_BUSOFF |
    CAN_IT_LAST_ERROR_CODE | CAN_IT_ERROR;

types::status start_hardware(bus selected)
{
    CAN_HandleTypeDef* handle = handle_of(selected);
    if (handle == nullptr)
    {
        return types::status::not_configured;
    }
    return HAL_CAN_Start(handle) == HAL_OK &&
                   HAL_CAN_ActivateNotification(handle, notifications) == HAL_OK
               ? types::status::ok
               : types::status::error;
}

} // namespace

CAN_HandleTypeDef* handle_of(bus selected) noexcept
{
    const bus_config* cfg = config_of(static_cast<std::size_t>(selected));
    if (cfg == nullptr || !cfg->enabled)
    {
        return nullptr;
    }
    detail::binding binding{};
    return detail::binding_for(selected, binding) ? binding.can : nullptr;
}

bus bus_of(CAN_HandleTypeDef* handle) noexcept
{
    for (std::size_t index = 0; index < bus_count; ++index)
    {
        const bus selected = static_cast<bus>(index);
        if (handle_of(selected) == handle)
        {
            return selected;
        }
    }
    return static_cast<bus>(0);
}

bool bus_enabled(std::size_t index) noexcept
{
    const bus_config* cfg = config_of(index);
    return cfg != nullptr && cfg->enabled;
}

bus_type configured_bus_type(std::size_t index) noexcept
{
    const bus_config* cfg = config_of(index);
    return cfg != nullptr ? cfg->type : bus_type::classic;
}

id_type filter_id_type_of(std::size_t index) noexcept
{
    const bus_config* cfg = config_of(index);
    return cfg != nullptr ? cfg->filter_id_type : id_type::standard;
}

types::status init(bus selected)
{
    const std::size_t index = static_cast<std::size_t>(selected);
    if (index >= bus_count)
    {
        return types::status::invalid_arg;
    }
    if (!bus_enabled(index))
    {
        return types::status::not_configured;
    }
    if (configured_bus_type(index) != bus_type::classic)
    {
        return types::status::not_configured;
    }
    if (initialized[index])
    {
        return types::status::ok;
    }

    CAN_HandleTypeDef* handle = handle_of(selected);
    if (handle == nullptr)
    {
        return types::status::not_configured;
    }
    if (ensure_err_sem(selected) != types::status::ok)
    {
        return types::status::error;
    }

    CAN_FilterTypeDef filter{};
    filter.FilterBank = handle->Instance == CAN1 ? 0U : 14U;
    filter.FilterMode = CAN_FILTERMODE_IDMASK;
    filter.FilterScale = CAN_FILTERSCALE_32BIT;
    filter.FilterIdLow = filter_id_type_of(index) == id_type::extended ? CAN_ID_EXT : CAN_ID_STD;
    filter.FilterMaskIdLow = CAN_ID_EXT | CAN_RTR_REMOTE;
    filter.FilterFIFOAssignment = CAN_RX_FIFO0;
    filter.FilterActivation = ENABLE;
    filter.SlaveStartFilterBank = 14U;
    if (HAL_CAN_ConfigFilter(handle, &filter) != HAL_OK)
    {
        return types::status::error;
    }

    const types::status status = start_hardware(selected);
    if (status == types::status::ok)
    {
        initialized[index] = true;
    }
    return status;
}

types::status restart(bus selected)
{
    const std::size_t index = static_cast<std::size_t>(selected);
    if (index >= bus_count)
    {
        return types::status::invalid_arg;
    }
    if (!bus_enabled(index))
    {
        return types::status::not_configured;
    }
    CAN_HandleTypeDef* handle = handle_of(selected);
    if (!initialized[index] || handle == nullptr)
    {
        return types::status::error;
    }
    if (HAL_CAN_Stop(handle) != HAL_OK)
    {
        return types::status::error;
    }
    return start_hardware(selected);
}

types::status transmit(bus selected, std::uint32_t id, const std::uint8_t* data,
                       std::uint16_t len)
{
    const std::size_t index = static_cast<std::size_t>(selected);
    if (index >= bus_count || data == nullptr || len == 0U || len > 8U)
    {
        return types::status::invalid_arg;
    }
    if (!bus_enabled(index))
    {
        return types::status::not_configured;
    }
    if (!initialized[index])
    {
        return types::status::error;
    }
    const id_type id_kind = filter_id_type_of(index);
    const std::uint32_t max_id =
        id_kind == id_type::extended ? 0x1FFFFFFFU : 0x7FFU;
    CAN_HandleTypeDef* handle = handle_of(selected);
    if (handle == nullptr || id > max_id)
    {
        return types::status::invalid_arg;
    }

    CAN_TxHeaderTypeDef header{};
    if (id_kind == id_type::extended)
    {
        header.ExtId = id;
        header.IDE = CAN_ID_EXT;
    }
    else
    {
        header.StdId = id;
        header.IDE = CAN_ID_STD;
    }
    header.RTR = CAN_RTR_DATA;
    header.DLC = len;
    header.TransmitGlobalTime = DISABLE;
    std::uint32_t mailbox = 0U;
    const HAL_StatusTypeDef status =
        HAL_CAN_AddTxMessage(handle, &header, const_cast<std::uint8_t*>(data), &mailbox);
    return status == HAL_OK ? types::status::ok
                            : status == HAL_BUSY ? types::status::busy : types::status::error;
}

types::status register_rx_callback(bus selected, rx_callback callback)
{
    if (!callback)
    {
        return types::status::invalid_arg;
    }
    const std::size_t index = static_cast<std::size_t>(selected);
    if (index >= bus_count)
    {
        return types::status::invalid_arg;
    }
    if (!bus_enabled(index))
    {
        return types::status::not_configured;
    }
    for (auto& slot : rx_slots[index])
    {
        if (!slot.callback)
        {
            slot.callback = callback;
            return types::status::ok;
        }
    }
    return types::status::error;
}

void unregister_rx_callbacks(bus selected)
{
    const std::size_t index = static_cast<std::size_t>(selected);
    if (index >= bus_count || !bus_enabled(index))
    {
        return;
    }
    for (auto& slot : rx_slots[index])
    {
        slot.callback = {};
    }
}

TX_SEMAPHORE* err_sem(bus selected)
{
    const std::size_t index = static_cast<std::size_t>(selected);
    return index < bus_count && err_sem_ready[index] ? &err_sems[index] : nullptr;
}

void dispatch_rx_from_isr(bus selected, const rx_frame& frame) noexcept
{
    notify_rx(selected, frame);
}

} // namespace bsp::can

extern "C" void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef* handle)
{
    const bsp::can::bus selected = bsp::can::bus_of(handle);
    const std::size_t index = static_cast<std::size_t>(selected);
    while (index < bsp::can::bus_count &&
           HAL_CAN_GetRxFifoFillLevel(handle, CAN_RX_FIFO0) != 0U)
    {
        CAN_RxHeaderTypeDef header{};
        bsp::can::rx_frame frame{};
        if (HAL_CAN_GetRxMessage(handle, CAN_RX_FIFO0, &header, frame.data) != HAL_OK)
        {
            return;
        }
        frame.id_kind = header.IDE == CAN_ID_EXT ? bsp::can::id_type::extended
                                                 : bsp::can::id_type::standard;
        if (header.RTR != CAN_RTR_DATA ||
            frame.id_kind != bsp::can::filter_id_type_of(index))
        {
            continue;
        }
        frame.id = frame.id_kind == bsp::can::id_type::extended ? header.ExtId : header.StdId;
        frame.len = static_cast<std::uint8_t>(header.DLC);
        frame.format = bsp::can::bus_type::classic;
        frame.bit_rate_switch = false;
        bsp::can::dispatch_rx_from_isr(selected, frame);
    }
}

extern "C" void HAL_CAN_ErrorCallback(CAN_HandleTypeDef* handle)
{
    const bsp::can::bus selected = bsp::can::bus_of(handle);
    const std::size_t index = static_cast<std::size_t>(selected);
    if (index >= bsp::can::bus_count)
    {
        return;
    }
    if ((HAL_CAN_GetError(handle) & HAL_CAN_ERROR_BOF) != 0U)
    {
        (void)HAL_CAN_Stop(handle);
    }
    TX_SEMAPHORE* semaphore = bsp::can::err_sem(selected);
    if (semaphore != nullptr)
    {
        (void)tx_semaphore_put(semaphore);
    }
}
