#include "bsp_can.hpp"
#include "bsp_can_diag.hpp"

#include <array>
#include <algorithm>
#include <cstring>

namespace bsp::can
{

namespace
{

const bus_config* config_of(std::size_t index) noexcept
{
    return index < bus_count ? &configs[index] : nullptr;
}

FDCAN_HandleTypeDef* handle_from_id(handle_id id) noexcept
{
    switch (id)
    {
    case handle_id::fdcan1:
        return &hfdcan1;
    case handle_id::fdcan2:
        return &hfdcan2;
    case handle_id::fdcan3:
        return &hfdcan3;
    default:
        return nullptr;
    }
}

} // namespace

FDCAN_HandleTypeDef* handle_of(bus b) noexcept
{
    const bus_config* cfg = config_of(static_cast<std::size_t>(b));
    return cfg != nullptr && cfg->enabled ? handle_from_id(cfg->handle) : nullptr;
}

bus bus_of(FDCAN_HandleTypeDef* handle) noexcept
{
    for (std::size_t i = 0; i < bus_count; ++i)
    {
        if (handle_of(static_cast<bus>(i)) == handle)
        {
            return static_cast<bus>(i);
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

namespace
{

struct rx_slot
{
    rx_handler handler = nullptr;
    void* user_data = nullptr;
};

std::array<bus_type, bus_count> active_types{};
std::array<bool, bus_count> initialized{};
std::array<std::array<rx_slot, max_rx_callbacks>, bus_count> rx_slots{};
std::array<TX_SEMAPHORE, bus_count> err_sems{};
std::array<bool, bus_count> err_sem_ready{};

uint32_t len_to_dlc(uint16_t len) noexcept
{
    if (len <= 8U)
    {
        return static_cast<uint32_t>(len);
    }
    if (len <= 12U) { return FDCAN_DLC_BYTES_12; }
    if (len <= 16U) { return FDCAN_DLC_BYTES_16; }
    if (len <= 20U) { return FDCAN_DLC_BYTES_20; }
    if (len <= 24U) { return FDCAN_DLC_BYTES_24; }
    if (len <= 32U) { return FDCAN_DLC_BYTES_32; }
    if (len <= 48U) { return FDCAN_DLC_BYTES_48; }
    return FDCAN_DLC_BYTES_64;
}

uint8_t dlc_to_len(uint32_t dlc) noexcept
{
    switch (dlc)
    {
    case FDCAN_DLC_BYTES_0: return 0;
    case FDCAN_DLC_BYTES_1: return 1;
    case FDCAN_DLC_BYTES_2: return 2;
    case FDCAN_DLC_BYTES_3: return 3;
    case FDCAN_DLC_BYTES_4: return 4;
    case FDCAN_DLC_BYTES_5: return 5;
    case FDCAN_DLC_BYTES_6: return 6;
    case FDCAN_DLC_BYTES_7: return 7;
    case FDCAN_DLC_BYTES_8: return 8;
    case FDCAN_DLC_BYTES_12: return 12;
    case FDCAN_DLC_BYTES_16: return 16;
    case FDCAN_DLC_BYTES_20: return 20;
    case FDCAN_DLC_BYTES_24: return 24;
    case FDCAN_DLC_BYTES_32: return 32;
    case FDCAN_DLC_BYTES_48: return 48;
    case FDCAN_DLC_BYTES_64: return 64;
    default: return 0;
    }
}

types::status ensure_err_sem(bus bus)
{
    const std::size_t idx = static_cast<std::size_t>(bus);
    if (idx >= bus_count)
    {
        return types::status::invalid_arg;
    }
    if (err_sem_ready[idx])
    {
        return types::status::ok;
    }
    if (tx_semaphore_create(&err_sems[idx], const_cast<CHAR*>("can_err"), 0) != TX_SUCCESS)
    {
        return types::status::error;
    }
    err_sem_ready[idx] = true;
    return types::status::ok;
}

void notify_rx(bus bus, const rx_frame& frame)
{
    const std::size_t idx = static_cast<std::size_t>(bus);
    if (idx >= bus_count)
    {
        return;
    }
    for (auto& slot : rx_slots[idx])
    {
        if (slot.handler != nullptr)
        {
            slot.handler(bus, frame, slot.user_data);
        }
    }
}

types::status configure_standard_filter(FDCAN_HandleTypeDef* handle, uint32_t fifo) noexcept
{
    FDCAN_FilterTypeDef filter{};
    filter.IdType = FDCAN_STANDARD_ID;
    filter.FilterIndex = 0U;
    filter.FilterType = FDCAN_FILTER_MASK;
    filter.FilterConfig = fifo == FDCAN_RX_FIFO1 ? FDCAN_FILTER_TO_RXFIFO1 : FDCAN_FILTER_TO_RXFIFO0;
    filter.FilterID1 = 0x000U;
    filter.FilterID2 = 0x000U;

    if (HAL_FDCAN_ConfigFilter(handle, &filter) != HAL_OK)
    {
        return types::status::error;
    }
    if (HAL_FDCAN_ConfigGlobalFilter(handle, FDCAN_REJECT, FDCAN_REJECT,
                                     FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE) != HAL_OK)
    {
        return types::status::error;
    }
    return types::status::ok;
}

types::status init_classic(bus bus)
{
    uint32_t notification = FDCAN_IT_RX_FIFO0_NEW_MESSAGE |
                            FDCAN_IT_ERROR_WARNING |
                            FDCAN_IT_ERROR_PASSIVE |
                            FDCAN_IT_ARB_PROTOCOL_ERROR |
                            FDCAN_IT_DATA_PROTOCOL_ERROR |
                            FDCAN_IT_BUS_OFF;
    if constexpr (config::feature::can_diag)
    {
        notification |= FDCAN_IT_RX_FIFO0_MESSAGE_LOST | FDCAN_IT_RX_FIFO1_MESSAGE_LOST;
    }

    FDCAN_HandleTypeDef* handle = handle_of(bus);
    if (handle == nullptr)
    {
        return types::status::invalid_arg;
    }
    types::status status = configure_standard_filter(handle, FDCAN_RX_FIFO0);
    if (status != types::status::ok)
    {
        return status;
    }
    if (HAL_FDCAN_ActivateNotification(handle, notification, 0U) != HAL_OK)
    {
        return types::status::error;
    }
    if (HAL_FDCAN_Start(handle) != HAL_OK)
    {
        return types::status::error;
    }
    return types::status::ok;
}

types::status init_fd(bus bus)
{
    uint32_t notification = FDCAN_IT_RX_FIFO1_NEW_MESSAGE |
                            FDCAN_IT_ERROR_WARNING |
                            FDCAN_IT_ERROR_PASSIVE |
                            FDCAN_IT_ARB_PROTOCOL_ERROR |
                            FDCAN_IT_DATA_PROTOCOL_ERROR |
                            FDCAN_IT_BUS_OFF;
    if constexpr (config::feature::can_diag)
    {
        notification |= FDCAN_IT_RX_FIFO0_MESSAGE_LOST | FDCAN_IT_RX_FIFO1_MESSAGE_LOST;
    }

    FDCAN_HandleTypeDef* handle = handle_of(bus);
    if (handle == nullptr)
    {
        return types::status::invalid_arg;
    }
    types::status status = configure_standard_filter(handle, FDCAN_RX_FIFO1);
    if (status != types::status::ok)
    {
        return status;
    }
    if (HAL_FDCAN_ActivateNotification(handle, notification, 0U) != HAL_OK)
    {
        return types::status::error;
    }
    if (HAL_FDCAN_EnableTxDelayCompensation(&hfdcan1) != HAL_OK)
    {
        return types::status::error;
    }
    if (HAL_FDCAN_ConfigTxDelayCompensation(&hfdcan1,13,13) != HAL_OK)
    {
        return types::status::error;
    }
    if (HAL_FDCAN_Start(handle) != HAL_OK)
    {
        return types::status::error;
    }
    return types::status::ok;
}

types::status transmit_frame(bus bus,
                             uint32_t id,
                             const uint8_t* data,
                             uint16_t len,
                             uint32_t fd_format)
{
    FDCAN_HandleTypeDef* handle = handle_of(bus);
    if (handle == nullptr || data == nullptr)
    {
        return types::status::invalid_arg;
    }

    FDCAN_TxHeaderTypeDef tx_header{};
    tx_header.Identifier = id;
    tx_header.IdType = (id > 0x7FFU) ? FDCAN_EXTENDED_ID : FDCAN_STANDARD_ID;
    tx_header.TxFrameType = FDCAN_DATA_FRAME;
    tx_header.DataLength = len_to_dlc(len);
    tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    tx_header.BitRateSwitch = (fd_format == FDCAN_FD_CAN) ? FDCAN_BRS_ON : FDCAN_BRS_OFF;
    tx_header.FDFormat = fd_format;
    tx_header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    tx_header.MessageMarker = 0U;

    uint8_t payload[64]{};
    const auto copy_len = std::min<std::size_t>(len, sizeof(payload));
    std::memcpy(payload, data, copy_len);

    if (HAL_FDCAN_AddMessageToTxFifoQ(handle, &tx_header, payload) != HAL_OK)
    {
        return types::status::error;
    }
    if constexpr (config::feature::can_diag)
    {
        ++can_diag_bus[static_cast<std::size_t>(bus)].tx_attempts_total;
    }
    return types::status::ok;
}

types::status transmit_classic(bus bus, uint32_t id, const uint8_t* data, uint16_t len)
{
    if (len > 8U)
    {
        return types::status::invalid_arg;
    }
    return transmit_frame(bus, id, data, len, FDCAN_CLASSIC_CAN);
}

types::status transmit_fd(bus bus, uint32_t id, const uint8_t* data, uint16_t len)
{
    if (len > 64U)
    {
        return types::status::invalid_arg;
    }
    return transmit_frame(bus, id, data, len, FDCAN_FD_CAN);
}

} // namespace

void handle_error(FDCAN_HandleTypeDef* handle, uint32_t error_status_its)
{
    const auto bus = bus_of(handle);
    const std::size_t idx = static_cast<std::size_t>(bus);
    if (idx >= bus_count)
    {
        return;
    }

    if constexpr (config::feature::can_diag)
    {
        if ((error_status_its & FDCAN_IT_ERROR_WARNING) != 0U)
        {
            ++can_diag_bus[idx].warning_total;
            ++can_diag_bus[idx].err_events_total;
        }
        if ((error_status_its & FDCAN_IT_ERROR_PASSIVE) != 0U)
        {
            ++can_diag_bus[idx].passive_total;
            ++can_diag_bus[idx].err_events_total;
        }
        if ((error_status_its & FDCAN_IT_BUS_OFF) != 0U)
        {
            ++can_diag_bus[idx].busoff_total;
            ++can_diag_bus[idx].err_events_total;
        }
    }

    if (active_types[idx] != bus_type::classic)
    {
        return;
    }

    if ((error_status_its & FDCAN_IT_ERROR_WARNING) != 0U ||
        (error_status_its & FDCAN_IT_ERROR_PASSIVE) != 0U ||
        (error_status_its & FDCAN_IT_ARB_PROTOCOL_ERROR) != 0U ||
        (error_status_its & FDCAN_IT_DATA_PROTOCOL_ERROR) != 0U)
    {
        if (err_sem_ready[idx])
        {
            tx_semaphore_put(&err_sems[idx]);
        }
    }

    if ((error_status_its & FDCAN_IT_BUS_OFF) != 0U)
    {
        HAL_FDCAN_Stop(handle);
        HAL_FDCAN_Start(handle);
    }
}

void receive(FDCAN_HandleTypeDef* handle, uint32_t rx_fifo, bus_type mode)
{
    const auto bus = bus_of(handle);
    const std::size_t idx = static_cast<std::size_t>(bus);
    if (idx >= bus_count || active_types[idx] != mode)
    {
        return;
    }

    FDCAN_RxHeaderTypeDef rx_header{};
    rx_frame frame{};
    if (HAL_FDCAN_GetRxMessage(handle, rx_fifo, &rx_header, frame.data) != HAL_OK)
    {
        return;
    }

    frame.id = rx_header.Identifier;
    frame.len = dlc_to_len(rx_header.DataLength);
    if constexpr (config::feature::can_diag)
    {
        ++can_diag_bus[idx].rx_frames_total;
    }
    notify_rx(bus, frame);
}// ?多态完了我怎么知道对应哪个can 哪个id有回调

types::status init(bus bus, bus_type type)
{
    const std::size_t idx = static_cast<std::size_t>(bus);
    if (idx >= bus_count)
    {
        return types::status::invalid_arg;
    }

    if (!bus_enabled(idx))
    {
        return types::status::not_configured;
    }

    if (initialized[idx])
    {
        return types::status::ok;
    }

    types::status status = ensure_err_sem(bus);
    if (status != types::status::ok)
    {
        return status;
    }

    if constexpr (config::feature::can_diag)
    {
        bsp::can::diag::init(); // 惰性创建采样定时器（只创建一次）
    }

    if (type == bus_type::classic)
    {
        status = init_classic(bus);
    }
    else
    {
        status = init_fd(bus);
    }

    if (status != types::status::ok)
    {
        return status;
    }

    active_types[idx] = type;
    initialized[idx] = true;
    return types::status::ok;
}

types::status transmit(bus bus, uint32_t id, const uint8_t* data, uint16_t len)
{
    if (data == nullptr || len == 0)
    {
        return types::status::invalid_arg;
    }

    const std::size_t idx = static_cast<std::size_t>(bus);
    if (idx >= bus_count)
    {
        return types::status::invalid_arg;
    }

    if (!bus_enabled(idx))
    {
        return types::status::not_configured;
    }

    if (!initialized[idx])
    {
        return types::status::error;
    }

    if (active_types[idx] == bus_type::classic)
    {
        return transmit_classic(bus, id, data, len);
    }

    return transmit_fd(bus, id, data, len);
}

types::status register_rx_handler(bus bus, rx_handler handler, void* user_data)
{
    if (handler == nullptr)
    {
        return types::status::invalid_arg;
    }

    const std::size_t idx = static_cast<std::size_t>(bus);
    if (idx >= bus_count)
    {
        return types::status::invalid_arg;
    }

    if (!bus_enabled(idx))
    {
        return types::status::not_configured;
    }

    for (auto& slot : rx_slots[idx])
    {
        if (slot.handler == nullptr)
        {
            slot.handler = handler;
            slot.user_data = user_data;
            return types::status::ok;
        }
    }

    return types::status::error;
}

void unregister_rx_handlers(bus bus)
{
    const std::size_t idx = static_cast<std::size_t>(bus);
    if (idx >= bus_count)
    {
        return;
    }

    if (!bus_enabled(idx))
    {
        return;
    }

    for (auto& slot : rx_slots[idx])
    {
        slot.handler = nullptr;
        slot.user_data = nullptr;
    }
}

TX_SEMAPHORE* err_sem(bus bus)
{
    const std::size_t idx = static_cast<std::size_t>(bus);
    if (idx >= bus_count || !err_sem_ready[idx])
    {
        return nullptr;
    }

    return &err_sems[idx];
}

} // namespace bsp::can

extern "C" void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef* hfdcan, uint32_t RxFifo0ITs)
{
    if constexpr (config::feature::can_diag)
    {
        if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_MESSAGE_LOST) != 0U)
        {
            ++can_diag_bus[static_cast<std::size_t>(bsp::can::bus_of(hfdcan))].rx_overrun_total;
        }
    }
    (void)RxFifo0ITs;
    bsp::can::receive(hfdcan, FDCAN_RX_FIFO0, bsp::can::bus_type::classic);
}

extern "C" void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef* hfdcan, uint32_t RxFifo1ITs)
{
    if constexpr (config::feature::can_diag)
    {
        if ((RxFifo1ITs & FDCAN_IT_RX_FIFO1_MESSAGE_LOST) != 0U)
        {
            ++can_diag_bus[static_cast<std::size_t>(bsp::can::bus_of(hfdcan))].rx_overrun_total;
        }
    }
    (void)RxFifo1ITs;
    bsp::can::receive(hfdcan, FDCAN_RX_FIFO1, bsp::can::bus_type::fd);
}

extern "C" void HAL_FDCAN_ErrorStatusCallback(FDCAN_HandleTypeDef* hfdcan, uint32_t ErrorStatusITs)
{
    bsp::can::handle_error(hfdcan, ErrorStatusITs);
}
