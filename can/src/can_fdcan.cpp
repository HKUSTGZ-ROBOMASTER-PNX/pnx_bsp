#include "can_fdcan.hpp"
#include "can_internal.hpp"

#include "fdcan.h"

namespace {

FDCAN_HandleTypeDef* handle_for(bsp::CanBus bus) {
    switch (bus) {
        case bsp::CanBus::Bus1:
            return &hfdcan1;
        case bsp::CanBus::Bus2:
            return &hfdcan2;
        case bsp::CanBus::Bus3:
            return &hfdcan3;
        default:
            return nullptr;
    }
}

uint32_t fd_bytes_to_dlc(uint16_t len) {
    if (len <= 8U) {
        static const uint32_t dlc_table[9] = {
            FDCAN_DLC_BYTES_0, FDCAN_DLC_BYTES_1, FDCAN_DLC_BYTES_2, FDCAN_DLC_BYTES_3,
            FDCAN_DLC_BYTES_4, FDCAN_DLC_BYTES_5, FDCAN_DLC_BYTES_6, FDCAN_DLC_BYTES_7,
            FDCAN_DLC_BYTES_8,
        };
        return dlc_table[len];
    }
    if (len <= 12U) {
        return FDCAN_DLC_BYTES_12;
    }
    if (len <= 16U) {
        return FDCAN_DLC_BYTES_16;
    }
    if (len <= 20U) {
        return FDCAN_DLC_BYTES_20;
    }
    if (len <= 24U) {
        return FDCAN_DLC_BYTES_24;
    }
    if (len <= 32U) {
        return FDCAN_DLC_BYTES_32;
    }
    if (len <= 48U) {
        return FDCAN_DLC_BYTES_48;
    }
    return FDCAN_DLC_BYTES_64;
}

} // namespace

namespace bsp {
namespace can_detail {

void init_fd(CanBus bus) {
    FDCAN_HandleTypeDef* handle = handle_for(bus);
    if (handle == nullptr) {
        return;
    }

    FDCAN_FilterTypeDef filter{};
    filter.IdType = FDCAN_STANDARD_ID;
    filter.FilterIndex = 0;
    filter.FilterType = FDCAN_FILTER_MASK;
    filter.FilterID1 = 0x00000000;
    filter.FilterID2 = 0x00000000;
    filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO1;

    HAL_FDCAN_ConfigFilter(handle, &filter);
    HAL_FDCAN_ConfigGlobalFilter(handle, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE,
                                 FDCAN_FILTER_REMOTE);
    HAL_FDCAN_ActivateNotification(handle, FDCAN_IT_RX_FIFO1_NEW_MESSAGE, 0);
    HAL_FDCAN_EnableTxDelayCompensation(handle);
    HAL_FDCAN_ConfigTxDelayCompensation(handle, 13, 13);
    HAL_FDCAN_Start(handle);
}

void transmit_fd(CanBus bus, uint32_t std_id, const uint8_t* data, uint16_t len) {
    FDCAN_HandleTypeDef* handle = handle_for(bus);
    if (handle == nullptr || data == nullptr || len == 0) {
        return;
    }

    FDCAN_TxHeaderTypeDef tx_header{};
    tx_header.IdType = FDCAN_STANDARD_ID;
    tx_header.TxFrameType = FDCAN_DATA_FRAME;
    tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    tx_header.BitRateSwitch = FDCAN_BRS_ON;
    tx_header.FDFormat = FDCAN_FD_CAN;
    tx_header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    tx_header.MessageMarker = 0;
    tx_header.Identifier = std_id;
    tx_header.DataLength = fd_bytes_to_dlc(len);

    HAL_FDCAN_AddMessageToTxFifoQ(handle, &tx_header, const_cast<uint8_t*>(data));
}

} // namespace can_detail
} // namespace bsp

extern "C" void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef* hfdcan, uint32_t RxFifo1ITs) {
    (void)RxFifo1ITs;
    const auto bus = bsp::can_detail::bus_from_handle(hfdcan);
    if (bsp::can_detail::query_bus_type(bus) != bsp::CanType::FD) {
        return;
    }

    FDCAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[64] = {0};
    if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO1, &rx_header, rx_data) != HAL_OK) {
        return;
    }

    const uint8_t len = bsp::can_detail::rx_dlc_to_len(rx_header.DataLength);
    bsp::can_detail::on_rx(bus, rx_header.Identifier, rx_data, len);
}
