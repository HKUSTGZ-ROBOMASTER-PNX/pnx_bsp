#include "can_classical.hpp"
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

uint32_t bytes_to_dlc(uint16_t len) {
    if (len > 8U) {
        len = 8U;
    }
    static const uint32_t dlc_table[9] = {
        FDCAN_DLC_BYTES_0, FDCAN_DLC_BYTES_1, FDCAN_DLC_BYTES_2, FDCAN_DLC_BYTES_3,
        FDCAN_DLC_BYTES_4, FDCAN_DLC_BYTES_5, FDCAN_DLC_BYTES_6, FDCAN_DLC_BYTES_7,
        FDCAN_DLC_BYTES_8,
    };
    return dlc_table[len];
}

} // namespace

namespace bsp {
namespace can_detail {

void init_classic(CanBus bus) {
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
    filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;

    HAL_FDCAN_ConfigFilter(handle, &filter);
    HAL_FDCAN_ConfigGlobalFilter(handle, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE,
                                 FDCAN_FILTER_REMOTE);
    HAL_FDCAN_ActivateNotification(handle,
                                   FDCAN_IT_RX_FIFO0_NEW_MESSAGE | FDCAN_IT_BUS_OFF |
                                       FDCAN_IT_ERROR_WARNING | FDCAN_IT_ERROR_PASSIVE |
                                       FDCAN_IT_ARB_PROTOCOL_ERROR | FDCAN_IT_DATA_PROTOCOL_ERROR,
                                   0);
    HAL_FDCAN_Start(handle);
}

void transmit_classic(CanBus bus, uint32_t std_id, const uint8_t* data, uint16_t len) {
    FDCAN_HandleTypeDef* handle = handle_for(bus);
    if (handle == nullptr || data == nullptr || len == 0) {
        return;
    }

    FDCAN_TxHeaderTypeDef tx_header{};
    tx_header.IdType = FDCAN_STANDARD_ID;
    tx_header.TxFrameType = FDCAN_DATA_FRAME;
    tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    tx_header.BitRateSwitch = FDCAN_BRS_OFF;
    tx_header.FDFormat = FDCAN_CLASSIC_CAN;
    tx_header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    tx_header.MessageMarker = 0;
    tx_header.Identifier = std_id;
    tx_header.DataLength = bytes_to_dlc(len);

    HAL_FDCAN_AddMessageToTxFifoQ(handle, &tx_header, const_cast<uint8_t*>(data));
}

} // namespace can_detail
} // namespace bsp

extern "C" void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef* hfdcan, uint32_t RxFifo0ITs) {
    (void)RxFifo0ITs;
    const auto bus = bsp::can_detail::bus_from_handle(hfdcan);
    if (bsp::can_detail::query_bus_type(bus) != bsp::CanType::Classic) {
        return;
    }

    FDCAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8] = {0};
    if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rx_header, rx_data) != HAL_OK) {
        return;
    }

    const uint8_t len = bsp::can_detail::rx_dlc_to_len(rx_header.DataLength);
    bsp::can_detail::on_rx(bus, rx_header.Identifier, rx_data, len);
}

extern "C" void HAL_FDCAN_ErrorStatusCallback(FDCAN_HandleTypeDef* hfdcan, uint32_t ErrorStatusITs) {
    const auto bus = bsp::can_detail::bus_from_handle(hfdcan);
    if (bsp::can_detail::query_bus_type(bus) != bsp::CanType::Classic) {
        return;
    }
    bsp::can_detail::on_error_status(bus, hfdcan, ErrorStatusITs);
}
