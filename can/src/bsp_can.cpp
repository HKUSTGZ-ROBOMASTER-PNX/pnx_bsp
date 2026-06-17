#include "bsp_can.hpp"

#include "bsp_board.hpp"
#include "can_classical.hpp"
#include "can_fdcan.hpp"
#include "can_internal.hpp"

#include "os.hpp"

namespace {

struct CanBusContext {
    bsp::CanType type = bsp::CanType::Classic;
    bool initialized = false;
    os::Semaphore error_sem{};
    bool error_sem_ready = false;

    struct RxSlot {
        bsp::CanRxCallback callback = nullptr;
        void* user_data = nullptr;
    };

    RxSlot rx_slots[bsp::kMaxCanRxCallbacks]{};
};

CanBusContext g_can_bus[3];

bool bus_enabled(bsp::CanBus bus) {
    switch (bus) {
        case bsp::CanBus::Bus1:
            return BSP_CAN_ENABLE_BUS1 != 0;
        case bsp::CanBus::Bus2:
            return BSP_CAN_ENABLE_BUS2 != 0;
        case bsp::CanBus::Bus3:
            return BSP_CAN_ENABLE_BUS3 != 0;
        default:
            return false;
    }
}

void dispatch_rx(bsp::CanBus bus, uint32_t id, const uint8_t* data, uint8_t len) {
    auto& ctx = g_can_bus[static_cast<uint8_t>(bus)];
    for (const auto& slot : ctx.rx_slots) {
        if (slot.callback != nullptr) {
            slot.callback(bus, id, data, len, slot.user_data);
        }
    }
}

} // namespace

namespace bsp {
namespace can_detail {

CanBus bus_from_handle(FDCAN_HandleTypeDef* handle) {
    if (handle == &hfdcan1) {
        return CanBus::Bus1;
    }
    if (handle == &hfdcan2) {
        return CanBus::Bus2;
    }
    if (handle == &hfdcan3) {
        return CanBus::Bus3;
    }
    return CanBus::Bus1;
}

CanType query_bus_type(CanBus bus) {
    return g_can_bus[static_cast<uint8_t>(bus)].type;
}

void on_rx(CanBus bus, uint32_t id, const uint8_t* data, uint8_t len) {
    dispatch_rx(bus, id, data, len);
}

void on_error_status(CanBus bus, FDCAN_HandleTypeDef* handle, uint32_t error_status_its) {
    auto& ctx = g_can_bus[static_cast<uint8_t>(bus)];

    if ((error_status_its & FDCAN_IT_ERROR_WARNING) || (error_status_its & FDCAN_IT_ERROR_PASSIVE) ||
        (error_status_its & FDCAN_IT_ARB_PROTOCOL_ERROR) ||
        (error_status_its & FDCAN_IT_DATA_PROTOCOL_ERROR)) {
        if (ctx.error_sem_ready) {
            os::sem_give(&ctx.error_sem);
        }
    }

    if ((error_status_its & FDCAN_IT_BUS_OFF) != 0U) {
        HAL_FDCAN_Stop(handle);
        HAL_FDCAN_Start(handle);
    }
}

uint8_t rx_dlc_to_len(uint32_t dlc) {
    switch (dlc) {
        case FDCAN_DLC_BYTES_0:
            return 0;
        case FDCAN_DLC_BYTES_1:
            return 1;
        case FDCAN_DLC_BYTES_2:
            return 2;
        case FDCAN_DLC_BYTES_3:
            return 3;
        case FDCAN_DLC_BYTES_4:
            return 4;
        case FDCAN_DLC_BYTES_5:
            return 5;
        case FDCAN_DLC_BYTES_6:
            return 6;
        case FDCAN_DLC_BYTES_7:
            return 7;
        case FDCAN_DLC_BYTES_8:
            return 8;
        case FDCAN_DLC_BYTES_12:
            return 12;
        case FDCAN_DLC_BYTES_16:
            return 16;
        case FDCAN_DLC_BYTES_20:
            return 20;
        case FDCAN_DLC_BYTES_24:
            return 24;
        case FDCAN_DLC_BYTES_32:
            return 32;
        case FDCAN_DLC_BYTES_48:
            return 48;
        case FDCAN_DLC_BYTES_64:
            return 64;
        default:
            return 8;
    }
}

} // namespace can_detail

Status can_init(CanBus bus, CanType type) {
    if (!bus_enabled(bus)) {
        return Status::NotConfigured;
    }
    auto& ctx = g_can_bus[static_cast<uint8_t>(bus)];
    if (!ctx.error_sem_ready) {
        char name[] = "CanErr";
        name[3] = static_cast<char>('0' + static_cast<uint8_t>(bus));
        if (os::sem_create(&ctx.error_sem, name, 0) != os::Status::Ok) {
            return Status::Error;
        }
        ctx.error_sem_ready = true;
    }
    if (type == CanType::Classic) {
        can_detail::init_classic(bus);
    } else {
        can_detail::init_fd(bus);
    }
    ctx.type = type;
    ctx.initialized = true;
    return Status::Ok;
}

Status can_transmit(CanBus bus, uint32_t std_id, const uint8_t* data, uint16_t len) {
    if (!bus_enabled(bus) || data == nullptr || len == 0) {
        return Status::InvalidArg;
    }
    auto& ctx = g_can_bus[static_cast<uint8_t>(bus)];
    if (!ctx.initialized) {
        return Status::Error;
    }
    if (ctx.type == CanType::Classic) {
        can_detail::transmit_classic(bus, std_id, data, len);
    } else {
        can_detail::transmit_fd(bus, std_id, data, len);
    }
    return Status::Ok;
}

Status can_register_rx_callback(CanBus bus, CanRxCallback callback, void* user_data) {
    if (callback == nullptr) {
        return Status::InvalidArg;
    }
    auto& ctx = g_can_bus[static_cast<uint8_t>(bus)];
    for (auto& slot : ctx.rx_slots) {
        if (slot.callback == nullptr) {
            slot.callback = callback;
            slot.user_data = user_data;
            return Status::Ok;
        }
    }
    return Status::Error;
}

void can_unregister_rx_callbacks(CanBus bus) {
    auto& ctx = g_can_bus[static_cast<uint8_t>(bus)];
    for (auto& slot : ctx.rx_slots) {
        slot.callback = nullptr;
        slot.user_data = nullptr;
    }
}

os::Semaphore* can_error_sem(CanBus bus) {
    auto& ctx = g_can_bus[static_cast<uint8_t>(bus)];
    return ctx.error_sem_ready ? &ctx.error_sem : nullptr;
}

} // namespace bsp
