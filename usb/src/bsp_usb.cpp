#include "bsp_usb.hpp"

#include "bridge_usb.h"
#include "tx_api.h"
#include "usb_otg.h"
#include "usb_tx_completion.hpp"
#include "ux_api.h"
#include "ux_device_class_cdc_acm.h"

#include <algorithm>
#include <array>
#include <atomic>
#include <cstdint>
#include <cstring>

#ifndef PNX_USB_DEVICE_IDENTITY_CONFIRMED
#define PNX_USB_DEVICE_IDENTITY_CONFIRMED 0
#endif

extern "C" UINT _ux_dcd_stm32_initialize(
    ULONG dcd_io, ULONG parameter);

namespace bsp::usb
{
namespace f407_internal
{

constexpr std::uint16_t common_max_transfer_size = 512U;
constexpr std::uint8_t common_tx_queue_depth = 2U;
constexpr std::uint16_t io_buffer_size =
    pnx::f407::usb_detail::cdc_full_speed_max_packet_size;
constexpr std::uint16_t worker_stack_size = 1536U;
constexpr ULONG rx_fifo_words = 128U;
constexpr ULONG ep0_tx_fifo_words = 16U;
constexpr ULONG cdc_command_tx_fifo_words = 16U;
constexpr ULONG cdc_data_tx_fifo_words = 32U;

struct tx_slot
{
    std::array<std::uint8_t, common_max_transfer_size> data{};
    std::uint16_t length = 0U;
};

config active_config{};
runtime_state active_state{};
runtime_state compatibility_snapshot{};
std::array<tx_slot, common_tx_queue_depth> tx_queue{};
std::uint8_t tx_head = 0U;
std::uint8_t tx_tail = 0U;
std::uint8_t tx_count = 0U;
bool initialized = false;

alignas(8) std::uint8_t worker_stack[worker_stack_size]{};
alignas(4) std::uint8_t tx_buffer[io_buffer_size]{};
TX_THREAD worker_thread{};
TX_SEMAPHORE write_signal{};
TX_MUTEX common_mutex{};
TX_MUTEX transport_mutex{};
pnx::f407::usb_detail::tx_session_state active_transport{};
pnx::f407::usb_detail::startup_lifecycle lifecycle{};
bool mutex_ready = false;
bool transport_mutex_ready = false;
bool signal_ready = false;
std::atomic<std::uint32_t> pending_connection_event{
    pnx::f407::usb_detail::connection_event_none};

static_assert(
    std::atomic<std::uint32_t>::is_always_lock_free,
    "F407 USB ISR-to-thread connection event must be lock-free");
capabilities hardware_capabilities() noexcept
{
    capabilities result{};
    result.device_mode = true;
    result.high_speed = false;
    result.supports_vbus_sense = false;
    result.max_packet_size = io_buffer_size;
    result.max_transfer_size = io_buffer_size;
    return result;
}

bool lock_state() noexcept
{
    return mutex_ready &&
           tx_mutex_get(&common_mutex, TX_WAIT_FOREVER) == TX_SUCCESS;
}

bool try_lock_state() noexcept
{
    return mutex_ready &&
           tx_mutex_get(&common_mutex, TX_NO_WAIT) == TX_SUCCESS;
}

void unlock_state() noexcept
{
    if (mutex_ready)
    {
        (void)tx_mutex_put(&common_mutex);
    }
}

class state_guard
{
public:
    state_guard() noexcept : locked_(lock_state()) {}
    state_guard(const state_guard&) = delete;
    state_guard& operator=(const state_guard&) = delete;

    ~state_guard()
    {
        if (locked_)
        {
            unlock_state();
        }
    }

    explicit operator bool() const noexcept
    {
        return locked_;
    }

private:
    bool locked_;
};

class write_state_guard
{
public:
    write_state_guard() noexcept : locked_(try_lock_state()) {}
    write_state_guard(const write_state_guard&) = delete;
    write_state_guard& operator=(const write_state_guard&) = delete;

    ~write_state_guard()
    {
        if (locked_)
        {
            unlock_state();
        }
    }

    explicit operator bool() const noexcept
    {
        return locked_;
    }

private:
    bool locked_;
};

bool lock_transport() noexcept
{
    return transport_mutex_ready &&
           tx_mutex_get(&transport_mutex, TX_WAIT_FOREVER) ==
               TX_SUCCESS;
}

void unlock_transport() noexcept
{
    if (transport_mutex_ready)
    {
        (void)tx_mutex_put(&transport_mutex);
    }
}

class transport_guard
{
public:
    transport_guard() noexcept : locked_(lock_transport()) {}
    transport_guard(const transport_guard&) = delete;
    transport_guard& operator=(const transport_guard&) = delete;

    ~transport_guard()
    {
        if (locked_)
        {
            unlock_transport();
        }
    }

    explicit operator bool() const noexcept
    {
        return locked_;
    }

private:
    bool locked_;
};

std::uint16_t transfer_limit() noexcept
{
    const capabilities hardware = hardware_capabilities();
    return std::min<std::uint16_t>(
        active_config.max_tx_size,
        std::min<std::uint16_t>(
            hardware.max_transfer_size, common_max_transfer_size));
}

void update_queue_state() noexcept
{
    active_state.tx_queue_size = tx_count;
    active_state.pending_write_len =
        tx_count == 0U ? 0U : tx_queue[tx_tail].length;
    active_state.tx_queue_high_water =
        std::max(active_state.tx_queue_high_water, tx_count);
}

bool device_ready(UX_SLAVE_CLASS_CDC_ACM* cdc) noexcept
{
    return cdc != nullptr &&
           lifecycle.controller_active() &&
           _ux_system_slave != nullptr &&
           _ux_system_slave->ux_system_slave_device.ux_slave_device_state ==
               UX_DEVICE_CONFIGURED;
}

void signal_tx() noexcept
{
    if (signal_ready)
    {
        (void)tx_semaphore_put(&write_signal);
    }
}

void publish_connection_event(bool connected) noexcept
{
    std::uint32_t expected =
        pending_connection_event.load(std::memory_order_relaxed);
    for (;;)
    {
        const std::uint32_t desired =
            pnx::f407::usb_detail::merge_connection_event(
                expected, connected);
        if (pending_connection_event.compare_exchange_weak(
                expected, desired, std::memory_order_release,
                std::memory_order_relaxed))
        {
            return;
        }
    }
}

void connection_changed(bool is_connected) noexcept
{
    if (!initialized && !active_state.initialized)
    {
        return;
    }

    state_guard guard{};
    if (!guard)
    {
        return;
    }

    if (is_connected)
    {
        if (!lifecycle.transport_usable())
        {
            return;
        }
        if (!active_state.connected)
        {
            ++active_state.connect_count;
        }
        active_state.connected = true;
        active_state.link = link_state::connected;
        return;
    }

    if (active_state.connected)
    {
        ++active_state.disconnect_count;
    }
    active_state.connected = false;
    active_state.link =
        lifecycle.faulted()
            ? link_state::fault
            : (active_state.initialized ? link_state::initialized
                                        : link_state::uninitialized);
    active_state.write_busy = false;
    active_state.read_busy = false;
    active_state.tx_drop_count += tx_count;
    tx_head = 0U;
    tx_tail = 0U;
    tx_count = 0U;
    update_queue_state();
}

void receive_completed(const std::uint8_t* data,
                       std::uint16_t length,
                       types::status status,
                       std::uint32_t native_status) noexcept
{
    void (*callback)(const std::uint8_t*, std::uint16_t, void*) = nullptr;
    void* user = nullptr;
    {
        state_guard guard{};
        if (!guard)
        {
            return;
        }
        active_state.read_busy = false;
        active_state.last_read_status = native_status;
        active_state.last_read_len = length;
        if (status != types::status::ok)
        {
            ++active_state.error_count;
            return;
        }
        if (!active_state.connected || data == nullptr ||
            length < active_config.min_rx_size)
        {
            return;
        }
        ++active_state.read_count;
        active_state.rx_bytes += length;
        callback = active_config.on_rx;
        user = active_config.user;
    }
    if (callback != nullptr)
    {
        callback(data, length, user);
    }
}

bool prepare_tx(std::uint8_t* output, std::uint16_t capacity,
                std::uint16_t& length) noexcept
{
    length = 0U;
    if (output == nullptr || capacity == 0U)
    {
        return false;
    }

    std::uint16_t (*fill)(std::uint8_t*, std::uint16_t, void*) = nullptr;
    void* user = nullptr;
    std::uint16_t fill_capacity = 0U;
    {
        state_guard guard{};
        if (!guard || !active_state.connected ||
            !lifecycle.transport_usable())
        {
            return false;
        }
        if (tx_count > 0U)
        {
            tx_slot& slot = tx_queue[tx_tail];
            if (slot.length > capacity)
            {
                ++active_state.error_count;
                ++active_state.tx_drop_count;
                tx_tail = static_cast<std::uint8_t>(
                    (tx_tail + 1U) % common_tx_queue_depth);
                --tx_count;
                update_queue_state();
                return false;
            }
            std::memcpy(output, slot.data.data(), slot.length);
            length = slot.length;
            tx_tail = static_cast<std::uint8_t>(
                (tx_tail + 1U) % common_tx_queue_depth);
            --tx_count;
            update_queue_state();
            active_state.write_busy = true;
            active_state.last_write_requested = length;
            return true;
        }
        fill = active_config.fill_tx;
        user = active_config.user;
        fill_capacity =
            std::min<std::uint16_t>(capacity, transfer_limit());
        if (fill != nullptr)
        {
            ++active_state.fill_count;
        }
    }

    if (fill == nullptr || fill_capacity == 0U)
    {
        return false;
    }
    const std::uint16_t filled = fill(output, fill_capacity, user);
    if (filled == 0U || filled > fill_capacity)
    {
        return false;
    }

    state_guard guard{};
    if (!guard || !active_state.connected ||
        !lifecycle.transport_usable())
    {
        return false;
    }
    length = filled;
    active_state.write_busy = true;
    active_state.last_write_requested = filled;
    active_state.pending_write_len = filled;
    return true;
}

void transmit_completed(std::uint16_t requested,
                        std::uint16_t actual,
                        types::status status,
                        std::uint32_t native_status) noexcept
{
    void (*callback)(std::uint16_t, std::uint16_t,
                     std::uint32_t, void*) = nullptr;
    void* user = nullptr;
    {
        state_guard guard{};
        if (!guard)
        {
            return;
        }
        active_state.write_busy = false;
        active_state.pending_write_len =
            tx_count == 0U ? 0U : tx_queue[tx_tail].length;
        active_state.last_write_requested = requested;
        active_state.last_write_actual = actual;
        active_state.last_write_status = native_status;
        if (status == types::status::ok && actual == requested)
        {
            ++active_state.write_count;
            active_state.tx_bytes += actual;
        }
        else
        {
            ++active_state.error_count;
        }
        callback = active_config.on_tx_done;
        user = active_config.user;
    }
    if (callback != nullptr)
    {
        callback(requested, actual, native_status, user);
    }
}

void cancel_prepared_tx() noexcept
{
    state_guard guard{};
    if (!guard)
    {
        return;
    }
    active_state.write_busy = false;
    active_state.pending_write_len =
        tx_count == 0U ? 0U : tx_queue[tx_tail].length;
    ++active_state.tx_drop_count;
}

void publish_native_error(UINT native_status) noexcept
{
    receive_completed(
        nullptr, 0U, types::status::error, native_status);
}

void enter_fault(UINT native_status) noexcept
{
    lifecycle.fail();
    pending_connection_event.store(
        pnx::f407::usb_detail::connection_event_none,
        std::memory_order_release);
    {
        transport_guard guard{};
        if (guard)
        {
            active_transport = {};
        }
    }

    state_guard guard{};
    if (!guard)
    {
        return;
    }
    active_state.connected = false;
    active_state.link = link_state::fault;
    active_state.write_busy = false;
    active_state.read_busy = false;
    active_state.last_read_status = native_status;
    active_state.tx_drop_count += tx_count;
    tx_head = 0U;
    tx_tail = 0U;
    tx_count = 0U;
    update_queue_state();
    ++active_state.error_count;
}

UINT cdc_write_complete(UX_SLAVE_CLASS_CDC_ACM* cdc, UINT status,
                        ULONG length)
{
    pnx::f407::usb_detail::guarded_tx_completion guarded{};
    {
        transport_guard guard{};
        if (!guard)
        {
            return UX_SUCCESS;
        }
        guarded = active_transport.on_callback(
            cdc, status == UX_SUCCESS,
            static_cast<std::uint16_t>(length));
        if (!guarded.accepted)
        {
            return UX_SUCCESS;
        }
        if (guarded.completion.next ==
            pnx::f407::usb_detail::tx_next_action::send_zlp)
        {
            const UINT zlp_status =
                ux_device_class_cdc_acm_write_with_callback(
                    cdc, tx_buffer, 0U);
            if (zlp_status == UX_SUCCESS)
            {
                return UX_SUCCESS;
            }
            guarded = active_transport.abort(cdc);
            status = zlp_status;
        }
    }
    if (!guarded.accepted)
    {
        return UX_SUCCESS;
    }

    transmit_completed(
        guarded.completion.requested,
        guarded.completion.actual,
        guarded.completion.success ? types::status::ok
                                   : types::status::error,
        status);
    signal_tx();
    return UX_SUCCESS;
}

UINT cdc_read_complete(UX_SLAVE_CLASS_CDC_ACM*, UINT status,
                       UCHAR* data, ULONG length)
{
    receive_completed(
        data, static_cast<std::uint16_t>(length),
        status == UX_SUCCESS ? types::status::ok : types::status::error,
        status);
    return UX_SUCCESS;
}

UINT start_controller() noexcept
{
    if (HAL_PCDEx_SetRxFiFo(
            &hpcd_USB_OTG_FS, rx_fifo_words) != HAL_OK ||
        HAL_PCDEx_SetTxFiFo(
            &hpcd_USB_OTG_FS, 0U, ep0_tx_fifo_words) != HAL_OK ||
        HAL_PCDEx_SetTxFiFo(
            &hpcd_USB_OTG_FS, 2U,
            cdc_command_tx_fifo_words) != HAL_OK ||
        HAL_PCDEx_SetTxFiFo(
            &hpcd_USB_OTG_FS, 3U, cdc_data_tx_fifo_words) != HAL_OK)
    {
        return UX_ERROR;
    }

    UINT status = _ux_dcd_stm32_initialize(
        reinterpret_cast<ULONG>(USB_OTG_FS),
        reinterpret_cast<ULONG>(&hpcd_USB_OTG_FS));
    if (status != UX_SUCCESS)
    {
        return status;
    }
    if (HAL_PCD_Start(&hpcd_USB_OTG_FS) != HAL_OK)
    {
        return UX_ERROR;
    }
    return lifecycle.mark_controller_started() ? UX_SUCCESS : UX_ERROR;
}

void worker_entry(ULONG)
{
    const UINT controller_status = start_controller();
    if (controller_status != UX_SUCCESS)
    {
        enter_fault(controller_status);
        return;
    }

    for (;;)
    {
        const UINT wait_status = tx_semaphore_get(
            &write_signal, active_config.period_ticks);
        if (wait_status != TX_SUCCESS &&
            wait_status != TX_NO_INSTANCE)
        {
            enter_fault(wait_status);
            return;
        }

        const std::uint32_t connection_event =
            pending_connection_event.exchange(
                pnx::f407::usb_detail::connection_event_none,
                std::memory_order_acq_rel);
        if ((connection_event &
             pnx::f407::usb_detail::connection_event_disconnected) !=
            0U)
        {
            connection_changed(false);
        }
        if ((connection_event &
             pnx::f407::usb_detail::connection_event_connected) !=
            0U)
        {
            connection_changed(true);
        }

        {
            state_guard guard{};
            if (guard)
            {
                ++active_state.tx_wake_count;
            }
        }
        pnx::f407::usb_detail::tx_ticket ticket{};
        UX_SLAVE_CLASS_CDC_ACM* cdc = nullptr;
        {
            transport_guard guard{};
            if (guard)
            {
                ticket = active_transport.begin_tx();
                cdc = static_cast<UX_SLAVE_CLASS_CDC_ACM*>(
                    ticket.instance);
            }
        }
        if (!ticket.valid() || !device_ready(cdc))
        {
            continue;
        }

        std::uint16_t requested = 0U;
        if (!prepare_tx(tx_buffer, sizeof(tx_buffer), requested))
        {
            continue;
        }

        UINT status = UX_ERROR;
        pnx::f407::usb_detail::guarded_tx_completion failure{};
        {
            transport_guard guard{};
            if (guard && active_transport.start(ticket, requested))
            {
                status =
                    ux_device_class_cdc_acm_write_with_callback(
                        cdc, tx_buffer, requested);
                if (status != UX_SUCCESS)
                {
                    failure = active_transport.abort(ticket);
                }
            }
        }
        if (status == UX_SUCCESS)
        {
            continue;
        }
        if (!failure.accepted)
        {
            cancel_prepared_tx();
            continue;
        }
        transmit_completed(
            failure.completion.requested,
            failure.completion.actual,
            types::status::error, status);
    }
}

types::status initialize_runtime() noexcept
{
    if (PNX_USB_DEVICE_IDENTITY_CONFIRMED == 0)
    {
        return types::status::not_configured;
    }

    if (!mutex_ready)
    {
        if (tx_mutex_create(
                &common_mutex, const_cast<CHAR*>("usb_common"),
                TX_INHERIT) != TX_SUCCESS)
        {
            return types::status::error;
        }
        mutex_ready = true;
    }
    if (!signal_ready)
    {
        if (tx_semaphore_create(
                &write_signal, const_cast<CHAR*>("usb_tx_ready"),
                0U) != TX_SUCCESS)
        {
            return types::status::error;
        }
        signal_ready = true;
    }
    if (!transport_mutex_ready)
    {
        if (tx_mutex_create(
                &transport_mutex,
                const_cast<CHAR*>("usb_transport"),
                TX_INHERIT) != TX_SUCCESS)
        {
            return types::status::error;
        }
        transport_mutex_ready = true;
    }
    if (tx_thread_create(
            &worker_thread, const_cast<CHAR*>("usb_f407_tx"),
            worker_entry, 0U, worker_stack, sizeof(worker_stack),
            active_config.write_priority,
            active_config.write_priority, TX_NO_TIME_SLICE,
            TX_AUTO_START) != TX_SUCCESS)
    {
        return types::status::error;
    }
    return types::status::ok;
}

} // namespace f407_internal

types::status init(const config& cfg) noexcept
{
    using namespace f407_internal;
    if (initialized)
    {
        if (!pnx::f407::usb_detail::same_config(
                active_config, cfg))
        {
            return types::status::invalid_arg;
        }
        return lifecycle.faulted() ? types::status::error
                                   : types::status::ok;
    }
    if (cfg.period_ticks == 0U || cfg.max_tx_size == 0U)
    {
        return types::status::invalid_arg;
    }

    active_config = cfg;
    active_state = {};
    tx_head = 0U;
    tx_tail = 0U;
    tx_count = 0U;
    active_transport = {};
    pending_connection_event.store(
        pnx::f407::usb_detail::connection_event_none,
        std::memory_order_relaxed);
    lifecycle.reset();
    lifecycle.schedule();

    const capabilities hardware = hardware_capabilities();
    if (!hardware.device_mode || hardware.max_transfer_size == 0U ||
        transfer_limit() == 0U)
    {
        active_state.link = link_state::fault;
        ++active_state.error_count;
        lifecycle.fail();
        return types::status::not_configured;
    }

    initialized = true;
    active_state.initialized = true;
    active_state.link = link_state::initialized;
    const types::status result = initialize_runtime();
    if (result != types::status::ok)
    {
        initialized = false;
        active_state.initialized = false;
        active_state.link = link_state::fault;
        ++active_state.error_count;
        lifecycle.fail();
        return result;
    }

    return types::status::ok;
}

write_result write(const std::uint8_t* data, std::size_t len) noexcept
{
    using namespace f407_internal;
    if (data == nullptr || len == 0U ||
        len > common_max_transfer_size)
    {
        return {types::status::invalid_arg, 0U};
    }
    if (!initialized || !lifecycle.transport_usable())
    {
        return {types::status::not_configured, 0U};
    }

    write_state_guard guard{};
    if (!guard)
    {
        return {types::status::busy, 0U};
    }
    if (!active_state.connected || !lifecycle.transport_usable())
    {
        return {types::status::not_configured, 0U};
    }
    if (len > transfer_limit())
    {
        return {types::status::invalid_arg, 0U};
    }
    if (tx_count >= common_tx_queue_depth)
    {
        ++active_state.tx_drop_count;
        return {types::status::busy, 0U};
    }

    tx_slot& slot = tx_queue[tx_head];
    std::memcpy(slot.data.data(), data, len);
    slot.length = static_cast<std::uint16_t>(len);
    tx_head = static_cast<std::uint8_t>(
        (tx_head + 1U) % common_tx_queue_depth);
    ++tx_count;
    update_queue_state();

    signal_tx();
    return {types::status::ok, static_cast<std::uint16_t>(len)};
}

bool connected() noexcept
{
    return snapshot().connected;
}

runtime_state snapshot() noexcept
{
    using namespace f407_internal;
    if (!initialized)
    {
        return active_state;
    }
    state_guard guard{};
    if (!guard)
    {
        return {};
    }
    runtime_state result = active_state;
    if (!lifecycle.transport_usable())
    {
        result.connected = false;
        if (result.link == link_state::connected)
        {
            result.link = lifecycle.faulted()
                              ? link_state::fault
                              : link_state::initialized;
        }
    }
    return result;
}

capabilities get_capabilities() noexcept
{
    using namespace f407_internal;
    capabilities result = hardware_capabilities();
    result.max_transfer_size =
        std::min<std::uint16_t>(
            result.max_transfer_size, common_max_transfer_size);
    result.tx_queue_depth = common_tx_queue_depth;
    return result;
}

const runtime_state& state() noexcept
{
    using namespace f407_internal;
    compatibility_snapshot = snapshot();
    return compatibility_snapshot;
}

} // namespace bsp::usb

extern "C" void usb_cdc_activate(void* cdc_acm_instance)
{
    using namespace bsp::usb::f407_internal;
    auto* cdc =
        static_cast<UX_SLAVE_CLASS_CDC_ACM*>(cdc_acm_instance);
    UX_SLAVE_CLASS_CDC_ACM_CALLBACK_PARAMETER callbacks{};
    callbacks.ux_device_class_cdc_acm_parameter_write_callback =
        cdc_write_complete;
    callbacks.ux_device_class_cdc_acm_parameter_read_callback =
        cdc_read_complete;
    const UINT status = ux_device_class_cdc_acm_ioctl(
        cdc, UX_SLAVE_CLASS_CDC_ACM_IOCTL_TRANSMISSION_START,
        &callbacks);
    if (status != UX_SUCCESS)
    {
        enter_fault(status);
        return;
    }
    {
        transport_guard guard{};
        if (!guard || !active_transport.activate(cdc))
        {
            (void)ux_device_class_cdc_acm_ioctl(
                cdc,
                UX_SLAVE_CLASS_CDC_ACM_IOCTL_TRANSMISSION_STOP,
                UX_NULL);
            enter_fault(UX_ERROR);
            return;
        }
    }
    if (!device_ready(cdc) || !lifecycle.mark_transport_ready())
    {
        {
            transport_guard guard{};
            if (guard)
            {
                (void)active_transport.deactivate(cdc);
            }
        }
        (void)ux_device_class_cdc_acm_ioctl(
            cdc, UX_SLAVE_CLASS_CDC_ACM_IOCTL_TRANSMISSION_STOP,
            UX_NULL);
        enter_fault(UX_ERROR);
        return;
    }
    publish_connection_event(true);
    signal_tx();
}

extern "C" void usb_cdc_deactivate(void* cdc_acm_instance)
{
    using namespace bsp::usb::f407_internal;
    auto* cdc =
        static_cast<UX_SLAVE_CLASS_CDC_ACM*>(cdc_acm_instance);
    bool owned = false;
    {
        transport_guard guard{};
        if (guard)
        {
            owned = active_transport.deactivate(cdc);
        }
    }
    if (cdc != nullptr)
    {
        (void)ux_device_class_cdc_acm_ioctl(
            cdc, UX_SLAVE_CLASS_CDC_ACM_IOCTL_TRANSMISSION_STOP,
            UX_NULL);
    }
    if (!owned)
    {
        return;
    }
    lifecycle.disconnect();
    publish_connection_event(false);
    signal_tx();
}

extern "C" void usb_cdc_parameter_change(void* cdc_acm_instance)
{
    UX_PARAMETER_NOT_USED(cdc_acm_instance);
}
