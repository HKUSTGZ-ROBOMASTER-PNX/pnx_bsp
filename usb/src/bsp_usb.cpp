#include "bsp_usb.hpp"

#include "bridge_usb.h"
#include "memory.h"

#include "ux_api.h"
#include "ux_device_class_cdc_acm.h"

#include <cstring>

namespace bsp::usb
{

namespace
{

constexpr uint16_t k_io_buffer_size = 512;
constexpr uint16_t k_read_stack_size = 1536;
constexpr uint16_t k_write_stack_size = 1536;

alignas(8) uint8_t read_stack[k_read_stack_size] RAM_D1_BSS{};
alignas(8) uint8_t write_stack[k_write_stack_size] RAM_D1_BSS{};
alignas(4) uint8_t rx_buffer[k_io_buffer_size] RAM_D1_BSS{};
alignas(4) uint8_t tx_buffer[k_io_buffer_size] RAM_D1_BSS{};

TX_THREAD read_thread RAM_D1_BSS{};
TX_THREAD write_thread RAM_D1_BSS{};
TX_SEMAPHORE write_signal RAM_D1_BSS{};

config cdc_config{};
runtime_state usb_state{};
bool initialized = false;

bool device_ready()
{
    UX_SLAVE_CLASS_CDC_ACM* cdc = usb_cdc_handle();
    if (cdc == UX_NULL)
    {
        return false;
    }

    UX_SLAVE_DEVICE* device = &_ux_system_slave->ux_system_slave_device;
    return device->ux_slave_device_state == UX_DEVICE_CONFIGURED;
}

void refresh_connection()
{
    usb_state.connected = device_ready();
}

void read_thread_entry(ULONG thread_input)
{
    (void)thread_input;

    while (true)
    {
        refresh_connection();
        if (usb_state.connected && cdc_config.on_rx != nullptr)
        {
            ULONG actual_length = 0;
            UX_SLAVE_CLASS_CDC_ACM* cdc = usb_cdc_handle();
            usb_state.read_busy = true;
            const UINT status = ux_device_class_cdc_acm_read(
                cdc, rx_buffer, static_cast<ULONG>(k_io_buffer_size), &actual_length);
            usb_state.read_busy = false;
            usb_state.last_read_status = status;
            usb_state.last_read_len = static_cast<uint16_t>(actual_length);

            if (status == UX_SUCCESS && actual_length >= cdc_config.min_rx_size)
            {
                ++usb_state.read_count;
                cdc_config.on_rx(rx_buffer, static_cast<uint16_t>(actual_length), cdc_config.user);
                tx_semaphore_put(&write_signal);
            }
            else if (status != UX_SUCCESS)
            {
                ++usb_state.error_count;
            }
        }

        tx_thread_sleep(cdc_config.period_ticks);
    }
}

void write_thread_entry(ULONG thread_input)
{
    (void)thread_input;

    tx_thread_sleep(10);

    while (true)
    {
        refresh_connection();
        if (!usb_state.connected || cdc_config.fill_tx == nullptr)
        {
            tx_thread_sleep(cdc_config.period_ticks);
            continue;
        }

        if (tx_semaphore_get(&write_signal, cdc_config.period_ticks) == TX_SUCCESS)
        {
            ++usb_state.tx_wake_count;
        }

        ++usb_state.fill_count;
        const uint16_t tx_len = cdc_config.fill_tx(tx_buffer, cdc_config.max_tx_size, cdc_config.user);
        usb_state.pending_write_len = tx_len;
        if (tx_len > 0)
        {
            ULONG actual_length = 0;
            UX_SLAVE_CLASS_CDC_ACM* cdc = usb_cdc_handle();
            usb_state.write_busy = true;
            usb_state.last_write_requested = tx_len;
            const UINT status = ux_device_class_cdc_acm_write(
                cdc, tx_buffer, static_cast<ULONG>(tx_len), &actual_length);
            usb_state.write_busy = false;
            usb_state.pending_write_len = 0;
            usb_state.last_write_status = status;
            usb_state.last_write_actual = static_cast<uint16_t>(actual_length);
            if (status == UX_SUCCESS && actual_length == tx_len)
            {
                ++usb_state.write_count;
            }
            else
            {
                ++usb_state.error_count;
            }

            if (cdc_config.on_tx_done != nullptr)
            {
                cdc_config.on_tx_done(
                    tx_len, static_cast<uint16_t>(actual_length), status, cdc_config.user);
            }
        }
    }
}

} // namespace

types::status init(const config& cfg)
{
    if (initialized)
    {
        return types::status::ok;
    }

    if (cfg.on_rx == nullptr && cfg.fill_tx == nullptr)
    {
        return types::status::error;
    }

    cdc_config = cfg;
    if (cdc_config.max_tx_size > k_io_buffer_size)
    {
        cdc_config.max_tx_size = k_io_buffer_size;
    }
    if (cdc_config.write_priority > cdc_config.read_priority)
    {
        cdc_config.write_priority = cdc_config.read_priority;
    }

    if (tx_semaphore_create(&write_signal, const_cast<CHAR*>("usb_cdc_tx_ready"), 0) != TX_SUCCESS)
    {
        return types::status::error;
    }

    if (tx_thread_create(&read_thread, const_cast<CHAR*>("usb_cdc_rx"), read_thread_entry, 0,
                         read_stack, sizeof(read_stack), cdc_config.read_priority, cdc_config.read_priority,
                         TX_NO_TIME_SLICE, TX_AUTO_START) != TX_SUCCESS)
    {
        return types::status::error;
    }

    if (tx_thread_create(&write_thread, const_cast<CHAR*>("usb_cdc_tx"), write_thread_entry, 0,
                         write_stack, sizeof(write_stack), cdc_config.write_priority, cdc_config.write_priority,
                         TX_NO_TIME_SLICE, TX_AUTO_START) != TX_SUCCESS)
    {
        return types::status::error;
    }

    initialized = true;
    return types::status::ok;
}

bool connected()
{
    refresh_connection();
    return usb_state.connected;
}

const runtime_state& state()
{
    refresh_connection();
    return usb_state;
}

} // namespace bsp::usb
