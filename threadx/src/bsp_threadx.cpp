#include "bsp_usb.hpp"
#include "main.h"
#include "tx_api.h"

static constexpr UINT usb_priority = 10;
static constexpr uint32_t usb_timeout_ms = 20;
static constexpr ULONG usb_idle_ticks = 1;
alignas(8) static uint8_t usb_stack[2048];
static TX_THREAD usb_thread;
static char usb_name[] = "usb_rx";

static void usb_entry(ULONG)
{
    for (;;)
    {
        if (bsp::usb::poll(usb_timeout_ms) != pnxtypes::status::ok)
        {
            tx_thread_sleep(usb_idle_ticks);
        }
    }
}

extern "C" __attribute__((weak)) void app_start()
{
}

extern "C" void tx_application_define(void*)
{
    if (bsp::usb::init() != pnxtypes::status::ok)
    {
        Error_Handler();
        return;
    }
    if (tx_thread_create(&usb_thread, usb_name, usb_entry, 0, usb_stack, sizeof(usb_stack),
                         usb_priority, usb_priority, TX_NO_TIME_SLICE, TX_AUTO_START) != TX_SUCCESS)
    {
        Error_Handler();
        return;
    }
    // Auto-start threads run after this initialization hook returns.
    app_start();
}
