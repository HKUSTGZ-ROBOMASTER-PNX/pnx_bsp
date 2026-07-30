#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define BSP_DIAGNOSTICS_ABI_VERSION 1U
#define BSP_DIAGNOSTICS_CRASH_MAGIC 0x504E5844U

enum bsp_diagnostics_reset_reason
{
    BSP_DIAGNOSTICS_RESET_NONE = 0U,
    BSP_DIAGNOSTICS_RESET_BROWNOUT = 1U << 0U,
    BSP_DIAGNOSTICS_RESET_PIN = 1U << 1U,
    BSP_DIAGNOSTICS_RESET_POWER_ON = 1U << 2U,
    BSP_DIAGNOSTICS_RESET_SOFTWARE = 1U << 3U,
    BSP_DIAGNOSTICS_RESET_IWDG = 1U << 4U,
    BSP_DIAGNOSTICS_RESET_WWDG = 1U << 5U,
    BSP_DIAGNOSTICS_RESET_LOW_POWER = 1U << 6U,
};

enum bsp_diagnostics_fault_kind
{
    BSP_DIAGNOSTICS_FAULT_NONE = 0U,
    BSP_DIAGNOSTICS_FAULT_NMI = 1U,
    BSP_DIAGNOSTICS_FAULT_HARD = 2U,
    BSP_DIAGNOSTICS_FAULT_MEMORY = 3U,
    BSP_DIAGNOSTICS_FAULT_BUS = 4U,
    BSP_DIAGNOSTICS_FAULT_USAGE = 5U,
    BSP_DIAGNOSTICS_FAULT_ERROR_HANDLER = 6U,
    BSP_DIAGNOSTICS_FAULT_THREADX_STACK = 7U,
};

typedef struct bsp_diagnostics_crash_record
{
    uint32_t magic;
    uint32_t version;
    uint32_t size;
    uint32_t sequence;
    uint32_t boot_count;
    uint32_t kind;
    uint32_t frame_valid;
    uint32_t exc_return;
    uint32_t msp;
    uint32_t psp;
    uint32_t r0;
    uint32_t r1;
    uint32_t r2;
    uint32_t r3;
    uint32_t r12;
    uint32_t lr;
    uint32_t pc;
    uint32_t xpsr;
    uint32_t cfsr;
    uint32_t hfsr;
    uint32_t dfsr;
    uint32_t afsr;
    uint32_t mmfar;
    uint32_t bfar;
    uint32_t context;
    uint32_t checksum;
} bsp_diagnostics_crash_record;

typedef struct bsp_diagnostics_snapshot
{
    uint32_t boot_count;
    uint32_t reset_flags_raw;
    uint32_t reset_reason_mask;
    uint32_t crash_valid;
    uint32_t crash_from_previous_boot;
    bsp_diagnostics_crash_record crash;
} bsp_diagnostics_snapshot;

uint32_t bsp_diagnostics_record_checksum(
    const bsp_diagnostics_crash_record* record);
int bsp_diagnostics_record_valid(
    const bsp_diagnostics_crash_record* record);

void bsp_diagnostics_boot(void);
#if defined(__GNUC__) || defined(__clang__)
__attribute__((noreturn))
#endif
void bsp_diagnostics_capture(
    enum bsp_diagnostics_fault_kind kind,
    const uint32_t* stacked_frame,
    uint32_t exc_return,
    uint32_t context,
    uint32_t fault_msp,
    uint32_t fault_psp);
#if defined(__GNUC__) || defined(__clang__)
__attribute__((noreturn))
#endif
void bsp_diagnostics_capture_error_handler(uint32_t caller_address);
#if defined(__GNUC__) || defined(__clang__)
__attribute__((noreturn))
#endif
void bsp_diagnostics_capture_threadx_stack(uint32_t thread_address);
void bsp_diagnostics_get_snapshot(bsp_diagnostics_snapshot* snapshot);

extern volatile bsp_diagnostics_crash_record pnx_crash_record;

#ifdef __cplusplus
}
#endif
