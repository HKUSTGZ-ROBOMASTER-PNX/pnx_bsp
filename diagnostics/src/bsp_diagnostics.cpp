#include "bsp_diagnostics.h"
#include "diagnostics_f407.hpp"

#include "stm32f4xx.h"

#include <cstddef>
#include <cstdint>
#include <limits>

extern "C" {

volatile bsp_diagnostics_crash_record pnx_crash_record
    __attribute__((section(".noinit"), used));

alignas(8) std::uint8_t pnx_fault_emergency_stack[512];

}

namespace
{

static_assert(BSP_DIAGNOSTICS_FAULT_NMI == 1U, "fault ABI changed");
static_assert(BSP_DIAGNOSTICS_FAULT_HARD == 2U, "fault ABI changed");
static_assert(BSP_DIAGNOSTICS_FAULT_MEMORY == 3U, "fault ABI changed");
static_assert(BSP_DIAGNOSTICS_FAULT_BUS == 4U, "fault ABI changed");
static_assert(BSP_DIAGNOSTICS_FAULT_USAGE == 5U, "fault ABI changed");
static_assert(BSP_DIAGNOSTICS_FAULT_ERROR_HANDLER == 6U,
              "fault ABI changed");
static_assert(BSP_DIAGNOSTICS_FAULT_THREADX_STACK == 7U,
              "fault ABI changed");

constexpr std::uint32_t boot_magic = 0x504E5842U;
constexpr std::uint32_t fnv_offset = 2166136261U;
constexpr std::uint32_t fnv_prime = 16777619U;
struct persistent_boot_state
{
    std::uint32_t magic;
    std::uint32_t version;
    std::uint32_t boot_count;
    std::uint32_t reset_flags_raw;
    std::uint32_t reset_reason_mask;
    std::uint32_t checksum;
};

volatile persistent_boot_state boot_state
    __attribute__((section(".noinit"), used));

std::uint32_t checksum_bytes_volatile(
    const volatile void* data, std::size_t length) noexcept
{
    if (data == nullptr)
    {
        return 0U;
    }
    const auto* bytes =
        static_cast<const volatile std::uint8_t*>(data);
    std::uint32_t hash = fnv_offset;
    for (std::size_t index = 0U; index < length; ++index)
    {
        hash ^= bytes[index];
        hash *= fnv_prime;
    }
    return hash;
}

std::uint32_t boot_checksum_volatile() noexcept
{
    return checksum_bytes_volatile(
        &boot_state, offsetof(persistent_boot_state, checksum));
}

bool boot_valid_volatile() noexcept
{
    return boot_state.magic == boot_magic &&
           boot_state.version == BSP_DIAGNOSTICS_ABI_VERSION &&
           boot_state.checksum == boot_checksum_volatile();
}

std::uint32_t crash_checksum_volatile() noexcept
{
    return checksum_bytes_volatile(
        &pnx_crash_record,
        offsetof(bsp_diagnostics_crash_record, checksum));
}

bool crash_valid_volatile() noexcept
{
    return pnx_crash_record.magic == BSP_DIAGNOSTICS_CRASH_MAGIC &&
           pnx_crash_record.version ==
               BSP_DIAGNOSTICS_ABI_VERSION &&
           pnx_crash_record.size ==
               sizeof(bsp_diagnostics_crash_record) &&
           pnx_crash_record.checksum ==
               crash_checksum_volatile();
}

std::uint32_t current_boot_count() noexcept
{
    return boot_valid_volatile() ? boot_state.boot_count : 0U;
}

bool frame_range_valid(const std::uint32_t* frame,
                       std::uint32_t cfsr) noexcept
{
    if (frame == nullptr ||
        !pnx::board::f407::diagnostics::
            basic_frame_pointer_valid(
                reinterpret_cast<std::uintptr_t>(frame), cfsr))
    {
        return false;
    }
    return true;
}

void clear_frame_fields() noexcept
{
    pnx_crash_record.frame_valid = 0U;
    pnx_crash_record.r0 = 0U;
    pnx_crash_record.r1 = 0U;
    pnx_crash_record.r2 = 0U;
    pnx_crash_record.r3 = 0U;
    pnx_crash_record.r12 = 0U;
    pnx_crash_record.lr = 0U;
    pnx_crash_record.pc = 0U;
    pnx_crash_record.xpsr = 0U;
}

void copy_crash_to_snapshot(
    bsp_diagnostics_crash_record& destination) noexcept
{
    destination.magic = pnx_crash_record.magic;
    destination.version = pnx_crash_record.version;
    destination.size = pnx_crash_record.size;
    destination.sequence = pnx_crash_record.sequence;
    destination.boot_count = pnx_crash_record.boot_count;
    destination.kind = pnx_crash_record.kind;
    destination.frame_valid = pnx_crash_record.frame_valid;
    destination.exc_return = pnx_crash_record.exc_return;
    destination.msp = pnx_crash_record.msp;
    destination.psp = pnx_crash_record.psp;
    destination.r0 = pnx_crash_record.r0;
    destination.r1 = pnx_crash_record.r1;
    destination.r2 = pnx_crash_record.r2;
    destination.r3 = pnx_crash_record.r3;
    destination.r12 = pnx_crash_record.r12;
    destination.lr = pnx_crash_record.lr;
    destination.pc = pnx_crash_record.pc;
    destination.xpsr = pnx_crash_record.xpsr;
    destination.cfsr = pnx_crash_record.cfsr;
    destination.hfsr = pnx_crash_record.hfsr;
    destination.dfsr = pnx_crash_record.dfsr;
    destination.afsr = pnx_crash_record.afsr;
    destination.mmfar = pnx_crash_record.mmfar;
    destination.bfar = pnx_crash_record.bfar;
    destination.context = pnx_crash_record.context;
    destination.checksum = pnx_crash_record.checksum;
}

} // namespace

extern "C" {

std::uint32_t bsp_diagnostics_record_checksum(
    const bsp_diagnostics_crash_record* record)
{
    if (record == nullptr)
    {
        return 0U;
    }
    const auto* bytes =
        reinterpret_cast<const std::uint8_t*>(record);
    std::uint32_t hash = fnv_offset;
    for (std::size_t index = 0U;
         index < offsetof(bsp_diagnostics_crash_record, checksum);
         ++index)
    {
        hash ^= bytes[index];
        hash *= fnv_prime;
    }
    return hash;
}

int bsp_diagnostics_record_valid(
    const bsp_diagnostics_crash_record* record)
{
    return record != nullptr &&
                   record->magic == BSP_DIAGNOSTICS_CRASH_MAGIC &&
                   record->version == BSP_DIAGNOSTICS_ABI_VERSION &&
                   record->size == sizeof(*record) &&
                   record->checksum ==
                       bsp_diagnostics_record_checksum(record)
               ? 1
               : 0;
}

void bsp_diagnostics_boot(void)
{
    const bool valid = boot_valid_volatile();
    const std::uint32_t next_boot_count =
        valid &&
                boot_state.boot_count !=
                    std::numeric_limits<std::uint32_t>::max()
            ? boot_state.boot_count + 1U
            : 1U;
    const std::uint32_t reset_flags = RCC->CSR;

    // Checksum is written last. A reset during this update leaves an invalid
    // record rather than a partially valid one.
    boot_state.checksum = 0U;
    boot_state.magic = boot_magic;
    boot_state.version = BSP_DIAGNOSTICS_ABI_VERSION;
    boot_state.boot_count = next_boot_count;
    boot_state.reset_flags_raw = reset_flags;
    boot_state.reset_reason_mask =
        pnx::board::f407::diagnostics::decode_reset_reasons(
            reset_flags);
    boot_state.checksum = boot_checksum_volatile();
    __DSB();
    RCC->CSR |= RCC_CSR_RMVF;
}

void bsp_diagnostics_capture(enum bsp_diagnostics_fault_kind kind,
                             const std::uint32_t* stacked_frame,
                             std::uint32_t exc_return,
                             std::uint32_t context,
                             std::uint32_t fault_msp,
                             std::uint32_t fault_psp)
{
    __disable_irq();

    const bool previous_valid = crash_valid_volatile();
    const std::uint32_t previous_sequence =
        previous_valid ? pnx_crash_record.sequence : 0U;
    const std::uint32_t sequence =
        previous_sequence !=
                std::numeric_limits<std::uint32_t>::max()
            ? previous_sequence + 1U
            : 1U;
    const std::uint32_t cfsr = SCB->CFSR;

    // Invalidate first and publish the checksum last.
    pnx_crash_record.checksum = 0U;
    pnx_crash_record.magic = BSP_DIAGNOSTICS_CRASH_MAGIC;
    pnx_crash_record.version = BSP_DIAGNOSTICS_ABI_VERSION;
    pnx_crash_record.size = sizeof(bsp_diagnostics_crash_record);
    pnx_crash_record.sequence = sequence;
    pnx_crash_record.boot_count = current_boot_count();
    pnx_crash_record.kind = static_cast<std::uint32_t>(kind);
    pnx_crash_record.exc_return = exc_return;
    pnx_crash_record.msp = fault_msp;
    pnx_crash_record.psp = fault_psp;
    pnx_crash_record.context = context;
    clear_frame_fields();
    if (frame_range_valid(stacked_frame, cfsr))
    {
        pnx_crash_record.frame_valid = 1U;
        pnx_crash_record.r0 = stacked_frame[0];
        pnx_crash_record.r1 = stacked_frame[1];
        pnx_crash_record.r2 = stacked_frame[2];
        pnx_crash_record.r3 = stacked_frame[3];
        pnx_crash_record.r12 = stacked_frame[4];
        pnx_crash_record.lr = stacked_frame[5];
        pnx_crash_record.pc = stacked_frame[6];
        pnx_crash_record.xpsr = stacked_frame[7];
    }
    pnx_crash_record.cfsr = cfsr;
    pnx_crash_record.hfsr = SCB->HFSR;
    pnx_crash_record.dfsr = SCB->DFSR;
    pnx_crash_record.afsr = SCB->AFSR;
    pnx_crash_record.mmfar = SCB->MMFAR;
    pnx_crash_record.bfar = SCB->BFAR;
    pnx_crash_record.checksum = crash_checksum_volatile();
    __DSB();
    __ISB();
    for (;;)
    {
        __NOP();
    }
}

__attribute__((naked, noreturn))
void bsp_diagnostics_capture_error_handler(
    std::uint32_t caller_address)
{
    __asm volatile(
        "mov r3, r0\n"
        "mrs r4, msp\n"
        "mrs r5, psp\n"
        "ldr r12, =pnx_fault_emergency_stack + 512\n"
        "msr msp, r12\n"
        "msr psp, r12\n"
        "mrs r12, control\n"
        "bic r12, r12, #2\n"
        "msr control, r12\n"
        "isb\n"
        "push {r4, r5}\n"
        "movs r0, #6\n"
        "movs r1, #0\n"
        "movs r2, #0\n"
        "b bsp_diagnostics_capture\n");
}

__attribute__((naked, noreturn))
void bsp_diagnostics_capture_threadx_stack(
    std::uint32_t thread_address)
{
    __asm volatile(
        "mov r3, r0\n"
        "mrs r4, msp\n"
        "mrs r5, psp\n"
        "ldr r12, =pnx_fault_emergency_stack + 512\n"
        "msr msp, r12\n"
        "msr psp, r12\n"
        "mrs r12, control\n"
        "bic r12, r12, #2\n"
        "msr control, r12\n"
        "isb\n"
        "push {r4, r5}\n"
        "movs r0, #7\n"
        "movs r1, #0\n"
        "movs r2, #0\n"
        "b bsp_diagnostics_capture\n");
}

void bsp_diagnostics_get_snapshot(
    bsp_diagnostics_snapshot* snapshot)
{
    if (snapshot == nullptr)
    {
        return;
    }
    *snapshot = {};
    if (boot_valid_volatile())
    {
        snapshot->boot_count = boot_state.boot_count;
        snapshot->reset_flags_raw = boot_state.reset_flags_raw;
        snapshot->reset_reason_mask =
            boot_state.reset_reason_mask;
    }
    copy_crash_to_snapshot(snapshot->crash);
    snapshot->crash_valid =
        bsp_diagnostics_record_valid(&snapshot->crash) != 0 ? 1U : 0U;
    snapshot->crash_from_previous_boot =
        snapshot->crash_valid != 0U &&
                snapshot->crash.boot_count + 1U ==
                    snapshot->boot_count
            ? 1U
            : 0U;
}

} // extern "C"
