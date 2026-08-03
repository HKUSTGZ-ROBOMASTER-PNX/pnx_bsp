.syntax unified
.cpu cortex-m4
.fpu fpv4-sp-d16
.thumb

.extern bsp_diagnostics_capture
.extern pnx_fault_emergency_stack

.macro PNX_FAULT_HANDLER name, kind
    .global \name
    .type \name, %function
    .thumb_func
\name:
    mov r2, lr
    mrs r4, msp
    mrs r5, psp
    tst r2, #4
    beq 1f
    mrs r1, psp
    b 2f
1:
    mrs r1, msp
2:
    tst r2, #16
    bne 3f
    adds r1, r1, #72
3:
    ldr r12, =pnx_fault_emergency_stack + 512
    msr msp, r12
    msr psp, r12
    sub sp, sp, #8
    str r4, [sp, #0]
    str r5, [sp, #4]
    movs r0, #\kind
    movs r3, #0
    b bsp_diagnostics_capture
    .size \name, . - \name
.endm

PNX_FAULT_HANDLER NMI_Handler, 1
PNX_FAULT_HANDLER HardFault_Handler, 2
PNX_FAULT_HANDLER MemManage_Handler, 3
PNX_FAULT_HANDLER BusFault_Handler, 4
PNX_FAULT_HANDLER UsageFault_Handler, 5
