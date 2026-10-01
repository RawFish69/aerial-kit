#include <stdint.h>

#include "arch.h"
#include "ak_fault.h"

/*
 * Reset path and vector table for the Cortex-M4.
 *
 * The *table* is not here any more. Which interrupts exist, and at which
 * position, is the part's own fact rather than the core's: the F405's motor
 * DMA is stream 4 and the AT32's is channel 1, at interrupt 15 and 56. What is
 * shared is above and below the table - the FPU enable, the fault capture, the
 * .data copy and the .bss clear - and the arch supplies the table through
 * "vectors.h". Each named entry is checked against the built image by
 * scripts/check-image.sh, so a table that drifts from the part fails the build
 * instead of producing a board that does nothing.
 *
 * AerialKit enables no peripheral interrupt it does not use; each one gets
 * added to the arch's table at its part's position in the milestone that
 * starts using it, rather than shipping a copied-out table nobody has checked.
 */

extern uint32_t _sidata, _sdata, _edata, _sbss, _ebss, _estack;

int ak_firmware_main(void);

void Reset_Handler(void);
void Default_Handler(void);

/* Reads the stacked frame and stops. Defined in fault.c. */
void ak_fault_capture(uint32_t *frame);

/* The hard fault handler has to look at the stack pointer itself before it can
 * call anything, because it does not know whether the faulting code was using
 * the main stack or the process stack. Six instructions of assembly is the
 * whole of it. */
__attribute__((naked, used))
void HardFault_Handler(void)
{
    __asm("tst lr, #4");
    __asm("ite eq");
    __asm("mrseq r0, msp");
    __asm("mrsne r0, psp");
    __asm("ldr r1, =ak_fault_capture");
    __asm("bx r1");
}

/* CP10 and CP11 full access, then a barrier so no floating-point instruction
 * is fetched before the coprocessor is on. The image is built hard-float, so
 * this has to happen before anything else that might use one. */
void ak_fpu_enable(void)
{
    SCB_CPACR |= (0xFu << 20);
    __asm volatile("dsb");
    __asm volatile("isb");
}

/* The part's own table, at the base of .isr_vector. Defined in the header so
 * that a second part's table is a second header rather than a second reset
 * path. */
#include "vectors.h"

void Default_Handler(void)
{
    /* A fault has nowhere to report yet. Stop here, visibly: the board stops
     * blinking and a debugger sees where. Fault reporting is its own work item
     * and it lands before anything flies. */
    for (;;) {
    }
}

/* A part's chance to act before anything is initialised (stm32f405/system.c).
 * Weak and undefined elsewhere, so the call is skipped on parts without one. */
void ak_arch_early(void) __attribute__((weak));

void Reset_Handler(void)
{
    if (ak_arch_early != 0) {
        ak_arch_early();
    }
    ak_fpu_enable();

    /* The vector table lives at the base of flash. On the STM32F405 the reset
     * value of VTOR is 0, which aliases there only because BOOT0 selects main
     * flash; saying it explicitly makes a later bootloader move a one-line
     * change here. */
    SCB_VTOR = (uint32_t)AK_FLASH_BASE;

    uint32_t *src = &_sidata;
    for (uint32_t *dst = &_sdata; dst < &_edata; dst++) {
        *dst = *src++;
    }
    for (uint32_t *dst = &_sbss; dst < &_ebss; dst++) {
        *dst = 0;
    }

    ak_firmware_main();

    for (;;) {
    }
}
