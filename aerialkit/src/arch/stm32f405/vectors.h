#ifndef AK_ARCH_STM32F405_VECTORS_H
#define AK_ARCH_STM32F405_VECTORS_H

/*
 * The F405's interrupt vector table.
 *
 * This used to live in src/arch/arm/cortex-m4/startup.c, because there was one
 * part and the table was a page of the reset path. It moved here when the
 * second part arrived: the reset path is Cortex-M4 and is shared, and *which
 * interrupts exist and at which position* is the part's own fact. The AT32's
 * table has its USARTs at the same three positions and its motor DMA channels
 * at 56 and 57, so a shared table would have been an F405 vector in an AT32
 * image - the pin-table mistake of [timer_def_at32f43x.h] in interrupt form.
 *
 * The shared file includes "vectors.h" and takes whatever table the arch
 * provides, which keeps one copy of the reset path, the FPU enable and the
 * fault capture. `check-image.sh` checks this table's length and each named
 * entry on the built image, so a table that drifts from the part is a build
 * failure rather than a board that does nothing.
 */

void Reset_Handler(void);
void Default_Handler(void);
void HardFault_Handler(void);
void SysTick_Handler(void);
void DMA1_Stream4_IRQHandler(void);
void USART1_IRQHandler(void);
void USART3_IRQHandler(void);

extern uint32_t _estack;

/* The sixteen core vectors, then this part's external interrupts at their
 * RM0090 positions. Only the ones in use are named; the table runs to the
 * highest of them, and anything past it needs the table extended rather than a
 * handler added.
 *
 *   interrupt 15  DMA1_Stream4  -> index 16 + 15 = 31
 *   interrupt 37  USART1        -> index 16 + 37 = 53
 *   interrupt 39  USART3        -> index 16 + 39 = 55
 *
 * 56 words. scripts/image-facts/stm32f405rg.txt carries that number and the
 * names below, and the built image is checked against both. */
__attribute__((section(".isr_vector"), used))
static void (*const ak_vectors[56])(void) = {
    (void (*)(void))(&_estack), /*  0: initial stack pointer */
    Reset_Handler,              /*  1: reset */
    Default_Handler,            /*  2: NMI */
    HardFault_Handler,          /*  3: hard fault */
    Default_Handler,            /*  4: memory management fault */
    Default_Handler,            /*  5: bus fault */
    Default_Handler,            /*  6: usage fault */
    0, 0, 0, 0,                 /*  7-10: reserved */
    Default_Handler,            /* 11: SVCall */
    Default_Handler,            /* 12: debug monitor */
    0,                          /* 13: reserved */
    Default_Handler,            /* 14: PendSV */
    SysTick_Handler,            /* 15: SysTick */

    Default_Handler, Default_Handler, Default_Handler, Default_Handler, /*  0- 3 */
    Default_Handler, Default_Handler, Default_Handler, Default_Handler, /*  4- 7 */
    Default_Handler, Default_Handler, Default_Handler, Default_Handler, /*  8-11 */
    Default_Handler, Default_Handler, Default_Handler,                  /* 12-14 */
    DMA1_Stream4_IRQHandler,                                            /* 15    */
    Default_Handler, Default_Handler, Default_Handler, Default_Handler, /* 16-19 */
    Default_Handler, Default_Handler, Default_Handler, Default_Handler, /* 20-23 */
    Default_Handler, Default_Handler, Default_Handler, Default_Handler, /* 24-27 */
    Default_Handler, Default_Handler, Default_Handler, Default_Handler, /* 28-31 */
    Default_Handler, Default_Handler, Default_Handler, Default_Handler, /* 32-35 */
    Default_Handler,                                                    /* 36    */
    USART1_IRQHandler,                                                  /* 37    */
    Default_Handler,                                                    /* 38    */
    USART3_IRQHandler,                                                  /* 39    */
};

#endif /* AK_ARCH_STM32F405_VECTORS_H */
