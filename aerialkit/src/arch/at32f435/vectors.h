#ifndef AK_ARCH_AT32F435_VECTORS_H
#define AK_ARCH_AT32F435_VECTORS_H

/*
 * The AT32F435's interrupt vector table.
 *
 * The core vectors are the Cortex-M4's and are the same as the F405's - the
 * reset path is shared, which is why this is a table and not a second
 * startup.c. Everything after index 16 is this part's own numbering, and it is
 * *not* the F405's: the numbers here come from Artery's device header, where
 * USART1/2/3 are 37, 38 and 39 and the two DMA channels a motor frame runs
 * through are 56 and 57.
 *
 *   interrupt 37  USART1          -> index 53   the console
 *   interrupt 38  USART2          -> index 54   the onboard receiver
 *   interrupt 39  USART3          -> index 55   the GPS
 *   interrupt 56  DMA1_Channel1   -> index 72   motor 1's DShot frame
 *   interrupt 57  DMA1_Channel2   -> index 73   motor 2's DShot frame
 *
 * 74 words, which is what scripts/image-facts/at32f435rg.txt carries and what
 * check-image.sh measures on the built image. The F405's table is 56 words
 * because its last interrupt is 39; a shared table would silently have been
 * four words too short here, and a table that is too short is a motor that
 * never finishes a frame.
 */

void Reset_Handler(void);
void Default_Handler(void);
void HardFault_Handler(void);
void SysTick_Handler(void);
void USART1_IRQHandler(void);
void USART2_IRQHandler(void);
void USART3_IRQHandler(void);
void DMA1_Channel1_IRQHandler(void);
void DMA1_Channel2_IRQHandler(void);

extern uint32_t _estack;

__attribute__((section(".isr_vector"), used))
static void (*const ak_vectors[74])(void) = {
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

    Default_Handler, Default_Handler, Default_Handler, Default_Handler, /* 16-19 */
    Default_Handler, Default_Handler, Default_Handler, Default_Handler, /* 20-23 */
    Default_Handler, Default_Handler, Default_Handler, Default_Handler, /* 24-27 */
    Default_Handler, Default_Handler, Default_Handler, Default_Handler, /* 28-31 */
    Default_Handler, Default_Handler, Default_Handler, Default_Handler, /* 32-35 */
    Default_Handler, Default_Handler, Default_Handler, Default_Handler, /* 36-39 */
    Default_Handler, Default_Handler, Default_Handler, Default_Handler, /* 40-43 */
    Default_Handler, Default_Handler, Default_Handler, Default_Handler, /* 44-47 */
    Default_Handler, Default_Handler, Default_Handler, Default_Handler, /* 48-51 */
    Default_Handler,                                                    /* 52    */
    USART1_IRQHandler,                                                  /* 37    */
    USART2_IRQHandler,                                                  /* 38    */
    USART3_IRQHandler,                                                  /* 39    */
    Default_Handler, Default_Handler, Default_Handler, Default_Handler, /* 56-59 */
    Default_Handler, Default_Handler, Default_Handler, Default_Handler, /* 60-63 */
    Default_Handler, Default_Handler, Default_Handler, Default_Handler, /* 64-67 */
    Default_Handler, Default_Handler, Default_Handler, Default_Handler, /* 68-71 */
    DMA1_Channel1_IRQHandler,                                           /* 72    */
    DMA1_Channel2_IRQHandler,                                           /* 73    */
};

#endif /* AK_ARCH_AT32F435_VECTORS_H */
