/*
 * Bit positions in the port's register definitions, checked on the host.
 *
 * These are the numbers that are wrong once and then cost a day: a stream
 * number, a flag offset, a field shift. They cannot be checked by running the
 * firmware - a wrong bit just does nothing, on a board, silently - so they are
 * checked here against the values read out of RM0090, where a test can fail
 * out loud.
 *
 * The include is a relative path on purpose: the host build does not put the
 * port's include directory on the search path, and it should not, because
 * src/core is not allowed to see it.
 */

#include <stdint.h>

#include "../src/arch/stm32f405/regs.h"
#include "tests.h"

void test_register_encodings(void)
{
    /* The ROM bootloader's address, which the `dfu` command jumps to: AN2606's
     * table for the STM32F40x puts the system memory at 0x1FFF0000, and a jump
     * anywhere else is a jump into nothing. The jump itself cannot be run here -
     * the host has nothing at that address - so what is pinned is the number. */
    expect("the rom bootloader is where the manual puts it",
           AK_BOOTLOADER_BASE == 0x1FFF0000u);

    /* RCC: the clock enables this firmware switches on. */
    expect("RCC APB1 timer 2 and 3 enables are bits 0 and 1",
           RCC_APB1ENR_TIM2EN == 0x1u && RCC_APB1ENR_TIM3EN == 0x2u);
    expect("RCC AHB1 DMA1 enable is bit 21", RCC_AHB1ENR_DMA1EN == (1u << 21));
    expect("GPIOA and GPIOC enables are bits 0 and 2",
           RCC_AHB1ENR_GPIOAEN == (1u << 0) && RCC_AHB1ENR_GPIOCEN == (1u << 2));
    expect("RCC APB2 ADC1 enable is bit 8", RCC_APB2ENR_ADC1EN == (1u << 8));

    /* USART frame format: the two bits that make an SBUS port out of the same
     * register as a CRSF one. Parity enable at bit 10 with PS at 9 choosing
     * even, and the two stop bits at CR2[13:12] = 10. RM0090 30.6.4/30.6.5. */
    expect("USART CR1: parity is PS bit 9 and PCE bit 10",
           USART_CR1_PS == (1u << 9) && USART_CR1_PCE == (1u << 10));
    expect("USART CR2: two stop bits is 10 at bit 12",
           USART_CR2_STOP_2 == (0x2u << 12));

    /* I2C: the enables, the flag positions and the two mode bits, all of which
     * are silent when wrong. RM0090 27.6. */
    expect("I2C1-3 are at 0x40005400, 0x40005800 and 0x40005C00",
           I2C1_BASE == 0x40005400UL && I2C2_BASE == 0x40005800UL &&
               I2C3_BASE == 0x40005C00UL);
    expect("RCC APB1 I2C1-3 enables are bits 21, 22 and 23",
           RCC_APB1ENR_I2C1EN == (1u << 21) &&
               RCC_APB1ENR_I2C2EN == (1u << 22) &&
               RCC_APB1ENR_I2C3EN == (1u << 23));
    expect("I2C CR1: PE bit 0, START 8, STOP 9, ACK 10, POS 11, SWRST 15",
           I2C_CR1_PE == (1u << 0) && I2C_CR1_START == (1u << 8) &&
               I2C_CR1_STOP == (1u << 9) && I2C_CR1_ACK == (1u << 10) &&
               I2C_CR1_POS == (1u << 11) && I2C_CR1_SWRST == (1u << 15));
    expect("I2C SR1: SB 0, ADDR 1, BTF 2, RXNE 6, TXE 7, AF 10",
           I2C_SR1_SB == (1u << 0) && I2C_SR1_ADDR == (1u << 1) &&
               I2C_SR1_BTF == (1u << 2) && I2C_SR1_RXNE == (1u << 6) &&
               I2C_SR1_TXE == (1u << 7) && I2C_SR1_AF == (1u << 10));
    expect("I2C CR2's frequency field is the low six bits",
           I2C_CR2_FREQ_MASK == 0x3Fu);

    /* USB OTG FS. These are the bits whose being wrong makes a device that
     * never enumerates, or enumerates at the wrong speed, or answers the wrong
     * endpoint - none of which reports itself anywhere. RM0090 33.12. */
    expect("the OTG FS blocks sit where the manual says",
           OTG_FS_BASE == 0x50000000UL && OTG_FS_DEVICE_BASE == 0x50000800UL);
    expect("RCC AHB2 gates the OTG FS at bit 7",
           RCC_AHB2ENR_OTGFSEN == (1u << 7));
    expect("GUSBCFG's PHYSEL is bit 6: the full-speed serial transceiver",
           OTG_GUSBCFG_PHYSEL == (1u << 6));
    expect("DCFG's speed field is 11 for full speed",
           OTG_DCFG_DSPD_FS == (0x3u << 0));
    expect("DCTL: RWUSIG bit 0 and the soft disconnect bit 1",
           OTG_DCTL_RWUSIG == 1u && OTG_DCTL_SDIS == 0x2u);
    expect("GRSTCTL: CSRST bit 0, TXFFLSH bit 5, AHBIDLE bit 31",
           OTG_GRSTCTL_CSRST == 1u && OTG_GRSTCTL_TXFFLSH == (1u << 5) &&
               OTG_GRSTCTL_AHBIDLE == (1u << 31));
    expect("GINTSTS: RXFLVL bit 4, USBRST 12, ENUMDNE 13, IEPINT 18, OEPINT 19",
           OTG_GINT_RXFLVL == (1u << 4) && OTG_GINT_USBRST == (1u << 12) &&
               OTG_GINT_ENUMDNE == (1u << 13) &&
               OTG_GINT_IEPINT == (1u << 18) &&
               OTG_GINT_OEPINT == (1u << 19));
    expect("an endpoint control word: EPENA 31, CNAK 26, STALL 21, USBAEP 15",
           OTG_DEPCTL_EPENA == (1u << 31) && OTG_DEPCTL_CNAK == (1u << 26) &&
               OTG_DEPCTL_STALL == (1u << 21) &&
               OTG_DEPCTL_USBAEP == (1u << 15));
    expect("a bulk IN endpoint names its FIFO in bits 22-25",
           OTG_DEPCTL_TXFNUM(1u) == (1u << 22) &&
               OTG_DEPCTL_EPTYP_BULK == (2u << 18));
    expect("endpoint 0's transfer size is seven bits and one packet",
           OTG_DIEPTSIZ0_XFRSIZ(64u) == 64u &&
               OTG_DIEPTSIZ0_PKTCNT1 == (1u << 19));
    /* Their addresses, taken without reading them: this test has no mapped
     * peripheral region, and these two registers are the pair whose being
     * confused is the bug the test file in test_arch.c exists for. */
    expect("the receive status queue is its own register, not the FIFO",
           (uintptr_t)&OTG_GRXSTSP == 0x50000020UL &&
               (uintptr_t)&OTG_FIFO(0u) == 0x50001000UL);
    expect("a status entry: endpoint 0-3, byte count from bit 4, kind from 17",
           OTG_GRXSTS_EPNUM == 0xFu && OTG_GRXSTS_BCNT == (0x7FFu << 4) &&
               OTG_GRXSTS_PKTSTS == (0xFu << 17));
    expect("a data packet is kind 2 and a setup packet kind 6",
           OTG_GRXSTS_DATA == 2u && OTG_GRXSTS_SETUP == 6u);
    expect("a bulk endpoint's transfer size is nineteen bits, packet count "
           "above it",
           OTG_DOEPTSIZ_XFRSIZ(64u) == 64u &&
               OTG_DOEPTSIZ_PKTCNT(1u) == (1u << 19));

    /* The I2C registers by offset, which is how the seam in i2c.c names them
     * when a host build puts a modelled bus behind it. They are the same seven
     * the macros above name; a seam that pointed at the wrong one would model
     * a peripheral that does not exist. */
    expect("the i2c registers have the offsets the manual gives them",
           I2C_OFF_CR1 == 0x00u && I2C_OFF_CR2 == 0x04u && I2C_OFF_DR == 0x10u &&
               I2C_OFF_SR1 == 0x14u && I2C_OFF_SR2 == 0x18u &&
               I2C_OFF_CCR == 0x1Cu && I2C_OFF_TRISE == 0x20u);

    /* ADC: the addresses, the enable, the software start and the flag. These
     * are the ones where being wrong is silent - a wrong address enables a
     * clock nobody uses, and a conversion that never starts has no error bit
     * to look at. RM0090 11.12 for the register map, 6.3.15 for the enable. */
    expect("ADC1 is at 0x40012000 and the common block at 0x40012300",
           ADC1_BASE == 0x40012000UL && ADC_COMMON_BASE == 0x40012300UL);
    expect("ADC CR2: ADON is bit 0, CONT is bit 1",
           ADC_CR2_ADON == 0x1u && ADC_CR2_CONT == 0x2u);
    expect("ADC CR2: end-of-conversion select is bit 10",
           ADC_CR2_EOCS == (1u << 10));
    expect("ADC CR2: the software start is bit 30",
           ADC_CR2_SWSTART == (1u << 30));
    expect("ADC SR: end of conversion is bit 1", ADC_SR_EOC == 0x2u);
    expect("ADC common prescaler /4 is 01 at bit 16",
           ADC_CCR_ADCPRE_DIV4 == (1u << 16));

    /* A sequence of one is L = 0, not L = 1: the field is the length minus
     * one, and getting it wrong converts twice as many channels as intended
     * and returns the second one. */
    expect("a regular sequence of one conversion is L = 0",
           ADC_SQR1_L(1u) == 0u && ADC_SQR1_L(4u) == (3u << 20));
    expect("the first regular channel is the low five bits of SQR3",
           ADC_SQR3_SQ1(10u) == 10u && ADC_SQR3_SQ1(0u) == 0u);
    expect("channel 10's sample time is in SMPR1, channel 3's in SMPR2",
           ADC_SMPR1_SMP(10u) == 0x7u &&
               ADC_SMPR2_SMP(3u) == (0x7u << 9));

    /* DMA stream control: direction, increments, both sizes 16-bit, priority,
     * transfer-complete interrupt, and the channel selection in the top byte. */
    expect("DMA direction memory to peripheral is 01",
           DMA_SxCR_DIR_M2P == (0x1u << 6));
    expect("DMA minc and pinc are bits 10 and 9",
           DMA_SxCR_MINC == (1u << 10) && DMA_SxCR_PINC == (1u << 9));
    expect("DMA PSIZE 16-bit is 01 at bit 11",
           (DMA_SxCR_PSIZE_16 >> 11) == 0x1u);
    expect("DMA MSIZE 16-bit is 01 at bit 13",
           (DMA_SxCR_MSIZE_16 >> 13) == 0x1u);
    expect("DMA high priority is 10 at bit 16", (DMA_SxCR_PL_HIGH >> 16) == 0x2u);
    expect("DMA channel selection sits at bit 25",
           DMA_SxCR_CHSEL(5) == (5u << 25) && DMA_SxCR_CHSEL(3) == (3u << 25));

    /* The flag registers cover streams 0-3 low and 4-7 high, six bits per
     * stream, and transfer-complete is the fifth of those six. */
    expect("transfer-complete for stream 1 is bit 11 of the low register",
           DMA_FLAG_TC(1u) == (1u << 11));
    expect("transfer-complete for stream 4 is bit 5 of the high register",
           DMA_FLAG_TC(4u) == (1u << 5));
    expect("clearing stream 4 clears the whole six-bit group",
           DMA_IFCR_CLEAR(4u) == 0x3Du);
    expect("the timer's DMA stream and channel are the ones INAV's table names",
           AK_DMA_TIM3_STREAM == 4u && AK_DMA_TIM3_CHANNEL == 5u);
    expect("DMA1 stream 4 is interrupt 15 - the vector table has to reach it",
           DMA1_STREAM4_IRQ == 15u);

    /* Timer DMA burst: DBA counts 32-bit words from the timer base, so CCR1 at
     * 0x34 is word 13, and DBL of 3 asks for four 16-bit transfers. */
    expect("the burst base is CCR1", TIM_DCR_DBA(0x34u / 4u) == 13u);
    expect("four burst transfers", TIM_DCR_DBL(3u) == (3u << 8));

    /* Timer registers this firmware writes, and the channel output enables.
     * Nothing here dereferences a register: a test that touches address
     * 0x40000000 on a development machine would be a segfault, not a failure. */
    expect("CCxE is bit 0 for channel 1 and walks up by four",
           TIM_CCER_CCE(1u) == 0x1u && TIM_CCER_CCE(2u) == 0x10u &&
           TIM_CCER_CCE(3u) == 0x100u && TIM_CCER_CCE(4u) == 0x1000u);
    expect("PWM mode 1 is 110", TIM_CCMR_OCM_PWM1 == 6u);
    expect("the timers are on the base addresses RM0090 gives them",
           TIM2_BASE == 0x40000000UL && TIM3_BASE == 0x40000400UL);
    expect("CCR1 is 0x34 and DMAR is 0x4c above the timer base",
           (0x34u / 4u) == 13u && (0x4Cu - 0x34u) == 24u);
}
