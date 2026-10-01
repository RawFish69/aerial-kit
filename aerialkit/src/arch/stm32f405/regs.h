#ifndef AK_ARCH_STM32F405_REGS_H
#define AK_ARCH_STM32F405_REGS_H

#include <stdint.h>

/*
 * Hand-written STM32F405 register definitions - the subset AerialKit uses.
 *
 * Addresses and bit positions come from RM0090 (STM32F405/407/415/417
 * reference manual): the memory map and section 6.3 for RCC, section 3.9 for
 * FLASH, section 8 for GPIO, section 30 for USART, and PM0214 for SysTick and
 * the SCB. Nothing here is copied from ST's CMSIS device headers or from any
 * flight firmware; the point is that every line is short enough to check
 * against the manual.
 *
 * Only what is used is defined. New peripherals get added here with the manual
 * section that justifies them.
 */

#define AK_REG32(addr) (*(volatile uint32_t *)(uintptr_t)(addr))

/* --- memory map (RM0090 Table 1) --- */
#define AK_FLASH_BASE 0x08000000UL

/* The ROM bootloader - the DFU that writes this part's flash - lives in the
 * system memory at 0x1FFF0000 (AN2606's table for the STM32F40x), and it is
 * entered by a vector-table jump: the first word of what is there is the stack
 * pointer and the second is where to branch. See src/arch/stm32f405/system.c,
 * where that jump is the `dfu` command's other half. */
#define AK_BOOTLOADER_BASE 0x1FFF0000UL

/* The backup domain's first register, which survives a system reset: the
 * one-word message a boot leaves for the next one (system.c). */
#define PWR_CR          AK_REG32(0x40007000UL)
#define PWR_CR_DBP      (1u << 8)
#define RTC_BKP0R       AK_REG32(0x40002850UL)
#define SYSCFG_MEMRMP   AK_REG32(0x40013800UL)
#define RCC_APB2ENR_SYSCFGEN (1u << 14)

/* TIM11, used once at boot as a frequency counter: its channel 1 can be fed
 * HSE / RTCPRE instead of a pin (TIM11_OR TI1_RMP = 2), which is how the
 * clock layer measures the crystal against HSI. */
#define RCC_APB2ENR_TIM11EN  (1u << 18)
#define TIM11_BASE      0x40014800UL
#define TIM11_CR1       AK_REG32(TIM11_BASE + 0x00)
#define TIM11_SR        AK_REG32(TIM11_BASE + 0x10)
#define TIM11_EGR       AK_REG32(TIM11_BASE + 0x14)
#define TIM11_CCMR1     AK_REG32(TIM11_BASE + 0x18)
#define TIM11_CCER      AK_REG32(TIM11_BASE + 0x20)
#define TIM11_PSC       AK_REG32(TIM11_BASE + 0x28)
#define TIM11_ARR       AK_REG32(TIM11_BASE + 0x2C)
#define TIM11_CCR1      AK_REG32(TIM11_BASE + 0x34)
#define TIM11_OR        AK_REG32(TIM11_BASE + 0x50)
#define AK_SRAM_BASE  0x20000000UL
#define AK_CCM_BASE   0x10000000UL

/* --- RCC (RM0090 6.3) --- */
#define RCC_BASE       0x40023800UL
#define RCC_CR         AK_REG32(RCC_BASE + 0x00)
#define RCC_PLLCFGR    AK_REG32(RCC_BASE + 0x04)
#define RCC_CFGR       AK_REG32(RCC_BASE + 0x08)
#define RCC_AHB1ENR    AK_REG32(RCC_BASE + 0x30)
#define RCC_AHB2ENR    AK_REG32(RCC_BASE + 0x34)
#define RCC_APB1ENR    AK_REG32(RCC_BASE + 0x40)
#define RCC_APB2ENR    AK_REG32(RCC_BASE + 0x44)

#define RCC_CR_HSION   (1u << 0)
#define RCC_CR_HSIRDY  (1u << 1)
#define RCC_CR_HSEON   (1u << 16)
#define RCC_CR_HSERDY  (1u << 17)
#define RCC_CR_PLLON   (1u << 24)
#define RCC_CR_PLLRDY  (1u << 25)

/* PLLCFGR: PLLM[5:0], PLLN[14:6], PLLP[17:16], PLLSRC[22], PLLQ[26:24].
 *
 * PLLP is the trap in this register. RM0090 6.3.2 encodes it as
 * PLLP = 2 * (field + 1), so field 0 is divide-by-2 and field 1 is
 * divide-by-4 - not "field = P / 2" as the width suggests. Getting this wrong
 * by one halves the system clock while the rest of the firmware still believes
 * it is at 168 MHz, which is exactly the kind of failure that shows up later
 * as a garbled console and a slow tick. */
#define RCC_PLL_M(m)      ((uint32_t)(m) & 0x3Fu)
#define RCC_PLL_N(n)      (((uint32_t)(n) & 0x1FFu) << 6)
#define RCC_PLL_P(p)      (((((uint32_t)(p) / 2u) - 1u) & 0x3u) << 16)
#define RCC_PLL_SRC_HSE   (1u << 22)
#define RCC_PLL_Q(q)      (((uint32_t)(q) & 0xFu) << 24)

/* CFGR: SW[1:0], SWS[3:2], HPRE[7:4], PPRE1[12:10], PPRE2[15:13]. */
#define RCC_CFGR_SW_PLL     (0x2u << 0)
#define RCC_CFGR_SW_MASK    (0x3u << 0)
#define RCC_CFGR_SWS_MASK   (0x3u << 2)
#define RCC_CFGR_SWS_PLL    (0x2u << 2)
#define RCC_CFGR_HPRE_DIV1  (0x0u << 4)
#define RCC_CFGR_PPRE1_DIV1 (0x0u << 10)
#define RCC_CFGR_PPRE1_DIV2 (0x4u << 10)
#define RCC_CFGR_PPRE1_DIV4 (0x5u << 10)
#define RCC_CFGR_PPRE2_DIV1 (0x0u << 13)
#define RCC_CFGR_PPRE2_DIV2 (0x4u << 13)

#define RCC_AHB1ENR_GPIOAEN      (1u << 0)
#define RCC_AHB1ENR_GPIOBEN      (1u << 1)
#define RCC_AHB1ENR_GPIOCEN      (1u << 2)
#define RCC_AHB1ENR_GPIODEN      (1u << 3)
#define RCC_AHB1ENR_GPIOEEN      (1u << 4)
#define RCC_AHB1ENR_CCMDATARAMEN (1u << 20)

#define RCC_APB1ENR_USART2EN (1u << 17)
#define RCC_APB1ENR_USART3EN (1u << 18)
#define RCC_APB1ENR_UART4EN  (1u << 19)
#define RCC_APB1ENR_UART5EN  (1u << 20)
#define RCC_APB1ENR_TIM2EN   (1u << 0)
#define RCC_APB1ENR_TIM3EN   (1u << 1)
#define RCC_APB1ENR_TIM4EN   (1u << 2)
#define RCC_APB1ENR_PWREN    (1u << 28)
#define RCC_APB2ENR_USART1EN (1u << 4)
#define RCC_APB2ENR_USART6EN (1u << 5)
#define RCC_APB2ENR_ADC1EN   (1u << 8)

#define RCC_AHB1ENR_DMA1EN   (1u << 21)
#define RCC_AHB1ENR_DMA2EN   (1u << 22)

/* AHB2: the USB OTG FS controller is on this bus, bit 7 (RM0090 6.3.14). A
 * device that never enumerates is usually this clock, which costs nothing to
 * check and nothing to forget. */
#define RCC_AHB2ENR_OTGFSEN  (1u << 7)

/* --- FLASH interface (RM0090 3.9) ---
 *
 * Defined from a base address and an offset, rather than as four absolute
 * addresses, because this is the one peripheral whose *registers* the host
 * build has to be able to intercept: `flash.c` reaches them through two small
 * functions instead of through AK_REG32 directly, and those functions need the
 * offsets. See tests/host_flash_model.c - and the note in flash.c about what
 * the interception buys and what it deliberately does not model. */
#define FLASH_BASE 0x40023C00UL

#define FLASH_ACR            AK_REG32(FLASH_BASE + 0x00u)
#define FLASH_ACR_LATENCY(n) ((uint32_t)(n) & 0x7u)
#define FLASH_ACR_PRFTEN     (1u << 8)
#define FLASH_ACR_ICEN       (1u << 9)
#define FLASH_ACR_DCEN       (1u << 10)

#define FLASH_KEYR AK_REG32(FLASH_BASE + 0x04u)
#define FLASH_SR   AK_REG32(FLASH_BASE + 0x0Cu)
#define FLASH_CR   AK_REG32(FLASH_BASE + 0x10u)

/* And the offsets, for the two functions in flash.c that go through the seam,
 * and for the model on the other side of it. */
#define FLASH_OFF_ACR   0x00u
#define FLASH_OFF_KEYR  0x04u
#define FLASH_OFF_SR    0x0Cu
#define FLASH_OFF_CR    0x10u

#define FLASH_KEY1 0x45670123u
#define FLASH_KEY2 0xCDEF89ABu

#define FLASH_SR_EOP    (1u << 0)
#define FLASH_SR_OPERR  (1u << 1)
#define FLASH_SR_WRPERR (1u << 4)
#define FLASH_SR_PGAERR (1u << 5)
#define FLASH_SR_PGPERR (1u << 6)
#define FLASH_SR_PGSERR (1u << 7)
#define FLASH_SR_ERRORS (FLASH_SR_OPERR | FLASH_SR_WRPERR | FLASH_SR_PGAERR | \
                         FLASH_SR_PGPERR | FLASH_SR_PGSERR)
#define FLASH_SR_BSY    (1u << 16)

#define FLASH_CR_PG      (1u << 0)
#define FLASH_CR_SER     (1u << 1)
#define FLASH_CR_SNB(s)  (((uint32_t)(s) & 0xFu) << 3)
#define FLASH_CR_PSIZE_X32 (0x2u << 8)
#define FLASH_CR_STRT    (1u << 16)
#define FLASH_CR_LOCK    (1u << 31)

/* --- GPIO (RM0090 8.4) --- */
#define GPIOA_BASE 0x40020000UL
#define GPIOB_BASE 0x40020400UL
#define GPIOC_BASE 0x40020800UL
#define GPIOD_BASE 0x40020C00UL
#define GPIOE_BASE 0x40021000UL

#define GPIO_MODER(base)   AK_REG32((base) + 0x00)
#define GPIO_OTYPER(base)  AK_REG32((base) + 0x04)
#define GPIO_OSPEEDR(base) AK_REG32((base) + 0x08)
#define GPIO_PUPDR(base)   AK_REG32((base) + 0x0C)
#define GPIO_IDR(base)     AK_REG32((base) + 0x10)
#define GPIO_ODR(base)     AK_REG32((base) + 0x14)
#define GPIO_BSRR(base)    AK_REG32((base) + 0x18)
#define GPIO_AFRL(base)    AK_REG32((base) + 0x20)
#define GPIO_AFRH(base)    AK_REG32((base) + 0x24)

#define GPIO_MODE_INPUT  0u
#define GPIO_MODE_OUTPUT 1u
#define GPIO_MODE_AF     2u
#define GPIO_MODE_ANALOG 3u

#define GPIO_OTYPE_PUSH_PULL  0u
#define GPIO_OTYPE_OPEN_DRAIN 1u

#define GPIO_SPEED_LOW    0u
#define GPIO_SPEED_MEDIUM 1u
#define GPIO_SPEED_HIGH   2u
#define GPIO_SPEED_FAST   3u

#define GPIO_PUPD_NONE   0u
#define GPIO_PUPD_PULLUP 1u
#define GPIO_PUPD_PULLDN 2u

/* --- USART (RM0090 30.6) --- */
#define USART1_BASE 0x40011000UL
#define USART2_BASE 0x40004400UL
#define USART3_BASE 0x40004800UL
#define USART6_BASE 0x40011400UL

#define USART_SR(base)  AK_REG32((base) + 0x00)
#define USART_DR(base)  AK_REG32((base) + 0x04)
#define USART_BRR(base) AK_REG32((base) + 0x08)
#define USART_CR1(base) AK_REG32((base) + 0x0C)
#define USART_CR2(base) AK_REG32((base) + 0x10)

#define USART_SR_ORE  (1u << 3)
#define USART_SR_RXNE (1u << 5)
#define USART_SR_TC   (1u << 6)
#define USART_SR_TXE  (1u << 7)

#define USART_CR1_RE (1u << 2)
#define USART_CR1_TE (1u << 3)
#define USART_CR1_PS (1u << 9)   /* 0 = even parity, 1 = odd */
#define USART_CR1_PCE (1u << 10) /* parity control enable */
#define USART_CR1_RXNEIE (1u << 5)
#define USART_CR1_UE (1u << 13)

/* CR2's STOP[13:12]: 00 one stop bit, 10 two. */
#define USART_CR2_STOP_2 (0x2u << 12)

/* --- Cortex-M4 system control (PM0214) --- */
#define SYSTICK_BASE 0xE000E010UL
#define SYSTICK_CTRL AK_REG32(SYSTICK_BASE + 0x00)
#define SYSTICK_LOAD AK_REG32(SYSTICK_BASE + 0x04)
#define SYSTICK_VAL  AK_REG32(SYSTICK_BASE + 0x08)

#define SYSTICK_CTRL_ENABLE    (1u << 0)
#define SYSTICK_CTRL_TICKINT   (1u << 1)
#define SYSTICK_CTRL_CLKSOURCE (1u << 2)

#define SCB_VTOR  AK_REG32(0xE000ED08UL)
#define SCB_CPACR AK_REG32(0xE000ED88UL)
#define SCB_AIRCR AK_REG32(0xE000ED0CUL)

#define SCB_AIRCR_SYSRESETREQ 0x05FA0004u

/* --- TIM2-TIM5 general purpose timers (RM0090 18.4) --- */
#define TIM2_BASE 0x40000000UL
#define TIM3_BASE 0x40000400UL
#define TIM4_BASE 0x40000800UL

#define TIM_CR1(base)   AK_REG32((base) + 0x00)
#define TIM_DIER(base)  AK_REG32((base) + 0x0C)
#define TIM_SR(base)    AK_REG32((base) + 0x10)
#define TIM_EGR(base)   AK_REG32((base) + 0x14)
#define TIM_CCMR1(base) AK_REG32((base) + 0x18)
#define TIM_CCMR2(base) AK_REG32((base) + 0x1C)
#define TIM_CCER(base)  AK_REG32((base) + 0x20)
#define TIM_CNT(base)   AK_REG32((base) + 0x24)
#define TIM_PSC(base)   AK_REG32((base) + 0x28)
#define TIM_ARR(base)   AK_REG32((base) + 0x2C)
#define TIM_CCR1(base)  AK_REG32((base) + 0x34)
#define TIM_CCR2(base)  AK_REG32((base) + 0x38)
#define TIM_CCR3(base)  AK_REG32((base) + 0x3C)
#define TIM_CCR4(base)  AK_REG32((base) + 0x40)
#define TIM_DCR(base)   AK_REG32((base) + 0x48)
#define TIM_DMAR(base)  AK_REG32((base) + 0x4C)

#define TIM_CR1_CEN  (1u << 0)
#define TIM_CR1_ARPE (1u << 7)

#define TIM_DIER_UIE   (1u << 0)
#define TIM_DIER_CC1DE (1u << 9)

#define TIM_SR_UIF   (1u << 0)
#define TIM_SR_CC1IF (1u << 1)

#define TIM_EGR_UG (1u << 0)

/* CCMR1: OC1M[6:4], OC1PE[3], and the same pair again for channel 2 at
 * [14:12]/[11]. CCMR2 repeats that for channels 3 and 4. PWM mode 1 is 110.
 * The shifting is done in the driver rather than in a macro here, because a
 * macro that expands to another macro's name is not a register accessor. */
#define TIM_CCMR_OCM_PWM1 6u

/* CCER: CCxE at bit 4*(x-1). */
#define TIM_CCER_CCE(ch) (1u << (4u * ((ch) - 1u)))

#define TIM_DCR_DBA(dba) (((uint32_t)(dba) & 0x1Fu) << 0)
#define TIM_DCR_DBL(dbl) (((uint32_t)(dbl) & 0x1Fu) << 8)

/* --- DMA1/DMA2 on the F4 (RM0090 10.5) --- */
#define DMA1_BASE 0x40026000UL
#define DMA2_BASE 0x40026400UL

#define DMA_LISR(base)  AK_REG32((base) + 0x00)
#define DMA_HISR(base)  AK_REG32((base) + 0x04)
#define DMA_LIFCR(base) AK_REG32((base) + 0x08)
#define DMA_HIFCR(base) AK_REG32((base) + 0x0C)

/* Stream register blocks start at 0x10 and are 0x18 apart. */
#define DMA_SxCR(base, s)   AK_REG32((base) + 0x10u + 0x18u * (s) + 0x00u)
#define DMA_SxNDTR(base, s) AK_REG32((base) + 0x10u + 0x18u * (s) + 0x04u)
#define DMA_SxPAR(base, s)  AK_REG32((base) + 0x10u + 0x18u * (s) + 0x08u)
#define DMA_SxM0AR(base, s) AK_REG32((base) + 0x10u + 0x18u * (s) + 0x0Cu)
#define DMA_SxFCR(base, s)  AK_REG32((base) + 0x10u + 0x18u * (s) + 0x14u)

#define DMA_SxCR_EN     (1u << 0)
#define DMA_SxCR_TCIE   (1u << 4)
#define DMA_SxCR_DIR_M2P (0x1u << 6)
#define DMA_SxCR_PINC   (1u << 9)
#define DMA_SxCR_MINC   (1u << 10)
#define DMA_SxCR_PSIZE_16 (0x1u << 11)
#define DMA_SxCR_MSIZE_16 (0x1u << 13)
#define DMA_SxCR_PL_HIGH  (0x2u << 16)
#define DMA_SxCR_CHSEL(c) (((uint32_t)(c) & 0x7u) << 25)

/* LISR/HISR flags per stream: the pattern repeats every 6 bits, streams 0-3 in
 * LISR and 4-7 in HISR, with stream 4 at bit 0 of HISR. */
#define DMA_FLAG_TC(stream)   (1u << (((stream) & 0x3u) * 6u + 5u))
#define DMA_IFCR_CLEAR(stream) (0x3Du << (((stream) & 0x3u) * 6u))

#define DMA1_STREAM4_IRQ 15u
#define USART1_IRQ 37u
#define USART3_IRQ 39u
#define NVIC_ISER0 AK_REG32(0xE000E100UL)
#define NVIC_ISER1 AK_REG32(0xE000E104UL)

/* TIM3_CH1 -> DMA1 Stream 4, channel 5 (INAV's timer_def_stm32f4xx.h, which is
 * generated from ST's data: DEF_TIM_DMA__BTCH_TIM3_CH1 D(1, 4, 5)). */
#define AK_DMA_TIM3_STREAM 4u
#define AK_DMA_TIM3_CHANNEL 5u

/* --- SPI1-SPI3 (RM0090 28.5). Only the master, mode 3, 8-bit case is
 * defined: that is what the sensor buses use, and a register map with unused
 * corners in it is a register map with unused mistakes in it. --- */
#define SPI1_BASE 0x40013000UL
#define SPI2_BASE 0x40003800UL
#define SPI3_BASE 0x40003C00UL

#define SPI_CR1(base) AK_REG32((base) + 0x00)
#define SPI_CR2(base) AK_REG32((base) + 0x04)
#define SPI_SR(base)  AK_REG32((base) + 0x08)
#define SPI_DR(base)  AK_REG32((base) + 0x0C)

#define SPI_CR1_CPHA  (1u << 0)
#define SPI_CR1_CPOL  (1u << 1)
#define SPI_CR1_MSTR  (1u << 2)
#define SPI_CR1_SPE   (1u << 6)
#define SPI_CR1_SSI   (1u << 8)
#define SPI_CR1_SSM   (1u << 9)

#define SPI_SR_RXNE (1u << 0)
#define SPI_SR_TXE  (1u << 1)
#define SPI_SR_BSY  (1u << 7)

#define SPI_CR1_BR_DIV8 (0x2u << 3)

/* --- ADC1-ADC3 and the common block (RM0090 11.12) ---
 *
 * Only the regular, single-conversion path is defined: this firmware measures
 * one slow thing - a battery through a divider - and the parts of the ADC that
 * matter for that are the enable, the software start, and the end-of-conversion
 * flag. An injected sequence, a scan of several channels and a DMA request are
 * all real features of the part and none of them are used here, so none of them
 * are written down.
 *
 * The three ADCs share the common block, and that is where the prescaler
 * lives. It is easy to miss, and its symptom is a conversion that never
 * completes: ADC1 has no clock of its own, and the divider that feeds it is
 * set once, for all three, at an address outside the register block. */
#define ADC1_BASE 0x40012000UL
#define ADC2_BASE 0x40012100UL
#define ADC3_BASE 0x40012200UL
#define ADC_COMMON_BASE 0x40012300UL

#define ADC_SR(base)    AK_REG32((base) + 0x00)
#define ADC_CR1(base)   AK_REG32((base) + 0x04)
#define ADC_CR2(base)   AK_REG32((base) + 0x08)
#define ADC_SMPR1(base) AK_REG32((base) + 0x0C)
#define ADC_SMPR2(base) AK_REG32((base) + 0x10)
#define ADC_SQR1(base)  AK_REG32((base) + 0x2C)
#define ADC_SQR3(base)  AK_REG32((base) + 0x34)
#define ADC_DR(base)    AK_REG32((base) + 0x4C)

#define ADC_CCR AK_REG32(ADC_COMMON_BASE + 0x04)

#define ADC_SR_EOC (1u << 1)

#define ADC_CR1_RES_12BIT (0x0u << 24)
#define ADC_CR2_ADON    (1u << 0)
#define ADC_CR2_CONT    (1u << 1)
#define ADC_CR2_EOCS    (1u << 10) /* end of *conversion*, not of sequence */
#define ADC_CR2_SWSTART (1u << 30)

/* SQR1's L[3:0] is the sequence length *minus one*, so one conversion is 0. */
#define ADC_SQR1_L(n) ((((uint32_t)(n) - 1u) & 0xFu) << 20)
/* SQR3 holds the first six channels of the regular sequence, five bits each. */
#define ADC_SQR3_SQ1(ch) ((uint32_t)(ch) & 0x1Fu)

/* Sample time, three bits per channel: SMPR2 for channels 0-9, SMPR1 for
 * 10-18. 480 cycles is the longest the part has and it is chosen on purpose -
 * a 10k/1k divider presents about a kilo-ohm to a pin whose sample capacitor
 * wants a much stiffer source than that at the short sample times. The cost is
 * 23 microseconds a reading at a 21 MHz ADC clock, taken ten times a second. */
#define ADC_SMPR_480CYCLES 0x7u
#define ADC_SMPR1_SMP(ch) \
    ((uint32_t)ADC_SMPR_480CYCLES << (3u * ((ch) - 10u)))
#define ADC_SMPR2_SMP(ch) ((uint32_t)ADC_SMPR_480CYCLES << (3u * (ch)))

/* CCR's ADCPRE is two bits and divides PCLK2 for all three ADCs:
 * 0 = /2, 1 = /4, 2 = /6, 3 = /8. This board runs APB2 at 84 MHz, so /4 is
 * 21 MHz - inside the part's 36 MHz limit, with room for the clock to be wrong
 * by a factor nobody has noticed yet. */
#define ADC_CCR_ADCPRE_DIV4 (0x1u << 16)

/* --- I2C1-I2C3 (RM0090 27.6) ---
 *
 * The F4's I2C is the older peripheral, not the newer "TIMINGR" one: it wants
 * a frequency range, a clock control register and a rise-time register, all
 * three computed from the APB1 clock. Every barometer the wing's own board can
 * carry is on I2C, and so are most of the compasses and the external sensors
 * anybody bolts to one, which is why this exists before a part has been picked.
 */
#define I2C1_BASE 0x40005400UL
#define I2C2_BASE 0x40005800UL
#define I2C3_BASE 0x40005C00UL

#define I2C_CR1(base)  AK_REG32((base) + 0x00)
#define I2C_CR2(base)  AK_REG32((base) + 0x04)
#define I2C_DR(base)   AK_REG32((base) + 0x10)
#define I2C_SR1(base)  AK_REG32((base) + 0x14)
#define I2C_SR2(base)  AK_REG32((base) + 0x18)
#define I2C_CCR(base)  AK_REG32((base) + 0x1C)
#define I2C_TRISE(base) AK_REG32((base) + 0x20)

/* The same seven registers by offset, for the one place that has to name them
 * that way: the seam in i2c.c, which a host build points at a modelled bus so
 * that the transaction sequences can run without a part. RM0090 27.6. */
#define I2C_OFF_CR1   0x00u
#define I2C_OFF_CR2   0x04u
#define I2C_OFF_DR    0x10u
#define I2C_OFF_SR1   0x14u
#define I2C_OFF_SR2   0x18u
#define I2C_OFF_CCR   0x1Cu
#define I2C_OFF_TRISE 0x20u

#define I2C_CR1_PE     (1u << 0)
#define I2C_CR1_START  (1u << 8)
#define I2C_CR1_STOP   (1u << 9)
#define I2C_CR1_ACK    (1u << 10)
/* POS moves where the NACK goes, and it is the whole of the two-byte read
 * problem: with POS = 0 the acknowledge for a two-byte read arrives after the
 * first byte, and the master ends up waiting for a third. RM0090 27.6.1. */
#define I2C_CR1_POS    (1u << 11)
#define I2C_CR1_SWRST  (1u << 15)

/* CR2 FREQ[5:0] is the APB1 clock in MHz - the peripheral divides its own
 * timing from that number, so a wrong one is not a slower bus, it is a bus
 * whose timings are all wrong by the same factor. */
#define I2C_CR2_FREQ_MASK 0x3Fu

#define I2C_SR1_SB    (1u << 0)
#define I2C_SR1_ADDR  (1u << 1)
#define I2C_SR1_BTF   (1u << 2)
#define I2C_SR1_RXNE  (1u << 6)
#define I2C_SR1_TXE   (1u << 7)
#define I2C_SR1_AF    (1u << 10)

/* CCR: the period in APB1 clocks, DUTY for the fast-mode split, FS for which
 * mode it is describing. */
#define I2C_CCR_FS   (1u << 15)
#define I2C_CCR_DUTY (1u << 14)

#define RCC_APB1ENR_I2C1EN (1u << 21)
#define RCC_APB1ENR_I2C2EN (1u << 22)
#define RCC_APB1ENR_I2C3EN (1u << 23)

/* --- USB OTG FS (RM0090 33.12) ---
 *
 * The full-speed device controller: how this board gets a console without a
 * USB-TTL adapter on PA2/PA3, and the goal's M0 asks for a banner over USB
 * specifically. The set below is what enumeration needs - the global block,
 * the device block, endpoint 0 for control transfers, and one IN endpoint for
 * the console's bytes.
 *
 * Three blocks, not one: the global registers at the base, the device
 * registers 2 KB above it, and the power/clock gate another 1.5 KB above that.
 * The FIFOs are a fourth thing again - 4 KB windows from 0x1000, each one an
 * address a byte is pushed to or popped from. FIFO 0 is shared by every OUT
 * endpoint and by endpoint 0's IN data, which is why the IN endpoint's own
 * buffer is placed above the receive FIFO in GRXFSIZ.
 */
#define OTG_FS_BASE        0x50000000UL
#define OTG_FS_DEVICE_BASE (OTG_FS_BASE + 0x800u)
#define OTG_FS_PCGCCTL     AK_REG32(OTG_FS_BASE + 0xE00u)
/* PCGCCTL bit 0, the phy clock gate: set means the phy clock is stopped. The
 * sibling AT32 port defines this same bit under this same name
 * (`at32f435/regs.h`), and this header declared the register without it. */
#define OTG_PCGCCTL_STOPPCLK (1u << 0)

/* Global (RM0090 33.12.1-33.12.13). */
#define OTG_GOTGCTL   AK_REG32(OTG_FS_BASE + 0x000u)
#define OTG_GOTGINT   AK_REG32(OTG_FS_BASE + 0x004u)
#define OTG_GAHBCFG   AK_REG32(OTG_FS_BASE + 0x008u)
#define OTG_GUSBCFG   AK_REG32(OTG_FS_BASE + 0x00Cu)
#define OTG_GRSTCTL   AK_REG32(OTG_FS_BASE + 0x010u)
#define OTG_GINTSTS   AK_REG32(OTG_FS_BASE + 0x014u)
#define OTG_GINTMSK   AK_REG32(OTG_FS_BASE + 0x018u)
/* The receive status queue. Every read pops one entry - which endpoint, how
 * many bytes, and what kind of packet - and the bytes it describes are then
 * read out of FIFO 0. Reading this word out of the FIFO instead is a device
 * that eats the first byte of every payload. RM0090 33.12.6. */
#define OTG_GRXSTSP   AK_REG32(OTG_FS_BASE + 0x020u)
#define OTG_GRXFSIZ   AK_REG32(OTG_FS_BASE + 0x024u)
#define OTG_GNPTXFSIZ AK_REG32(OTG_FS_BASE + 0x028u)
/* The core's own power switch and its two VBUS-sensing controls. This part's
 * Synopsys core comes out of reset *powered down* and *watching for VBUS*, and
 * a device that is either of those things puts nothing on the bus for a host
 * to find - see the two bits and the note in usb.c. Offsets from RM0090's
 * OTG_FS register map (0x38 clears the power-down bit, which is what ST's own
 * driver calls "deactivate the power down"). */
#define OTG_GCCFG           AK_REG32(OTG_FS_BASE + 0x038u)
#define OTG_GCCFG_PWRDWN    (1u << 16)
#define OTG_GCCFG_NOVBUSSENS (1u << 21)
/* One per IN endpoint above zero, 4 bytes each from 0x104: endpoint 1's
 * transmit FIFO size and where it starts.
 *
 * Endpoint 0 is not the zeroth of these. It has its own register at 0x028 -
 * the one this header already names `OTG_GNPTXFSIZ`, and that ST's names
 * `DIEPTXF0_HNPTXFSIZ`, "EP0 / Non Periodic Tx FIFO Size Register" - while
 * 0x100, where an array indexed from zero would land, is `HPTXFSIZ`, the
 * **host** periodic FIFO. ST's device branch writes the two separately:
 *
 *   USB_OTG_WRITE_REG32(&pdev->regs.GREGS->DIEPTXF0_HNPTXFSIZ, ...);   <- EP0
 *   USB_OTG_WRITE_REG32(&pdev->regs.GREGS->DIEPTXF[0],           ...);   <- EP1
 *
 * (`upstream/betaflight-2026.6.1/lib/main/STM32_USB_OTG_Driver/src/usb_core.c`,
 * `USB_OTG_CoreInitDev`; the layout is in that tree's `inc/usb_regs.h`.)
 *
 * **This macro was `0x100 + 4n`.** That got endpoint 1 and endpoint 2 right
 * by accident - 0x104 and 0x108 - and sent endpoint 0 into the host register.
 * So endpoint 0's FIFO was configured by nothing and sat at its reset value of
 * zero: no start address and no depth. Endpoint 0 is the one every host uses
 * first, to read the device descriptor, and that read is the one that fails on
 * every boot. The comment above said "above zero" all along; the arithmetic
 * below said otherwise. */
#define OTG_DIEPTXF(n) \
    AK_REG32(OTG_FS_BASE + ((n) == 0u ? 0x028u : 0x104u + 4u * ((n) - 1u)))

/* Device (RM0090 33.12.16-33.12.28). */
#define OTG_DCFG       AK_REG32(OTG_FS_DEVICE_BASE + 0x000u)
#define OTG_DCTL       AK_REG32(OTG_FS_DEVICE_BASE + 0x004u)
#define OTG_DSTS       AK_REG32(OTG_FS_DEVICE_BASE + 0x008u)
#define OTG_DIEPMSK    AK_REG32(OTG_FS_DEVICE_BASE + 0x010u)
#define OTG_DOEPMSK    AK_REG32(OTG_FS_DEVICE_BASE + 0x014u)
#define OTG_DAINT      AK_REG32(OTG_FS_DEVICE_BASE + 0x018u)
#define OTG_DAINTMSK   AK_REG32(OTG_FS_DEVICE_BASE + 0x01Cu)
#define OTG_DIEPEMPMSK AK_REG32(OTG_FS_DEVICE_BASE + 0x034u)

/* Endpoint 0: control, both directions, which is the whole of enumeration. */
#define OTG_DIEPCTL0  AK_REG32(OTG_FS_DEVICE_BASE + 0x100u)
#define OTG_DIEPINT0  AK_REG32(OTG_FS_DEVICE_BASE + 0x108u)
#define OTG_DIEPTSIZ0 AK_REG32(OTG_FS_DEVICE_BASE + 0x110u)
#define OTG_DOEPCTL0  AK_REG32(OTG_FS_DEVICE_BASE + 0x300u)
#define OTG_DOEPINT0  AK_REG32(OTG_FS_DEVICE_BASE + 0x308u)
#define OTG_DOEPTSIZ0 AK_REG32(OTG_FS_DEVICE_BASE + 0x310u)

/* Every other endpoint: 0x20 bytes apart, and the OUT half 0x200 past the IN. */
#define OTG_DIEPCTL(n)  AK_REG32(OTG_FS_DEVICE_BASE + 0x100u + 0x20u * (n))
#define OTG_DIEPINT(n)  AK_REG32(OTG_FS_DEVICE_BASE + 0x108u + 0x20u * (n))
#define OTG_DIEPTSIZ(n) AK_REG32(OTG_FS_DEVICE_BASE + 0x110u + 0x20u * (n))
#define OTG_DOEPCTL(n)  AK_REG32(OTG_FS_DEVICE_BASE + 0x300u + 0x20u * (n))
#define OTG_DOEPINT(n)  AK_REG32(OTG_FS_DEVICE_BASE + 0x308u + 0x20u * (n))
#define OTG_DOEPTSIZ(n) AK_REG32(OTG_FS_DEVICE_BASE + 0x310u + 0x20u * (n))

/* The FIFO windows: write a byte to push, read one to pop. */
#define OTG_FIFO(n) AK_REG32(OTG_FS_BASE + 0x1000u + 0x1000u * (n))

/* GUSBCFG: PHYSEL selects the full-speed serial transceiver, the only one this
 * part has - without it nothing enumerates at all. */
#define OTG_GUSBCFG_PHYSEL (1u << 6)
/* TRDTIM[13:10]: the turn-around time, in phy clocks. The reference's table
 * (`usb_dcd_int.c`, `DCD_HandleEnumDone_ISR`) gives 5 for every AHB from
 * 34.3 MHz up to 168 - which is this part's whole range - so there is one value
 * here rather than a table. */
#define OTG_GUSBCFG_TRDTIM(v) (((uint32_t)(v) & 0xFu) << 10)
#define OTG_GUSBCFG_TRDTIM_48MHZ 5u
#define OTG_GUSBCFG_FDMOD  (1u << 30) /* force device mode */

#define OTG_GAHBCFG_GINTMSK  (1u << 0)
#define OTG_GAHBCFG_TXFELVL  (1u << 7)
#define OTG_GAHBCFG_PTXFELVL (1u << 8)

/* GRSTCTL: reset the core and flush a FIFO. AHBIDLE says the core has stopped
 * touching the AHB, which is what has to be true before CSRST. */
#define OTG_GRSTCTL_CSRST   (1u << 0)
#define OTG_GRSTCTL_RXFFLSH (1u << 4)
#define OTG_GRSTCTL_TXFFLSH (1u << 5)
#define OTG_GRSTCTL_TXFNUM(f) (((uint32_t)(f) & 0x1Fu) << 6)
#define OTG_GRSTCTL_AHBIDLE (1u << 31)

/* GINTSTS/GINTMSK, the ones a device cares about. */
#define OTG_GINT_RXFLVL  (1u << 4)  /* a packet is in the receive FIFO */
#define OTG_GINT_USBRST  (1u << 12) /* the host reset the bus */
#define OTG_GINT_ENUMDNE (1u << 13) /* enumeration done: the speed is known */
#define OTG_GINT_IEPINT  (1u << 18) /* an IN endpoint finished something */
#define OTG_GINT_OEPINT  (1u << 19) /* an OUT endpoint did */
#define OTG_GINT_WKUPINT (1u << 31)

/* One receive status entry: the endpoint in the low four bits, the byte count
 * above it, then the data PID, then what kind of packet it was. A data packet
 * has its bytes behind it in the FIFO; a setup packet is always eight bytes;
 * the rest are the ends of transfers and have nothing to read. */
#define OTG_GRXSTS_EPNUM  0xFu
#define OTG_GRXSTS_BCNT   (0x7FFu << 4)
#define OTG_GRXSTS_PKTSTS (0xFu << 17)
#define OTG_GRXSTS_DATA   2u /* an OUT data packet */
#define OTG_GRXSTS_SETUP_DONE 4u /* the setup stage is over: STUP follows */
#define OTG_GRXSTS_SETUP  6u /* a setup packet */

/* DCFG: the device's address and the speed it answers at. */
#define OTG_DCFG_DSPD_FS (0x3u << 0) /* full speed */
#define OTG_DCFG_DAD(a)  (((uint32_t)(a) & 0x7Fu) << 4)

/* DCTL: SDIS is the soft disconnect a device uses to make the host
 * re-enumerate it - what to do once the descriptors are in place.
 *
 * The two that followed it were `OTG_DCTL_CGINAK` on bit 2 and
 * `OTG_DCTL_CGONAK` on bit 3, named as the clear-the-global-NAK controls. Bits
 * 2 and 3 are `GNPINNAKSTS` and `GOUTNAKSTS` - **read-only status**, not
 * controls - and the controls are bit 8 and bit 10. Both readings come from the
 * vendor's own register description rather than from a datasheet skim: ST's
 * `USB_OTG_DCTL_TypeDef` bitfield (`usb_regs.h`, which packs from bit 0) puts
 * `gnpinnaksts` at 2, `goutnaksts` at 3, `sgnpinnak` at 7 and `cgnpinnak` at 8;
 * the AT32 SVD's USB_OTG device registers give the same offsets field for
 * field. Nothing used the old pair, which is the only reason a wrong name on a
 * wrong bit never became a wrong write. */
#define OTG_DCTL_RWUSIG      (1u << 0)
#define OTG_DCTL_SDIS        (1u << 1)
#define OTG_DCTL_GNPINNAKSTS (1u << 2) /* read-only: a global IN NAK is in force */
#define OTG_DCTL_GOUTNAKSTS  (1u << 3) /* read-only */
/* Write 1 to release every non-periodic IN endpoint. A USB reset sets the
 * global IN NAK - endpoint 0 with the rest - so this is what lets the device
 * answer the host at all. */
#define OTG_DCTL_CGNPINNAK   (1u << 8)

/* DSTS: what the core ended up at, and whether the bus is suspended. */
#define OTG_DSTS_SUSPSTS      (1u << 0)
#define OTG_DSTS_ENUMSPD_MASK (0x3u << 1)

/* DIEPCTL/DOEPCTL share these bit positions in both directions. */
#define OTG_DEPCTL_MPS(v)     ((uint32_t)(v) & 0x7FFu)
#define OTG_DEPCTL_USBAEP     (1u << 15) /* this endpoint exists */
#define OTG_DEPCTL_EPTYP_CTRL (0u << 18)
#define OTG_DEPCTL_EPTYP_BULK (2u << 18)
#define OTG_DEPCTL_STALL      (1u << 21)
#define OTG_DEPCTL_TXFNUM(n)  (((uint32_t)(n) & 0xFu) << 22)
#define OTG_DEPCTL_CNAK       (1u << 26) /* clear NAK: ready to answer */
#define OTG_DEPCTL_SNAK       (1u << 27)
#define OTG_DEPCTL_EPDIS      (1u << 30)
#define OTG_DEPCTL_EPENA      (1u << 31)

/* DIEPINT/DOEPINT: transfer complete, and - endpoint 0 only - a setup packet. */
#define OTG_DEPINT_XFRC   (1u << 0)
#define OTG_DEPINT_EPDISD (1u << 1)
#define OTG_DEPINT_STUP   (1u << 3)

/* Endpoint 0's transfer size, and how many setup packets have arrived. */
#define OTG_DIEPTSIZ0_XFRSIZ(v) ((uint32_t)(v) & 0x7Fu)
#define OTG_DIEPTSIZ0_PKTCNT1   (1u << 19)
#define OTG_DOEPTSIZ0_XFRSIZ(v) ((uint32_t)(v) & 0x7Fu)
#define OTG_DOEPTSIZ0_PKTCNT1   (1u << 19)
#define OTG_DOEPTSIZ0_STUPCNT(s) (((uint32_t)(s) & 0x3u) << 29)

/* The same arithmetic on an endpoint that is not zero, where the size is
 * nineteen bits rather than seven. A bulk OUT endpoint is armed one packet at
 * a time: write both fields, then EPENA and CNAK in its control word. RM0090
 * 33.12.45. */
#define OTG_DOEPTSIZ_XFRSIZ(v) ((uint32_t)(v) & 0x7FFFFu)
#define OTG_DOEPTSIZ_PKTCNT(n) (((uint32_t)(n) & 0x3FFu) << 19)

#endif /* AK_ARCH_STM32F405_REGS_H */
