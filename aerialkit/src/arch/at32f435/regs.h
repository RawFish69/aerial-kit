#ifndef AK_ARCH_AT32F435_REGS_H
#define AK_ARCH_AT32F435_REGS_H

#include <stdint.h>

/*
 * Hand-written AT32F435 register definitions - the subset AerialKit uses.
 *
 * Addresses and bit positions come from Artery's own CMSIS device header and
 * standard-peripheral library, read in this workspace's pinned INAV checkout
 * (`upstream/inav-9.1.0` at `e519b69`):
 *
 *   lib/main/AT32F43x/Drivers/CMSIS/Device/ST/AT32F43x/at32f435_437.h
 *       - the memory map (`PERIPH_BASE` and the APB1/APB2/AHB1 windows) and the
 *         base address of each peripheral;
 *   lib/main/AT32F43x/Drivers/AT32F43x_StdPeriph_Driver/inc/
 *       - the bit layouts: `at32f435_437_crm.h` for the clock unit,
 *         `at32f435_437_flash.h` for the controller's divider, `_pwc.h` for
 *         the regulator's voltage.
 *
 * Artery's header says its contents are Artery's copyrighted work, so nothing
 * here is copied from it: every line is a statement about the part that the
 * datasheet or the reference manual also makes, written in this project's own
 * shape and named this project's way where the reference's names would be
 * misleading. 03-attribution.md records that these files were read.
 *
 * **The address map is close enough to the STM32F4's to be a trap.** Both parts
 * put their clock unit at 0x40023800 and their GPIO ports at 0x40020000. What
 * is not the same is what the registers *mean* - the clock has a different PLL
 * (ms/ns/fr rather than M/N/P/Q), the pins are configured by a source and mux
 * scheme rather than MODER/AFR, and the flash controller has 2 KB sectors - so
 * this file is its own, and the F405's is not included by anything here.
 *
 * Only what is used is defined. New peripherals get added with the manual
 * section or the reference file that justifies them, as this one does.
 */

#define AK_REG32(addr) (*(volatile uint32_t *)(uintptr_t)(addr))

/* --- memory map (Artery at32f435_437.h: PERIPH_BASE and the three windows) --- */
#define AK_FLASH_BASE  0x08000000UL
#define AK_PERIPH_BASE 0x40000000UL
#define AK_APB1_BASE   (AK_PERIPH_BASE)
#define AK_APB2_BASE   (AK_PERIPH_BASE + 0x10000UL)
#define AK_AHB1_BASE   (AK_PERIPH_BASE + 0x20000UL)

/* --- CRM: the clock unit (at32f435_437_crm.h, the crm_type struct) ----------
 *
 * ctrl   +0x00  enables and their ready flags
 * pllcfg +0x04  the multiplier, the input divider, the output divider, source
 * cfg    +0x08  the system-clock selector and the bus dividers
 */
#define CRM_BASE   (AK_AHB1_BASE + 0x3800UL)
#define CRM_CTRL   AK_REG32(CRM_BASE + 0x00UL)
#define CRM_PLLCFG AK_REG32(CRM_BASE + 0x04UL)
#define CRM_CFG    AK_REG32(CRM_BASE + 0x08UL)

#define CRM_CTRL_HICKEN   (1u << 0)  /* the internal 8 MHz clock */
#define CRM_CTRL_HICKSTBL (1u << 1)
#define CRM_CTRL_HEXTEN   (1u << 16) /* the crystal */
#define CRM_CTRL_HEXTSTBL (1u << 17)
#define CRM_CTRL_PLLEN    (1u << 24)
#define CRM_CTRL_PLLSTBL  (1u << 25)

/* pllcfg: the PLL is (source / ms) * ns / fr, with fr a power of two written as
 * its exponent - 1 means divide by two. This is not the F4's M/N/P/Q scheme,
 * which is the first thing a port gets wrong. */
#define CRM_PLLCFG_MS_SHIFT  0
#define CRM_PLLCFG_MS_MASK   0xFu
#define CRM_PLLCFG_NS_SHIFT  6
#define CRM_PLLCFG_NS_MASK   0x1FFu
#define CRM_PLLCFG_FR_SHIFT  16
#define CRM_PLLCFG_FR_MASK   0x7u
#define CRM_PLLCFG_RCS       (1u << 22) /* 0 = internal, 1 = crystal */
#define CRM_PLL_SOURCE_HICK  0u
#define CRM_PLL_SOURCE_HEXT  1u
#define CRM_PLL_FR_1         0u /* divide by 1 */
#define CRM_PLL_FR_2         1u /* divide by 2 */
#define CRM_PLL_FR_4         2u
#define CRM_PLL_FR_8         3u
#define CRM_PLL_FR_16        4u

/* cfg: the switch, the switch's status (read-only on the part, and what the
 * code has to wait for), and the three bus dividers. */
#define CRM_CFG_SCLKSEL_MASK 0x3u
#define CRM_CFG_SCLKSTS_MASK 0xCu
#define CRM_CFG_SCLKSTS_SHIFT 2
#define CRM_CFG_SCLKSEL_HICK 0u
#define CRM_CFG_SCLKSEL_HEXT 1u
#define CRM_CFG_SCLKSEL_PLL  2u
#define CRM_CFG_AHBDIV_SHIFT 4
#define CRM_CFG_AHBDIV_MASK  0xFu
#define CRM_CFG_APB1DIV_SHIFT 10
#define CRM_CFG_APB1DIV_MASK  0x7u
#define CRM_CFG_APB2DIV_SHIFT 13
#define CRM_CFG_APB2DIV_MASK  0x7u
/* The dividers are encoded, not written as numbers: divide by one is 0 and
 * divide by two is 4 on this part (CRM_AHB_DIV_1, CRM_APB1_DIV_2). */
#define CRM_DIV_1 0x0u
#define CRM_DIV_2 0x4u

/* The peripheral clock enables. Artery's encoding for these is a register
 * offset and a bit inside it (`MAKE_VALUE(0x30, 0)` for GPIOA, `0x44, 4` for
 * USART1), so both halves are written out here rather than hidden behind a
 * macro that would have to be decoded to be checked. */
#define CRM_AHBEN1 AK_REG32(CRM_BASE + 0x30UL)
#define CRM_AHBEN2 AK_REG32(CRM_BASE + 0x34UL)
#define CRM_APB1EN AK_REG32(CRM_BASE + 0x40UL)
#define CRM_APB2EN AK_REG32(CRM_BASE + 0x44UL)

/* And two registers far above those, which are about the USB clock and nothing
 * else:
 *
 *   misc1 +0xA0  hick_to_usb[13]: 1 takes the USB clock from the internal
 *                clock, 0 from the PLL - and the PLL is what this port runs,
 *                so the bit is cleared
 *   misc2 +0xA4  usbdiv[15:12]: what the USB clock is divided by on its way
 *                out. The divider is fractional and encoded (CRM_USB_DIV_6 is
 *                0x0B), which is why the port does not do the arithmetic
 *                itself: at 288 MHz the answer is the reference's own encoding,
 *                288 / 6 = 48 MHz, and a wrong one is a device that enumerates
 *                on nothing or not at all.
 */
#define CRM_MISC1 AK_REG32(CRM_BASE + 0xA0UL)
#define CRM_MISC2 AK_REG32(CRM_BASE + 0xA4UL)
#define CRM_MISC1_HICK_TO_USB (1u << 13)
#define CRM_MISC2_USBDIV_SHIFT 12
#define CRM_MISC2_USBDIV_MASK  0xFu
#define CRM_USBDIV_6 0xBu /* 288 MHz / 6 = 48 MHz, the only rate a full-speed
                           * device works at */

/* --- GPIO (at32f435_437_gpio.h, the gpio_type struct) ----------------------
 *
 * The configuration scheme is this part's own, and it is the one an STM32 port
 * will get wrong by pattern-matching: there is no MODER and no AFR. Each pin
 * has two bits of *mode* in cfgr, its output type and pull strength in their
 * own per-pin bitfields, and - only in mux mode - a four-bit function number
 * in muxl or muxh.
 *
 *   cfgr  +0x00  2 bits per pin: 0 input, 1 output, 2 mux, 3 analog
 *   omode +0x04  1 bit per pin: 0 push-pull, 1 open-drain
 *   odrvr +0x08  2 bits per pin: 1 stronger, 2 moderate
 *   pull  +0x0C  2 bits per pin: 0 none, 1 up, 2 down
 *   idt   +0x10  input data, one bit per pin
 *   odt   +0x14  output data, one bit per pin
 *   scr   +0x18  write a 1 to set that pin's output
 *   clr   +0x28  write a 1 to clear it
 *   muxl  +0x20  pins 0-7, four bits each: the mux function number
 *   muxh  +0x24  pins 8-15
 */
#define GPIOA_BASE (AK_AHB1_BASE + 0x0000UL)
#define GPIOB_BASE (AK_AHB1_BASE + 0x0400UL)
#define GPIOC_BASE (AK_AHB1_BASE + 0x0800UL)
#define GPIOD_BASE (AK_AHB1_BASE + 0x0C00UL)
#define GPIOE_BASE (AK_AHB1_BASE + 0x1000UL)
#define GPIOF_BASE (AK_AHB1_BASE + 0x1400UL)
#define GPIOG_BASE (AK_AHB1_BASE + 0x1800UL)
#define GPIOH_BASE (AK_AHB1_BASE + 0x1C00UL)
#define AK_GPIO_PORT_STEP 0x400u

#define AK_GPIO_CFGR(port)  AK_REG32((port) + 0x00UL)
#define AK_GPIO_OMODE(port) AK_REG32((port) + 0x04UL)
#define AK_GPIO_ODRVR(port) AK_REG32((port) + 0x08UL)
#define AK_GPIO_PULL(port)  AK_REG32((port) + 0x0CUL)
#define AK_GPIO_IDT(port)   AK_REG32((port) + 0x10UL)
#define AK_GPIO_ODT(port)   AK_REG32((port) + 0x14UL)
#define AK_GPIO_SCR(port)   AK_REG32((port) + 0x18UL)
#define AK_GPIO_MUXL(port)  AK_REG32((port) + 0x20UL)
#define AK_GPIO_MUXH(port)  AK_REG32((port) + 0x24UL)
#define AK_GPIO_CLR(port)   AK_REG32((port) + 0x28UL)

#define AK_GPIO_MODE_INPUT  0u
#define AK_GPIO_MODE_OUTPUT 1u
#define AK_GPIO_MODE_MUX    2u
#define AK_GPIO_MODE_ANALOG 3u

#define AK_GPIO_PULL_NONE 0u
#define AK_GPIO_PULL_UP   1u
#define AK_GPIO_PULL_DOWN 2u

#define AK_GPIO_DRIVE_MODERATE 2u
#define AK_GPIO_DRIVE_STRONGER 1u

/* The drive strength a board asks for, which is the third argument of
 * ak_pin_output() on both ports - the F405's names, because a board file reads
 * the same either way, and this part's *numbers*, because the field is a
 * different one. This part has four settings and no use here for the top two,
 * so the port maps zero to the moderate driver and anything else to the
 * stronger one (gpio.c), and the middle name is the same value as the top: two
 * names for one setting is honest, three settings where there are two is not.
 *
 * One thing that is *not* the same as the F405's: there, OSPEEDR also sets the
 * slew rate, so "fast" is about edges as well as current. Here it is current
 * only, and a servo or DShot pin wants the stronger driver for the same reason
 * - it is driving a wire, not a light. */
#define GPIO_SPEED_LOW  0u
#define GPIO_SPEED_MEDIUM 1u
#define GPIO_SPEED_HIGH 1u
#define GPIO_SPEED_FAST 1u

/* --- the core's own (Cortex-M4, as on the F405 port; PM0214 section 4) ----- */
#define AK_SYSTICK_BASE 0xE000E010UL
#define SYSTICK_CTRL    AK_REG32(AK_SYSTICK_BASE + 0x00UL)
#define SYSTICK_LOAD    AK_REG32(AK_SYSTICK_BASE + 0x04UL)
#define SYSTICK_VAL     AK_REG32(AK_SYSTICK_BASE + 0x08UL)
#define AK_SYSTICK_CTRL_ENABLE    (1u << 0)
#define AK_SYSTICK_CTRL_TICKINT   (1u << 1)
#define AK_SYSTICK_CTRL_CLKSOURCE (1u << 2) /* the processor clock, not the /8 */

/* --- USART (at32f435_437_usart.h, the usart_type struct) -------------------
 *
 * The register order and the bit positions are close to the STM32F4's - this
 * part is an F4-alike and its UART is too - which is exactly why the *layout*
 * is written out here rather than assumed. The two differences that matter to
 * this firmware: the divisor lives in one 16-bit field named `div` rather than
 * a BRR made of a mantissa and a fraction, and there is a TX/RX **swap** bit
 * in ctrl2 that the F405's USART does not have at all (the wing's GPS port
 * needs it, and it is the reason a pin map cannot simply be copied).
 *
 *   sts   +0x00  flags: rdbf (a byte is waiting), tdbe (room to send),
 *                tdc (the last byte has left the shift register), and the
 *                error flags
 *   dt    +0x04  the data register: write to send, read to receive
 *   baudr +0x08  div[15:0], the divisor
 *   ctrl1 +0x0C  uen (the port), ten, ren, and the interrupt enables
 *   ctrl2 +0x10  stopbn[13:12], trpswap[15]
 */
#define USART1_BASE (AK_APB2_BASE + 0x1000UL)
#define USART2_BASE (AK_APB1_BASE + 0x4400UL)
#define USART3_BASE (AK_APB1_BASE + 0x4800UL)

#define AK_USART_STS(usart)   AK_REG32((usart) + 0x00UL)
#define AK_USART_DT(usart)    AK_REG32((usart) + 0x04UL)
#define AK_USART_BAUDR(usart) AK_REG32((usart) + 0x08UL)
#define AK_USART_CTRL1(usart) AK_REG32((usart) + 0x0CUL)
#define AK_USART_CTRL2(usart) AK_REG32((usart) + 0x10UL)

#define AK_USART_STS_PERR  (1u << 0)
#define AK_USART_STS_FERR  (1u << 1)
#define AK_USART_STS_NERR  (1u << 2)
#define AK_USART_STS_ROERR (1u << 3)
#define AK_USART_STS_RDBF  (1u << 5) /* a received byte is waiting */
#define AK_USART_STS_TDC   (1u << 6) /* transmission of the last byte is done */
#define AK_USART_STS_TDBE  (1u << 7) /* the transmit buffer is empty */

#define AK_USART_CTRL1_REN (1u << 2)
#define AK_USART_CTRL1_TEN (1u << 3)
#define AK_USART_CTRL1_RDBFIEN (1u << 5) /* interrupt when a byte arrives */
#define AK_USART_CTRL1_PEN (1u << 10)
#define AK_USART_CTRL1_UEN (1u << 13)

/* The interrupt controller, which is the Cortex-M4's own and the same on any
 * part: the set-enable registers begin at 0xE000E100 and each holds 32 IRQs.
 * The USART numbers on this part are 37, 38 and 39 - the same as the F405's,
 * which is a coincidence of two vendors numbering the same peripherals. */
#define AK_NVIC_ISER(n) AK_REG32(0xE000E100UL + (n) * 4UL)
#define AK_USART1_IRQ 37u
#define AK_USART2_IRQ 38u
#define AK_USART3_IRQ 39u

/* The system control block is the Cortex-M4's as well, and not the vendor's:
 * these are the same three addresses the F405 port writes, for the same reason
 * the NVIC above is - the core is the core. They are here because the shared
 * reset path (src/arch/arm/cortex-m4/startup.c) uses VTOR and CPACR on both
 * parts, and `reboot` uses AIRCR. */
#define SCB_VTOR  AK_REG32(0xE000ED08UL)
#define SCB_AIRCR AK_REG32(0xE000ED0CUL)
#define SCB_CPACR AK_REG32(0xE000ED88UL)
#define SCB_AIRCR_SYSRESETREQ 0x05FA0004u

#define AK_USART_CTRL2_STOP_SHIFT 12
#define AK_USART_CTRL2_STOP_MASK  0x3u
#define AK_USART_CTRL2_STOP_1     0u
#define AK_USART_CTRL2_STOP_2     2u
#define AK_USART_CTRL2_TRPSWAP    (1u << 15)

/* --- SPI (at32f435_437_spi.h, the spi_type struct) ------------------------
 *
 * An F4-alike again, with the two names that matter changed and one field
 * added: the master bit is `msten`, the divider is split between `mdiv_l` in
 * ctrl1 and `mdiv_h` in ctrl2 (with `mdiv3en` for a divide-by-three), and the
 * flags are `rdbf` and `tdbe` rather than RXNE and TXE.
 *
 *   ctrl1 +0x00  clkpha, clkpol, msten, mdiv_l[5:3], spien, ltf, swcsen,
 *                swcsil (the software chip-select bits)
 *   ctrl2 +0x04  mdiv_h[8], mdiv3en[9]
 *   sts   +0x08  rdbf[0], tdbe[1], bf[7] (busy), and the error flags
 *   dt    +0x0C  the data register
 */
#define SPI1_BASE (AK_APB2_BASE + 0x3000UL)
#define SPI2_BASE (AK_APB1_BASE + 0x3800UL)
#define SPI3_BASE (AK_APB1_BASE + 0x3C00UL)

#define AK_SPI_CTRL1(spi) AK_REG32((spi) + 0x00UL)
#define AK_SPI_CTRL2(spi) AK_REG32((spi) + 0x04UL)
#define AK_SPI_STS(spi)   AK_REG32((spi) + 0x08UL)
#define AK_SPI_DT(spi)    AK_REG32((spi) + 0x0CUL)

#define AK_SPI_CTRL1_CLKPHA (1u << 0)
#define AK_SPI_CTRL1_CLKPOL (1u << 1)
#define AK_SPI_CTRL1_MSTEN  (1u << 2)
#define AK_SPI_CTRL1_MDIV_SHIFT 3
#define AK_SPI_CTRL1_MDIV_MASK  0x7u
#define AK_SPI_CTRL1_SPIEN  (1u << 6)
#define AK_SPI_CTRL1_SWCSEN (1u << 9) /* software chip select */
#define AK_SPI_CTRL1_ORA    (1u << 10) /* the idle level of the chip-select pin */

#define AK_SPI_CTRL2_MDIV_H  (1u << 8)
#define AK_SPI_CTRL2_MDIV3EN (1u << 9)

#define AK_SPI_STS_RDBF (1u << 0) /* a received byte is waiting */
#define AK_SPI_STS_TDBE (1u << 1) /* the transmit buffer is empty */
#define AK_SPI_STS_BF   (1u << 7) /* the bus is busy */

/* The divider the field encodes is a power of two: 0 is divide by two and each
 * step doubles it, up to 7 for divide by 256 in the low field alone. */
#define AK_SPI_MDIV_DIV8  2u
#define AK_SPI_MDIV_DIV16 3u

/* --- I2C (at32f435_437_i2c.h, the i2c_type struct) ------------------------
 *
 * This is the *newer* I2C peripheral - the one with a single timing register
 * rather than the F405's clock-control and rise-time pair - so the arithmetic
 * that configures it is different, not renamed, and the transfer sequences are
 * different too. That difference is why INAV's own AT32 driver carries a timing
 * calculator rather than the F4 constants.
 *
 * **The first version of this block had the conditions in the wrong register,
 * and nothing caught it because nothing used them yet.** It said start,
 * acknowledge and stop were bits 16, 17 and 18 of ctrl1 - which is where
 * `sctrl` and `stretch` live, and a reserved bit - and that there was one data
 * register at +0x20. Reading Artery's own structure for this part (and the
 * sequences its master routines perform) says otherwise, and the difference is
 * not cosmetic:
 *
 *   - the **address, the direction and the byte count are a transfer**, set up
 *     in ctrl2 (`saddr`, `dir`, `cnt`) and started with `genstart`; this
 *     peripheral drives the address phase itself rather than being fed an
 *     address byte through the data register as the F4 is;
 *   - **start and stop are ctrl2 bits 13 and 14**, and the acknowledge for a
 *     read is `nacken` at bit 15 - not ctrl1;
 *   - the data registers are at **+0x24 (rxdt, read) and +0x28 (txdt, write)**,
 *     with the PEC at +0x20. Artery's header comments all three as "+0x20",
 *     which is what invited the error: the comment is a copy-paste and the C
 *     layout (three consecutive members of one struct) is not.
 *
 * The flags below were right in that first version and are unchanged: they are
 * bits 0 to 6 of sts, and `clr` clears each by name.
 *
 *   ctrl1  +0x00  i2cen[0], the seven interrupt enables, dflt[11:8] (the
 *                 digital noise filter), sctrl[16], stretch[17]
 *   ctrl2  +0x04  saddr[9:0], dir[10], genstart[13], genstop[14], nacken[15],
 *                 cnt[23:16], rlden[24] (reload), astopen[25] (auto stop)
 *   clkctrl +0x10 scll[7:0], sclh[15:8], sdad[19:16], scld[23:20], and the
 *                 prescaler split between divh[27:24] and divl[31:28] - the
 *                 same habit this part's SPI has, and the reason the timing
 *                 arithmetic has to know about both fields
 *   sts    +0x18  tdbe[0], tdis[1], rdbf[2], addrf[3], ackfail[4], stopf[5],
 *                 tdc[6], tcrld[7], the error flags, and busyf[15]
 *   clr    +0x1C  writing a one clears the flag of that name
 *   rxdt   +0x24  the byte that arrived - read only
 *   txdt   +0x28  the byte to send - write only
 */
#define I2C1_BASE (AK_APB1_BASE + 0x5400UL)
#define I2C2_BASE (AK_APB1_BASE + 0x5800UL)
#define I2C3_BASE (AK_APB1_BASE + 0x5C00UL)

/* The offsets are named as well as the macros, because this is the one
 * peripheral in the port whose driver goes through a seam that takes a base and
 * an offset (i2c.c, so that a host build can put a modelled bus behind it), and
 * a second copy of these numbers in that file is a second copy to keep in
 * step. */
#define AK_I2C_CTRL1_OFF   0x00UL
#define AK_I2C_CTRL2_OFF   0x04UL
#define AK_I2C_OADDR1_OFF  0x08UL
#define AK_I2C_CLKCTRL_OFF 0x10UL
#define AK_I2C_STS_OFF     0x18UL
#define AK_I2C_CLR_OFF     0x1CUL
#define AK_I2C_RXDT_OFF    0x24UL
#define AK_I2C_TXDT_OFF    0x28UL

#define AK_I2C_CTRL1(i2c)   AK_REG32((i2c) + AK_I2C_CTRL1_OFF)
#define AK_I2C_CTRL2(i2c)   AK_REG32((i2c) + AK_I2C_CTRL2_OFF)
#define AK_I2C_OADDR1(i2c)  AK_REG32((i2c) + AK_I2C_OADDR1_OFF)
#define AK_I2C_CLKCTRL(i2c) AK_REG32((i2c) + AK_I2C_CLKCTRL_OFF)
#define AK_I2C_STS(i2c)     AK_REG32((i2c) + AK_I2C_STS_OFF)
#define AK_I2C_CLR(i2c)     AK_REG32((i2c) + AK_I2C_CLR_OFF)
#define AK_I2C_RXDT(i2c)    AK_REG32((i2c) + AK_I2C_RXDT_OFF)
#define AK_I2C_TXDT(i2c)    AK_REG32((i2c) + AK_I2C_TXDT_OFF)

#define AK_I2C_CTRL1_I2CEN (1u << 0)
#define AK_I2C_CTRL1_DFLT_SHIFT 8
#define AK_I2C_CTRL1_DFLT_MASK  0xFu

/* The transfer, which is what makes this peripheral different to drive: a
 * write of ctrl2 is the address phase, the direction, the number of bytes and
 * the start condition at once.
 *
 * `AK_I2C_CTRL2_KEEP` is everything a new transfer replaces, and getting it
 * wrong is not a cosmetic mistake: the first version of this constant left out
 * bits 16 to 25 - the byte count, the reload bit and auto-stop - because the
 * comment beside it named Artery's mask (0x03FF67FF) and the value beside it
 * was only the low half of it. A second transfer then OR'd its count into the
 * previous one's: three bytes, then one, and the peripheral was told to move
 * three. The modelled bus caught it the first time it ran, which is what the
 * model is for.
 *
 * `nacken` is deliberately *not* in the mask: it is a property of how this
 * driver talks, not of one transfer. */
#define AK_I2C_CTRL2_SADDR_SHIFT 0
#define AK_I2C_CTRL2_SADDR_MASK  0x3FFu
#define AK_I2C_CTRL2_DIR (1u << 10)
#define AK_I2C_CTRL2_GENSTART (1u << 13)
#define AK_I2C_CTRL2_GENSTOP (1u << 14)
#define AK_I2C_CTRL2_NACKEN (1u << 15)
#define AK_I2C_CTRL2_CNT_SHIFT 16
#define AK_I2C_CTRL2_CNT_MASK 0xFFu
#define AK_I2C_CTRL2_RLDEN (1u << 24)   /* load the next count instead of stopping */
#define AK_I2C_CTRL2_ASTOPEN (1u << 25) /* stop by itself when the count runs out */
#define AK_I2C_CTRL2_KEEP 0x03FF67FFu

#define AK_I2C_STS_TDBE    (1u << 0) /* room to send a byte */
#define AK_I2C_STS_TDIS    (1u << 1) /* there is room for the next byte to send */
#define AK_I2C_STS_RDBF    (1u << 2) /* a received byte is waiting */
#define AK_I2C_STS_ADDRF   (1u << 3) /* the address phase is over */
#define AK_I2C_STS_ACKFAIL (1u << 4) /* the device did not acknowledge */
#define AK_I2C_STS_STOPF   (1u << 5)
#define AK_I2C_STS_TDC     (1u << 6) /* the byte has left the shift register */
#define AK_I2C_STS_TCRLD   (1u << 7) /* the count ran out and reload is on */
#define AK_I2C_STS_BUSERR  (1u << 8)
#define AK_I2C_STS_ARLOST  (1u << 9)
#define AK_I2C_STS_BUSYF   (1u << 15)

/* The flags that mean the transfer went wrong for a caller who is talking to
 * one device: a device that did not answer, a bus error, or a lost
 * arbitration. Every one of them has to be cleared by name before the next
 * transfer, which is what the clr register is for. */
#define AK_I2C_STS_ERR (AK_I2C_STS_ACKFAIL | AK_I2C_STS_BUSERR | \
                        AK_I2C_STS_ARLOST)

/* And the same names on the clear register, where a one clears that flag. */
#define AK_I2C_CLR_ADDRF   (1u << 3)
#define AK_I2C_CLR_ACKFAIL (1u << 4)
#define AK_I2C_CLR_STOPF   (1u << 5)
#define AK_I2C_CLR_BUSERR  (1u << 8)
#define AK_I2C_CLR_ARLOST  (1u << 9)

/* The timing register's fields, named as the arithmetic names them. */
#define AK_I2C_CLKCTRL_SCLL_SHIFT  0
#define AK_I2C_CLKCTRL_SCLH_SHIFT  8
#define AK_I2C_CLKCTRL_SDAD_SHIFT  16
#define AK_I2C_CLKCTRL_SCLD_SHIFT  20
#define AK_I2C_CLKCTRL_DIVH_SHIFT  24
#define AK_I2C_CLKCTRL_DIVL_SHIFT  28
#define AK_I2C_CLKCTRL_FIELD_MASK  0xFu

/* --- ADC (at32f435_437_adc.h, the adc_type struct) ------------------------
 *
 * The shape is the F4's - a sequence, a sample time per channel, a trigger and
 * a data register - with this part's names and one thing moved: the clock
 * prescaler lives in a *common* block shared by both ADCs, and it divides the
 * AHB clock rather than the APB one, so the number to ask for changes with the
 * core's speed. At 288 MHz a divide-by-eight is 36 MHz, which is what this part
 * (like the F4) is rated for, and a divider that is wrong does not raise a flag
 * - it returns numbers that are wrong in a way that looks like a pack.
 *
 *   sts   +0x00  occe[1]: the ordinary conversion finished
 *   ctrl1 +0x04  ocpcnt[15:13]: how many ordinary conversions, sqen[8]: scan
 *   ctrl2 +0x08  adcen[0], adcal[2], adcalinit[3], ocete[29:28], ocswtrg[30]
 *   spt1  +0x0C  sample time per channel, three bits each, channels 0-9
 *   spt2  +0x10  the same for channels 10-17
 *   osq3  +0x34  the ordinary sequence, the first entry at [4:0]
 *   odt   +0x4C  the ordinary data: the conversion's result
 */
#define ADC1_BASE    (AK_APB2_BASE + 0x2000UL)
#define ADCCOM_BASE  (AK_APB2_BASE + 0x2300UL)

#define AK_ADC_STS(adc)   AK_REG32((adc) + 0x00UL)
#define AK_ADC_CTRL1(adc) AK_REG32((adc) + 0x04UL)
#define AK_ADC_CTRL2(adc) AK_REG32((adc) + 0x08UL)
#define AK_ADC_SPT1(adc)  AK_REG32((adc) + 0x0CUL)
#define AK_ADC_SPT2(adc)  AK_REG32((adc) + 0x10UL)
#define AK_ADC_OSQ3(adc)  AK_REG32((adc) + 0x34UL)
#define AK_ADC_ODT(adc)   AK_REG32((adc) + 0x4CUL)
#define AK_ADCCOM_CTRL    AK_REG32(ADCCOM_BASE + 0x00UL)

#define AK_ADC_STS_OCCE (1u << 1) /* the conversion finished */

#define AK_ADC_CTRL1_OCPCNT_SHIFT 13 /* how many conversions in the sequence */
#define AK_ADC_CTRL1_OCPCNT_MASK  0x7u

#define AK_ADC_CTRL2_ADCEN    (1u << 0)
#define AK_ADC_CTRL2_ADCSWTRG (1u << 30) /* start a conversion from here */
#define AK_ADC_CTRL2_ADCAL    (1u << 2)  /* calibrate */
#define AK_ADC_CTRL2_ADCALINIT (1u << 3) /* reset the calibration */

#define AK_ADC_SPT_SHIFT(c) ((uint32_t)((c) % 10u) * 3u)
#define AK_ADC_SPT_MASK     0x7u
/* The longest sample time the field has: 640.5 cycles on this part, which is
 * the same "take your time, this is a battery divider" choice the F405 port
 * makes with its longest. */
#define AK_ADC_SPT_LONGEST  0x7u

/* The common block's clock divider, in AHB clocks: 0 is divide by two, and each
 * step adds one. */
#define AK_ADCCOM_CTRL_DIV_SHIFT 16
#define AK_ADCCOM_CTRL_DIV_MASK  0xFu
#define AK_ADC_HCLK_DIV_8 0x06u /* 288 MHz / 8 = 36 MHz */

/* --- TMR: the timers (at32f435_437_tmr.h, the tmr_type struct) -------------
 *
 * This part's timers are the F4's, renamed: the period register is `pr` rather
 * than `arr`, the compare values are `cNdt` rather than `ccrn`, the prescaler is
 * `div`, the control register is `ctrl1`, the capture/compare modes are `ccm1`
 * and `ccm2`, the channel enables are `cce`, the interrupt/DMA enables are
 * `iden`, the event generator is `eveg` and the update flag lives in `ists`.
 * The *values* are the same: the output mode 6 is PWM mode A, exactly as 110b
 * is PWM mode 1 on the F4, and the preload enable sits where OCxPE does.
 *
 *   ctrl1   +0x00  tmren[0] (the counter), ocmen[3] (auto-reload preload)
 *   iden    +0x0C  c1den[9]..c4den[12]: the per-channel DMA requests
 *   ccm1    +0x18  the mode and preload of channels 1 and 2
 *   ccm2    +0x1C  the same for channels 3 and 4
 *   cce     +0x20  c1en[0]..: the channel outputs
 *   div     +0x28  the prescaler
 *   pr      +0x2C  the period
 *   cNdt    +0x34+4*(n-1)  the compare values
 *   dmactrl +0x48  addr[4:0] the burst base in words, dtb[12:8] its length
 *   dmadt   +0x4C  the register a DMA burst writes into
 */
#define TMR2_BASE (AK_APB1_BASE + 0x0000UL)
#define TMR3_BASE (AK_APB1_BASE + 0x0400UL)
#define TMR4_BASE (AK_APB1_BASE + 0x0800UL)

#define AK_TMR_CTRL1(t)   AK_REG32((t) + 0x00UL)
#define AK_TMR_IDEN(t)    AK_REG32((t) + 0x0CUL)
#define AK_TMR_ISTS(t)    AK_REG32((t) + 0x10UL)
#define AK_TMR_EVEG(t)    AK_REG32((t) + 0x14UL)
#define AK_TMR_CCM1(t)    AK_REG32((t) + 0x18UL)
#define AK_TMR_CCM2(t)    AK_REG32((t) + 0x1CUL)
#define AK_TMR_CCE(t)     AK_REG32((t) + 0x20UL)
#define AK_TMR_DIV(t)     AK_REG32((t) + 0x28UL)
#define AK_TMR_PR(t)      AK_REG32((t) + 0x2CUL)
#define AK_TMR_C1DT(t)    AK_REG32((t) + 0x34UL)
#define AK_TMR_C2DT(t)    AK_REG32((t) + 0x38UL)
#define AK_TMR_C3DT(t)    AK_REG32((t) + 0x3CUL)
#define AK_TMR_C4DT(t)    AK_REG32((t) + 0x40UL)
#define AK_TMR_DMACTRL(t) AK_REG32((t) + 0x48UL)
#define AK_TMR_DMADT(t)   AK_REG32((t) + 0x4CUL)

#define AK_TMR_CTRL1_TMREN (1u << 0)
#define AK_TMR_CTRL1_OCMEN (1u << 3) /* auto-reload preload */
#define AK_TMR_IDEN_C1DEN  (1u << 9)
#define AK_TMR_EVEG_UG     (1u << 0)
#define AK_TMR_CCE_C1EN    (1u << 0)
#define AK_TMR_CCE_C2EN    (1u << 4)

/* The output mode and its preload bit, for channels 1 and 2 in ccm1 and for 3
 * and 4 in ccm2. The mode values are the F4's: this one is PWM mode A, which is
 * the F4's PWM mode 1. */
#define AK_TMR_CCM_MODE_PWM_A 6u
#define AK_TMR_CCM_MODE_SHIFT(ch) ((ch) == 1u || (ch) == 3u ? 4u : 12u)
#define AK_TMR_CCM_PRELOAD(ch)    (1u << (AK_TMR_CCM_MODE_SHIFT(ch) - 1u))

/* The burst: the base is a word offset from the timer's own base address, and
 * `dtb` is one less than the number of transfers, which is how the F4's DCR
 * counts them. c1dt is at 0x34, so the base is 0x34/4 = 13. */
#define AK_TMR_DMACTRL_ADDR(addr) ((uint32_t)(addr) / 4u)
#define AK_TMR_DMACTRL_DTB(n)     ((uint32_t)((n) - 1u) << 8)

/* The mux numbers this board's timer pins take, from the reference's own pin
 * table (`timer_def_at32f43x.h`: PB6 and PB7 are mux 2 for TMR4, PB8 and PB9
 * are mux 1 for TMR2). */
#define AK_GPIO_MUX_1 0x1u
#define AK_GPIO_MUX_2 0x2u

/* --- DMA (at32f435_437_dma.h, the dma_type and dma_channel_type structs) ---
 *
 * The controller keeps the flags, and each channel has its own control, count
 * and addresses - the F4's arrangement with this part's names. What is *not*
 * the F4's is how a channel is wired to a peripheral event: the request is
 * selected through a small multiplexer of its own, so a channel sees a request
 * id rather than a fixed table entry (`DMAMUX_DMAREQ_ID_TMR4_CH1` = 0x43 for
 * the first motor channel), and the controller has to be told to use that
 * table at all.
 *
 *   sts     +0x00  the channel flags
 *   clr     +0x04  writing a one clears them
 *   muxsel  +0x100 tblsel[0]: use the request multiplexer
 *   channel n at +0x08+0x14*(n-1): ctrl, dtcnt, paddr, maddr
 *   the multiplexer's own channel n at +0x104+4*(n-1): muxctrl.reqsel[6:0]
 */
#define DMA1_BASE (AK_AHB1_BASE + 0x6400UL)

#define AK_DMA_STS(base)   AK_REG32((base) + 0x00UL)
#define AK_DMA_CLR(base)   AK_REG32((base) + 0x04UL)
#define AK_DMA_MUXSEL(base) AK_REG32((base) + 0x100UL)
#define AK_DMA_CH(base, n)  ((base) + 0x08UL + 0x14UL * ((n) - 1u))
#define AK_DMA_CH_CTRL(base, n)   AK_REG32(AK_DMA_CH(base, n) + 0x00UL)
#define AK_DMA_CH_DTCNT(base, n)  AK_REG32(AK_DMA_CH(base, n) + 0x04UL)
#define AK_DMA_CH_PADDR(base, n)  AK_REG32(AK_DMA_CH(base, n) + 0x08UL)
#define AK_DMA_CH_MADDR(base, n)  AK_REG32(AK_DMA_CH(base, n) + 0x0CUL)
#define AK_DMAMUX_CH(base, n)     AK_REG32((base) + 0x104UL + 4UL * ((n) - 1u))

#define AK_DMA_CTRL_CHEN    (1u << 0)
#define AK_DMA_CTRL_FDTIEN  (1u << 1) /* interrupt when the transfer finishes */
#define AK_DMA_CTRL_DTD     (1u << 4) /* 1 = memory to peripheral */
#define AK_DMA_CTRL_MINC    (1u << 7)
#define AK_DMA_CTRL_PWIDTH_SHIFT 8
#define AK_DMA_CTRL_MWIDTH_SHIFT 10
#define AK_DMA_WIDTH_HALFWORD 1u
#define AK_DMA_CTRL_CHPL_SHIFT 12
#define AK_DMA_PRIORITY_HIGH 2u

#define AK_DMAMUX_REQSEL_MASK 0x7Fu
#define AK_DMAMUX_REQ_TMR4_CH1 0x43u
#define AK_DMAMUX_REQ_TMR4_CH2 0x44u

/* The channel numbers the interrupt controller knows: DMA1 channel n is IRQ 55
 * plus n. */
#define AK_DMA1_CH1_IRQ 56u
#define AK_DMA1_CH2_IRQ 57u

/* --- FLASH: the controller's clock divider (at32f435_437_flash.h) ----------
 *
 * The flash cannot be clocked as fast as the core, so the divider goes up
 * before the PLL does - the same idea as the F4's wait states, done this part's
 * way. `fdiv` is what the code asks for; `fdiv_sts` is what the controller is
 * actually dividing by, and a change is not in force until they agree.
 */
#define FLASH_REG_BASE (AK_AHB1_BASE + 0x3C00UL)
#define FLASH_DIVR     AK_REG32(FLASH_REG_BASE + 0x60UL)
#define FLASH_DIVR_FDIV_MASK  0x3u
#define FLASH_DIVR_FDIV_STS   0x30u /* [5:4], the divider in force */
#define FLASH_DIVR_FDIV_STS_SHIFT 4
#define FLASH_CLOCK_DIV_1 0u
#define FLASH_CLOCK_DIV_2 3u
#define FLASH_CLOCK_DIV_3 1u

/* The rest of the controller (at32f435_437_flash.h, the flash_type struct).
 *
 * Two things about this part are not the F405's, and both matter to a driver
 * that erases and programs:
 *
 *   - a **page** is 2 KB, not the F405's 16 to 128 KB sectors, so every address
 *     arithmetic is recomputed rather than rescaled;
 *   - the part is **two banks of 512 KB** (on the 1 MB part this project
 *     targets), and each bank has its *own* registers: the second bank's
 *     unlock, status, control and address live at their own offsets. A driver
 *     that always used the first bank's registers would erase the wrong
 *     controller - and on a part whose record lives in the top of the second
 *     bank, that is the difference between saving a configuration and erasing
 *     half the firmware.
 */
#define FLASH_UNLOCK    AK_REG32(FLASH_REG_BASE + 0x04UL)
#define FLASH_STS       AK_REG32(FLASH_REG_BASE + 0x0CUL)
#define FLASH_CTRL      AK_REG32(FLASH_REG_BASE + 0x10UL)
#define FLASH_ADDR      AK_REG32(FLASH_REG_BASE + 0x14UL)

#define FLASH2_UNLOCK   AK_REG32(FLASH_REG_BASE + 0x44UL)
#define FLASH2_STS      AK_REG32(FLASH_REG_BASE + 0x4CUL)
#define FLASH2_CTRL     AK_REG32(FLASH_REG_BASE + 0x50UL)
#define FLASH2_ADDR     AK_REG32(FLASH_REG_BASE + 0x54UL)

#define FLASH_STS_OBF      (1u << 0) /* an operation is running */
#define FLASH_STS_PRGMERR  (1u << 2)
#define FLASH_STS_EPPERR   (1u << 4)
#define FLASH_STS_ODF      (1u << 5) /* the operation finished */
#define FLASH_STS_ERRORS   (FLASH_STS_PRGMERR | FLASH_STS_EPPERR)

#define FLASH_CTRL_FPRGM   (1u << 0) /* program */
#define FLASH_CTRL_SECERS  (1u << 1) /* page erase */
#define FLASH_CTRL_ERSTR   (1u << 6) /* start the erase */
#define FLASH_CTRL_OPLK    (1u << 7) /* the lock bit: 1 is locked */

#define FLASH_UNLOCK_KEY1  0x45670123u
#define FLASH_UNLOCK_KEY2  0xCDEF89ABu

/* The two banks of a 1 MB part: 512 KB each, and the bank a driver is talking
 * to is decided by the address it is operating on. */
#define AK_FLASH_BANK1_START 0x08000000UL
#define AK_FLASH_BANK1_END   0x0807FFFFUL
#define AK_FLASH_BANK2_START 0x08080000UL
#define AK_FLASH_BANK2_END   0x080FFFFFUL
#define AK_FLASH_PAGE_BYTES  0x800u /* 2 KB */

/* And the ROM: Artery's own DFU bootloader lives here, in the system memory the
 * part maps below the application's flash. Artery's firmware library returns
 * this address from `systemBootloaderAddress()` in `system_at32f43x.c` (the
 * comment above it reads "AT32 DIAGRAM2-1 AT32F435/437 DFU BOOTLOADER ADDR"),
 * and entering it is a vector-table jump - the first word is the stack pointer
 * and the second is where to branch.
 *
 * The STM32F405's ROM bootloader happens to be at the same address, which is
 * two vendors putting their ROM in the same place rather than one part's fact
 * shared: each port carries its own constant and its own citation. */
#define AK_BOOTLOADER_BASE 0x1FFF0000UL

/* --- SCFG: the system configuration, and the memory map (at32f435_437_scfg.h)
 *
 * cfg1 +0x00  mem_map_sel[2:0]: what address 0x00000000 is. 0 is the main
 *             flash - the part as it comes out of reset and what the firmware
 *             runs from; 1 maps the *boot memory* there instead, which is the
 *             ROM this part's DFU lives in.
 *
 * This is the half the STM32F405 port needed and found by measurement: a
 * branch to the ROM's own address, taken from a running firmware, is not a way
 * into it, because the ROM reads its boot strap at *reset* and a branch is not
 * a reset. What works is to make the map say boot memory and then reset, which
 * is what each port's system.c does - the register is a different one on each
 * (SYSCFG_MEMRMP there, SCFG_CFG1 here) and it is spelled `mem_map_sel` rather
 * than a single bit.
 *
 * The clock has to be on to write it: SCFG is on APB2 and its enable is bit 14
 * of CRM_APB2EN (Artery's `CRM_SCFG_PERIPH_CLOCK` is `MAKE_VALUE(0x44, 14)` -
 * the same register and bit this file already names). */
#define SCFG_BASE   (AK_APB2_BASE + 0x3800UL)
#define SCFG_CFG1   AK_REG32(SCFG_BASE + 0x00UL)
#define SCFG_CFG1_MEM_MAP_MASK 0x7u
#define SCFG_MEM_MAP_MAIN      0u /* the application's flash, out of reset */
#define SCFG_MEM_MAP_BOOT      1u /* the ROM the DFU bootloader is in */
#define CRM_APB2EN_SCFG        (1u << 14)

/* --- PWC: the core regulator's voltage (at32f435_437_pwc.h) ---------------
 *
 * 1.3 V is what Artery's own clock code asks for before it enables the PLL at
 * 288 MHz; its comment names the range that setting is qualified for.
 */
#define PWC_BASE      (AK_APB1_BASE + 0x7000UL)
#define PWC_LDOOV     AK_REG32(PWC_BASE + 0x10UL)
#define PWC_LDOOV_MASK 0x7u
#define PWC_LDO_OUTPUT_1V3 0x1u
#define PWC_LDO_OUTPUT_1V2 0x0u

/* --- USB, the device side (at32f435_437_usb.h, the otg_* structs) ----------
 *
 * This part's USB is the same Synopsys full-speed OTG core the STM32F405 has,
 * at the same place in the memory map (0x50000000, which is AHBPERIPH2_BASE + 0
 * here), with the same register offsets and the same bit positions - which is
 * why the driver above it is the F405 port's design rather than a second one.
 * Three things are this part's, and each is a device that does not work at all
 * if it is missed:
 *
 *   - **the core starts in power-down.** `gccfg.pwrdown` (+0x38, bit 16) has to
 *     be set before anything else does anything, and Artery's own library does
 *     it in the same place: right after the core reset;
 *   - **the phy clock is gated.** `pcgcctl.stoppclk` (+0xE00, bit 0) is opened
 *     by clearing it;
 *   - **there is no PHYSEL bit.** On the F405, GUSBCFG bit 6 selects the
 *     full-speed serial transceiver; this part has only that transceiver and
 *     its header has no such field. What it has there instead is the turn-around
 *     time (`usbtrdtim`, bits 13:10), which Artery's device library writes as 5
 *     once enumeration completes. So a driver that copied the F405's
 *     `PHYSEL | FDMOD` line would be writing zero into this part's turn-around
 *     time and leaving the mode bit alone.
 *
 * One comment in that header is wrong and worth knowing about: the device status
 * register is commented "offset:0x80C" and is at 0x808 - the struct's own order
 * (dcfg, dctl, dsts, reserved, diepmsk at 0x810) settles it, and so does the
 * F4's manual. It is the same kind of copy-paste as the three "+0x20" data
 * registers noted in the I2C block above.
 */
#define OTG_FS_BASE        (AK_PERIPH_BASE + 0x10000000UL)
#define OTG_FS_DEVICE_BASE (OTG_FS_BASE + 0x800u)
#define OTG_FS_PCGCCTL     AK_REG32(OTG_FS_BASE + 0xE00u)

#define OTG_GOTGCTL   AK_REG32(OTG_FS_BASE + 0x000u)
#define OTG_GOTGINT   AK_REG32(OTG_FS_BASE + 0x004u)
#define OTG_GAHBCFG   AK_REG32(OTG_FS_BASE + 0x008u)
#define OTG_GUSBCFG   AK_REG32(OTG_FS_BASE + 0x00Cu)
#define OTG_GRSTCTL   AK_REG32(OTG_FS_BASE + 0x010u)
#define OTG_GINTSTS   AK_REG32(OTG_FS_BASE + 0x014u)
#define OTG_GINTMSK   AK_REG32(OTG_FS_BASE + 0x018u)
/* The receive status queue: every read pops one entry - which endpoint, how many
 * bytes, and what kind of packet - and the bytes it describes are then read out
 * of FIFO 0. Reading this word out of the FIFO instead is a device that eats the
 * first byte of every payload. */
#define OTG_GRXSTSP   AK_REG32(OTG_FS_BASE + 0x020u)
#define OTG_GRXFSIZ   AK_REG32(OTG_FS_BASE + 0x024u)
#define OTG_GCCFG     AK_REG32(OTG_FS_BASE + 0x038u)
/* One per IN endpoint above zero, 4 bytes each from 0x104. Endpoint 0 has its
 * own register at 0x028 and is **not** the zeroth of these.
 *
 * Artery's own SVD names both (`upstream/inav-9.1.0/dev/svd/
 * AT32F437xx_v2.svd`, `USB_OTG1_GLOBAL`):
 *
 *   0x028  DIEPTXF0    "IN Endpoint TxFIFO 0 transmit FIFO size register"
 *   0x028  GNPTXFSIZ   the same register, host-side name
 *   0x100  HPTXFSIZ    "OTGFS **Host** periodic transmit FIFO size register"
 *   0x104  DIEPTXF1    "OTGFS device IN endpoint transmit FIFO size"
 *
 * This macro was `0x100 + 4n`, which put endpoints 1 and 2 in the right place
 * by accident and sent endpoint 0 into the host's periodic register. Endpoint 0
 * then had no transmit FIFO configured at all, and endpoint 0 is the one every
 * host reads the device descriptor from first. The STM32F405 port had the
 * identical macro and the identical fault; its SVD agrees with this one.
 *
 * **Neither board has been flashed with this fix.** It is the same edit that
 * was verified on the F405's host tests; this port's own tests check the
 * address too, but no AT32 has run it. */
#define OTG_DIEPTXF(n) \
    AK_REG32(OTG_FS_BASE + ((n) == 0u ? 0x028u : 0x104u + 4u * ((n) - 1u)))

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

/* GCCFG: out of power-down, which is where this core starts, and VBUS sensing
 * off. The second bit was added on 2026-09-17 after the *F405* on the bench put
 * nothing on the USB bus: this core holds D+ down while it is powered down *or*
 * waiting for VBUS, and this firmware is a device with no host role and no
 * board fact that says the sense pin is wired. See usb.c. */
#define OTG_GCCFG_PWRDOWN (1u << 16)
#define OTG_GCCFG_NOVBUSSENS (1u << 21)

/* PCGCCTL: the phy clock gate. Clearing it is opening it. */
#define OTG_PCGCCTL_STOPPCLK (1u << 0)

/* GUSBCFG: force device mode, and the turn-around time. There is no PHYSEL bit
 * on this part - see the note above. */
#define OTG_GUSBCFG_USBTRDTIM(v) (((uint32_t)(v) & 0xFu) << 10)
#define OTG_GUSBCFG_TRDTIM_48MHZ 5u /* USB_TRDTIM_16 in the reference's terms */
#define OTG_GUSBCFG_FDMOD        (1u << 30)

#define OTG_GAHBCFG_GINTMSK  (1u << 0)
#define OTG_GAHBCFG_TXFELVL  (1u << 7)
#define OTG_GAHBCFG_PTXFELVL (1u << 8)

#define OTG_GRSTCTL_CSRST   (1u << 0)
#define OTG_GRSTCTL_RXFFLSH (1u << 4)
#define OTG_GRSTCTL_TXFFLSH (1u << 5)
#define OTG_GRSTCTL_TXFNUM(f) (((uint32_t)(f) & 0x1Fu) << 6)
#define OTG_GRSTCTL_AHBIDLE (1u << 31)

#define OTG_GINT_RXFLVL  (1u << 4)
#define OTG_GINT_USBRST  (1u << 12)
#define OTG_GINT_ENUMDNE (1u << 13)
#define OTG_GINT_IEPINT  (1u << 18)
#define OTG_GINT_OEPINT  (1u << 19)
#define OTG_GINT_WKUPINT (1u << 31)

/* One receive status entry: the endpoint in the low four bits, the byte count
 * above it, then the data PID, then what kind of packet it was. The two kinds
 * this driver cares about are the same numbers the F4's core uses. */
#define OTG_GRXSTS_EPNUM  0xFu
#define OTG_GRXSTS_BCNT   (0x7FFu << 4)
#define OTG_GRXSTS_PKTSTS (0xFu << 17)
#define OTG_GRXSTS_DATA   2u
#define OTG_GRXSTS_SETUP_DONE 4u /* the setup stage is over: STUP follows */
#define OTG_GRXSTS_SETUP  6u

/* DCFG: the device's address and the speed it answers at. */
#define OTG_DCFG_DSPD_FS (0x3u << 0)
#define OTG_DCFG_DAD(a)  (((uint32_t)(a) & 0x7Fu) << 4)

/* DCTL: the soft disconnect, which is what makes a host re-enumerate. */
#define OTG_DCTL_RWUSIG      (1u << 0)
#define OTG_DCTL_SDIS        (1u << 1)
#define OTG_DCTL_GNPINNAKSTS (1u << 2) /* read-only: a global IN NAK is in force */
#define OTG_DCTL_GOUTNAKSTS  (1u << 3) /* read-only */
/* Write 1 to release every non-periodic IN endpoint, endpoint 0 included, which
 * a USB reset NAKs until this is written.
 *
 * The F405's copy of this header carried bits 2 and 3 under the names of this
 * control and its OUT twin, which are read-only status; both headers now read
 * the same way, as the init comment in this port's `usb.c` says they should.
 * The offsets come from the AT32 SVD's own DCTL description - `gnpinnaksts`,
 * `goutnaksts`, `cgnpinnak` - which the F405's ST header agrees with field for
 * field. */
#define OTG_DCTL_CGNPINNAK   (1u << 8)

/* DSTS: what the core ended up at, and whether the bus is suspended. */
#define OTG_DSTS_SUSPSTS      (1u << 0)
#define OTG_DSTS_ENUMSPD_MASK (0x3u << 1)

/* DIEPCTL/DOEPCTL share these bit positions in both directions. */
#define OTG_DEPCTL_MPS(v)     ((uint32_t)(v) & 0x7FFu)
#define OTG_DEPCTL_USBAEP     (1u << 15)
#define OTG_DEPCTL_EPTYP_CTRL (0u << 18)
#define OTG_DEPCTL_EPTYP_BULK (2u << 18)
#define OTG_DEPCTL_STALL      (1u << 21)
#define OTG_DEPCTL_TXFNUM(n)  (((uint32_t)(n) & 0xFu) << 22)
#define OTG_DEPCTL_CNAK       (1u << 26)
#define OTG_DEPCTL_SNAK       (1u << 27)
#define OTG_DEPCTL_EPDIS      (1u << 30)
#define OTG_DEPCTL_EPENA      (1u << 31)

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
 * nineteen bits rather than seven. A bulk OUT endpoint is armed one packet at a
 * time: write both fields, then EPENA and CNAK in its control word. */
#define OTG_DOEPTSIZ_XFRSIZ(v) ((uint32_t)(v) & 0x7FFFFu)
#define OTG_DOEPTSIZ_PKTCNT(n) (((uint32_t)(n) & 0x3FFu) << 19)

#endif /* AK_ARCH_AT32F435_REGS_H */
