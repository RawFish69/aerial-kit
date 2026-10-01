#ifndef AK_ARCH_AT32F435_ARCH_H
#define AK_ARCH_AT32F435_ARCH_H

#include <stdint.h>

#include "regs.h"
#include "ak_output.h"

/*
 * The AT32F435 port: the wing's own flight controller, which is an Artery part
 * rather than an STM32 ([25-at32-survey.md] is the survey that decided to write
 * it at all, and it has the chip, the board and what is still open about the
 * board).
 *
 * **The contract is the F405 port's, and the implementation is not.** A board
 * file answers `ak_board.h`, the arch layer moves registers, and the flight
 * core is untouched - that is what makes this a port rather than a rewrite. But
 * the registers are this part's, so nothing here is shared with
 * `src/arch/stm32f405/`: the clock has a different PLL, the pins are configured
 * by a different scheme, and the flash has different sectors.
 *
 * What is here so far is the clock, because it is the first thing a board needs
 * and the first thing that can be checked without one. The rest arrives with
 * the peripheral it belongs to, in the order the survey lists.
 */

/*
 * One arch per firmware, and two of them in one host test binary. The test
 * build compiles this file beside the F405's, and both define the contract's
 * names, so the Makefile defines AK_HOST_AT32 for this object alone and every
 * entry point here is renamed for it. The firmware build never defines it, so
 * the board file calls the plain names it expects. This block comes first,
 * because it has to rename the declarations below as well as the definitions.
 */
#ifdef AK_HOST_AT32
#define ak_clk_init      ak_at32_clk_init
#define ak_clk_sysclk_hz ak_at32_clk_sysclk_hz
#define ak_clk_apb1_hz   ak_at32_clk_apb1_hz
#define ak_clk_apb2_hz   ak_at32_clk_apb2_hz
#define ak_clk_hse_ok    ak_at32_clk_hse_ok
#define ak_clk_hse_hz    ak_at32_clk_hse_hz
#define ak_pin_output           ak_at32_pin_output
#define ak_pin_af               ak_at32_pin_af
#define ak_pin_drive            ak_at32_pin_drive
#define ak_pin_af_open_drain    ak_at32_pin_af_open_drain
#define ak_pin_analog           ak_at32_pin_analog
#define ak_pin_set              ak_at32_pin_set
#define ak_pin_toggle           ak_at32_pin_toggle
#define ak_pin_get              ak_at32_pin_get
#define ak_arch_time_init       ak_at32_time_init
#define ak_arch_time_ms         ak_at32_time_ms
#define ak_time_init            ak_at32_portable_time_init
#define ak_time_ms              ak_at32_portable_time_ms
#define SysTick_Handler         ak_at32_systick_handler
#define ak_uart_init            ak_at32_uart_init
#define ak_uart_init_swap       ak_at32_uart_init_swap
#define ak_uart_init_af         ak_at32_uart_init_af
#define ak_console_attach       ak_at32_console_attach
#define ak_console_port         ak_at32_console_port
#define ak_uart_write_bytes     ak_at32_uart_write_bytes
#define ak_console_write_raw    ak_at32_console_write_raw
#define ak_uart_poll_rx         ak_at32_uart_poll_rx
#define ak_uart_rx_init_format  ak_at32_uart_rx_init_format
#define ak_uart_rx_init_af      ak_at32_uart_rx_init_af
#define ak_flash_erase_page     ak_at32_flash_erase_page
#define ak_flash_program        ak_at32_flash_program
#define ak_spi_init             ak_at32_spi_init
#define ak_spi_transfer         ak_at32_spi_transfer
/* And the loop under it, which the host build keeps for the register test. */
#define ak_spi_transfer_loop    ak_at32_spi_transfer_loop
#define ak_i2c_init             ak_at32_i2c_init
#define ak_arch_reset           ak_at32_arch_reset
#define ak_arch_bootloader      ak_at32_arch_bootloader
#define ak_i2c_read_reg         ak_at32_i2c_read_reg
#define ak_i2c_write_reg        ak_at32_i2c_write_reg
#define ak_uart_rx_init         ak_at32_uart_rx_init
#define ak_uart_rx_pop          ak_at32_uart_rx_pop
#define ak_uart_rx_dropped      ak_at32_uart_rx_dropped
#define USART1_IRQHandler       ak_at32_usart1_irq
#define USART2_IRQHandler       ak_at32_usart2_irq
#define USART3_IRQHandler       ak_at32_usart3_irq
#define ak_adc_init             ak_at32_adc_init
#define ak_adc_read_counts      ak_at32_adc_read_counts
#define ak_output_init          ak_at32_output_init
#define ak_output_write         ak_at32_output_write
#define ak_output_set_rate      ak_at32_output_set_rate
#define ak_output_dshot_hz      ak_at32_output_dshot_hz
#define ak_output_dshot_period  ak_at32_output_dshot_period
#define ak_output_ccr_zero      ak_at32_output_ccr_zero
#define ak_output_ccr_one       ak_at32_output_ccr_one
#define ak_output_frames_sent   ak_at32_output_frames_sent
#define ak_output_frames_skipped ak_at32_output_frames_skipped
#define ak_output_busy          ak_at32_output_busy
#define ak_usb_init             ak_at32_usb_init
#define ak_usb_poll             ak_at32_usb_poll
#define ak_usb_write            ak_at32_usb_write
#define ak_usb_read             ak_at32_usb_read
#define ak_usb_ready            ak_at32_usb_ready
#define ak_usb_dropped          ak_at32_usb_dropped
#define ak_usb_rx_dropped       ak_at32_usb_rx_dropped
#define DMA1_Channel1_IRQHandler ak_at32_dma1_ch1_irq
#define DMA1_Channel2_IRQHandler ak_at32_dma1_ch2_irq
#endif

/*
 * Clock (clk.c). The names are the F405 port's because the *contract* is the
 * same: a board brings the clock up and then asks what it ended up with. The
 * argument is the one place the two ports' signatures differ, and it is the
 * F405's half of this item that is missing here: that port *measures* its
 * crystal at boot and takes nothing, while this part has no way to measure one
 * (clk.c's header has the register-level evidence) and is therefore told. A
 * board passes the value out of its own header, the same way it passes its
 * console's pins and its USB pair - this layer does not read a board header.
 */
void     ak_clk_init(uint32_t hext_hz);
uint32_t ak_clk_sysclk_hz(void);
uint32_t ak_clk_apb1_hz(void);
uint32_t ak_clk_apb2_hz(void);
int      ak_clk_hse_ok(void); /* 0 when the board fell back to the internal clock */

/* The crystal the PLL was built on, in Hz, or 0 on the internal clock - the
 * same number ak_clk_init() was handed, kept so that a board can print it
 * beside the frequency it produced. On this part it is a *declaration* rather
 * than a measurement: there is no way to time HEXT against anything it did not
 * come from. clk.c's header has the evidence. */
uint32_t ak_clk_hse_hz(void);

/*
 * The arithmetic on its own, so it can be checked without a register: what the
 * PLL produces from a reference clock, an input divider, a multiplier and an
 * output-divider exponent. `fr` is the field's encoding, not the divisor - see
 * regs.h, where 1 means divide by two.
 */
uint32_t ak_at32_pll_hz(uint32_t ref_hz, unsigned ms, unsigned ns, unsigned fr);

/*
 * And the inverse, which is what the port actually needs: the ms/ns/fr that
 * turn a board's crystal into the target exactly, within the ranges Artery
 * documents. 1 and the three numbers when it lands, 0 when this part cannot
 * hit the target from that crystal - which is a real answer and not a
 * rounding: the USB clock is this PLL over six, so only an exact 288 MHz is
 * 48 MHz. Exported because the host test asks it directly - pinning its answer
 * for the board's own crystal and its refusal for one this part cannot use -
 * as well as driving three crystals through the registers that no board in
 * this tree carries.
 */
uint32_t ak_at32_pll_cfg_for(uint32_t hext_hz, uint32_t target_hz,
                            uint32_t *ms_out, uint32_t *ns_out,
                            uint32_t *fr_out);

/* Pins (gpio.c).
 *
 * `ak_pin_t` is a port base and a bit, the same shape the F405 port uses, so a
 * board file reads the same way on both parts - and the numbers inside are not
 * the same, which is what the port's host test is for. */
typedef struct {
    uint32_t port; /* GPIOA_BASE ... */
    uint8_t  pin;  /* 0-15 */
} ak_pin_t;

#define AK_PIN(port_, pin_) ((ak_pin_t){ .port = (port_), .pin = (pin_) })

void ak_pin_output(ak_pin_t pin, int open_drain, uint32_t speed);
void ak_pin_af(ak_pin_t pin, uint8_t af, uint32_t pull);
/* The drive strength alone, for a pin whose mode is already chosen - the USB
 * data pins, which this part's own configuration gives the stronger driver. */
void ak_pin_drive(ak_pin_t pin, uint32_t speed);
void ak_pin_af_open_drain(ak_pin_t pin, uint8_t af, uint32_t pull);
void ak_pin_analog(ak_pin_t pin);
void ak_pin_set(ak_pin_t pin, int level);
void ak_pin_toggle(ak_pin_t pin);
int  ak_pin_get(ak_pin_t pin);

/* The tick (systick.c). SysTick is the same core peripheral as on any other
 * Cortex-M4, so this file is the F405's with this part's clock behind it - the
 * reload is the system clock over a thousand, and the system clock is whatever
 * the clock code ended up on. */
void     ak_arch_time_init(void);
uint32_t ak_arch_time_ms(void);

/* And the names this port supplies to the rest of the firmware: the portable
 * clock contract of core/time.h, and the core's own SysTick handler, which a
 * vector table points at and no header otherwise declares. They are declared
 * here as well so that the host test can name them after the rename above. */
void     ak_time_init(void);
uint32_t ak_time_ms(void);
void     SysTick_Handler(void);

/* A reset (system.c). The core's AIRCR on this part as well: the same write
 * the F405 port makes, because the register belongs to the Cortex-M4 and not
 * to either vendor. `reboot` on the console is the only caller. */
void ak_arch_reset(void);

/* And the other way out (system.c): a jump into the part's ROM bootloader,
 * which is where Artery's DFU lives. It does not return. On this board that is
 * the difference between reflashing from a terminal and needing a solder joint
 * - see the file. */
void ak_arch_bootloader(void);

/* USART (usart.c).
 *
 * The console's port for now: configure, attach, write, and poll for a typed
 * byte. The interrupt-driven receive the receiver and the GPS need - one ring
 * per port, emptied by the main loop - arrives with those milestones, because
 * this port has nothing to put in a ring yet.
 *
 * `ak_uart_init_swap` exists because this part can swap its transmit and
 * receive pins in the peripheral, and the wing's GPS port is wired that way.
 * The F405 cannot do it at all, which is why the two ports' APIs differ here
 * by one function rather than agreeing exactly. */
void ak_uart_init(uint32_t usart, ak_pin_t tx, ak_pin_t rx, uint32_t baud,
                  uint8_t af);
void ak_uart_init_swap(uint32_t usart, ak_pin_t tx, ak_pin_t rx, uint32_t baud,
                       uint8_t af);

/* And when the two pins' function numbers differ, which on this board they do
 * on exactly one port: the receiver listens on PB0 with function 6 and talks
 * on PA8 with function 8. The F405 port takes one number because every F4
 * USART pin it uses shares one. */
void ak_uart_init_af(uint32_t usart, ak_pin_t tx, uint8_t tx_af, ak_pin_t rx,
                     uint8_t rx_af, uint32_t baud);
void ak_console_attach(uint32_t usart);
uint32_t ak_console_port(void);
int  ak_uart_write_bytes(uint32_t usart, const char *data, unsigned len);
int  ak_uart_poll_rx(uint32_t usart, uint8_t *byte);

/* Interrupt-driven receive: one ring per port, emptied by the main loop, which
 * is what the receiver (its own port) and the GPS (another) need so that
 * neither can lose the other's bytes. The console keeps its polling. */
void ak_uart_rx_init(uint32_t usart, ak_pin_t tx, ak_pin_t rx, uint32_t baud,
                     uint8_t af);

/* The same port, for a receiver that does not speak 8N1: SBUS is 100000 baud,
 * even parity, two stop bits. The F405 port has the same second entry point,
 * because the protocol belongs to the receiver and not to the part. */
void ak_uart_rx_init_format(uint32_t usart, ak_pin_t tx, ak_pin_t rx,
                            uint32_t baud, uint8_t af, int even_parity,
                            int two_stop_bits);

/* The general form the two above are wrappers over: each pin's own function
 * number, the frame, and the peripheral's pin swap. The receiver uses it (two
 * function numbers, 8N1), the GPS uses it (one function number, the swap bit),
 * and SBUS - if a receiver with an inverter in front of it is ever fitted -
 * would use all three. */
void ak_uart_rx_init_af(uint32_t usart, ak_pin_t tx, uint8_t tx_af, ak_pin_t rx,
                        uint8_t rx_af, uint32_t baud, int even_parity,
                        int two_stop_bits, int swap);

int  ak_uart_rx_pop(uint32_t usart, uint8_t *byte);
uint32_t ak_uart_rx_dropped(uint32_t usart);

/* The three receive handlers, which a vector table points at and no header
 * otherwise declares - declared here so the host test can call them by the
 * names the rename above gives them. */
void USART1_IRQHandler(void);
void USART2_IRQHandler(void);
void USART3_IRQHandler(void);

/* The pack's divider (adc.c). One channel, one conversion at a time, polled:
 * the caller sets the pin up as analog with ak_pin_analog() and then asks for
 * counts. The prescaler is the common block's and divides the AHB clock. */
void ak_adc_init(uint32_t adc, uint32_t channel);
int  ak_adc_read_counts(uint32_t adc, uint16_t *counts);

/* Motors and servos (output.c): the servos as ordinary PWM on TMR2, the motors
 * as DShot on TMR4 with one DMA channel each through this part's request
 * multiplexer. See the file for why per-channel rather than the F405's burst. */
void ak_output_init(void);
void ak_output_write(const ak_output_frame_t *frame);
void ak_output_set_rate(uint32_t khz);
uint32_t ak_output_dshot_hz(void);
uint16_t ak_output_dshot_period(void);
uint16_t ak_output_ccr_zero(void);
uint16_t ak_output_ccr_one(void);
uint32_t ak_output_frames_sent(void);
uint32_t ak_output_frames_skipped(void);
int ak_output_busy(void);

/*
 * USB, as a virtual serial port (usb.c). The same contract as the F405 port's,
 * because it is the same core: init brings the device up and connects it, poll
 * drives enumeration and the bulk endpoint and has to be called often (from
 * wherever the console is polled), write queues bytes for the host, read hands
 * back a byte the host typed, and neither blocks - what will not fit either
 * direction is dropped and counted. ready() is 1 once the host has configured
 * the device, which is the point at which its serial driver has bound and what
 * is written is worth reading.
 *
 * The board names the two data pins, as it does for every other port: this
 * layer moves registers and does not know a pin map. What it does know is this
 * part's clock and the three registers that differ from the F405's - see the
 * file.
 */
void     ak_usb_init(ak_pin_t dm, ak_pin_t dp);
void     ak_usb_poll(void);
unsigned ak_usb_write(const char *data, unsigned len);
int      ak_usb_read(char *byte);
int      ak_usb_ready(void);
uint32_t ak_usb_dropped(void);
uint32_t ak_usb_rx_dropped(void);

/* The two DMA handlers a motor frame finishes in, declared here for the same
 * reason the USART ones are: a vector table points at them and no header
 * otherwise says so. */
void DMA1_Channel1_IRQHandler(void);
void DMA1_Channel2_IRQHandler(void);

/* The console's sink, which core/console.h declares and this port provides -
 * declared here as well so that the host test can name it after the rename
 * above, the same reason ak_time_ms() is. */
void ak_console_write_raw(const char *data, unsigned len);

/* The internal flash (flash.c).
 *
 * Two operations, and the shape is this part's: an erase takes an *address*
 * (the page it is in, 2 KB aligned) rather than the F405's sector number,
 * because this controller has an address register and two banks to choose
 * between. `ak_flash_program` is the same either way - word aligned, whole
 * words, and it refuses a length or address that crosses a bank. */
int ak_flash_erase_page(uint32_t address);
int ak_flash_program(uint32_t address, const void *source, uint32_t length);

/* SPI master for the sensor bus (spi.c): mode 3, eight bits, software chip
 * select the caller drives. The gyro on this board is an ICM-42688P on SPI1,
 * which is the part the F405 port's SPI was written for as well. */
void ak_spi_init(uint32_t spi, ak_pin_t sck, ak_pin_t miso, ak_pin_t mosi,
                 uint8_t af);
int  ak_spi_transfer(uint32_t spi, const uint8_t *tx, uint8_t *rx, unsigned len);

/* I2C for the barometer's bus (i2c.c): the pins, the clock, the timing register
 * and the transfers.
 *
 * The same contract as the F405 port - a register number and then the bytes,
 * 0 on success and -1 on a timeout, and a bus that is left able to try again -
 * with this part's sequences behind it. They are not the same sequences: this
 * peripheral is told the address, the direction and the byte count at once and
 * performs the address phase, the acknowledge and the stop itself, where the
 * F405 is fed an address byte and has to move its own acknowledge. See the file
 * for the three differences that matter. */
void ak_i2c_init(uint32_t i2c, ak_pin_t scl, ak_pin_t sda, uint8_t af,
                 uint32_t speed_hz);
int  ak_i2c_read_reg(uint32_t i2c, uint8_t address, uint8_t reg, uint8_t *buf,
                     unsigned len);
int  ak_i2c_write_reg(uint32_t i2c, uint8_t address, uint8_t reg,
                      uint8_t value);

#endif /* AK_ARCH_AT32F435_ARCH_H */
