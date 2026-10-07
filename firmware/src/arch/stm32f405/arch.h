#ifndef AK_ARCH_STM32F405_ARCH_H
#define AK_ARCH_STM32F405_ARCH_H

#include <stdint.h>

#include "regs.h"
#include "ak_output.h"

/*
 * The STM32F405 port. Boards choose pins; this layer moves registers.
 */

typedef struct {
    uint32_t port; /* GPIOA_BASE ... */
    uint8_t  pin;  /* 0-15 */
} ak_pin_t;

#define AK_PIN(port_, pin_) ((ak_pin_t){ .port = (port_), .pin = (pin_) })

/* Clock (clk.c) */
void     ak_clk_init(void);
uint32_t ak_clk_sysclk_hz(void);
uint32_t ak_clk_apb1_hz(void);
uint32_t ak_clk_apb2_hz(void);
int      ak_clk_hse_ok(void); /* 0 when the board fell back to HSI */
uint32_t ak_clk_hse_hz(void); /* the crystal as measured against HSI; 0 if not */

/* GPIO (gpio.c) */
void ak_pin_output(ak_pin_t pin, int open_drain, uint32_t speed);
void ak_pin_af(ak_pin_t pin, uint8_t af, uint32_t pull);
/* For a wire that is only ever allowed to pull down - I2C's, and nothing
 * else's here. */
void ak_pin_af_open_drain(ak_pin_t pin, uint8_t af, uint32_t pull);
void ak_pin_analog(ak_pin_t pin);
void ak_pin_set(ak_pin_t pin, int level);
void ak_pin_toggle(ak_pin_t pin);
int  ak_pin_get(ak_pin_t pin);

/* USART (usart.c).
 *
 * Configuring a port and choosing the console are separate operations on
 * purpose. They were one, and it was wrong the moment a second port existed:
 * initialising the receiver and then the GPS left the console pointing at the
 * GPS's transmit pin, so the firmware booted silently and the console was dead
 * on hardware - a fault no host test can see, because the port code is not in
 * the host build. */
void ak_uart_init(uint32_t usart, ak_pin_t tx, ak_pin_t rx, uint32_t baud,
                  uint8_t af);

/* Makes a configured port the sink for console output. Called once, by the
 * board, for the port the console is actually wired to. */
void ak_console_attach(uint32_t usart);

/* Which port console output is going to. Exposed so the board can check that
 * the firmware's idea of the console matches its own, which is a mistake that
 * otherwise shows up as a silent board. */
uint32_t ak_console_port(void);

/* Writes bytes out of any configured port, for the ones that need to talk as
 * well as listen. */
int ak_uart_write_bytes(uint32_t usart, const char *data, unsigned len);

/* Returns 1 and stores the byte when one has arrived. Polled, because the
 * console is typed at by one person and an interrupt for that would be a
 * interrupt to get wrong. */
int ak_uart_poll_rx(uint32_t usart, uint8_t *byte);

/* Interrupt-driven receive. Each port gets its own ring buffer, and the bytes
 * are emptied from the main loop - the receiver is on one port and the GPS on
 * another, and neither should be able to lose the other's data. */
void ak_uart_rx_init(uint32_t usart, ak_pin_t tx, ak_pin_t rx, uint32_t baud,
                     uint8_t af);
/* The same, with the line settings SBUS wants: 100000 baud, even parity, two
 * stop bits. Kept as a separate entry point rather than a flag on the one
 * above, because every other port in this firmware is 8N1 and a boolean
 * argument at four call sites is a boolean argument nobody reads. */
void ak_uart_rx_init_format(uint32_t usart, ak_pin_t tx, ak_pin_t rx,
                            uint32_t baud, uint8_t af, int even_parity,
                            int two_stop_bits);
int  ak_uart_rx_pop(uint32_t usart, uint8_t *byte);
uint32_t ak_uart_rx_dropped(uint32_t usart);

/* SPI master, mode 3, 8-bit, software chip select (the caller drives CS). */
void ak_spi_init(uint32_t spi, ak_pin_t sck, ak_pin_t miso, ak_pin_t mosi,
                 uint8_t af);
int  ak_spi_transfer(uint32_t spi, const uint8_t *tx, uint8_t *rx, unsigned len);

/* The DMA path (spi.c, roadmap phase 1.2). `_start` programs the streams and
 * returns with the receive stream's interrupt armed; the bytes are in `rx`
 * when it fires. `ak_spi_transfer_dma` is start-then-wait, for a caller that
 * wants the read finished before it goes on.
 *
 * Only SPI1 has a stream on this part, and a bus without one is refused rather
 * than served slowly - the table and its two refusals are in spi.c, and they
 * are the reason this is not simply `ak_spi_transfer` with a flag. */
int  ak_spi_transfer_dma_start(uint32_t spi, const uint8_t *tx, uint8_t *rx,
                               unsigned len);
int  ak_spi_transfer_dma(uint32_t spi, const uint8_t *tx, uint8_t *rx,
                         unsigned len);

/* ADC (adc.c). One channel, one conversion at a time, polled: the caller sets
 * a pin up as analog with ak_pin_analog() and then asks for counts. */
void ak_adc_init(uint32_t adc, uint32_t channel);
int  ak_adc_read_counts(uint32_t adc, uint16_t *counts);

/* I2C (i2c.c). Master, 7-bit addresses, register-oriented: a byte for the
 * register, then the bytes. That is the shape every sensor on the bus has, and
 * the shape ak_bus.h presents, which is why a driver written for one bus works
 * on the other. Both directions return 0 on success and -1 on a timeout, and a
 * timeout resets the peripheral before it does. */
void ak_i2c_init(uint32_t i2c, ak_pin_t scl, ak_pin_t sda, uint8_t af,
                 uint32_t speed_hz);
int  ak_i2c_read_reg(uint32_t i2c, uint8_t address, uint8_t reg, uint8_t *buf,
                     unsigned len);
int  ak_i2c_write_reg(uint32_t i2c, uint8_t address, uint8_t reg, uint8_t value);

/*
 * Free a bus a slave is holding SDA down on. Returns 1 if the bus was stuck
 * and is now free, 0 if it was not stuck (the usual case, and free to ask),
 * and -1 if nine clocks and a STOP did not free it - which is a short, or
 * something that is not an I2C slave, and is a fault no master can clock away.
 *
 * The read and write above call this themselves, so nothing has to ask; it is
 * exposed because "is the bus stuck, and can it be freed" is a question the
 * board's own health reporting wants to answer out loud.
 */
int ak_i2c_bus_recover(uint32_t i2c, ak_pin_t scl, ak_pin_t sda, uint8_t af,
                       uint32_t speed_hz);

/* SysTick (systick.c). The microsecond reading is the millisecond one plus the
 * fraction SysTick is holding - see the note on ak_arch_time_us for the
 * rollover it has to read around. On the host build both come from one counter
 * a test moves, and `ak_host_tick_advance_us` is the finer of the two steps. */
void     ak_arch_time_init(void);
uint32_t ak_arch_time_ms(void);
uint32_t ak_arch_time_us(void);

/* Host only: the tick is a variable no timer moves there, so a test has to move
 * it. `ak_host_tick_advance` steps whole milliseconds and every caller of it
 * predates the microsecond clock, so it leaves the fraction alone rather than
 * clearing it; the `_us` form is the finer step and exists for the scheduler's
 * deadlines, which a millisecond cannot express.
 *
 * Declared unconditionally rather than behind `#ifdef AK_HOST_TICK`: that flag
 * is set per *object* (see the Makefile's two rules for it) and no header
 * defines it, so a guard here would hide these from the very tests that need
 * them. A target build that calls one gets an undefined reference, which is the
 * right failure - there is no SysTick to move on a board that has one running. */
void ak_host_tick_advance(uint32_t ms);
void ak_host_tick_advance_us(uint32_t us);

/* Startup support (startup.c) */
void ak_fpu_enable(void);

/* System control (system.c) */
void ak_arch_reset(void);
/* And the other way out: the part's ROM bootloader (system.c). It does not
 * return - see the file - and it is what the console's `dfu` command is. */
void ak_arch_bootloader(void);

/*
 * Motors and servos (output.c).
 *
 * One servo output: the pad it comes out on, its alternate function, and which
 * channel of the servo timer it is.
 *
 * Which pads is a *board* fact and arrives here as an argument, the same rule
 * the console's pins, the receiver's port and the USB pair follow - this layer
 * does not read a board header. On this part it is not cosmetic either: the
 * WeAct board's servos are TIM2 channels 1 and 2 on PA0/PA1, and the Feather's
 * breakout brings out neither pad, so its servos are TIM4 channels 3 and 4 on
 * PB8/PB9. The channel is therefore not always 1 and 2.
 *
 * Both timers are on APB1, so both count at 84 MHz and the 1 us tick, the 20 ms
 * period and the pulse-width-in-microseconds writing in output.c are the same
 * arithmetic on either; a board naming a timer that is not one of those two
 * gets no clock and a servo that does not move.
 */
typedef struct {
    uint32_t port;    /* GPIOA_BASE, GPIOB_BASE */
    uint8_t  pin;
    uint8_t  af;      /* 1 on TIM2's pads, 2 on TIM4's */
    uint8_t  channel; /* 1..4 */
} ak_servo_out_t;

/* `servo_timer` is the one timer the servo bank is on and `servos` is where its
 * channels come out; more than AK_MAX_SERVOS entries are ignored, because the
 * frame the core hands to ak_output_write() carries that many. */
void ak_output_init(uint32_t servo_timer, const ak_servo_out_t *servos,
                    unsigned count);
/* Which timer the servos ended up on, for a board's report: naming it from a
 * literal is how a banner ends up describing a clock the firmware is not on,
 * which the AT32 port paid for in 2026-09-29 (traps 203). "none" before init. */
const char *ak_output_servo_timer_name(void);
void ak_output_write(const ak_output_frame_t *frame);
void ak_output_set_rate(uint32_t khz);
uint32_t ak_output_dshot_hz(void);
uint16_t ak_output_dshot_period(void);
uint16_t ak_output_ccr_zero(void);
uint16_t ak_output_ccr_one(void);
uint32_t ak_output_frames_sent(void);
uint32_t ak_output_frames_skipped(void);
int ak_output_busy(void);

/* USB, as a virtual serial port (usb.c). Init brings the device up and
 * connects it; poll drives enumeration and the bulk endpoint and has to be
 * called often, from wherever the console is polled; write queues bytes for
 * the host, read hands back a byte the host typed, and neither blocks - what
 * will not fit either direction is dropped and counted. ready() is 1 once the
 * host has configured the device, which is the point at which its serial
 * driver has bound and what is written is worth reading. */
/* The board names the two data pins, as it does for every other port: this
 * layer moves registers and does not know a pin map. */
void     ak_usb_init(ak_pin_t dm, ak_pin_t dp);
void     ak_usb_poll(void);
unsigned ak_usb_write(const char *data, unsigned len);
int      ak_usb_read(char *byte);
int      ak_usb_ready(void);
uint32_t ak_usb_dropped(void);
uint32_t ak_usb_rx_dropped(void);

/* Internal flash (flash.c). The sectors are numbered as RM0090 numbers them,
 * 0-11 on a 1 MB part; sector 11 is the last 128 KB and is where saved
 * configuration lives. */
int ak_flash_erase_sector(uint8_t sector);
int ak_flash_program(uint32_t address, const void *source, uint32_t length);

#endif /* AK_ARCH_STM32F405_ARCH_H */
