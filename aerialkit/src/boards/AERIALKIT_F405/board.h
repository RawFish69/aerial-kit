#ifndef AK_BOARD_AERIALKIT_F405_H
#define AK_BOARD_AERIALKIT_F405_H

#include "arch.h"

/* The arch this board is built against. It is also the default, so this line
 * changes no build - it is here so that the pairing is checked in both
 * directions rather than only away from the default: the Makefile refuses
 * `ARCH=at32f435` with this board by name, instead of failing at link with
 * `undefined reference to ak_flash_erase_sector`. See the block at the top of
 * the Makefile. */
#define AK_BOARD_ARCH stm32f405

/*
 * Board: WeAct STM32F405RGT6 core board (STM32F405RGT6, 1 MB flash).
 *
 * The three hardware facts this file assumes are listed with the evidence for
 * each in docs/02-hardware.md, and each one is a single line to correct:
 *
 *   HSE 8 MHz             - measured on this board, 2026-09-29, by the board's
 *                           own output: the banner's `clocks:` line reads
 *                           `HSE 8 MHz x PLL` on the measured branch (no
 *                           `, assumed`), and the image that printed it runs
 *                           `PLLM = 8` with USB enumerating, which puts
 *                           HSE/PLLM at 1 MHz. This line said 12 MHz for a day,
 *                           changed on 2026-09-28 on an 11.95 MHz reading taken
 *                           on the board that had failed to enumerate - a part
 *                           that is not this one. See the block below.
 *   LED on PC13, active low
 *   Console on USART2 PA2/PA3 at 115200 8N1, alternate function 7
 *
 * The crystal has now been checked against the board, by the board's own
 * output. The LED and the console pins have not. If the LED never lights, that
 * pin is the first suspect; if the banner is empty on the console it has not
 * been flashed on real hardware since this file was written instead.
 */

/* The crystal on PH0/PH1, which is the first fact in the list above. It is a
 * macro because one place in this file has to name a crystal - the banner's
 * fallback, for when clk.c's measurement did not fire - and the value here has
 * been wrong in both directions, so the reasoning is worth keeping.
 *
 * This file said 8 MHz. On 2026-09-28 `clk.c` gained a measurement (TIM11 timing
 * the crystal against HSI) and a boot that ran it read 11.95 MHz, so the line
 * was changed to 12 and the 8 was recorded as the stale constant that had held
 * the part at 252 MHz and USB at 72. The 8 was right and the 12 was the
 * mistake: 11.95 MHz is not this board's crystal. The board that failed to
 * enumerate was a 12 MHz part running an image built for an 8 MHz one, which is
 * the same 252 MHz arithmetic with the two boards swapped. Nothing in the tree
 * records that reading's own provenance - `board.c` prints the crystal in whole
 * megahertz, so no flash of this firmware could have shown 11.95 MHz - and the
 * UID was not read on 2026-09-27 or 2026-09-28, so the two boards are told
 * apart by their arithmetic and not by their serial numbers.
 *
 * What settled it is the board's own output on 2026-09-29: `banner` read
 * `HSE 8 MHz x PLL, apb1 42 MHz, apb2 84 MHz` at `168 MHz sysclk`, on an image
 * that runs `PLLM = 8`; USB enumerated on it, and 48 MHz at PLLQ=7 with PLLM=8
 * is HSE x 6, so HSE is 8 MHz and not 12. The 168 MHz is not the firmware's own
 * claim about itself: the host timed 30.000 s against 30,000 board
 * milliseconds, so the assumed sysclk is the real one.
 *
 * The `HSE 8 MHz` in that banner is the measurement firing, and it is worth
 * being precise about what that is worth: `build_clock_summary()` rounds to
 * whole megahertz, so it cannot distinguish a reading of 7.75 from one of 8.00
 * and it cannot corroborate the number above - only the USB corner and the
 * host-timed sysclk can. No instrument has measured this crystal. A counter
 * pointed at PH0/PH1 would settle it directly and has not been done. */
#define AK_BOARD_HSE_MHZ         8u

#define AK_BOARD_LED_PIN         AK_PIN(GPIOC_BASE, 13)
#define AK_BOARD_LED_ACTIVE_LOW  1

#define AK_BOARD_CONSOLE_USART   USART2_BASE
#define AK_BOARD_CONSOLE_AF      7
#define AK_BOARD_CONSOLE_TX      AK_PIN(GPIOA_BASE, 2)
#define AK_BOARD_CONSOLE_RX      AK_PIN(GPIOA_BASE, 3)
#define AK_BOARD_CONSOLE_BAUD    115200u

/* Outputs. The timer decides the pin, so these are not a free choice: motors
 * are TIM3 channels 1-4 and the servos are TIM2 channels 1-2. None of the six
 * is the console, the LED or USB, and all six are ordinary header pins here.
 *
 * This is a bare dev board, so every channel a timer drives reaches a pin - the
 * count `ak_board_output_shape()` reports is the timer's, and it is the same
 * board file that would say otherwise if a pad were missing. The Feather is the
 * board where that is not true.
 *
 * The servos are the one output the *board* chooses and not the timer: the
 * motor channels are TIM3's and are the arch layer's, because one DMA burst
 * writes all four of them, but servo PWM is ordinary PWM and its timer is
 * passed in. TIM2 channels 1-2 on PA0/PA1 is this board's answer, and it is the
 * pair the arch layer used to reach for by itself before the Feather arrived -
 * whose breakout brings out neither pad.
 *
 * Whether a scope can reach them, and whether a real ESC accepts the DShot
 * timing, is a bench question rather than a build one. */
#define AK_BOARD_MOTOR1_PIN AK_PIN(GPIOA_BASE, 6) /* TIM3_CH1, the burst trigger */
#define AK_BOARD_MOTOR2_PIN AK_PIN(GPIOA_BASE, 7) /* TIM3_CH2 */
#define AK_BOARD_MOTOR3_PIN AK_PIN(GPIOB_BASE, 0) /* TIM3_CH3 */
#define AK_BOARD_MOTOR4_PIN AK_PIN(GPIOB_BASE, 1) /* TIM3_CH4 */
#define AK_BOARD_SERVO_TIMER TIM2_BASE            /* both pads below are its */
#define AK_BOARD_SERVO1_PIN AK_PIN(GPIOA_BASE, 0) /* TIM2_CH1 */
#define AK_BOARD_SERVO2_PIN AK_PIN(GPIOA_BASE, 1) /* TIM2_CH2 */

/* Receiver. CRSF runs at 420000 baud 8N1 and needs a UART of its own, so the
 * console keeps USART2 and the receiver gets USART1 on PA10 (RX) and PA9 (TX -
 * CRSF telemetry out, which is the same wire a Crossfire receiver forwards to
 * the handset). SBUS uses the same port at 100000 baud 8E2.
 *
 * SBUS is *inverted* - it idles low and starts with a high bit - and the F405's
 * USART cannot invert its own pins, which is a part limit rather than a
 * firmware one. The signal needs one transistor (or a spare gate on the board)
 * between the receiver's SBUS pad and PA10. AerialKit does not claim hardware
 * it does not have: with no inverter fitted, `rc` says so and the counts show
 * framing errors rather than a plausible wrong baud rate. (Some receivers -
 * ELRS boards and a few FrSky ones - have an un-inverted SBUS pad as well,
 * which needs none of this.) */
#define AK_BOARD_RC_USART   USART1_BASE
#define AK_BOARD_RC_AF      7
#define AK_BOARD_RC_TX      AK_PIN(GPIOA_BASE, 9)
#define AK_BOARD_RC_RX      AK_PIN(GPIOA_BASE, 10)
#define AK_BOARD_RC_BAUD    420000u
#define AK_BOARD_RC_SBUS_BAUD 100000u
#define AK_BOARD_RC_INVERTER  0 /* no transistor between the receiver and PA10 */

/* GPS on USART3 (PB10 transmit, PB11 receive, AF7) at 9600 baud: the rate a
 * u-blox module speaks out of the box. Configuring it up to 115200 is a later
 * step, and it needs a UBX configuration frame going the other way. */
#define AK_BOARD_GPS_USART  USART3_BASE
#define AK_BOARD_GPS_AF     7
#define AK_BOARD_GPS_TX     AK_PIN(GPIOB_BASE, 10)
#define AK_BOARD_GPS_RX     AK_PIN(GPIOB_BASE, 11)
#define AK_BOARD_GPS_BAUD   9600u

/* IMU bus: SPI2 on PB13/PB14/PB15 with the chip select on PB12, driven by
 * hand. The InvenSense parts want mode 3 and a chip select the master owns,
 * which is what arch/stm32f405/spi.c does.
 *
 * There is no IMU on the bench board, so this exists to be ready and testable:
 * `spi` on the console writes a pattern with MOSI jumpered to MISO and reads it
 * back, which is the one way to check the port without a sensor. */
#define AK_BOARD_IMU_SPI     SPI2_BASE
#define AK_BOARD_IMU_AF      5
#define AK_BOARD_IMU_SCK     AK_PIN(GPIOB_BASE, 13)
#define AK_BOARD_IMU_MISO    AK_PIN(GPIOB_BASE, 14)
#define AK_BOARD_IMU_MOSI    AK_PIN(GPIOB_BASE, 15)
#define AK_BOARD_IMU_CS      AK_PIN(GPIOB_BASE, 12)

/*
 * Barometer: I2C1 on PB6 (SCL) and PB7 (SDA), 400 kHz, at address 0x77.
 *
 * I2C rather than SPI because that is where the barometer is on the wing's own
 * board - the GHF435 AIO puts its baro on I2C2 and the same is true of most
 * AIOs - so this is the arrangement AerialKit will actually meet. PB6/PB7 are
 * free here: they are not a timer channel the outputs want, not a UART, and
 * not the SPI the IMU is on.
 *
 * The address is a strap on the part (SDO high is 0x77 on a DPS310 and on a
 * BMP280), which is why it is a board fact rather than something probed: one
 * bus, one address, and a part that answers to the other one needs this line
 * changed. Nothing is fitted on the bench board, so the bus is not handed to
 * the core at all - a probe that cannot answer would otherwise be a boot that
 * waits on a stuck bus.
 */
#define AK_BOARD_BARO_I2C      I2C1_BASE
#define AK_BOARD_BARO_SCL      AK_PIN(GPIOB_BASE, 6)
#define AK_BOARD_BARO_SDA      AK_PIN(GPIOB_BASE, 7)
#define AK_BOARD_BARO_AF       4
#define AK_BOARD_BARO_SPEED    400000u
#define AK_BOARD_BARO_ADDRESS  0x77u
/* The "is it there" flags are guarded so a *build* can set them as well as a
 * hand editing this file: `make EXTRA_CFLAGS=-DAK_BOARD_BARO_FITTED=1` compiles
 * the same board with the part fitted, which is how the fitted half of this
 * file gets built at all (scripts/fw build checks both, see docs/02-hardware.md
 * and docs/17-esp32-port.md). Without the guard the define below wins and a
 * build that asked for a fitted board quietly gets the bare one. */
#ifndef AK_BOARD_BARO_FITTED
#define AK_BOARD_BARO_FITTED   0 /* solder one to PB6/PB7 and set this to 1 */
#endif

/*
 * Rangefinder: the same I2C1 pair, at the address a TOF10120 answers to.
 *
 * Two devices on one bus is what I2C is for, and it is why this is a second
 * bus with its own address rather than a second port: PB6/PB7 are already
 * configured by the barometer's init whether or not anything is fitted to
 * them, so a rangefinder is a part, a wire to the same two pins and this line.
 *
 * The strap is the part's own - 0x52 is what a TOF10120 answers to out of the
 * box - and it is a board fact rather than something probed for the same
 * reason the barometer's is: one bus, one address, and a part that has been
 * reprogrammed to another one needs this line changed rather than a scan.
 *
 * Nothing is fitted on the bench board, so the bus is not handed to the core
 * at all.
 */
#define AK_BOARD_RANGE_I2C      I2C1_BASE
#define AK_BOARD_RANGE_ADDRESS  0x52u
#define AK_BOARD_RANGE_FITTED   0 /* wire one to PB6/PB7 and set this to 1 */

/*
 * Battery voltage: ADC1 input 10 on PC0, behind a 10k/1k divider.
 *
 * PC0 is the pin because it is the first one left: not a timer channel the
 * outputs want, not a UART, not the SPI the sensors are on, and not the LED.
 * The divider is 10k from the pack's positive terminal to PC0 and 1k from PC0
 * to ground, which is 11 volts of pack for every volt at the pin - a 6S lands
 * at 2.3 volts, below the 3.3 volt reference with room to spare. The bottom
 * resistor is also what makes an unplugged pack read as zero rather than as
 * whatever the pin happened to be holding.
 *
 * Nothing is fitted on the bench board, so the pin floats and the reading
 * means nothing. That is a *board* fact and it is written down as one, below,
 * rather than left to the arithmetic: a floating pin can read a perfectly
 * plausible 1.5 volts, which the core would multiply by eleven and report as a
 * healthy 4S. The ADC is configured either way, so `battery` still shows the
 * counts and the conversion - the part that proves the register work - and the
 * only step left when the resistors go in is this one line.
 */
#define AK_BOARD_VBAT_ADC        ADC1_BASE
#define AK_BOARD_VBAT_CHANNEL    10u
#define AK_BOARD_VBAT_PIN        AK_PIN(GPIOC_BASE, 0)
#ifndef AK_BOARD_VBAT_FITTED
#define AK_BOARD_VBAT_FITTED     0 /* the 10k/1k pair is not soldered in yet */
#endif
/* The reference is the analog supply, which on this board is the 3.3 volt
 * rail - a nominal, not a measurement. A real reference is worth checking
 * against a multimeter once, and the correction it needs is what the
 * `vbat_ratio` parameter is for. */
#define AK_BOARD_VBAT_VREF       3.3f
#define AK_BOARD_VBAT_FULL_SCALE 4095u /* 12 bits */

/* USB: PA11 is D- and PA12 is D+, on the board's own connector, and alternate
 * function 10 is the OTG FS controller. This is the console's second road -
 * the one that works with no adapter on PA2/PA3, which is the goal's M0
 * "prints a banner over USB". */
#define AK_BOARD_USB_DM AK_PIN(GPIOA_BASE, 11)
#define AK_BOARD_USB_DP AK_PIN(GPIOA_BASE, 12)

#endif /* AK_BOARD_AERIALKIT_F405_H */
