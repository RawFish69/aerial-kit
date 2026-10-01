#ifndef AK_BOARD_AERIALKIT_GHF435_H
#define AK_BOARD_AERIALKIT_GHF435_H

#include "arch.h"

/*
 * The arch this board is built against, for the Makefile to read - see the
 * "three variables, two of them wrong together" block at the top of the
 * Makefile, which is where the check lives.
 *
 * `make BOARD=AERIALKIT_GHF435` on its own leaves ARCH at its default of
 * stm32f405, and the first thing that happens then is this board compiled
 * against the STM32F405 headers. It stops with three errors, and none of them
 * names ARCH:
 *
 *   board.c:229:44: error: 'AK_FLASH_PAGE_BYTES' undeclared here (not in a
 *                   function); did you mean 'AK_FLASHLOG_SLOT_BYTES'?
 *   board.c:240:16: error: expression in static assertion is not an integer
 *   board.h:237:39: error: 'GPIOH_BASE' undeclared (first use in this
 *                   function); did you mean 'GPIOA_BASE'?
 *
 * - the last of them in the middle of the barometer's pin definitions, so all
 * three read as defects in this board's own files. They are not: the board
 * builds, with ARCH=at32f435 PART=at32f435rg, which is the invocation
 * docs/28-build.md gives. This was recorded as "does not build on main" on
 * 2026-09-28 with the first and third of those lines quoted as the evidence,
 * and the next person to read that record reproduces exactly these lines and
 * believes it. One line here and the mismatch is refused by name instead.
 *
 * The other two ARM boards carry the same line so that the pairing is checked in
 * both directions - `BOARD=AERIALKIT_F405 ARCH=at32f435` is the one that gets
 * furthest, because the F405's board file compiles against the AT32 headers and
 * the failure is `undefined reference to ak_flash_erase_sector` at link.
 */
#define AK_BOARD_ARCH at32f435

/*
 * One board per firmware, and two of them in the host test binary.
 *
 * `tests/test_arch_at32.c` drives this board file the way `tests/test_arch.c`
 * drives the F405's - the console, the configuration record in this part's
 * flash, the log's regions, the barometer's bus, the receiver's pins - and both
 * board files cannot define `ak_board_init()` at once. So this one's entry
 * points are renamed for the host build alone, exactly as this part's arch
 * entry points are (see src/arch/at32f435/arch.h): the firmware build never
 * defines AK_HOST_AT32, so the core calls the plain names it expects.
 *
 * The list is long because the board contract is: it is the price of the test
 * binary linking two parts, and it is cheaper than a second copy of the test
 * set compiled for this part alone.
 */
#ifdef AK_HOST_AT32
#define ak_board_init                  ak_ghf435_board_init
#define ak_board_name                  ak_ghf435_board_name
#define ak_board_default_airframe      ak_ghf435_default_airframe
#define ak_board_led_set               ak_ghf435_led_set
#define ak_board_led_toggle            ak_ghf435_led_toggle
#define ak_board_led_state             ak_ghf435_led_state
#define ak_board_boot_mark             ak_ghf435_board_boot_mark
#define ak_board_clock_summary         ak_ghf435_clock_summary
#define ak_board_config_read           ak_ghf435_config_read
#define ak_board_config_write          ak_ghf435_config_write
#define ak_board_console_poll_rx       ak_ghf435_console_poll_rx
#define ak_board_net_ready             ak_ghf435_net_ready
#define ak_board_net_connected         ak_ghf435_net_connected
#define ak_board_net_poll_rx           ak_ghf435_net_poll_rx
#define ak_board_net_write             ak_ghf435_net_write
#define ak_board_net_report            ak_ghf435_net_report
#define ak_board_net_start             ak_ghf435_net_start
#define ak_board_param_table           ak_ghf435_param_table
#define ak_board_retained_ram          ak_ghf435_retained_ram
#define ak_board_log_store             ak_ghf435_log_store
#define ak_board_reboot                ak_ghf435_reboot
#define ak_board_enter_bootloader      ak_ghf435_enter_bootloader
#define ak_board_console_port          ak_ghf435_console_port
#define ak_board_console_attached_port ak_ghf435_console_attached_port
#define ak_board_clock_ok              ak_ghf435_clock_ok
#define ak_board_clock_sysclk_hz       ak_ghf435_clock_sysclk_hz
#define ak_board_clock_apb1_hz         ak_ghf435_clock_apb1_hz
#define ak_board_output_dshot_hz       ak_ghf435_output_dshot_hz
#define ak_board_output_dshot_period   ak_ghf435_output_dshot_period
#define ak_board_output_ccr_zero       ak_ghf435_output_ccr_zero
#define ak_board_output_ccr_one        ak_ghf435_output_ccr_one
#define ak_board_output_frames_sent    ak_ghf435_output_frames_sent
#define ak_board_output_init           ak_ghf435_output_init
#define ak_board_output_ready          ak_ghf435_output_ready
#define ak_board_output_shape          ak_ghf435_output_shape
#define ak_board_output_write          ak_ghf435_output_write
#define ak_board_output_set_rate       ak_ghf435_output_set_rate
#define ak_board_output_report         ak_ghf435_output_report
#define ak_board_rc_init               ak_ghf435_rc_init
#define ak_board_rc_set_protocol       ak_ghf435_rc_set_protocol
#define ak_board_rc_inverted           ak_ghf435_rc_inverted
#define ak_board_rc_send               ak_ghf435_rc_send
#define ak_board_rc_poll               ak_ghf435_rc_poll
#define ak_board_rc_dropped            ak_ghf435_rc_dropped
#define ak_board_gps_init              ak_ghf435_gps_init
#define ak_board_gps_poll              ak_ghf435_gps_poll
#define ak_board_gps_dropped           ak_ghf435_gps_dropped
#define ak_board_gps_send              ak_ghf435_gps_send
#define ak_board_imu_init              ak_ghf435_imu_init
#define ak_board_imu_bus               ak_ghf435_imu_bus
#define ak_board_baro_bus              ak_ghf435_baro_bus
#define ak_board_range_bus             ak_ghf435_range_bus
#define ak_board_spi_loopback          ak_ghf435_spi_loopback
#define ak_board_battery_init          ak_ghf435_battery_init
#define ak_board_battery_ready         ak_ghf435_battery_ready
#define ak_board_battery_pin_volts     ak_ghf435_battery_pin_volts
#define ak_board_battery_report        ak_ghf435_battery_report
#endif

/*
 * And the contract itself, read *after* the block above on purpose: the
 * declarations in `ak_board.h` are the ones that have to end up naming what
 * this build defines, and a header that declared the plain names first would
 * leave every call here undeclared. In the firmware build the block above is
 * inert and this is an ordinary include.
 */
#include "ak_board.h"

/*
 * Board: JHEMCU GHF435 AIO V2 - the twin-motor elevon wing's own flight
 * controller, and the third part AerialKit runs on.
 *
 * It is not an STM32. The chip is an Artery **AT32F435** (INAV builds this
 * board as `TWINWINGS_GHF435V2` for the AT32F435RGT7), so the port beneath this
 * file is src/arch/at32f435/ rather than src/arch/stm32f405/, and every pin
 * below is configured through that part's scheme - a per-pin mode, a function
 * number in one of two mux registers, and a drive strength of its own.
 *
 * **Where these facts come from.** The reference target in this workspace,
 * `upstream/inav-9.1.0/src/main/target/TWINWINGS_GHF435V2/target.h` at the
 * pinned revision, and the pin-function table that target's drivers are built
 * from, `src/main/drivers/timer_def_at32f43x.h`. Two of them were verified on
 * this physical board before this file existed, and they are marked below,
 * because they are the two that a manufacturer's drawing gets wrong:
 *
 *   - **the receiver is on USART2, not USART1.** A bound transmitter produced
 *     CRSF channel data on USART2 and nothing at all on USART1.
 *   - **the GPS port has its transmit and receive pins swapped in the
 *     peripheral**, which is why ak_board_gps_init() uses the AT32's swap bit
 *     rather than the pin pair the drawing names.
 *
 * Everything else here is read from that target and has not been measured on
 * the board: the LED, the sensor buses, the output timers and the pack's
 * divider. Each is one line to correct, and the preflight report is what
 * distinguishes "absent" from "broken" until then.
 *
 * Two things on this board are deliberately not driven by this file yet, and
 * they are gaps rather than ports: the **MAX7456 OSD** on SPI2 and the
 * **M25P16 blackbox flash** on SPI3. AerialKit's log goes to the internal
 * flash's free pages, so neither is between this firmware and a first flight.
 */

/* **The oscillator: an 8 MHz can, and on this part it is a declaration and not
 * a measurement.** The F405 port measures its crystal at boot and does not
 * assume one, because that board turned out to carry 12 MHz where the code said
 * 8. This part has no way to do that - every timer input it can route an
 * external clock to is listed in `at32f435_437_tmr.h`'s `tmr_input_remap_type`
 * and none of them is HEXT, and the clock unit's own trims belong to the internal
 * RC, calibrated *from* the crystal rather than measuring it. So the board says
 * what is soldered to it and the PLL is built from that:
 * clk.c derives the PLL's ms/ns/fr from this number rather than carrying three
 * literals that assume one crystal.
 *
 * The value is Artery's own for this family (`HEXT_VALUE` in
 * at32f435_437_conf.h) and it is what this board's stock firmware runs: INAV
 * builds TWINWINGS_GHF435V2 for 288 MHz SYSCLK with USB at 48, which is
 * (8 / 1) * 72 / 2 and cannot be reached from any other crystal this part's
 * PLL ranges admit. Switching to a 12 or 16 MHz can would still land (the
 * arithmetic is in clk.c and the host test drives those), and a 25 MHz one
 * would not - USB here is this PLL over six, so a crystal that cannot make
 * exactly 288 MHz exactly is a crystal this firmware will not run USB from.
 *
 * It is read by `board.c` and passed to `ak_clk_init()`, which is where the
 * arch layer learns it: this port's arch files do not include a board header,
 * for the same reason they do not know a pin map. */
#define AK_BOARD_HEXT_HZ 8000000u

/* LED on PC13, lit by pulling the pin low - the same part number and the same
 * sense the stock AIO target uses. */
#define AK_BOARD_LED_PIN        AK_PIN(GPIOC_BASE, 13)
#define AK_BOARD_LED_ACTIVE_LOW 1

/* Console on USART1 (PA9 transmit, PA10 receive, function 7) at 115200 8N1.
 *
 * USART1 is the port this board's own target leaves *unassigned*: the
 * manufacturing drawing puts the onboard receiver here and the live board says
 * otherwise (see below), so it is the one UART with nothing on it - which makes
 * it the console. The USB device below is the second door, and the one that
 * needs no adapter: this board is on a USB port whenever it is being flashed. */
#define AK_BOARD_CONSOLE_USART USART1_BASE
#define AK_BOARD_CONSOLE_AF    7
#define AK_BOARD_CONSOLE_TX    AK_PIN(GPIOA_BASE, 9)
#define AK_BOARD_CONSOLE_RX    AK_PIN(GPIOA_BASE, 10)
#define AK_BOARD_CONSOLE_BAUD  115200u

/* USB: PA11 is D- and PA12 is D+, function 10, on the board's own USB
 * connector - the OTG FS transceiver this part shares with the STM32F405
 * (src/arch/at32f435/usb.c is that port's driver, with this part's clock and
 * three registers that differ). The board's target enables it as its VCP, which
 * is how this board's own INAV build gets a console.
 *
 * This is what makes a first flash readable: the board is on the Pi's USB port
 * to be programmed, and with the USB device up the same cable gives
 * `/dev/ttyACM0` and a banner instead of a UART adapter on PA9/PA10. */
#define AK_BOARD_USB_DM AK_PIN(GPIOA_BASE, 11)
#define AK_BOARD_USB_DP AK_PIN(GPIOA_BASE, 12)

/* Outputs. On this part the timer decides the pin, as on the F405 - but the
 * timers are different ones and they run at 288 MHz rather than 84, which is
 * the number a copy of the F405's output code would get wrong. Both channels
 * below are the wing's: two motors and two elevon servos.
 *
 *   motors  PB6, PB7   TMR4 channels 1 and 2, function 2, DShot
 *   servos  PB8, PB9   TMR2 channels 1 and 2, function 1, 50 Hz PWM
 *
 * PB8/PB9 are UART5's pads on the manufacturer's drawing; the wing's target
 * gives UART5 up on purpose so the servos can have TMR2, which is the same
 * trade this file makes. The firmware's output driver holds the pin facts
 * itself (src/arch/at32f435/output.c), because a timer channel is not a free
 * choice; they are written here as well so that the two cannot drift apart
 * silently. */
#define AK_BOARD_MOTOR1_PIN AK_PIN(GPIOB_BASE, 6) /* TMR4_CH1 */
#define AK_BOARD_MOTOR2_PIN AK_PIN(GPIOB_BASE, 7) /* TMR4_CH2 */
#define AK_BOARD_SERVO1_PIN AK_PIN(GPIOB_BASE, 8) /* TMR2_CH1 */
#define AK_BOARD_SERVO2_PIN AK_PIN(GPIOB_BASE, 9) /* TMR2_CH2 */

/* Receiver: the **onboard ELRS receiver**, on USART2, verified live - a bound
 * transmitter produces CRSF channel data here and produced nothing on USART1
 * whatever the drawing says. The two pins take different function numbers,
 * which is a property of this part's mux registers: receive on PB0 is function
 * 6 and transmit on PA8 is function 8.
 *
 * CRSF is 420000 baud 8N1. SBUS would be 100000 baud 8E2 on the same wire, and
 * it would need an inverting transistor in front of PB0: this part has the
 * swap bit (see the GPS below) but no invert bit, so SBUS is the F405's problem
 * here too and the port says so rather than pretending. The onboard receiver
 * speaks CRSF, which is what makes this board's default a receiver and not a
 * wire. */
#define AK_BOARD_RC_USART      USART2_BASE
#define AK_BOARD_RC_TX         AK_PIN(GPIOA_BASE, 8)
#define AK_BOARD_RC_TX_AF      8
#define AK_BOARD_RC_RX         AK_PIN(GPIOB_BASE, 0)
#define AK_BOARD_RC_RX_AF      6
#define AK_BOARD_RC_BAUD       420000u
#define AK_BOARD_RC_SBUS_BAUD  100000u
#define AK_BOARD_RC_INVERTER   0 /* no transistor between the receiver and PB0 */

/* GPS on USART3: PB11 transmits and PB10 receives *after* the peripheral's
 * swap, which is the pair the live board runs (Betaflight's resource map for
 * this board has SERIAL_TX 3 = B11 and SERIAL_RX 3 = B10, while the drawing
 * names them the other way round). Without the swap the firmware would drive
 * PB10, which this board presents as receive - two outputs onto one wire.
 *
 * 9600 baud is what a u-blox module speaks out of the box; raising it needs a
 * UBX configuration frame going back the other way, which is its own step and
 * already exists in the core. */
#define AK_BOARD_GPS_USART     USART3_BASE
#define AK_BOARD_GPS_AF        7
#define AK_BOARD_GPS_TX        AK_PIN(GPIOB_BASE, 11)
#define AK_BOARD_GPS_RX        AK_PIN(GPIOB_BASE, 10)
#define AK_BOARD_GPS_BAUD      9600u

/* IMU bus: SPI1 on PA5/PA6/PA7 with the chip select on PA4, driven by hand,
 * function 5 on all four. The part on the board is an ICM-42688P, detected
 * live, and the driver for it already exists in src/core/sensors/ - so this is
 * the board whose IMU is a fact rather than a hope, unlike the bench F405.
 *
 * The orientation is *not* a fact yet. The reference target carries CW90 for
 * the ICM while the Betaflight target for the same board says CW180 plus a
 * board yaw of -45 degrees, and those cannot both be right. It is a parameter
 * (align_board_*) that has to be measured on the aircraft, not copied - which
 * is why nothing here claims a number for it. */
#define AK_BOARD_IMU_SPI     SPI1_BASE
#define AK_BOARD_IMU_AF      5
#define AK_BOARD_IMU_SCK     AK_PIN(GPIOA_BASE, 5)
#define AK_BOARD_IMU_MISO    AK_PIN(GPIOA_BASE, 6)
#define AK_BOARD_IMU_MOSI    AK_PIN(GPIOA_BASE, 7)
#define AK_BOARD_IMU_CS      AK_PIN(GPIOA_BASE, 4)
#define AK_BOARD_IMU_FITTED  1 /* an ICM-42688P, detected live on this board */

/* Barometer: I2C2 on PH2 (SCL) and PH3 (SDA), function 4, 400 kHz, at 0x77.
 * A DPS310 is fitted - AerialKit has had its driver since the F405 port, and
 * it is the same driver here.
 *
 * The pins are configured at boot whether or not the part is fitted, for the
 * same reason the F405 does it: fitting or removing a barometer is then a line
 * rather than a bring-up. The transfers behind it are this part's own
 * (src/arch/at32f435/i2c.c) and are not the F405's: the address, the direction
 * and the byte count go into one register and the peripheral performs the
 * address phase, the acknowledge and the stop itself. */
#define AK_BOARD_BARO_I2C      I2C2_BASE
#define AK_BOARD_BARO_SCL      AK_PIN(GPIOH_BASE, 2)
#define AK_BOARD_BARO_SDA      AK_PIN(GPIOH_BASE, 3)
#define AK_BOARD_BARO_AF       4
#define AK_BOARD_BARO_SPEED    400000u
#define AK_BOARD_BARO_ADDRESS  0x77u
#define AK_BOARD_BARO_FITTED   1 /* a DPS310, according to the reference target */

/* Rangefinder: the same two wires and the address a TOF10120 answers to.
 * Nothing is fitted to this board, so the bus is not handed over. */
#define AK_BOARD_RANGE_I2C     I2C2_BASE
#define AK_BOARD_RANGE_ADDRESS 0x52u
#define AK_BOARD_RANGE_FITTED  0

/* Battery voltage: ADC1 input 1 on PA0, which is where this board's own
 * divider is (the reference target calls it ADC_CHANNEL_1_PIN). PA1 is the
 * current shunt's channel and is not used yet: AerialKit has no current
 * parameter, and a number nobody can act on is worse than a gap that is
 * written down.
 *
 * **The divider is fitted** - this is an AIO with a battery lead, and it
 * measures its own pack - but its ratio is *not* known here, and it matters:
 * the board reports volts at the pin and the core multiplies by `vbat_ratio`,
 * whose default is the F405 bench board's 10k/1k. A multimeter across a pack
 * and the `vbat_ratio` line is how that gets set, and until it has been, the
 * voltage on the console is the right shape and the wrong scale.
 *
 * The reference is the analog supply, the 3.3 volt rail: a nominal, not a
 * measurement. */
#define AK_BOARD_VBAT_ADC        ADC1_BASE
#define AK_BOARD_VBAT_CHANNEL    1u
#define AK_BOARD_VBAT_PIN        AK_PIN(GPIOA_BASE, 0)
#define AK_BOARD_VBAT_FITTED     1 /* on the board: a divider, not a pad */
#define AK_BOARD_VBAT_VREF       3.3f
#define AK_BOARD_VBAT_FULL_SCALE 4095u /* 12 bits */

/* The beeper on PC15 and the LED strip are on the board and not driven: a
 * beeper AerialKit has no pattern for, and a strip that the reference target
 * disables for bring-up as well. Neither is between this board and a first
 * flight, and both are one line away when something wants them. */

#endif /* AK_BOARD_AERIALKIT_GHF435_H */
