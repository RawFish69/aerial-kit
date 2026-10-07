#ifndef AK_BOARD_FEATHER_F405_H
#define AK_BOARD_FEATHER_F405_H

#include "arch.h"

/* The arch this board is built against - the same part as the WeAct board, so
 * the same arch. Declared for the check, not for the default: see the block at
 * the top of the Makefile. */
#define AK_BOARD_ARCH stm32f405

/*
 * One board per firmware, and two boards on one part in the host test binary.
 *
 * This is the Adafruit breakout next to the WeAct dev board, and they are the
 * same silicon: the arch layer is identical for both, which is the point of
 * the arch layer not including a board header. So the only file that differs
 * between them is this one - and two board files cannot both define
 * `ak_board_init()` in one binary. The entry points are therefore renamed for
 * the host build alone, exactly as the AT32's are in the sibling board file
 * and for the same reason: the firmware build never defines AK_HOST_FEATHER,
 * so the core calls the plain names it expects.
 *
 * The list is the board contract, and it is the same 56 names the wing's board
 * renames - a contract with a name missing is a linker error rather than a
 * silent one, which is what makes writing it out longhand acceptable.
 */
#ifdef AK_HOST_FEATHER
#define ak_board_init                  ak_feather_board_init
#define ak_board_name                  ak_feather_board_name
#define ak_board_default_airframe      ak_feather_default_airframe
#define ak_board_led_set               ak_feather_led_set
#define ak_board_led_toggle            ak_feather_led_toggle
#define ak_board_led_state             ak_feather_led_state
#define ak_board_boot_mark             ak_feather_board_boot_mark
#define ak_board_clock_summary         ak_feather_clock_summary
#define ak_board_config_read           ak_feather_config_read
#define ak_board_config_write          ak_feather_config_write
#define ak_board_console_poll_rx       ak_feather_console_poll_rx
#define ak_board_net_ready             ak_feather_net_ready
#define ak_board_net_connected         ak_feather_net_connected
#define ak_board_net_poll_rx           ak_feather_net_poll_rx
#define ak_board_net_write             ak_feather_net_write
#define ak_board_net_report            ak_feather_net_report
#define ak_board_net_start             ak_feather_net_start
#define ak_board_param_table           ak_feather_param_table
#define ak_board_retained_ram          ak_feather_retained_ram
/*
 * The long log's ring is the block ak_board_retained_ram() returns, and on this
 * board that is RAM the startup code does not clear: it survives a reset and not
 * a power cycle, which is what the boot report's "from the run before" means.
 * See ak_board.h for what the core does with the answer.
 */
#define AK_BOARD_LOG_RETAINED 1
#define ak_board_log_store             ak_feather_log_store
#define ak_board_reboot                ak_feather_reboot
#define ak_board_enter_bootloader      ak_feather_enter_bootloader
#define ak_board_console_port          ak_feather_console_port
#define ak_board_console_attached_port ak_feather_console_attached_port
#define ak_board_clock_ok              ak_feather_clock_ok
#define ak_board_clock_sysclk_hz       ak_feather_clock_sysclk_hz
#define ak_board_clock_apb1_hz         ak_feather_clock_apb1_hz
#define ak_board_output_dshot_hz       ak_feather_output_dshot_hz
#define ak_board_output_dshot_period   ak_feather_output_dshot_period
#define ak_board_output_ccr_zero       ak_feather_output_ccr_zero
#define ak_board_output_ccr_one        ak_feather_output_ccr_one
#define ak_board_output_frames_sent    ak_feather_output_frames_sent
#define ak_board_output_init           ak_feather_output_init
#define ak_board_output_ready          ak_feather_output_ready
#define ak_board_output_shape          ak_feather_output_shape
#define ak_board_output_write          ak_feather_output_write
#define ak_board_output_set_rate       ak_feather_output_set_rate
#define ak_board_output_report         ak_feather_output_report
#define ak_board_rc_init               ak_feather_rc_init
#define ak_board_rc_set_protocol       ak_feather_rc_set_protocol
#define ak_board_rc_inverted           ak_feather_rc_inverted
#define ak_board_rc_send               ak_feather_rc_send
#define ak_board_rc_poll               ak_feather_rc_poll
#define ak_board_rc_dropped            ak_feather_rc_dropped
#define ak_board_gps_init              ak_feather_gps_init
#define ak_board_gps_poll              ak_feather_gps_poll
#define ak_board_gps_dropped           ak_feather_gps_dropped
#define ak_board_gps_send              ak_feather_gps_send
#define ak_board_imu_init              ak_feather_imu_init
#define ak_board_imu_bus               ak_feather_imu_bus
#define ak_board_baro_bus              ak_feather_baro_bus
#define ak_board_range_bus             ak_feather_range_bus
#define ak_board_spi_loopback          ak_feather_spi_loopback
#define ak_board_battery_init          ak_feather_battery_init
#define ak_board_battery_ready         ak_feather_battery_ready
#define ak_board_battery_pin_volts     ak_feather_battery_pin_volts
#define ak_board_battery_report        ak_feather_battery_report
#endif

/*
 * Board: Adafruit Feather STM32F405 Express (STM32F405RGT6, 1 MB flash).
 *
 * The same part as the WeAct board with a different breakout, which is what
 * makes this port interesting: the silicon's limits are identical and the
 * *header* is not, so every fact below is a fact about which pads Adafruit
 * brought out. The pin map is not from memory - it is from the board's own
 * variant file in ST's Arduino core,
 *
 *   framework-arduinoststm32, variants/STM32F4xx/F405RGT_F415RGT/
 *   variant_FEATHER_F405.h and PeripheralPins_FEATHER_F405.c
 *
 * which lists every broken-out pad and the alternate function each carries.
 * Where this file says a pin is *not* available, that file simply does not
 * define it.
 *
 * Three of this board's facts shape the whole port and each is worth reading
 * before changing anything:
 *
 *   1. **The HSE is 12 MHz** - and so is the WeAct board's. This line said
 *      "not the WeAct's 8" until 2026-09-29, which was the 8 MHz that board's
 *      own header had assumed and has since retracted: at 8 MHz the F405 asks
 *      its VCO for 504 MHz, past the part's 432 MHz limit, which is exactly
 *      why that board never enumerated and exactly what a copy of its file
 *      would have done here. The same file says `HSE_VALUE 12000000U`, and the
 *      schematic puts a 12 MHz crystal on PH0/PH1. The clock layer's PLL
 *      divides HSE by PLLM to reach a 1 MHz reference, so M=12 is what this
 *      board needs; a 12 MHz crystal with the old M=8 asks the VCO for the
 *      same 504 MHz and the part does not politely refuse - it locks outside
 *      its own datasheet and runs at 252 MHz with USB at 72, which is what the
 *      WeAct board measured on 2026-09-27, and not the HSI fallback this line
 *      predicted until 2026-09-29. See
 *      src/arch/stm32f405/clk.c, which measures the crystal at boot and sets
 *      PLLM from it, so the define below records the fact rather than sets it.
 *
 *   2. **The motor timer is not the WeAct's, and four of its six pads do not
 *      reach the header.** Motors 3 and 4 are TIM3 CH3/CH4 on PB0/PB1, and
 *      neither pad is in the variant file; only PA6 and PA7 (Feather pins A2 and
 *      A3) come out, so two motor channels are usable rather than four. The
 *      servos were the same story until 2026-09-29: they sat on the arch's
 *      TIM2 CH1/CH2 pads, PA0 and PA1, and *neither is broken out either* - so
 *      the board drove pulses into two pads with nothing on them. They are now
 *      TIM4 CH3/CH4 on PB8 and PB9 (Feather pins 9 and 10), which the variant
 *      file does list, and the servo bank is a board fact handed to
 *      `ak_output_init()` rather than the arch's own choice - see the note above
 *      AK_BOARD_SERVO_TIMER. `ak_board_output_shape()` reports what can actually
 *      be plugged in: two motors end to end, two servos.
 *
 *   3. **Only two UARTs can be used, and one of them only as the console.**
 *      This target has USART1 on PA9/PA10, and both of those pads are tied to
 *      +3V3 on the board - Adafruit's own variant file lists PA9/PA10 nowhere.
 *      Of the rest, the arch layer's receive path has interrupt handlers for
 *      USART1 and USART3 only (src/arch/stm32f405/usart.c), and the console's
 *      receive path is *polled* while the receiver's is not. So the receiver
 *      takes USART3 - the one interrupt-capable port left, and the board's own
 *      `Serial` - and the console takes USART6, which needs no handler because
 *      it never asks for one. A GPS has nowhere left to go; there is no third
 *      UART, exactly as on the ESP32-C3 devkit.
 *
 * Nothing in this file has been checked against the board in hand. The two
 * that will announce themselves are the LED and the console: if the status pin
 * never lights, PC1 is the first suspect, and if the banner is empty on PB10
 * it was never flashed rather than this file being wrong about the pin.
 */

/* PC1, and *active high* - the schematic drives it through a 2.2k resistor to
 * the LED's anode and the cathode is grounded, so a driven-high pad lights it.
 * The WeAct board is the other way round on PC13, which is why the polarity is
 * a board fact and not a constant. */
#define AK_BOARD_LED_PIN         AK_PIN(GPIOC_BASE, 1)
#define AK_BOARD_LED_ACTIVE_LOW  0

/* The crystal on PH0/PH1, as this board's file declares it: 12 / 12 = 1 MHz
 * reference, x 336 = 336 MHz VCO, / 2 = 168 MHz, / 7 = 48 MHz for USB.
 *
 * The 12 is **declared and not measured**, and it is worth being exact about
 * where it comes from, because the crystal line has been wrong on two other
 * boards in this tree and a reader arriving from either will want to know
 * whether this is the third. It is not a copy of the value
 * `AERIALKIT_F405/board.h` carried for a day and retracted (traps 212, 213):
 * this file said 12 in its first version, `f50a048`, before that one did, and
 * its stated basis is Adafruit's own variant file and schematic - the block at
 * the top of this file quotes the first of those, `HSE_VALUE 12000000U`, and
 * neither is carried in this repository, so the 12 can be read here but not
 * re-checked here. What is missing for this line is a measurement of *this*
 * board: the honest sentence `f50a048` wrote about this file - "Nothing in
 * this file has been checked against the board in hand" - still stands for the
 * one row that sounds like a reading.
 *
 * **And the F405's correction does not transfer here.** That board's crystal is
 * 8 MHz and its 12 was wrong; this board's 12 has a different source, so
 * "the WeAct turned out to be 8" is not a reason to change this line to 8. It
 * is the same mistake in the other direction, and it is the one traps 206 and
 * 212 are about: a number moved between two boards because they looked alike.
 * What is known here is that 12 rests on a document, not on an instrument.
 *
 * What that costs is smaller than it sounds, and the direction matters. This
 * macro is read in exactly one place - the banner's fallback in `board.c`, for
 * the boot where `clk.c`'s measurement did not fire - and `pll_m_for()` in
 * `clk.c` takes PLLM from the *measurement*, not from here. So a wrong number
 * here cannot put the part on the wrong clock tree; it can only mislabel a
 * banner on a board whose crystal failed to start, which is the one board a
 * person is reading that line to diagnose. The banner says `, assumed` when it
 * uses this value, so the two are told apart in the text.
 *
 * To settle it: the measured branch of that same line on hardware, or a
 * counter on PH0/PH1. `tests/test_board_feather.c` can only reach the fallback
 * - a process has no TIM11 capture to fire - so it pins the fallback and says
 * so, which is not the same as confirming this number. */
#define AK_BOARD_HSE_MHZ         12u

/* Console on USART6, PC6 (TX) and PC7 (RX), AF8. Fifth choice and the only one
 * left: USART1's pads are tied to +3V3, USART2 is on PA2/PA3 where PA2 is not
 * broken out and PA3 is the battery divider, and USART3 is the receiver's for
 * the reason in the header. USART6 needs no interrupt because the console's
 * receive path polls - see ak_board_console_poll_rx(). */
#define AK_BOARD_CONSOLE_USART   USART6_BASE
#define AK_BOARD_CONSOLE_AF      8
#define AK_BOARD_CONSOLE_TX      AK_PIN(GPIOC_BASE, 6)
#define AK_BOARD_CONSOLE_RX      AK_PIN(GPIOC_BASE, 7)
#define AK_BOARD_CONSOLE_BAUD    115200u

/* Outputs. The motors are TIM3 channels 1-4, as on the WeAct board and for the
 * same reason - the DShot burst is one timer and there is no second one to move
 * it to - so two of the four pads are on this header and two are not:
 *
 *   PA6 (A2) TIM3_CH1 - the burst trigger, and an ESC
 *   PA7 (A3) TIM3_CH2 - an ESC
 *   PB0, PB1 (TIM3_CH3/4) - pads on the package that this breakout does not
 *   bring out.
 *
 * The servos are the board's own choice and not the arch's, which is what
 * AK_BOARD_SERVO_TIMER is: TIM4 channels 3-4 on PB8/PB9, the pair the variant
 * file lists as pins 9 and 10 and the only free timer pads on the header. See
 * board.c for the table and the shape. */
#define AK_BOARD_MOTOR1_PIN AK_PIN(GPIOA_BASE, 6) /* TIM3_CH1, the burst trigger */
#define AK_BOARD_MOTOR2_PIN AK_PIN(GPIOA_BASE, 7) /* TIM3_CH2 */

/* TIM4 rather than the arch's default TIM2: TIM2's pads are PA0/PA1 and this
 * breakout brings out neither. TIM4_CH3 is PB8 and TIM4_CH4 is PB9, both in
 * Adafruit's variant file at AF2, and both free - the console is USART6 on
 * PC6/PC7, the receiver USART3 on PB10/PB11, and the Qwiic I2C1 is PB6/PB7. */
#define AK_BOARD_SERVO_TIMER TIM4_BASE
#define AK_BOARD_SERVO1_PIN  AK_PIN(GPIOB_BASE, 8) /* TIM4_CH3, Feather pin 9 */
#define AK_BOARD_SERVO2_PIN  AK_PIN(GPIOB_BASE, 9) /* TIM4_CH4, Feather pin 10 */

/* Receiver: CRSF at 420000 8N1 and SBUS at 100000 8E2 on USART3, PB10 (TX)
 * and PB11 (RX), AF7 - the board's own `Serial` pads and, with USART1 gone,
 * the only interrupt-capable port the console does not own.
 *
 * SBUS is inverted and the F405 cannot invert its own pins, so it needs a
 * transistor between the receiver and PB11, exactly as on the WeAct board.
 * With no inverter fitted, `rc` says so and the counts show framing errors
 * rather than a plausible wrong baud rate. */
#define AK_BOARD_RC_USART   USART3_BASE
#define AK_BOARD_RC_AF      7
#define AK_BOARD_RC_TX      AK_PIN(GPIOB_BASE, 10)
#define AK_BOARD_RC_RX      AK_PIN(GPIOB_BASE, 11)
#define AK_BOARD_RC_BAUD    420000u
#define AK_BOARD_RC_SBUS_BAUD 100000u
#define AK_BOARD_RC_INVERTER  0 /* no transistor between the receiver and PB11 */

/* No GPS section: this breakout brings out two UARTs that can be used, one is
 * the receiver and the other is the console, and the arch layer's receive path
 * has no handler for a third even if a pad existed. The board file answers the
 * whole GPS contract with no-ops, which is the same shape the ESP32-C3 devkit
 * uses for the same reason. */

/*
 * IMU bus: **I2C1** on PB6 (SCL) and PB7 (SDA), 400 kHz, AF4, at one of four
 * addresses, probed at boot.
 *
 * This is the port's reason for existing. Every other IMU in this repository
 * is a chip on a SPI bus; this one is a breakout on the Feather's Qwiic
 * connector, which is these two pads with 10k pull-ups already on the board.
 * Two parts are supported there, and which one is fitted is a question the
 * board answers by asking rather than a header constant:
 *
 *   - **Bosch BNO055** (src/core/sensors/ak_imu_bno055.c) - the fixed wing's
 *     IMU. 0x28 with its ADR pin low, 0x29 with it high. Which of the two the
 *     breakout in hand is strapped to was an open question on 2026-10-05, and
 *     probing both settles it on the board's first boot: the console's `imu:`
 *     line prints the address that answered.
 *   - **ST LSM6DSO** (src/core/sensors/ak_imu_lsm6dso.c) - the part the port
 *     was written against. 0x6B with SA0 high, 0x6A with it low.
 *
 * The LSM6DSO's address was a single fact here until 2026-10-05, and that fact
 * had already been wrong once: the line said 0x6A and the first scan of the
 * real bus, on 2026-09-30, found the part at 0x6B. The firmware never spoke to
 * it and reported `nothing answered on the bus`, which was exactly true. The
 * line ended by saying that a board fitted with a second part would be the
 * moment to make the strap a probed field, and a BNO055 whose strap nobody had
 * read is that moment.
 *
 * The probe is an address scan, not a part probe: the first of these four to
 * acknowledge is the one the bus talks to, and the core's who-am-i then decides
 * which driver it is. The order is the aircraft's preference - the BNO055 is
 * the fixed wing's IMU, so with both breakouts on the chain it is the one
 * flown. A BNO055 does not acknowledge at all for ~650 ms after power-up while
 * it boots, so a scan that finds nobody waits and tries again, up to
 * AK_BOARD_IMU_PROBE_MS; that is the whole cost on a board with no IMU fitted,
 * and it ends in `nothing answered on the bus` exactly as before.
 *
 * Unlike the barometer below, this bus is handed to the core unconditionally:
 * the IMU is what this board is for. */
#define AK_BOARD_IMU_I2C      I2C1_BASE
#define AK_BOARD_IMU_SCL      AK_PIN(GPIOB_BASE, 6)
#define AK_BOARD_IMU_SDA      AK_PIN(GPIOB_BASE, 7)
#define AK_BOARD_IMU_AF       4
#define AK_BOARD_IMU_SPEED    400000u
/* In probe order. */
#define AK_BOARD_IMU_BNO055_ADDRESS      0x28u
#define AK_BOARD_IMU_BNO055_ALT_ADDRESS  0x29u
#define AK_BOARD_IMU_LSM6DSO_ADDRESS     0x6Bu /* measured on the bench part */
#define AK_BOARD_IMU_LSM6DSO_ALT_ADDRESS 0x6Au
#define AK_BOARD_IMU_PROBE_MS            900u
#define AK_BOARD_IMU_PROBE_STEP_MS       10u

/*
 * Barometer: the same I2C1 pair the IMU is on, at 0x77.
 *
 * Two parts on one bus is what I2C is for, and on this board the pair is
 * already up for the IMU - so a barometer is a second part on the Qwiic chain
 * (or two wires soldered to the same pads) and one line here. The address is
 * the part's own strap: SDO high is 0x77 on a DPS310 and on a BMP280.
 *
 * Nothing is fitted, so the bus is not handed to the core at all - a probe
 * that cannot answer is a boot that waits on a stuck bus.
 */
#define AK_BOARD_BARO_I2C      I2C1_BASE
#define AK_BOARD_BARO_SCL      AK_PIN(GPIOB_BASE, 6)
#define AK_BOARD_BARO_SDA      AK_PIN(GPIOB_BASE, 7)
#define AK_BOARD_BARO_AF       4
#define AK_BOARD_BARO_SPEED    400000u
#define AK_BOARD_BARO_ADDRESS  0x77u
/* Guarded so a build can set it as well as a hand: `make
 * EXTRA_CFLAGS=-DAK_BOARD_BARO_FITTED=1` compiles the same board with the part
 * fitted, which is how the fitted half of this file gets built at all. */
#ifndef AK_BOARD_BARO_FITTED
#define AK_BOARD_BARO_FITTED   0 /* add one to the Qwiic chain and set this to 1 */
#endif

/* Rangefinder: the same pair again, at the address a TOF10120 answers to. The
 * pins are already configured by the IMU's init, so this is an address and
 * nothing else. Nothing is fitted, so the bus is not handed over. */
#define AK_BOARD_RANGE_I2C      I2C1_BASE
#define AK_BOARD_RANGE_ADDRESS  0x52u
#define AK_BOARD_RANGE_FITTED   0 /* wire one to PB6/PB7 and set this to 1 */

/*
 * Battery voltage: ADC1 input 3 on PA3.
 *
 * PA3 is the pad Adafruit's own variant file labels `VDIV`, so there is a
 * divider on the board and this is where it lands - which is why the pin and
 * the channel are stated and not guessed. What is *not* known is the ratio:
 * the variant file names the pad and not the resistors, and the ratio and the
 * cell count are the core's parameters (`vbat_ratio`, `vbat_cells`) rather
 * than this file's. So the flag below stays off until someone puts a
 * multimeter on it.
 *
 * That is the honest direction, and it matters: a divider that is there but
 * unmeasured, multiplied by whatever ratio a default happens to be, reports a
 * confident voltage that is wrong. The ADC is configured either way, so
 * `battery` still shows the counts and proves the register work; the one line
 * below is the whole of what a measurement changes.
 */
#define AK_BOARD_VBAT_ADC        ADC1_BASE
#define AK_BOARD_VBAT_CHANNEL    3u
#define AK_BOARD_VBAT_PIN        AK_PIN(GPIOA_BASE, 3)
#ifndef AK_BOARD_VBAT_FITTED
#define AK_BOARD_VBAT_FITTED     0 /* labelled VDIV on the board; ratio unmeasured */
#endif
#define AK_BOARD_VBAT_VREF       3.3f
#define AK_BOARD_VBAT_FULL_SCALE 4095u /* 12 bits */

/* USB: PA11 is D- and PA12 is D+, on the board's own connector - the same two
 * pads as the WeAct board, since it is the same part. This is the console's
 * second road and the one a laptop already has a cable for. */
#define AK_BOARD_USB_DM AK_PIN(GPIOA_BASE, 11)
#define AK_BOARD_USB_DP AK_PIN(GPIOA_BASE, 12)

#endif /* AK_BOARD_FEATHER_F405_H */
