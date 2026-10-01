#ifndef AK_BOARD_ESP32S3DEV_H
#define AK_BOARD_ESP32S3DEV_H

#include "esp.h"

/*
 * Board: an ESP32-S3-DevKitC-1, the third chip in this family to get a board
 * file - and the second one that can be *run* on this machine, because
 * Espressif's QEMU has an esp32s3 machine alongside the esp32 one.
 *
 * The differences from the WROOM board are the ones the chip and the devkit
 * have: a different console UART's default pins (43/44 rather than IDF's
 * ESP32 defaults), the USB-Serial-JTAG peripheral on 19 and 20 which this file
 * therefore leaves alone, flash and PSRAM on 26-32, and one thing worth
 * knowing before somebody reports "the LED does not work": **the DevKitC-1's
 * on-board LED is an addressable WS2812 on GPIO48**, not a plain LED, so the
 * boot's three blinks and the heartbeat need a plain LED wired to the pin
 * below rather than the part Espressif fitted. Driving it as a GPIO is what
 * the port does; lighting the RGB part is a driver this project has not
 * written.
 *
 * The output pins below are the one thing here that is a *guess*, and they are
 * a guess in a specific way: they are ordinary GPIOs on a devkit, four for
 * motors and two for servos, chosen because nothing else in this file uses
 * them. They are one line each to change when there is a board to change them
 * to - which is what the goal's "a board with a name" means, and why the
 * preflight prints what the outputs ended up being rather than what they were
 * meant to be.
 *
 * Everything else reports itself absent, which is what the preflight check
 * exists to distinguish from broken.
 */

#define AK_BOARD_LED_GPIO 48 /* a plain LED wired here; the DevKitC-1's own is a WS2812 */

#define AK_BOARD_CONSOLE_UART AK_ESP_CONSOLE_UART()
#define AK_BOARD_CONSOLE_BAUD 115200u

/* Outputs. DShot goes out of the RMT peripheral, one channel per motor, and
 * the servos are LEDC channels: neither choice is a pin's, unlike the F405
 * where the timer decided. */
/* How many outputs this chip can actually drive, which is a *chip* fact before
 * it is a board one: DShot here is one RMT transmit channel per motor, and
 * this ESP32 has eight of them against the ESP32-C3's two. The C3's board file
 * says two for exactly that reason, and the preflight's mix-versus-board line
 * is what tells its owner a quadrotor needs four. */
#define AK_BOARD_MOTORS 4u
#define AK_BOARD_SERVOS 2u

#define AK_BOARD_MOTOR1_GPIO 4
#define AK_BOARD_MOTOR2_GPIO 5
#define AK_BOARD_MOTOR3_GPIO 6
#define AK_BOARD_MOTOR4_GPIO 7
#define AK_BOARD_SERVO1_GPIO 15
#define AK_BOARD_SERVO2_GPIO 16

/* Receiver. CRSF at 420000 8N1 and SBUS at 100000 8E2 share one port: the
 * core owns the choice - it is a parameter - and the port follows it. UART2's
 * pins are the free pair most devkits break out.
 *
 * SBUS is inverted, and this is where the second target differs from the
 * first in a way worth knowing: an F405 cannot invert its own receive line, so
 * its board file asks for a transistor. This chip inverts in its GPIO matrix,
 * so the port does it when the protocol is SBUS and the transistor is not
 * needed. That is why ak_board_rc_inverted() answers 1 here and 0 there. */
#define AK_BOARD_RC_UART     2
#define AK_BOARD_RC_TX_GPIO  17
#define AK_BOARD_RC_RX_GPIO  18
#define AK_BOARD_RC_BAUD     420000u
#define AK_BOARD_RC_SBUS_BAUD 100000u

/* GPS on UART1 at 9600 baud: the rate a u-blox module speaks out of the box.
 * RX on GPIO4 and TX on GPIO13, which is the pair the configuration frames
 * need when the module is moved to a faster rate later. */
#define AK_BOARD_GPS_UART    1
#define AK_BOARD_GPS_TX_GPIO 8
#define AK_BOARD_GPS_RX_GPIO 9
#define AK_BOARD_GPS_BAUD    9600u

/* IMU: SPI2, mode 3, 1 MHz to start with, on the devkit's usual SPI pins. The
 * chip select is the driver's, unlike the F405 where it is driven by hand.
 *
 * A devkit has no inertial sensor, so the bus is not handed to the core until
 * someone says one is fitted - a probe with nothing on the bus is not
 * dangerous, it is just a slow way of learning what this line already says.
 * `spi` on the console runs the loopback either way: one jumper from GPIO23 to
 * GPIO19 is what turns this from a pin map into a fact. */
#define AK_BOARD_IMU_SCK_GPIO  12
#define AK_BOARD_IMU_MISO_GPIO 13
#define AK_BOARD_IMU_MOSI_GPIO 11
#define AK_BOARD_IMU_CS_GPIO   10
/* Guarded, so a build can set them as well as a hand: the fitted half of this
 * file is the half a bare devkit never compiles, which is how the Wi-Fi half of
 * this port came to be broken for days. `scripts/esp32-proto.sh` builds this
 * board with the radio and all three parts fitted
 * (docs/17-esp32-port.md). */
#ifndef AK_BOARD_IMU_FITTED
#define AK_BOARD_IMU_FITTED    0 /* solder one on and set this to 1 */
#endif

/* Barometer: I2C0 at 400 kHz on the devkit's usual pair, address 0x77 (SDO
 * high on a DPS310 or a BMP280 - a strap on the part, which is why it is a
 * board fact rather than something to probe for).
 *
 * Unlike the IMU's bus, this one is *asked* rather than declared: this chip's
 * I2C driver answers "is anything at that address?" inside a bounded time, so
 * the board finds out at boot and prints what it found. A wire off, a part
 * with the other strap, or a dead bus are then three different lines on the
 * console instead of one silence. */
#define AK_BOARD_BARO_PORT     0
#define AK_BOARD_BARO_SCL_GPIO 38
#define AK_BOARD_BARO_SDA_GPIO 39
#define AK_BOARD_BARO_SPEED    400000u
#define AK_BOARD_BARO_ADDRESS  0x77u

/* Rangefinder: the same two wires and the same "ask the bus" answer, at the
 * address a TOF10120 answers to. It is a second bus on the same port for the
 * same reason it is on the F405: two parts on one bus is an address apart, and
 * a board may have either of them, both or neither. */
#define AK_BOARD_RANGE_PORT     0
#define AK_BOARD_RANGE_SCL_GPIO 38
#define AK_BOARD_RANGE_SDA_GPIO 39
#define AK_BOARD_RANGE_SPEED    400000u
#define AK_BOARD_RANGE_ADDRESS  0x52u

/*
 * Battery voltage: ADC1 channel 6, which is GPIO34.
 *
 * GPIO34 is an input-only pin with a converter channel on it, and it is the
 * one left: not a motor or a servo, not a UART, not the SPI or the I2C pair,
 * not the LED and not the console. The divider is the same 10k/1k the F405
 * board is drawn with - 11 volts of pack for every volt at the pin, which puts
 * a 6S at 2.3 volts, inside the widest attenuation this part has.
 *
 * Nothing is fitted on a bare devkit, so the pin floats - and a floating pin
 * reads something entirely plausible, which the core would multiply by eleven
 * and report as a healthy pack. That is why this is a board fact and not a
 * detection: set it to 1 when the resistors are in.
 */
#define AK_BOARD_VBAT_UNIT      1 /* ADC1 */
#define AK_BOARD_VBAT_CHANNEL   0 /* GPIO1 on this part */
#define AK_BOARD_VBAT_GPIO      1
#ifndef AK_BOARD_VBAT_FITTED
#define AK_BOARD_VBAT_FITTED    0 /* solder the 10k/1k pair in and set this to 1 */
#endif

#endif /* AK_BOARD_ESP32DEV_H */
