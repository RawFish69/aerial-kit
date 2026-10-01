#include "board.h"

#include "ak_board.h"
#include "ak_console.h"
#include "ak_dshot_timing.h"
#include "ak_log.h"
#include "ak_params.h"
#include "ak_version.h"

#include "esp_attr.h"

static char clock_summary[64];

void ak_board_init(void)
{
    ak_esp_console_init(AK_BOARD_CONSOLE_BAUD);
    /* Before anything prints: if the chip restarted because it panicked, the
     * boot report should say so. */
    ak_esp_note_reset_reason();
    /* The port is what the arch layer will report; attaching it here is what
     * makes console output and the preflight check agree. */
    ak_console_attach(AK_BOARD_CONSOLE_UART);
    ak_esp_led_init(AK_BOARD_LED_GPIO);
    /* The network is *not* started here. Its credentials are parameters, so it
     * waits for the saved configuration to be loaded into them - the core
     * calls ak_board_net_start() once that has happened. Starting it here would
     * be a board that comes up on last build's network, or on none. */

    /* No PLL setup and no crystal to check: IDF has already brought the clock
     * up, and this is where a port says what it ended up with. */
    const char *prefix = "CPU ";
    unsigned at = 0;
    while (*prefix != '\0') {
        clock_summary[at++] = *prefix++;
    }
    uint32_t mhz = ak_esp_cpu_mhz();
    char digits[4];
    unsigned n = 0;
    if (mhz >= 100u) {
        digits[n++] = (char)('0' + (mhz / 100u) % 10u);
    }
    if (mhz >= 10u) {
        digits[n++] = (char)('0' + (mhz / 10u) % 10u);
    }
    digits[n++] = (char)('0' + mhz % 10u);
    for (unsigned i = 0; i < n; i++) {
        clock_summary[at++] = digits[i];
    }
    const char *suffix = " MHz";
    while (*suffix != '\0') {
        clock_summary[at++] = *suffix++;
    }
    clock_summary[at] = '\0';
}

const char *ak_board_name(void)
{
    return AK_BOARD_STR " / ESP32-S3-DevKitC-1";
}

/*
 * Quad-X: the core's default, and what this board has always flown. Said out
 * loud rather than left out, because the entry point is the board contract and
 * a board that cannot fly the default has to be able to say so - see the note
 * in ak_board.h, and src/boards/FEATHER_F405/board.c, which is the board that
 * does.
 */
uint32_t ak_board_default_airframe(void)
{
    return 0u;
}

void ak_board_led_set(int on)
{
    ak_esp_led_set(on);
}

void ak_board_led_toggle(void)
{
    ak_esp_led_set(!ak_esp_led_state());
}

int ak_board_led_state(void)
{
    return ak_esp_led_state();
}

const char *ak_board_clock_summary(void)
{
    return clock_summary;
}

int ak_board_clock_ok(void)
{
    /* IDF reports a real CPU frequency; there is no crystal to fall back from. */
    return ak_esp_cpu_mhz() > 0u;
}

uint32_t ak_board_clock_sysclk_hz(void)
{
    return ak_esp_cpu_mhz() * 1000000u;
}

uint32_t ak_board_clock_apb1_hz(void)
{
    return ak_esp_cpu_mhz() * 1000000u;
}

int ak_board_config_read(void *buf, uint32_t len)
{
    return ak_esp_config_read(buf, len);
}

int ak_board_config_write(const void *buf, uint32_t len)
{
    return ak_esp_config_write(buf, len);
}

int ak_board_console_poll_rx(char *byte)
{
    return ak_esp_console_poll_rx((uint8_t *)byte);
}

uint32_t ak_board_console_port(void)
{
    return AK_BOARD_CONSOLE_UART;
}

uint32_t ak_board_console_attached_port(void)
{
    return ak_console_port();
}

void ak_board_reboot(void)
{
    ak_arch_reset();
}

/*
 * And the DFU path, which this part does not have: its bootloader lives in ROM
 * and is entered by the straps at reset (or by `idf.py`, which drives them over
 * the serial port), not by anything the firmware can jump to. So `dfu` on the
 * console says this board cannot rather than pretending - which is a different
 * sentence from a board whose jump is broken, and the console prints the one
 * that is true.
 */
int ak_board_enter_bootloader(void)
{
    return 0;
}

/* --- outputs --------------------------------------------------------------
 *
 * DShot and servo PWM, out of the RMT and the LEDC: the chip half is in
 * src/arch/esp32/output.c and these are the board contract's thin wrappers
 * over it, the same shape as the network functions above. Whether the
 * channels came up is the driver's answer and not this file's guess - the
 * preflight report has a line for "absent" and a line for "present but
 * wrong", and a port that conflates them teaches its reader to ignore it. */

/* As long as the board says it drives - the chip is what limits that,
 * and a board that asks for more than it has would be driving pins
 * that do not exist. */
static const int output_motor_gpios[AK_BOARD_MOTORS] = {
    AK_BOARD_MOTOR1_GPIO, AK_BOARD_MOTOR2_GPIO,
    AK_BOARD_MOTOR3_GPIO, AK_BOARD_MOTOR4_GPIO,
};
static const int output_servo_gpios[AK_MAX_SERVOS] = {
    AK_BOARD_SERVO1_GPIO, AK_BOARD_SERVO2_GPIO,
};

int ak_board_output_ready(void) { return ak_esp_output_ready(); }
/* What this board drives, from one place: the board header's own counts. */
void ak_board_output_shape(unsigned *motors, unsigned *servos)
{
    *motors = AK_BOARD_MOTORS;
    *servos = AK_BOARD_SERVOS;
}
void ak_board_output_init(void)
{
    ak_esp_output_init(output_motor_gpios, AK_BOARD_MOTORS,
                       output_servo_gpios, AK_BOARD_SERVOS);
}
void ak_board_output_write(const ak_output_frame_t *frame) { ak_esp_output_write(frame); }
void ak_board_output_set_rate(uint32_t khz) { ak_esp_output_set_rate(khz); }
void ak_board_output_report(ak_printf_fn out) { ak_esp_output_report(out); }
uint32_t ak_board_output_dshot_hz(void) { return ak_esp_output_dshot_hz(); }
uint32_t ak_board_output_dshot_period(void) { return ak_esp_output_dshot_period(); }
uint32_t ak_board_output_ccr_zero(void) { return ak_esp_output_ccr_zero(); }
uint32_t ak_board_output_ccr_one(void) { return ak_esp_output_ccr_one(); }
uint32_t ak_board_output_frames_sent(void) { return ak_esp_output_frames_sent(); }

/* --- the receiver and the GPS ---------------------------------------------
 *
 * Both are UARTs on this chip, and the chip half is in src/arch/esp32/uart.c:
 * the board names the port, the pins and the baud, which is all a pin map is.
 * The line settings for the receiver follow the protocol the core selected -
 * CRSF and SBUS do not even share a baud rate - and this chip inverts SBUS in
 * its GPIO matrix rather than asking for a transistor the way the F405's board
 * file has to. */

void ak_board_rc_init(void)
{
    ak_esp_rc_init(AK_BOARD_RC_UART, AK_BOARD_RC_TX_GPIO, AK_BOARD_RC_RX_GPIO,
                   AK_BOARD_RC_BAUD, AK_BOARD_RC_SBUS_BAUD);
}

void ak_board_rc_set_protocol(uint32_t protocol)
{
    ak_esp_rc_set_protocol(protocol);
}

int ak_board_rc_poll(uint8_t *byte) { return ak_esp_rc_poll(byte); }
uint32_t ak_board_rc_dropped(void) { return ak_esp_rc_dropped(); }
int ak_board_rc_inverted(void) { return ak_esp_rc_inverted(); }
int ak_board_rc_send(const char *data, unsigned len)
{
    return ak_esp_rc_send(data, len);
}

void ak_board_gps_init(void)
{
    ak_esp_gps_init(AK_BOARD_GPS_UART, AK_BOARD_GPS_TX_GPIO,
                    AK_BOARD_GPS_RX_GPIO, AK_BOARD_GPS_BAUD);
}

int ak_board_gps_poll(uint8_t *byte) { return ak_esp_gps_poll(byte); }
uint32_t ak_board_gps_dropped(void) { return ak_esp_gps_dropped(); }

int ak_board_gps_send(const char *data, unsigned len)
{
    return ak_esp_gps_send(data, len);
}

/* The network, which on this target is the point of the target: the port
 * brings up a netif and a listening socket, and the core serves its config
 * protocol and its telemetry over whatever is connected. */
int ak_board_net_ready(void) { return ak_esp_net_ready(); }
int ak_board_net_connected(void) { return ak_esp_net_connected(); }
int ak_board_net_poll_rx(char *byte) { return ak_esp_net_poll_rx((uint8_t *)byte); }
void ak_board_net_write(const char *data, unsigned len)
{
    (void)ak_esp_net_write((const uint8_t *)data, len);
}
void ak_board_net_report(ak_printf_fn out) { ak_esp_net_report(out); }

/* Called by the core once the saved configuration has been loaded, because
 * what the radio joins - or whether it carries the network itself - is in the
 * parameters. A board that cannot get an address still has to boot, print and
 * fly, so a failure here is a line on the console rather than a stop. */
void ak_board_net_start(void) { ak_esp_net_start(); }

/* The Wi-Fi credentials are the board's own parameters, and they go in the one
 * table the console, the config protocol and `save` all use. */
unsigned ak_board_param_table(ak_param_t *items, unsigned count)
{
    return ak_esp_net_params(items, count);
}

/*
 * Retained RAM: the ESP32 keeps a slice of RTC slow memory across a reset, so
 * the long log survives one here too - smaller, because there are eight
 * kilobytes of it and the port asks for a shorter ring. The capacity is a
 * build option for exactly this reason; the log checks it on resume, so a ring
 * left by a build with a different one is thrown away rather than misread.
 */
static RTC_NOINIT_ATTR ak_log_t retained_log;

void *ak_board_retained_ram(unsigned *bytes)
{
    *bytes = sizeof retained_log;
    return &retained_log;
}

/* The blackbox in flash: the partition this build reserved for it, which is
 * ports/esp32/partitions.csv's `aerialkit` region. A build whose table has no
 * such partition gets 0 here, and the console says why. */
const ak_flashlog_store_t *ak_board_log_store(void)
{
    return ak_esp_log_store();
}

/* --- the sensor buses -----------------------------------------------------
 *
 * Two transports, one interface: the drivers in src/core/sensors ask for a
 * register and never learn which wire answered, which is what makes the same
 * barometer driver run on an STM32 with a hand-written I2C master and on this
 * chip with IDF's. The chip halves are in src/arch/esp32/spi.c and i2c.c.
 *
 * The difference between the two buses here is deliberate and it is about what
 * can be *learned at boot*:
 *
 * - The IMU's bus is declared (AK_BOARD_IMU_FITTED). Finding out whether a
 *   sensor is there means probing a part with a driver, and a probe on a dead
 *   bus costs a timeout per attempt - a slow way to learn what one line
 *   already says.
 * - The barometer's bus is asked. A device address on I2C is a bounded
 *   question with three possible answers, and all three are worth different
 *   lines on the console: something answered, nothing answered, or the bus
 *   itself did not come up. A flag cannot tell those apart, and a flag that is
 *   wrong is worse than a boot that takes fifty milliseconds.
 */

static const ak_bus_t *baro_bus;
static const ak_bus_t *imu_bus;

static void range_bus_init(void);

void ak_board_imu_init(void)
{
    int answered;

    if (AK_BOARD_IMU_FITTED) {
        imu_bus = ak_esp_spi_imu_bus(AK_BOARD_IMU_SCK_GPIO,
                                     AK_BOARD_IMU_MISO_GPIO,
                                     AK_BOARD_IMU_MOSI_GPIO,
                                     AK_BOARD_IMU_CS_GPIO);
    }

    /* The I2C bus is configured here whether or not a part is fitted, for the
     * same reason the F405 configures its barometer pins: fitting one is then
     * a part and a line of configuration rather than a bring-up. */
    answered = ak_esp_i2c_probe(AK_BOARD_BARO_PORT, AK_BOARD_BARO_SCL_GPIO,
                                AK_BOARD_BARO_SDA_GPIO, AK_BOARD_BARO_SPEED,
                                AK_BOARD_BARO_ADDRESS);
    if (answered == 1) {
        baro_bus = ak_esp_i2c_baro_bus(AK_BOARD_BARO_PORT,
                                       AK_BOARD_BARO_SCL_GPIO,
                                       AK_BOARD_BARO_SDA_GPIO,
                                       AK_BOARD_BARO_SPEED,
                                       AK_BOARD_BARO_ADDRESS);
        ak_console_printf("baro:      a device answered at 0x%02x on i2c%d\r\n",
                          (unsigned)AK_BOARD_BARO_ADDRESS, AK_BOARD_BARO_PORT);
    } else if (answered == 0) {
        ak_console_printf("baro:      i2c%d came up, nothing answered at "
                          "0x%02x - no barometer fitted\r\n",
                          AK_BOARD_BARO_PORT, (unsigned)AK_BOARD_BARO_ADDRESS);
    } else {
        ak_console_printf("baro:      i2c%d did not come up - no barometer "
                          "bus\r\n", AK_BOARD_BARO_PORT);
    }

    /* And the other part that lives on those two wires, on its own address. */
    range_bus_init();
}

const ak_bus_t *ak_board_imu_bus(void)
{
    return imu_bus;
}

const ak_bus_t *ak_board_baro_bus(void)
{
    return baro_bus;
}

/* The rangefinder on the same wires, asked the same way: this chip's I2C
 * driver answers "is anything at that address" inside a bounded time, so a
 * part that is not fitted is a line on the console rather than a boot spent
 * waiting for an acknowledgement that will not come. */
static const ak_bus_t *range_bus;

static void range_bus_init(void)
{
    int answered = ak_esp_i2c_probe(AK_BOARD_RANGE_PORT, AK_BOARD_RANGE_SCL_GPIO,
                                    AK_BOARD_RANGE_SDA_GPIO,
                                    AK_BOARD_RANGE_SPEED,
                                    AK_BOARD_RANGE_ADDRESS);
    if (answered == 1) {
        range_bus = ak_esp_i2c_baro_bus(AK_BOARD_RANGE_PORT,
                                        AK_BOARD_RANGE_SCL_GPIO,
                                        AK_BOARD_RANGE_SDA_GPIO,
                                        AK_BOARD_RANGE_SPEED,
                                        AK_BOARD_RANGE_ADDRESS);
        ak_console_printf("range:     a device answered at 0x%02x on i2c%d\r\n",
                          (unsigned)AK_BOARD_RANGE_ADDRESS,
                          AK_BOARD_RANGE_PORT);
    }
}

const ak_bus_t *ak_board_range_bus(void)
{
    return range_bus;
}

void ak_board_spi_loopback(ak_printf_fn out)
{
    ak_esp_spi_loopback(AK_BOARD_IMU_SCK_GPIO, AK_BOARD_IMU_MISO_GPIO,
                        AK_BOARD_IMU_MOSI_GPIO, out);
}

/*
 * The flight pack, on the converter this chip has.
 *
 * The core's contract is "volts at your pin": the divider ratio, the cell
 * count and the thresholds are the same code on both chips, and this is the
 * half that is about this chip - a channel, an attenuation and the calibration
 * in eFuse that turns counts into volts (src/arch/esp32/adc.c).
 *
 * Nothing is fitted on a bare devkit, and the answer to that is *not* zero
 * volts: a floating pin reads something plausible, which the core would
 * multiply by the divider and call a healthy pack. So the board says "not
 * fitted" for as long as `AK_BOARD_VBAT_FITTED` is 0, and a read that cannot
 * be taken comes back negative - which the core reads as "no reading" and not
 * as a flat pack.
 */
static int vbat_ready;

void ak_board_battery_init(void)
{
    vbat_ready = AK_BOARD_VBAT_FITTED &&
                 ak_esp_adc_init(AK_BOARD_VBAT_UNIT,
                                 AK_BOARD_VBAT_CHANNEL) == 0;
}

int ak_board_battery_ready(void) { return vbat_ready; }

float ak_board_battery_pin_volts(void)
{
    int mv;

    if (!vbat_ready) {
        return -1.0f;
    }
    mv = ak_esp_adc_mv(AK_BOARD_VBAT_UNIT, AK_BOARD_VBAT_CHANNEL);
    return mv < 0 ? -1.0f : (float)mv / 1000.0f;
}

void ak_board_battery_report(ak_printf_fn out)
{
    int mv;

    out("adc:       GPIO%d, ADC%d channel %d, 10k over 1k\n",
        AK_BOARD_VBAT_GPIO, AK_BOARD_VBAT_UNIT, AK_BOARD_VBAT_CHANNEL);
    if (!vbat_ready) {
        out("           no divider fitted. Set AK_BOARD_VBAT_FITTED in\n"
            "           src/boards/ESP32DEV/board.h when the pair is in\n");
        return;
    }
    ak_esp_adc_report(out);
    mv = ak_esp_adc_mv(AK_BOARD_VBAT_UNIT, AK_BOARD_VBAT_CHANNEL);
    if (mv < 0) {
        out("raw:       no reading - a conversion that did not finish, not an\n"
            "           empty battery\n");
        return;
    }
    out("raw:       %d.%03d V at the pin\n", mv / 1000, mv % 1000);
}
