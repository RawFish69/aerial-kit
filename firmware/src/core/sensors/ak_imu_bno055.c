#include "ak_imu.h"
#include "ak_imu_bno055.h"

/*
 * The Bosch BNO055, over I2C.
 *
 * The fixed wing on the Adafruit Feather F405 carries one, on the Qwiic chain
 * beside (or instead of) the LSM6DSO the port was written against. It is an
 * unusual part to put behind an IMU interface, because it is three sensors and
 * a Cortex-M0 running Bosch's fusion on one die, and the obvious way to use it
 * is to ask that processor for an attitude. This driver deliberately does not,
 * and the reasons are worth writing down once:
 *
 *   - **The fusion output is 100 Hz.** The control loop runs at 1 kHz and its
 *     estimator wants gyro at that rate; an attitude that is up to ten loops
 *     old is a delay the rate loop would have to be detuned around.
 *   - **Fusion modes lock the sensors.** In IMU or NDOF mode the part chooses
 *     the gyro's range and filter and the accelerometer's range (4 g) and will
 *     not take ours. 4 g clips on a hard landing; a fixed range we did not pick
 *     is a fact about the aircraft we cannot see from the firmware.
 *   - **It would be a second estimator.** The firmware's own (ak_estimator.c)
 *     is the one the navigation, the preflight and the log all read, and is
 *     the one with tests behind it. Two attitudes that disagree is a worse
 *     failure than one attitude that is slightly worse.
 *
 * So the part runs in **AMG** - accelerometer, magnetometer and gyro, raw, no
 * fusion - and the firmware reads gyro and accelerometer like any other IMU.
 * The fusion processor's calibration and the power-on self test are still
 * readable (ak_imu_bno055.h) for the console, because "is the part alive" is a
 * question this part can answer about itself.
 *
 * Every register number and timing below is from Bosch's datasheet,
 * BST-BNO055-DS000, section 4 (register map) and table 3-6 (mode switching).
 */

/* Page 0. */
#define BNO055_CHIP_ID        0x00u
#define BNO055_CHIP_ID_VALUE  0xA0u
#define BNO055_PAGE_ID        0x07u
#define BNO055_ACC_DATA       0x08u /* accel x/y/z, mag x/y/z, gyro x/y/z */
#define BNO055_CALIB_STAT     0x35u
#define BNO055_ST_RESULT      0x36u
#define BNO055_SYS_STATUS     0x39u
#define BNO055_SYS_ERR        0x3Au
#define BNO055_UNIT_SEL       0x3Bu
#define BNO055_OPR_MODE       0x3Du
#define BNO055_PWR_MODE       0x3Eu
#define BNO055_SYS_TRIGGER    0x3Fu

/* Page 1: sensor configuration, only writable in CONFIG mode and only obeyed
 * outside the fusion modes - which is the second reason for AMG above. */
#define BNO055_ACC_CONFIG     0x08u
#define BNO055_GYR_CONFIG_0   0x0Au
#define BNO055_GYR_CONFIG_1   0x0Bu

#define BNO055_PWR_NORMAL     0x00u
#define BNO055_SYS_RST        0x20u

/* UNIT_SEL: accelerometer in mg (bit 0) and gyro in radians per second
 * (bit 1). The other bits (Euler angles, temperature, orientation convention)
 * describe outputs this driver does not read and are left at zero. */
#define BNO055_UNITS_MG_RPS   0x03u
#define BNO055_ACCEL_LSB_PER_G   1000.0f /* 1 LSB = 1 mg */
#define BNO055_GYRO_LSB_PER_RADS 900.0f  /* datasheet table 3-22 */

/* ACC_Config: normal power (bits 7:5 = 0), 250 Hz bandwidth (bits 4:2 = 5),
 * 16 g (bits 1:0 = 3). 16 g rather than 8: a fixed wing's landings are where
 * an accelerometer clips, and a clipped sample on the ground is one the
 * estimator would believe. The bandwidth is the accelerometer's own filter and
 * the estimator only uses this sensor slowly, so it is set well below the
 * 1000 Hz the part offers. */
#define BNO055_ACC_16G_250HZ  0x17u

/* GYR_Config_0: 523 Hz bandwidth (bits 5:3 = 0), 2000 dps (bits 2:0 = 0) -
 * the widest the part has on both, which is what every other gyro in this
 * repository is configured to, and the firmware's own filter chain does the
 * rest. GYR_Config_1 zero is normal power. */
#define BNO055_GYR_2000DPS_523HZ 0x00u
#define BNO055_GYR_NORMAL     0x00u

/* Table 3-6: from any mode into CONFIG takes 19 ms, out of CONFIG into any
 * other mode 7 ms. A reset is a full boot, 650 ms typical; the part does not
 * answer its own chip id until it is done, so the wait is a poll with a bound
 * rather than a fixed sleep. */
#define BNO055_TO_CONFIG_MS   25u /* the table's 19 ms, rounded up, as a delay
                                   * only where nothing is read back */
/* How long a mode switch's poll is allowed to go on, and how often it reads.
 * The bound is deliberately far past the table's 7 ms: the whole point of the
 * poll is to stop depending on a typical being the worst case, so a bound set
 * at the typical would put the same assumption back in the one place it was
 * removed from. The poll interval is 1 ms because the number it produces is
 * reported as a measurement of this part, and a 5 ms interval would report
 * every switch as "somewhere in the first five milliseconds". */
#define BNO055_MODE_POLL_MS   1u
#define BNO055_MODE_SETTLE_MS 250u
/* How long the part is given to come out of its power-on self test. The
 * datasheet's 650 ms reset figure is the part answering at all; the self test
 * runs after that and is what the mode write has to wait for. */
#define BNO055_BOOT_SETTLE_MS 2000u
#define BNO055_RESET_WAIT_MS  650u
#define BNO055_BOOT_POLL_MS   10u
#define BNO055_BOOT_POLLS     50u /* another 500 ms past the typical boot */
#define BNO055_WRITE_MS       1u

static int bno055_write_wait(const ak_bus_t *bus, uint8_t reg, uint8_t value,
                             unsigned delay_ms)
{
    if (ak_bus_write(bus, reg, value) != 0) {
        return -1;
    }
    ak_bus_delay_ms(bus, delay_ms);
    return 0;
}

/*
 * Wait for the part to answer its chip id, after a reset or a power-up.
 *
 * Reads that fail are not errors here: a BNO055 mid-boot does not acknowledge
 * its address at all, so a failed read is the expected answer for the first
 * few hundred milliseconds. Only running out of polls is a failure.
 */
static int bno055_wait_booted(const ak_bus_t *bus)
{
    for (unsigned i = 0; i < BNO055_BOOT_POLLS; i++) {
        uint8_t id = 0;
        if (ak_bus_read(bus, BNO055_CHIP_ID, &id, 1) == 0 &&
            id == BNO055_CHIP_ID_VALUE) {
            return 0;
        }
        ak_bus_delay_ms(bus, BNO055_BOOT_POLL_MS);
    }
    return -1;
}

int ak_bno055_read_mode(const ak_bus_t *bus, uint8_t *mode)
{
    uint8_t value = 0;
    if (ak_bus_read(bus, BNO055_OPR_MODE, &value, 1) != 0) {
        return -1;
    }
    *mode = (uint8_t)(value & 0x0Fu);
    return 0;
}

int ak_bno055_read_status(const ak_bus_t *bus, uint8_t *status, uint8_t *error)
{
    uint8_t value[2];
    if (ak_bus_read(bus, BNO055_SYS_STATUS, value, 2) != 0) {
        return -1;
    }
    *status = value[0];
    *error = value[1];
    return 0;
}

int ak_bno055_read_calib(const ak_bus_t *bus, ak_bno055_calib_t *calib)
{
    uint8_t value = 0;
    if (ak_bus_read(bus, BNO055_CALIB_STAT, &value, 1) != 0) {
        return -1;
    }
    calib->sys = (uint8_t)((value >> 6) & 0x03u);
    calib->gyro = (uint8_t)((value >> 4) & 0x03u);
    calib->accel = (uint8_t)((value >> 2) & 0x03u);
    calib->mag = (uint8_t)(value & 0x03u);
    return 0;
}

int ak_bno055_read_selftest(const ak_bus_t *bus, ak_bno055_selftest_t *st)
{
    uint8_t value = 0;
    if (ak_bus_read(bus, BNO055_ST_RESULT, &value, 1) != 0) {
        return -1;
    }
    st->accel = (uint8_t)(value & 0x01u);
    st->mag = (uint8_t)((value >> 1) & 0x01u);
    st->gyro = (uint8_t)((value >> 2) & 0x01u);
    st->mcu = (uint8_t)((value >> 3) & 0x01u);
    return 0;
}

/*
 * Wait for the part to report a given SYS_STATUS, bounded. Returns how long
 * that took in milliseconds, or -1 if it never did.
 *
 * SYS_STATUS and not OPR_MODE, which is the thing this driver was written
 * twice to learn. **OPR_MODE reads back what was last written to it the
 * moment it is written.** On the bench board a mode write made while the part
 * was still running its power-on self test read back as the new mode within
 * 1 ms - faster than the 7 ms table 3-6 gives for the switch - and the part
 * then finished booting into CONFIG with the command discarded: mode 0x0,
 * status 0, and zeros from every data register, with the driver's read-back
 * insisting the part was in AMG. SYS_STATUS is the register that says what the
 * part is actually doing, and its values are the ones to wait on: 4 while the
 * self test runs, 0 when the part is idle in CONFIG, 6 when it is running
 * without fusion, which is what AMG is.
 */
static int bno055_wait_status(const ak_bus_t *bus, uint8_t want,
                              unsigned settle_ms)
{
    for (unsigned waited = 0; waited <= settle_ms;
         waited += BNO055_MODE_POLL_MS) {
        uint8_t status = 0;
        uint8_t error = 0;
        ak_bus_delay_ms(bus, BNO055_MODE_POLL_MS);
        if (ak_bno055_read_status(bus, &status, &error) == 0 &&
            status == want) {
            return (int)(waited + BNO055_MODE_POLL_MS);
        }
    }
    return -1;
}

/*
 * Switch the part's operating mode, and wait for the part to *be* in it.
 *
 * Table 3-6 gives the switching time as 7 ms out of CONFIG and 19 ms into it,
 * and those are typicals, so this does not sleep for one: it writes the mode
 * and then waits on the status that mode produces, bounded. How long that took
 * is kept, because it is a real measurement of this part and the console
 * prints it (ak_bno055_last_switch_ms).
 */
static unsigned mode_switch_ms;

static int bno055_set_mode(const ak_bus_t *bus, uint8_t mode,
                           uint8_t want_status, unsigned settle_ms)
{
    if (ak_bus_write(bus, BNO055_OPR_MODE, mode) != 0) {
        return -1;
    }
    int took = bno055_wait_status(bus, want_status, settle_ms);
    if (took < 0) {
        return -1;
    }
    mode_switch_ms = (unsigned)took;
    return 0;
}

unsigned ak_bno055_last_switch_ms(void)
{
    return mode_switch_ms;
}

static int bno055_init(const ak_bus_t *bus, ak_printf_fn out)
{
    /*
     * Page 0 and CONFIG first, then a reset. The reset is for the same reason
     * the LSM6DSO's is: a dev board that has been running since its last power
     * cycle may have a mode, a page or a unit selection left by whatever spoke
     * to it before, and every write below assumes the reset values. The page
     * select comes first because SYS_TRIGGER is a page-0 register and a part
     * left on page 1 would take the reset as a write to something else.
     *
     * The CONFIG switch here is the table's delay and not a poll, and that is
     * deliberate: the reset in the next line makes whatever the part answers
     * now irrelevant, so there is nothing a read-back here would establish.
     * Every other mode switch below polls.
     */
    if (bno055_write_wait(bus, BNO055_PAGE_ID, 0x00u, BNO055_WRITE_MS) != 0 ||
        bno055_write_wait(bus, BNO055_OPR_MODE, AK_BNO055_MODE_CONFIG,
                          BNO055_TO_CONFIG_MS) != 0 ||
        bno055_write_wait(bus, BNO055_SYS_TRIGGER, BNO055_SYS_RST,
                          BNO055_RESET_WAIT_MS) != 0 ||
        bno055_wait_booted(bus) != 0) {
        return -1;
    }

    /*
     * The chip id answers before the part is finished booting. On the bench
     * board it answered 0xA0 with SYS_STATUS still reading 4 - "executing
     * self-test" - and every write made in that window was discarded when the
     * part came out of it, the mode command included. That is the fault this
     * wait exists for: the self test is over when the part reports itself
     * idle, which is the state it is in when it is sitting in CONFIG.
     *
     * Not treated as fatal on its own. If the part never goes idle, the mode
     * write and the read-back below are the tests that will say so, and they
     * say it with the part's own registers in the message.
     */
    (void)bno055_wait_status(bus, AK_BNO055_STATUS_IDLE, BNO055_BOOT_SETTLE_MS);

    /* After the reset the part is in CONFIG on page 0; both are written again
     * rather than assumed, because "the reset left it there" is a datasheet
     * claim and the write is one transaction. This one waits for the part to
     * be idle in CONFIG rather than for the register to echo the write. */
    if (bno055_write_wait(bus, BNO055_PAGE_ID, 0x00u, BNO055_WRITE_MS) != 0 ||
        bno055_set_mode(bus, AK_BNO055_MODE_CONFIG, AK_BNO055_STATUS_IDLE,
                        BNO055_MODE_SETTLE_MS) != 0 ||
        bno055_write_wait(bus, BNO055_PWR_MODE, BNO055_PWR_NORMAL,
                          BNO055_WRITE_MS) != 0 ||
        bno055_write_wait(bus, BNO055_PAGE_ID, 0x01u, BNO055_WRITE_MS) != 0 ||
        bno055_write_wait(bus, BNO055_ACC_CONFIG, BNO055_ACC_16G_250HZ,
                          BNO055_WRITE_MS) != 0 ||
        bno055_write_wait(bus, BNO055_GYR_CONFIG_0, BNO055_GYR_2000DPS_523HZ,
                          BNO055_WRITE_MS) != 0 ||
        bno055_write_wait(bus, BNO055_GYR_CONFIG_1, BNO055_GYR_NORMAL,
                          BNO055_WRITE_MS) != 0 ||
        bno055_write_wait(bus, BNO055_PAGE_ID, 0x00u, BNO055_WRITE_MS) != 0 ||
        bno055_write_wait(bus, BNO055_UNIT_SEL, BNO055_UNITS_MG_RPS,
                          BNO055_WRITE_MS) != 0) {
        return -1;
    }
    /* Deliberately not checked here: the read-back below is the check, and it
     * is a better one - it names the mode and status the part actually
     * reported, where a return code from this call could only say "it never
     * settled". */
    (void)bno055_set_mode(bus, AK_BNO055_MODE_AMG, AK_BNO055_STATUS_NO_FUSION,
                          BNO055_MODE_SETTLE_MS);

    /*
     * Read back what the part says it is doing, and believe the status rather
     * than the mode: a BNO055 that echoed every write and is still in CONFIG
     * streams zeros from every data register - plausible numbers for a level
     * aircraft on a bench, which is exactly the failure a read-back exists to
     * catch before the estimator believes it. OPR_MODE alone would not catch
     * it, because OPR_MODE reads back what was written to it whether or not
     * the part acted on it.
     */
    uint8_t mode = 0;
    uint8_t status = 0;
    uint8_t error = 0;
    if (ak_bno055_read_mode(bus, &mode) != 0 ||
        ak_bno055_read_status(bus, &status, &error) != 0) {
        return -1;
    }
    if (mode != AK_BNO055_MODE_AMG || status != AK_BNO055_STATUS_NO_FUSION) {
        if (out != 0) {
            out("imu:       bno055 is in mode 0x%x, status %u, error %u "
                "(wanted mode 0x%x and status %u)\n",
                mode, status, error, AK_BNO055_MODE_AMG,
                AK_BNO055_STATUS_NO_FUSION);
        }
        return -1;
    }

    if (out != 0) {
        ak_bno055_selftest_t st = { 0, 0, 0, 0 };
        (void)ak_bno055_read_selftest(bus, &st);
        out("imu:       configured raw (AMG, no fusion): 16 g, 2000 dps, "
            "over I2C; self test accel %s mag %s gyro %s mcu %s\n",
            st.accel ? "ok" : "FAIL", st.mag ? "ok" : "FAIL",
            st.gyro ? "ok" : "FAIL", st.mcu ? "ok" : "FAIL");
    }
    return 0;
}

static float to_float_signed_le(uint8_t lo, uint8_t hi)
{
    return (float)(int16_t)((uint16_t)hi << 8 | lo);
}

static int bno055_read(const ak_bus_t *bus, ak_imu_sample_t *sample)
{
    /*
     * One burst from ACC_DATA to the end of GYR_DATA: accelerometer, then the
     * magnetometer this driver does not use, then the gyro. Reading through
     * the magnetometer's six bytes costs six byte-times on the wire and buys
     * one transaction instead of two, which on a part that stretches the clock
     * is the cheaper of the two.
     */
    uint8_t raw[18];
    if (ak_bus_read(bus, BNO055_ACC_DATA, raw, sizeof raw) != 0) {
        return -1;
    }

    for (int axis = 0; axis < 3; axis++) {
        const unsigned a = (unsigned)(axis * 2);
        const unsigned g = (unsigned)(12 + axis * 2);
        sample->accel[axis] = to_float_signed_le(raw[a], raw[a + 1]) /
                              BNO055_ACCEL_LSB_PER_G;
        sample->gyro[axis] = to_float_signed_le(raw[g], raw[g + 1]) /
                             BNO055_GYRO_LSB_PER_RADS;
    }
    return 0;
}

/*
 * The interrupt pin, which on this part is not a gyro data-ready line.
 *
 * The BNO055's INT output is driven by its motion engine (any-motion,
 * no-motion, high-rate, high-g) and every source is disabled at reset - which
 * init guarantees, because it resets the part. So "off" is already true and is
 * answered as taken without touching the bus. "On" is refused: there is no
 * per-sample data-ready on this part for the loop to be woken by, and a
 * caller asking for one has to learn that rather than be told yes. The
 * register bits of the motion engine are not written here at all, because
 * nothing in this firmware wants those interrupts.
 */
static int bno055_configure_drdy(const ak_bus_t *bus, int enable)
{
    (void)bus;
    return enable ? -1 : 0;
}

const ak_imu_driver_t ak_imu_bno055 = {
    .name = "bno055",
    .whoami_reg = BNO055_CHIP_ID,
    .whoami_value = BNO055_CHIP_ID_VALUE,
    .init = bno055_init,
    .read = bno055_read,
    .configure_drdy = bno055_configure_drdy,
    /* In AMG mode the sample rate follows the bandwidth chosen above and the
     * datasheet does not give the pairing as a table this driver could answer
     * from; the honest answer is "not stated", which a null hook is. */
    .set_rate = 0,
};
