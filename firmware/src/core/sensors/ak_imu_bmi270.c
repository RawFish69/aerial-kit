#include "ak_imu.h"

#include "ak_imu_bmi270_config.h"
#include "ak_math.h"

/*
 * Bosch BMI270.
 *
 * The part the newer boards carry, and the first one here whose *data* is
 * invalid until its own microcontroller has been given a program: an eight
 * kilobyte configuration file that has to be uploaded after every reset, or
 * the gyro and accelerometer report nothing and say so in an internal status
 * register. That upload is the whole difference between this driver and the
 * others, and it is why ak_bus_t grew an optional burst write - eight
 * thousand transactions one byte at a time is the same bytes and a different
 * bus session.
 *
 * Every address and configuration value below was read out of the reference
 * implementation in fc-firmware-workspace/upstream rather than recalled:
 *
 *   the register map, the upload sequence (PWR_CONF off, INIT_CTRL 0, the
 *   file, INIT_CTRL 1), the soft reset, the accelerometer and gyro
 *   configuration encodings, the interrupt setup and the who-am-i:
 *     INAV 9.1.0 src/main/drivers/accgyro/accgyro_bmi270.c @ e519b69
 *
 * The configuration file itself is Bosch's, BSD-3-Clause, and is the one
 * copied thing in the tree - see ak_imu_bmi270_config.c and
 * docs/03-attribution.md. Nothing else here was copied.
 */

#define BMI_CHIP_ID        0x00u
#define BMI_DATA           0x0Cu /* accel x/y/z then gyro x/y/z, 12 bytes */
#define BMI_INTERNAL_STATUS 0x21u
#define BMI_ACC_CONF       0x40u
#define BMI_ACC_RANGE      0x41u
#define BMI_GYRO_CONF      0x42u
#define BMI_GYRO_RANGE     0x43u
#define BMI_INT1_IO_CTRL   0x53u
#define BMI_INT_MAP_DATA   0x58u
#define BMI_INIT_CTRL      0x59u
#define BMI_INIT_DATA      0x5Eu
#define BMI_PWR_CONF       0x7Cu
#define BMI_PWR_CTRL       0x7Du
#define BMI_CMD            0x7Eu

#define BMI_WHOAMI_270     0x24u

#define BMI_CMD_SOFTRESET  0xB6u

/* Performance mode, which is the one that keeps the gyro's noise down. */
#define BMI_PWR_CONF_HP    0x00u
/* Gyro and accelerometer on. The temperature sensor is not enabled - nothing
 * here reads it, and an unused sensor is an unused sensor. */
#define BMI_PWR_CTRL_ON    0x06u

/*
 * 1600 Hz out of both, with the accelerometer's filter in high-performance
 * mode, and the gyro's noise and filter in their performance settings. OSR4 is
 * the widest of the three oversampling ratios, which is what the reference
 * uses at this rate: the sample rate is high enough that the bandwidth is not
 * the limiting factor and the noise is.
 *
 *   ACC_CONF  = high performance (0x80) | OSR4 (0x00) | 1600 Hz (0x0C)
 *   GYRO_CONF = filter performance (0x80) | noise performance (0x40) | 1600 Hz
 */
/* The two configuration bytes without their rate fields: the accelerometer's
 * high-performance bit and its OSR setting, and the gyro's filter and noise
 * performance bits. The rate is the low nibble of each and is what phase 1.4
 * writes; at 1600 Hz these are 0x8C and 0xCC, which is the pair the comment
 * above describes. */
#define BMI_ACC_CONF_BASE    0x80u
#define BMI_GYRO_CONF_BASE   0xC0u

/*
 * The rates this part's two sensors can be put at, and the difference between
 * the two lists is the whole reason `set_rate` prints a sentence.
 *
 * The gyro reaches 3200 Hz - its native rate, and the number phase 1.4's target
 * names for a BMI270 - while the accelerometer stops at 1600. Asked for 3200
 * Hz, this part ends up with a gyro at 3200 and an accelerometer at 1600, and
 * every sample's two halves are then a different age. That is a fact about the
 * part and not a mistake in the request, so it is said out loud rather than
 * rounded to whichever of the two the caller happened to mean.
 *
 * Three rates each, not the eight the registers can express: 3200, 1600 and 800
 * are the values the pinned reference names for this part (Betaflight 2026.6.1,
 * accgyro_spi_bmi270.c: ODR3200 = 0x0D for the gyro, ODR1600 = 0x0C and
 * ODR800 = 0x0B for both). The codes below those are in the part's register map
 * and not in anything this tree can check, so a request for one is refused.
 */
typedef struct {
    uint32_t hz;
    uint8_t  code;
} bmi_odr_t;

static const bmi_odr_t bmi_gyro_odr[] = {
    { 3200u, 0x0Du },
    { 1600u, 0x0Cu },
    {  800u, 0x0Bu },
};

static const bmi_odr_t bmi_accel_odr[] = {
    { 1600u, 0x0Cu },
    {  800u, 0x0Bu },
};

#define BMI_DEFAULT_ODR_HZ 1600u

static const bmi_odr_t *bmi_odr_for(const bmi_odr_t *table, unsigned n,
                                    uint32_t hz)
{
    for (unsigned i = 0; i < n; i++) {
        if (hz >= table[i].hz) {
            return &table[i];
        }
    }
    return 0;
}


#define BMI_ACC_RANGE_16G    0x03u /* 2048 counts per g */
#define BMI_GYRO_RANGE_2000DPS 0x08u /* 16.384 counts per dps */

#define BMI_INT_MAP_DRDY_INT1 0x04u
#define BMI_INT1_ACTIVE_HIGH  0x0Au /* active high (0x02) | output enable (0x08) */

/* The internal status register's bit 0 is "the configuration file is loaded
 * and the feature engine has finished starting". A part that answers 0 here
 * has not taken the upload, and its data is not to be believed. */
#define BMI_STATUS_CONFIG_OK 0x01u

#define BMI_ACC_LSB_PER_G     2048.0f
#define BMI_GYRO_LSB_PER_DPS  16.384f

/* The reference waits a hundred milliseconds after the soft reset and shorter
 * times between the configuration writes; the bus contract here is in
 * milliseconds, so each of those becomes one. This runs once, on the ground. */
#define BMI_RESET_DELAY_MS 100u
#define BMI_CONFIG_DELAY_MS 1u
/* The part needs a moment after the last byte of the configuration file before
 * it is told to apply it. */
#define BMI_UPLOAD_SETTLE_MS 10u

static int bmi_write(const ak_bus_t *bus, uint8_t reg, uint8_t value)
{
    if (ak_bus_write(bus, reg, value) != 0) {
        return -1;
    }
    ak_bus_delay_ms(bus, BMI_CONFIG_DELAY_MS);
    return 0;
}

static uint32_t bmi_set_rate(const ak_bus_t *bus, uint32_t hz, ak_printf_fn out)
{
    const bmi_odr_t *gyro =
        bmi_odr_for(bmi_gyro_odr,
                    (unsigned)(sizeof bmi_gyro_odr / sizeof bmi_gyro_odr[0]), hz);
    const bmi_odr_t *accel =
        bmi_odr_for(bmi_accel_odr,
                    (unsigned)(sizeof bmi_accel_odr / sizeof bmi_accel_odr[0]), hz);

    /* Both or neither: the gyro's list is the longer one, so a rate that
     * reaches the accelerometer's top but is below the gyro's bottom is the
     * only way to have one without the other, and it cannot happen with these
     * two lists - asserted rather than assumed, because the failure would be a
     * half-configured part. */
    if (gyro == 0 || accel == 0) {
        return 0;
    }

    if (bmi_write(bus, BMI_ACC_CONF,
                  (uint8_t)(BMI_ACC_CONF_BASE | accel->code)) != 0 ||
        bmi_write(bus, BMI_GYRO_CONF,
                  (uint8_t)(BMI_GYRO_CONF_BASE | gyro->code)) != 0) {
        return 0;
    }

    if (out != 0 && accel->hz != gyro->hz) {
        out("imu:       bmi270: gyro %u Hz, accelerometer %u Hz - this part's "
            "accelerometer has no rate above 1600\n",
            gyro->hz, accel->hz);
    }
    return gyro->hz;
}

/*
 * The gyro's data-ready on interrupt pin 1, or off.
 *
 * Two registers again, and the split between them is this part's: INT1_IO_CTRL
 * describes the *pad* - active high, and output enabled, which is what makes it
 * a driven line rather than an input - and INT_MAP_DATA says which internal
 * source is routed to it. Turning the interrupt off clears the mapping and
 * leaves the pad configured: an enabled output with nothing mapped to it sits
 * low, which is a cheaper state to leave a part in than a pad left as an input
 * with the part's own pull deciding what the trace does.
 */
static int bmi_configure_drdy(const ak_bus_t *bus, int enable)
{
    if (!enable) {
        return bmi_write(bus, BMI_INT_MAP_DATA, 0x00u);
    }
    if (bmi_write(bus, BMI_INT_MAP_DATA, BMI_INT_MAP_DRDY_INT1) != 0 ||
        bmi_write(bus, BMI_INT1_IO_CTRL, BMI_INT1_ACTIVE_HIGH) != 0) {
        return -1;
    }
    return 0;
}

static int bmi_init(const ak_bus_t *bus, ak_printf_fn out)
{
    /*
     * Start from the part's own defaults. A sensor that was already running -
     * after a warm reset of the flight controller, or on a second `imu` call -
     * would otherwise keep whatever the last configuration left in it.
     */
    if (ak_bus_write(bus, BMI_CMD, BMI_CMD_SOFTRESET) != 0) {
        return -1;
    }
    ak_bus_delay_ms(bus, BMI_RESET_DELAY_MS);

    /* The configuration upload, in three parts: stop the part's power saving,
     * open the transfer, send the file, close it. */
    if (bmi_write(bus, BMI_PWR_CONF, BMI_PWR_CONF_HP) != 0 ||
        bmi_write(bus, BMI_INIT_CTRL, 0x00u) != 0) {
        return -1;
    }
    if (ak_bus_write_burst(bus, BMI_INIT_DATA, ak_bmi270_config_file,
                           AK_BMI270_CONFIG_SIZE) != 0) {
        if (out != 0) {
            out("imu:       bmi270 took no configuration file\n");
        }
        return -1;
    }
    ak_bus_delay_ms(bus, BMI_UPLOAD_SETTLE_MS);
    if (bmi_write(bus, BMI_INIT_CTRL, 0x01u) != 0) {
        return -1;
    }

    /*
     * And then ask whether it worked. This is the check the reference does not
     * make and the one worth making: without it, a part that refused the
     * upload is a part that answers reads with zeros, and zeros are a
     * perfectly plausible accelerometer reading at free fall.
     */
    uint8_t status = 0;
    if (ak_bus_read(bus, BMI_INTERNAL_STATUS, &status, 1) != 0) {
        return -1;
    }
    if ((status & BMI_STATUS_CONFIG_OK) == 0u) {
        if (out != 0) {
            out("imu:       bmi270 did not accept its configuration (status 0x%02x)\n",
                status);
        }
        return -1;
    }

    /* The two rate writes go through the same table and the same function a
     * later `set gyro_rate_hz` reaches, so `init`'s rate and the settable rates
     * cannot come apart. A zero here is a table that does not contain
     * BMI_DEFAULT_ODR_HZ, which is a bug in this file rather than a board. */
    if (bmi_set_rate(bus, BMI_DEFAULT_ODR_HZ, 0) == 0u ||
        bmi_write(bus, BMI_ACC_RANGE, BMI_ACC_RANGE_16G) != 0 ||
        bmi_write(bus, BMI_GYRO_RANGE, BMI_GYRO_RANGE_2000DPS) != 0 ||
        bmi_configure_drdy(bus, 1) != 0 ||
        bmi_write(bus, BMI_PWR_CONF, BMI_PWR_CONF_HP) != 0 ||
        bmi_write(bus, BMI_PWR_CTRL, BMI_PWR_CTRL_ON) != 0) {
        return -1;
    }

    if (out != 0) {
        out("imu:       configured for %u Hz, gyro 2000 dps, accel 16 g\n",
            (unsigned)BMI_DEFAULT_ODR_HZ);
    }
    return 0;
}

static float to_float_signed(uint16_t raw)
{
    return (float)(int16_t)raw;
}

static int bmi_read(const ak_bus_t *bus, ak_imu_sample_t *sample)
{
    uint8_t raw[12];

    if (ak_bus_read(bus, BMI_DATA, raw, sizeof raw) != 0) {
        return -1;
    }

    /* Accel first at 0x0C, gyro at 0x12, both little-endian sixteen-bit. One
     * burst, because a sensor read twice is a sensor read at two times. */
    for (unsigned axis = 0; axis < 3; axis++) {
        uint16_t a = (uint16_t)raw[axis * 2u] |
                     (uint16_t)((uint16_t)raw[axis * 2u + 1u] << 8);
        uint16_t g = (uint16_t)raw[6u + axis * 2u] |
                     (uint16_t)((uint16_t)raw[6u + axis * 2u + 1u] << 8);

        sample->accel[axis] = to_float_signed(a) / BMI_ACC_LSB_PER_G;
        sample->gyro[axis] = ak_deg2rad(to_float_signed(g) /
                                        BMI_GYRO_LSB_PER_DPS);
    }
    sample->valid = 1;
    return 0;
}

const ak_imu_driver_t ak_imu_bmi270 = {
    .name = "bmi270",
    .whoami_reg = BMI_CHIP_ID,
    .whoami_value = BMI_WHOAMI_270,
    .init = bmi_init,
    .read = bmi_read,
    .configure_drdy = bmi_configure_drdy,
    .set_rate = bmi_set_rate,
};
