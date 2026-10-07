#include "ak_imu.h"

#include "ak_math.h"

/*
 * The InvenSense MPU-6x00 family: the MPU-6000, the MPU-6050 that answers the
 * same way over I2C, and the MPU-6500 and MPU-9250 that answer to the same
 * register at the same address with a different value in it.
 *
 * The part every flight controller of the last decade has had on it, and the
 * one a board found in a drawer is most likely to be. Its register map is not
 * the ICM-42688-P's: the data registers start at 0x3B rather than 0x1F, there
 * is no bank select, the clock choice is a three-bit field rather than a
 * power-management register, and the sample rate comes from a divider rather
 * than an output-data-rate encoding. Same shape of driver, different map.
 *
 * Every address, bit and scale below was read out of the reference
 * implementations in fc-firmware-workspace/upstream rather than recalled:
 *
 *   register map, the init order, the DLPF and full-scale encodings, the
 *   who-am-i value, and the 1/340 temperature scale this driver does not use:
 *     Betaflight 2026.6.1 src/main/drivers/accgyro/accgyro_spi_mpu6000.c and
 *     accgyro_mpu.h @ 6dbc4218
 *   16.4 counts per dps at ±2000 dps:  the same tree, and the ICM's datasheet
 *   agrees, which is why the constant is not repeated in this file
 *
 * docs/03-attribution.md carries the revisions. No code was copied: this is a
 * register map written against the datasheet's own table numbers.
 *
 * The 6500 and the 9250 are here because those register maps and the 6000's
 * are the same map: the reference implementation's MPU-6500 path writes the
 * same CONFIG, GYRO_CONFIG, ACCEL_CONFIG, INT_PIN_CFG and INT_ENABLE bytes with
 * the same bit encodings this file does, and it uses one DLPF table for the
 * family - which is what this driver does too, so `0x03` is "the 42 Hz
 * setting" on any of them whether or not each datasheet's table calls it 42.
 *
 * One difference, written down rather than smoothed over: the reference's
 * MPU-6500 sequence begins by resetting the part (PWR_MGMT_1 bit 7, a
 * SIGNAL_PATH_RESET, and a hundred milliseconds of waiting) and this one does
 * not, because the sequence below is the 6000's and it is the one the host
 * tests pin. A 6500 that misbehaves on a bench is worth trying that reset
 * against first.
 */

#define MPU_WHOAMI          0x75
#define MPU_WHOAMI_6000     0x68 /* also the MPU-6050 and MPU-3050 */
#define MPU_WHOAMI_6500     0x70
#define MPU_WHOAMI_9250     0x71

#define MPU_SMPLRT_DIV      0x19
#define MPU_CONFIG          0x1A
#define MPU_GYRO_CONFIG     0x1B
#define MPU_ACCEL_CONFIG    0x1C
#define MPU_INT_PIN_CFG     0x37
#define MPU_INT_ENABLE      0x38
#define MPU_DATA            0x3B /* accel x/y/z, temperature, gyro x/y/z */
#define MPU_USER_CTRL       0x6A
#define MPU_PWR_MGMT_1      0x6B
#define MPU_PWR_MGMT_2      0x6C

/* Clock from the gyro's PLL rather than the internal oscillator: the internal
 * one drifts with temperature, and a gyro whose sample rate drifts with it is a
 * gyro that measures its own clock error. */
#define MPU_CLK_PLL_Z       0x03
/* The part has an auxiliary I2C bus for a magnetometer. Nothing here uses it,
 * so it is switched off rather than left listening on somebody's pins. */
#define MPU_I2C_IF_DIS      0x10

/* DLPF 42 Hz, which puts the gyro's output rate at 1 kHz - and with a 1 kHz
 * output rate, a sample-rate divider of zero is 1 kHz. */
#define MPU_DLPF_42HZ       0x03

/*
 * The rate the sample divider is set to, and the rates it can be set to.
 *
 * The register's own definition, quoted in the pinned reference
 * (Betaflight 2026.6.1, accgyro_mpu6050.c: "SMPLRT_DIV = 0 Sample Rate =
 * Gyroscope Output Rate / (1 + SMPLRT_DIV)"), is the whole of the arithmetic
 * below: with the DLPF above the output rate is 1 kHz, so the divisor for a
 * wanted rate is 1000/hz - 1.
 *
 * The list is the rates that divide 1000 exactly. Every other value between
 * them is a rate the part can be *near* and not at, and a firmware that
 * answered "3 kHz" with a divider of 0 would be claiming a rate it does not
 * have; one asked for 3000 Hz gets the 1000 it already had, by name.
 *
 * One divider drives both sensors on this part, so unlike the BMI270 the
 * accelerometer cannot be left behind - whatever the gyro is at, the
 * accelerometer is at too.
 */
#define MPU_DEFAULT_RATE_HZ 1000u

static const uint32_t mpu_rates[] = {
    1000u, 500u, 250u, 200u, 125u, 100u, 50u, 25u, 20u, 10u, 5u,
};

#define MPU_GYRO_FS_2000DPS 0x18
#define MPU_ACCEL_FS_16G    0x18

/* Any register read clears the data-ready interrupt, so the pin does not have
 * to be acked separately. Nothing here reads the pin, but a board that fits one
 * later should not find it stuck. */
#define MPU_INT_ANYRD_CLEAR 0x10
#define MPU_INT_DATA_RDY    0x01

#define MPU_ACCEL_LSB_PER_G   2048.0f
#define MPU_GYRO_LSB_PER_DPS  16.4f

/* The reference driver waits 15 microseconds between configuration writes. The
 * bus contract here is in milliseconds, which is a hundred times longer than
 * the part asks for - and this runs once, at boot, on the ground. */
#define MPU_CONFIG_DELAY_MS 1u

static int mpu_write(const ak_bus_t *bus, uint8_t reg, uint8_t value)
{
    if (ak_bus_write(bus, reg, value) != 0) {
        return -1;
    }
    ak_bus_delay_ms(bus, MPU_CONFIG_DELAY_MS);
    return 0;
}

/* Phase 1.4's hook: the fastest rate in `mpu_rates` that is not above `hz`,
 * written as a divider - see the table's comment for the arithmetic. The
 * output rate the part is configured for is 1 kHz and is not itself settable,
 * so a request above it lands on 1000. */
static uint32_t mpu_set_rate(const ak_bus_t *bus, uint32_t hz, ak_printf_fn out)
{
    (void)out;

    for (unsigned i = 0; i < sizeof mpu_rates / sizeof mpu_rates[0]; i++) {
        if (hz >= mpu_rates[i]) {
            const uint8_t divider =
                (uint8_t)(MPU_DEFAULT_RATE_HZ / mpu_rates[i] - 1u);
            if (mpu_write(bus, MPU_SMPLRT_DIV, divider) != 0) {
                return 0;
            }
            return mpu_rates[i];
        }
    }
    return 0;
}

/*
 * The data-ready interrupt on the part's INT pin, or off.
 *
 * Two registers, and they are a pair on this part: INT_PIN_CFG says what the
 * pin does - here, that any register read clears the latch (ANYRD_CLEAR), so
 * the line does not stay asserted after the handler has read the sample it was
 * raised for - and INT_ENABLE says which of the part's internal sources is
 * allowed to drive it. Neither is meaningful without the other.
 *
 * This comment is older than this function and said as much: "Nothing here
 * reads the pin, but a board that fits one later should not find it stuck."
 * The board that fits one later now has a call to reach this by.
 */
static int mpu_configure_drdy(const ak_bus_t *bus, int enable)
{
    if (!enable) {
        return mpu_write(bus, MPU_INT_ENABLE, 0x00);
    }
    if (mpu_write(bus, MPU_INT_PIN_CFG, MPU_INT_ANYRD_CLEAR) != 0 ||
        mpu_write(bus, MPU_INT_ENABLE, MPU_INT_DATA_RDY) != 0) {
        return -1;
    }
    return 0;
}

static int mpu_init(const ak_bus_t *bus, ak_printf_fn out)
{
    /* The order is the reference implementation's: clock, then the I2C bus off,
     * then the axes on, then the rates, then the ranges, then the interrupt. A
     * part that is still asleep ignores most of it. */
    if (ak_bus_write(bus, MPU_PWR_MGMT_1, MPU_CLK_PLL_Z) != 0) {
        return -1;
    }
    ak_bus_delay_ms(bus, MPU_CONFIG_DELAY_MS);

    if (mpu_write(bus, MPU_USER_CTRL, MPU_I2C_IF_DIS) != 0 ||
        mpu_write(bus, MPU_PWR_MGMT_2, 0x00) != 0 ||
        mpu_set_rate(bus, MPU_DEFAULT_RATE_HZ, 0) == 0u ||
        mpu_write(bus, MPU_CONFIG, MPU_DLPF_42HZ) != 0 ||
        mpu_write(bus, MPU_GYRO_CONFIG, MPU_GYRO_FS_2000DPS) != 0 ||
        mpu_write(bus, MPU_ACCEL_CONFIG, MPU_ACCEL_FS_16G) != 0 ||
        mpu_configure_drdy(bus, 1) != 0) {
        return -1;
    }

    if (out != 0) {
        /* Not the part's name: which of the family answered is the driver
         * table's business and it prints that itself. What is worth saying
         * here is what it was configured to. */
        out("imu:       configured for 1 kHz, gyro 2000 dps, accel 16 g\n");
    }
    return 0;
}

static float to_float_signed(uint16_t raw)
{
    return (float)(int16_t)raw;
}

static int mpu_read(const ak_bus_t *bus, ak_imu_sample_t *sample)
{
    uint8_t raw[14];

    if (ak_bus_read(bus, MPU_DATA, raw, sizeof raw) != 0) {
        return -1;
    }

    /* Big endian, and in one burst: the part latches its registers for a
     * multi-byte read, so the three axes come from the same sample. Reading
     * them one at a time gives an attitude that shakes. */
    for (int axis = 0; axis < 3; axis++) {
        uint16_t a = (uint16_t)((uint16_t)raw[axis * 2] << 8 | raw[axis * 2 + 1]);
        uint16_t g = (uint16_t)((uint16_t)raw[8 + axis * 2] << 8 |
                                raw[8 + axis * 2 + 1]);
        sample->accel[axis] = to_float_signed(a) / MPU_ACCEL_LSB_PER_G;
        sample->gyro[axis] = ak_deg2rad(to_float_signed(g) /
                                        MPU_GYRO_LSB_PER_DPS);
    }
    sample->valid = 1;
    return 0;
}

const ak_imu_driver_t ak_imu_mpu6000 = {
    .name = "mpu6000",
    .whoami_reg = MPU_WHOAMI,
    .whoami_value = MPU_WHOAMI_6000,
    .init = mpu_init,
    .read = mpu_read,
    .configure_drdy = mpu_configure_drdy,
    .set_rate = mpu_set_rate,
};

/* The same driver, two more parts. Same register map, same initialisation, same
 * twelve bytes of output - a different value in the who-am-i register is the
 * whole of the difference the firmware can see. */
const ak_imu_driver_t ak_imu_mpu6500 = {
    .name = "mpu6500",
    .whoami_reg = MPU_WHOAMI,
    .whoami_value = MPU_WHOAMI_6500,
    .init = mpu_init,
    .read = mpu_read,
    .configure_drdy = mpu_configure_drdy,
    .set_rate = mpu_set_rate,
};

const ak_imu_driver_t ak_imu_mpu9250 = {
    .name = "mpu9250",
    .whoami_reg = MPU_WHOAMI,
    .whoami_value = MPU_WHOAMI_9250,
    .init = mpu_init,
    .read = mpu_read,
    .configure_drdy = mpu_configure_drdy,
    .set_rate = mpu_set_rate,
};
