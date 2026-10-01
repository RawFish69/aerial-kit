#include "ak_imu.h"

#include "ak_math.h"

/*
 * InvenSense ICM-42688-P, and the ICM-42605 that reports the same registers.
 *
 * Every address and value below was read out of the reference implementations
 * in fc-firmware-workspace/upstream rather than recalled:
 *
 *   registers, FSR/ODR encoding   Betaflight 2026.6.1
 *                                 src/main/drivers/accgyro/accgyro_spi_icm426xx.c
 *   who-am-i values               the same tree's accgyro_mpu.h
 *   gyro scale 16.4 LSB per dps   INAV 9.1.0 accgyro_icm42605.c
 *   accel scale 2048 LSB per g    Betaflight's acc_1G for the 16 g range
 *
 * docs/03-attribution.md carries the revisions. None of it is copied code: it
 * is a register map and four constants, written here against the datasheet's
 * section numbers.
 */

#define ICM_WHOAMI          0x75
#define ICM_WHOAMI_42688P   0x47
#define ICM_WHOAMI_42605    0x42

#define ICM_REG_BANK_SEL    0x76
#define ICM_BANK_0          0x00
#define ICM_PWR_MGMT0       0x4E
#define ICM_PWR_ACCEL_LN    (3u << 0)
#define ICM_PWR_GYRO_LN     (3u << 2)
#define ICM_GYRO_CONFIG0    0x4F
#define ICM_ACCEL_CONFIG0   0x50
#define ICM_GYRO_ACCEL_CFG0 0x52
#define ICM_INTF_CONFIG1    0x4D
#define ICM_INTF_AFSR_MASK  0xC0u
#define ICM_INTF_AFSR_OFF   0x40u
#define ICM_ACCEL_DATA_X1   0x1F /* x, y, z, then gyro x, y, z */

/* Both ranges are the widest the part offers, which is what a flight
 * controller wants: clipping a hard manoeuvre costs more than resolution. */
#define ICM_GYRO_FS_2000DPS_ODR_1KHZ  ((0u << 5) | 6u)
#define ICM_ACCEL_FS_16G_ODR_1KHZ     ((0u << 5) | 6u)
#define ICM_UI_FILT_LOW_LATENCY       ((15u << 4) | 15u)

#define ICM_ACCEL_LSB_PER_G   2048.0f
#define ICM_GYRO_LSB_PER_DPS  16.4f

#define ICM_CONFIG_DELAY_MS   15u

static float to_float_signed(uint16_t raw)
{
    return (float)(int16_t)raw;
}

static int icm_init(const ak_bus_t *bus, ak_printf_fn out)
{
    uint8_t scratch = 0;

    if (ak_bus_write(bus, ICM_REG_BANK_SEL, ICM_BANK_0) != 0) {
        return -1;
    }

    /* Turn both sensors off before configuring them: the datasheet asks for it
     * (ICM-42688-P section 12.9) and a part configured while running takes some
     * of the writes and ignores others. */
    if (ak_bus_write(bus, ICM_PWR_MGMT0, 0x00) != 0) {
        return -1;
    }

    /* The gyro output can stall unless AFSR is turned off. Betaflight clears
     * the same two bits for the same reason. */
    if (ak_bus_read(bus, ICM_INTF_CONFIG1, &scratch, 1) != 0) {
        return -1;
    }
    uint8_t intf = (uint8_t)((scratch & ~ICM_INTF_AFSR_MASK) | ICM_INTF_AFSR_OFF);
    if (ak_bus_write(bus, ICM_INTF_CONFIG1, intf) != 0) {
        return -1;
    }

    if (ak_bus_write(bus, ICM_GYRO_CONFIG0, ICM_GYRO_FS_2000DPS_ODR_1KHZ) != 0 ||
        ak_bus_write(bus, ICM_ACCEL_CONFIG0, ICM_ACCEL_FS_16G_ODR_1KHZ) != 0 ||
        ak_bus_write(bus, ICM_GYRO_ACCEL_CFG0, ICM_UI_FILT_LOW_LATENCY) != 0) {
        return -1;
    }
    ak_bus_delay_ms(bus, ICM_CONFIG_DELAY_MS);

    /* Low noise mode on both: the gyro needs a few milliseconds after this
     * before its output is meaningful, which is why the delay is here and not
     * just after the configuration writes. */
    if (ak_bus_write(bus, ICM_PWR_MGMT0, ICM_PWR_ACCEL_LN | ICM_PWR_GYRO_LN) != 0) {
        return -1;
    }
    ak_bus_delay_ms(bus, ICM_CONFIG_DELAY_MS);

    /* The part has to still be answering as one of the two this driver serves,
     * which is what catches a sensor that vanished or a bus that stopped
     * working half way through configuration. It was a check for 0x47 alone
     * until the ICM-42605 was added to the table, and that entry failed its own
     * driver's init - found by the test that was written for the 42605. */
    if (ak_bus_read(bus, ICM_WHOAMI, &scratch, 1) != 0 ||
        (scratch != ICM_WHOAMI_42688P && scratch != ICM_WHOAMI_42605)) {
        return -1;
    }

    if (out != 0) {
        out("imu:       configured for 1 kHz, gyro 2000 dps, accel 16 g\n");
    }
    return 0;
}

static int icm_read(const ak_bus_t *bus, ak_imu_sample_t *sample)
{
    uint8_t raw[12];

    /* One burst for all six axes: two reads can straddle a sample boundary and
     * mix two moments into one attitude. */
    if (ak_bus_read(bus, ICM_ACCEL_DATA_X1, raw, sizeof raw) != 0) {
        return -1;
    }

    float ax = to_float_signed((uint16_t)((raw[0] << 8) | raw[1]));
    float ay = to_float_signed((uint16_t)((raw[2] << 8) | raw[3]));
    float az = to_float_signed((uint16_t)((raw[4] << 8) | raw[5]));
    float gx = to_float_signed((uint16_t)((raw[6] << 8) | raw[7]));
    float gy = to_float_signed((uint16_t)((raw[8] << 8) | raw[9]));
    float gz = to_float_signed((uint16_t)((raw[10] << 8) | raw[11]));

    sample->accel[0] = ax / ICM_ACCEL_LSB_PER_G;
    sample->accel[1] = ay / ICM_ACCEL_LSB_PER_G;
    sample->accel[2] = az / ICM_ACCEL_LSB_PER_G;

    sample->gyro[0] = ak_deg2rad(gx / ICM_GYRO_LSB_PER_DPS);
    sample->gyro[1] = ak_deg2rad(gy / ICM_GYRO_LSB_PER_DPS);
    sample->gyro[2] = ak_deg2rad(gz / ICM_GYRO_LSB_PER_DPS);
    return 0;
}

const ak_imu_driver_t ak_imu_icm42688 = {
    .name = "icm42688p",
    .whoami_reg = ICM_WHOAMI,
    .whoami_value = ICM_WHOAMI_42688P,
    .init = icm_init,
    .read = icm_read,
};

/* The ICM-42605 is the same part with a different who-am-i and a smaller
 * gyro range: the register map, the configuration and the scaling above all
 * apply, which is exactly why it is a second table entry rather than a second
 * driver. A board that fits one is a board this firmware already drives. */
const ak_imu_driver_t ak_imu_icm42605 = {
    .name = "icm42605",
    .whoami_reg = ICM_WHOAMI,
    .whoami_value = ICM_WHOAMI_42605,
    .init = icm_init,
    .read = icm_read,
};
