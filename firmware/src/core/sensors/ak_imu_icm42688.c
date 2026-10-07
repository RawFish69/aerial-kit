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

/* The interrupt block. Until this driver grew these four registers it wrote
 * none of them, which made the ICM the only part here with no data-ready
 * output configured at all: an ICM-42688 board would have had a gyro whose
 * every sample the loop had to come and ask for, with no line to say when.
 * The values are Betaflight's (accgyro_spi_icm426xx.c, the INT_CONFIG block),
 * which is the only reference in the tree that configures this part's
 * interrupts. */
#define ICM_INT_CONFIG      0x14
#define ICM_INT_CONFIG0     0x63
#define ICM_INT_CONFIG1     0x64
#define ICM_INT_SOURCE0     0x65

/* Pulsed (bit 2 clear, not latched), push-pull (bit 1), active high (bit 0).
 * Pulsed because a level that stays asserted until the part is read is a line
 * the EXTI fires on once and then cannot fire on again until the handler has
 * completed a bus transaction - which is the opposite of what an interrupt is
 * wanted for here. */
#define ICM_INT_CONFIG_VAL  0x03u

/* Clear on sensor-register read, which is the mode that makes the pulse end
 * when the burst the handler is about to do lands. */
#define ICM_INT_CONFIG0_VAL 0x00u

/* Bit 3 of INT_SOURCE0 routes the UI data-ready to INT1. That is the whole of
 * "route data-ready to INT1" on this part. */
#define ICM_INT_SOURCE0_DRDY_INT1 0x08u

/* INT_CONFIG1 bit 4 is INT_ASYNC_RESET, which is set out of reset; the
 * datasheet's own note, quoted in the reference, is "User should change setting
 * to 0 from default setting of 1, for proper INT1 and INT2 pin operation". Bit
 * 5 disables the de-assert, which is what makes the line a pulse rather than
 * something the handler has to come back and release, and bit 6 lengthens that
 * pulse to 8 us - both of which want to be wide enough for an EXTI to catch.
 * This one is a read-modify-write because the register has bits this driver
 * does not name, and Betaflight's own write is the same read-modify-write. */
#define ICM_INT_CONFIG1_ASYNC_RESET_BIT 4u
#define ICM_INT_CONFIG1_TDEASSERT_BIT   5u
#define ICM_INT_CONFIG1_TPULSE_BIT      6u
#define ICM_INT_CONFIG1_ON  ((1u << ICM_INT_CONFIG1_TDEASSERT_BIT) | \
                             (1u << ICM_INT_CONFIG1_TPULSE_BIT))
#define ICM_INT_CONFIG1_MASK ((1u << ICM_INT_CONFIG1_ASYNC_RESET_BIT) | \
                              ICM_INT_CONFIG1_ON)

/* Both ranges are the widest the part offers, which is what a flight
 * controller wants: clipping a hard manoeuvre costs more than resolution. The
 * full-scale field is the top of each configuration register and the ODR field
 * is the bottom nibble, which is why the two share a byte. */
#define ICM_FS_GYRO_2000DPS  0u
#define ICM_FS_ACCEL_16G     0u
#define ICM_UI_FILT_LOW_LATENCY       ((15u << 4) | 15u)

/* How long the part is given after the configuration writes, and after an ODR
 * change, before its output is read. Declared here rather than beside the
 * scaling constants because `icm_set_rate` below is a caller. */
#define ICM_CONFIG_DELAY_MS   15u

/*
 * The rates this driver can put the part at - the whole of what phase 1.4's
 * `gyro_rate_hz` can reach on an ICM - and there are four of them.
 *
 * They are the four the pinned reference names and no more. Betaflight's
 * `odrLUT` for this family (2026.6.1, accgyro_spi_icm426xx.c, "see GYRO_ODR in
 * section 5.6") carries 8k, 4k, 2k and 1k; the part's register has codes below
 * 1 kHz as well, and this driver does not carry them because nothing in this
 * tree can check them against a source it has. A rate this table does not have
 * is refused by name rather than guessed at, which is the difference between
 * "the aircraft is at 4 kHz" and "the aircraft is at something".
 *
 * The accelerometer's table is the same four values - the two registers take
 * the same ODR field - so one entry configures both and a sample's halves are
 * always the same age.
 */
typedef struct {
    uint32_t hz;
    uint8_t  code;
} icm_odr_t;

static const icm_odr_t icm_odr_table[] = {
    { 8000u, 3u },
    { 4000u, 4u },
    { 2000u, 5u },
    { 1000u, 6u },
};

/* What `init` leaves the part at, and the top of what the table above can be
 * asked for at boot. There is a test that this number is one of the table's:
 * `init` looks it up like any other request, so a value that is not in the
 * table would fail the open rather than quietly configure nothing. */
#define ICM_DEFAULT_ODR_HZ 1000u

/* The fastest rate in the table that is not above `hz`, or null. Null is a
 * request below every rate this part offers - 500 Hz on an ICM - and the caller
 * turns it into a refusal rather than rounding it up to a kilohertz the person
 * did not ask for. */
static const icm_odr_t *icm_odr_for(uint32_t hz)
{
    for (unsigned i = 0; i < sizeof icm_odr_table / sizeof icm_odr_table[0]; i++) {
        if (hz >= icm_odr_table[i].hz) {
            return &icm_odr_table[i];
        }
    }
    return 0;
}

/* Both configuration registers, because both sensors' rates are the same
 * number: see the table's comment. */
static int icm_write_odr(const ak_bus_t *bus, const icm_odr_t *odr)
{
    if (ak_bus_write(bus, ICM_GYRO_CONFIG0,
                     (uint8_t)((ICM_FS_GYRO_2000DPS << 5) | odr->code)) != 0 ||
        ak_bus_write(bus, ICM_ACCEL_CONFIG0,
                     (uint8_t)((ICM_FS_ACCEL_16G << 5) | odr->code)) != 0) {
        return -1;
    }
    return 0;
}

/*
 * Phase 1.4's hook. The write is the pair of registers above and the wait is
 * the same one `init` uses after configuring them, for the same reason: the
 * part's output needs a few milliseconds after an ODR change before it means
 * anything, and a loop that read it immediately would be reading samples from
 * the rate it just left.
 */
static uint32_t icm_set_rate(const ak_bus_t *bus, uint32_t hz, ak_printf_fn out)
{
    (void)out;
    const icm_odr_t *odr = icm_odr_for(hz);

    if (odr == 0) {
        return 0;
    }
    if (icm_write_odr(bus, odr) != 0) {
        return 0;
    }
    ak_bus_delay_ms(bus, ICM_CONFIG_DELAY_MS);
    return odr->hz;
}

#define ICM_ACCEL_LSB_PER_G   2048.0f
#define ICM_GYRO_LSB_PER_DPS  16.4f

/*
 * Route the gyro's data-ready to INT1, or stop routing it.
 *
 * The whole of "on" is INT_SOURCE0's bit 3. The three writes around it describe
 * the *shape* of the pulse rather than what it means: without INT_CONFIG and
 * INT_CONFIG0 the part is left with whatever it powers up with, which is a
 * latched line - and a latched line is one the EXTI fires on once and can
 * never fire on again, because nothing releases it until a read that the
 * interrupt was supposed to trigger. INT_CONFIG1's async-reset bit is the
 * datasheet's own "change this to 0 or INT1 does not work".
 *
 * Turning it *off* writes only INT_SOURCE0. The pin's shape does not matter on
 * a board that is not listening to it, and leaving the other three alone keeps
 * the off path the exact inverse of the on path's one load-bearing bit.
 *
 * Bank 0 is selected first rather than assumed: this can be called on its own,
 * long after `icm_init` left the part on bank 0, and a caller that changed
 * banks in between should not silently get a write to the wrong register.
 */
static int icm_configure_drdy(const ak_bus_t *bus, int enable)
{
    uint8_t current = 0;

    if (ak_bus_write(bus, ICM_REG_BANK_SEL, ICM_BANK_0) != 0) {
        return -1;
    }

    if (!enable) {
        return ak_bus_write(bus, ICM_INT_SOURCE0, 0x00u);
    }

    if (ak_bus_write(bus, ICM_INT_CONFIG, ICM_INT_CONFIG_VAL) != 0 ||
        ak_bus_write(bus, ICM_INT_CONFIG0, ICM_INT_CONFIG0_VAL) != 0 ||
        ak_bus_write(bus, ICM_INT_SOURCE0, ICM_INT_SOURCE0_DRDY_INT1) != 0) {
        return -1;
    }

    if (ak_bus_read(bus, ICM_INT_CONFIG1, &current, 1) != 0) {
        return -1;
    }

    return ak_bus_write(bus, ICM_INT_CONFIG1,
                        (uint8_t)((current & (uint8_t)~ICM_INT_CONFIG1_MASK) |
                                  ICM_INT_CONFIG1_ON));
}

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

    /* The rate `init` leaves the part at, through the same table and the same
     * write a later `set gyro_rate_hz` goes through - so the two cannot
     * disagree about which code means which rate. */
    const icm_odr_t *odr = icm_odr_for(ICM_DEFAULT_ODR_HZ);

    if (odr == 0 ||
        icm_write_odr(bus, odr) != 0 ||
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

    /* Data-ready out on INT1, so that the part leaves `init` in the same state
     * the other three drivers' parts do. Before this call the ICM was the one
     * part here with the line left unconfigured, which on a board that wires it
     * is a gyro nothing can tell the loop about. */
    if (icm_configure_drdy(bus, 1) != 0) {
        return -1;
    }

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
    .configure_drdy = icm_configure_drdy,
    .set_rate = icm_set_rate,
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
    .configure_drdy = icm_configure_drdy,
    .set_rate = icm_set_rate,
};
