#include "ak_imu.h"

#include "ak_math.h"

/*
 * The ST LSM6DSO - the part on the Qwiic breakout that is soldered to the
 * Feather F405's I2C connector, which makes this the first IMU in this
 * repository that is not on a SPI bus and not on the board.
 *
 * That last part is what shapes this file. Every reference implementation of
 * this part that exists is a SPI one:
 *
 *   ST's own register map, the init order, the full-scale and ODR encodings,
 *   the bit masks, and the 0.070 dps/LSB scale:
 *     Betaflight 2026.6.1 src/main/drivers/accgyro/accgyro_spi_lsm6dso_init.c,
 *     accgyro_spi_lsm6dso.c and accgyro_spi_lsm6dso.h @ 6dbc4218
 *   the same map independently, and the one place its value is named as the
 *   *disable* value rather than the enable value it is being used as:
 *     INAV 9.1.0 src/main/drivers/accgyro/accgyro_lsm6dxx.{c,h}
 *   2048 counts per g at 16 g:  both of the above (`acc_1G = 512 * 4`), which
 *   is the same constant the MPU driver in this tree uses
 *
 * docs/03-attribution.md carries the revisions. No code was copied: this is a
 * register map written against the datasheet's own table numbers.
 *
 * Both of those references drive the part over SPI, and the difference is not
 * a detail. Their configuration sets CTRL4_C's I2C_DISABLE bit, which on a
 * part reached over I2C would switch off the bus the next write arrives on.
 * This driver writes that bit clear and says so at the write. Nothing in the
 * two references could have caught that, because neither of them has ever run
 * this part over I2C - which is the reason the sentence is here rather than in
 * a commit message.
 *
 * Two more differences from the references, written down rather than smoothed
 * over, because a person comparing this file against them will find both:
 *
 *   * The gyro scale. Betaflight sets 0.070 dps/LSB and cites the datasheet
 *     section and symbol for it (section 4.1, G_So, "70 mdps/LSB"). INAV's
 *     one LSM6DXX driver sets `1.0f / 16.4f` for every part it detects, the
 *     LSM6DSO included - and 16.4 counts per dps is the MPU-6000's number, not
 *     this part's. The two disagree by about 14%, which is a gyro that reads
 *     low by an eighth and an attitude that drifts against it. This file takes
 *     Betaflight's, because that is the one of the two that names its source.
 *     The number wants checking against the datasheet at the bench; it is one
 *     constant and one line.
 *
 *   * INT2_CTRL. Betaflight writes 0x02 here under a comment that reads
 *     "Disable interrupt pin 2". INAV's header names 0x00 as the value that
 *     disables a data-ready interrupt and its own config writes that. This file
 *     writes 0x00: bit 1 is the gyro's data-ready enable on that pin, and a
 *     part nobody is going to ask about its interrupts should not be raising
 *     them on a wire that on this board goes nowhere.
 *
 * The second accelerometer filter is the references' one other disagreement -
 * Betaflight's DSO file takes LPF1, INAV's generic one takes LPF2 - and this
 * follows the DSO file, since this is that part.
 *
 * NOT here, and deliberately: the LSM6DSL and the LSM6DS3, which answer the
 * same register map at the same addresses with 0x6A and 0x69 in the who-am-i
 * register. For the MPU family that difference is the whole of it and one
 * driver covers three parts; here it is not. The references configure the DSL
 * with a different CTRL6_C mask (0x13 against the DSO's 0x17), so adding it
 * means a second initialisation, and a second initialisation written from a
 * mask difference I have not run on a part is a guess wearing a driver's
 * clothes. A DSL on the bench answers 0x6A, `ak_imu_open` prints it, and the
 * port is a copy of this file with the other mask in it.
 *
 * The who-am-i value is not a part number: 0x6C is what the LSM6DSO, the
 * LSM6DSOX and the LSM6DSO32 all answer. The three have different full-scale
 * tables, so a breakout that reads 0x6C and is not a plain DSO wants its
 * sensitivity table checked before the accelerometer is believed - the gyro
 * half of this file is unaffected, since ±2000 dps is 70 mdps/LSB on all
 * three.
 */

#define LSM6DSO_WHOAMI        0x0F
#define LSM6DSO_WHOAMI_VALUE  0x6C
#define LSM6DSO_INT1_CTRL     0x0D
#define LSM6DSO_INT2_CTRL     0x0E
#define LSM6DSO_CTRL1_XL      0x10
#define LSM6DSO_CTRL2_G       0x11
#define LSM6DSO_CTRL3_C       0x12
#define LSM6DSO_CTRL4_C       0x13
#define LSM6DSO_CTRL6_C       0x15
#define LSM6DSO_CTRL9_XL      0x18

/* The twelve bytes a read returns, in this order: gyro x/y/z, then accel
 * x/y/z. Note the two ways this is not the MPU-6000: the axes are the other
 * way round (gyro first, and at a lower address), and they are little endian.
 * An attitude built from a driver that got either one wrong is plausible and
 * wrong, which is what the host test pins. */
#define LSM6DSO_OUT          0x22

/* 833 Hz for the accelerometer, 6664 Hz for the gyro - the references'
 * arrangement, and eight gyro samples per accelerometer one. The accel half of
 * a burst therefore repeats between its own samples, which is a fact about
 * this part's configuration that a caller choosing a loop rate wants to know
 * about. */
#define LSM6DSO_ODR_833HZ    0x07u /* accel, at gyro/8; the gyro's own table
                                    * below carries the codes it can take. */

/*
 * The rates the gyro can be put at by phase 1.4's `gyro_rate_hz`, and the four
 * are the ones the pinned reference names for this part: the LSM6DSO's own init
 * file in Betaflight 2026.6.1 (accgyro_spi_lsm6dso_init.c) carries ODR833 =
 * 0x07, ODR1667 = 0x08, ODR3332 = 0x09 and ODR6664 = 0x0A for both CTRL1_XL and
 * CTRL2_G - the same four codes in either register, which is why one table
 * serves both here. The register map has codes below 833 Hz; nothing in this
 * tree names them, so a request for one is refused rather than guessed.
 *
 * The accelerometer is not moved by `set_rate`. It stays at the 833 Hz `init`
 * put it at, which is the reference's arrangement and one gyro sample in eight:
 * the estimator's accelerometer correction is a slow outer loop that 833 Hz is
 * eight times more than enough for, and this is the one part here whose two
 * sensors are *meant* to run at different rates. A caller that wants the
 * accelerometer faster has no way to ask for it yet, and that is a gap this
 * milestone leaves stated rather than closed.
 */
typedef struct {
    uint32_t hz;
    uint8_t  code;
} lsm6dso_odr_t;

static const lsm6dso_odr_t lsm6dso_gyro_odr[] = {
    { 6664u, 0x0Au },
    { 3332u, 0x09u },
    { 1660u, 0x08u },
    {  833u, 0x07u },
};

#define LSM6DSO_DEFAULT_ODR_HZ 6664u

#define LSM6DSO_FS_16G       0x01u
#define LSM6DSO_FS_2000DPS   0x03u

/* Accelerometer output from LPF1, which is the DSO file's choice of the two. */
#define LSM6DSO_XL_LPF1      0x00u

/* Bit 6 stops the part updating a pair of output registers while they are
 * being read - the one that matters here, because a 12-byte burst spans six of
 * them and a gyro that moves mid-read is a sample that never existed. Bit 2
 * makes the address increment during that burst, which is what turns twelve
 * transactions into one. Both references set both. */
#define LSM6DSO_CTRL3_MASK   0x7Cu
#define LSM6DSO_CTRL3_BDU    0x40u
#define LSM6DSO_CTRL3_IF_INC 0x04u
#define LSM6DSO_CTRL3_RESET  0x01u

/* Bit 1 turns the gyro's LPF1 on. Bit 2 is I2C_DISABLE, which both references
 * set and which this driver clears - see the header. The mask covers both, so
 * the write that enables the filter is the same write that has to name what it
 * wants done with the bus. */
#define LSM6DSO_CTRL4_MASK   0x06u
#define LSM6DSO_CTRL4_LPF1_G 0x02u

/* Bit 4 is XL_HM_MODE and on this part zero is the *high performance* setting.
 * Bits 2:0 pick the gyro's LPF1 cutoff; 0 is 335.5 Hz. */
#define LSM6DSO_CTRL6_MASK   0x17u
#define LSM6DSO_CTRL6_XL_HP  0x00u
#define LSM6DSO_CTRL6_335HZ  0x00u

/* The part has an I3C mode on the same pins. Nothing here speaks it. */
#define LSM6DSO_CTRL9_MASK   0x02u
#define LSM6DSO_CTRL9_I3C_OFF 0x02u

/* Bit 1 of either interrupt-control register is the gyro's data-ready enable. */
#define LSM6DSO_INT_DRDY_G   0x02u

#define LSM6DSO_ACCEL_LSB_PER_G 2048.0f
#define LSM6DSO_GYRO_DPS_PER_LSB 0.070f

/* The part asks for 100 ms after a soft reset - it is reloading its own trim
 * from wherever it keeps it - and a couple of microseconds between
 * configuration writes. The bus contract here is in milliseconds, which is
 * three orders of magnitude longer than the second one; this runs once, at
 * boot, on the ground, which is why that is acceptable rather than why it was
 * overlooked. */
#define LSM6DSO_RESET_DELAY_MS 100u
#define LSM6DSO_CONFIG_DELAY_MS 1u

/* The wait is an argument rather than a constant because the reset's is a
 * hundred times the others', and a part that is left alone for a millisecond
 * after being told to reload its own trim is a part that is being configured
 * while it is still reading. */
static int lsm6dso_write_wait(const ak_bus_t *bus, uint8_t reg, uint8_t value,
                              unsigned delay_ms)
{
    if (ak_bus_write(bus, reg, value) != 0) {
        return -1;
    }
    ak_bus_delay_ms(bus, delay_ms);
    return 0;
}

/*
 * The gyro's data-ready on interrupt pin 1, or off.
 *
 * INT1_CTRL's bit 1 is the gyro's data-ready enable, and it is the whole of
 * this call: there is no pulse shape to set on this part, because the line is
 * a push-pull pulse the part raises for the configured duration by itself.
 *
 * The pair of writes `init` used to make directly still happens here and in the
 * same order - this is the same register with the same value written the same
 * way, reached through a name rather than spelled out twice. What it buys is
 * that a board with no INT pad can turn the line off through the same call the
 * other three drivers implement, instead of the driver having no off switch.
 */
static int lsm6dso_configure_drdy(const ak_bus_t *bus, int enable)
{
    return lsm6dso_write_wait(bus, LSM6DSO_INT1_CTRL,
                              enable ? LSM6DSO_INT_DRDY_G : 0x00u,
                              LSM6DSO_CONFIG_DELAY_MS);
}

/*
 * Read a configuration register, replace the bits this driver owns, write it
 * back, wait.
 *
 * The alternative - writing a constant - is what the MPU driver in this tree
 * does, and it is right there for a reason that does not hold here: that
 * driver's constants come from a reference that runs the same sequence on a
 * part it has just reset, so the value it writes is the whole register. These
 * four registers have bits in them that neither reference names (CTRL6_C bit 3
 * among them), and a constant would have to say something about them. A
 * read-modify-write says nothing: the driver owns what the mask covers and
 * hands the rest back untouched.
 *
 * A read that fails is a part that will not configure, and the caller turns
 * that into a failed open rather than a part configured halfway.
 */
static int lsm6dso_write_bits(const ak_bus_t *bus, uint8_t reg, uint8_t mask,
                              uint8_t value, unsigned delay_ms)
{
    uint8_t current = 0;

    if (ak_bus_read(bus, reg, &current, 1) != 0) {
        return -1;
    }
    return lsm6dso_write_wait(bus, reg,
                              (uint8_t)((current & (uint8_t)~mask) | value),
                              delay_ms);
}

/* Phase 1.4's hook. Only CTRL2_G is written: the accelerometer's rate is
 * CTRL1_XL's and this driver leaves it where `init` put it - see the table's
 * comment. The full-scale field goes back in with the rate because the two
 * share a byte, and the write is a read-free constant rather than a
 * read-modify-write for the same reason `init`'s is. */
static uint32_t lsm6dso_set_rate(const ak_bus_t *bus, uint32_t hz,
                                 ak_printf_fn out)
{
    (void)out;

    for (unsigned i = 0;
         i < sizeof lsm6dso_gyro_odr / sizeof lsm6dso_gyro_odr[0]; i++) {
        if (hz >= lsm6dso_gyro_odr[i].hz) {
            if (lsm6dso_write_wait(bus, LSM6DSO_CTRL2_G,
                                   (uint8_t)((lsm6dso_gyro_odr[i].code << 4) |
                                             (LSM6DSO_FS_2000DPS << 2)),
                                   LSM6DSO_CONFIG_DELAY_MS) != 0) {
                return 0;
            }
            return lsm6dso_gyro_odr[i].hz;
        }
    }
    return 0;
}

static int lsm6dso_init(const ak_bus_t *bus, ak_printf_fn out)
{
    /* Reset first, and wait the hundred milliseconds it asks for. Every write
     * below assumes the part is at its reset values, and on a dev board that
     * has been running since the last power cycle it is not: without this, the
     * configuration lands on whatever a previous sketch left in the registers.
     * It is also the read-modify-writes' precondition - they preserve bits this
     * driver does not own, and "the bits it does not own are the reset ones" is
     * only true after a reset. */
    if (lsm6dso_write_bits(bus, LSM6DSO_CTRL3_C, LSM6DSO_CTRL3_RESET,
                           LSM6DSO_CTRL3_RESET,
                           LSM6DSO_RESET_DELAY_MS) != 0) {
        return -1;
    }

    if (lsm6dso_configure_drdy(bus, 1) != 0 ||
        lsm6dso_write_wait(bus, LSM6DSO_INT2_CTRL, 0x00u,
                           LSM6DSO_CONFIG_DELAY_MS) != 0 ||
        lsm6dso_write_wait(bus, LSM6DSO_CTRL1_XL,
                           (uint8_t)((LSM6DSO_ODR_833HZ << 4) |
                                     (LSM6DSO_FS_16G << 2) |
                                     (LSM6DSO_XL_LPF1 << 1)),
                           LSM6DSO_CONFIG_DELAY_MS) != 0 ||
        lsm6dso_set_rate(bus, LSM6DSO_DEFAULT_ODR_HZ, 0) == 0u ||
        lsm6dso_write_bits(bus, LSM6DSO_CTRL3_C, LSM6DSO_CTRL3_MASK,
                           LSM6DSO_CTRL3_BDU | LSM6DSO_CTRL3_IF_INC,
                           LSM6DSO_CONFIG_DELAY_MS) != 0 ||
        /* The one write in this file that a SPI reference could not have got
         * right for us: bit 2 is I2C_DISABLE and this is an I2C part. */
        lsm6dso_write_bits(bus, LSM6DSO_CTRL4_C, LSM6DSO_CTRL4_MASK,
                           LSM6DSO_CTRL4_LPF1_G,
                           LSM6DSO_CONFIG_DELAY_MS) != 0 ||
        lsm6dso_write_bits(bus, LSM6DSO_CTRL6_C, LSM6DSO_CTRL6_MASK,
                           LSM6DSO_CTRL6_XL_HP | LSM6DSO_CTRL6_335HZ,
                           LSM6DSO_CONFIG_DELAY_MS) != 0 ||
        lsm6dso_write_bits(bus, LSM6DSO_CTRL9_XL, LSM6DSO_CTRL9_MASK,
                           LSM6DSO_CTRL9_I3C_OFF,
                           LSM6DSO_CONFIG_DELAY_MS) != 0) {
        return -1;
    }

    if (out != 0) {
        /* Not the part's name: which of the family answered is the driver
         * table's business and it prints that itself. What is worth saying here
         * is what it was configured to - and that the wire is I2C, which on a
         * board whose IMU is a breakout is the thing most likely to be wrong. */
        out("imu:       configured for 833 Hz accel / 6664 Hz gyro, "
            "16 g, 2000 dps, over I2C\n");
    }
    return 0;
}

static float to_float_signed_le(uint8_t lo, uint8_t hi)
{
    return (float)(int16_t)((uint16_t)hi << 8 | lo);
}

static int lsm6dso_read(const ak_bus_t *bus, ak_imu_sample_t *sample)
{
    uint8_t raw[12];

    /* One burst, and the part latches each register pair for the duration of
     * it, so the six values come from one sample rather than six instants. */
    if (ak_bus_read(bus, LSM6DSO_OUT, raw, sizeof raw) != 0) {
        return -1;
    }

    for (int axis = 0; axis < 3; axis++) {
        uint16_t g = (uint16_t)(axis * 2);
        uint16_t a = (uint16_t)(6 + axis * 2);
        sample->gyro[axis] = ak_deg2rad(to_float_signed_le(raw[g], raw[g + 1]) *
                                        LSM6DSO_GYRO_DPS_PER_LSB);
        sample->accel[axis] =
            to_float_signed_le(raw[a], raw[a + 1]) / LSM6DSO_ACCEL_LSB_PER_G;
    }
    sample->valid = 1;
    return 0;
}

const ak_imu_driver_t ak_imu_lsm6dso = {
    .name = "lsm6dso",
    .whoami_reg = LSM6DSO_WHOAMI,
    .whoami_value = LSM6DSO_WHOAMI_VALUE,
    .init = lsm6dso_init,
    .read = lsm6dso_read,
    .configure_drdy = lsm6dso_configure_drdy,
    .set_rate = lsm6dso_set_rate,
};
