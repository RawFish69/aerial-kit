#include "ak_baro.h"

/*
 * Infineon DPS310, and the SPL06-003 that is the same part under another name.
 *
 * The register map, the initialisation order and - the part that actually
 * matters - the compensation arithmetic were read out of the reference
 * implementations rather than recalled:
 *
 *   registers, reset, oversampling, the coefficient decode, and sections 4.9.1
 *   and 4.9.2 of the datasheet as written out in code:
 *     INAV 9.1.0 src/main/drivers/barometer/barometer_dps310.c @ e519b69
 *
 * docs/03-attribution.md carries the revision. No code was copied: this is the
 * same arithmetic written against the datasheet's own section numbers, with the
 * names this repository uses.
 *
 * Two things about a barometer are worth knowing before reading further. It
 * measures pressure, not height, and the conversion is a standard-atmosphere
 * curve plus a reference taken on the ground - so absolute altitude from one is
 * weather, and the *change* since take-off is the height. And it is slow and
 * oversampled on purpose: the part averages sixteen measurements per result,
 * which is a few tens of centimetres of noise at 32 Hz, and that is the trade
 * that makes it useful for holding an altitude at all.
 */

#define DPS_REG_PSR_B2      0x00 /* pressure, three bytes, big endian first */
#define DPS_REG_TMP_B2      0x03
#define DPS_REG_PRS_CFG     0x06
#define DPS_REG_TMP_CFG     0x07
#define DPS_REG_MEAS_CFG    0x08
#define DPS_REG_CFG_REG     0x09
#define DPS_REG_RESET       0x0C
#define DPS_REG_ID          0x0D
#define DPS_REG_COEF        0x10
#define DPS_REG_COEF_SRCE   0x28

#define DPS_ID_DPS310       0x10
#define DPS_ID_SPL06        0x11

#define DPS_RESET_SOFT      0x09

#define DPS_MEAS_COEF_RDY   (1u << 7)
#define DPS_MEAS_SENSOR_RDY (1u << 6)
#define DPS_MEAS_TMP_RDY    (1u << 5)
#define DPS_MEAS_PRS_RDY    (1u << 4)
#define DPS_MEAS_MODE_MASK  0x07u
#define DPS_MEAS_TEMP_ONCE  0x02u
#define DPS_MEAS_CONTINUOUS 0x07u

#define DPS_PRS_RATE_32HZ   0x50u
#define DPS_PRS_OVERSAMPLE  0x04u /* 16 times, the datasheet's "standard" */
#define DPS_TMP_RATE_32HZ   0x50u
#define DPS_TMP_OVERSAMPLE  0x04u
#define DPS_TMP_EXT         0x80u /* temperature from the MEMS, not the ASIC */

/* With sixteen-times oversampling the two result registers are shifted, and the
 * part scales the raw values for us: the datasheet asks for these bits and the
 * scale factors below to match. */
#define DPS_CFG_SHIFTS      (0x04u | 0x08u)
#define DPS_SCALE_16X       253952.0f

#define DPS_RESET_DELAY_MS  40u

typedef struct {
    int16_t c0;
    int16_t c1;
    int32_t c00;
    int32_t c10;
    int16_t c01;
    int16_t c11;
    int16_t c20;
    int16_t c21;
    int16_t c30;
    int16_t c31; /* SPL06-003 only */
    int16_t c40; /* SPL06-003 only */
} dps_coefficients_t;

static dps_coefficients_t dps_cal;
static uint8_t            dps_id;

/* The coefficient registers are packed end to end and each field may straddle
 * a byte boundary, which is why this is a function and not three lines at each
 * call site. */
static int32_t twos_complement(uint32_t raw, unsigned bits)
{
    uint32_t sign = 1u << (bits - 1u);
    if (raw & sign) {
        return (int32_t)raw - (int32_t)(1u << bits);
    }
    return (int32_t)raw;
}

static int dps_read_regs(const ak_bus_t *bus, uint8_t reg, uint8_t *buf,
                         unsigned len)
{
    return ak_bus_read(bus, reg, buf, len) == 0 ? 0 : -1;
}

static int dps_write_reg(const ak_bus_t *bus, uint8_t reg, uint8_t value)
{
    return ak_bus_write(bus, reg, value) == 0 ? 0 : -1;
}

static int dps_set_bits(const ak_bus_t *bus, uint8_t reg, uint8_t bits)
{
    uint8_t value = 0;
    if (dps_read_regs(bus, reg, &value, 1) != 0) {
        return -1;
    }
    if ((value & bits) != bits) {
        return dps_write_reg(bus, reg, (uint8_t)(value | bits));
    }
    return 0;
}

static int dps_read_coefficients(const ak_bus_t *bus)
{
    uint8_t coef[21];
    unsigned length = dps_id == DPS_ID_SPL06 ? 21u : 18u;

    if (dps_read_regs(bus, DPS_REG_COEF, coef, length) != 0) {
        return -1;
    }

    /* Every field is two's complement, and the bit ranges are the datasheet's
     * table 8 - the comments are the bit ranges, not a decoration. */
    dps_cal.c0 = (int16_t)twos_complement(((uint32_t)coef[0] << 4) |
                                              (((uint32_t)coef[1] >> 4) & 0x0Fu),
                                          12);
    dps_cal.c1 = (int16_t)twos_complement((((uint32_t)coef[1] & 0x0Fu) << 8) |
                                              (uint32_t)coef[2],
                                          12);
    dps_cal.c00 = twos_complement(((uint32_t)coef[3] << 12) |
                                      ((uint32_t)coef[4] << 4) |
                                      (((uint32_t)coef[5] >> 4) & 0x0Fu),
                                  20);
    dps_cal.c10 = twos_complement((((uint32_t)coef[5] & 0x0Fu) << 16) |
                                      ((uint32_t)coef[6] << 8) |
                                      (uint32_t)coef[7],
                                  20);
    dps_cal.c01 = (int16_t)twos_complement(((uint32_t)coef[8] << 8) |
                                               (uint32_t)coef[9],
                                           16);
    dps_cal.c11 = (int16_t)twos_complement(((uint32_t)coef[10] << 8) |
                                               (uint32_t)coef[11],
                                           16);
    dps_cal.c20 = (int16_t)twos_complement(((uint32_t)coef[12] << 8) |
                                               (uint32_t)coef[13],
                                           16);
    dps_cal.c21 = (int16_t)twos_complement(((uint32_t)coef[14] << 8) |
                                               (uint32_t)coef[15],
                                           16);
    dps_cal.c30 = (int16_t)twos_complement(((uint32_t)coef[16] << 8) |
                                               (uint32_t)coef[17],
                                           16);

    dps_cal.c31 = 0;
    dps_cal.c40 = 0;
    if (dps_id == DPS_ID_SPL06) {
        dps_cal.c31 = (int16_t)twos_complement(((uint32_t)coef[18] << 4) |
                                                   (((uint32_t)coef[19] >> 4) & 0x0Fu),
                                               12);
        dps_cal.c40 = (int16_t)twos_complement((((uint32_t)coef[19] & 0x0Fu) << 8) |
                                                   (uint32_t)coef[20],
                                               12);
    }
    return 0;
}

static int dps_init(const ak_bus_t *bus, ak_printf_fn out)
{
    uint8_t status = 0;
    uint8_t id = 0;

    if (dps_read_regs(bus, DPS_REG_ID, &id, 1) != 0) {
        return -1;
    }
    dps_id = id;

    /* A soft reset, then the two ready bits: a part that has not finished
     * reading its own factory coefficients will answer everything else with
     * zeros, which look exactly like a vacuum. */
    if (dps_write_reg(bus, DPS_REG_RESET, DPS_RESET_SOFT) != 0) {
        return -1;
    }
    ak_bus_delay_ms(bus, DPS_RESET_DELAY_MS);

    if (dps_read_regs(bus, DPS_REG_MEAS_CFG, &status, 1) != 0) {
        return -1;
    }
    if ((status & (DPS_MEAS_COEF_RDY | DPS_MEAS_SENSOR_RDY)) !=
        (DPS_MEAS_COEF_RDY | DPS_MEAS_SENSOR_RDY)) {
        return -1;
    }

    if (dps_read_coefficients(bus) != 0) {
        return -1;
    }

    /* One temperature measurement first, and it is not about the temperature:
     * the pressure result is compensated with the temperature, so a part that
     * has never measured one has no compensated pressure to give. */
    uint8_t mode = 0;
    if (dps_read_regs(bus, DPS_REG_MEAS_CFG, &mode, 1) != 0) {
        return -1;
    }
    mode = (uint8_t)((mode & (uint8_t)~DPS_MEAS_MODE_MASK) | DPS_MEAS_TEMP_ONCE);
    if (dps_write_reg(bus, DPS_REG_MEAS_CFG, mode) != 0) {
        return -1;
    }
    ak_bus_delay_ms(bus, DPS_RESET_DELAY_MS);

    if (dps_set_bits(bus, DPS_REG_PRS_CFG,
                     DPS_PRS_RATE_32HZ | DPS_PRS_OVERSAMPLE) != 0 ||
        dps_set_bits(bus, DPS_REG_TMP_CFG,
                     DPS_TMP_RATE_32HZ | DPS_TMP_OVERSAMPLE) != 0 ||
        dps_set_bits(bus, DPS_REG_CFG_REG, DPS_CFG_SHIFTS) != 0) {
        return -1;
    }

    if (dps_id != DPS_ID_SPL06) {
        /* The datasheet's temperature source bit: on the DPS310 the MEMS
         * temperature is the one the coefficients were measured against. */
        uint8_t source = 0;
        if (dps_read_regs(bus, DPS_REG_COEF_SRCE, &source, 1) != 0) {
            return -1;
        }
        if (dps_set_bits(bus, DPS_REG_TMP_CFG,
                         (uint8_t)(source & (uint8_t)DPS_TMP_EXT)) != 0) {
            return -1;
        }
    }

    if (dps_read_regs(bus, DPS_REG_MEAS_CFG, &mode, 1) != 0) {
        return -1;
    }
    mode = (uint8_t)((mode & (uint8_t)~DPS_MEAS_MODE_MASK) | DPS_MEAS_CONTINUOUS);
    if (dps_write_reg(bus, DPS_REG_MEAS_CFG, mode) != 0) {
        return -1;
    }

    if (out != 0) {
        out("baro:      %s\n", dps_id == DPS_ID_SPL06 ? "spl06-003" : "dps310");
    }
    return 0;
}

static int dps_read(const ak_bus_t *bus, ak_baro_sample_t *sample)
{
    uint8_t status = 0;
    uint8_t raw[6];

    if (dps_read_regs(bus, DPS_REG_MEAS_CFG, &status, 1) != 0) {
        return -1;
    }
    if ((status & DPS_MEAS_PRS_RDY) == 0u) {
        return 0; /* nothing new, which is not an error */
    }

    if (dps_read_regs(bus, DPS_REG_PSR_B2, raw, sizeof raw) != 0) {
        return -1;
    }

    int32_t p_raw = twos_complement(((uint32_t)raw[0] << 16) |
                                        ((uint32_t)raw[1] << 8) |
                                        (uint32_t)raw[2],
                                    24);
    int32_t t_raw = twos_complement(((uint32_t)raw[3] << 16) |
                                        ((uint32_t)raw[4] << 8) |
                                        (uint32_t)raw[5],
                                    24);

    float p_scaled = (float)p_raw / DPS_SCALE_16X;
    float t_scaled = (float)t_raw / DPS_SCALE_16X;

    /* Datasheet section 4.9.1 and 4.9.2, as written out in the reference
     * driver: pressure is a cubic in the scaled pressure reading with the
     * temperature crossing into it, and the temperature is a line. */
    float c00 = (float)dps_cal.c00;
    float c10 = (float)dps_cal.c10;
    float c20 = (float)dps_cal.c20;
    float c30 = (float)dps_cal.c30;
    float c01 = (float)dps_cal.c01;
    float c11 = (float)dps_cal.c11;
    float c21 = (float)dps_cal.c21;
    float c40 = (float)dps_cal.c40;
    float c31 = (float)dps_cal.c31;

    if (dps_id == DPS_ID_SPL06) {
        sample->pressure_pa = c00 +
            p_scaled * (c10 + p_scaled * (c20 + p_scaled * (c30 + p_scaled * c40))) +
            t_scaled * c01 +
            t_scaled * p_scaled * (c11 + p_scaled * (c21 + p_scaled * c31));
    } else {
        sample->pressure_pa = c00 +
            p_scaled * (c10 + p_scaled * (c20 + p_scaled * c30)) +
            t_scaled * c01 +
            t_scaled * p_scaled * (c11 + p_scaled * c21);
    }
    sample->temperature_c = (float)dps_cal.c0 * 0.5f + (float)dps_cal.c1 * t_scaled;
    sample->valid = 1;
    return 1;
}

const ak_baro_driver_t ak_baro_dps310 = {
    "dps310", DPS_REG_ID, DPS_ID_DPS310, dps_init, dps_read,
};

/* The SPL06-003 is the same part under another name, with three more
 * coefficient bytes and one more term in the pressure polynomial. Two entries
 * rather than one with two values, because the probe is what says which
 * arithmetic to use. */
const ak_baro_driver_t ak_baro_spl06 = {
    "spl06-003", DPS_REG_ID, DPS_ID_SPL06, dps_init, dps_read,
};
