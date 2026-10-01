#ifndef AK_ARCH_AT32F435_I2C_TIMING_H
#define AK_ARCH_AT32F435_I2C_TIMING_H

#include <stdint.h>

#include "regs.h"

/*
 * The AT32's I2C timing register, as arithmetic rather than as a constant.
 *
 * This part's I2C is the newer peripheral: one `clkctrl` register describing
 * the whole bus - the two halves of the SCL period, the data setup and hold
 * delays, and a prescaler - rather than the F405's FREQ/CCR/TRISE trio. So the
 * arithmetic is not the F405's renamed; it is the other one, and it is the
 * single most consequential difference in this port for the barometer on the
 * bus. A wrong `clkctrl` does not fail: it runs the bus at the wrong speed, and
 * a bus too fast reads exactly like a barometer that is not there.
 *
 * The formula is the one INAV and ST's own tools use, and the constants are the
 * I2C specification's:
 *
 *   - the SCL period is the two halves of the period, each `n + 1` prescaled
 *     clocks, plus the bus's own rise and fall and the part's analog filter;
 *   - SCLDEL is the data setup time the device is promised
 *     (trmax + tsuDATmin), SDADEL the data hold time;
 *   - the prescaler is searched from 1 upwards until every field fits its
 *     width, which is why this is a loop rather than one expression - at a
 *     144 MHz bus and 400 kHz the first fit is at a prescaler of three, and a
 *     formula that stopped at one would produce a register with fields too
 *     large to hold the answer.
 *
 * The numbers behind it, from the specification for fast mode (up to 400 kHz):
 * a rise and fall time of 300 ns at worst, 100 ns typical, a data setup time of
 * 100 ns, no hold time required, and a 50 ns analog filter delay in the part.
 * The 4.75 below is the split ST's own tools use between the two halves of the
 * low period, and it is kept because matching the reference implementation is
 * worth more here than a derivation nobody can check without a scope.
 */

#define AK_I2C_T_RISE_MAX_NS  300u
#define AK_I2C_T_FALL_MAX_NS  300u
#define AK_I2C_T_SU_DAT_NS    100u
#define AK_I2C_T_HD_DAT_NS      0u
#define AK_I2C_T_RISE_NS      100u
#define AK_I2C_T_FALL_NS      100u
#define AK_I2C_T_AF_NS         70u
#define AK_I2C_T_AF_MIN_NS     50u

/* The prescaler field is eight bits on this part, split between two nibbles. */
#define AK_I2C_PRESC_MIN  1u
#define AK_I2C_PRESC_MAX  254u

static inline uint32_t ak_at32_i2c_presc_pair(unsigned presc)
{
    /* divh holds the high nibble and divl the low one - the same split this
     * part's SPI uses for its divider, and the reason both are written here
     * rather than one. */
    return ((uint32_t)(presc >> 4) & 0xFu) << AK_I2C_CLKCTRL_DIVH_SHIFT |
           ((uint32_t)(presc & 0xFu) & 0xFu) << AK_I2C_CLKCTRL_DIVL_SHIFT;
}

/*
 * The register value for a bus running at `rate_khz` on a peripheral clock of
 * `pclk_hz`. Zero means the two cannot be reconciled at all - which for the
 * clocks and rates this firmware uses does not happen, and is returned rather
 * than silently rounded because a bus at the wrong speed is worse than one that
 * is off: see the top of this file.
 */
static inline uint32_t ak_at32_i2c_clkctrl(uint32_t pclk_hz, uint32_t rate_khz)
{
    if (pclk_hz == 0u || rate_khz == 0u) {
        return 0u;
    }

    const float t_i2c_ns = 1.0e9f / (float)pclk_hz;
    const float t_scl_ns = 1.0e6f / (float)rate_khz;
    /* The two synchronisation times, which are the same here because this
     * arithmetic uses one rise and one fall time: the bus, the part's filter,
     * and the three clocks the peripheral takes to see an edge. */
    const float t_sync_ns = (float)AK_I2C_T_FALL_NS + (float)AK_I2C_T_AF_NS +
                            3.0f * t_i2c_ns;

    for (unsigned presc = AK_I2C_PRESC_MIN; presc <= AK_I2C_PRESC_MAX; presc++) {
        float unit_ns = (float)(presc + 1u) * t_i2c_ns;
        float scldel_f = ((float)AK_I2C_T_RISE_MAX_NS +
                          (float)AK_I2C_T_SU_DAT_NS) / unit_ns - 1.0f;
        float sdadel_f = ((float)AK_I2C_T_FALL_MAX_NS +
                          (float)AK_I2C_T_HD_DAT_NS - (float)AK_I2C_T_AF_MIN_NS -
                          3.0f * t_i2c_ns) / unit_ns;
        float sclhl = (t_scl_ns - 2.0f * t_sync_ns) / unit_ns - 1.0f;

        if (sclhl < 2.0f) {
            continue; /* the period cannot hold the bus's own edges */
        }

        float sclh_f = sclhl / 4.75f;
        float scll_f = sclhl - sclh_f;

        unsigned scldel = scldel_f < 0.0f ? 0u : (unsigned)(scldel_f + 0.5f);
        unsigned sdadel = sdadel_f < 0.0f ? 0u : (unsigned)(sdadel_f + 0.5f);
        unsigned sclh = (unsigned)sclh_f;
        unsigned scll = (unsigned)(scll_f + 0.5f);

        if (scldel > AK_I2C_CLKCTRL_FIELD_MASK ||
            sdadel > AK_I2C_CLKCTRL_FIELD_MASK || sclh < 1u || scll < 1u ||
            sclh > 255u || scll > 255u) {
            continue;
        }

        return ak_at32_i2c_presc_pair(presc) |
               ((uint32_t)(scldel & AK_I2C_CLKCTRL_FIELD_MASK)
                << AK_I2C_CLKCTRL_SCLD_SHIFT) |
               ((uint32_t)(sdadel & AK_I2C_CLKCTRL_FIELD_MASK)
                << AK_I2C_CLKCTRL_SDAD_SHIFT) |
               ((uint32_t)sclh << AK_I2C_CLKCTRL_SCLH_SHIFT) |
               ((uint32_t)scll << AK_I2C_CLKCTRL_SCLL_SHIFT);
    }
    return 0u;
}

/*
 * And the other direction, for the checks: what rate the register above
 * actually asks for, in kHz. It is the same arithmetic read backwards, which is
 * how a test can hold the answer to the rate it meant - the two are written
 * next to each other so they cannot drift.
 */
static inline uint32_t ak_at32_i2c_rate_khz(uint32_t clkctrl, uint32_t pclk_hz)
{
    if (clkctrl == 0u || pclk_hz == 0u) {
        return 0u;
    }

    unsigned scll = (unsigned)(clkctrl >> AK_I2C_CLKCTRL_SCLL_SHIFT) & 0xFFu;
    unsigned sclh = (unsigned)(clkctrl >> AK_I2C_CLKCTRL_SCLH_SHIFT) & 0xFFu;
    unsigned presc = (((unsigned)(clkctrl >> AK_I2C_CLKCTRL_DIVH_SHIFT) & 0xFu)
                      << 4) |
                     ((unsigned)(clkctrl >> AK_I2C_CLKCTRL_DIVL_SHIFT) & 0xFu);
    float t_i2c_ns = 1.0e9f / (float)pclk_hz;
    float t_sync_ns = (float)AK_I2C_T_FALL_NS + (float)AK_I2C_T_AF_NS +
                      3.0f * t_i2c_ns;
    float period_ns = ((float)(scll + 1u) + (float)(sclh + 1u)) *
                          (float)(presc + 1u) * t_i2c_ns +
                      2.0f * t_sync_ns;

    return (uint32_t)(1.0e6f / period_ns + 0.5f);
}

#endif /* AK_ARCH_AT32F435_I2C_TIMING_H */
