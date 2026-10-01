/*
 * The barometer: a DPS310 on a fake bus, and the altitude curve.
 *
 * Two halves, and they need different kinds of evidence. The *driver* is a
 * register map and a piece of the datasheet's arithmetic, so what can be
 * checked without a part is that it asks for the right registers, decodes the
 * coefficients the way the datasheet packs them, and computes what the formula
 * says - the expected numbers below were worked out separately, in Python,
 * from the same datasheet equations, and pinned here.
 *
 * The *altitude* is arithmetic this firmware wrote itself (there is no libm in
 * the image and no pow()), so that half is checked against libm's exact curve
 * over the whole range a flight uses, the way the other hand-written maths in
 * this repository is.
 */

#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "ak_baro.h"
#include "tests.h"

/* --- a DPS310, as a register file ----------------------------------------- */

static uint8_t regs[256];
static int     fail_reads;
static int     fail_writes;
static int     fail_read_register = -1; /* one register the bus will not answer */
static unsigned fail_at_write;          /* or writes that fail from the Nth on */
static unsigned delays;
static unsigned writes_done;

static int bus_read(void *ctx, uint8_t reg, uint8_t *buf, unsigned len)
{
    (void)ctx;
    if (fail_reads) {
        return -1;
    }
    if (fail_read_register >= 0 && (int)reg == fail_read_register) {
        return -1;
    }
    for (unsigned i = 0; i < len; i++) {
        buf[i] = regs[(reg + i) & 0xFFu];
    }
    return 0;
}

static int bus_write(void *ctx, uint8_t reg, uint8_t value)
{
    (void)ctx;
    if (fail_writes) {
        return -1;
    }
    if (fail_at_write != 0u && writes_done + 1u >= fail_at_write) {
        return -1;
    }
    writes_done++;
    regs[reg] = value;
    return 0;
}

static void bus_delay(void *ctx, unsigned ms)
{
    (void)ctx;
    delays += ms;
}

static const ak_bus_t fake_bus = {
    .read = bus_read,
    .write = bus_write,
    .delay_ms = bus_delay,
    .ctx = 0,
};

/* The coefficient block, packed the way the datasheet packs it: c0 and c1
 * straddle a byte, c00 and c10 straddle two, and the rest are byte pairs. */
static void put_coefficients(int32_t c0, int32_t c1, int32_t c00, int32_t c10,
                             int32_t c01, int32_t c11, int32_t c20, int32_t c21,
                             int32_t c30)
{
    uint32_t u0 = (uint32_t)c0 & 0xFFFu;
    uint32_t u1 = (uint32_t)c1 & 0xFFFu;
    uint32_t u00 = (uint32_t)c00 & 0xFFFFFu;
    uint32_t u10 = (uint32_t)c10 & 0xFFFFFu;
    uint32_t u01 = (uint32_t)c01 & 0xFFFFu;
    uint32_t u11 = (uint32_t)c11 & 0xFFFFu;
    uint32_t u20 = (uint32_t)c20 & 0xFFFFu;
    uint32_t u21 = (uint32_t)c21 & 0xFFFFu;
    uint32_t u30 = (uint32_t)c30 & 0xFFFFu;

    regs[0x10] = (uint8_t)(u0 >> 4);
    regs[0x11] = (uint8_t)(((u0 & 0x0Fu) << 4) | ((u1 >> 8) & 0x0Fu));
    regs[0x12] = (uint8_t)(u1 & 0xFFu);
    regs[0x13] = (uint8_t)(u00 >> 12);
    regs[0x14] = (uint8_t)((u00 >> 4) & 0xFFu);
    regs[0x15] = (uint8_t)(((u00 & 0x0Fu) << 4) | ((u10 >> 16) & 0x0Fu));
    regs[0x16] = (uint8_t)((u10 >> 8) & 0xFFu);
    regs[0x17] = (uint8_t)(u10 & 0xFFu);
    regs[0x18] = (uint8_t)(u01 >> 8);
    regs[0x19] = (uint8_t)(u01 & 0xFFu);
    regs[0x1A] = (uint8_t)(u11 >> 8);
    regs[0x1B] = (uint8_t)(u11 & 0xFFu);
    regs[0x1C] = (uint8_t)(u20 >> 8);
    regs[0x1D] = (uint8_t)(u20 & 0xFFu);
    regs[0x1E] = (uint8_t)(u21 >> 8);
    regs[0x1F] = (uint8_t)(u21 & 0xFFu);
    regs[0x20] = (uint8_t)(u30 >> 8);
    regs[0x21] = (uint8_t)(u30 & 0xFFu);
}

/* The three bytes an SPL06-003 has and a DPS310 does not: c31 and c40, both
 * twelve-bit fields, with c31's low four bits and c40's high four sharing the
 * middle register. They are the last three bytes of the coefficient block. */
static void put_spl06_coefficients(int32_t c31, int32_t c40)
{
    uint32_t u31 = (uint32_t)c31 & 0xFFFu;
    uint32_t u40 = (uint32_t)c40 & 0xFFFu;

    regs[0x22] = (uint8_t)(u31 >> 4);
    regs[0x23] = (uint8_t)(((u31 & 0x0Fu) << 4) | ((u40 >> 8) & 0x0Fu));
    regs[0x24] = (uint8_t)(u40 & 0xFFu);
}

static void put_raw(int32_t pressure, int32_t temperature)
{
    uint32_t p = (uint32_t)pressure;
    uint32_t t = (uint32_t)temperature;
    regs[0x00] = (uint8_t)(p >> 16);
    regs[0x01] = (uint8_t)((p >> 8) & 0xFFu);
    regs[0x02] = (uint8_t)(p & 0xFFu);
    regs[0x03] = (uint8_t)(t >> 16);
    regs[0x04] = (uint8_t)((t >> 8) & 0xFFu);
    regs[0x05] = (uint8_t)(t & 0xFFu);
}

static void reset_registers(void)
{
    memset(regs, 0, sizeof regs);
    fail_reads = 0;
    fail_writes = 0;
    fail_read_register = -1;
    fail_at_write = 0u;
    delays = 0;
    writes_done = 0u;
    regs[0x0D] = 0x10; /* who-am-i: a DPS310 */
    /* Coefficient and sensor ready, and a pressure result waiting. */
    regs[0x08] = 0xF0;
}

/* --- the driver ----------------------------------------------------------- */

static void test_detection(void)
{
    uint8_t seen = 0;

    reset_registers();
    regs[0x0D] = 0x11; /* an SPL06-003, the same part under another name */
    const ak_baro_driver_t *spl06 = ak_baro_detect(&fake_bus, &seen);
    expect("the SPL06 is recognised as itself",
           spl06 != 0 && seen == 0x11 && strcmp(spl06->name, "spl06-003") == 0);

    reset_registers();
    expect("a DPS310 answers as a DPS310",
           ak_baro_detect(&fake_bus, &seen) != 0 && seen == 0x10);

    /* A register file of 0xFF is a bus with nothing on it, or a part this
     * build does not know. Either way it is not a barometer. */
    memset(regs, 0xFF, sizeof regs);
    expect("a bus full of 0xFF is not a barometer",
           ak_baro_detect(&fake_bus, &seen) == 0 && seen == 0xFF);

    reset_registers();
    expect("and a device that cannot be read at all is not one either",
           (fail_reads = 1, ak_baro_detect(&fake_bus, &seen) == 0));
    fail_reads = 0;
}

static void test_open(void)
{
    ak_baro_t baro;

    /* A part that has not finished reading its own coefficients answers
     * everything else with zeros, which look exactly like a vacuum: refusing
     * it is the difference between "no barometer" and "a barometer at 0 Pa". */
    reset_registers();
    regs[0x08] = 0x00;
    expect("a part whose coefficients are not ready does not open",
           ak_baro_open(&baro, &fake_bus, 0) != 0 && !baro.present);

    reset_registers();
    expect("a configured part opens", ak_baro_open(&baro, &fake_bus, 0) == 0 &&
           baro.present && baro.samples == 0 && baro.errors == 0);

    /* It went through the datasheet's sequence: a reset, a wait, a temperature
     * measurement, and another wait before the coefficients mean anything. */
    expect("and the init followed the datasheet's waits",
           delays >= 80u && (regs[0x09] & 0x0C) == 0x0C &&
           (regs[0x08] & 0x07) == 0x07);
}

static void test_compensation(void)
{
    ak_baro_t baro;
    ak_baro_sample_t sample;

    reset_registers();
    /* Coefficients and raw readings chosen to give a sea-level-ish pressure.
     * The expected numbers below are this datasheet equation computed
     * separately, not the output of this code:
     *
     *   P = c00 + p*(c10 + p*(c20 + p*c30)) + t*c01 + t*p*(c11 + p*c21)
     *   T = c0*0.5 + c1*t
     *
     * with p = Praw/253952 and t = Traw/253952 (sixteen-times oversampling). */
    /* The order is the datasheet's: c0, c1, c00, c10, c01, c11, c20, c21,
     * c30. */
    put_coefficients(1500, -1200, 100000, 800, -30, 25, 120, -8, 40);
    put_raw(1000000, 500000);
    (void)ak_baro_open(&baro, &fake_bus, 0);

    expect("a ready part returns a sample",
           ak_baro_read(&baro, &sample) == 1 && sample.valid);
    expect("and the pressure is the one the datasheet's formula gives",
           fabsf(sample.pressure_pa - 107343.77f) < 0.05f);
    expect("with the temperature the same formula gives",
           fabsf(sample.temperature_c - (-1612.6512f)) < 0.01f);

    /* Negative coefficients and negative raw readings, which is where a
     * two's-complement decode that is off by a bit shows up: the coefficients
     * straddle byte boundaries and the raws are twenty-four bits. */
    reset_registers();
    put_coefficients(-980, 1450, 98650, -640, 44, -17, -95, 6, -22);
    put_raw(-250000, -800000);
    (void)ak_baro_open(&baro, &fake_bus, 0);
    expect("negative coefficients and readings decode too",
           ak_baro_read(&baro, &sample) == 1 &&
           fabsf(sample.pressure_pa - 98999.32f) < 0.05f &&
           fabsf(sample.temperature_c - (-5057.7923f)) < 0.01f);

    /* No new measurement is not a failure: a polled part says "nothing yet"
     * most of the time, and treating that as an error would fill the error
     * counter with the normal case. */
    regs[0x08] &= (uint8_t)~0x10u;
    expect("nothing ready is not an error",
           ak_baro_read(&baro, &sample) == 0 && baro.errors == 0 &&
           !sample.valid);

    regs[0x08] |= 0x10u;
    fail_reads = 1;
    expect("but a bus that stops answering is",
           ak_baro_read(&baro, &sample) < 0 && baro.errors == 1);
    fail_reads = 0;
}

/*
 * The altitude, against the curve it approximates.
 *
 * The series is exact arithmetic on an approximation of a standard atmosphere,
 * and the reference here is the closed form with libm's pow(), which is the one
 * thing the image cannot have. If somebody changes a coefficient in the series
 * this fails with the error at the height where it broke.
 */
static void test_altitude_curve(void)
{
    const float reference = 101325.0f;
    float worst = 0.0f;
    float worst_at = 0.0f;

    expect("at the reference pressure the height is zero",
           fabsf(ak_baro_altitude_m(reference, reference)) < 0.001f);

    for (int height = -100; height <= 2000; height += 5) {
        /* The standard atmosphere, inverted: the pressure at that height. */
        float pressure = reference *
                         powf(1.0f - (float)height / 44330.0f, 5.25500f);
        float got = ak_baro_altitude_m(pressure, reference);
        float error = fabsf(got - (float)height);
        if (error > worst) {
            worst = error;
            worst_at = (float)height;
        }
    }

    expect("the series tracks the standard atmosphere over a flight's range",
           worst < 0.10f);
    if (worst >= 0.10f) {
        printf("        worst error %.3f m at %.0f m\n", (double)worst,
               (double)worst_at);
    }

    /* Far above the aircraft, the series is clamped rather than left to
     * diverge: a wrong number is better than an infinity in a control loop. */
    expect("and it stays finite where the series would diverge",
           ak_baro_altitude_m(1000.0f, reference) < 9000.0f &&
           ak_baro_altitude_m(1.0f, reference) < 9000.0f);

    /* A pressure that is not a pressure: zero, or the reference missing. */
    expect("and nonsense in gives zero out rather than a number",
           ak_baro_altitude_m(0.0f, reference) == 0.0f &&
           ak_baro_altitude_m(reference, 0.0f) == 0.0f);
}

/*
 * And the SPL06-003, which is the same part under another name with three more
 * coefficient bytes and one more term in the pressure polynomial.
 *
 * `ak_baro_detect()` has returned it since the driver was written, and the
 * branch has been there just as long, and no test had ever taken it: every
 * sample read above is a DPS310's, so the twelve-bit split of c31 and c40, the
 * sign of a negative one, and the two extra terms of the polynomial had been
 * compiled and never executed. That is the arithmetic a wing's altitude would
 * be flown on if the part fitted to the board were an SPL06 rather than a
 * DPS310 - and the two are pin-compatible, which is exactly why a board can
 * have either.
 *
 * The coefficients are the DPS310 case's, so the only difference in the answer
 * is the two extra terms. The expected pressures are this part's datasheet
 * equation computed separately, the way the numbers above were:
 *
 *   P = c00 + p*(c10 + p*(c20 + p*(c30 + p*c40)))
 *           + t*c01 + t*p*(c11 + p*(c21 + p*c31))
 */
static void test_the_spl06s_extra_terms(void)
{
    ak_baro_t baro;
    ak_baro_sample_t sample;

    /* c31 = -7 (0xFF9) is negative on purpose: it is a twelve-bit field split
     * across two registers, so a missing sign extension - or c31's low four
     * bits and c40's high four read the wrong way round - changes this
     * answer. */
    reset_registers();
    regs[0x0D] = 0x11; /* the SPL06: a coefficient block three bytes longer */
    put_coefficients(1500, -1200, 100000, 800, -30, 25, 120, -8, 40);
    put_spl06_coefficients(-7, 12);
    put_raw(1000000, 500000);

    expect("the spl06 opens",
           ak_baro_open(&baro, &fake_bus, 0) == 0 && baro.present);
    expect("a ready spl06 returns a sample",
           ak_baro_read(&baro, &sample) == 1 && sample.valid);
    expect("and the pressure is the one the eleven-term formula gives",
           fabsf(sample.pressure_pa - 109387.44f) < 0.05f);
    expect("which is not the dps310's shorter formula",
           fabsf(sample.pressure_pa - 107343.77f) > 1.0f);
    expect("with the temperature on the same line as before",
           fabsf(sample.temperature_c - (-1612.6512f)) < 0.01f);

    /* And the other half of the packing: a positive c31, and a c40 whose high
     * four bits are not zero, so the nibble that moves between the two fields
     * carries information in both directions. */
    reset_registers();
    regs[0x0D] = 0x11;
    put_coefficients(1500, -1200, 100000, 800, -30, 25, 120, -8, 40);
    put_spl06_coefficients(9, 300);
    put_raw(1000000, 500000);
    (void)ak_baro_open(&baro, &fake_bus, 0);
    expect("and c31 read from the other half of its byte lands where it should",
           ak_baro_read(&baro, &sample) == 1 &&
               fabsf(sample.pressure_pa - 180555.51f) < 0.05f);
}

/*
 * And the ways the part does not come up, plus the line it prints when it
 * does.
 *
 * Each of the driver's write-and-read steps has a `return -1` behind it, and
 * the opening message names the part that answered - which the boot hands to
 * the console, so it is the first line a person sees about their barometer.
 * None of the three had been reached: the tests above either open a working
 * part with nowhere to print, or fail the very first transaction. A bus that
 * fails *part-way* through the initialisation is the one that leaves a part
 * half-configured, which is worse than one that never answered.
 */
static char baro_said[128];

static int baro_saying(const char *fmt, ...)
{
    va_list ap;
    int n;

    va_start(ap, fmt);
    n = vsnprintf(baro_said, sizeof baro_said, fmt, ap);
    va_end(ap);
    return n;
}

static void test_the_ways_a_barometer_does_not_come_up(void)
{
    ak_baro_t baro;

    /* One register the bus will not answer: the temperature-source read, which
     * is the step that differs between the DPS310 and the SPL06. */
    reset_registers();
    fail_read_register = 0x28;
    expect("a part whose source register will not read does not open",
           ak_baro_open(&baro, &fake_bus, 0) != 0 && !baro.present);

    /* And a bus that takes writes at first and then stops: the third
     * configuration write, inside the chain that sets the rates. */
    reset_registers();
    fail_at_write = 4u;
    expect("a bus that stops taking writes part-way does not open",
           ak_baro_open(&baro, &fake_bus, 0) != 0 && !baro.present);

    /* And the line the boot prints about the part it found - with the sink,
     * because that is the only way it is ever printed. */
    reset_registers();
    baro_said[0] = '\0';
    expect("a part that comes up opens",
           ak_baro_open(&baro, &fake_bus, baro_saying) == 0);
    expect("and says which part it is",
           strstr(baro_said, "baro:      dps310") != 0);

    reset_registers();
    regs[0x0D] = 0x11;
    baro_said[0] = '\0';
    (void)ak_baro_open(&baro, &fake_bus, baro_saying);
    expect("and names the spl06 as itself, which is a different part to order",
           strstr(baro_said, "baro:      spl06-003") != 0);
}

void test_barometer(void)
{
    test_detection();
    test_open();
    test_compensation();
    test_the_spl06s_extra_terms();
    test_the_ways_a_barometer_does_not_come_up();
    test_altitude_curve();
}
