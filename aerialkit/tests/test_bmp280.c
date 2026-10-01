/*
 * The BMP280, against the datasheet's own worked example.
 *
 * The interesting half of this driver is 64-bit integer arithmetic with shifts
 * of 33 and 47 bits in it, and a narrowing mistake in that arithmetic does not
 * fail - it produces a number that looks like a pressure. So the coefficients
 * and raw readings below are the datasheet's worked example, and the expected
 * outputs were computed separately, in Python, from the same equations.
 *
 * That computation is worth a line of its own: the first version of it used
 * Python's `//`, which floors, where C's `/` truncates toward zero. On these
 * numbers the two differ. The manual's answer for the example - 25.08 degrees
 * and 100653 pascals - is what the checked implementation produces, which is
 * how the discrepancy was found.
 */

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "ak_baro.h"
#include "tests.h"

typedef struct {
    uint8_t  regs[256];
    uint8_t  wrote_reg[24];
    uint8_t  wrote_value[24];
    unsigned writes;
    unsigned reads;
    int      fail_reads;
} fake_t;

static fake_t fake;

static int fake_read(void *ctx, uint8_t reg, uint8_t *buf, unsigned len)
{
    fake_t *bus = ctx;
    bus->reads++;
    if (bus->fail_reads) {
        return -1;
    }
    for (unsigned i = 0; i < len; i++) {
        buf[i] = bus->regs[(reg + i) & 0xFFu];
    }
    return 0;
}

static int fake_write(void *ctx, uint8_t reg, uint8_t value)
{
    fake_t *bus = ctx;
    if (bus->writes < 24u) {
        bus->wrote_reg[bus->writes] = reg;
        bus->wrote_value[bus->writes] = value;
    }
    bus->writes++;
    bus->regs[reg] = value;
    return 0;
}

static void fake_delay(void *ctx, unsigned ms)
{
    (void)ctx;
    (void)ms;
}

static const ak_bus_t bus = {
    .read = fake_read,
    .write = fake_write,
    .delay_ms = fake_delay,
    .ctx = &fake,
};

static int wrote(unsigned index, uint8_t reg, uint8_t value)
{
    return index < fake.writes && fake.wrote_reg[index] == reg &&
           fake.wrote_value[index] == value;
}

/* The datasheet's worked example, coefficient by coefficient, as the twelve
 * little-endian pairs the part would have at 0x88 - written one field at a
 * time, because two of the twelve are unsigned and ten are signed and an
 * array that held all of them would have to be one or the other. */
static void put_field(unsigned at, uint16_t raw)
{
    fake.regs[at] = (uint8_t)(raw & 0xFFu);
    fake.regs[at + 1u] = (uint8_t)(raw >> 8);
}

static void load_coefficients(void)
{
    put_field(0x88, 27504u);              /* T1, unsigned */
    put_field(0x8A, (uint16_t)26435);     /* T2 */
    put_field(0x8C, (uint16_t)-1000);     /* T3 */
    put_field(0x8E, 36477u);              /* P1, unsigned */
    put_field(0x90, (uint16_t)-10685);    /* P2 */
    put_field(0x92, (uint16_t)3024);      /* P3 */
    put_field(0x94, (uint16_t)2855);      /* P4 */
    put_field(0x96, (uint16_t)140);       /* P5 */
    put_field(0x98, (uint16_t)-7);        /* P6 */
    put_field(0x9A, (uint16_t)15500);     /* P7 */
    put_field(0x9C, (uint16_t)-14600);    /* P8 */
    put_field(0x9E, (uint16_t)6000);      /* P9 */
}

static void load_reading(int32_t adc_p, int32_t adc_t)
{
    fake.regs[0xF7] = (uint8_t)((uint32_t)adc_p >> 12);
    fake.regs[0xF8] = (uint8_t)((uint32_t)adc_p >> 4);
    fake.regs[0xF9] = (uint8_t)(((uint32_t)adc_p << 4) & 0xF0u);
    fake.regs[0xFA] = (uint8_t)((uint32_t)adc_t >> 12);
    fake.regs[0xFB] = (uint8_t)((uint32_t)adc_t >> 4);
    fake.regs[0xFC] = (uint8_t)(((uint32_t)adc_t << 4) & 0xF0u);
}

static void reset_bus(uint8_t whoami)
{
    memset(&fake, 0, sizeof fake);
    fake.regs[0xD0] = whoami;
    load_coefficients();
    /* Status: not measuring, so the first read after init has a sample. */
    fake.regs[0xF3] = 0x00;
}

static void test_detection(void)
{
    ak_baro_t baro;

    reset_bus(0x58);
    expect("a bmp280 is detected",
           ak_baro_open(&baro, &bus, 0) == 0 && baro.driver != 0 &&
               strcmp(baro.driver->name, "bmp280") == 0);

    /* The BME280 answers a different chip id in the same register and is the
     * same part for everything this driver does. */
    reset_bus(0x60);
    expect("a bme280 is detected as itself",
           ak_baro_open(&baro, &bus, 0) == 0 && baro.driver != 0 &&
               strcmp(baro.driver->name, "bme280") == 0);

    reset_bus(0x57);
    expect("and a neighbouring chip id is not",
           ak_baro_open(&baro, &bus, 0) != 0);
}

static void test_the_sequence(void)
{
    ak_baro_t baro;

    reset_bus(0x58);
    expect("the part opens", ak_baro_open(&baro, &bus, 0) == 0);

    expect("1. the reset command goes to the reset register",
           wrote(0, 0xE0, 0xB6));
    expect("2. the filter is 8x and not 16x, which is one bit away",
           wrote(1, 0xF5, 0x0Cu));
    expect("3. and a forced measurement of 8x pressure starts",
           wrote(2, 0xF4, 0x31));

    /* The coefficients are read in one go from 0x88: twenty-four bytes, which
     * is twelve two-byte fields. */
    expect("the calibration was read in one burst",
           fake.reads >= 3u);
}

static void test_the_arithmetic(void)
{
    ak_baro_t baro;
    ak_baro_sample_t sample;

    /* The datasheet's example: 25.08 degrees and 100653 pascals. */
    reset_bus(0x58);
    (void)ak_baro_open(&baro, &bus, 0);
    load_reading(415148, 519888);
    expect("the datasheet's worked example reads",
           ak_baro_read(&baro, &sample) == 1 && sample.valid == 1);
    expect("and the temperature is 25.08 degrees",
           fabsf(sample.temperature_c - 25.08f) < 0.005f);
    expect("and the pressure is 100653 pascals, to the quarter pascal the "
           "24.8 fixed point can express",
           fabsf(sample.pressure_pa - 100653.2539f) < 0.5f);

    /* Two more points, one cold and low and one hot and high, computed the
     * same way - because one point cannot tell a wrong shift from a wrong
     * constant that happens to work at room temperature. */
    reset_bus(0x58);
    (void)ak_baro_open(&baro, &bus, 0);
    load_reading(500000, 400000);
    expect("a cold, low reading reads",
           ak_baro_read(&baro, &sample) == 1);
    expect("-12.64 degrees and 81199.7 pascals",
           fabsf(sample.temperature_c + 12.64f) < 0.005f &&
               fabsf(sample.pressure_pa - 81199.6992f) < 0.5f);

    reset_bus(0x58);
    (void)ak_baro_open(&baro, &bus, 0);
    load_reading(300000, 600000);
    expect("a hot, high reading reads",
           ak_baro_read(&baro, &sample) == 1);
    expect("50.11 degrees and 125300.7 pascals",
           fabsf(sample.temperature_c - 50.11f) < 0.005f &&
               fabsf(sample.pressure_pa - 125300.6680f) < 0.5f);

    /* And it is a height a person can check: the standard atmosphere at the
     * example's pressure against sea level is about 120 metres up. */
    expect("the height the core computes from it is plausible",
           fabsf(ak_baro_altitude_m(100653.25f, 101325.0f) - 56.0f) < 2.0f);
}

static void test_the_next_measurement(void)
{
    ak_baro_t baro;
    ak_baro_sample_t sample;

    /* A part that is still converting answers 0 - "nothing new" - and must not
     * hand back the previous frame as though it were fresh. */
    reset_bus(0x58);
    (void)ak_baro_open(&baro, &bus, 0);
    load_reading(415148, 519888);
    fake.regs[0xF3] = 0x08;
    expect("a part that is still measuring has no sample",
           ak_baro_read(&baro, &sample) == 0);

    /* And when it has one, the next measurement is started before the driver
     * returns - so the part converts while the aircraft does something else
     * rather than while the driver waits. */
    fake.regs[0xF3] = 0x00;
    expect("a part that has finished has one",
           ak_baro_read(&baro, &sample) == 1);
    unsigned writes_after = fake.writes;
    expect("and the next measurement was started",
           wrote(writes_after - 1u, 0xF4, 0x31));

    /* A bus that stops answering is an error, not a sample. */
    fake.fail_reads = 1;
    expect("a bus that stops answering is an error",
           ak_baro_read(&baro, &sample) < 0);
}

void test_bmp280(void)
{
    test_detection();
    test_the_sequence();
    test_the_arithmetic();
    test_the_next_measurement();
}
