/*
 * The BMP388, against two independent transcriptions of its arithmetic.
 *
 * The interesting half of this driver is that the same datasheet gives the
 * compensation twice - a float variant with scaled coefficients and a
 * fixed-point one in 64-bit integers - and they do *not* come out in the same
 * units. The fixed-point result is hundredths of a pascal; the float one is
 * pascals. Both are right, and a driver that mixed them up would report a
 * pressure a hundred times too high: still "a pressure", still in the right
 * ballpark for no atmosphere, and an altitude out by eleven kilometres.
 *
 * So the numbers below were computed twice, separately, from the two variants,
 * and the check is that they agree to within a hundredth of a pascal. That is
 * what caught the factor of a hundred on the way in - and the Bosch API's own
 * comment ("if pressure is 9528709 which is 9528709/100 = 95287.09 Pascal")
 * is what settled which variant was which.
 *
 * The coefficients are not a real part's: they are shaped like one, and chosen
 * so that a raw reading of 8000000 lands on standard sea level. What is being
 * tested is the arithmetic, not a particular sensor.
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

/* The trimming data as the part would hold it: the temperature five bytes from
 * 0x31, the pressure sixteen from 0x36, little-endian, with P5 and P6
 * unsigned and the rest of the pressure words signed. */
static void put16(unsigned at, uint16_t raw)
{
    fake.regs[at] = (uint8_t)(raw & 0xFFu);
    fake.regs[at + 1u] = (uint8_t)(raw >> 8);
}

static void load_calibration(void)
{
    put16(0x31u, 27200u);  /* T1 */
    put16(0x33u, 26500u);  /* T2 */
    fake.regs[0x35u] = 0xFEu; /* T3 = -2 */

    put16(0x36u, 20000u);  /* P1 */
    put16(0x38u, 13100u);  /* P2 */
    fake.regs[0x3Au] = 0u;      /* P3 */
    fake.regs[0x3Bu] = 0u;      /* P4 */
    put16(0x3Cu, 7833u);   /* P5, unsigned */
    put16(0x3Eu, 26000u);  /* P6, unsigned */
    fake.regs[0x40u] = 0u;      /* P7 */
    fake.regs[0x41u] = 0u;      /* P8 */
    put16(0x42u, 8500u);   /* P9 */
    fake.regs[0x44u] = 0u;      /* P10 */
    fake.regs[0x45u] = 0u;      /* P11 */
}

/* One raw sample: pressure at 0x04, temperature at 0x07, three bytes each,
 * least significant first. */
static void put_sample(uint32_t raw_pressure, uint32_t raw_temperature)
{
    for (unsigned i = 0; i < 3u; i++) {
        fake.regs[0x04u + i] = (uint8_t)((raw_pressure >> (8u * i)) & 0xFFu);
        fake.regs[0x07u + i] = (uint8_t)((raw_temperature >> (8u * i)) & 0xFFu);
    }
}

static void ready(int ready)
{
    fake.regs[0x03u] = ready ? 0x60u : 0x00u;
}

static void setup(void)
{
    memset(&fake, 0, sizeof fake);
    fake.regs[0x00u] = 0x50u; /* chip id */
    load_calibration();
}

void test_bmp388(void)
{
    setup();

    /* The probe is the barometer layer's, and this part answers at 0x50. */
    uint8_t whoami = 0;
    const ak_baro_driver_t *driver = ak_baro_detect(&bus, &whoami);
    expect("a bmp388 is detected by its chip id",
           driver == &ak_baro_bmp388 && whoami == 0x50u);
    expect("and the bmp390 is the same driver under another id",
           (fake.regs[0x00u] = 0x60u,
            ak_baro_detect(&bus, &whoami) == &ak_baro_bmp390));
    fake.regs[0x00u] = 0x50u;

    /* And a part that is not there is not claimed by either of them. */
    fake.regs[0x00u] = 0x58u; /* a BMP280 */
    expect("a chip id this driver does not know is not claimed",
           ak_baro_detect(&bus, &whoami) == 0);
    fake.regs[0x00u] = 0x50u;

    ak_baro_t baro;
    expect("opening the part configures it",
           ak_baro_open(&baro, &bus, 0) == 0 && baro.present);
    expect("with a reset first, as the datasheet asks",
           wrote(0u, 0x7Eu, 0xB6u));
    expect("then the oversampling, and a forced measurement",
           wrote(3u, 0x1Cu, 0x03u) && wrote(4u, 0x1Bu, 0x13u));

    /*
     * The worked sample. Both variants of the datasheet's arithmetic were
     * computed separately for these coefficients and a raw reading of
     * 8000000/8000000: the fixed-point one the driver uses gives 25.58 C and
     * 10132489 hundredths of a pascal, and the float one gives 25.5806 C and
     * 101324.90 Pa. They agree to a hundredth of a pascal, which is the point
     * of the check.
     */
    ready(0);
    ak_baro_sample_t sample;
    memset(&sample, 0, sizeof sample);
    expect("a conversion still running is not a sample",
           ak_baro_read(&baro, &sample) == 0 && !sample.valid);

    put_sample(8000000u, 8000000u);
    ready(1);
    expect("a new sample is compensated",
           ak_baro_read(&baro, &sample) == 1 && sample.valid);
    expect("the temperature is the datasheet's, in degrees",
           fabsf(sample.temperature_c - 25.58f) < 0.01f);
    expect("and the pressure is in pascals, not hundredths of one",
           fabsf(sample.pressure_pa - 101324.89f) < 0.5f);
    expect("which is a pressure an aircraft can fly in",
           sample.pressure_pa > 90000.0f && sample.pressure_pa < 110000.0f);

    /* The next conversion was started, so the part measures while the caller
     * does something else. */
    expect("and the following conversion is already running",
           wrote(fake.writes - 1u, 0x1Bu, 0x13u));

    /* A hundred counts up is a small rise, in the right direction. */
    put_sample(8000100u, 8000000u);
    ready(1);
    expect("more counts is more pressure",
           ak_baro_read(&baro, &sample) == 1 &&
               sample.pressure_pa > 101324.89f &&
               sample.pressure_pa < 101325.5f);

    /* And a colder part reads colder. */
    put_sample(8000000u, 7000000u);
    ready(1);
    expect("a colder raw temperature is a colder temperature",
           ak_baro_read(&baro, &sample) == 1 &&
               fabsf(sample.temperature_c - 0.90f) < 0.01f);

    /*
     * A pressure outside what the part can measure is an error rather than a
     * sample: the altitude arithmetic would turn it into a height, and a
     * height nobody can fly to is worse than no reading.
     */
    /* The full-scale raw reading: about 168000 Pa, which is above what the
     * part measures and above what the Bosch API will report. */
    put_sample(16777215u, 8000000u);
    ready(1);
    expect("a pressure outside the part's range is refused",
           ak_baro_read(&baro, &sample) < 0);

    /* A bus that stops answering is an error, not a stale reading. */
    fake.fail_reads = 1;
    expect("a bus that stops answering is an error",
           ak_baro_read(&baro, &sample) < 0);
    fake.fail_reads = 0;
}
