/*
 * The BMI270.
 *
 * Two things make this driver different from the other three, and both are
 * what the checks are about.
 *
 * The first is that its data is invalid until eight kilobytes of its own
 * firmware have been uploaded, so "did the upload happen" is a question the
 * bus can answer exactly: every byte, in order, and the sum of them held
 * against Bosch's own array. A truncated copy of that array compiles
 * perfectly and would fail this.
 *
 * The second is that the upload goes through ak_bus_write_burst when the
 * transport has one, and one byte at a time when it does not. Both paths are
 * run here and both are held against the same bytes, because the whole point
 * of the optional call is that a driver never finds out which one it got.
 */

#include <stdio.h>
#include <stdarg.h>
#include <string.h>

#include "ak_imu.h"
#include "ak_imu_bmi270_config.h"
#include "tests.h"

/* What a fake bus records about one configuration upload. */
typedef struct {
    uint8_t  regs[256];
    unsigned writes;
    unsigned reads;
    unsigned config_bytes;  /* bytes written to the configuration register */
    uint32_t config_sum;    /* and their sum, which is the whole check */
    unsigned config_calls;  /* how many writes carried them */
    unsigned ctrl_writes;   /* writes to INIT_CTRL, which must happen in order */
    uint8_t  ctrl_value[4];
    unsigned status_reads;
    int      fail_writes;
    int      fail_reads;
    int      fail_burst;    /* the burst only: the file never gets there */
    unsigned fail_at_write;   /* single writes fail from this one onwards */
    int      fail_after_status; /* or the bus dies once the part has spoken */
} fake_t;

static fake_t fake;

static int fake_read(void *ctx, uint8_t reg, uint8_t *buf, unsigned len)
{
    fake_t *bus = ctx;
    bus->reads++;
    if (bus->fail_reads) {
        return -1;
    }
    if (reg == 0x21u) {
        bus->status_reads++;
    }
    for (unsigned i = 0; i < len; i++) {
        buf[i] = bus->regs[(reg + i) & 0xFFu];
    }
    return 0;
}

static void note_write(fake_t *bus, uint8_t reg, uint8_t value)
{
    bus->writes++;
    bus->regs[reg] = value;
    if (reg == 0x5Eu) {
        bus->config_bytes++;
        bus->config_sum += value;
    }
    if (reg == 0x59u && bus->ctrl_writes < 4u) {
        bus->ctrl_value[bus->ctrl_writes] = value;
    }
    if (reg == 0x59u) {
        bus->ctrl_writes++;
    }
}

static int fake_write(void *ctx, uint8_t reg, uint8_t value)
{
    fake_t *bus = ctx;
    if (bus->fail_writes) {
        return -1;
    }
    if (bus->fail_at_write != 0u && bus->writes + 1u >= bus->fail_at_write) {
        return -1;
    }
    if (bus->fail_after_status && bus->status_reads > 0u) {
        return -1;
    }
    note_write(bus, reg, value);
    return 0;
}

static int fake_write_burst(void *ctx, uint8_t reg, const uint8_t *buf,
                            unsigned len)
{
    fake_t *bus = ctx;
    if (bus->fail_writes || bus->fail_burst) {
        return -1;
    }
    bus->config_calls++;
    for (unsigned i = 0; i < len; i++) {
        note_write(bus, reg, buf[i]);
    }
    return 0;
}

static void fake_delay(void *ctx, unsigned ms)
{
    (void)ctx;
    (void)ms;
}

/* Two buses, the same in every way except that one of them can take a burst.
 * A real transport is one or the other; a driver must not be able to tell. */
static const ak_bus_t bus_byte_at_a_time = {
    .read = fake_read,
    .write = fake_write,
    .write_burst = 0,
    .delay_ms = fake_delay,
    .ctx = &fake,
};

static const ak_bus_t bus_with_burst = {
    .read = fake_read,
    .write = fake_write,
    .write_burst = fake_write_burst,
    .delay_ms = fake_delay,
    .ctx = &fake,
};

static void fake_reset(uint8_t whoami, uint8_t status)
{
    memset(&fake, 0, sizeof fake);
    fake.regs[0x00] = whoami;
    fake.regs[0x21] = status;
}

static int open_with(const ak_bus_t *bus, ak_imu_t *imu)
{
    return ak_imu_open(imu, bus, 0);
}

static void test_detection(void)
{
    ak_imu_t imu;

    fake_reset(0x24, 0x01);
    expect("a bmi270 is detected",
           open_with(&bus_with_burst, &imu) == 0 && imu.driver != 0 &&
               strcmp(imu.driver->name, "bmi270") == 0);

    /* The chip id lives at register 0x00, which is not where any of the other
     * three parts keep theirs - the probe is a value at an address, and this
     * is the address. */
    fake_reset(0x23, 0x01);
    expect("and the neighbouring id is not",
           open_with(&bus_with_burst, &imu) != 0);
}

static void test_the_configuration_upload(void)
{
    ak_imu_t imu;
    uint32_t expected_sum = 0;

    for (unsigned i = 0; i < AK_BMI270_CONFIG_SIZE; i++) {
        expected_sum += ak_bmi270_config_file[i];
    }

    /* The published array itself, first: eight kilobytes, starting with 0xc8
     * and ending with 0xc1. Those two bytes are Bosch's, and a copy that lost
     * its tail would still compile. */
    expect("Bosch's configuration file is eight kilobytes",
           AK_BMI270_CONFIG_SIZE == 8192u);
    expect("and it starts and ends where Bosch's does",
           ak_bmi270_config_file[0] == 0xC8u &&
               ak_bmi270_config_file[AK_BMI270_CONFIG_SIZE - 1u] == 0xC1u);

    fake_reset(0x24, 0x01);
    expect("the part configures", open_with(&bus_with_burst, &imu) == 0);

    /* Every byte of the file reached the part, through the register the
     * upload uses, and the sum of what arrived is the sum of what was
     * published. */
    expect("every byte of it was written to the configuration register",
           fake.config_bytes == AK_BMI270_CONFIG_SIZE);
    expect("and the bytes that arrived are the bytes that were published",
           fake.config_sum == expected_sum);
    expect("in one burst rather than eight thousand writes",
           fake.config_calls == 1u);

    /* The transfer is opened and closed around it: INIT_CTRL 0, the file,
     * INIT_CTRL 1. A part that is never told the file has finished being
     * written keeps its data invalid. */
    expect("and the transfer was opened and closed around it",
           fake.ctrl_writes == 2u && fake.ctrl_value[0] == 0x00u &&
               fake.ctrl_value[1] == 0x01u);

    /* The status register is *read*, and the part is asked before any of its
     * data is believed. */
    expect("the part was asked whether it accepted the file",
           fake.status_reads == 1u);

    /* Then the configuration: accelerometer and gyro rates, ranges and the
     * interrupt. Read out of the register file the fake kept, which is what
     * the part would have. */
    expect("accel 1600 Hz in high performance mode",
           fake.regs[0x40] == 0x8C);
    expect("accel range 16 g", fake.regs[0x41] == 0x03);
    expect("gyro 1600 Hz with its noise and filter settings",
           fake.regs[0x42] == 0xCC);
    expect("gyro range 2000 dps", fake.regs[0x43] == 0x08);
    expect("the data-ready interrupt is routed to int1 and enabled",
           fake.regs[0x58] == 0x04 && fake.regs[0x53] == 0x0A);
    expect("and the part is left in performance mode with both sensors on",
           fake.regs[0x7C] == 0x00 && fake.regs[0x7D] == 0x06);
    expect("the soft reset came first", fake.regs[0x7E] == 0xB6);
}

static void test_a_part_that_refuses_the_file(void)
{
    ak_imu_t imu;

    /* Status 0x00 is "the feature engine has not finished starting", which is
     * what a part says when it did not take the upload. Its data is not to be
     * believed, so the driver must refuse the part rather than read zeros from
     * it - zeros are a plausible accelerometer reading at free fall. */
    fake_reset(0x24, 0x00);
    expect("a part that did not take its configuration is refused",
           open_with(&bus_with_burst, &imu) != 0);

    /* And a bus that stops taking writes fails at the upload rather than
     * carrying on to configure a part that has nothing to configure. */
    fake_reset(0x24, 0x01);
    fake.fail_writes = 1;
    expect("and a bus that refuses writes is a part that will not open",
           open_with(&bus_with_burst, &imu) != 0);
}

static void test_the_two_write_paths_agree(void)
{
    ak_imu_t imu;
    uint32_t expected_sum = 0;
    unsigned burst_bytes;
    uint32_t burst_sum;
    unsigned burst_calls;

    for (unsigned i = 0; i < AK_BMI270_CONFIG_SIZE; i++) {
        expected_sum += ak_bmi270_config_file[i];
    }

    fake_reset(0x24, 0x01);
    expect("a bus with no burst write still configures the part",
           open_with(&bus_byte_at_a_time, &imu) == 0);
    burst_bytes = fake.config_bytes;
    burst_sum = fake.config_sum;
    burst_calls = fake.config_calls;

    expect("the same number of bytes went one at a time",
           burst_bytes == AK_BMI270_CONFIG_SIZE);
    expect("and the same bytes",
           burst_sum == expected_sum && burst_sum == expected_sum);
    expect("in eight thousand calls, which is what the burst is for",
           burst_calls == 0u && fake.writes > AK_BMI270_CONFIG_SIZE);
}

/*
 * And a sample, which is the half nothing had ever run.
 *
 * Everything above is about getting the part configured. What the flight loop
 * actually calls is this: one twelve-byte burst, and the arithmetic that turns
 * it into g and radians a second. It had never executed anywhere - the driver's
 * read path was compiled and never called - and a scale or a byte order wrong
 * there is not a broken sensor, it is an aircraft whose attitude estimate is
 * plausible and wrong, which is the kind of fault that survives a bench and
 * ends a flight. So the numbers below are chosen so that each one pins
 * something: a value that is exactly one LSB of the range in use, a *negative*
 * one (which is where a missing sign extension shows), zeros where a wrong
 * register offset would put the gyro, and a gyro reading at full scale.
 */
static void test_a_sample(void)
{
    ak_imu_t imu;
    ak_imu_sample_t sample;

    fake_reset(0x24, 0x01); /* the id, and "the configuration was accepted" */
    expect("the part opens before anything is read from it",
           open_with(&bus_with_burst, &imu) == 0);

    /* Accel x = 0x0800 = 2048, which is 1 g at this range's 2048 LSB/g.
     * Accel y = 0xFC00 = -1024, which is half a g the other way.
     * Accel z = 0. Gyro x and y = 0, gyro z = 0x4000 = 16384, which is
     * 1000 deg/s at 16.384 LSB/dps. */
    fake.regs[0x0C] = 0x00u; fake.regs[0x0D] = 0x08u;
    fake.regs[0x0E] = 0x00u; fake.regs[0x0F] = 0xFCu;
    fake.regs[0x10] = 0x00u; fake.regs[0x11] = 0x00u;
    fake.regs[0x12] = 0x00u; fake.regs[0x13] = 0x00u;
    fake.regs[0x14] = 0x00u; fake.regs[0x15] = 0x00u;
    fake.regs[0x16] = 0x00u; fake.regs[0x17] = 0x40u;

    expect("a sample reads, and is marked valid",
           ak_imu_read(&imu, &sample) == 0 && sample.valid == 1);
    expect("the accelerometer is in g, little-endian and signed",
           sample.accel[0] > 0.9999f && sample.accel[0] < 1.0001f &&
               sample.accel[1] > -0.5001f && sample.accel[1] < -0.4999f &&
               sample.accel[2] > -0.0001f && sample.accel[2] < 0.0001f);
    expect("the gyroscope is in radians a second",
           sample.gyro[0] > -0.0001f && sample.gyro[0] < 0.0001f &&
               sample.gyro[1] > -0.0001f && sample.gyro[1] < 0.0001f &&
               sample.gyro[2] > 17.4532f && sample.gyro[2] < 17.4534f);
    expect("and the read is counted, with nothing counted as an error",
           imu.samples == 1u && imu.errors == 0u);

    /*
     * And a bus that does not answer: the driver must not hand the flight loop
     * half a sample. `valid` is what the loop looks at - the record above is
     * left as it was, so a stale attitude is impossible to mistake for a new
     * one - and the error is counted, because "how often does this happen" is
     * the question a log is asked after a flight.
     */
    fake.fail_reads = 1;
    sample.valid = 1;
    expect("a bus that does not answer is an error, not a sample",
           ak_imu_read(&imu, &sample) < 0 && sample.valid == 0);
    expect("and it is counted as one",
           imu.samples == 1u && imu.errors == 1u);
    fake.fail_reads = 0;
    expect("and the part is read again as soon as it answers",
           ak_imu_read(&imu, &sample) == 0 && sample.valid == 1 &&
               sample.accel[0] > 0.9999f && sample.accel[0] < 1.0001f);
}

/*
 * And the lines a person reads when the part will not open.
 *
 * `ak_imu_open()` takes a sink and the boot hands it the console, so a BMI270
 * that took the file and then says in its status register that it did not - or
 * one whose bus refused the file - prints a line naming the part and, where it
 * matters, the status byte. Both of those branches need a sink to be reached at
 * all, and every test above opens the part with none: the messages were
 * compiled, and no test had ever seen one come out. On a bench that message is
 * the whole diagnosis - a part that answers its identity register and then
 * reads zeros is otherwise indistinguishable from a part that works and is
 * sitting still.
 */
static char said[256];

static int saying(const char *fmt, ...)
{
    va_list ap;
    size_t used = strlen(said);
    int n;

    if (used >= sizeof said - 1u) {
        return 0;
    }
    va_start(ap, fmt);
    /* Appended, not replaced: `ak_imu_open()` prints its own line *after* the
     * driver's, so a sink that overwrote would throw away the sentence this
     * test is about. */
    n = vsnprintf(said + used, sizeof said - used, fmt, ap);
    va_end(ap);
    return n;
}

static void test_the_messages_an_imu_that_will_not_open_prints(void)
{
    ak_imu_t imu;

    /* The file went; the part says it did not take it. */
    said[0] = '\0';
    fake_reset(0x24, 0x00);
    expect("a part that did not take its configuration does not open",
           ak_imu_open(&imu, &bus_with_burst, saying) != 0);
    expect("and it says so, with the status byte a person can look up",
           strstr(said, "did not accept its configuration (status 0x00)") != 0);

    /* The file never got there: the burst write failed and nothing else did. */
    said[0] = '\0';
    fake_reset(0x24, 0x01);
    fake.fail_burst = 1;
    expect("a part whose bus refuses the file does not open",
           ak_imu_open(&imu, &bus_with_burst, saying) != 0);
    expect("and that is a different sentence from the one above",
           strstr(said, "bmi270 took no configuration file") != 0);

    /* And a bus that will not even answer the status read. */
    said[0] = '\0';
    fake_reset(0x24, 0x01);
    fake.fail_reads = 1;
    expect("a part whose status cannot be read does not open",
           ak_imu_open(&imu, &bus_with_burst, saying) != 0);

    /* A bus that dies before the part is even woken, and one that dies after
     * it answered the status read - the second write each half of the boot-up
     * makes, and the difference between "nothing works" and "the part is up
     * and the wire is not". */
    fake_reset(0x24, 0x01);
    fake.fail_at_write = 2u; /* the soft reset went, the wake-up did not */
    expect("a bus that fails after the soft reset does not open",
           ak_imu_open(&imu, &bus_with_burst, 0) != 0);

    fake_reset(0x24, 0x01);
    fake.fail_after_status = 1;
    expect("a bus that fails once the part has spoken does not open",
           ak_imu_open(&imu, &bus_with_burst, 0) != 0);

    /* And the line the boot prints when it does come up, which is the only
     * positive statement an IMU makes about itself on a bench. */
    said[0] = '\0';
    fake_reset(0x24, 0x01);
    expect("a part that takes its configuration opens",
           ak_imu_open(&imu, &bus_with_burst, saying) == 0);
    expect("and says what it was configured for",
           strstr(said, "configured for 1600 Hz") != 0);
}

void test_bmi270(void)
{
    test_detection();
    test_the_configuration_upload();
    test_a_part_that_refuses_the_file();
    test_the_two_write_paths_agree();
    test_a_sample();
    test_the_messages_an_imu_that_will_not_open_prints();
}
