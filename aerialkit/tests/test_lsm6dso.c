/*
 * The LSM6DSO, over I2C.
 *
 * This is the first IMU driver in this repository that is not a SPI one, and
 * the check that matters most is the one a SPI reference could not have
 * written: CTRL4_C's I2C_DISABLE bit. Both references set it. On this part, on
 * this bus, that is the bit that switches off the wire the next write arrives
 * on - so the test starts the part with the bit already set, the way a part
 * that a previous SPI sketch configured would be, and holds the driver to
 * clearing it.
 *
 * The other half is the read. The LSM6DSO puts its gyro at a lower address than
 * its accelerometer and its bytes the other way up from the MPU-6000, so a
 * driver edited from the MPU one would be wrong twice in ways that produce
 * plausible numbers. The values below are chosen so that each one pins
 * something: exactly one LSB of the range in use, a negative one, which is
 * where a missing sign extension shows, a zero where a wrong register offset
 * would put the other sensor's axis, and a gyro reading at a third of full
 * scale.
 */

#include <stdio.h>
#include <stdarg.h>
#include <string.h>

#include "ak_imu.h"
#include "tests.h"

#define FAKE_LOG 16

/* What the fake part records about the sequence it was put through. */
typedef struct {
    uint8_t  regs[256];
    unsigned writes;
    unsigned ctrl_reads;          /* reads of the four control registers */
    unsigned reset_writes;        /* writes of SW_RESET, which must be one */
    unsigned delay_calls;
    unsigned ms_after_reset;      /* the wait that follows the reset write */
    unsigned pending_reset;
    uint8_t  log_reg[FAKE_LOG];   /* the first writes, in order */
    uint8_t  log_value[FAKE_LOG];
    unsigned logged;
    int      fail_reads;
    int      fail_writes;
    unsigned fail_at_write;
} fake_t;

static fake_t fake;

static int is_ctrl(uint8_t reg)
{
    return reg == 0x12u || reg == 0x13u || reg == 0x15u || reg == 0x18u;
}

static int fake_read(void *ctx, uint8_t reg, uint8_t *buf, unsigned len)
{
    fake_t *bus = ctx;
    if (bus->fail_reads) {
        return -1;
    }
    if (is_ctrl(reg)) {
        bus->ctrl_reads++;
    }
    for (unsigned i = 0; i < len; i++) {
        buf[i] = bus->regs[(reg + i) & 0xFFu];
    }
    return 0;
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

    bus->writes++;
    if (bus->logged < FAKE_LOG) {
        bus->log_reg[bus->logged] = reg;
        bus->log_value[bus->logged] = value;
        bus->logged++;
    }
    if (reg == 0x12u && (value & 0x01u) != 0u) {
        bus->reset_writes++;
        bus->pending_reset = 1u;
        /* SW_RESET is a strobe: the part clears it itself, so a read after the
         * reset sees it clear. The model has to do that, or the driver's later
         * read-modify-write of this register would hand the reset bit back -
         * which is not a fault the part can have, and a model that invented it
         * would fail a driver that is right. */
        value = (uint8_t)(value & (uint8_t)~0x01u);
    }
    bus->regs[reg] = value;
    return 0;
}

static void fake_delay(void *ctx, unsigned ms)
{
    fake_t *bus = ctx;
    bus->delay_calls++;
    if (bus->pending_reset) {
        bus->pending_reset = 0;
        bus->ms_after_reset = ms;
    }
}

static const ak_bus_t bus = {
    .read = fake_read,
    .write = fake_write,
    .write_burst = 0,
    .delay_ms = fake_delay,
    .ctx = &fake,
};

static void fake_reset(uint8_t whoami)
{
    memset(&fake, 0, sizeof fake);
    fake.regs[0x0Fu] = whoami;
}

static int open_imu(ak_imu_t *imu)
{
    return ak_imu_open(imu, &bus, 0);
}

static void test_detection(void)
{
    const ak_imu_driver_t *found;
    uint8_t seen = 0;

    fake_reset(0x6C);
    found = ak_imu_detect(&bus, &seen);
    /* By name rather than by symbol: the name is what the boot prints on a
     * bench, and it is the one thing a person reading a console has to go on. */
    expect("an lsm6dso is detected",
           found != 0 && strcmp(found->name, "lsm6dso") == 0);

    /* The id lives at 0x0F, which is where the LSM6DSL and the LSM6DS3 keep
     * theirs too - with 0x6A and 0x69 in it. Neither is this driver's part:
     * the references configure the DSL with a different CTRL6_C mask, so it is
     * a second initialisation and not a second name. A DSL on the bench has to
     * be refused here rather than configured with the wrong mask. */
    fake_reset(0x6A);
    expect("and the LSM6DSL next to it is not",
           ak_imu_detect(&bus, &seen) == 0 && seen == 0x6Au);

    fake_reset(0x69);
    expect("nor the LSM6DS3",
           ak_imu_detect(&bus, &seen) == 0 && seen == 0x69u);
}

static void test_the_configuration(void)
{
    ak_imu_t imu;

    fake_reset(0x6C);
    expect("the part opens", open_imu(&imu) == 0);

    /* The reset is the first thing written, and the hundred milliseconds it
     * asks for follows it. Every read-modify-write below preserves the bits
     * this driver does not own, and "the bits it does not own are the reset
     * ones" is only true because of these two lines. */
    expect("the soft reset is the first write",
           fake.logged > 0u && fake.log_reg[0] == 0x12u &&
               (fake.log_value[0] & 0x01u) != 0u);
    expect("and the hundred milliseconds it asks for is waited",
           fake.reset_writes == 1u && fake.ms_after_reset == 100u);

    expect("accel 833 Hz, 16 g, output from LPF1",
           fake.regs[0x10] == ((0x07u << 4) | (0x01u << 2) | (0x00u << 1)));
    expect("gyro 6664 Hz at 2000 dps",
           fake.regs[0x11] == ((0x0Au << 4) | (0x03u << 2)));
    expect("the outputs latch during a burst and the address increments",
           (fake.regs[0x12] & 0x44u) == 0x44u);
    expect("the gyro's LPF1 is on", (fake.regs[0x13] & 0x02u) == 0x02u);
    expect("the accelerometer is in high performance mode and the gyro "
           "filter is at 335 Hz",
           (fake.regs[0x15] & 0x17u) == 0x00u);
    expect("the I3C mode is off", (fake.regs[0x18] & 0x02u) == 0x02u);

    expect("the gyro's data-ready is on interrupt 1",
           fake.regs[0x0D] == 0x02u);
    /* Betaflight writes 0x02 here under a comment that says "disable". Bit 1
     * is the gyro's data-ready enable, so 0x02 is the opposite of what the
     * comment says; this driver writes the value INAV's header names as the
     * disable. */
    expect("and interrupt 2 is left disabled, not enabled",
           fake.regs[0x0E] == 0x00u);
}

static void test_the_bus_the_part_is_reached_on(void)
{
    ak_imu_t imu;

    /* A part a previous SPI configuration left with its I2C interface switched
     * off - which is what both references would leave behind, and what a
     * breakout moved from a SPI sketch to a Qwiic cable would be. */
    fake_reset(0x6C);
    fake.regs[0x13] = 0x04u; /* CTRL4_C: I2C_DISABLE set */
    expect("a part left with I2C_DISABLE set still opens",
           open_imu(&imu) == 0);
    expect("and the bit this driver is talking over is clear afterwards",
           (fake.regs[0x13] & 0x04u) == 0x00u);
}

static void test_what_the_read_modify_writes_preserve(void)
{
    ak_imu_t imu;

    /* Bits the masks do not cover are the part's, and must come back
     * unchanged: CTRL3_C bit 7 (BOOT, read-only), CTRL6_C bit 3, and CTRL9_XL
     * bit 7. A driver that wrote constants would clear all three, and nothing
     * on a bench would notice until one of them mattered. */
    fake_reset(0x6C);
    fake.regs[0x12] = 0x80u;
    fake.regs[0x15] = 0x08u;
    fake.regs[0x18] = 0x80u;
    expect("the part opens", open_imu(&imu) == 0);

    expect("CTRL3_C keeps its BOOT bit and gains the two it is owed",
           fake.regs[0x12] == 0xC4u);
    expect("CTRL6_C keeps the bit its mask does not name",
           fake.regs[0x15] == 0x08u);
    expect("CTRL9_XL keeps its top bit",
           fake.regs[0x18] == 0x82u);

    /* Four registers are read-modify-written and no others: the reset is one
     * of them, so five reads of the control registers in all. A driver that
     * had grown a sixth, or that had started reading something it then threw
     * away, shows up here. */
    expect("exactly four registers were read before being written",
           fake.ctrl_reads == 5u && fake.writes == 9u);
}

/*
 * And a sample - the half that turns bytes into an attitude.
 *
 * Gyro at 0x22 and accelerometer at 0x28, which is the MPU driver's order the
 * other way round, and both little endian, which is the MPU's byte order the
 * other way round. Every one of those four mistakes has a plausible-looking
 * output, so each axis below is a value that a swap or a byte-order error
 * cannot produce by accident.
 */
static void test_a_sample(void)
{
    ak_imu_t imu;
    ak_imu_sample_t sample;

    fake_reset(0x6C);
    expect("the part opens before anything is read from it",
           open_imu(&imu) == 0);

    /* Gyro x = 0. Gyro y = 0xFC00 = -1024, which is -71.68 dps at
     * 0.070 dps/LSB. Gyro z = 0x4000 = 16384, or 1146.88 dps.
     * Accel x = 0x0800 = 2048, which is 1 g at this range's 2048 LSB/g.
     * Accel y = 0xFC00 = -1024, half a g the other way. Accel z = 0. */
    fake.regs[0x22] = 0x00u; fake.regs[0x23] = 0x00u;
    fake.regs[0x24] = 0x00u; fake.regs[0x25] = 0xFCu;
    fake.regs[0x26] = 0x00u; fake.regs[0x27] = 0x40u;
    fake.regs[0x28] = 0x00u; fake.regs[0x29] = 0x08u;
    fake.regs[0x2A] = 0x00u; fake.regs[0x2B] = 0xFCu;
    fake.regs[0x2C] = 0x00u; fake.regs[0x2D] = 0x00u;

    expect("a sample reads, and is marked valid",
           ak_imu_read(&imu, &sample) == 0 && sample.valid == 1);
    expect("the gyroscope is little-endian, signed and in radians a second",
           sample.gyro[0] > -0.0001f && sample.gyro[0] < 0.0001f &&
               sample.gyro[1] > -1.2521f && sample.gyro[1] < -1.2501f &&
               sample.gyro[2] > 20.0158f && sample.gyro[2] < 20.0178f);
    expect("the accelerometer is in g, little-endian and signed",
           sample.accel[0] > 0.9999f && sample.accel[0] < 1.0001f &&
               sample.accel[1] > -0.5001f && sample.accel[1] < -0.4999f &&
               sample.accel[2] > -0.0001f && sample.accel[2] < 0.0001f);
    expect("and the read is counted, with nothing counted as an error",
           imu.samples == 1u && imu.errors == 0u);

    /* A bus that stops answering must not hand the flight loop half a sample.
     * `valid` is what the loop looks at, and the record is left as it was, so
     * a stale attitude cannot be mistaken for a new one. */
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

static void test_a_part_that_will_not_configure(void)
{
    ak_imu_t imu;

    /* A bus that refuses writes is a part that never gets configured, and a
     * part that never gets configured is refused rather than read from. */
    fake_reset(0x6C);
    fake.fail_writes = 1;
    expect("a bus that refuses writes is a part that will not open",
           open_imu(&imu) != 0);

    /* And one that refuses only reads: the write of the reset is itself a
     * read-modify-write, so this fails at the first thing the driver does. */
    fake_reset(0x6C);
    fake.fail_reads = 1;
    expect("so is a bus that will not answer a read",
           open_imu(&imu) != 0);

    /* And a bus that dies partway, after the reset and before the last write -
     * the half-open case, where the part is neither working nor untouched. */
    fake_reset(0x6C);
    fake.fail_at_write = 8u;
    expect("and one that dies partway through the configuration",
           open_imu(&imu) != 0);
}

/*
 * And the two lines a person reads on a bench: the one the boot prints when
 * the part comes up, and the one it prints when something else answered on the
 * wire. The second is the whole diagnosis for a Qwiic cable plugged into the
 * wrong header - a part that answers 0x6A is a DSL, and a part that answers
 * nothing is a cable.
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
    /* Appended, not replaced: `ak_imu_open()` prints its own line after the
     * driver's, so a sink that overwrote would throw away the sentence this
     * test is about. */
    n = vsnprintf(said + used, sizeof said - used, fmt, ap);
    va_end(ap);
    return n;
}

static void test_the_messages_a_bench_session_reads(void)
{
    ak_imu_t imu;

    said[0] = '\0';
    fake_reset(0x6C);
    expect("a part that configures opens",
           ak_imu_open(&imu, &bus, saying) == 0);
    expect("and the boot names the wire it is on",
           strstr(said, "lsm6dso") != 0 &&
               strstr(said, "over I2C") != 0);

    /* The neighbour from the same family, which this driver deliberately does
     * not claim. A person who has soldered the wrong breakout on gets the id
     * and nothing else, which is the point: the id is what identifies it. */
    said[0] = '\0';
    fake_reset(0x6A);
    expect("a part this driver does not claim does not open",
           ak_imu_open(&imu, &bus, saying) != 0);
    expect("and says which id answered",
           strstr(said, "answered 0x6a") != 0);

    /* And nothing at all on the bus: an unplugged cable, which on a dev board
     * with a Qwiic connector is the first thing to check. */
    said[0] = '\0';
    fake_reset(0x00);
    expect("an empty bus does not open", ak_imu_open(&imu, &bus, saying) != 0);
    expect("and says so rather than naming a part",
           strstr(said, "nothing answered") != 0);
}

void test_lsm6dso(void)
{
    test_detection();
    test_the_configuration();
    test_the_bus_the_part_is_reached_on();
    test_what_the_read_modify_writes_preserve();
    test_a_sample();
    test_a_part_that_will_not_configure();
    test_the_messages_a_bench_session_reads();
}
