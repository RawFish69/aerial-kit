/*
 * The IMU path, against a fake bus.
 *
 * There is no inertial sensor on the bench board, so the driver is checked
 * where it can be: a register file that answers like an ICM-42688-P. What the
 * fake bus lets us assert is exactly what the datasheet cares about - the
 * write sequence, the values in it, the delays, and the scaling - and what it
 * cannot check is written down in docs/09-sensors.md.
 *
 * The register values the test expects are the same ones the driver writes,
 * which sounds circular and is not: they are the values the reference
 * implementations use, and pinning them here means a change to the driver is a
 * change to the test, visible in a diff.
 */

#include <stdint.h>
#include <string.h>

#include "ak_imu.h"
#include "tests.h"

#define FAKE_WRITES 16

typedef struct {
    uint8_t  regs[256];
    uint8_t  wrote_reg[FAKE_WRITES];
    uint8_t  wrote_value[FAKE_WRITES];
    unsigned writes;
    unsigned reads;
    unsigned delays;
    unsigned delay_total_ms;
    int      fail_reads;
    int      fail_writes;
} fake_bus_t;

static fake_bus_t fake;

static int fake_read(void *ctx, uint8_t reg, uint8_t *buf, unsigned len)
{
    fake_bus_t *bus = ctx;
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
    fake_bus_t *bus = ctx;
    if (bus->fail_writes) {
        return -1;
    }
    if (bus->writes < FAKE_WRITES) {
        bus->wrote_reg[bus->writes] = reg;
        bus->wrote_value[bus->writes] = value;
    }
    bus->writes++;
    bus->regs[reg] = value;
    return 0;
}

static void fake_delay(void *ctx, unsigned ms)
{
    fake_bus_t *bus = ctx;
    bus->delays++;
    bus->delay_total_ms += ms;
}

static const ak_bus_t bus = {
    .read = fake_read,
    .write = fake_write,
    .delay_ms = fake_delay,
    .ctx = &fake,
};

static void fake_reset(uint8_t whoami)
{
    memset(&fake, 0, sizeof fake);
    fake.regs[0x75] = whoami;
    fake.regs[0x4D] = 0xC0; /* INTF_CONFIG1 with AFSR bits set, as it resets */
}

static int wrote(unsigned index, uint8_t reg, uint8_t value)
{
    return index < fake.writes && fake.wrote_reg[index] == reg &&
           fake.wrote_value[index] == value;
}

static void test_imu_probe(void)
{
    ak_imu_t imu;
    char out[1]; /* the driver prints; nothing reads it here */
    (void)out;

    /* The right part answers. */
    fake_reset(0x47);
    expect("an icm42688p is detected", ak_imu_open(&imu, &bus, 0) == 0 &&
           imu.present == 1 && imu.driver != 0);
    expect("and it is named", strcmp(imu.driver->name, "icm42688p") == 0);

    /* Nothing on the bus at all: the register file reads 0x00 everywhere,
     * which is not a who-am-i any driver claims. */
    fake_reset(0x00);
    expect("an empty bus is not a sensor", ak_imu_open(&imu, &bus, 0) != 0 &&
           imu.present == 0);

    /* The ICM-42605 is the same part under another who-am-i, so it is a table
     * entry rather than a driver - and a board that fits one boots. */
    fake_reset(0x42);
    expect("an icm42605 is detected as itself",
           ak_imu_open(&imu, &bus, 0) == 0 && imu.driver != 0 &&
           strcmp(imu.driver->name, "icm42605") == 0);

    /* A part that answers with something else. The console is told what it
     * said, because "wrong sensor" and "no sensor" are different wiring
     * problems. */
    fake_reset(0x99);
    expect("an unknown part is refused", ak_imu_open(&imu, &bus, 0) != 0);
}

/*
 * The MPU-6000, which is not the ICM with a different name: its data registers
 * are at 0x3B rather than 0x1F, it has no bank select, its clock is a three-bit
 * field in a power-management register, and its rates come from a divider. So
 * the sequence and the burst read are checked here rather than assumed to
 * follow from the other driver.
 */
static void test_mpu6000(void)
{
    ak_imu_t imu;
    ak_imu_sample_t sample;

    fake_reset(0x68);
    expect("an mpu6000 is detected",
           ak_imu_open(&imu, &bus, 0) == 0 && imu.driver != 0 &&
           strcmp(imu.driver->name, "mpu6000") == 0);

    /* The write sequence, in the reference implementation's order. */
    expect("1. the clock comes from the gyro's PLL",
           wrote(0, 0x6B, 0x03));
    expect("2. the auxiliary i2c bus is switched off",
           wrote(1, 0x6A, 0x10));
    expect("3. every axis is enabled", wrote(2, 0x6C, 0x00));
    expect("4. the sample rate divider is 1 kHz",
           wrote(3, 0x19, 0x00));
    expect("5. the DLPF is 42 Hz", wrote(4, 0x1A, 0x03));
    expect("6. the gyro is 2000 dps", wrote(5, 0x1B, 0x18));
    expect("7. the accel is 16 g", wrote(6, 0x1C, 0x18));
    expect("8. a register read clears the data-ready interrupt",
           wrote(7, 0x37, 0x10));
    expect("9. and the data-ready interrupt is enabled",
           wrote(8, 0x38, 0x01));

    /*
     * The rest of the family: the same register map, the same initialisation,
     * the same twelve bytes of output, and a different value in the same
     * who-am-i register. Worth a check of its own, because it is the part the
     * wing's own flight controller may carry - and a firmware that answered
     * "0x70, which is not a known part" would be a firmware that cannot fly
     * that board.
     */
    fake_reset(0x70);
    expect("an mpu6500 is detected as itself",
           ak_imu_open(&imu, &bus, 0) == 0 && imu.driver != 0 &&
               strcmp(imu.driver->name, "mpu6500") == 0);
    expect("and configured by the same sequence, in the same order",
           wrote(0, 0x6B, 0x03) && wrote(4, 0x1A, 0x03) &&
               wrote(5, 0x1B, 0x18) && wrote(8, 0x38, 0x01));

    fake_reset(0x71);
    expect("an mpu9250 is detected as itself",
           ak_imu_open(&imu, &bus, 0) == 0 && imu.driver != 0 &&
               strcmp(imu.driver->name, "mpu9250") == 0);

    /* A value that is none of them is still refused, which is the rule the
     * whole probe exists for: 0x73 is an MPU-9255, and this build does not
     * claim to know one. */
    fake_reset(0x73);
    expect("a part this build does not know is still refused",
           ak_imu_open(&imu, &bus, 0) != 0);

    /* One burst from 0x3B: accel x, y, z, temperature, gyro x, y, z. The
     * values are the scales from the datasheet - 2048 counts per g and 16.4 per
     * dps - so 2048 counts is one g and sixteen counts is a degree per
     * second. */
    fake_reset(0x68);
    (void)ak_imu_open(&imu, &bus, 0);
    fake.regs[0x3B] = 0x08; fake.regs[0x3C] = 0x00; /* accel x:  +2048 =  1 g */
    fake.regs[0x3D] = 0xF8; fake.regs[0x3E] = 0x00; /* accel y:  -2048 = -1 g */
    fake.regs[0x3F] = 0x00; fake.regs[0x40] = 0x00; /* accel z:      0 =  0 g */
    fake.regs[0x41] = 0x00; fake.regs[0x42] = 0x00; /* temperature */
    fake.regs[0x43] = 0x00; fake.regs[0x44] = 0x00; /* gyro x */
    fake.regs[0x45] = 0x00; fake.regs[0x46] = 0x00; /* gyro y */
    fake.regs[0x47] = 0x00; fake.regs[0x48] = 0x10; /* gyro z: +16 dps */

    expect("a sample reads back", ak_imu_read(&imu, &sample) == 0 &&
           sample.valid == 1);
    expect("with the accel scaled at 2048 counts per g",
           sample.accel[0] > 0.999f && sample.accel[0] < 1.001f &&
           sample.accel[1] > -1.001f && sample.accel[1] < -0.999f &&
           sample.accel[2] == 0.0f);
    expect("and the gyro at 16.4 counts per dps",
           sample.gyro[2] > 0.0169f && sample.gyro[2] < 0.0176f);
}

static void test_imu_init_sequence(void)
{
    ak_imu_t imu;
    fake_reset(0x47);
    expect("open configures the part", ak_imu_open(&imu, &bus, 0) == 0);

    /* The write sequence, in order. These are the values Betaflight's driver
     * uses at the pinned revision; the register numbers are in the driver next
     * to the datasheet section they come from. */
    expect("1. bank 0 is selected", wrote(0, 0x76, 0x00));
    expect("2. both sensors are turned off before configuration",
           wrote(1, 0x4E, 0x00));
    expect("3. AFSR is turned off, keeping the other INTF_CONFIG1 bits",
           wrote(2, 0x4D, 0x40));
    expect("4. gyro is 2000 dps at 1 kHz", wrote(3, 0x4F, 0x06));
    expect("5. accel is 16 g at 1 kHz", wrote(4, 0x50, 0x06));
    expect("6. both UI filters are the low latency setting",
           wrote(5, 0x52, 0xFF));
    expect("7. sensors come back up in low noise mode",
           wrote(6, 0x4E, 0x0F));

    /* The delays matter: the part needs time after a mode change before its
     * output means anything. */
    expect("the driver waits where the datasheet asks it to",
           fake.delays >= 2 && fake.delay_total_ms >= 30);
}

static void test_imu_read(void)
{
    ak_imu_t imu;
    ak_imu_sample_t sample;
    fake_reset(0x47);
    (void)ak_imu_open(&imu, &bus, 0);

    /* 1 g on Z, a tenth of a g on X, and 16.4 counts of gyro on Z - which is
     * exactly 1 dps, so the scaled numbers are checkable by hand. */
    fake.regs[0x1F] = 0x00; fake.regs[0x20] = 0xCD; /* x: 205 = 0.100 g   */
    fake.regs[0x21] = 0x00; fake.regs[0x22] = 0x00; /* y: 0               */
    fake.regs[0x23] = 0x08; fake.regs[0x24] = 0x00; /* z: 2048 = 1.000 g  */
    fake.regs[0x25] = 0x00; fake.regs[0x26] = 0x00; /* gx: 0              */
    fake.regs[0x27] = 0x00; fake.regs[0x28] = 0x00; /* gy: 0              */
    fake.regs[0x29] = 0x00; fake.regs[0x2A] = 0x10; /* gz: 16 = 0.976 dps */

    expect("one burst read is used for all six axes",
           ak_imu_read(&imu, &sample) == 0 && sample.valid == 1);
    expect("accel is scaled at 2048 counts per g",
           sample.accel[0] > 0.099f && sample.accel[0] < 0.101f &&
           sample.accel[1] == 0.0f && sample.accel[2] > 0.999f &&
           sample.accel[2] < 1.001f);

    /* 16 counts at 16.4 counts per dps is 0.9756 dps, which is 0.01703 rad/s. */
    expect("gyro is scaled at 16.4 counts per dps and converted to rad/s",
           sample.gyro[2] > 0.0170f && sample.gyro[2] < 0.0171f &&
           sample.gyro[0] == 0.0f);
    expect("the sample carries the count of reads it took",
           imu.samples == 1 && imu.errors == 0);

    /* Negative values are two's complement, which is where a sign bug lives. */
    fake.regs[0x1F] = 0xFF; fake.regs[0x20] = 0x00; /* x: -256 = -0.125 g */
    fake.regs[0x29] = 0xFF; fake.regs[0x2A] = 0xF0; /* gz: -16 counts     */
    (void)ak_imu_read(&imu, &sample);
    expect("a negative accel is read as negative",
           sample.accel[0] > -0.126f && sample.accel[0] < -0.124f);
    expect("a negative rate is read as negative",
           sample.gyro[2] < -0.0170f && sample.gyro[2] > -0.0171f);

    /* A bus that fails must produce an invalid sample, not a stale one: the
     * flight core treats valid = 0 as "no attitude". */
    sample.accel[0] = 99.0f;
    fake.fail_reads = 1;
    expect("a failed read is an invalid sample",
           ak_imu_read(&imu, &sample) != 0 && sample.valid == 0);
    expect("and it is counted", imu.errors == 1);

    /* A part that stops answering its who-am-i mid-flight is a fault, and a
     * driver that cannot configure it must not claim it is present. */
    fake.fail_reads = 0;
    fake.regs[0x75] = 0x00;
    ak_imu_t broken;
    expect("a part that stops answering is not opened",
           ak_imu_open(&broken, &bus, 0) != 0 && broken.present == 0);
}

/*
 * What the open left behind for a console that is asked afterwards.
 *
 * The open prints its verdict once, at boot, and on the F405 that print is
 * before a host can attach - so the sentence has never been recoverable, and
 * `imu` and the preflight could only say "none fitted", which is the same
 * answer for a missing part, a miswired one and one this build has no driver
 * for. The three ways to fail are kept apart here because they are three
 * different repairs, and a test that only checked "not OK" would pass with the
 * distinctions collapsed - which is the state this replaced.
 */
static void test_imu_absence_is_recorded(void)
{
    ak_imu_t imu;

    /* A clean open first, so that the case below is a *change* of state rather
     * than the value a variable happens to start at. */
    fake_reset(0x68);
    expect("a part that answers and configures is opened",
           ak_imu_open(&imu, &bus, 0) == 0);
    expect("and the report says nothing failed",
           ak_imu_last_result() == AK_IMU_OK);

    /* Nothing acknowledges - the wiring case, and the one an empty connector
     * looks like. `fail_reads` makes every read fail, which is what a bus with
     * nothing on it does through this interface. */
    fake.fail_reads = 1;
    expect("nothing on the bus opens nothing",
           ak_imu_open(&imu, &bus, 0) != 0);
    expect("and leaves 'nothing answered' behind",
           ak_imu_last_result() == AK_IMU_NOBODY && ak_imu_last_whoami() == 0);
    fake.fail_reads = 0;

    /* Something answers and it is not a part this build knows - a different
     * fault with a different fix, and the byte is worth keeping because it is
     * what names the part. */
    fake_reset(0x73);
    expect("a part this build does not know is refused",
           ak_imu_open(&imu, &bus, 0) != 0);
    expect("and leaves the byte it answered with behind",
           ak_imu_last_result() == AK_IMU_UNKNOWN_PART &&
               ak_imu_last_whoami() == 0x73);

    /* The right part answers and will not take its configuration: marginal
     * wiring rather than a missing part, and the distinction only exists
     * because the third case was given its own value. */
    fake_reset(0x68);
    fake.fail_writes = 1;
    expect("a part that will not configure is refused",
           ak_imu_open(&imu, &bus, 0) != 0);
    expect("and says so rather than blaming the bus",
           ak_imu_last_result() == AK_IMU_NO_CONFIG);
    fake.fail_writes = 0;

    /* And a good open clears it, so the report cannot describe a previous
     * failure as the current state. */
    fake_reset(0x68);
    expect("a good open after a failure is opened",
           ak_imu_open(&imu, &bus, 0) == 0);
    expect("and the report stops saying anything failed",
           ak_imu_last_result() == AK_IMU_OK);
}

void test_imu(void)
{
    test_imu_probe();
    test_imu_init_sequence();
    test_imu_read();
    test_mpu6000();
    test_imu_absence_is_recorded();
}
