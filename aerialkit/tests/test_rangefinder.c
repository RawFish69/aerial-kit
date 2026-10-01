/*
 * The rangefinder: a TOF10120 on a fake bus, and the filter between it and the
 * two things that ask it questions.
 *
 * What can be checked without a part is what the driver *asks for* - the
 * register written before the part will answer, the address register read back
 * as "something is there", the two bytes of distance and which end of them is
 * which - and then the parts of the answer that are the firmware's own: the
 * period, the staleness, and the two ways a reading can be impossible.
 *
 * The one thing no test here can say is whether a real TOF10120 returns those
 * numbers. That is a part, a wire and a bench, and it is written down as
 * unverified with the rest of the sensors.
 */

#include <stdio.h>
#include <string.h>

#include "ak_rangefinder.h"
#include "tests.h"

/* --- a TOF10120, as a register file --------------------------------------- */

static uint8_t regs[256];
static int     fail_reads;
static unsigned bus_reads;
static unsigned delays;

static int bus_read(void *ctx, uint8_t reg, uint8_t *buf, unsigned len)
{
    (void)ctx;
    bus_reads++;
    if (fail_reads) {
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

static void part_reset(void)
{
    memset(regs, 0, sizeof regs);
    regs[0x0F] = 0x52; /* the part's own I2C address, which is never zero */
    fail_reads = 0;
    bus_reads = 0;
    delays = 0;
}

/* What the part reports, in millimetres, big endian: the layout INAV's driver
 * reads at register 0x00. */
static void part_distance(int32_t mm)
{
    regs[0x00] = (uint8_t)(((uint32_t)mm >> 8) & 0xFFu);
    regs[0x01] = (uint8_t)((uint32_t)mm & 0xFFu);
}

static void test_driver(void)
{
    ak_rangefinder_t rgf;
    int32_t mm = 0;

    /* --- an empty bus is not a rangefinder -------------------------------- */

    part_reset();
    regs[0x0F] = 0; /* what a floating bus reads */
    expect("a bus with nothing on it is not a rangefinder",
           ak_rangefinder_open(&rgf, &fake_bus, 0) != 0 && !rgf.present);
    expect("and the part was not left believing it had been configured",
           regs[0x09] == 0x01u);

    /* --- and one that answers is ----------------------------------------- */

    part_reset();
    expect("a part that answers is found",
           ak_rangefinder_open(&rgf, &fake_bus, 0) == 0 && rgf.present);
    expect("and it is the driver this build has",
           rgf.driver == &ak_range_tof10120);
    /* The one register write the part needs before it will answer at all, and
     * the settling delay after it. */
    expect("the part is put in the mode its driver expects",
           regs[0x09] == 0x01u);
    expect("and given a moment to take it",
           delays >= 100u);

    /* --- the reading ------------------------------------------------------ */

    part_distance(1234);
    expect("a distance comes back in millimetres",
           ak_rangefinder_read(&rgf, 1000u, &mm) == 1 && mm == 1234);

    /*
     * The blind zone. A part this close is reading its own case, not the
     * ground, so what comes out is the minimum it can actually see - which is
     * still an answer to the only question being asked down here ("is the
     * ground right there"), and is not a fault.
     */
    part_distance(0);
    expect("a reading inside the blind zone is the minimum, not a zero",
           ak_rangefinder_read(&rgf, 1200u, &mm) == 1 && mm == 30);
    part_distance(12);
    expect("and inside the blind zone it reports the minimum it can see",
           ak_rangefinder_read(&rgf, 1400u, &mm) == 1 && mm == 30);

    /* 2000 mm is the end of the part's range, and INAV's rule is "at or past
     * it there is nothing to report". */
    /* The gap here is deliberate: the part has just been reading the inside
     * of its own case, and a jump to the far end of its range is a rate the
     * filter above is entitled to disbelieve. Half a second later it is three
     * metres a second, which is a descent the aircraft can make. */
    part_distance(1999);
    expect("the far end of the range is still a distance",
           ak_rangefinder_read(&rgf, 1901u, &mm) == 1 && mm == 1999);
    part_distance(2000);
    expect("and the end of the range itself is not",
           ak_rangefinder_read(&rgf, 2001u, &mm) == 0 &&
           rgf.distance_mm == AK_RANGE_NONE);
    part_distance(3000);
    expect("nor is anything past it",
           ak_rangefinder_read(&rgf, 2101u, &mm) == 0);
    expect("both of which are counted as out of range, not as faults",
           rgf.out_of_range == 2u && rgf.faults == 0u);

    /* --- the period ------------------------------------------------------- */

    part_reset();
    (void)ak_rangefinder_open(&rgf, &fake_bus, 0);
    part_distance(900);
    (void)ak_rangefinder_read(&rgf, 5000u, &mm);
    unsigned reads_before = bus_reads;
    part_distance(1100);
    expect("the part is not asked twice inside its own period",
           ak_rangefinder_read(&rgf, 5050u, &mm) == AK_RANGE_IDLE &&
           bus_reads == reads_before);
    expect("and the reading it already had is the one it keeps",
           rgf.distance_mm == 900);
    expect("but it is asked again when the period is up",
           ak_rangefinder_read(&rgf, 5100u, &mm) == AK_RANGE_READING &&
           mm == 1100);
    /*
     * And the distinction that matters to whoever is counting faults: a pass
     * where the part was not asked is not an answer from it, and a part that
     * says "nothing in range" *is* answering. Before the two were separated,
     * a dead part made the console's quiet/answering pair flip once per poll
     * and the count of failed reads never rose above one - which is the bug
     * the `rangefindfail` session in the simulator found.
     */
    expect("and a pass that did not ask the part is not an answer from it",
           ak_rangefinder_read(&rgf, 5120u, &mm) == AK_RANGE_IDLE);

    /* --- a part that stops answering -------------------------------------- */

    part_reset();
    (void)ak_rangefinder_open(&rgf, &fake_bus, 0);
    part_distance(1500);
    (void)ak_rangefinder_read(&rgf, 6000u, &mm);
    fail_reads = 1;
    expect("a read that fails is a fault",
           ak_rangefinder_read(&rgf, 6200u, &mm) == -1 && rgf.faults == 1u);
    expect("and it takes the last distance with it",
           rgf.distance_mm == AK_RANGE_NONE);
    expect("so a part that has stopped is not a part saying the ground is "
           "close",
           !ak_rangefinder_valid(&rgf, 6200u, AK_RANGE_STALE_MS));
    fail_reads = 0;
    part_distance(1400);
    expect("and it comes back when the part does",
           ak_rangefinder_read(&rgf, 6400u, &mm) == 1);
}

static void test_filter(void)
{
    ak_rangefinder_t rgf;
    int32_t mm = 0;

    /* --- a reading the aircraft cannot have made -------------------------- */

    part_reset();
    (void)ak_rangefinder_open(&rgf, &fake_bus, 0);
    part_distance(1900);
    (void)ak_rangefinder_read(&rgf, 1000u, &mm);

    /* 450 mm in 100 ms is four and a half metres a second, which the aircraft
     * can do; 850 is eight and a half, which a descent capped at three cannot.
     * The first is kept and the second is not, and the difference between them
     * is the whole filter. */
    part_distance(1450);
    expect("a reading the aircraft could have made is kept",
           ak_rangefinder_read(&rgf, 1100u, &mm) == 1 && mm == 1450);
    part_distance(600);
    expect("one it could not is refused",
           ak_rangefinder_read(&rgf, 1200u, &mm) == 0 && rgf.rejected == 1u);
    expect("and the distance that was there stays there",
           rgf.distance_mm == 1450);
    expect("while the reading is still one the aircraft can use",
           ak_rangefinder_valid(&rgf, 1200u, AK_RANGE_STALE_MS));

    /*
     * And the same reading a moment later is a different claim. Two hundred
     * milliseconds after the last reading that was *believed*, 850 mm is four
     * and a quarter metres a second - a descent this aircraft can make - so it
     * is believed rather than keeping a number the aircraft may have left
     * behind. That is the whole reason the filter measures a rate: a fixed
     * step limit would have gone on refusing for the rest of the flight, and
     * "the sensor is stuck" is not a diagnosis a stuck filter can make.
     */
    part_distance(600);
    expect("but the same reading later is a rate the aircraft could have made",
           ak_rangefinder_read(&rgf, 1400u, &mm) == 1 && mm == 600);
    expect("and what it threw away is still counted, not forgotten",
           rgf.rejected == 1u && rgf.distance_mm == 600);

    /* And a reading past the end of the range is not a spike in disguise: it
     * is the part saying there is nothing down there, which is a different
     * answer from a wrong distance and is counted as one. */
    part_distance(2500);
    uint32_t rejected_before = rgf.rejected;
    expect("a reading past the end of the range is out of range, not a spike",
           ak_rangefinder_read(&rgf, 1600u, &mm) == 0 &&
           rgf.rejected == rejected_before && rgf.out_of_range == 1u);

    /* --- and a jump that has had time to happen --------------------------- */

    part_reset();
    (void)ak_rangefinder_open(&rgf, &fake_bus, 0);
    part_distance(1900);
    (void)ak_rangefinder_read(&rgf, 2000u, &mm);
    /* The aircraft was out of range, or the part was not answering, and now
     * it is: with more than the staleness window between the two readings the
     * difference is not evidence of a spike, because nothing measured the
     * aircraft in between. */
    part_distance(200);
    expect("a jump after a gap is not treated as a spike",
           ak_rangefinder_read(&rgf, 2000u + AK_RANGE_STALE_MS + 1u,
                               &mm) == 1 && mm == 200);
    expect("which is counted as neither a rejection nor a fault",
           rgf.rejected == 0u && rgf.faults == 0u);

    /* --- staleness -------------------------------------------------------- */

    part_reset();
    (void)ak_rangefinder_open(&rgf, &fake_bus, 0);
    part_distance(400);
    (void)ak_rangefinder_read(&rgf, 8000u, &mm);
    expect("a reading is good while it is young",
           ak_rangefinder_valid(&rgf, 8000u, AK_RANGE_STALE_MS));
    expect("and not once it is old",
           !ak_rangefinder_valid(&rgf, 8000u + AK_RANGE_STALE_MS + 1u,
                                 AK_RANGE_STALE_MS));
    expect("and a board with no part never claims one",
           (memset(&rgf, 0, sizeof rgf), !ak_rangefinder_valid(&rgf, 10u, 100u)));
}

void test_rangefinder(void)
{
    test_driver();
    test_filter();
}
