/*
 * The blackbox.
 *
 * A log is a thing you only look at after something has gone wrong, which is
 * the worst time to discover that it recorded the wrong thing, lost the part
 * that mattered, or prints in units nobody can read. So: the ring keeps the
 * newest data, the dump is oldest-first, the header names the units, and one
 * record's exact line is pinned.
 */

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "ak_log.h"
#include "ak_flashlog.h"
#include "tests.h"

/* Big enough for a full ring: a truncated capture would silently shorten the
 * line count this test checks. */
static char   captured[49152];
static size_t captured_len;

static int capture_printf(const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    int written = vsnprintf(captured + captured_len,
                            sizeof captured - captured_len, fmt, ap);
    va_end(ap);
    if (written > 0 && captured_len + (size_t)written < sizeof captured) {
        captured_len += (size_t)written;
    }
    return written;
}

static int capture_has(const char *needle)
{
    return strstr(captured, needle) != 0;
}

/*
 * A ring that outlives a reset is a ring full of somebody else's data until it
 * has been checked. Every one of these is a way that goes wrong: RAM that was
 * never a log, a ring from a build with a different record layout, a ring from
 * a build with a different capacity, and a header that survived while the
 * counters next to it did not.
 */
static void test_resume(void)
{
    static ak_log_t kept;
    static ak_log_t fresh;
    ak_log_record_t record;

    /* A ring with records in it, as the previous run would have left it. */
    ak_log_init(&kept);
    for (int i = 0; i < 5; i++) {
        memset(&record, 0, sizeof record);
        record.time_ms = (uint32_t)(1000 + i);
        record.state = 1;
        ak_log_push(&kept, &record);
    }

    expect("a ring that was a log resumes with its records",
           ak_log_resume(&kept) == 1 && kept.count == 5 &&
           ak_log_get(&kept, 0, &record) && record.time_ms == 1000);

    /* Zeroed memory - a board that has just had its first power-on, or a host
     * process starting. This is the common case and it must not become a log
     * with a plausible-looking count. */
    memset(&fresh, 0, sizeof fresh);
    expect("memory that was never a log starts empty",
           ak_log_resume(&fresh) == 0 && fresh.count == 0 &&
           fresh.written == 0);

    /* A ring from a build whose records were laid out differently. */
    ak_log_init(&fresh);
    fresh.version = (uint16_t)(AK_LOG_VERSION + 1u);
    expect("a ring from another layout is thrown away",
           ak_log_resume(&fresh) == 0 && fresh.version == AK_LOG_VERSION);

    /* A ring built for a different capacity: this is the ESP32's ring read by
     * the F405's build, or either read after somebody changed the option. */
    ak_log_init(&fresh);
    fresh.capacity = (uint16_t)(AK_LOG_CAPACITY - 1u);
    expect("a ring built for another capacity is thrown away",
           ak_log_resume(&fresh) == 0 && fresh.capacity == AK_LOG_CAPACITY);

    /* A header that survived but a counter that did not: a ring pointing past
     * the end of its own array is a ring that would be read out of bounds. */
    ak_log_init(&fresh);
    fresh.head = (uint16_t)(AK_LOG_CAPACITY + 7u);
    expect("a ring pointing past its own array is thrown away",
           ak_log_resume(&fresh) == 0 && fresh.head == 0);

    ak_log_init(&fresh);
    fresh.count = (uint16_t)(AK_LOG_CAPACITY + 1u);
    expect("and so is one claiming more records than it holds",
           ak_log_resume(&fresh) == 0 && fresh.count == 0);

    /* What survives is dumped: a log nobody can read is not evidence. */
    captured_len = 0;
    captured[0] = '\0';
    ak_log_dump(&kept, capture_printf);
    expect("and the resumed records dump like any others",
           capture_has("# aerialkit blackbox, 5 records") &&
           capture_has("1000,") && capture_has("1004,"));
}

static int capture_count_lines(void)
{
    int lines = 0;
    for (const char *p = captured; *p != '\0'; p++) {
        if (*p == '\n') {
            lines++;
        }
    }
    return lines;
}

static ak_log_record_t record_at(uint32_t time_ms, int16_t roll_gyro,
                                 uint8_t motor)
{
    ak_log_record_t record;
    memset(&record, 0, sizeof record);
    record.time_ms = time_ms;
    record.gyro[0] = roll_gyro;
    record.motor[0] = motor;
    record.state = 1;
    record.flags = AK_LOG_RC_LIVE | AK_LOG_IMU_VALID;
    return record;
}

static void test_ring(void)
{
    ak_log_t log;
    ak_log_init(&log);

    captured[0] = '\0';
    captured_len = 0;
    ak_log_dump(&log, capture_printf);
    expect("an empty log dumps its header and nothing else",
           capture_has("time_ms,gyro_x") && capture_count_lines() == 4);

    /* Fill it past capacity: the newest data has to survive, because that is
     * the data you want after something went wrong. */
    for (uint32_t i = 0; i < AK_LOG_CAPACITY + 10u; i++) {
        ak_log_record_t record = record_at(1000u + i, (int16_t)i, 0);
        ak_log_push(&log, &record);
    }
    expect("the ring holds its capacity", log.count == AK_LOG_CAPACITY);
    expect("and counts what it overwrote", log.overwritten == 10u);
    expect("and counts everything it was given",
           log.written == AK_LOG_CAPACITY + 10u);

    captured[0] = '\0';
    captured_len = 0;
    ak_log_dump(&log, capture_printf);
    expect("the dump says it starts part way through",
           capture_has("overwritten, so this starts part way through"));
    expect("and writes one line per record",
           capture_count_lines() == (int)AK_LOG_CAPACITY + 4);

    /* Oldest first means the first record printed is the one 10 pushes ago. */
    char first[64];
    snprintf(first, sizeof first, "\n%u,", 1010u);
    expect("the oldest surviving record is the first line",
           capture_has(first));
}

static void test_units(void)
{
    ak_log_t log;
    ak_log_init(&log);
    ak_log_set_decimation(&log, 4);

    ak_log_record_t record = record_at(12345u, -1234, 254u);
    record.accel[2] = 1000;     /* 1.000 g */
    record.attitude[0] = -455;  /* -45.5 degrees */
    record.stick[1] = 500;      /* half pitch */
    record.torque[2] = -7;      /* -7 percent */
    ak_log_push(&log, &record);

    captured[0] = '\0';
    captured_len = 0;
    ak_log_dump(&log, capture_printf);

    expect("the header names the units",
           capture_has("gyro 0.1 dps") && capture_has("accel 0.001 g"));
    expect("and says how often it sampled", capture_has("every 4 loop"));
    expect("a record prints its exact values",
           capture_has("12345,-1234,0,0,0,0,1000,-455,0,0,0,0,500,0,0,0,0,"
                       "-7,254,0,0,0,1,3"));

    ak_log_reset(&log);
    captured[0] = '\0';
    captured_len = 0;
    ak_log_dump(&log, capture_printf);
    expect("resetting empties it", log.count == 0 && log.written == 0 &&
           !capture_has("12345"));
}

static void test_wire_records(void)
{
    ak_log_t log;
    ak_log_init(&log);

    for (uint32_t i = 0; i < 5; i++) {
        ak_log_record_t record = record_at(100u + i, (int16_t)(i * 10), (uint8_t)i);
        ak_log_push(&log, &record);
    }

    /* Index 0 is the oldest, which is what makes a streaming reader's life
     * simple: it starts at zero and walks up. */
    ak_log_record_t got;
    expect("index 0 is the oldest record",
           ak_log_get(&log, 0, &got) == 1 && got.time_ms == 100 &&
           got.gyro[0] == 0);
    expect("and the last index is the newest",
           ak_log_get(&log, 4, &got) == 1 && got.time_ms == 104);
    expect("an index past the end is refused", ak_log_get(&log, 5, &got) == 0);

    /* After a wrap, index 0 must still be the oldest *surviving* record. */
    ak_log_init(&log);
    for (uint32_t i = 0; i < AK_LOG_CAPACITY + 3u; i++) {
        ak_log_record_t record = record_at(i, 0, 0);
        ak_log_push(&log, &record);
    }
    expect("after a wrap, index 0 is the oldest the ring still holds",
           ak_log_get(&log, 0, &got) == 1 && got.time_ms == 3u);
    expect("and the last index is still the newest",
           ak_log_get(&log, AK_LOG_CAPACITY - 1u, &got) == 1 &&
           got.time_ms == AK_LOG_CAPACITY + 2u);

    /* The wire encoding is fixed width, explicitly little-endian, and the same
     * size whatever the compiler thinks about padding. */
    ak_log_record_t record = record_at(0x01020304u, -2, 7u);
    record.accel[1] = 1000;
    record.stick[2] = -1000;
    record.torque[0] = -50;
    record.state = 2;
    record.flags = 3;

    uint8_t wire[AK_LOG_WIRE_BYTES];
    expect("the record encodes to a fixed size",
           ak_log_encode_record(&record, wire, sizeof wire) == AK_LOG_WIRE_BYTES);
    expect("time is little-endian",
           wire[0] == 0x04 && wire[1] == 0x03 && wire[2] == 0x02 && wire[3] == 0x01);
    expect("a negative gyro is two's complement",
           wire[4] == 0xFE && wire[5] == 0xFF);
    expect("fields land where the layout says",
           wire[12] == 0xE8 && wire[13] == 0x03 &&   /* accel y = 1000  */
           wire[30] == 0x18 && wire[31] == 0xFC &&   /* stick yaw = -1000 */
           wire[34] == 0xCE);                        /* torque roll = -50 */
    /* And the two fields this record grew last: the yaw (0.1 deg) after the
     * roll and pitch, and the height above the take-off reference after that,
     * which is where a landing is read from. */
    {
        ak_log_record_t angled = record_at(0u, 0, 0);
        uint8_t angled_wire[AK_LOG_WIRE_BYTES];
        int16_t yaw = -450;   /* -45.0 deg */
        int32_t alt = 123456; /* 123.456 m up */
        int32_t lat = 521234567;   /* 52.1234567 deg */
        int32_t lon = -1224194300; /* -122.4194300 deg */

        angled.yaw = yaw;
        angled.alt_mm = alt;
        angled.lat_e7 = lat;
        angled.lon_e7 = lon;
        (void)ak_log_encode_record(&angled, angled_wire, sizeof angled_wire);
        expect("and the yaw and the height are at the offsets the layout says",
               angled_wire[20] == 0x3E && angled_wire[21] == 0xFE &&
               angled_wire[22] == 0x40 && angled_wire[23] == 0xE2 &&
               angled_wire[24] == 0x01 && angled_wire[25] == 0x00);
        /* And the position, which is the last thing the record grew: the two
         * numbers that say where the aircraft was, in the module's own 1e-7
         * degrees, signed and little-endian like everything else here. */
        expect("and the position is after the state and the flags",
               angled_wire[43] == 0x87 && angled_wire[44] == 0x68 &&
               angled_wire[45] == 0x11 && angled_wire[46] == 0x1F &&
               angled_wire[47] == 0x04 && angled_wire[48] == 0x47 &&
               angled_wire[49] == 0x08 && angled_wire[50] == 0xB7);
    }
    expect("and the state and flags are where the layout puts them",
           wire[41] == 2 && wire[42] == 3);
    expect("a buffer that is too small is refused",
           ak_log_encode_record(&record, wire, 8) == 0);
}

/* --- the log in flash ------------------------------------------------------
 *
 * A store the test owns: two small sectors of ordinary memory, erased to all
 * ones and programmed word by word, which is exactly what the part does and
 * exactly what the F405 board hands to the log for real. Small on purpose -
 * two sectors of four slots is enough to make the log fill a sector, stop, be
 * given room, wrap, and be read back in order, and it is quick enough to do it
 * several times over. The same code runs against the real region and the
 * modelled controller in tests/test_arch.c.
 */

#define FAKE_SECTOR_BYTES 272u /* 32-byte header + 4 slots of 60 */
#define FAKE_SECTORS 2u

static uint8_t fake_flash[FAKE_SECTORS][FAKE_SECTOR_BYTES];

/* The part this stands in for is one flat region with sectors inside it, so
 * the accesses below go through a byte pointer into the whole array rather
 * than through `fake_flash[0][offset]`: indexing the *first row* at an offset
 * past the row is out of bounds of that sub-array even though the bytes are
 * there in the object, and the sanitizer run named it three times in this
 * file. */
static uint8_t *const fake_bytes = &fake_flash[0][0];

static uint32_t fake_read(uint32_t address)
{
    uint32_t offset = address - (uint32_t)(uintptr_t)fake_bytes;
    uint32_t word = 0u;

    if ((offset + 4u) > sizeof fake_flash) {
        return 0xFFFFFFFFu; /* outside the part, which is the same as erased */
    }
    for (unsigned i = 0u; i < 4u; i++) {
        word |= (uint32_t)fake_bytes[offset + i] << (8u * i);
    }
    return word;
}

static int fake_erase(unsigned index)
{
    if (index >= FAKE_SECTORS) {
        return -1;
    }
    memset(fake_flash[index], 0xFF, FAKE_SECTOR_BYTES);
    return 0;
}

/* Word by word, and only into erased words: a test double that lets a log
 * rewrite a word would let a bug through that the part would refuse. */
static int fake_write(uint32_t address, const void *bytes, unsigned len)
{
    const uint8_t *source = bytes;
    uint32_t offset = address - (uint32_t)(uintptr_t)fake_bytes;

    /* The part programs whole words, so a write whose last word would hang off
     * the end of the region is refused rather than half done - and the rounding
     * is the check, because `offset + len` equal to the end is a write of one,
     * two or three bytes that this model has no word for. */
    if ((offset + ((len + 3u) & ~3u)) > sizeof fake_flash) {
        return -1;
    }
    for (unsigned i = 0u; i < len; i += 4u) {
        for (unsigned byte = 0u; byte < 4u; byte++) {
            uint8_t *cell = &fake_bytes[offset + i + byte];
            uint8_t wanted = source[i + byte];

            if (*cell != 0xFFu && *cell != wanted) {
                return -1; /* the part would raise an error here */
            }
            *cell = (uint8_t)(*cell & wanted);
        }
    }
    return 0;
}

/* The addresses are the array's own, so they are set at run time rather than
 * in an initialiser: a pointer to a static object is not a constant
 * expression, and pretending otherwise is how a test stops being portable. */
static ak_flashlog_region_t fake_regions[FAKE_SECTORS];

static void fake_setup(void)
{
    for (unsigned i = 0u; i < FAKE_SECTORS; i++) {
        fake_regions[i].base = (uint32_t)(uintptr_t)&fake_flash[i][0];
        fake_regions[i].bytes = FAKE_SECTOR_BYTES;
    }
}

static const ak_flashlog_store_t fake_store = {
    .regions = fake_regions,
    .count = FAKE_SECTORS,
    .read = fake_read,
    .read_block = 0, /* filled in by the block-read test below */
    .erase = fake_erase,
    .write = fake_write,
};

/* The store the tests hand to the log. `resume` takes it as an argument rather
 * than calling the board, which is what lets the whole ring run here and what
 * keeps this file out of the way of the board's own store. */
static ak_flashlog_store_t store = fake_store;

/* Every flash test starts from a part that has been erased: the point of the
 * log is that it finds what is there, so what is there has to be known. */
static void fake_start(void)
{
    fake_setup();
    store = fake_store;
    for (unsigned i = 0u; i < FAKE_SECTORS; i++) {
        fake_erase(i);
    }
}

static ak_log_record_t flash_record(uint32_t time_ms, uint8_t state)
{
    ak_log_record_t record;

    memset(&record, 0, sizeof record);
    record.time_ms = time_ms;
    record.state = state;
    record.motor[0] = (uint8_t)(time_ms & 0xFFu);
    return record;
}

static void test_flash_round_trip(void)
{
    /* The decoder has to be the encoder's inverse, field for field: the log in
     * flash is written with one and read with the other, and a disagreement
     * between them is a log of plausible nonsense. */
    ak_log_record_t original = record_at(0x01020304u, -2, 7u);
    original.accel[2] = -1000;
    original.attitude[1] = 1234;
    original.stick[3] = 1000;
    original.torque[2] = -100;
    original.motor[3] = 254u;
    original.state = 3;
    original.flags = 3;

    uint8_t wire[AK_LOG_WIRE_BYTES];
    ak_log_record_t decoded;

    expect("a record encodes to the wire size",
           ak_log_encode_record(&original, wire, sizeof wire) ==
               AK_LOG_WIRE_BYTES);
    expect("and decodes back to the same record",
           ak_log_decode_record(wire, sizeof wire, &decoded) == 1 &&
               decoded.time_ms == original.time_ms &&
               decoded.gyro[0] == original.gyro[0] &&
               decoded.accel[2] == original.accel[2] &&
               decoded.attitude[1] == original.attitude[1] &&
               decoded.stick[3] == original.stick[3] &&
               decoded.torque[2] == original.torque[2] &&
               decoded.motor[3] == original.motor[3] &&
               decoded.state == original.state &&
               decoded.flags == original.flags);
    expect("a buffer too short to hold one is refused",
           ak_log_decode_record(wire, AK_LOG_WIRE_BYTES - 1u, &decoded) == 0);
}

static void test_flash_ring(void)
{
    ak_flashlog_t log;

    fake_start();
    expect("an erased part is a fresh log, not an error",
           ak_flashlog_resume(&log, &store) == AK_FLASHLOG_OK &&
               ak_flashlog_count(&log) == 0u);
    expect("and it knows how much room it has",
           log.slots == 4u && log.sector == 0u);

    /* Four slots: a sector exactly full, and the fifth record has to go into
     * the second sector - which is erased here, so it just carries on. */
    for (uint32_t i = 0u; i < 4u; i++) {
        ak_log_record_t record = flash_record(1000u + i, 1u);
        expect("a record goes into the flash log",
               ak_flashlog_push(&log, &record) == AK_FLASHLOG_OK);
    }
    expect("and the count follows", ak_flashlog_count(&log) == 4u);
    expect("the first sector holds exactly its slots",
           log.sector == 0u && log.slot == 4u);

    ak_log_record_t record = flash_record(2000u, 2u);
    expect("the next record moves into the second sector",
           ak_flashlog_push(&log, &record) == AK_FLASHLOG_OK &&
               log.sector == 1u && log.slot == 1u);

    /* Now the second sector fills, and behind it is the *first* one, which
     * still holds records - so the log stops until somebody erases it. */
    for (uint32_t i = 0u; i < 3u; i++) {
        record = flash_record(2001u + i, 2u);
        expect("filling the second sector works",
               ak_flashlog_push(&log, &record) == AK_FLASHLOG_OK);
    }
    record = flash_record(3000u, 1u);
    expect("a full ring stops instead of erasing in flight",
           ak_flashlog_push(&log, &record) == AK_FLASHLOG_NEEDS_ERASE);
    expect("and it will not erase while it is flying",
           ak_flashlog_service(&log, 0) == AK_FLASHLOG_NEEDS_ERASE);
    expect("but on the ground it takes the sector it needs",
           ak_flashlog_service(&log, 1) == AK_FLASHLOG_OK &&
               ak_flashlog_push(&log, &record) == AK_FLASHLOG_OK &&
               log.sector == 0u && log.slot == 1u);
    /* Eight records went in; the sector that was erased held four of them, so
     * five are left. That subtraction is the whole cost of a ring over
     * sectors, and getting it wrong is how a log claims to hold data it threw
     * away. */
    expect("the count drops what the erase took with it",
           ak_flashlog_count(&log) == 5u);

    /* And a reader that has just booted finds it all: the sectors in sequence
     * order, the oldest record first, whatever order they sit in the region. */
    ak_flashlog_t resumed;
    expect("a resumed log finds every record",
           ak_flashlog_resume(&resumed, &store) == AK_FLASHLOG_OK &&
               ak_flashlog_count(&resumed) == 5u);
    expect("and carries on where it stopped",
           resumed.sector == 0u && resumed.slot == 1u);

    captured[0] = '\0';
    captured_len = 0;
    expect("the dump writes every record it found",
           ak_flashlog_dump(&resumed, capture_printf) == 5u);
    expect("with the oldest first",
           capture_has("# aerialkit blackbox (flash), 5 records in 2 sectors") &&
               capture_has("2000,") && capture_has("3000,"));
    expect("and the records are the ones that were written",
           capture_has("2002,"));

    /* The same records the other way: by index, without reading the ones
     * before, which is what a protocol client pulling a log does. The two
     * have to agree - a dump and a record-at that disagree would mean one of
     * them is lying about the order. */
    ak_log_record_t got;
    expect("a record can be read by index",
           ak_flashlog_record_at(&resumed, 0u, &got) == 1 &&
               got.time_ms == 2000u);
    expect("in the same order the dump uses",
           ak_flashlog_record_at(&resumed, 4u, &got) == 1 &&
               got.time_ms == 3000u);
    expect("and the last one is the count minus one",
           ak_flashlog_record_at(&resumed, ak_flashlog_count(&resumed) - 1u,
                                 &got) == 1 &&
               got.time_ms == 3000u);
    expect("an index past the end is nothing",
           ak_flashlog_record_at(&resumed, ak_flashlog_count(&resumed), &got) ==
               0);
}

static void test_flash_damage(void)
{
    ak_flashlog_t log;
    /* The fifth record is the first slot of the second sector, and that is the
     * one this test tears in half. */
    uint32_t offset = 32u;

    fake_start();
    ak_flashlog_resume(&log, &store);
    for (uint32_t i = 0u; i < 5u; i++) {
        ak_log_record_t record = flash_record(100u + i, 1u);
        (void)ak_flashlog_push(&log, &record);
    }

    /* A slot the power cut in half: the marker word is there and the payload
     * is not. It has to be skipped and counted, not decoded. */
    fake_flash[1][offset + 12u] = 0x00; /* one byte of a payload word, only */
    fake_flash[1][offset + 4u] = 0x00;  /* and the checksum does not match it */

    captured[0] = '\0';
    captured_len = 0;
    ak_flashlog_dump(&log, capture_printf);
    expect("a slot whose checksum fails is skipped and counted",
           capture_has("1 slot skipped - torn or damaged"));

    /* And by index it is *there* and refused: it holds a place in the order,
     * so a reader that skipped it silently would shift every record after it.
     * The count includes it, the record does not come back, and the client
     * counts a hole rather than reading the next record as this one. */
    ak_log_record_t got;
    expect("a torn slot is counted in the log's length",
           ak_flashlog_count(&log) == 5u);
    expect("and refused when that index is asked for",
           ak_flashlog_record_at(&log, 4u, &got) == -1);

    /* A sector header that did not survive: the whole sector goes, and the
     * rest of the log is still readable. */
    fake_start();
    ak_flashlog_resume(&log, &store);
    for (uint32_t i = 0u; i < 5u; i++) {
        ak_log_record_t record = flash_record(100u + i, 1u);
        (void)ak_flashlog_push(&log, &record);
    }
    fake_flash[0][8] ^= 0x01; /* the header's sector number */
    expect("a damaged header takes its sector out of the log",
           ak_flashlog_resume(&log, &store) == AK_FLASHLOG_OK &&
               ak_flashlog_count(&log) == 1u);

    /* And a part that was somebody else's: a header with the right magic and
     * the wrong version is not this log's, so it is not read and not counted. */
    fake_start();
    ak_flashlog_resume(&log, &store);
    ak_log_record_t record = flash_record(1u, 1u);
    (void)ak_flashlog_push(&log, &record);
    fake_flash[0][4] = 99u; /* version 99, and the checksum with it */
    fake_flash[0][28] ^= 0xFF;
    expect("a log from a different version is ignored, not misread",
           ak_flashlog_resume(&log, &store) == AK_FLASHLOG_OK &&
               ak_flashlog_count(&log) == 0u);
}

static void test_flash_clear_and_absence(void)
{
    ak_flashlog_t log;

    fake_start();
    ak_flashlog_resume(&log, &store);
    for (uint32_t i = 0u; i < 5u; i++) {
        ak_log_record_t record = flash_record(100u + i, 1u);
        (void)ak_flashlog_push(&log, &record);
    }
    expect("clearing the log empties it",
           ak_flashlog_clear(&log) == AK_FLASHLOG_OK &&
               ak_flashlog_count(&log) == 0u);
    expect("and leaves the sectors erased",
           fake_flash[0][0] == 0xFFu && fake_flash[1][0] == 0xFFu);
    ak_log_record_t fresh = flash_record(7u, 1u);
    expect("so the next record starts a new log at the beginning",
           ak_flashlog_push(&log, &fresh) == AK_FLASHLOG_OK &&
               log.sector == 0u && log.slot == 1u);

    /* A board with no flash for this at all: the answer the console prints. */
    store = (ak_flashlog_store_t){ .regions = 0, .count = 0u, .read = 0,
                                   .read_block = 0, .erase = 0, .write = 0 };
    expect("a board with no flash log says so rather than pretending",
           ak_flashlog_resume(&log, &store) == AK_FLASHLOG_NO_STORAGE &&
               ak_flashlog_push(&log, &fresh) == AK_FLASHLOG_NO_STORAGE &&
               ak_flashlog_count(&log) == 0u);
    store = fake_store;
}

/*
 * The same log, read through two transports.
 *
 * The core reads the log a *word* at a time, and that is the right thing on
 * the F405: flash is memory there, so a word is a load. On a chip whose flash
 * is behind a driver it is a transaction per four bytes, and a resume touches
 * about two thousand words - which is why the store may offer read_block, and
 * why this test exists in two halves. The first half is that both paths find
 * the same log; the second is that the block path is actually the cheaper one,
 * because a store whose read_block is never called would pass the first half
 * on its own.
 */
static unsigned fake_read_calls;
static unsigned fake_block_calls;

static uint32_t fake_read_counted(uint32_t address)
{
    fake_read_calls++;
    return fake_read(address);
}

static int fake_read_block_counted(uint32_t address, uint8_t *out,
                                   unsigned len)
{
    uint32_t offset = address - (uint32_t)(uintptr_t)fake_bytes;

    fake_block_calls++;
    if (offset + len > sizeof fake_flash) {
        return -1;
    }
    memcpy(out, &fake_bytes[offset], len);
    return 0;
}

static void test_block_reads(void)
{
    ak_flashlog_t filler;
    ak_flashlog_t word_log;
    ak_flashlog_t block_log;
    ak_flashlog_store_t word_store = fake_store;
    ak_flashlog_store_t block_store = fake_store;
    unsigned word_calls;
    unsigned block_calls;
    char name[128];

    word_store.read = fake_read_counted;
    block_store.read = fake_read_counted;
    block_store.read_block = fake_read_block_counted;

    /* Both sectors full, written the ordinary way: what is being compared is
     * how the two transports *read* a log, not how it got written. */
    fake_start();
    (void)ak_flashlog_resume(&filler, &store);
    for (uint32_t i = 0u; i < 8u; i++) {
        ak_log_record_t record = flash_record(2000u + i, 2u);
        (void)ak_flashlog_push(&filler, &record);
    }

    fake_read_calls = 0u;
    fake_block_calls = 0u;
    expect("a full log read a word at a time comes back",
           ak_flashlog_resume(&word_log, &word_store) == AK_FLASHLOG_OK &&
               ak_flashlog_count(&word_log) == 8u);
    word_calls = fake_read_calls;

    fake_read_calls = 0u;
    fake_block_calls = 0u;
    expect("and the same log through a store that reads spans",
           ak_flashlog_resume(&block_log, &block_store) == AK_FLASHLOG_OK &&
               ak_flashlog_count(&block_log) == 8u);
    block_calls = fake_block_calls;

    expect("finds the same log - sector, slot, records and torn slots",
           block_log.sector == word_log.sector &&
               block_log.slot == word_log.slot &&
               block_log.records == word_log.records &&
               block_log.torn == word_log.torn &&
               block_log.sequence == word_log.sequence);

    snprintf(name, sizeof name,
             "in %u calls instead of %u - a header or a record at a time",
             block_calls, word_calls);
    expect(name, block_calls > 0u && word_calls > 0u &&
                     block_calls * 4u < word_calls);
}

void test_blackbox(void)
{
    test_ring();
    test_units();
    test_wire_records();
    test_resume();
    test_flash_round_trip();
    test_flash_ring();
    test_block_reads();
    test_flash_damage();
    test_flash_clear_and_absence();
}
