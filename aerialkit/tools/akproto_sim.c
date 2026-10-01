/*
 * akproto_sim - the firmware's half of the config protocol, on stdin/stdout.
 *
 *   build-host/aerialkit-sim < frames.bin > replies.bin
 *
 * It links the real parameter table and the real flight-config defaults, so a
 * client driving this simulator is looking at the same names, the same ranges
 * and the same text as it would over a serial line to a board. That is the
 * point: the client can be tested here, end to end, instead of being tested
 * separately at each end and hoped to fit in the middle.
 *
 * The status reply is filled with fixed plausible numbers, because there is no
 * flight here to report.
 */

#include <stdio.h>
#include <string.h>

#include "ak_flight.h"
#include "ak_flashlog.h"
#include "ak_log.h"
#include "ak_mixer.h"
#include "ak_proto.h"

static ak_flight_t  flight;
static ak_param_t   items[AK_PARAMS_MAX];
static ak_params_t  params;
static uint32_t     saves;
static ak_log_t     blackbox;

/* A short synthetic log, so a client has something real to pull. Time counts
 * up and the gyro with it, which makes a row-for-row check easy to read. */
static void fill_log(void)
{
    ak_log_init(&blackbox);
    for (uint32_t i = 0; i < 6; i++) {
        ak_log_record_t record;
        memset(&record, 0, sizeof record);
        record.time_ms = 1000u + i * 4u;
        record.gyro[0] = (int16_t)(100 + i);
        record.accel[2] = 1000;
        record.attitude[0] = (int16_t)(i * 10);
        /* The two fields the record grew last, at values a client can check
         * its own offsets against: a yaw that is negative and a height that is
         * not a plausible other field. */
        record.yaw = (int16_t)(-900 - (int16_t)i);
        record.alt_mm = (int32_t)(23000 + (int32_t)i * 10);
        /* And the position, the field after that: distinctive in both signs,
         * because a client that reads it as unsigned gets a longitude in the
         * Pacific. */
        record.lat_e7 = (int32_t)(521234500 + (int32_t)i);
        record.lon_e7 = (int32_t)(-49876500 + (int32_t)i);
        record.stick[3] = 500;
        record.motor[0] = 40;
        record.motor[1] = 40;
        record.motor[2] = (uint8_t)(60 + i);
        record.motor[3] = 60;
        record.state = 1;
        record.flags = 3;
        ak_log_push(&blackbox, &record);
    }
}

/*
 * Two logs on this side of the test, which is what makes the source selector
 * worth having: the fast ring in RAM, filled by fill_log() above, and a log in
 * flash - a region of ordinary memory here, written by the same code the board
 * runs. A source this simulator does not have (the long ring) answers -1,
 * which is how the protocol says "not here" rather than "empty", and the
 * client's checks look at both answers.
 */
#define SIM_FLASH_SECTORS 2u
#define SIM_FLASH_SECTOR_BYTES 512u

static uint8_t sim_flash[SIM_FLASH_SECTORS][SIM_FLASH_SECTOR_BYTES];
static ak_flashlog_region_t sim_flash_regions[SIM_FLASH_SECTORS];
static ak_flashlog_t sim_flash_log;
static int sim_flash_state = AK_FLASHLOG_NO_STORAGE;

/* The region is flat with sectors inside it, so the accesses go through a byte
 * pointer into the whole array: `sim_flash[0][offset]` with `offset` past the
 * first sector is an index out of bounds of that sub-array even though the
 * bytes are there, which is what the sanitizer run named in this file, in
 * tools/fw_sim.c and in tests/test_log.c. */
static uint8_t *const sim_flash_bytes = &sim_flash[0][0];

static uint32_t sim_flash_read(uint32_t address)
{
    uint32_t offset = address - (uint32_t)(uintptr_t)sim_flash_bytes;
    uint32_t word = 0u;

    if ((offset + 4u) > sizeof sim_flash) {
        return 0xFFFFFFFFu; /* outside the part, which reads as erased */
    }
    for (unsigned i = 0u; i < 4u; i++) {
        word |= (uint32_t)sim_flash_bytes[offset + i] << (8u * i);
    }
    return word;
}

static int sim_flash_erase(unsigned index)
{
    if (index >= SIM_FLASH_SECTORS) {
        return -1;
    }
    memset(sim_flash[index], 0xFF, SIM_FLASH_SECTOR_BYTES);
    return 0;
}

static int sim_flash_write(uint32_t address, const void *bytes, unsigned len)
{
    const uint8_t *source = bytes;
    uint32_t offset = address - (uint32_t)(uintptr_t)sim_flash_bytes;

    if ((offset + len) > sizeof sim_flash) {
        return -1;
    }
    for (unsigned i = 0u; i < len; i++) {
        uint8_t *cell = &sim_flash_bytes[offset + i];

        if (*cell != 0xFFu && *cell != source[i]) {
            return -1;
        }
        *cell = (uint8_t)(*cell & source[i]);
    }
    return 0;
}

static const ak_flashlog_store_t sim_flash_store = {
    .regions = sim_flash_regions,
    .count = SIM_FLASH_SECTORS,
    .read = sim_flash_read,
    .read_block = 0, /* memory, and the check here is about the protocol */
    .erase = sim_flash_erase,
    .write = sim_flash_write,
};

/* Three records in the flash log, written the way the firmware writes them:
 * through the real log, into a store that happens to be memory. */
static void fill_flash_log(void)
{
    for (unsigned i = 0u; i < SIM_FLASH_SECTORS; i++) {
        sim_flash_regions[i].base = (uint32_t)(uintptr_t)&sim_flash[i][0];
        sim_flash_regions[i].bytes = SIM_FLASH_SECTOR_BYTES;
        (void)sim_flash_erase(i);
    }
    sim_flash_state = ak_flashlog_resume(&sim_flash_log, &sim_flash_store);
    for (uint32_t i = 0u; i < 3u; i++) {
        ak_log_record_t record;

        memset(&record, 0, sizeof record);
        record.time_ms = 2000u + i * 5u;
        record.gyro[1] = (int16_t)(200 + i);
        record.motor[3] = 60;
        record.state = 2;
        (void)ak_flashlog_push(&sim_flash_log, &record);
    }
}

static int32_t sim_log_count(void *ctx, uint8_t source)
{
    (void)ctx;
    if (source == AK_PROTO_LOG_FAST) {
        return blackbox.count;
    }
    if (source == AK_PROTO_LOG_FLASH) {
        return sim_flash_state == AK_FLASHLOG_OK
                   ? (int32_t)ak_flashlog_count(&sim_flash_log)
                   : -1;
    }
    return -1;
}

static unsigned sim_log_record(void *ctx, uint8_t source, uint16_t index,
                               uint8_t *out, unsigned capacity)
{
    (void)ctx;
    ak_log_record_t record;

    if (source == AK_PROTO_LOG_FLASH) {
        if (sim_flash_state != AK_FLASHLOG_OK ||
            ak_flashlog_record_at(&sim_flash_log, index, &record) != 1) {
            return 0;
        }
        return ak_log_encode_record(&record, out, capacity);
    }
    if (source != AK_PROTO_LOG_FAST) {
        return 0;
    }
    if (!ak_log_get(&blackbox, index, &record)) {
        return 0;
    }
    return ak_log_encode_record(&record, out, capacity);
}

static void sim_status(void *ctx, ak_proto_status_t *out)
{
    (void)ctx;
    out->flight_state = 1;      /* armed, in the fiction of a running aircraft */
    out->link_live = 1;
    out->gps_fix_type = 3;
    out->gps_satellites = 11;
    out->roll_ddeg = 125;
    out->pitch_ddeg = -40;
    out->yaw_ddeg = 900;
    out->lat_e7 = 521234567;
    out->lon_e7 = 49876543;
    out->motor[0] = 40;
    out->motor[1] = 40;
    out->motor[2] = 60;
    out->motor[3] = 60;
}

static int sim_save(void *ctx)
{
    (void)ctx;
    saves++;
    return 0;
}

int main(void)
{
    ak_flight_init(&flight, &ak_mixer_elevon_wing);
    unsigned count = ak_flight_param_table(&flight, items, AK_PARAMS_MAX);
    ak_params_init(&params, items, count);
    fill_log();
    fill_flash_log();

    ak_proto_io_t io = {
        .params = &params,
        /* A reading of this build, not a plan: `ak_proto.c` has a `case` for
         * AK_PROTO_CMD_PARAM_INFO and for AK_PROTO_CMD_PARAM_HELP, so this
         * simulator answers them, and a capability word that left the bit clear
         * would under-report — the same class of lie in the other direction as
         * a board claiming a command it does not have.
         *
         * The other bits are deliberately absent, and each for its own reason.
         * APPLIES_ON_WRITE is not set because there is no `on_change` here and
         * nothing to re-apply: this is a table with a flight model beside it,
         * not an aircraft. GATES_ON_ARMED is not set because there is no
         * `writable` callback and no flight state to consult — the simulator
         * spends its whole run in whatever state `sim_status` reports. The rest
         * of the AK_PROTO_FEATURE_* bits name commands ak_proto.c has no `case`
         * for, so setting one would be a promise this binary cannot keep. */
        .features = AK_PROTO_FEATURE_PARAM_INFO,
        .status = sim_status,
        .save = sim_save,
        .log_count = sim_log_count,
        .log_record = sim_log_record,
        .ctx = 0,
    };
    ak_proto_t proto;
    ak_proto_init(&proto);

    uint32_t now_ms = 0;
    int c;
    while ((c = getchar()) != EOF) {
        now_ms++;
        uint8_t response[AK_PROTO_FRAME_MAX];
        unsigned written = ak_proto_feed(&proto, &io, (uint8_t)c, now_ms, response,
                                         sizeof response);
        if (written > 0) {
            fwrite(response, 1, written, stdout);
            fflush(stdout);
        }
    }

    /* The counters go to stderr so they do not confuse a client reading the
     * protocol from stdout. */
    fprintf(stderr, "sim: %u bytes, %u frames, %u responses, %u bad crc, "
                    "%u bad length, %u unknown, %u saves\n",
            proto.bytes, proto.frames, proto.responses, proto.bad_crc,
            proto.bad_length, proto.unknown_commands, saves);
    return 0;
}
