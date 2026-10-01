/*
 * The config protocol: framing, and every command's reply.
 *
 * The interesting cases are the ugly ones - a corrupted byte, a frame that
 * stops half way, a parameter index past the end, a command that does not
 * exist - because those are what a serial line and a hurried script produce.
 * A protocol that answers a good frame and hangs on a bad one is worse than no
 * protocol at all.
 *
 * The frames here are built and parsed by this file from the wire description,
 * so the implementation is not being checked against itself.
 */

#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "ak_cli.h"
#include "ak_console_link.h"
#include "ak_proto.h"
#include "ak_baro.h"
#include "ak_imu.h"
#include "ak_log.h"
#include "ak_rangefinder.h"
#include "ak_version.h"
#include "tests.h"

static unsigned build_request(uint8_t *frame, uint8_t command,
                              const uint8_t *args, unsigned args_length)
{
    unsigned at = 5;
    frame[0] = AK_PROTO_SYNC1;
    frame[1] = AK_PROTO_SYNC2;
    frame[2] = AK_PROTO_VERSION;
    frame[3] = command;
    for (unsigned i = 0; i < args_length; i++) {
        frame[at++] = args[i];
    }
    frame[4] = (uint8_t)(at - 5u);
    uint16_t crc = ak_proto_crc16(&frame[2], at - 2u);
    frame[at++] = (uint8_t)(crc & 0xFFu);
    frame[at++] = (uint8_t)(crc >> 8);
    return at;
}

typedef struct {
    uint8_t  command;
    uint8_t  payload[AK_PROTO_MAX_PAYLOAD];
    unsigned length;
    int      valid;
} parsed_t;

static parsed_t parse_reply(const uint8_t *frame, unsigned length)
{
    parsed_t out;
    memset(&out, 0, sizeof out);

    if (length < 7u || frame[0] != AK_PROTO_SYNC1 || frame[1] != AK_PROTO_SYNC2) {
        return out;
    }
    unsigned payload = frame[4];
    if (payload > sizeof out.payload || length != 5u + payload + 2u) {
        return out;
    }
    uint16_t crc = ak_proto_crc16(&frame[2], 3u + payload);
    if (crc != (uint16_t)(frame[5 + payload] |
                          ((uint16_t)frame[6 + payload] << 8))) {
        return out;
    }
    out.command = frame[3];
    out.length = payload;
    for (unsigned i = 0; i < payload; i++) {
        out.payload[i] = frame[5 + i];
    }
    out.valid = 1;
    return out;
}

static ak_param_t        items[8];
static ak_params_t       params;
static ak_proto_t        proto;
static ak_proto_io_t     io;
static ak_proto_status_t status_out;
static int               saves;
/* How many times the protocol has told the aircraft that a parameter moved.
 * Before B3 there was nowhere to put this callback and the count was always
 * zero - a wire `set` reached the table and stopped. */
static unsigned          changes;
static ak_log_record_t   log_records[4];
static uint16_t          log_records_count;

static void fill_status(void *ctx, ak_proto_status_t *out)
{
    (void)ctx;
    *out = status_out;
}

static int fake_save(void *ctx)
{
    (void)ctx;
    saves++;
    return 0;
}

static void fake_on_change(void)
{
    changes++;
}

static int32_t fake_log_count(void *ctx, uint8_t source)
{
    (void)ctx;
    /* The long log is not here either, and neither is the flash one: -1 for
     * both, which is the answer the tests below are about. */
    return source == AK_PROTO_LOG_FAST ? log_records_count : -1;
}

static unsigned fake_log_record(void *ctx, uint8_t source, uint16_t index,
                                uint8_t *out, unsigned capacity)
{
    (void)ctx;
    if (source != AK_PROTO_LOG_FAST || index >= log_records_count) {
        return 0;
    }
    return ak_log_encode_record(&log_records[index], out, capacity);
}

static void setup(void)
{
    static float roll_kp = 0.25f;
    static float alt_kp = 1.5f;
    static uint32_t rate = 300u;
    unsigned n = 0;

    n = ak_params_add_float(items, n, "roll_kp", "test float", &roll_kp, 3,
                            0.0f, 2.0f, AK_PARAM_GROUP_NONE);
    n = ak_params_add_float(items, n, "alt_kp", "test float", &alt_kp, 2, 0.0f,
                            5.0f, AK_PARAM_GROUP_NONE);
    n = ak_params_add_u32(items, n, "dshot_khz", "test int", &rate, 150u, 600u, AK_PARAM_GROUP_OUTPUTS);
    ak_params_init(&params, items, n);

    memset(&status_out, 0, sizeof status_out);
    status_out.flight_state = 1;
    status_out.link_live = 1;
    status_out.gps_fix_type = 3;
    status_out.gps_satellites = 11;
    status_out.roll_ddeg = -125;
    status_out.pitch_ddeg = 40;
    status_out.yaw_ddeg = 900;
    status_out.lat_e7 = 521234567;
    status_out.lon_e7 = -1224194300;
    status_out.motor[0] = 200;

    io.params = &params;
    io.status = fill_status;
    io.on_change = fake_on_change;
    io.save = fake_save;
    /* Cleared here rather than only left to the zero of a static, so a test
     * that installs a guard cannot leak it into the ones that follow - which is
     * how "every other check runs against an ungated board" would quietly stop
     * being true. */
    io.writable = 0;
    io.log_count = fake_log_count;
    io.log_record = fake_log_record;
    io.ctx = 0;

    ak_proto_init(&proto);
    saves = 0;
    changes = 0;

    log_records_count = 4;
    for (unsigned i = 0; i < 4; i++) {
        memset(&log_records[i], 0, sizeof log_records[i]);
        log_records[i].time_ms = 1000u + i;
        log_records[i].gyro[2] = (int16_t)(i * 10);
        log_records[i].state = 1;
    }
}

/* Sends a request byte by byte and parses whatever reply came out of it. */
static int exchange(const uint8_t *request, unsigned request_length,
                    parsed_t *reply, uint32_t now_ms)
{
    uint8_t response[AK_PROTO_FRAME_MAX];
    unsigned written = 0;

    for (unsigned i = 0; i < request_length; i++) {
        unsigned got = ak_proto_feed(&proto, &io, request[i], now_ms, response,
                                     sizeof response);
        if (got > 0) {
            written = got;
        }
    }
    if (written == 0) {
        if (reply != 0) {
            memset(reply, 0, sizeof *reply);
        }
        return 0;
    }
    if (reply != 0) {
        *reply = parse_reply(response, written);
    }
    return (int)written;
}

/*
 * Which log a client is talking about.
 *
 * The three logs on the aircraft are the same interface from a tool's side -
 * ask for one, then read records by index - and this is the asking: a source
 * that exists is selected and reported with its count, a source that does not
 * exist is refused, and LOG_INFO and LOG_GET follow whatever was selected.
 */
static void test_log_sources(void)
{
    uint8_t request[AK_PROTO_FRAME_MAX];
    parsed_t reply;

    setup();

    uint8_t fast[] = { AK_PROTO_LOG_FAST };
    expect("a source that exists is selected and counted",
           exchange(request,
                    build_request(request, AK_PROTO_CMD_LOG_SOURCE, fast, 1),
                    &reply, 1000) > 0 &&
               reply.valid && reply.payload[0] == 0 &&
               reply.payload[1] == AK_PROTO_LOG_FAST &&
               reply.payload[2] == 4 && reply.payload[3] == 0);

    /* The flash log is not on this fake device, and the answer says so rather
     * than reporting an empty log: a tool that cannot tell the two apart
     * reports "the crash left nothing" when the truth is "you are asking the
     * wrong device". */
    uint8_t flash[] = { AK_PROTO_LOG_FLASH };
    exchange(request, build_request(request, AK_PROTO_CMD_LOG_SOURCE, flash, 1),
             &reply, 1000);
    expect("a source that does not exist is refused",
           reply.valid && reply.payload[0] != 0);
    expect("and the one that did is still selected",
           reply.payload[1] == AK_PROTO_LOG_FAST);

    /* With no payload at all the command still answers, which is how a client
     * asks "what is selected now". */
    exchange(request, build_request(request, AK_PROTO_CMD_LOG_SOURCE, 0, 0),
             &reply, 1000);
    expect("asking with no source reports the current one",
           reply.valid && reply.payload[0] == 0 &&
               reply.payload[1] == AK_PROTO_LOG_FAST);

    /* And LOG_INFO reads the selection: the same count, from the same log. */
    exchange(request, build_request(request, AK_PROTO_CMD_LOG_INFO, 0, 0),
             &reply, 1000);
    expect("the record count is the selected log's",
           reply.valid && reply.payload[0] == 4 && reply.payload[1] == 0);
}

static void test_hello(void)
{
    uint8_t request[AK_PROTO_FRAME_MAX];
    parsed_t reply;

    setup();
    unsigned length = build_request(request, AK_PROTO_CMD_HELLO, 0, 0);
    expect("a hello gets a reply",
           exchange(request, length, &reply, 1000) > 0 && reply.valid);
    expect("the reply carries the command it answers",
           reply.command == (AK_PROTO_CMD_HELLO | AK_PROTO_RESPONSE_BIT));
    expect("with the protocol version", reply.payload[0] == AK_PROTO_VERSION);
    /* The product this binary was built as, not a literal: the same tests now
     * run for three targets, and a hardcoded one is a test that passes on the
     * first and fails on the second - which is how these two were found. */
    expect("the product name",
           strcmp((char *)&reply.payload[1], AK_PRODUCT_STR) == 0);
    expect("and the parameter count",
           reply.payload[1 + strlen((char *)&reply.payload[1]) + 1] == 3);
    expect("every reply is counted", proto.responses == 1);
}

/*
 * The capability word and the configuration hash.
 *
 * The walk below is the point of the test rather than an implementation
 * detail. It reads the four fields a client written against the *old* HELLO
 * reads, and then looks for the two new ones wherever that walk left off - so
 * a change that interleaved them, or that made the product string's length
 * depend on the new fields, fails here rather than on someone's bench.
 */
static void test_hello_capability_word(void)
{
    uint8_t request[AK_PROTO_FRAME_MAX];
    parsed_t reply;

    setup();
    io.features = AK_PROTO_FEATURE_APPLIES_ON_WRITE | AK_PROTO_FEATURE_RC_CHANNELS;
    unsigned length = build_request(request, AK_PROTO_CMD_HELLO, 0, 0);
    exchange(request, length, &reply, 1000);

    unsigned at = 1u + (unsigned)strlen((char *)&reply.payload[1]) + 1u + 2u;
    at += (unsigned)strlen((char *)&reply.payload[at]) + 1u;

    expect("the capability word is appended, not interleaved",
           at + 8u <= reply.length);
    if (at + 8u > reply.length) {
        return;
    }

    uint32_t features = 0;
    uint32_t hash = 0;
    for (unsigned i = 0; i < 4u; i++) {
        features |= (uint32_t)reply.payload[at + i] << (8u * i);
        hash |= (uint32_t)reply.payload[at + 4u + i] << (8u * i);
    }

    expect("it carries the bits this board was told it has",
           features == (AK_PROTO_FEATURE_APPLIES_ON_WRITE |
                        AK_PROTO_FEATURE_RC_CHANNELS));
    /* The other direction, and the one that matters more: a bit the board did
     * not set must not appear. A word that reported capabilities nobody asked
     * it to have is the failure this field exists to make impossible. */
    expect("and no bit this board did not set",
           (features & AK_PROTO_FEATURE_OUTPUT_TEST) == 0u &&
           (features & AK_PROTO_FEATURE_GATES_ON_ARMED) == 0u);
    expect("the hash is of the table the same reply just described",
           hash == ak_params_hash(&params));
    expect("and the whole reply is still inside one frame",
           reply.length <= AK_PROTO_MAX_PAYLOAD);

    /* A board with nothing optional on it says zero, which is a claim, and it
     * is a different payload from a board that says nothing at all. */
    setup();
    io.features = 0u;
    length = build_request(request, AK_PROTO_CMD_HELLO, 0, 0);
    exchange(request, length, &reply, 1000);
    at = 1u + (unsigned)strlen((char *)&reply.payload[1]) + 1u + 2u;
    at += (unsigned)strlen((char *)&reply.payload[at]) + 1u;
    expect("a board with no optional commands still sends the word",
           at + 8u <= reply.length && reply.payload[at] == 0u &&
           reply.payload[at + 1u] == 0u && reply.payload[at + 2u] == 0u &&
           reply.payload[at + 3u] == 0u);
}

static void test_parameters(void)
{
    uint8_t request[AK_PROTO_FRAME_MAX];
    parsed_t reply;

    setup();
    uint8_t get0[] = { 0 };
    exchange(request, build_request(request, AK_PROTO_CMD_PARAM_GET, get0, 1),
             &reply, 1000);
    expect("getting a parameter returns its name and its value",
           reply.valid && reply.payload[0] == 0 &&
           strcmp((char *)&reply.payload[1], "roll_kp") == 0 &&
           strcmp((char *)&reply.payload[9], "0.250") == 0);

    uint8_t get9[] = { 9 };
    exchange(request, build_request(request, AK_PROTO_CMD_PARAM_GET, get9, 1),
             &reply, 1000);
    expect("an index past the end is refused rather than read",
           reply.valid && reply.payload[0] == 1);

    /* Setting takes text and goes through the same range check the console
     * uses, so a value the console would refuse is refused here too. */
    uint8_t set_ok[] = { 0, '0', '.', '5' };
    exchange(request, build_request(request, AK_PROTO_CMD_PARAM_SET, set_ok,
                                    sizeof set_ok), &reply, 1000);
    float value = *(float *)items[0].value;
    expect("setting a parameter reaches the live value",
           reply.valid && reply.payload[0] == 0 && value > 0.49f && value < 0.51f);
    expect("and an accepted value has nothing to add to it",
           reply.valid && reply.payload[1] == '\0');
    /* And it reaches the aircraft, not only the table. The console has always
     * called its change callback after an accepted `set`; the wire had nowhere
     * to put one, so a client could read back a value the hardware was not
     * running. This is the check that says the two links do the same thing
     * when a parameter moves - see tests/oracle/test_config_policy.c for the
     * measurement that showed they did not. */
    expect("and an accepted set tells the aircraft", changes == 1u);

    uint8_t set_bad[] = { 0, '9' };
    exchange(request, build_request(request, AK_PROTO_CMD_PARAM_SET, set_bad,
                                    sizeof set_bad), &reply, 1000);
    expect("a value out of range is refused, not clamped",
           reply.valid && reply.payload[0] == 2 && *(float *)items[0].value < 0.51f);
    /* And the refusal carries the board's own words about it, which is what the
     * configurator shows in place of a status code: the table already wrote
     * this message for the console, and the wire carries it now. */
    expect("and the refusal says why, in the table's own words",
           reply.valid && strstr((char *)&reply.payload[1], "out of") != 0);
    expect("and a refused set tells the aircraft nothing, because nothing moved",
           changes == 1u);

    exchange(request, build_request(request, AK_PROTO_CMD_PARAM_SAVE, 0, 0),
             &reply, 1000);
    expect("saving asks the board to write",
           reply.valid && reply.payload[0] == 0 && saves == 1);
}

/* The armed write gate, and the reset that is only safe because of it.
 *
 * The gate is the fifth status byte's whole reason for existing, so what these
 * checks are about is not "does the board refuse" but *which* refusals it
 * reports and whether the answer is taken at the moment of the request. A gate
 * latched when a client connected would pass a check that flipped the flag once
 * before the first call and never again, which is the shape of check that would
 * have let the bug through - so the flag is moved *between* two otherwise
 * identical requests below. */
static int writable_now;

static int fake_writable(void *ctx)
{
    (void)ctx;
    return writable_now;
}

static void test_write_gate(void)
{
    uint8_t request[AK_PROTO_FRAME_MAX];
    parsed_t reply;

    setup();
    io.writable = fake_writable;

    /* Read rather than assume: `setup()` re-registers a `static` float whose
     * value a test above has already moved, so "the default is 0.25" is true
     * only on the first run of the suite and the check would pass for the wrong
     * reason on any run after it. What this test is about is that a value does
     * not move, so the value it starts at is captured here. */
    const float before0 = *(float *)items[0].value;
    const float before1 = *(float *)items[1].value;
    const uint32_t before2 = *(uint32_t *)items[2].value;

    /* Armed. Every write route refuses, and refuses *before* looking at what
     * was asked for. */
    writable_now = 0;

    uint8_t set_ok[] = { 0, '0', '.', '5' };
    exchange(request, build_request(request, AK_PROTO_CMD_PARAM_SET, set_ok,
                                    sizeof set_ok), &reply, 1000);
    expect("while armed a set is refused as a policy, not as a bad value",
           reply.valid && reply.payload[0] == 5);
    expect("and it says which policy, in words a screen can show",
           reply.valid && strstr((char *)&reply.payload[1], "armed") != 0);
    expect("and the value did not move, because the refusal came first",
           *(float *)items[0].value == before0);
    /* Every entry, not just the named one. The claim is that the guard runs
     * *before* the index is looked up, so a table where one row moved and the
     * rest did not would be the guard running after it - and checking the row
     * the request named is exactly the check that would not notice. */
    expect("nor did any other row, since the refusal came before the index "
           "was read",
           *(float *)items[1].value == before1 &&
               *(uint32_t *)items[2].value == before2);
    expect("and the aircraft was not told anything", changes == 0u);

    /* And the refusal is not the index check wearing a different number: a
     * request naming a parameter that does not exist is refused for the armed
     * reason too, because that answer does not depend on the index. */
    uint8_t set_bad_index[] = { 200, '1' };
    exchange(request, build_request(request, AK_PROTO_CMD_PARAM_SET,
                                    set_bad_index, sizeof set_bad_index),
             &reply, 1000);
    expect("and a set naming no parameter is refused as armed, not as absent - "
           "naming a real one would not have worked either",
           reply.valid && reply.payload[0] == 5);

    exchange(request, build_request(request, AK_PROTO_CMD_PARAM_SAVE, 0, 0),
             &reply, 1000);
    expect("while armed a save is refused with the same status",
           reply.valid && reply.payload[0] == 5 && saves == 0);

    uint8_t default_all[] = { 2 };
    exchange(request, build_request(request, AK_PROTO_CMD_PARAM_DEFAULT,
                                    default_all, sizeof default_all),
             &reply, 1000);
    expect("while armed a reset is refused, which is the one that matters most",
           reply.valid && reply.payload[0] == 5);

    /* Disarmed - and the answer changes with no reconnect, which is the
     * property a connect-time latch would not have. */
    writable_now = 1;

    exchange(request, build_request(request, AK_PROTO_CMD_PARAM_SET, set_ok,
                                    sizeof set_ok), &reply, 1000);
    expect("the same request a moment later is accepted, so the gate is asked "
           "per request rather than remembered from the connection",
           reply.valid && reply.payload[0] == 0 && changes == 1u);

    exchange(request, build_request(request, AK_PROTO_CMD_PARAM_SAVE, 0, 0),
             &reply, 1000);
    expect("and saving is allowed again", reply.valid && saves == 1);
}

static void test_param_default(void)
{
    uint8_t request[AK_PROTO_FRAME_MAX];
    parsed_t reply;

    setup();
    io.writable = fake_writable;
    writable_now = 1;

    /* What the table will restore, read from the table rather than typed: the
     * registered defaults are whatever this run registered, and `setup()` is
     * called once per test. */
    const float def0 = *(float *)items[0].value;
    const float def1 = *(float *)items[1].value;
    const uint32_t def2 = *(uint32_t *)items[2].value;

    /* A request that names nothing is refused rather than read as "everything".
     * This is the check the mode byte exists for: a frame truncated in transit
     * and a deliberate bare request are the same bytes, and one of the two
     * meanings would be a factory reset. */
    exchange(request, build_request(request, AK_PROTO_CMD_PARAM_DEFAULT, 0, 0),
             &reply, 1000);
    expect("a reset that names nothing is refused, not read as 'everything'",
           reply.valid && reply.payload[0] == 2);
    expect("and it says what a request has to name",
           reply.valid && strstr((char *)&reply.payload[1], "reset") != 0);

    /* Mode 0 is not a synonym for 2 either - it is the same request as a bare
     * one, and gets the same answer. */
    uint8_t default_zero[] = { 0 };
    exchange(request, build_request(request, AK_PROTO_CMD_PARAM_DEFAULT,
                                    default_zero, sizeof default_zero),
             &reply, 1000);
    expect("and mode 0 is that same request, not a third meaning",
           reply.valid && reply.payload[0] == 2);

    /* Move two parameters off their defaults, then put one back. */
    uint8_t set0[] = { 0, '1', '.', '5' };
    uint8_t set1[] = { 1, '3', '.', '0' };
    exchange(request, build_request(request, AK_PROTO_CMD_PARAM_SET, set0,
                                    sizeof set0), &reply, 1000);
    exchange(request, build_request(request, AK_PROTO_CMD_PARAM_SET, set1,
                                    sizeof set1), &reply, 1000);
    expect("two parameters are moved off their defaults to start with",
           *(float *)items[0].value != def0 && *(float *)items[1].value != def1);

    uint8_t default_one[] = { 1, 0 };
    exchange(request, build_request(request, AK_PROTO_CMD_PARAM_DEFAULT,
                                    default_one, sizeof default_one),
             &reply, 1000);
    expect("resetting one row restores the value it was registered with",
           reply.valid && reply.payload[0] == 0 &&
           *(float *)items[0].value == def0);
    expect("and leaves every other row alone",
           *(float *)items[1].value != def1);
    expect("and the single-row reset tells the aircraft, as a set does",
           changes == 3u);

    uint8_t default_bad[] = { 1, 200 };
    exchange(request, build_request(request, AK_PROTO_CMD_PARAM_DEFAULT,
                                    default_bad, sizeof default_bad),
             &reply, 1000);
    expect("naming a row that does not exist is refused as absent",
           reply.valid && reply.payload[0] == 1);

    uint8_t default_all[] = { 2 };
    exchange(request, build_request(request, AK_PROTO_CMD_PARAM_DEFAULT,
                                    default_all, sizeof default_all),
             &reply, 1000);
    expect("reset everything restores every row",
           reply.valid && reply.payload[0] == 0 &&
           *(float *)items[0].value == def0 &&
           *(float *)items[1].value == def1 &&
           *(uint32_t *)items[2].value == def2);
    expect("and it tells the aircraft once", changes == 4u);

    /* A row of every type, because the three-way switch inside is exactly the
     * kind of thing that gets a copy and then drifts. */
    uint8_t set2[] = { 2, '4', '0', '0' };
    exchange(request, build_request(request, AK_PROTO_CMD_PARAM_SET, set2,
                                    sizeof set2), &reply, 1000);
    expect("an integer parameter can be moved too",
           *(uint32_t *)items[2].value == 400u);
    uint8_t default_int[] = { 1, 2 };
    exchange(request, build_request(request, AK_PROTO_CMD_PARAM_DEFAULT,
                                    default_int, sizeof default_int),
             &reply, 1000);
    expect("and resetting it restores an integer, not a float read as one",
           reply.valid && *(uint32_t *)items[2].value == def2);
}

/* A board with no `writable` at all is not gated - a device with no aircraft to
 * arm is not a device that is permanently disarmed. Every check above
 * `test_write_gate` runs with it null, so this is the assertion that says the
 * rest of the suite is exercising that path on purpose rather than by
 * accident. */
static void test_no_gate_without_a_board(void)
{
    uint8_t request[AK_PROTO_FRAME_MAX];
    parsed_t reply;

    setup();
    expect("a board that does not answer the question is not refused",
           io.writable == 0);
    uint8_t set_ok[] = { 0, '0', '.', '5' };
    exchange(request, build_request(request, AK_PROTO_CMD_PARAM_SET, set_ok,
                                    sizeof set_ok), &reply, 1000);
    expect("and its writes go through", reply.valid && reply.payload[0] == 0);
}

static void test_status(void)
{
    uint8_t request[AK_PROTO_FRAME_MAX];
    parsed_t reply;

    setup();
    exchange(request, build_request(request, AK_PROTO_CMD_STATUS, 0, 0), &reply,
             1000);
    expect("status answers", reply.valid);
    expect("with the flight state, the link, the fix type and the satellites",
           reply.payload[0] == 1 && reply.payload[1] == 1 &&
           reply.payload[2] == 3 && reply.payload[3] == 11);

    int16_t roll = (int16_t)(reply.payload[4] | ((uint16_t)reply.payload[5] << 8));
    int16_t yaw = (int16_t)(reply.payload[8] | ((uint16_t)reply.payload[9] << 8));
    expect("roll comes back signed, in tenths of a degree", roll == -125);
    expect("and yaw likewise", yaw == 900);

    int32_t lat = (int32_t)((uint32_t)reply.payload[10] |
                            ((uint32_t)reply.payload[11] << 8) |
                            ((uint32_t)reply.payload[12] << 16) |
                            ((uint32_t)reply.payload[13] << 24));
    int32_t lon = (int32_t)((uint32_t)reply.payload[14] |
                            ((uint32_t)reply.payload[15] << 8) |
                            ((uint32_t)reply.payload[16] << 16) |
                            ((uint32_t)reply.payload[17] << 24));
    expect("position survives the round trip, sign included",
           lat == 521234567 && lon == -1224194300);
    expect("and the motor outputs are in there", reply.payload[18] == 200);
}

static void test_bad_frames(void)
{
    uint8_t request[AK_PROTO_FRAME_MAX];
    parsed_t reply;

    /* One corrupted byte, anywhere in the body, must be caught. Corrupting the
     * *length* is a different case and deliberately not this test: a length
     * that disagrees with the bytes makes the parser wait for bytes that never
     * come, which the gap timeout handles, not the checksum. */
    setup();
    unsigned length = build_request(request, AK_PROTO_CMD_STATUS, 0, 0);
    request[3] ^= 0x01; /* the command byte */
    expect("a corrupted frame gets no reply",
           exchange(request, length, &reply, 1000) == 0);
    expect("and is counted as a bad checksum",
           proto.bad_crc == 1 && proto.responses == 0);

    /* Noise, a frame that stops half way, then a real one: the parser has to
     * find the sync pair again rather than staying lost. */
    setup();
    for (int i = 0; i < 20; i++) {
        ak_proto_feed(&proto, &io, (uint8_t)(0x10 + i), 1000, 0, 0);
    }
    length = build_request(request, AK_PROTO_CMD_STATUS, 0, 0);
    for (unsigned i = 0; i < 6; i++) {
        ak_proto_feed(&proto, &io, request[i], 1000, 0, 0);
    }
    expect("a truncated frame is abandoned after a gap, and the next one decodes",
           exchange(request, length, &reply, 1200) > 0 && reply.valid);

    /* A length larger than the payload buffer is refused by length. */
    setup();
    uint8_t long_frame[] = { AK_PROTO_SYNC1, AK_PROTO_SYNC2, AK_PROTO_VERSION,
                             AK_PROTO_CMD_STATUS, 200, 0x00 };
    for (unsigned i = 0; i < sizeof long_frame; i++) {
        ak_proto_feed(&proto, &io, long_frame[i], 1000, 0, 0);
    }
    expect("an over-long frame is refused by length", proto.bad_length == 1);

    /* An unknown command is answered rather than ignored: a client that gets
     * silence cannot tell a missing feature from a broken wire. */
    setup();
    exchange(request, build_request(request, 0x55, 0, 0), &reply, 1000);
    expect("an unknown command is answered with a refusal",
           reply.valid && reply.payload[0] == 0x7F && proto.unknown_commands == 1);
}

static void test_log(void)
{
    uint8_t request[AK_PROTO_FRAME_MAX];
    parsed_t reply;

    setup();
    exchange(request, build_request(request, AK_PROTO_CMD_LOG_INFO, 0, 0), &reply,
             1000);
    expect("the log reports how many records it holds",
           reply.valid && (reply.payload[0] | (reply.payload[1] << 8)) == 4);

    uint8_t index0[] = { 0, 0 };
    exchange(request, build_request(request, AK_PROTO_CMD_LOG_GET, index0, 2),
             &reply, 1000);
    expect("a record comes back with a status byte and 51 bytes of record",
           reply.valid && reply.payload[0] == 0 &&
           reply.length == 1u + AK_LOG_WIRE_BYTES);

    /* The record's own bytes, parsed the way a client would: little-endian, in
     * the layout written down in ak_log.h. */
    const uint8_t *wire = &reply.payload[1];
    uint32_t time = (uint32_t)wire[0] | ((uint32_t)wire[1] << 8) |
                    ((uint32_t)wire[2] << 16) | ((uint32_t)wire[3] << 24);
    int16_t gyro_z = (int16_t)(wire[8] | ((uint16_t)wire[9] << 8));
    expect("the first record is the oldest, with its own timestamp",
           time == 1000u && gyro_z == 0);
    expect("and its state is where the layout puts it",
           wire[41] == 1);

    uint8_t index1[] = { 1, 0 };
    exchange(request, build_request(request, AK_PROTO_CMD_LOG_GET, index1, 2),
             &reply, 1000);
    time = (uint32_t)reply.payload[1] | ((uint32_t)reply.payload[2] << 8);
    gyro_z = (int16_t)(reply.payload[9] | ((uint16_t)reply.payload[10] << 8));
    expect("the next record follows it", time == 1001u && gyro_z == 10);

    uint8_t index9[] = { 9, 0 };
    exchange(request, build_request(request, AK_PROTO_CMD_LOG_GET, index9, 2),
             &reply, 1000);
    expect("an index past the end is refused rather than wrapped",
           reply.valid && reply.payload[0] == 1);
}

/*
 * Telemetry: the same body the STATUS reply carries, pushed rather than
 * answered, with an uptime in front of it.
 *
 * Two things here are the whole design. The pushed frame's command has no
 * response bit, which is what lets a client tell a stream from its own replies
 * on one connection - so a client can parse both ends of a conversation with
 * one piece of code. And the body is *built by the same function* as the
 * STATUS reply: a stream and a poll that could disagree about where the
 * aircraft is would be worse than no stream.
 */
static void test_telemetry(void)
{
    uint8_t request[AK_PROTO_FRAME_MAX];
    uint8_t frame[AK_PROTO_FRAME_MAX];
    parsed_t reply;

    setup();

    /* This parser stands in for the network link, because that is the one that
     * pushes: the console's answers 0 to a subscribe, and the check for that is
     * just below the rates. */
    proto.can_stream = 1u;

    uint8_t status_frame[AK_PROTO_FRAME_MAX];
    exchange(request, build_request(request, AK_PROTO_CMD_STATUS, 0, 0), &reply,
             1000);
    memcpy(status_frame, reply.payload, reply.length);
    unsigned status_length = reply.length;

    /* Nothing is streamed until somebody asks. */
    expect("a fresh parser streams nothing", proto.telemetry_hz == 0);

    uint8_t wanted[] = { 20 };
    exchange(request, build_request(request, AK_PROTO_CMD_TELEMETRY, wanted, 1),
             &reply, 1000);
    expect("asking for a rate is answered with the rate that will be sent",
           reply.valid && reply.payload[0] == 20 && proto.telemetry_hz == 20);

    /* More than the firmware will send is clamped rather than refused: a client
     * that asks for 200 Hz gets told 50, not silence. */
    uint8_t too_fast[] = { 200 };
    exchange(request,
             build_request(request, AK_PROTO_CMD_TELEMETRY, too_fast, 1),
             &reply, 1000);
    expect("and a rate it will not send is brought down to one it will",
           reply.valid && reply.payload[0] == AK_PROTO_TELEMETRY_MAX_HZ &&
           proto.telemetry_hz == AK_PROTO_TELEMETRY_MAX_HZ);

    uint8_t stop[] = { 0 };
    exchange(request, build_request(request, AK_PROTO_CMD_TELEMETRY, stop, 1),
             &reply, 1000);
    expect("and zero stops it", reply.valid && reply.payload[0] == 0 &&
           proto.telemetry_hz == 0);

    /* And the link that cannot push says so, rather than agreeing to a rate no
     * frame will ever arrive at: the console is a wire somebody types at, so
     * its parser has can_stream clear and a subscribe gets 0 back. A client
     * that waited for frames on that answer would wait for ever, which is what
     * this check exists to make impossible. */
    proto.can_stream = 0u;
    exchange(request, build_request(request, AK_PROTO_CMD_TELEMETRY, wanted, 1),
             &reply, 1000);
    expect("a link that cannot stream answers with no rate at all",
           reply.valid && reply.payload[0] == 0 && proto.telemetry_hz == 0);
    proto.can_stream = 1u;

    /* The frame itself. */
    unsigned length = ak_proto_telemetry_frame(&io, 12345u, frame,
                                               sizeof frame);
    expect("a telemetry frame is built", length > 0);

    parsed_t pushed = parse_reply(frame, length);
    expect("and it frames the way every other message does", pushed.valid);
    expect("with the command, and no response bit on it",
           pushed.command == AK_PROTO_CMD_TELEMETRY);
    expect("the uptime comes first",
           (pushed.payload[0] | (pushed.payload[1] << 8) |
            (pushed.payload[2] << 16) | ((uint32_t)pushed.payload[3] << 24)) ==
               12345u);
    expect("and the rest is the status body, byte for byte",
           pushed.length == 4u + status_length &&
           memcmp(&pushed.payload[4], status_frame, status_length) == 0);
}

/*
 * A parser that has been initialised is a parser whose every byte this code
 * decided.
 *
 * This group exists because that was once false. `ak_proto_init` set its fields
 * one by one from a list, the struct later grew a `can_stream`, and the list did
 * not grow with it - so the field kept whatever the memory held. The flight
 * firmware never noticed, because its parser is `static` and BSS arrives zeroed,
 * but `tools/akproto_sim.c` declares one on the stack. There a subscribe could
 * answer with a rate the simulator has no way to send, and a client waited for
 * frames that were never coming: a test that passed or failed on what the stack
 * happened to contain.
 *
 * A list of field names cannot be checked by reading it. The property can, and
 * this is it, stated without naming a single field: three parsers that arrive at
 * `ak_proto_init` holding three different things must leave holding the same
 * thing. Whatever the defaults are, and whatever the struct grows next, that
 * holds only if the initialiser defines the whole of it - so a field added and
 * forgotten shows up here rather than on somebody's bench.
 */
static void test_parser_initialisation(void)
{
    ak_proto_t zeroed;
    ak_proto_t patterned;
    ak_proto_t saturated;

    /* Three contents no default could produce, so agreeing afterwards cannot be
     * a coincidence of the pattern. */
    memset(&zeroed, 0x00, sizeof zeroed);
    memset(&patterned, 0xA5, sizeof patterned);
    memset(&saturated, 0xFF, sizeof saturated);

    ak_proto_init(&zeroed);
    ak_proto_init(&patterned);
    ak_proto_init(&saturated);

    expect("parsers initialised from different memory agree byte for byte",
           memcmp(&zeroed, &patterned, sizeof zeroed) == 0 &&
           memcmp(&zeroed, &saturated, sizeof zeroed) == 0);

    /* And the two answers a caller depends on before it sends anything: a fresh
     * parser is between frames, and it has not been told it may stream. */
    expect("a fresh parser is waiting for a sync byte",
           patterned.state != 0xFFu && saturated.state != 0xFFu);
    expect("and a fresh parser does not stream", saturated.can_stream == 0u);
    expect("and a fresh parser has asked for no rate",
           patterned.telemetry_hz == 0u);
    expect("and a fresh parser has counted nothing",
           patterned.frames == 0u && patterned.bytes == 0u);
}

/* --- parameter metadata: what a parameter is ----------------------------- */

/*
 * PARAM_INFO pages the table's description of itself, and PARAM_HELP hands over
 * the prose beside one row.
 *
 * Both exist because the wire carried values and nothing else. A client that
 * wanted to show a range had to invent one, and the app in this repository did
 * exactly that: a thirty-two row snapshot of a ninety-two parameter board,
 * joined by name, which is a claim about the board dressed as a reading of it.
 *
 * The entries below are decoded by this file from the wire description in
 * docs/16-protocol.md rather than by the builder that wrote them, so the
 * encoding is checked against something other than itself. The table they run
 * against is the worst case the constants allow: the table full, every name as
 * long as the board's longest, every help longer than one frame, all three
 * types, one secret and one text default long enough that a page has to say it
 * cannot carry it.
 */

#define META_COUNT     AK_PARAMS_MAX  /* 96: the most the table holds */
#define META_HELP_LEN  150u           /* longer than a frame, so it walks */
#define META_TEXT_MAX  32u
#define META_LONG_DEF  50u            /* the text default that will not fit a
                                       * page at 60 bytes of buffer */

static ak_param_t    meta_items[AK_PARAMS_MAX];
static ak_params_t   meta_params;
static ak_proto_t    meta_proto;
static ak_proto_io_t meta_io;
static float         meta_float[AK_PARAMS_MAX];
static uint32_t      meta_u32[AK_PARAMS_MAX];
static char          meta_text[16][AK_PARAM_VALUE_MAX];
static char          meta_def[16][AK_PARAM_VALUE_MAX];
static char          meta_name[AK_PARAMS_MAX][AK_PARAM_NAME_MAX];
static char          meta_help[AK_PARAMS_MAX][META_HELP_LEN + 1u];
/* What each entry should carry, computed here rather than with the firmware's
 * own formatter: a test that spells a bound with the function under test is
 * checking that function against itself. */
static char          meta_low[AK_PARAMS_MAX][16];
static char          meta_high[AK_PARAMS_MAX][16];
static char          meta_def_text[AK_PARAMS_MAX][AK_PARAM_VALUE_MAX];

/* Which rows are text, and which of those is the secret and the long default.
 * Stated as constants because the test below reads them. */
#define META_SECRET_ROW 7u
#define META_LONG_ROW   84u

static int meta_is_text(unsigned row)
{
    return (row % 7u) == 0u;
}

static void meta_setup(void)
{
    unsigned n = 0;
    unsigned slot = 0;

    for (unsigned i = 0; i < META_COUNT; i++) {
        /* Twenty-two characters, which is the longest name the board has today
         * (`wing_descend_pitch_deg`), and different at the end - two rows that
         * differ only past the sixteenth byte are exactly what the name-length
         * bug was made of. */
        snprintf(meta_name[i], sizeof meta_name[i], "wing_descend_pitch_%03u", i);

        for (unsigned j = 0; j < META_HELP_LEN; j++) {
            meta_help[i][j] = (char)('a' + (char)((i + j) % 26u));
        }
        meta_help[i][META_HELP_LEN] = '\0';

        uint8_t group = (uint8_t)((i % (unsigned)(AK_PARAM_GROUP_COUNT - 1u)) + 1u);

        if (meta_is_text(i)) {
            unsigned len = (i == META_LONG_ROW) ? META_LONG_DEF : 3u;
            for (unsigned j = 0; j < len; j++) {
                meta_def[slot][j] = (char)('d' + (char)(j % 20u));
            }
            meta_def[slot][len] = '\0';

            uint8_t flags = (i == META_SECRET_ROW) ? (uint8_t)AK_PARAM_SECRET
                                                   : (uint8_t)0u;
            n = ak_params_add_text(meta_items, n, meta_name[i], meta_help[i],
                                   meta_text[slot], META_TEXT_MAX, flags,
                                   meta_def[slot], group);
            /* A secret's default is served the way its value is: the marker the
             * console prints, never the string. The long row is not the secret
             * one, so it keeps its text. */
            if (flags != 0u) {
                snprintf(meta_def_text[i], sizeof meta_def_text[i], "***");
            } else {
                snprintf(meta_def_text[i], sizeof meta_def_text[i], "%s",
                         meta_def[slot]);
            }
            slot++;
        } else if ((i % 3u) == 0u) {
            meta_u32[i] = (uint32_t)(i * 7u);
            n = ak_params_add_u32(meta_items, n, meta_name[i], meta_help[i],
                                  &meta_u32[i], 0u, 1000u, group);
            snprintf(meta_low[i], sizeof meta_low[i], "%u", 0u);
            snprintf(meta_high[i], sizeof meta_high[i], "%u", 1000u);
            snprintf(meta_def_text[i], sizeof meta_def_text[i], "%u",
                     meta_u32[i]);
        } else {
            meta_float[i] = (float)(i % 10u) * 0.125f;
            n = ak_params_add_float(meta_items, n, meta_name[i], meta_help[i],
                                    &meta_float[i], 3u, 0.0f, 100.0f, group);
            snprintf(meta_low[i], sizeof meta_low[i], "%.3f", 0.0);
            snprintf(meta_high[i], sizeof meta_high[i], "%.3f", 100.0);
            snprintf(meta_def_text[i], sizeof meta_def_text[i], "%.3f",
                     (double)meta_float[i]);
        }
    }

    ak_params_init(&meta_params, meta_items, n);

    memset(&meta_io, 0, sizeof meta_io);
    meta_io.params = &meta_params;
    meta_io.ctx = 0;
    ak_proto_init(&meta_proto);
}

static int meta_exchange(const uint8_t *request, unsigned request_length,
                         parsed_t *reply, unsigned capacity)
{
    uint8_t response[AK_PROTO_FRAME_MAX];
    unsigned written = 0;

    if (capacity > sizeof response) {
        capacity = sizeof response;
    }
    for (unsigned i = 0; i < request_length; i++) {
        unsigned got = ak_proto_feed(&meta_proto, &meta_io, request[i], 1000u,
                                     response, capacity);
        if (got > 0) {
            written = got;
        }
    }
    if (written == 0) {
        if (reply != 0) {
            memset(reply, 0, sizeof *reply);
        }
        return 0;
    }
    if (reply != 0) {
        *reply = parse_reply(response, written);
    }
    return (int)written;
}

typedef struct {
    const char *name;
    uint8_t     type;
    uint8_t     group;
    uint8_t     decimals;
    uint8_t     flags;
    uint8_t     max_len;
    const char *low;
    const char *high;
    const char *def;
} meta_entry_t;

static const char *meta_take_string(const parsed_t *reply, unsigned *at)
{
    const char *start = (const char *)&reply->payload[*at];

    while (*at < reply->length && reply->payload[*at] != 0u) {
        (*at)++;
    }
    if (*at >= reply->length) {
        return 0; /* ran off the end: not a terminated string */
    }
    (*at)++;
    return start;
}

/* One entry, read the way a client reads it - never a struct copy, because the
 * entry is what the wire says it is and not what this build happens to hold. */
static int meta_read_entry(const parsed_t *reply, unsigned *at, meta_entry_t *out)
{
    memset(out, 0, sizeof *out);

    out->name = meta_take_string(reply, at);
    if (out->name == 0 || *at + 4u > reply->length) {
        return 0;
    }
    out->type = reply->payload[(*at)++];
    out->group = reply->payload[(*at)++];
    out->decimals = reply->payload[(*at)++];
    out->flags = reply->payload[(*at)++];

    if (out->type == AK_PARAM_TEXT) {
        if (*at + 1u > reply->length) {
            return 0;
        }
        out->max_len = reply->payload[(*at)++];
    } else {
        out->low = meta_take_string(reply, at);
        out->high = meta_take_string(reply, at);
        if (out->low == 0 || out->high == 0) {
            return 0;
        }
    }
    out->def = meta_take_string(reply, at);
    return out->def != 0;
}

/*
 * The walk a configurator does on connect, and the two invariants that make it
 * safe: no page is longer than a frame may be, and the entries agree index by
 * index with what PARAM_GET answers for the same index.
 *
 * The second is the one that matters most, because a page and a value are two
 * enumeration paths and this table has already shipped a build where two
 * enumerations of it disagreed.
 */
static void test_parameter_metadata(void)
{
    uint8_t request[AK_PROTO_FRAME_MAX];
    parsed_t reply;
    unsigned index = 0;
    unsigned seen = 0;
    unsigned pages = 0;
    unsigned length = 0;

    meta_setup();

    while (index < META_COUNT && pages < 4u * META_COUNT) {
        uint8_t args[] = { (uint8_t)index };
        length = build_request(request, AK_PROTO_CMD_PARAM_INFO, args,
                               sizeof args);
        meta_exchange(request, length, &reply, AK_PROTO_FRAME_MAX);
        pages++;

        expect("a metadata page is a valid frame", reply.valid);
        expect("and its payload is inside the protocol's maximum",
               reply.valid && reply.length <= AK_PROTO_MAX_PAYLOAD);
        if (!reply.valid || reply.length < 3u) {
            break;
        }
        expect("a page answers from the index it was asked for",
               reply.payload[1] == (uint8_t)index);

        unsigned status = reply.payload[0];
        unsigned carried = reply.payload[2];
        /* Nothing in this table is too big for a frame, so a stop here would be
         * the walk failing, not the table ending. */
        expect("a page of this table is never refused", status == 0u);
        expect("and never comes back empty before the last row", carried > 0u);
        if (status != 0u || carried == 0u) {
            break;
        }

        unsigned at = 3u;
        for (unsigned k = 0; k < carried; k++) {
            meta_entry_t entry;
            unsigned row = index + k;
            int ok = meta_read_entry(&reply, &at, &entry);

            expect("an entry decodes to the end of the page", ok);
            if (!ok) {
                break;
            }
            expect("the entry is the row the walk is on",
                   strcmp(entry.name, meta_name[row]) == 0);
            expect("and it is the row's index that PARAM_GET answers for",
                   entry.group == meta_items[row].group &&
                   entry.flags == meta_items[row].flags);

            if (entry.type == AK_PARAM_TEXT) {
                expect("a text entry says how long its text may be",
                       entry.type == meta_items[row].type &&
                       entry.max_len == META_TEXT_MAX);
                expect("and carries the default `defaults` would restore",
                       strcmp(entry.def, meta_def_text[row]) == 0);
                expect("with no numeric bounds, which it does not have",
                       entry.low == 0 && entry.high == 0);
            } else {
                expect("a numeric entry spells its bounds the way its value is "
                       "spelled",
                       strcmp(entry.low, meta_low[row]) == 0 &&
                       strcmp(entry.high, meta_high[row]) == 0);
                expect("and its default, the same way",
                       strcmp(entry.def, meta_def_text[row]) == 0);
                expect("and says which of the two numeric types it is",
                       entry.type == meta_items[row].type);
            }
            seen++;
        }
        expect("the page is exactly as long as its entries",
               at == reply.length || carried == 0u);
        index += carried;
    }

    expect("the walk reaches every parameter in the table", seen == META_COUNT);
    expect("and it took more than one page, which is why it is paged",
           pages > 2u);

    /* A secret's default is the marker, not the string. A default is a value,
     * and the one value this table never prints is the Wi-Fi password - a
     * screen that could read it here could read it without asking for the
     * parameter at all. */
    expect("a secret's default is not served",
           strcmp(meta_def_text[META_SECRET_ROW], "***") == 0 &&
           strstr(meta_def[META_SECRET_ROW / 7u], "***") == 0);

    /* The two enumeration paths, row by row. PARAM_GET answers a value and
     * PARAM_INFO answers a description, and both name the same index - which is
     * what a client joins on, and what a stale build-time table got wrong. */
    unsigned mismatches = 0;
    for (unsigned row = 0; row < META_COUNT; row++) {
        uint8_t args[] = { (uint8_t)row };
        length = build_request(request, AK_PROTO_CMD_PARAM_GET, args,
                               sizeof args);
        meta_exchange(request, length, &reply, AK_PROTO_FRAME_MAX);
        if (!reply.valid || reply.payload[0] != 0u ||
            strcmp((char *)&reply.payload[1], meta_name[row]) != 0) {
            mismatches++;
        }
    }
    expect("PARAM_INFO and PARAM_GET name the same parameter at every index",
           mismatches == 0u);

    /* A request that names no index is refused rather than answered from zero:
     * the answer would look exactly like a complete table. */
    length = build_request(request, AK_PROTO_CMD_PARAM_INFO, 0, 0);
    meta_exchange(request, length, &reply, AK_PROTO_FRAME_MAX);
    expect("a page asked for from nowhere is refused",
           reply.valid && reply.payload[0] == 1u);
}

/*
 * A page that cannot hold even one entry says so, and one that can says which.
 *
 * This is the case the design has to answer rather than hang on. A client that
 * read `carried == 0` as "the table ends here" would stop early and never learn
 * the row exists; one that re-asked from the same index would ask forever. A
 * caller with a smaller array than a frame is not hypothetical - it is what
 * `ak_proto_feed`'s capacity argument is for - and a long text default is the
 * row that shows it, because a text entry is the longest one there is.
 */
static void test_metadata_page_that_cannot_fit(void)
{
    uint8_t request[AK_PROTO_FRAME_MAX];
    parsed_t reply;

    meta_setup();

    uint8_t long_args[] = { (uint8_t)META_LONG_ROW };
    unsigned length = build_request(request, AK_PROTO_CMD_PARAM_INFO, long_args,
                                    sizeof long_args);
    meta_exchange(request, length, &reply, 60u);
    expect("a page too small for the row it is on refuses rather than truncates",
           reply.valid && reply.payload[0] == 2u && reply.payload[1] == META_LONG_ROW &&
           reply.payload[2] == 0u);
    expect("and the refusal is distinguishable from the end of the table",
           reply.payload[1] < META_COUNT);

    /* The next row is a numeric one, and it does fit the same small page - so
     * the refusal above is about the entry and not about the capacity. */
    uint8_t next_args[] = { (uint8_t)(META_LONG_ROW + 1u) };
    length = build_request(request, AK_PROTO_CMD_PARAM_INFO, next_args,
                           sizeof next_args);
    meta_exchange(request, length, &reply, 60u);
    expect("and the row after it is carried by the same small page",
           reply.valid && reply.payload[0] == 0u && reply.payload[2] == 1u);
    expect("with its name intact", reply.valid &&
           strcmp((char *)&reply.payload[3], meta_name[META_LONG_ROW + 1u]) == 0);
}

/*
 * The walk again, at every frame size a caller might hand the builder.
 *
 * `capacity` is a caller's argument - `ak_proto_feed`'s, and behind it the array
 * the console link, the network link and the simulator each declare. A page that
 * is right at 104 bytes and wrong at 60 is a page that is wrong, because the
 * size is not the builder's to assume, and a page whose entries are right at one
 * size and cut at another is the same failure wearing a different number.
 *
 * This is where the boundary is hit on purpose. The whole table is walked once
 * per capacity, and at every size the same two things have to hold: each entry
 * arrives terminated, and the page is exactly as long as the entries it carried.
 * The failure this exists to keep out wrote a string's characters and not its
 * terminator when the string ended exactly at the end of the room - a page whose
 * length and CRC were both correct, its last entry reading a bound out of the
 * next entry's bytes. It was unreachable while every reply stopped well short of
 * capacity; the board's own `arm_accel_lpf_hz` reached it the first time a
 * builder filled a frame, and the synthetic table reaches it here the moment the
 * entry happens to land on the boundary for one of these ninety-five sizes.
 */
static void test_metadata_across_every_frame_size(void)
{
    uint8_t request[AK_PROTO_FRAME_MAX];
    parsed_t reply;
    unsigned previous = 0;

    meta_setup();

    for (unsigned capacity = 10u; capacity <= AK_PROTO_FRAME_MAX; capacity++) {
        unsigned index = 0;
        unsigned pages = 0;
        const char *fault = 0;

        while (index < META_COUNT && pages <= META_COUNT && fault == 0) {
            uint8_t args[] = { (uint8_t)index };
            unsigned length = build_request(request, AK_PROTO_CMD_PARAM_INFO,
                                            args, sizeof args);
            int written = meta_exchange(request, length, &reply, capacity);
            pages++;

            if (!reply.valid) {
                fault = "the reply is not a valid frame";
                break;
            }
            /* The invariant the capacity argument *is*: a builder handed an
             * array never writes past the end of it. */
            if ((unsigned)written > capacity) {
                fault = "the reply is longer than the array given";
                break;
            }
            if (reply.length > AK_PROTO_MAX_PAYLOAD) {
                fault = "the payload is over the protocol's maximum";
                break;
            }
            if (reply.payload[1] != (uint8_t)index) {
                fault = "the page answers from the wrong index";
                break;
            }

            unsigned status = reply.payload[0];
            unsigned carried = reply.payload[2];
            if (status != 0u) {
                /* The one refusal a page has, and it has to carry nothing: the
                 * walk stops here rather than being told a shorter truth. */
                if (status != 2u || carried != 0u) {
                    fault = "a page was refused without saying so";
                }
                break;
            }
            if (carried == 0u) {
                break;
            }

            unsigned at = 3u;
            for (unsigned k = 0; k < carried; k++) {
                meta_entry_t entry;
                if (!meta_read_entry(&reply, &at, &entry)) {
                    fault = "an entry is not terminated";
                    break;
                }
                if (strcmp(entry.name, meta_name[index + k]) != 0) {
                    fault = "an entry is not the row the walk is on";
                    break;
                }
            }
            if (fault != 0) {
                break;
            }
            if (at != reply.length) {
                fault = "the page is not exactly its entries";
                break;
            }
            index += carried;
        }

        if (fault == 0 && index < previous) {
            fault = "a larger array served fewer rows than a smaller one";
        }
        previous = index;

        char name[128];
        snprintf(name, sizeof name, "capacity %u serves whole entries "
                 "(%u rows, %s)", capacity, index,
                 fault ? fault : "no fault");
        expect(name, fault == 0);
    }

    /* And the top of the range is the whole table: the sweep's floor is a
     * property of the frame size, not of the table. */
    char whole[96];
    snprintf(whole, sizeof whole, "the full table arrives at the protocol's "
             "own frame size (%u of %u rows)", previous, (unsigned)META_COUNT);
    expect(whole, previous == META_COUNT);
}

/*
 * The prose, walked by offset.
 *
 * Offset and not a page number, which is what makes "the text was longer than a
 * frame" not a case at all: a client is done when offset + len == total, and
 * there is no path through this command that loses the tail of a help string
 * quietly. The help strings here are longer than a frame on purpose.
 */
static void test_parameter_help(void)
{
    uint8_t request[AK_PROTO_FRAME_MAX];
    parsed_t reply;
    char got[META_HELP_LEN + 1u];
    unsigned row = 3u;
    unsigned offset = 0;
    unsigned rounds = 0;
    unsigned total;

    meta_setup();
    total = (unsigned)strlen(meta_help[row]);
    memset(got, 0, sizeof got);

    while (rounds < 32u) {
        uint8_t args[] = { (uint8_t)row, (uint8_t)(offset & 0xFFu),
                           (uint8_t)(offset >> 8) };
        unsigned length = build_request(request, AK_PROTO_CMD_PARAM_HELP, args,
                                        sizeof args);
        meta_exchange(request, length, &reply, AK_PROTO_FRAME_MAX);
        rounds++;

        expect("a help page is a valid frame", reply.valid);
        expect("and its payload is inside the protocol's maximum",
               reply.valid && reply.length <= AK_PROTO_MAX_PAYLOAD);
        if (!reply.valid || reply.length < 7u) {
            break;
        }

        unsigned back = (unsigned)(reply.payload[2] |
                                   ((unsigned)reply.payload[3] << 8));
        unsigned said = (unsigned)(reply.payload[4] |
                                   ((unsigned)reply.payload[5] << 8));
        unsigned len = reply.payload[6];

        expect("the help is for the row that was asked for",
               reply.payload[1] == (uint8_t)row);
        expect("the offset answered is the offset asked for", back == offset);
        expect("and the total is the whole text, not this page of it",
               said == total);
        expect("the page is exactly as long as it says it is",
               reply.length == 7u + len);
        expect("no page is empty while there is text left",
               len > 0u || offset == total);
        if (7u + len > reply.length) {
            break;
        }
        if (offset + len <= META_HELP_LEN) {
            memcpy(&got[offset], &reply.payload[7], len);
        }
        offset += len;
        if (offset >= total) {
            break;
        }
    }

    expect("walking the help by offset returns the whole text, unaltered",
           offset == total && strcmp(got, meta_help[row]) == 0);
    expect("and it took more than one frame, which is why it is a walk",
           rounds > 1u);

    /* Past the end is an empty tail rather than an error: a client that asks
     * once too often gets the same answer as one that stops, and neither has to
     * know which it is. */
    uint8_t past[] = { (uint8_t)row, (uint8_t)((total + 5u) & 0xFFu),
                       (uint8_t)((total + 5u) >> 8) };
    unsigned length = build_request(request, AK_PROTO_CMD_PARAM_HELP, past,
                                    sizeof past);
    meta_exchange(request, length, &reply, AK_PROTO_FRAME_MAX);
    expect("an offset past the end is an empty tail, not an error",
           reply.valid && reply.payload[0] == 0u && reply.payload[6] == 0u);

    uint8_t short_args[] = { 0u, 0u };
    length = build_request(request, AK_PROTO_CMD_PARAM_HELP, short_args,
                           sizeof short_args);
    meta_exchange(request, length, &reply, AK_PROTO_FRAME_MAX);
    expect("a help request that names no offset is refused",
           reply.valid && reply.payload[0] == 1u);

    uint8_t past_end[] = { (uint8_t)META_COUNT, 0u, 0u };
    length = build_request(request, AK_PROTO_CMD_PARAM_HELP, past_end,
                           sizeof past_end);
    meta_exchange(request, length, &reply, AK_PROTO_FRAME_MAX);
    expect("and a row past the end is refused rather than read",
           reply.valid && reply.payload[0] == 1u);
}

/* --- the console link: two readers on one wire --------------------------- */

/*
 * The config protocol and the human console share the port the console is on,
 * told apart only by a frame's first byte. Neither reader can recognise the
 * other's bytes, so the arbitration between them is the one place in the
 * firmware where a byte can be handed to the wrong reader - and it was, in two
 * directions at once, for as long as the rule was written out by hand inside
 * main.c's receive loop.
 *
 * Both directions are driven here through the same entry point that loop calls,
 * so this is a test of the firmware's actual dispatch and not of a copy of it.
 */

static char     console_text[2048];
static size_t   console_used;
static ak_cli_t console_cli;
static ak_flight_t console_flight;

static int console_out(const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    int written = vsnprintf(console_text + console_used,
                            sizeof console_text - console_used, fmt, ap);
    va_end(ap);
    if (written > 0 && console_used + (size_t)written < sizeof console_text) {
        console_used += (size_t)written;
    }
    return written;
}

static void console_setup(void)
{
    ak_cli_io_t cli_io;

    memset(&cli_io, 0, sizeof cli_io);
    cli_io.out = console_out;
    memset(&console_flight, 0, sizeof console_flight);
    ak_cli_init(&console_cli, &cli_io, &params, &console_flight);
    console_text[0] = '\0';
    console_used = 0;
}

static int console_said(const char *needle)
{
    console_text[sizeof console_text - 1u] = '\0';
    return strstr(console_text, needle) != 0;
}

static void console_forget_output(void)
{
    console_text[0] = '\0';
    console_used = 0;
}

/* Every byte of `bytes` arriving on the shared port, in order, at one instant -
 * the way a port hands over a burst it has already buffered. */
static void feed_all(const uint8_t *bytes, unsigned length, uint32_t now_ms)
{
    uint8_t response[AK_PROTO_FRAME_MAX];

    for (unsigned i = 0; i < length; i++) {
        (void)ak_console_link_feed(&proto, &io, &console_cli, bytes[i], now_ms,
                                   response, sizeof response);
    }
}

static void type_line(const char *line, uint32_t now_ms)
{
    feed_all((const uint8_t *)line, (unsigned)strlen(line), now_ms);
}

static void test_console_link(void)
{
    uint8_t   request[AK_PROTO_FRAME_MAX];
    unsigned  length;
    uint32_t  after_gap = 1000u + AK_PROTO_GAP_MS + 1u;

    /* A command typed straight after a well-formed frame. Every byte of that
     * frame belonged to the parser, so nothing of it is in the line buffer and
     * the command runs normally. This half already worked; it is here because
     * the fix for the other half must not cost it. */
    setup();
    console_setup();
    length = build_request(request, AK_PROTO_CMD_HELLO, 0, 0);
    feed_all(request, length, 1000u);
    expect("a well-formed frame is counted and leaves no half-typed line",
           proto.frames == 1u && console_cli.len == 0u &&
           console_cli.overflow == 0u);
    console_forget_output();
    type_line("version\r", 1100u);
    expect("a command typed after a frame is still answered",
           console_said("product:"));

    /* A frame the parser abandons over a bad length. It gives up on the length
     * byte, before the sender has finished, so the rest of that frame is still
     * arriving - and those bytes are not somebody typing. They used to be: the
     * printable ones went into the line buffer, and a 0x0D or 0x0A among them
     * printed a prompt. That prompt is what a script reads to mean "the last
     * command has finished" (bench_check.py's read_until), so a prompt nobody
     * asked for is a script that stops reading in the middle of an answer. */
    setup();
    console_setup();
    {
        static const uint8_t abandoned[] = {
            AK_PROTO_SYNC1, AK_PROTO_SYNC2, AK_PROTO_VERSION,
            AK_PROTO_CMD_STATUS, 200u, /* past AK_PROTO_MAX_PAYLOAD */
            'j', 'u', 'n', 'k', '\r', '\n', 0x41u, 0x42u
        };
        feed_all(abandoned, sizeof abandoned, 1000u);
    }
    expect("the tail of a frame abandoned over a bad length is not typed at "
           "the console", !console_said("junk"));
    expect("and it prints no prompt nobody asked for", !console_said("ak> "));
    expect("while the parser still records the bad length",
           proto.bad_length == 1u);

    /* The wire is the frame's until it goes quiet for longer than the gap. A
     * line arriving inside that window belongs to the frame, not to a person -
     * so this must stay unanswered, or the firmware would be guessing. */
    console_forget_output();
    type_line("version\r", 1000u);
    expect("a line sent while the wire is still draining is not run",
           !console_said("product:"));

    console_forget_output();
    type_line("version\r", after_gap);
    expect("and once the gap has passed the console answers again",
           console_said("product:"));

    /* The other direction: a frame that stops part way. Until the gap expires
     * the parser reports itself busy, and a text byte handed to it there is
     * discarded - so a command typed into that window lost its first character
     * and ran as "unknown command: ersion". */
    setup();
    console_setup();
    length = build_request(request, AK_PROTO_CMD_STATUS, 0, 0);
    feed_all(request, 6u, 1000u);        /* the frame, cut off mid-checksum */
    expect("the cut-off frame is not a frame", proto.frames == 0u);
    console_forget_output();
    type_line("version\r", after_gap);
    expect("a command after a half-frame keeps its first character",
           console_said("version"));
    expect("and it runs rather than being called unknown",
           console_said("product:"));
}

/* The receiver's state as the tests hand it to the protocol, and the reply as
 * this file parses it back out of the wire description. */
static ak_proto_rc_t rc_out;

static void fake_rc_state(void *ctx, ak_proto_rc_t *out)
{
    (void)ctx;
    *out = rc_out;
}

typedef struct {
    int      ok;
    uint8_t  status;
    uint8_t  flags;
    uint8_t  protocol;
    uint8_t  count;
    uint16_t raw[AK_PROTO_RC_MAX];
    int16_t  sticks[4];
    uint8_t  switches;
    uint32_t bytes;
    uint32_t frames;
    uint32_t crc_errors;
    uint32_t rejected;
    uint32_t lost;
    uint32_t failsafe_frames;
    uint32_t dropped;
} rc_reply_t;

/* Parses an RC_CHANNELS reply strictly: `ok` means the payload was exactly the
 * size its own `count` implies, which is the property the clamp below exists to
 * keep. A parser that walked off the end of a short payload would be reading
 * the sticks out of the counters. */
static rc_reply_t parse_rc(const parsed_t *reply)
{
    rc_reply_t out;
    memset(&out, 0, sizeof out);

    if (!reply->valid || reply->length < 1u) {
        return out;
    }
    out.status = reply->payload[0];
    if (out.status != AK_PROTO_RC_OK || reply->length < 4u) {
        return out;
    }

    unsigned at = 1;
    out.flags = reply->payload[at++];
    out.protocol = reply->payload[at++];
    out.count = reply->payload[at++];
    if (out.count > AK_PROTO_RC_MAX) {
        return out;
    }
    /* Four header bytes, two per channel, four sticks, the switches, and seven
     * counters. */
    unsigned expected = 4u + (unsigned)out.count * 2u + 8u + 1u + 28u;
    if (reply->length != expected) {
        return out;
    }

    for (unsigned i = 0; i < out.count; i++) {
        out.raw[i] = (uint16_t)(reply->payload[at] |
                                ((uint16_t)reply->payload[at + 1] << 8));
        at += 2;
    }
    for (unsigned i = 0; i < 4; i++) {
        out.sticks[i] = (int16_t)(reply->payload[at] |
                                  ((uint16_t)reply->payload[at + 1] << 8));
        at += 2;
    }
    out.switches = reply->payload[at++];

    uint32_t *counters[7] = { &out.bytes,      &out.frames, &out.crc_errors,
                              &out.rejected,   &out.lost,   &out.failsafe_frames,
                              &out.dropped };
    for (unsigned i = 0; i < 7; i++) {
        *counters[i] = (uint32_t)reply->payload[at] |
                       ((uint32_t)reply->payload[at + 1] << 8) |
                       ((uint32_t)reply->payload[at + 2] << 16) |
                       ((uint32_t)reply->payload[at + 3] << 24);
        at += 4;
    }

    out.ok = 1;
    return out;
}

/* A live frame, as a board that has one would describe it. */
static void rc_frame(uint16_t base)
{
    memset(&rc_out, 0, sizeof rc_out);
    rc_out.flags = AK_PROTO_RC_LINK | AK_PROTO_RC_DECODED | AK_PROTO_RC_TELEMETRY;
    rc_out.protocol = 0;
    rc_out.count = 8;
    for (unsigned i = 0; i < 8; i++) {
        rc_out.raw[i] = (uint16_t)(base + i);
    }
    rc_out.sticks[0] = 125;
    rc_out.sticks[1] = -250;
    rc_out.sticks[2] = 0;
    rc_out.sticks[3] = 1000;
    rc_out.switches = AK_PROTO_RC_ANGLE;
    rc_out.bytes = 4000000000u; /* past 2^31, so a u32 read as i32 shows up */
    rc_out.frames = 12345678u;
    rc_out.crc_errors = 7;
    rc_out.rejected = 3;
    rc_out.dropped = 65537u; /* past 2^16, so a u16 slip shows up */
}

static void test_rc_channels(void)
{
    uint8_t request[8];
    parsed_t reply;
    rc_reply_t rc;

    /* A board with no receiver at all. One byte and no more: a frame of zeros
     * here would be a claim about a receiver this board does not have. */
    io.rc_state = 0;
    exchange(request, build_request(request, AK_PROTO_CMD_RC_CHANNELS, 0, 0),
             &reply, 1000);
    expect("a board with no receiver input says so, rather than answering "
           "zeros that would read as a receiver with no link",
           reply.valid && reply.length == 1u &&
           reply.payload[0] == AK_PROTO_RC_NONE);

    io.rc_state = fake_rc_state;

    /* No frame yet: the counters are real and the sticks are not. */
    memset(&rc_out, 0, sizeof rc_out);
    rc_out.protocol = 1;
    rc_out.count = 8;
    rc_out.dropped = 12u;
    exchange(request, build_request(request, AK_PROTO_CMD_RC_CHANNELS, 0, 0),
             &reply, 1000);
    rc = parse_rc(&reply);
    expect("a receiver that has never framed answers with the link clear",
           rc.ok && rc.status == AK_PROTO_RC_OK &&
           (rc.flags & AK_PROTO_RC_LINK) == 0);
    expect("and with the sticks flagged as not decoded",
           (rc.flags & AK_PROTO_RC_DECODED) == 0);
    expect("and its counters are still reported, so a silent receiver is not "
           "the same answer as an absent one",
           rc.dropped == 12u && rc.frames == 0u);

    /* The distinction the flag exists for, and the only one that separates two
     * replies whose sticks are both all zero: a frame the decode would not use,
     * and a handset sitting centred. Same bytes below the flag, different
     * meaning, and a screen that drew them the same way would be telling a
     * person their sticks are centred when the board has no idea. */
    rc_out.flags = AK_PROTO_RC_LINK;
    exchange(request, build_request(request, AK_PROTO_CMD_RC_CHANNELS, 0, 0),
             &reply, 1000);
    rc_reply_t undecoded = parse_rc(&reply);

    rc_out.flags = AK_PROTO_RC_LINK | AK_PROTO_RC_DECODED;
    exchange(request, build_request(request, AK_PROTO_CMD_RC_CHANNELS, 0, 0),
             &reply, 1000);
    rc_reply_t centred = parse_rc(&reply);

    expect("zeroed sticks with the decode flag clear and zeroed sticks with it "
           "set are told apart by that flag alone",
           undecoded.ok && centred.ok && undecoded.flags != centred.flags &&
           undecoded.sticks[0] == centred.sticks[0] &&
           undecoded.sticks[3] == centred.sticks[3]);

    /* A real frame. */
    rc_frame(172u);
    exchange(request, build_request(request, AK_PROTO_CMD_RC_CHANNELS, 0, 0),
             &reply, 1000);
    rc = parse_rc(&reply);
    expect("a live frame answers with the link and the decode both set",
           rc.ok && (rc.flags & AK_PROTO_RC_LINK) != 0 &&
           (rc.flags & AK_PROTO_RC_DECODED) != 0);
    expect("the counts come back in order, little-endian, as they arrived",
           rc.count == 8u && rc.raw[0] == 172u && rc.raw[7] == 179u);

    /* The sticks are the firmware's numbers, forwarded. If the protocol ever
     * started recomputing them this is where it would show: these values are
     * not derivable from the counts above at any deadband. */
    expect("the sticks are the firmware's own per-mille numbers, passed "
           "through and not recomputed from the counts",
           rc.sticks[0] == 125 && rc.sticks[1] == -250 && rc.sticks[2] == 0 &&
           rc.sticks[3] == 1000);
    expect("and a negative stick survives as a negative, not as a large "
           "unsigned one", rc.sticks[1] == -250);
    expect("the switches travel with them", rc.switches == AK_PROTO_RC_ANGLE);

    expect("counters wider than 16 bits survive, so none of them is being "
           "written through a narrower field",
           rc.bytes == 4000000000u && rc.frames == 12345678u &&
           rc.dropped == 65537u && rc.crc_errors == 7u && rc.rejected == 3u);

    /* The two board facts a client needs to explain a link that is not
     * working. */
    rc_out.flags = AK_PROTO_RC_LINK | AK_PROTO_RC_DECODED |
                   AK_PROTO_RC_NO_INVERTER;
    rc_out.protocol = 1;
    rc_out.failsafe_frames = 5u;
    rc_out.lost = 9u;
    exchange(request, build_request(request, AK_PROTO_CMD_RC_CHANNELS, 0, 0),
             &reply, 1000);
    rc = parse_rc(&reply);
    expect("an SBUS link on a board with no inverter says so",
           rc.ok && (rc.flags & AK_PROTO_RC_NO_INVERTER) != 0);
    expect("and SBUS reports no return path, because it has none",
           (rc.flags & AK_PROTO_RC_TELEMETRY) == 0);
    expect("and the two counters that only SBUS keeps are carried",
           rc.lost == 9u && rc.failsafe_frames == 5u);

    rc_out.flags = AK_PROTO_RC_LINK | AK_PROTO_RC_FAILSAFE;
    exchange(request, build_request(request, AK_PROTO_CMD_RC_CHANNELS, 0, 0),
             &reply, 1000);
    rc = parse_rc(&reply);
    expect("a receiver in its own failsafe is distinguishable from a live one",
           rc.ok && (rc.flags & AK_PROTO_RC_FAILSAFE) != 0 &&
           (rc.flags & AK_PROTO_RC_DECODED) == 0);

    /* A caller that filled `count` beyond what the struct holds. The reply has
     * to stay self-consistent: `count` must agree with the number of entries
     * actually written, or a client reads the first stick as channel 9. */
    rc_frame(172u);
    rc_out.count = 200u;
    exchange(request, build_request(request, AK_PROTO_CMD_RC_CHANNELS, 0, 0),
             &reply, 1000);
    rc = parse_rc(&reply);
    expect("a count past what the struct holds is clamped to what was written, "
           "so the reply cannot say a length it did not write",
           rc.ok && rc.count == AK_PROTO_RC_MAX);
    expect("and the sticks are still where the clamped count puts them",
           rc.sticks[0] == 125 && rc.sticks[3] == 1000);

    /* The frame bound, checked rather than reasoned about: a full reply is the
     * biggest thing this opcode can produce. */
    rc_frame(172u);
    rc_out.count = AK_PROTO_RC_MAX;
    exchange(request, build_request(request, AK_PROTO_CMD_RC_CHANNELS, 0, 0),
             &reply, 1000);
    rc = parse_rc(&reply);
    expect("a full reply parses, and its payload is inside the protocol's "
           "maximum",
           rc.ok && reply.length <= AK_PROTO_MAX_PAYLOAD);
    expect("and a full reply is not truncated: the size its count implies is "
           "the size it has",
           reply.length == 4u + (unsigned)rc.count * 2u + 8u + 1u + 28u);

    io.rc_state = 0;
}

/* --- SENSOR_INFO ---------------------------------------------------------- */

/*
 * The sizes the wire description gives each body, counted here rather than
 * taken from the structs. A struct's sizeof is a compiler's opinion - it
 * includes padding and it changes when a field is added - and this file's job
 * is to hold the *documented* layout, so a field added to ak_proto_imu_t
 * without a line here is a check that fails rather than a reply that grows.
 */
static const unsigned sensor_body_length[AK_PROTO_SENSOR_TOPICS] = {
    /* IMU     */ 12u + 1u + 1u + 6u + 6u + 6u + 6u + 4u + 4u,
    /* BARO    */ 12u + 4u + 2u + 1u + 4u + 4u + 1u + 4u + 4u + 4u + 4u + 4u + 4u,
    /* RANGE   */ 12u + 1u + 2u + 4u + 4u + 4u + 4u + 4u + 4u + 4u + 4u + 2u,
    /* BATTERY */ 1u + 1u + 1u + 1u + 2u + 2u + 2u + 2u + 1u + 2u + 2u +
        4u + 4u + 4u,
    /* GPS     */ 1u + 1u + 1u + 1u + 1u + 4u + 4u + 4u + 4u + 4u + 1u + 4u +
        4u + 4u + 4u + 1u + 1u + 4u + 4u + 4u,
};

/* What the fake board answers with: `present` here, and a body this file fills
 * per topic so a reply can be checked against something that is not all
 * zeros - the failure a zeroed body would hide. */
static ak_proto_sensor_t sensor_out;
static uint8_t           sensor_present;

static void fake_sensor_state(void *ctx, uint8_t topic, ak_proto_sensor_t *out)
{
    (void)ctx;
    *out = sensor_out;
    out->topic = topic;
    out->present = sensor_present;
}

static int exchange_sensor(uint8_t topic, parsed_t *reply)
{
    uint8_t request[8];
    return exchange(request,
                    build_request(request, AK_PROTO_CMD_SENSOR_INFO, &topic, 1u),
                    reply, 1000);
}

static void test_sensor_info(void)
{
    uint8_t request[8];
    parsed_t reply;

    /* A board that reports no sensors at all. Every topic is refused, in range
     * or not - which is true of it, and is why there is no separate status for
     * "this build has no sensors". */
    io.sensor_state = 0;
    for (unsigned topic = 0; topic < AK_PROTO_SENSOR_TOPICS; topic++) {
        exchange_sensor((uint8_t)topic, &reply);
        expect("a board with no sensor reporting refuses every topic it is "
               "asked about",
               reply.valid && reply.length == 3u &&
                   reply.payload[0] == AK_PROTO_SENSOR_NO_SUCH &&
                   reply.payload[1] == topic && reply.payload[2] == 0u);
    }

    io.sensor_state = fake_sensor_state;

    /* A topic this build does not have, and a request that named nothing. The
     * second is the one worth checking: "no topic byte" must not be read as
     * topic zero, because a truncated frame would then answer with the IMU's
     * numbers and a client would have no way to tell. */
    exchange_sensor((uint8_t)AK_PROTO_SENSOR_TOPICS, &reply);
    expect("a topic past the last one is refused rather than clamped to it",
           reply.valid && reply.length == 3u &&
               reply.payload[0] == AK_PROTO_SENSOR_NO_SUCH &&
               reply.payload[1] == AK_PROTO_SENSOR_TOPICS);

    exchange_sensor(200u, &reply);
    expect("and so is a topic value that could never be one",
           reply.valid && reply.length == 3u &&
               reply.payload[0] == AK_PROTO_SENSOR_NO_SUCH &&
               reply.payload[1] == 200u);

    exchange(request, build_request(request, AK_PROTO_CMD_SENSOR_INFO, 0, 0),
             &reply, 1000);
    expect("a request that named no topic is refused, not answered with the "
           "first one - a frame cut short must not read as a question about "
           "the imu",
           reply.valid && reply.length == 3u &&
               reply.payload[0] == AK_PROTO_SENSOR_NO_SUCH &&
               reply.payload[1] == AK_PROTO_SENSOR_TOPICS);

    /* Nothing fitted: the body is not there. This is the whole point of the
     * opcode's shape, so it is checked on every topic rather than one. */
    sensor_present = 0;
    for (unsigned topic = 0; topic < AK_PROTO_SENSOR_TOPICS; topic++) {
        exchange_sensor((uint8_t)topic, &reply);
        expect("a topic this build has but this board has not fitted answers "
               "with the body left off",
               reply.valid && reply.length == 3u &&
                   reply.payload[0] == AK_PROTO_SENSOR_OK &&
                   reply.payload[1] == topic && reply.payload[2] == 0u);
    }

    /* The distinction the present byte exists for. Both of these replies carry
     * every body byte as zero; one of them means "no barometer" and the other
     * means "a barometer reading zero pressure", and they differ only in their
     * length and one byte. A client that read the body without checking
     * `present` would see the same numbers twice. */
    memset(&sensor_out, 0, sizeof sensor_out);
    sensor_present = 1;
    exchange_sensor(AK_PROTO_SENSOR_BARO, &reply);
    unsigned absent_length = 3u;
    expect("a fitted sensor reading zero is a longer reply than an absent one, "
           "so the two cannot be read the same way",
           reply.valid && reply.length == absent_length + sensor_body_length[AK_PROTO_SENSOR_BARO] &&
               reply.payload[2] == 1u);

    sensor_present = 0;
    exchange_sensor(AK_PROTO_SENSOR_BARO, &reply);
    expect("and the absent one is exactly as long as it says it is",
           reply.valid && reply.length == absent_length && reply.payload[2] == 0u);

    /* Every topic, at its documented length, with a body that is not zeros -
     * so a reply whose length happens to be right but whose fields are in the
     * wrong order fails here rather than passing on a length alone. */
    for (unsigned topic = 0; topic < AK_PROTO_SENSOR_TOPICS; topic++) {
        memset(&sensor_out, 0, sizeof sensor_out);
        sensor_present = 1;
        /* A pattern keyed on the topic and the byte's own offset: every byte of
         * the body is different from its neighbour, so a field written twice or
         * skipped moves everything after it. */
        uint8_t *bytes = (uint8_t *)&sensor_out.as;
        for (unsigned i = 0; i < sizeof sensor_out.as; i++) {
            bytes[i] = (uint8_t)(1u + topic * 31u + i);
        }
        exchange_sensor((uint8_t)topic, &reply);
        expect("a fitted sensor's reply is the length the wire description "
               "gives it",
               reply.valid &&
                   reply.length == 3u + sensor_body_length[topic] &&
                   reply.payload[0] == AK_PROTO_SENSOR_OK &&
                   reply.payload[1] == topic && reply.payload[2] == 1u);
        expect("and its payload is inside the protocol's maximum",
               reply.valid && reply.length <= AK_PROTO_MAX_PAYLOAD);
    }

    /* The driver name is a fixed-width field, so a two-character id and a
     * ten-character one produce the same length. That is what lets a client
     * find the numbers after it without measuring. */
    memset(&sensor_out, 0, sizeof sensor_out);
    sensor_present = 1;
    memcpy(sensor_out.as.imu.driver, "ab", 3);
    sensor_out.as.imu.samples = 0x11223344u;
    exchange_sensor(AK_PROTO_SENSOR_IMU, &reply);
    expect("a short driver name does not shorten the reply",
           reply.valid && reply.length == 3u + sensor_body_length[AK_PROTO_SENSOR_IMU]);
    expect("and the field is padded rather than left as whatever followed it",
           reply.payload[3] == 'a' && reply.payload[4] == 'b' &&
               reply.payload[5] == 0u &&
               reply.payload[3u + AK_PROTO_SENSOR_NAME - 1u] == 0u);
    expect("and the numbers after the name are where the fixed width puts them",
           reply.payload[3u + 12u + 1u + 1u + 6u + 6u + 6u + 6u] == 0x44u &&
               reply.payload[3u + 12u + 1u + 1u + 6u + 6u + 6u + 6u + 3u] == 0x11u);

    io.sensor_state = 0;
}

/*
 * Every driver name this tree ships has to fit the fixed field.
 *
 * The alternative is a name cut in half on a wire nobody looks at until a
 * bench, and the tables are right here - the same argument as the board-name
 * checks, which exist because "it will fit" was wrong once.
 */
static void test_sensor_names_fit(void)
{
    for (const ak_imu_driver_t *const *slot = ak_imu_drivers; *slot != 0; slot++) {
        expect("every imu driver's name fits the wire's fixed name field",
               strlen((*slot)->name) < AK_PROTO_SENSOR_NAME);
    }
    for (const ak_baro_driver_t *const *slot = ak_baro_drivers; *slot != 0; slot++) {
        expect("every barometer driver's name fits it too",
               strlen((*slot)->name) < AK_PROTO_SENSOR_NAME);
    }
    for (const ak_rangefinder_driver_t *const *slot = ak_range_drivers; *slot != 0;
         slot++) {
        expect("and every rangefinder driver's name",
               strlen((*slot)->name) < AK_PROTO_SENSOR_NAME);
    }
}

void test_protocol(void)
{
    test_parser_initialisation();
    test_hello();
    test_hello_capability_word();
    test_parameters();
    test_write_gate();
    test_param_default();
    test_no_gate_without_a_board();
    test_rc_channels();
    test_sensor_info();
    test_sensor_names_fit();
    test_status();
    test_log();
    test_log_sources();
    test_telemetry();
    test_parameter_metadata();
    test_metadata_page_that_cannot_fit();
    test_metadata_across_every_frame_size();
    test_parameter_help();
    test_bad_frames();
    test_console_link();
}
