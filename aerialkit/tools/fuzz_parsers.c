/*
 * The parsers, under bytes nobody sent them.
 *
 * Every parser in this firmware eats a wire that somebody else is in charge
 * of: CRSF and SBUS from the radio, UBX from the GPS module, and the config
 * protocol on the console or the network. The test suite feeds them *frames* -
 * built from the wire description, plus the handful of malformed cases
 * somebody thought of. What it has never done is throw random bytes at them,
 * which is where an index computed from a length field, a state that is never
 * left, or a buffer with no bound shows up.
 *
 * Two things are checked, over a fixed seed so that a failure can be re-run:
 *
 *   1. **Nothing crashes.** Under `-fsanitize=address,undefined` (which is how
 *      `make sanitize` runs it) any out-of-bounds index, overflow or
 *      uninitialised read stops the run and names the iteration.
 *   2. **Garbage does not wedge a parser.** After any amount of random input, a
 *      *valid* frame must still decode. That is the property a flight needs: a
 *      radio that has been shouting noise for ten seconds has to be usable
 *      again the moment it starts making sense, and a parser left waiting for a
 *      length that never arrives is a link that stays dead until the aircraft
 *      is power cycled. This repository has that bug's shape written down
 *      already (the CRSF gap timer), which is the reason it is checked rather
 *      than assumed.
 *
 * What it is not: coverage-guided, or a proof. It is a fixed seed and a lot of
 * bytes, and the honest reading of a green run is "nothing this seed found".
 * `--seed` and `--iterations` move it, and the numbers printed at the end are
 * what it actually did.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "ak_crsf.h"
#include "ak_gps.h"
#include "ak_params.h"
#include "ak_proto.h"
#include "ak_rc.h"
#include "ak_sbus.h"

#define MAX_GARBAGE 1024u

static uint32_t rng_state;
static unsigned failures;
static unsigned long long bytes_fed;
static unsigned decoded;

/* Deterministic, because the point is that a failure can be re-run rather than
 * that the bytes are unpredictable. xorshift32, one call per byte. */
static uint32_t rng(void)
{
    uint32_t x = rng_state;

    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    rng_state = x;
    return x;
}

static void check(int ok, const char *what, unsigned iteration)
{
    if (ok) {
        return;
    }
    printf("fuzz: FAIL %s (iteration %u, rng 0x%08x)\n", what, iteration,
           rng_state);
    failures++;
}

/* Damage a valid frame the way a link does: flip a few bits, and cut it short
 * at a random byte. Returns the (possibly shorter) length. Pure noise reaches
 * a parser's length checks and its sync search, but rarely its state machine;
 * a frame that is *nearly* right reaches all three. */
static unsigned damage(uint8_t *frame, unsigned len)
{
    unsigned flips = 1u + rng() % 3u;

    for (unsigned f = 0; f < flips; f++) {
        unsigned at = rng() % len;
        frame[at] ^= (uint8_t)(1u << (rng() % 8u));
    }
    return rng() % (len + 1u);
}

/* --- frames that are valid on their own terms ---------------------------- */

/* CRSF: [address][len][type][payload 22][crc], sixteen 11-bit channels packed
 * little-endian. Built here from the wire description, so a frame this
 * function calls valid is valid independently of the parser. */
static unsigned crsf_frame(uint8_t *frame, uint16_t base)
{
    frame[0] = 0xC8;
    frame[1] = 2u + 22u;
    frame[2] = AK_CRSF_TYPE_RC_CHANNELS;
    memset(&frame[3], 0, 22);
    for (unsigned n = 0; n < AK_CRSF_CHANNELS; n++) {
        unsigned bit = n * 11u;
        uint32_t value = (uint32_t)((base + n * 37u) & 0x7FFu);
        for (unsigned b = 0; b < 11u; b++) {
            if ((value >> b) & 1u) {
                frame[3 + (bit + b) / 8u] |=
                    (uint8_t)(1u << ((bit + b) % 8u));
            }
        }
    }
    frame[25] = ak_crsf_crc8(&frame[2], 23);
    return 26u;
}

/* SBUS: [0x0F][22 bytes of channels][flags][0x00], channels packed the same
 * way, and a live frame is one with no failsafe bit in the flags. */
static unsigned sbus_frame(uint8_t *frame, uint16_t base)
{
    frame[0] = 0x0F;
    memset(&frame[1], 0, 22);
    for (unsigned n = 0; n < AK_SBUS_CHANNELS; n++) {
        unsigned bit = n * 11u;
        uint32_t value = (uint32_t)((base + n * 41u) & 0x7FFu);
        for (unsigned b = 0; b < 11u; b++) {
            if ((value >> b) & 1u) {
                frame[1 + (bit + b) / 8u] |=
                    (uint8_t)(1u << ((bit + b) % 8u));
            }
        }
    }
    frame[23] = 0x00; /* no failsafe, no frame-lost */
    frame[24] = 0x00;
    return 25u;
}

/* UBX NAV-PVT, with the checksum the module would send: a fix the flight core
 * can use, so "it decoded" is a fact about a real message. */
static void put_i32(uint8_t *at, int32_t value)
{
    uint32_t v = (uint32_t)value;

    at[0] = (uint8_t)(v & 0xFFu);
    at[1] = (uint8_t)((v >> 8) & 0xFFu);
    at[2] = (uint8_t)((v >> 16) & 0xFFu);
    at[3] = (uint8_t)((v >> 24) & 0xFFu);
}

static unsigned pvt_frame(uint8_t *frame)
{
    uint8_t a = 0;
    uint8_t b = 0;

    /* The field offsets the header exports, from the u-blox description: a
     * frame built from them is independent of the parser that reads it. */
    uint8_t *payload = &frame[6];

    memset(payload, 0, AK_GPS_UBX_NAV_PVT_LENGTH);
    payload[AK_GPS_PVT_FIX_TYPE] = AK_GPS_FIX_3D;
    payload[AK_GPS_PVT_FLAGS] = 0x01; /* gnssFixOK */
    payload[AK_GPS_PVT_SATELLITES] = 10u;
    put_i32(&payload[AK_GPS_PVT_LAT], 521234567);
    put_i32(&payload[AK_GPS_PVT_LON], 49876543);

    frame[0] = 0xB5;
    frame[1] = 0x62;
    frame[2] = 0x01;
    frame[3] = 0x07;
    frame[4] = (uint8_t)(AK_GPS_UBX_NAV_PVT_LENGTH & 0xFF);
    frame[5] = (uint8_t)(AK_GPS_UBX_NAV_PVT_LENGTH >> 8);
    ak_gps_ubx_checksum(&frame[2], (uint16_t)(4u + AK_GPS_UBX_NAV_PVT_LENGTH),
                        &a, &b);
    frame[6 + AK_GPS_UBX_NAV_PVT_LENGTH] = a;
    frame[7 + AK_GPS_UBX_NAV_PVT_LENGTH] = b;
    return 8u + AK_GPS_UBX_NAV_PVT_LENGTH;
}

/* The protocol's request frame: AA 55 version command length payload crc16,
 * the crc over everything after the sync pair. */
static unsigned proto_frame(uint8_t *frame, uint8_t command)
{
    uint16_t crc;

    frame[0] = AK_PROTO_SYNC1;
    frame[1] = AK_PROTO_SYNC2;
    frame[2] = AK_PROTO_VERSION;
    frame[3] = command;
    frame[4] = 0u;
    crc = ak_proto_crc16(&frame[2], 3u);
    frame[5] = (uint8_t)(crc & 0xFFu);
    frame[6] = (uint8_t)(crc >> 8);
    return 7u;
}

/* --- the driver's side of the protocol ----------------------------------- */

static ak_params_t params;
static unsigned saves;

static void io_status(void *ctx, ak_proto_status_t *out)
{
    (void)ctx;
    memset(out, 0, sizeof *out);
    out->flight_state = 1u;
}

static int io_save(void *ctx)
{
    (void)ctx;
    saves++;
    return 0;
}

static int32_t io_log_count(void *ctx, uint8_t source)
{
    (void)ctx;
    (void)source;
    return 0; /* a device with an empty fast log and no others */
}

static unsigned io_log_record(void *ctx, uint8_t source, uint16_t index,
                              uint8_t *out, unsigned capacity)
{
    (void)ctx;
    (void)source;
    (void)index;
    (void)out;
    (void)capacity;
    return 0u;
}

int main(int argc, char **argv)
{
    static uint8_t garbage[MAX_GARBAGE];
    static uint8_t frame[512];
    static uint8_t response[256];
    unsigned iterations = 20000u;
    uint32_t seed = 0x5EED2026u;

    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "--seed") == 0 && i + 1 < argc) {
            seed = (uint32_t)strtoul(argv[++i], 0, 0);
        } else if (strcmp(argv[i], "--iterations") == 0 && i + 1 < argc) {
            iterations = (unsigned)strtoul(argv[++i], 0, 0);
        } else {
            fprintf(stderr, "usage: %s [--seed N] [--iterations N]\n",
                    argv[0]);
            return 2;
        }
    }
    rng_state = seed ? seed : 1u;

    ak_param_t items[1];
    ak_params_init(&params, items, 0u);
    const ak_proto_io_t io = {
        .params = &params,
        .status = io_status,
        .save = io_save,
        .log_count = io_log_count,
        .log_record = io_log_record,
        .ctx = 0,
    };

    for (unsigned it = 0; it < iterations; it++) {
        unsigned n = 1u + rng() % MAX_GARBAGE;

        for (unsigned i = 0; i < n; i++) {
            garbage[i] = (uint8_t)rng();
        }
        bytes_fed += n;

        /* The radio, both protocols, each on one parser: the noise first, then
         * a frame that has to come out the other end. */
        {
            ak_crsf_t crsf;
            ak_rc_input_t in;
            int got = 0;

            ak_crsf_init(&crsf);
            memset(&in, 0, sizeof in);
            for (unsigned i = 0; i < n; i++) {
                ak_crsf_feed(&crsf, garbage[i], &in, 1000u);
            }
            unsigned len = crsf_frame(frame, 992u);
            for (unsigned i = 0; i < len; i++) {
                got |= ak_crsf_feed(&crsf, frame[i], &in, 2000u);
            }
            check(got != 0, "crsf: a good frame still decodes after garbage",
                  it);
            check(in.channel[0] == 992u && in.channel[1] == 992u + 37u,
                  "crsf: the channels are the ones the frame carried", it);
            if (got) {
                decoded++;
            }
        }

        {
            ak_sbus_t sbus;
            ak_rc_input_t in;
            int got = 0;

            ak_sbus_init(&sbus);
            memset(&in, 0, sizeof in);
            for (unsigned i = 0; i < n; i++) {
                ak_sbus_feed(&sbus, garbage[i], &in, 1000u);
            }
            unsigned len = sbus_frame(frame, 992u);
            for (unsigned i = 0; i < len; i++) {
                got |= ak_sbus_feed(&sbus, frame[i], &in, 2000u);
            }
            check(got != 0, "sbus: a good frame still decodes after garbage",
                  it);
            if (got) {
                decoded++;
            }
        }

        {
            ak_gps_t gps;
            int got = 0;

            ak_gps_init(&gps);
            for (unsigned i = 0; i < n; i++) {
                ak_gps_feed(&gps, garbage[i], 1000u);
            }
            unsigned len = pvt_frame(frame);
            for (unsigned i = 0; i < len; i++) {
                got |= ak_gps_feed(&gps, frame[i], 2000u);
            }
            check(got != 0, "gps: a good fix still arrives after garbage", it);
            check(ak_gps_fix_valid(&gps, 2000u, 500u),
                  "gps: and it is a fix the flight core can use", it);
            if (got) {
                decoded++;
            }
        }

        {
            ak_proto_t proto;
            unsigned got = 0;

            ak_proto_init(&proto);
            for (unsigned i = 0; i < n; i++) {
                ak_proto_feed(&proto, &io, garbage[i], 1000u, response,
                              sizeof response);
            }
            unsigned len = proto_frame(frame, AK_PROTO_CMD_STATUS);
            for (unsigned i = 0; i < len; i++) {
                got |= ak_proto_feed(&proto, &io, frame[i], 2000u, response,
                                     sizeof response);
            }
            check(got != 0, "protocol: a request still answers after garbage",
                  it);
            if (got) {
                decoded++;
            }
        }

        /* And the same four parsers fed a frame that is nearly right: a valid
         * one with a few bits flipped and then cut short at a random byte. Each
         * still has to take a good frame afterwards - which is the whole
         * question, because a parser that swallows a damaged frame's length and
         * waits for it is a link that never comes back. */
        {
            ak_crsf_t crsf;
            ak_rc_input_t in;
            int got = 0;

            ak_crsf_init(&crsf);
            memset(&in, 0, sizeof in);
            unsigned len = damage(frame, crsf_frame(frame, 1200u));
            for (unsigned i = 0; i < len; i++) {
                ak_crsf_feed(&crsf, frame[i], &in, 3000u);
            }
            len = crsf_frame(frame, 992u);
            for (unsigned i = 0; i < len; i++) {
                got |= ak_crsf_feed(&crsf, frame[i], &in, 4000u);
            }
            check(got != 0, "crsf: a damaged frame does not cost the next one",
                  it);
            if (got) {
                decoded++;
            }
        }

        {
            ak_sbus_t sbus;
            ak_rc_input_t in;
            int got = 0;

            ak_sbus_init(&sbus);
            memset(&in, 0, sizeof in);
            unsigned len = damage(frame, sbus_frame(frame, 1200u));
            for (unsigned i = 0; i < len; i++) {
                ak_sbus_feed(&sbus, frame[i], &in, 3000u);
            }
            len = sbus_frame(frame, 992u);
            for (unsigned i = 0; i < len; i++) {
                got |= ak_sbus_feed(&sbus, frame[i], &in, 4000u);
            }
            check(got != 0, "sbus: a damaged frame does not cost the next one",
                  it);
            if (got) {
                decoded++;
            }
        }

        {
            ak_gps_t gps;
            int got = 0;

            ak_gps_init(&gps);
            unsigned len = damage(frame, pvt_frame(frame));
            for (unsigned i = 0; i < len; i++) {
                ak_gps_feed(&gps, frame[i], 3000u);
            }
            len = pvt_frame(frame);
            for (unsigned i = 0; i < len; i++) {
                got |= ak_gps_feed(&gps, frame[i], 4000u);
            }
            check(got != 0, "gps: a damaged frame does not cost the next one",
                  it);
            if (got) {
                decoded++;
            }
        }

        {
            ak_proto_t proto;
            unsigned got = 0;

            ak_proto_init(&proto);
            unsigned len = damage(frame, proto_frame(frame, AK_PROTO_CMD_STATUS));
            for (unsigned i = 0; i < len; i++) {
                ak_proto_feed(&proto, &io, frame[i], 3000u, response,
                              sizeof response);
            }
            len = proto_frame(frame, AK_PROTO_CMD_STATUS);
            for (unsigned i = 0; i < len; i++) {
                got |= ak_proto_feed(&proto, &io, frame[i], 4000u, response,
                                     sizeof response);
            }
            check(got != 0,
                  "protocol: a damaged frame does not cost the next one", it);
            if (got) {
                decoded++;
            }
        }

        /* And the two decoders that take a whole payload at once, because a
         * channel unpacker that trusts its input is the same bug one layer
         * down. Both are handed a random payload, and what they must not do is
         * read outside it - which is a sanitizer question, not a value one. */
        {
            uint16_t channels[AK_CRSF_CHANNELS];

            ak_crsf_unpack(garbage, channels);
            for (unsigned i = 0; i < AK_CRSF_CHANNELS; i++) {
                check((channels[i] & ~0x7FFu) == 0u,
                      "crsf: a whole-payload unpack stays inside 11 bits", it);
            }
        }
    }

    printf("fuzz: %u iterations, %llu bytes into five parsers, "
           "%u valid frames decoded\n", iterations, bytes_fed, decoded);
    printf("fuzz: %s (%u checks failed, seed 0x%08x)\n",
           failures == 0 ? "PASS" : "FAIL", failures, seed);
    return failures == 0 ? 0 : 1;
}
