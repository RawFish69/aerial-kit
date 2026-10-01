/*
 * The firmware's half of the cross-repository contract vectors.
 *
 * This is not a test. It is a *producer*: it runs the firmware's own encoders
 * - ak_attitude_ddeg(), ak_proto_telemetry_frame(), ak_log_encode_record() -
 * over inputs chosen to sit on the edges, and prints what they returned as
 * JSON. tools/contract_vectors.py adds the one group this program cannot
 * produce (the NED->ENU rotation, which belongs to the Python repository
 * because this firmware has no ENU anywhere in it) and writes
 * contract/aerialkit-contract-v1.json.
 *
 * The point of doing it this way rather than writing the expected bytes down
 * by hand is provenance. A vector that a person typed is a second opinion
 * about the format. A vector that this program printed is the format, and the
 * checker in tools/contract_check.py - which shares no code with either side -
 * is the second opinion.
 *
 * Everything here is deterministic: no clock, no device, no allocation. The
 * inputs are in the output next to the answers, so a reader can re-derive
 * every byte without running anything.
 *
 * See docs/31-contract.md.
 */
#include <stdio.h>
#include <stddef.h>
#include <string.h>
#include <stdint.h>

#include "ak_proto.h"
#include "ak_log.h"
#include "ak_math.h"

/* A float's exact bits, so that "the input was pi/2" is not a claim about how
 * a decimal literal was rounded. The checker reads the bits, not the decimal. */
static uint32_t bits(float value)
{
    uint32_t out;
    memcpy(&out, &value, sizeof out);
    return out;
}

static void print_hex(const uint8_t *bytes, unsigned length)
{
    for (unsigned i = 0; i < length; i++) {
        printf("%02x", bytes[i]);
    }
}

/* ------------------------------------------------------------------ */
/* Group 1: the attitudes, in each unit the firmware writes them in.    */
/* ------------------------------------------------------------------ */

typedef struct {
    const char *what;
    float radians;
} angle_case_t;

/* The set is chosen so that every way the conversion can surprise somebody is
 * in it: the four cardinal angles, the exact wrap point, a value a hair inside
 * it, an angle past one turn, a multi-turn angle, the wrap limit and past it,
 * a subnormal, and a NaN. */
static const angle_case_t ANGLES[] = {
    { "zero",              0.0f },
    { "negative zero",    -0.0f },
    { "45 degrees",       0.7853982f },
    { "quarter turn",     1.5707964f },
    { "minus quarter",   -1.5707964f },
    { "one radian",       1.0f },
    { "three radians",    3.0f },
    { "just inside pi",   3.1415925f },
    { "pi",               3.1415927f },
    { "minus pi",        -3.1415927f },
    { "just past pi",     3.1415930f },
    { "half turn plus",   3.15f },
    { "359.9 degrees",    6.281807f },
    { "one turn",         6.2831855f },
    { "five turns",       31.415928f },
    { "minus five turns", -31.415928f },
    { "wrap limit",       1.0e5f },
    { "past the limit",   1.0e5f * 1.5f },
    { "one eighth",       0.125f },
    { "tiny",             1.0e-30f },
};

/* Past AK_WRAP_LIMIT the firmware says an angle that large is not an attitude
 * and answers zero rather than answering about an angle nobody has. The
 * infinity and the NaN take the same path. Written as bits so that no
 * compiler's warning about a constant NaN changes the program. */
static const uint32_t SPECIAL_BITS[] = {
    0x7fc00000u, /* quiet NaN */
    0x7f800000u, /* +inf */
    0xff800000u, /* -inf */
};

static void emit_angles(void)
{
    printf("    \"attitude\": {\n      \"cases\": [\n");
    unsigned count = sizeof ANGLES / sizeof ANGLES[0];
    for (unsigned i = 0; i < count; i++) {
        float radians = ANGLES[i].radians;
        printf("        {\"what\": \"%s\", \"radians\": %.9g, "
               "\"bits\": \"0x%08x\", \"mrad\": %d, \"ddeg\": %d},\n",
               ANGLES[i].what, (double)radians, bits(radians),
               (int)(radians * 1000.0f), (int)ak_attitude_ddeg(radians));
    }
    unsigned specials = sizeof SPECIAL_BITS / sizeof SPECIAL_BITS[0];
    for (unsigned i = 0; i < specials; i++) {
        float radians;
        uint32_t raw = SPECIAL_BITS[i];
        memcpy(&radians, &raw, sizeof radians);
        /* No "radians" key: a NaN has no decimal spelling that survives a
         * round trip through a text file, and printing one would invite a
         * reader to believe the decimal. The bits are the input. */
        printf("        {\"what\": \"bits 0x%08x\", \"bits\": \"0x%08x\", "
               "\"mrad\": %d, \"ddeg\": %d}%s\n",
               raw, raw, (int)(radians * 1000.0f),
               (int)ak_attitude_ddeg(radians),
               i + 1 == specials ? "" : ",");
    }
    printf("      ]\n    },\n");
}

/* ------------------------------------------------------------------ */
/* Group 2: one telemetry frame, built by the protocol's own encoder.   */
/* ------------------------------------------------------------------ */

/* The status this frame carries. Chosen so that the signed fields are
 * exercised in both directions - a positive and a negative angle, a position
 * in the western and southern hemispheres - and so that the motor bytes cover
 * both endpoints of their range. */
#define STATUS_NOW_MS 1234567u

static void fill_status(ak_proto_status_t *status)
{
    memset(status, 0, sizeof *status);
    status->flight_state = 4u;      /* on autopilot, in ak_flight.h's numbering */
    status->link_live = 1u;
    status->gps_fix_type = 3u;
    status->gps_satellites = 11u;
    status->roll_ddeg = -450;       /* -45.0 degrees */
    status->pitch_ddeg = 1234;      /* 123.4 degrees */
    status->yaw_ddeg = -1799;       /* a hair inside the wrap */
    status->lat_e7 = -338675000;    /* Sydney, in the southern hemisphere */
    status->lon_e7 = 1512093000;
    status->motor[0] = 0u;          /* stopped */
    status->motor[1] = 254u;        /* the top of the range, not 255 */
    status->motor[2] = 255u;        /* 255 means "no output at all" */
    status->motor[3] = 128u;
}

static void status_fill_cb(void *ctx, ak_proto_status_t *out)
{
    *out = *(const ak_proto_status_t *)ctx;
}

static void emit_telemetry(void)
{
    ak_proto_status_t status;
    fill_status(&status);

    ak_proto_io_t io;
    memset(&io, 0, sizeof io);
    io.status = status_fill_cb;
    io.ctx = &status;

    uint8_t frame[AK_PROTO_FRAME_MAX];
    unsigned length = ak_proto_telemetry_frame(&io, STATUS_NOW_MS, frame,
                                               sizeof frame);

    printf("    \"telemetry\": {\n");
    printf("      \"now_ms\": %u,\n", STATUS_NOW_MS);
    printf("      \"status\": {\n");
    printf("        \"flight_state\": %u, \"link_live\": %u, "
           "\"gps_fix_type\": %u, \"gps_satellites\": %u,\n",
           status.flight_state, status.link_live, status.gps_fix_type,
           status.gps_satellites);
    printf("        \"roll_ddeg\": %d, \"pitch_ddeg\": %d, "
           "\"yaw_ddeg\": %d,\n",
           status.roll_ddeg, status.pitch_ddeg, status.yaw_ddeg);
    printf("        \"lat_e7\": %d, \"lon_e7\": %d,\n",
           status.lat_e7, status.lon_e7);
    printf("        \"motor\": [%u, %u, %u, %u]\n", status.motor[0],
           status.motor[1], status.motor[2], status.motor[3]);
    printf("      },\n");
    printf("      \"frame_length\": %u,\n", length);
    /* The payload is the 4-byte timestamp and then the status body. The two
     * are different numbers and confusing them is the mistake this line
     * exists to make impossible: a frame is 5 header + 26 payload + 2 crc. */
    printf("      \"payload_length\": %u,\n", length - 7u);
    printf("      \"status_body_length\": %u,\n", length - 7u - 4u);
    printf("      \"frame_hex\": \"");
    print_hex(frame, length);
    printf("\"\n    },\n");
}

/* ------------------------------------------------------------------ */
/* Group 3: one blackbox record, on the wire and in memory.             */
/* ------------------------------------------------------------------ */

static void fill_record(ak_log_record_t *record)
{
    memset(record, 0, sizeof *record);
    record->time_ms = 4000000000u;   /* past the 32-bit ms sign, on purpose */
    record->gyro[0] = -32768;        /* the bottom of the range */
    record->gyro[1] = 32767;         /* and the top */
    record->gyro[2] = -1;
    record->accel[0] = 1000;         /* 1.000 g, sitting still */
    record->accel[1] = -1000;
    record->accel[2] = -999;
    record->attitude[0] = -450;      /* -45.0 degrees */
    record->attitude[1] = 1799;
    record->yaw = -1800;             /* the far edge of the wrapped field */
    record->alt_mm = -12345;         /* below the take-off reference */
    record->stick[0] = -1000;
    record->stick[1] = 1000;
    record->stick[2] = 0;
    record->stick[3] = 1000;
    record->torque[0] = -100;
    record->torque[1] = 100;
    record->torque[2] = 0;
    record->motor[0] = 255u;         /* no output */
    record->motor[1] = 254u;
    record->motor[2] = 0u;
    record->motor[3] = 127u;
    record->state = 5u;              /* circling down */
    record->flags = AK_LOG_RC_LIVE | AK_LOG_IMU_VALID | AK_LOG_GPS_VALID;
    record->lat_e7 = -338675000;
    record->lon_e7 = 1512093000;
}

static void emit_log_record(void)
{
    ak_log_record_t record;
    fill_record(&record);

    uint8_t wire[AK_LOG_WIRE_BYTES];
    unsigned length = ak_log_encode_record(&record, wire, sizeof wire);

    printf("    \"log_record\": {\n");
    printf("      \"sizeof_record\": %zu,\n", sizeof(ak_log_record_t));
    printf("      \"wire_bytes\": %u,\n", AK_LOG_WIRE_BYTES);
    printf("      \"encoded_length\": %u,\n", length);
    printf("      \"offsets\": {");
    printf("\"time_ms\": %zu, \"gyro\": %zu, \"accel\": %zu, "
           "\"attitude\": %zu, \"yaw\": %zu, \"alt_mm\": %zu, ",
           offsetof(ak_log_record_t, time_ms), offsetof(ak_log_record_t, gyro),
           offsetof(ak_log_record_t, accel),
           offsetof(ak_log_record_t, attitude), offsetof(ak_log_record_t, yaw),
           offsetof(ak_log_record_t, alt_mm));
    printf("\"stick\": %zu, \"torque\": %zu, \"motor\": %zu, "
           "\"state\": %zu, \"flags\": %zu, \"lat_e7\": %zu, "
           "\"lon_e7\": %zu},\n",
           offsetof(ak_log_record_t, stick), offsetof(ak_log_record_t, torque),
           offsetof(ak_log_record_t, motor), offsetof(ak_log_record_t, state),
           offsetof(ak_log_record_t, flags), offsetof(ak_log_record_t, lat_e7),
           offsetof(ak_log_record_t, lon_e7));
    printf("      \"record\": {\n");
    printf("        \"time_ms\": %u,\n", record.time_ms);
    printf("        \"gyro\": [%d, %d, %d],\n", record.gyro[0], record.gyro[1],
           record.gyro[2]);
    printf("        \"accel\": [%d, %d, %d],\n", record.accel[0],
           record.accel[1], record.accel[2]);
    printf("        \"attitude\": [%d, %d],\n", record.attitude[0],
           record.attitude[1]);
    printf("        \"yaw\": %d,\n", record.yaw);
    printf("        \"alt_mm\": %d,\n", record.alt_mm);
    printf("        \"stick\": [%d, %d, %d, %d],\n", record.stick[0],
           record.stick[1], record.stick[2], record.stick[3]);
    printf("        \"torque\": [%d, %d, %d],\n", record.torque[0],
           record.torque[1], record.torque[2]);
    printf("        \"motor\": [%u, %u, %u, %u],\n", record.motor[0],
           record.motor[1], record.motor[2], record.motor[3]);
    printf("        \"state\": %u, \"flags\": %u,\n", record.state,
           record.flags);
    printf("        \"lat_e7\": %d, \"lon_e7\": %d\n", record.lat_e7,
           record.lon_e7);
    printf("      },\n");
    printf("      \"wire_hex\": \"");
    print_hex(wire, length);
    printf("\"\n    }\n");
}

/* Valid JSON on its own - no trailing comma on the last group - because
 * tools/contract_vectors.py parses this rather than concatenating text onto
 * it. A generator that emitted a fragment would make every reader of the
 * final file depend on the concatenation staying right. */
int main(void)
{
    printf("{\n");
    printf("  \"generator\": \"tools/contract_vectors.c\",\n");
    emit_angles();
    emit_telemetry();
    emit_log_record();
    printf("}\n");
    return 0;
}
