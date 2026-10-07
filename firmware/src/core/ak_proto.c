#include "ak_proto.h"

#include "ak_log.h"
#include "ak_text.h"
#include "ak_version.h"

enum {
    STATE_SYNC1 = 0,
    STATE_SYNC2,
    STATE_BODY,
    STATE_CRC_LOW,
    STATE_CRC_HIGH,
    /*
     * A frame was cut short - its length byte says more than a frame can hold -
     * so the bytes that follow are the rest of a frame whose end this parser
     * does not know. They are read looking for the next sync pair, which is
     * byte for byte what STATE_SYNC1 does with them, and the difference between
     * the two states is not in this file at all: it is what the caller does.
     *
     * A state that parses identically to another one looks redundant, so it is
     * worth saying why it is not. The console link's caller hands a byte to the
     * human console when the parser is between frames and the byte is not a
     * sync - so "between frames" has to mean "the wire is not in the middle of
     * a binary frame", and after a bad length it still is. Without this state
     * the tail of the abandoned frame was typed at the console: its printable
     * bytes went into the line buffer, and any 0x0D or 0x0A among them printed
     * a prompt nobody asked for. A stray prompt is not cosmetic - it is what
     * tools/bench_check.py reads to mean "the last command has finished".
     */
    STATE_DRAIN,
};

/* CRC-16/CCITT-FALSE: polynomial 0x1021, initial value 0xFFFF, no reflection,
 * no final xor. Chosen because it catches what a serial line does - a dropped
 * byte changes the length the sender wrote, and the crc covers that length -
 * and because it is four lines to write and one to check. */
uint16_t ak_proto_crc16(const uint8_t *data, unsigned length)
{
    uint16_t crc = 0xFFFFu;

    for (unsigned i = 0; i < length; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (int bit = 0; bit < 8; bit++) {
            crc = (crc & 0x8000u) != 0 ? (uint16_t)((crc << 1) ^ 0x1021u)
                                       : (uint16_t)(crc << 1);
        }
    }
    return crc;
}

void ak_proto_init(ak_proto_t *proto)
{
    /* Zero the whole thing rather than listing the fields, and then set the
     * ones whose default is not zero.
     *
     * The list this replaces had thirteen entries and the struct has sixteen
     * fields: `can_stream` was added to the struct and never added here, so it
     * kept whatever the memory held. On the flight firmware that was invisible
     * - `static ak_proto_t proto;` lands in BSS, which the C runtime has
     * already zeroed - but `tools/akproto_sim.c` declares its parser on the
     * stack, and there the same omission meant a subscribe could answer with a
     * rate the simulator had no way to send. A test that asks the simulator to
     * refuse a stream therefore passed or failed depending on what the stack
     * happened to contain, which is the worst kind of failure: not a bug in the
     * code under test, and not reproducible from the code either.
     *
     * Anything added to `ak_proto_t` from here on is initialised by
     * construction. Callers that need a different value set it *after* this
     * call, which is what `main.c` already does for the network link's
     * `can_stream`. */
    *proto = (ak_proto_t){0};
    proto->state = STATE_SYNC1;
    proto->log_source = AK_PROTO_LOG_FAST;
}

static unsigned append_u8(uint8_t *response, unsigned at, uint8_t value,
                          unsigned capacity)
{
    if (at < capacity) {
        response[at++] = value;
    }
    return at;
}

static unsigned append_u16le(uint8_t *response, unsigned at, uint16_t value,
                             unsigned capacity)
{
    at = append_u8(response, at, (uint8_t)(value & 0xFFu), capacity);
    return append_u8(response, at, (uint8_t)(value >> 8), capacity);
}

static unsigned append_i16le(uint8_t *response, unsigned at, int16_t value,
                             unsigned capacity)
{
    return append_u16le(response, at, (uint16_t)value, capacity);
}

static unsigned append_u32le(uint8_t *response, unsigned at, uint32_t raw,
                             unsigned capacity)
{
    for (int i = 0; i < 4; i++) {
        at = append_u8(response, at, (uint8_t)((raw >> (8 * i)) & 0xFFu), capacity);
    }
    return at;
}

static unsigned append_i32le(uint8_t *response, unsigned at, int32_t value,
                             unsigned capacity)
{
    return append_u32le(response, at, (uint32_t)value, capacity);
}

/* The whole string and its terminator, or nothing at all.
 *
 * "Or nothing" is the part that matters, and it took the real table to show why.
 * The version this replaces wrote characters while `at + 1 < capacity` and then
 * wrote the terminator under the same condition, so a string landing *exactly*
 * at the end of the room - the natural "it just fits" case - had all of its
 * characters written and no terminator after them. The reply was still a legal
 * frame with a correct length and a correct CRC; what it contained was a string
 * that ran into whatever the next field was. Every caller that had pre-checked
 * its fit believed the string was written whole, so nothing upstream could see
 * it.
 *
 * It was unreachable while every reply stopped well short of capacity. The first
 * builder to fill a frame - PARAM_INFO, whose pages are sized to end where the
 * room ends - reached it on the board's own `arm_accel_lpf_hz`, and the entry
 * after it read a fourth bound out of its default's missing terminator.
 *
 * Writing a *prefix* instead would be the other way to be wrong: half a name is
 * not a shorter name, it is the next field's bytes relabelled. A caller that
 * wants to know whether its string fitted can measure it the same way this
 * does; a caller that does not cares only that the payload ends where the
 * length says it does. */
static unsigned append_string(uint8_t *response, unsigned at, const char *text,
                              unsigned capacity)
{
    unsigned length = 0;
    while (text[length] != '\0') {
        length++;
    }
    if (at + length + 1u > capacity) {
        return at;
    }
    for (unsigned i = 0; i < length; i++) {
        response[at++] = (uint8_t)text[i];
    }
    response[at++] = 0;
    return at;
}

static unsigned start_response(uint8_t *response, uint8_t command,
                               unsigned capacity)
{
    if (capacity < 8u) {
        return 0;
    }
    response[0] = AK_PROTO_SYNC1;
    response[1] = AK_PROTO_SYNC2;
    response[2] = AK_PROTO_VERSION;
    response[3] = (uint8_t)(command | AK_PROTO_RESPONSE_BIT);
    response[4] = 0; /* length, filled in by finish_response */
    return 5;
}

static unsigned finish_response(uint8_t *response, unsigned at)
{
    response[4] = (uint8_t)(at - 5u);
    uint16_t crc = ak_proto_crc16(&response[2], at - 2u);
    response[at++] = (uint8_t)(crc & 0xFFu);
    response[at++] = (uint8_t)(crc >> 8);
    return at;
}

/* What the aircraft is doing, in fixed units a console with no floating point
 * can print. Shared by the STATUS reply and the telemetry stream, because they
 * are the same answer: one is asked for and the other is pushed. */
static unsigned append_status_body(uint8_t *response, unsigned at,
                                   const ak_proto_io_t *io, unsigned capacity)
{
    ak_proto_status_t status;
    for (unsigned i = 0; i < sizeof status; i++) {
        ((uint8_t *)&status)[i] = 0;
    }
    if (io->status != 0) {
        io->status(io->ctx, &status);
    }
    at = append_u8(response, at, status.flight_state, capacity);
    at = append_u8(response, at, status.link_live, capacity);
    at = append_u8(response, at, status.gps_fix_type, capacity);
    at = append_u8(response, at, status.gps_satellites, capacity);
    at = append_i16le(response, at, status.roll_ddeg, capacity);
    at = append_i16le(response, at, status.pitch_ddeg, capacity);
    at = append_i16le(response, at, status.yaw_ddeg, capacity);
    at = append_i32le(response, at, status.lat_e7, capacity);
    at = append_i32le(response, at, status.lon_e7, capacity);
    for (unsigned i = 0; i < 4; i++) {
        at = append_u8(response, at, status.motor[i], capacity);
    }
    return at;
}

unsigned ak_proto_telemetry_frame(const ak_proto_io_t *io, uint32_t now_ms,
                                  uint8_t *out, unsigned capacity)
{
    if (capacity < 8u) {
        return 0;
    }
    out[0] = AK_PROTO_SYNC1;
    out[1] = AK_PROTO_SYNC2;
    out[2] = AK_PROTO_VERSION;
    out[3] = AK_PROTO_CMD_TELEMETRY; /* no response bit: this was not asked for */
    out[4] = 0;                      /* length, filled in below */

    unsigned at = append_i32le(out, 5u, (int32_t)now_ms, capacity);
    at = append_status_body(out, at, io, capacity);

    /* A buffer too small for the body and its CRC is no frame at all: it used
     * to come back with the body cut and no checksum, and a length byte that
     * did not match what was written. */
    if (at + 2u > capacity) {
        return 0;
    }
    out[4] = (uint8_t)(at - 5u);
    uint16_t crc = ak_proto_crc16(&out[2], at - 2u);
    at = append_u8(out, at, (uint8_t)(crc & 0xFFu), capacity);
    at = append_u8(out, at, (uint8_t)(crc >> 8), capacity);
    return at;
}

/* One frame of a log stream, built to be pushed: the command byte has no
 * response bit, exactly as the telemetry frame's does not, so a client tells a
 * stream from its own answers the same way on both.
 *
 * The body carries the index and the source in every frame, which the LOG_GET
 * reply does not. A reply does not have to say what it is answering - it is
 * arriving because something asked - and a push has nothing above it tying it
 * to a request. Those four bytes are what let a client see a hole *as* a hole
 * rather than as a record that did not arrive.
 *
 * `log_stream_index` advances here and not in the caller, so the two failure
 * modes that matter cannot happen: a caller that advanced on its own would skip
 * a record whose frame was never built, and a caller that advanced only on
 * success would resend the record the board refused, forever, because a refusal
 * is not a transient error. */
unsigned ak_proto_log_stream_frame(ak_proto_t *proto, const ak_proto_io_t *io,
                                   uint8_t *out, unsigned capacity)
{
    /* A frame holds a header, four bytes of index and source, a whole record
     * and a CRC. A caller with less room than that gets nothing rather than a
     * record with its tail cut off: the `append_*` helpers write what fits and
     * report where they stopped, so a record built into too small a buffer
     * would come out short and still carry a valid CRC over its short self. A
     * client would decode it as a complete record with the next field's bytes
     * where its last ones should be. */
    if (capacity < 5u + 4u + AK_LOG_WIRE_BYTES + 2u) {
        return 0u;
    }
    if (proto->log_stream_hz == 0u) {
        return 0u;
    }

    out[0] = AK_PROTO_SYNC1;
    out[1] = AK_PROTO_SYNC2;
    out[2] = AK_PROTO_VERSION;
    out[3] = AK_PROTO_CMD_LOG_STREAM; /* no response bit: this was not asked for */
    out[4] = 0;                       /* length, filled in below */

    uint8_t  source = proto->log_stream_source;
    uint16_t index = proto->log_stream_index;
    unsigned at;

    if (index >= proto->log_stream_end) {
        /* The range is finished, and it says so rather than going quiet. */
        at = append_u8(out, 5u, AK_PROTO_LOG_STREAM_DONE, capacity);
        at = append_u8(out, at, source, capacity);
        at = append_u16le(out, at, index, capacity);
        proto->log_stream_hz = 0u;
    } else {
        uint8_t record[AK_LOG_WIRE_BYTES];
        unsigned length = io->log_record != 0
                              ? io->log_record(io->ctx, source, index, record,
                                               sizeof record)
                              : 0u;
        if (length == 0u) {
            /* Refused, and named: the client is told which index it was, so a
             * gap in the log is a fact it has rather than one it infers. */
            at = append_u8(out, 5u, AK_PROTO_LOG_STREAM_HOLE, capacity);
            at = append_u8(out, at, source, capacity);
            at = append_u16le(out, at, index, capacity);
        } else {
            at = append_u8(out, 5u, AK_PROTO_LOG_STREAM_RECORD, capacity);
            at = append_u8(out, at, source, capacity);
            at = append_u16le(out, at, index, capacity);
            for (unsigned i = 0; i < length && i < AK_LOG_WIRE_BYTES; i++) {
                at = append_u8(out, at, record[i], capacity);
            }
        }
        proto->log_stream_index = (uint16_t)(index + 1u);
    }

    out[4] = (uint8_t)(at - 5u);
    uint16_t crc = ak_proto_crc16(&out[2], at - 2u);
    at = append_u8(out, at, (uint8_t)(crc & 0xFFu), capacity);
    at = append_u8(out, at, (uint8_t)(crc >> 8), capacity);
    return at;
}

/* Whether this board will take a configuration write at this moment.
 *
 * Null `writable` reads as yes, and that is deliberate: a device with no
 * aircraft to arm is not permanently armed, and inventing a refusal for it
 * would make every bench tool and every simulator refuse a `set` for a reason
 * that does not exist on that device. What a client is entitled to know is
 * whether the guard is *there* - which is what AK_PROTO_FEATURE_GATES_ON_ARMED
 * says, and the two are set together in main.c.
 *
 * Asked per request, never cached: the answer changes when the aircraft arms,
 * and a link that stays up across that is the normal case. */
static int write_allowed(const ak_proto_io_t *io)
{
    return io->writable == 0 || io->writable(io->ctx) != 0;
}

/* The payload of a request is a command byte followed by its arguments, so a
 * reply carries the command it answers. */
/*
 * A driver's name, always AK_PROTO_SENSOR_NAME bytes.
 *
 * Not append_string, and the difference is the point: that one writes the name
 * and its terminator and stops, so a reply carrying it would be as long as the
 * name happened to be - and a body whose length depends on which part answered
 * is a body a client has to measure before it can find the numbers after it.
 * Here the name is a field of fixed width, NUL-padded, and a body's length is a
 * number this file can state.
 *
 * A name too long for the field is cut rather than refused, because the
 * alternative at run time is a reply that cannot be built at all. What keeps
 * that from happening quietly is a host test: tests/test_proto.c walks every
 * driver table this tree has and fails if one would not fit - the same shape as
 * the board-name checks, and for the same reason, which is that the alternative
 * is finding out on a bench.
 */
static unsigned append_name(uint8_t *response, unsigned at, const char *name,
                            unsigned capacity)
{
    unsigned i = 0;
    while (i < AK_PROTO_SENSOR_NAME && name[i] != '\0') {
        at = append_u8(response, at, (uint8_t)name[i], capacity);
        i++;
    }
    while (i < AK_PROTO_SENSOR_NAME) {
        at = append_u8(response, at, 0, capacity);
        i++;
    }
    return at;
}

/* The body of a SENSOR_INFO reply, by topic.
 *
 * Fixed per topic, so a client can check a reply's length against the topic it
 * asked for and know the frame is not a shorter one misread. Every array here
 * is three long - an axis triple - and the two that are not are the counters,
 * which are the console's own: a sensor that reports a value and no history is
 * a sensor nobody can decide about. */
static unsigned append_sensor_body(uint8_t *response, unsigned at,
                                   const ak_proto_sensor_t *sensor,
                                   unsigned capacity)
{
    switch (sensor->topic) {
    case AK_PROTO_SENSOR_IMU: {
        const ak_proto_imu_t *imu = &sensor->as.imu;
        at = append_name(response, at, imu->driver, capacity);
        at = append_u8(response, at, imu->absent_reason, capacity);
        at = append_u8(response, at, imu->whoami, capacity);
        for (unsigned i = 0; i < 3; i++) {
            at = append_i16le(response, at, imu->accel[i], capacity);
        }
        for (unsigned i = 0; i < 3; i++) {
            at = append_i16le(response, at, imu->gyro[i], capacity);
        }
        for (unsigned i = 0; i < 3; i++) {
            at = append_i16le(response, at, imu->align[i], capacity);
        }
        for (unsigned i = 0; i < 3; i++) {
            at = append_i16le(response, at, imu->gyro_bias[i], capacity);
        }
        at = append_u32le(response, at, imu->samples, capacity);
        at = append_u32le(response, at, imu->errors, capacity);
        break;
    }
    case AK_PROTO_SENSOR_BARO: {
        const ak_proto_baro_t *baro = &sensor->as.baro;
        at = append_name(response, at, baro->driver, capacity);
        at = append_i32le(response, at, baro->pressure_pa, capacity);
        at = append_i16le(response, at, baro->temperature_cdeg, capacity);
        at = append_u8(response, at, baro->have_reference, capacity);
        at = append_i32le(response, at, baro->reference_pa, capacity);
        at = append_i32le(response, at, baro->height_cm, capacity);
        at = append_u8(response, at, baro->have_gps_reference, capacity);
        at = append_i32le(response, at, baro->fused_cm, capacity);
        at = append_u32le(response, at, baro->samples, capacity);
        at = append_u32le(response, at, baro->errors, capacity);
        at = append_u32le(response, at, baro->fails, capacity);
        at = append_u32le(response, at, baro->baro_samples, capacity);
        at = append_u32le(response, at, baro->gps_samples, capacity);
        break;
    }
    case AK_PROTO_SENSOR_RANGE: {
        const ak_proto_range_t *range = &sensor->as.range;
        at = append_name(response, at, range->driver, capacity);
        at = append_u8(response, at, range->address, capacity);
        at = append_u16le(response, at, range->max_mm, capacity);
        at = append_i32le(response, at, range->distance_mm, capacity);
        at = append_u32le(response, at, range->age_ms, capacity);
        at = append_u32le(response, at, range->samples, capacity);
        at = append_u32le(response, at, range->out_of_range, capacity);
        at = append_u32le(response, at, range->rejected, capacity);
        at = append_u32le(response, at, range->faults, capacity);
        at = append_u32le(response, at, range->fails, capacity);
        at = append_u32le(response, at, range->land_mm, capacity);
        at = append_u16le(response, at, range->agree_cm, capacity);
        break;
    }
    case AK_PROTO_SENSOR_BATTERY: {
        const ak_proto_battery_t *battery = &sensor->as.battery;
        at = append_u8(response, at, battery->ready, capacity);
        at = append_u8(response, at, battery->have_reading, capacity);
        at = append_u8(response, at, battery->state, capacity);
        at = append_u8(response, at, battery->cells, capacity);
        at = append_u16le(response, at, battery->volts_cv, capacity);
        at = append_u16le(response, at, battery->per_cell_cv, capacity);
        at = append_i16le(response, at, battery->pin_mv, capacity);
        at = append_u16le(response, at, battery->ratio_milli, capacity);
        at = append_u8(response, at, battery->rth, capacity);
        at = append_u16le(response, at, battery->warn_cell_mv, capacity);
        at = append_u16le(response, at, battery->critical_cell_mv, capacity);
        at = append_u32le(response, at, battery->samples, capacity);
        at = append_u32le(response, at, battery->rejected, capacity);
        at = append_u32le(response, at, battery->returns, capacity);
        break;
    }
    case AK_PROTO_SENSOR_GPS: {
        const ak_proto_gps_t *gps = &sensor->as.gps;
        at = append_u8(response, at, gps->have_fix, capacity);
        at = append_u8(response, at, gps->fix_type, capacity);
        at = append_u8(response, at, gps->fix_ok, capacity);
        at = append_u8(response, at, gps->satellites, capacity);
        at = append_u8(response, at, gps->valid_now, capacity);
        at = append_i32le(response, at, gps->lat_e7, capacity);
        at = append_i32le(response, at, gps->lon_e7, capacity);
        at = append_i32le(response, at, gps->alt_msl_mm, capacity);
        at = append_i32le(response, at, gps->speed_mm_s, capacity);
        at = append_i32le(response, at, gps->course_e5, capacity);
        at = append_u8(response, at, gps->have_home, capacity);
        at = append_i32le(response, at, gps->home_lat_e7, capacity);
        at = append_i32le(response, at, gps->home_lon_e7, capacity);
        at = append_i32le(response, at, gps->home_distance_m, capacity);
        at = append_i32le(response, at, gps->home_bearing_cdeg, capacity);
        at = append_u8(response, at, gps->returning, capacity);
        at = append_u8(response, at, gps->rth_enabled, capacity);
        at = append_u32le(response, at, gps->fixes, capacity);
        at = append_u32le(response, at, gps->dropped, capacity);
        at = append_u32le(response, at, gps->config_sends, capacity);
        break;
    }
    default:
        /* Unreachable from the wire: the dispatch refuses a topic outside the
         * range this header defines before it gets here. Written out anyway,
         * because the alternative to saying "no body" is writing nothing and
         * letting the caller believe a body went out. */
        break;
    }
    return at;
}

static unsigned handle_command(ak_proto_t *proto, const ak_proto_io_t *io,
                               const uint8_t *buffer, unsigned capacity,
                               uint8_t *response)
{
    /* frame body: version, command, length, payload... */
    uint8_t command = buffer[1];
    const uint8_t *args = buffer + 3;
    unsigned args_length = buffer[2];

    /*
     * Two bytes of every frame are the CRC, and finish_response writes them
     * after the builders have finished. So the last two bytes of the caller's
     * array do not belong to a builder, whatever it is handed - and handing
     * them over means a response built into bytes that are then written over,
     * which is a frame two bytes longer than its own length field says.
     *
     * Reserving it here rather than at each call site, because a call site that
     * forgets is two bytes past the end of `uint8_t response[AK_PROTO_FRAME_MAX]`
     * - which is what both links in main.c and both in the simulator declare.
     * It was latent while every reply was short enough to stop well before the
     * buffer ran out; PARAM_INFO is the first builder that can fill it, so it
     * is fixed here, before it, and the canary in tests/test_proto.c is what
     * stops it being latent again.
     *
     * The second clamp is a different promise, and it is the one a *client*
     * depends on: no reply's payload may be longer than AK_PROTO_MAX_PAYLOAD,
     * because that is the size every client's receive buffer is and the size
     * the frame format is defined around. A caller with a generous array does
     * not thereby make a 97-byte payload legal. Without this the reserved bytes
     * alone would let a builder reach 97, and a client that sized its buffer
     * from the constant would drop the frame as malformed - which is the worst
     * shape of failure available here, since the board would have sent it and
     * counted it as sent.
     *
     * A caller that passes a smaller array than a frame needs gets a shorter
     * reply, not a longer one: `capacity` is an upper bound throughout, and
     * PARAM_INFO is where that becomes visible (a page that cannot hold even
     * one entry says so rather than cutting one in half).
     */
    if (capacity < 10u) {
        return 0;
    }
    capacity -= 2u;
    if (capacity > 5u + AK_PROTO_MAX_PAYLOAD) {
        capacity = 5u + AK_PROTO_MAX_PAYLOAD;
    }

    unsigned at = start_response(response, command, capacity);
    /* Big enough for the longest text parameter: this buffer is what a param
     * get answers with, and a Wi-Fi name that arrived truncated would be a
     * configuration the tool quietly gets wrong. */
    char text[AK_PARAM_VALUE_MAX];

    if (at == 0) {
        return 0;
    }

    switch (command) {
    case AK_PROTO_CMD_HELLO: {
        char changed[12];
        at = append_u8(response, at, AK_PROTO_VERSION, capacity);
        at = append_string(response, at, AK_PRODUCT_STR, capacity);
        at = append_u16le(response, at, (uint16_t)io->params->count, capacity);
        ak_format_uint(io->params->changed, 0, changed, sizeof changed);
        at = append_string(response, at, changed, capacity);
        /* Appended, never interleaved with the four fields above. That is what
         * lets this be added without moving AK_PROTO_VERSION: a client written
         * against the old HELLO reads its four fields and stops, and a board
         * written before this change answers with a payload that ends here -
         * which the client reports as a capability word it does not have,
         * rather than as a board with no capabilities.
         *
         * The hash is of the parameter table and is what a saved configuration
         * can be held against: two boards reporting the same hash have the
         * same parameters in the same order with the same names, so a file
         * from one is meaningful on the other, and two that differ are not.
         * It is computed here rather than carried in `io` because the protocol
         * already owns the table it is a hash of. */
        at = append_u32le(response, at, io->features, capacity);
        at = append_u32le(response, at, ak_params_hash(io->params), capacity);
        break;
    }

    case AK_PROTO_CMD_PARAM_GET: {
        if (args_length < 1u || args[0] >= io->params->count) {
            at = append_u8(response, at, 1, capacity);
            break;
        }
        const ak_param_t *item = &io->params->items[args[0]];
        ak_params_get_text(item, text, sizeof text);
        at = append_u8(response, at, 0, capacity);
        at = append_string(response, at, item->name, capacity);
        at = append_string(response, at, text, capacity);
        break;
    }

    case AK_PROTO_CMD_PARAM_SET: {
        char message[48];

        /* The policy first, and before the index is even looked at: whether
         * this board will take a write at all does not depend on which
         * parameter was named, and a client told "no such parameter" while the
         * aircraft is armed would reasonably conclude that naming a real one
         * would have worked. It would not have.
         *
         * The reason travels in the message field beside the status, which is
         * the same field the table's own words use, so a client that reads only
         * the status byte is unaffected and one that reads the text can say
         * why. The console has carried text here since the table was written;
         * this is the first refusal that comes from the protocol rather than
         * from the table. */
        if (!write_allowed(io)) {
            at = append_u8(response, at, AK_PROTO_WRITE_REFUSED_ARMED, capacity);
            at = append_string(response, at,
                               "refused: the aircraft is armed", capacity);
            break;
        }
        if (args_length < 2u || args[0] >= io->params->count) {
            at = append_u8(response, at, AK_PROTO_WRITE_NO_SUCH, capacity);
            at = append_string(response, at, "no such parameter", capacity);
            break;
        }
        const ak_param_t *item = &io->params->items[args[0]];

        /* The value is the rest of the payload, as text. The parameter table
         * parses and range-checks it, so the protocol never has to know what a
         * parameter means - which is the whole reason there is one table. */
        unsigned value_length = args_length - 1u;
        if (value_length >= sizeof text) {
            /* Refused, not cut: a 64-character WPA key truncated to 63 passed
             * the parameter's own length check and answered OK, storing a key
             * that is not the one that was sent. The console path refuses the
             * same value; so does this now. */
            at = append_u8(response, at, AK_PROTO_WRITE_REJECTED, capacity);
            at = append_string(response, at, "value too long", capacity);
            break;
        }
        for (unsigned i = 0; i < value_length; i++) {
            text[i] = (char)args[1 + i];
        }
        text[value_length] = '\0';

        message[0] = '\0';
        int result = ak_params_set(io->params, item->name, text, message,
                                   sizeof message);
        at = append_u8(response, at, result == 0 ? AK_PROTO_WRITE_OK
                                                 : AK_PROTO_WRITE_REJECTED,
                       capacity);
        /* And the aircraft is told, because a parameter that only reached the
         * table has not reached anything. This is the console's `set` making
         * the same call at the same point, and it is the whole of what was
         * missing: the table moved over the wire and the hardware did not, so
         * a client could read back a rate the timer was not running.
         *
         * On success only. A refused value changed nothing, and calling this
         * for it would re-apply a configuration in response to a request that
         * did nothing - which is harmless here and is not a thing to leave
         * lying around for the next parameter to mean something by. */
        if (result == 0 && io->on_change != 0) {
            io->on_change();
        }
        /* And the table's own words about what it just did, which is the half a
         * configurator needs: "out of 0.000..1.000" is the difference between a
         * refused value and a puzzle. The console has printed this message
         * since the table was written; the wire did not carry it, so the one
         * client with a text field to fill in was the one that could not say
         * why. It is appended rather than replacing the status byte, so a
         * client that only reads the status is unaffected - and the check that
         * says so is in tests/test_proto.c. */
        at = append_string(response, at, message, capacity);
        break;
    }

    case AK_PROTO_CMD_PARAM_SAVE: {
        int status = AK_PROTO_WRITE_OK;
        if (!write_allowed(io)) {
            /* Gated here as well as in `save_parameters()`, which passes the
             * same predicate down to the storage. Both are wanted: this one
             * means the refusal is reported as a policy with a status a client
             * can render, and that one means the guard does not depend on the
             * write having come through this file. A board whose storage
             * enforced it alone would still be safe and could not say why. */
            status = AK_PROTO_WRITE_REFUSED_ARMED;
        } else if (io->save == 0) {
            status = AK_PROTO_WRITE_NO_STORAGE; /* nowhere to save */
        } else if (io->save(io->ctx) != 0) {
            status = AK_PROTO_WRITE_STORAGE_ERROR; /* the board refused the write */
        } else {
            ak_params_mark_saved(io->params);
        }
        at = append_u8(response, at, (uint8_t)status, capacity);
        break;
    }

    case AK_PROTO_CMD_PARAM_DEFAULT: {
        /* `mode` says what to reset, and a request that names nothing is
         * refused rather than read as "everything". See the header: an empty
         * frame that means a factory reset is the one thing this opcode must
         * not do, because a frame truncated in transit is indistinguishable
         * from a deliberate one. */
        if (!write_allowed(io)) {
            at = append_u8(response, at, AK_PROTO_WRITE_REFUSED_ARMED, capacity);
            at = append_string(response, at,
                               "refused: the aircraft is armed", capacity);
            break;
        }
        if (args_length < 1u) {
            at = append_u8(response, at, AK_PROTO_WRITE_REJECTED, capacity);
            at = append_string(response, at,
                               "name what to reset: 1 <index>, or 2 for all",
                               capacity);
            break;
        }

        uint8_t mode = args[0];
        if (mode == 1u) {
            if (args_length < 2u || args[1] >= io->params->count) {
                at = append_u8(response, at, AK_PROTO_WRITE_NO_SUCH, capacity);
                at = append_string(response, at, "no such parameter", capacity);
                break;
            }
            at = append_u8(response, at, AK_PROTO_WRITE_OK, capacity);
            /* Only the named row, and by index: the table's own default is the
             * one it was registered with, and a client that named an index is
             * naming the row it saw - the name is what the row is *called*, the
             * index is what it is. */
            ak_params_default_one(io->params, args[1]);
        } else if (mode == 2u) {
            at = append_u8(response, at, AK_PROTO_WRITE_OK, capacity);
            ak_params_reset(io->params);
        } else {
            at = append_u8(response, at, AK_PROTO_WRITE_REJECTED, capacity);
            at = append_string(response, at, "mode is 1 (one) or 2 (all)",
                               capacity);
            break;
        }

        /* A defaulted table is a changed configuration, exactly as a `set` is,
         * so the aircraft is told the same way. Without this the board would
         * hold the old airframe in its mixer while the table said otherwise -
         * the failure the `on_change` callback exists to prevent, one route
         * further along. */
        if (io->on_change != 0) {
            io->on_change();
        }
        break;
    }

    case AK_PROTO_CMD_STATUS: {
        at = append_status_body(response, at, io, capacity);
        break;
    }

    case AK_PROTO_CMD_TELEMETRY: {
        /* A rate, and the answer is the rate that will actually be sent: this
         * firmware has an opinion about what is worth streaming, and a client
         * that asked for more should know what it is going to get rather than
         * work it out from the spacing.
         *
         * Including the answer "none": a console link cannot push frames at a
         * client - a stream there would arrive among a person's keystrokes -
         * so it says 0 rather than agreeing to a rate that will never arrive,
         * which is what a client would otherwise wait for. */
        uint32_t wanted = args_length >= 1u ? args[0] : 0u;
        if (!proto->can_stream) {
            wanted = 0u;
        }
        if (wanted > AK_PROTO_TELEMETRY_MAX_HZ) {
            wanted = AK_PROTO_TELEMETRY_MAX_HZ;
        }
        proto->telemetry_hz = (uint8_t)wanted;
        at = append_u8(response, at, (uint8_t)wanted, capacity);
        break;
    }

    case AK_PROTO_CMD_LOG_INFO: {
        int32_t count = io->log_count != 0
                            ? io->log_count(io->ctx, proto->log_source)
                            : -1;

        at = append_u16le(response, at,
                          count > 0 ? (uint16_t)count : 0u, capacity);
        break;
    }

    case AK_PROTO_CMD_LOG_GET: {
        if (args_length < 2u || io->log_record == 0) {
            at = append_u8(response, at, 1, capacity);
            break;
        }
        uint16_t index = (uint16_t)(args[0] | ((uint16_t)args[1] << 8));
        /* Big enough for the record the layout defines, and no bigger: a buffer
         * written down as a number is a buffer that stops being big enough the
         * next time the record grows, and it fails as "no record" rather than
         * as anything a person would recognise. It was 48. */
        uint8_t record[AK_LOG_WIRE_BYTES];
        unsigned length = io->log_record(io->ctx, proto->log_source, index,
                                         record, sizeof record);
        if (length == 0) {
            at = append_u8(response, at, 1, capacity);
            break;
        }
        at = append_u8(response, at, 0, capacity);
        for (unsigned i = 0; i < length && at < capacity; i++) {
            at = append_u8(response, at, record[i], capacity);
        }
        break;
    }

    case AK_PROTO_CMD_LOG_SOURCE: {
        /* Ask for a source and be told what it is: status, which source is
         * selected now, and how many records it holds. A device that has two
         * of the three logs says so for the third rather than pretending the
         * request worked and answering zero records forever. */
        uint8_t wanted = args_length >= 1u ? args[0] : AK_PROTO_LOG_FAST;
        int32_t count = io->log_count != 0 ? io->log_count(io->ctx, wanted) : -1;

        if (wanted > AK_PROTO_LOG_MAX || count < 0) {
            at = append_u8(response, at, 1, capacity);
            at = append_u8(response, at, proto->log_source, capacity);
            at = append_u16le(response, at, 0u, capacity);
            break;
        }
        proto->log_source = wanted;
        at = append_u8(response, at, 0, capacity);
        at = append_u8(response, at, wanted, capacity);
        at = append_u16le(response, at, (uint16_t)count, capacity);
        break;
    }

    case AK_PROTO_CMD_LOG_STREAM: {
        /* A range and a rate, and the answer is the range and the rate that
         * will actually be sent. The same rule as TELEMETRY: a client that
         * asked for more than this firmware will give should be told what it
         * is going to get rather than work it out from the spacing.
         *
         * The reply is six numbers, and every one of them is a fact about what
         * happens next rather than an echo of what was asked:
         *
         *   status  0 accepted, 1 refused (no such log on this board)
         *   source  the log that will be streamed
         *   first   the first index that will be sent
         *   count   how many records will be sent, after clamping
         *   rate    the frames per second that will be sent, after clamping
         *
         * A reply of `0, source, first, 0, 0` is the answer "nothing will be
         * sent", and it is the same answer whether the range named no records,
         * the rate was zero, or the link cannot push frames at all. That is
         * deliberate and it is TELEMETRY's answer too: the client's job is to
         * notice that no stream is coming, not to be told which of three
         * sentences to print.
         *
         * `proto->log_source` is *not* touched. It selects the ring LOG_GET
         * reads from, and a person reading one record at a time should not
         * have their selection moved by a stream they started and stopped. */
        if (args_length < 6u) {
            /* A short frame is refused rather than defaulted. The defaults here
             * would all be harmful: a request that named no range would become
             * "the whole log", and a request that named no rate would become
             * one this firmware chose. `PARAM_DEFAULT` refuses its bare form for
             * the same reason and it is the same booby trap. */
            at = append_u8(response, at, 1, capacity);
            at = append_u8(response, at, 0, capacity);
            at = append_u16le(response, at, 0u, capacity);
            at = append_u16le(response, at, 0u, capacity);
            at = append_u8(response, at, 0, capacity);
            break;
        }

        uint8_t  wanted_source = args[0];
        uint16_t first = (uint16_t)(args[1] | ((uint16_t)args[2] << 8));
        uint16_t count = (uint16_t)(args[3] | ((uint16_t)args[4] << 8));
        uint32_t rate = args[5];

        int32_t total = (wanted_source <= AK_PROTO_LOG_MAX && io->log_count != 0)
                            ? io->log_count(io->ctx, wanted_source)
                            : -1;
        if (total < 0) {
            /* No such log. Not "empty" - a device with two of the three rings
             * says which one it cannot read, the way LOG_SOURCE does. */
            at = append_u8(response, at, 1, capacity);
            at = append_u8(response, at, wanted_source, capacity);
            at = append_u16le(response, at, 0u, capacity);
            at = append_u16le(response, at, 0u, capacity);
            at = append_u8(response, at, 0, capacity);
            break;
        }

        if (first > (uint16_t)total) {
            first = (uint16_t)total;
        }
        uint16_t available = (uint16_t)((uint16_t)total - first);
        if (count > available) {
            count = available;
        }
        if (!proto->can_stream || count == 0u || rate == 0u) {
            /* Nothing will be sent, so stop anything already running and say
             * zero rather than agreeing to a stream that will never arrive. */
            proto->log_stream_hz = 0u;
            at = append_u8(response, at, 0, capacity);
            at = append_u8(response, at, wanted_source, capacity);
            at = append_u16le(response, at, 0u, capacity);
            at = append_u16le(response, at, 0u, capacity);
            at = append_u8(response, at, 0, capacity);
            break;
        }
        if (rate > AK_PROTO_LOG_STREAM_MAX_HZ) {
            rate = AK_PROTO_LOG_STREAM_MAX_HZ;
        }

        proto->log_stream_source = wanted_source;
        proto->log_stream_index = first;
        /* `end` is exclusive, and it is the answer to "when do I stop" that the
         * frame builder reads. Crafting it as start+count with a 16-bit sum
         * would wrap at 65536 - and this board's flash ring holds 6 552, so a
         * wrap is not reachable today, which is exactly the kind of reasoning
         * that stops being true the first time a board overflows the uint16
         * index. Clamping to `total` cannot wrap. */
        proto->log_stream_end =
            (uint16_t)(first + count <= (uint16_t)total ? first + count
                                                        : (uint16_t)total);
        proto->log_stream_hz = (uint8_t)rate;

        at = append_u8(response, at, 0, capacity);
        at = append_u8(response, at, wanted_source, capacity);
        at = append_u16le(response, at, first, capacity);
        at = append_u16le(response, at, count, capacity);
        at = append_u8(response, at, (uint8_t)rate, capacity);
        break;
    }

    case AK_PROTO_CMD_PARAM_INFO: {
        /* A page of the table's own description of itself. The layout is in
         * docs/16-protocol.md; what matters here is that the page is built by
         * measuring an entry before writing it, so a page that runs out of room
         * stops *between* entries and reports how many it wrote. Appending and
         * hoping would cut an entry in half, and half an entry reads as the next
         * entry's bytes - a wrong name against a right value, which is the one
         * mistake this table has made before. */
        unsigned count = io->params->count;

        if (args_length < 1u) {
            /* A page has to say from where. Answering a request that named no
             * index would be answering a question nobody asked, and the answer
             * would look exactly like a complete table. */
            at = append_u8(response, at, 1, capacity);
            break;
        }

        unsigned first = args[0];
        unsigned status = 0;
        unsigned status_at = at;

        at = append_u8(response, at, 0, capacity); /* status, filled in below */
        at = append_u8(response, at, (uint8_t)first, capacity);
        unsigned carried_at = at;
        at = append_u8(response, at, 0, capacity); /* carried, filled in below */
        unsigned carried = 0;

        for (unsigned index = first; index < count; index++) {
            const ak_param_t *item = &io->params->items[index];
            char low[16], high[16], def[AK_PARAM_VALUE_MAX];
            unsigned size = ak_strlen(item->name) + 1u + 4u;

            low[0] = '\0';
            high[0] = '\0';
            if (item->type == AK_PARAM_TEXT) {
                ak_params_get_default_text(item, def, sizeof def);
                size += 1u + ak_strlen(def) + 1u;
            } else {
                (void)ak_params_format_value(item, item->min, low, sizeof low);
                (void)ak_params_format_value(item, item->max, high, sizeof high);
                ak_params_get_default_text(item, def, sizeof def);
                size += ak_strlen(low) + 1u + ak_strlen(high) + 1u +
                        ak_strlen(def) + 1u;
            }

            if (at + size > capacity) {
                /* A page that ran out of room is fine; the next one starts where
                 * this stopped. A *single* entry that cannot fit a frame is not,
                 * and it has to be said out loud: a client that reads
                 * `carried == 0` as "the end" would stop early and never learn
                 * the entry exists, and one that re-asks from the same index
                 * would ask forever. There is no third answer available - the
                 * entry cannot be split - so this is the one that says so. */
                if (carried == 0u) {
                    status = 2;
                }
                break;
            }

            at = append_string(response, at, item->name, capacity);
            at = append_u8(response, at, item->type, capacity);
            at = append_u8(response, at, item->group, capacity);
            at = append_u8(response, at, item->decimals, capacity);
            at = append_u8(response, at, item->flags, capacity);
            if (item->type == AK_PARAM_TEXT) {
                /* A text parameter has no numeric bounds; what bounds it is how
                 * many characters it may hold, and that is what goes in the byte
                 * a numeric parameter would begin its minimum in. The client
                 * reads `type` first, so the two shapes are told apart by the
                 * entry itself rather than by a version. */
                at = append_u8(response, at, item->max_len, capacity);
            } else {
                at = append_string(response, at, low, capacity);
                at = append_string(response, at, high, capacity);
            }
            at = append_string(response, at, def, capacity);
            carried++;
        }

        response[carried_at] = (uint8_t)carried;
        response[status_at] = (uint8_t)status;
        break;
    }

    case AK_PROTO_CMD_PARAM_HELP: {
        if (args_length < 3u || args[0] >= io->params->count) {
            at = append_u8(response, at, 1, capacity);
            break;
        }
        unsigned index = args[0];
        const char *help = io->params->items[index].help;
        unsigned total = (help != 0) ? ak_strlen(help) : 0u;
        unsigned offset = (unsigned)(args[1] | ((unsigned)args[2] << 8));

        /* Walked by offset rather than paged, which is what makes "the text was
         * longer than one frame" not a case: `offset` past the end is an empty
         * tail, and a client knows it has the whole thing when
         * offset + len == total. */
        if (offset > total) {
            offset = total;
        }

        /* Seven bytes of header - status, index, offset, total, length - and
         * then the text, so the room left for text is what the header does not
         * use. Counted here rather than computed from a constant so that adding
         * a field to the header is a change in one place. */
        unsigned room = (capacity > at + 7u) ? capacity - (at + 7u) : 0u;
        unsigned part = total - offset;
        if (part > room) {
            part = room;
        }
        if (part > 0xFFu) {
            part = 0xFFu;
        }

        at = append_u8(response, at, 0, capacity);
        at = append_u8(response, at, (uint8_t)index, capacity);
        at = append_u16le(response, at, (uint16_t)offset, capacity);
        at = append_u16le(response, at, (uint16_t)total, capacity);
        at = append_u8(response, at, (uint8_t)part, capacity);
        for (unsigned i = 0; i < part; i++) {
            at = append_u8(response, at, (uint8_t)help[offset + i], capacity);
        }
        break;
    }

    case AK_PROTO_CMD_RC_CHANNELS: {
        if (io->rc_state == 0) {
            /* One byte and no more. Everything after it - flags, channels,
             * counters - would be a claim about a receiver this board does not
             * have, and zeros there would read on a screen as a receiver with a
             * dead link rather than as no receiver port at all. */
            at = append_u8(response, at, AK_PROTO_RC_NONE, capacity);
            break;
        }

        ak_proto_rc_t rc;
        for (unsigned i = 0; i < sizeof rc; i++) {
            ((uint8_t *)&rc)[i] = 0;
        }
        io->rc_state(io->ctx, &rc);

        /* Clamped to what will actually be written, not to what the struct can
         * hold: a `count` larger than the entries that follow would have the
         * client reading the sticks as channels. The caller is the firmware's
         * own, but the frame's self-consistency is this function's to keep, and
         * a clamp here is one comparison against a parser that trusts an
         * impossible count on a link that carries a person's aircraft. */
        uint8_t count = rc.count > AK_PROTO_RC_MAX ? (uint8_t)AK_PROTO_RC_MAX : rc.count;

        at = append_u8(response, at, AK_PROTO_RC_OK, capacity);
        at = append_u8(response, at, rc.flags, capacity);
        at = append_u8(response, at, rc.protocol, capacity);
        at = append_u8(response, at, count, capacity);
        for (unsigned i = 0; i < count; i++) {
            at = append_u16le(response, at, rc.raw[i], capacity);
        }
        for (unsigned i = 0; i < 4; i++) {
            at = append_i16le(response, at, rc.sticks[i], capacity);
        }
        at = append_u8(response, at, rc.switches, capacity);
        at = append_u32le(response, at, rc.bytes, capacity);
        at = append_u32le(response, at, rc.frames, capacity);
        at = append_u32le(response, at, rc.crc_errors, capacity);
        at = append_u32le(response, at, rc.rejected, capacity);
        at = append_u32le(response, at, rc.lost, capacity);
        at = append_u32le(response, at, rc.failsafe_frames, capacity);
        at = append_u32le(response, at, rc.dropped, capacity);
        break;
    }

    case AK_PROTO_CMD_SENSOR_INFO: {
        /* The topic is echoed on every path, including the refusals, so a
         * client polling several can tell which answer it is holding without
         * keeping a queue in step with a link that may drop a frame. A short
         * frame is a request that named nothing, and naming nothing is not a
         * default - it is refused the same way a topic this build does not have
         * is, because "which sensor did you mean" has no answer that is safe to
         * guess. */
        uint8_t topic = args_length > 0u ? args[0] : (uint8_t)AK_PROTO_SENSOR_TOPICS;

        if (io->sensor_state == 0 || topic >= AK_PROTO_SENSOR_TOPICS) {
            at = append_u8(response, at, AK_PROTO_SENSOR_NO_SUCH, capacity);
            at = append_u8(response, at, topic, capacity);
            at = append_u8(response, at, 0, capacity);
            break;
        }

        ak_proto_sensor_t sensor;
        for (unsigned i = 0; i < sizeof sensor; i++) {
            ((uint8_t *)&sensor)[i] = 0;
        }
        sensor.topic = topic;
        io->sensor_state(io->ctx, topic, &sensor);

        at = append_u8(response, at, AK_PROTO_SENSOR_OK, capacity);
        at = append_u8(response, at, topic, capacity);
        at = append_u8(response, at, sensor.present ? 1u : 0u, capacity);
        /* The body only when there is something to say. This is the whole
         * reason the opcode has a `present` byte rather than a convention about
         * zeroes: a body of zeros is a reading, and a board with no barometer
         * has no reading to report - only the absence of one. The length of
         * this reply is the answer, and it does not depend on the caller
         * having zeroed its struct honestly. */
        if (sensor.present) {
            at = append_sensor_body(response, at, &sensor, capacity);
        }
        break;
    }

    case AK_PROTO_CMD_OUTPUT_INFO: {
        /* A read with no argument, so there is nothing to get wrong and no
         * refusal for a malformed request: the empty frame is the only frame
         * this opcode defines, and a frame with bytes on the end is a client
         * from the future whose extra arguments this build ignores - which is
         * the same statement as "an old board reads the page it has always
         * read", from the other side.
         *
         * The header carries the count *and* the split between the two kinds,
         * because they answer different questions: the count is how many
         * entries follow, and the split is what the aircraft is. A client
         * drawing "4 motors, 2 servos" should not have to count descriptors to
         * say it, and a client that did would be wrong the first time a board
         * had an output kind it did not know. */
        if (io->outputs == 0) {
            at = append_u8(response, at, AK_PROTO_OUTPUT_INFO_NONE, capacity);
            at = append_u8(response, at, 0, capacity);
            at = append_u8(response, at, 0, capacity);
            at = append_u8(response, at, 0, capacity);
            at = append_u8(response, at, AK_PROTO_OUTPUT_TEST_MAX_PCT,
                           capacity);
            break;
        }

        ak_proto_output_t outputs[AK_PROTO_OUTPUT_MAX];
        unsigned total = io->outputs(io->ctx, outputs, AK_PROTO_OUTPUT_MAX);

        if (total > AK_PROTO_OUTPUT_MAX) {
            /* Refused, not truncated. A page shown as the whole aircraft is a
             * client drawing a wing with one elevon, and it has no way to
             * notice: the list is well-formed and short. */
            at = append_u8(response, at, AK_PROTO_OUTPUT_INFO_TOO_MANY,
                           capacity);
            at = append_u8(response, at, 0, capacity);
            at = append_u8(response, at, 0, capacity);
            at = append_u8(response, at, 0, capacity);
            at = append_u8(response, at, AK_PROTO_OUTPUT_TEST_MAX_PCT,
                           capacity);
            break;
        }

        if (total == 0u) {
            /* A callback that answers with nothing. The board exists and has a
             * way to enumerate - and it says there is nothing there, which on a
             * real board means it is not ready yet. Answered as NONE rather
             * than as OK-with-an-empty-list for the reason the status byte
             * gives: `count = 0, motors = 0, servos = 0` is a well-formed
             * sentence saying "this aircraft has no outputs", and a client
             * drawing it draws a flight controller with no motors and no
             * servos, which is not a thing. NONE says the true thing instead:
             * there is no list. */
            at = append_u8(response, at, AK_PROTO_OUTPUT_INFO_NONE, capacity);
            at = append_u8(response, at, 0, capacity);
            at = append_u8(response, at, 0, capacity);
            at = append_u8(response, at, 0, capacity);
            at = append_u8(response, at, AK_PROTO_OUTPUT_TEST_MAX_PCT,
                           capacity);
            break;
        }

        unsigned motors = 0;
        unsigned servos = 0;
        for (unsigned i = 0; i < total; i++) {
            if (outputs[i].kind == AK_PROTO_OUTPUT_SERVO) {
                servos++;
            } else {
                motors++;
            }
        }

        at = append_u8(response, at, AK_PROTO_OUTPUT_INFO_OK, capacity);
        at = append_u8(response, at, (uint8_t)total, capacity);
        at = append_u8(response, at, (uint8_t)motors, capacity);
        at = append_u8(response, at, (uint8_t)servos, capacity);
        at = append_u8(response, at, AK_PROTO_OUTPUT_TEST_MAX_PCT, capacity);

        for (unsigned i = 0; i < total; i++) {
            at = append_u8(response, at, outputs[i].kind, capacity);
            at = append_u8(response, at, outputs[i].index, capacity);
            at = append_u8(response, at, outputs[i].reversed ? 1u : 0u,
                           capacity);
            at = append_i16le(response, at, outputs[i].trim_us, capacity);
            at = append_u16le(response, at, outputs[i].travel_us, capacity);
        }
        break;
    }

    case AK_PROTO_CMD_OUTPUT_TEST: {
        /* A short frame is refused rather than defaulted, for PARAM_DEFAULT's
         * reason and LOG_STREAM's - and here the harmful default is worse than
         * either of those. An op that named nothing would become "whichever
         * verb", and a level that named nothing would become a number this
         * firmware chose. On the one command in this protocol that spins a
         * motor, both are worse than an answer saying nothing happened.
         *
         * Every field is echoed on every path, `0xFF` where the request did not
         * carry one - out of range for both `kind` and `index`, so a client can
         * tell "you did not say" from "I did not do". */
        if (args_length < 4u) {
            at = append_u8(response, at, AK_PROTO_OUTPUT_TEST_NO_OP, capacity);
            at = append_u8(response, at,
                           args_length > 0u ? args[0] : (uint8_t)0xFF, capacity);
            at = append_u8(response, at,
                           args_length > 1u ? args[1] : (uint8_t)0xFF, capacity);
            at = append_u8(response, at,
                           args_length > 2u ? args[2] : (uint8_t)0xFF, capacity);
            at = append_u8(response, at, 0, capacity);
            at = append_u16le(response, at, 0u, capacity);
            break;
        }

        uint8_t op = args[0];
        uint8_t kind = args[1];
        uint8_t index = args[2];
        uint8_t level = args[3];

        int status;
        ak_proto_output_test_t driven;
        driven.level_pct = 0;
        driven.remaining_ms = 0u;

        if (op != AK_PROTO_OUTPUT_TEST_HOLD && op != AK_PROTO_OUTPUT_TEST_STOP) {
            status = AK_PROTO_OUTPUT_TEST_NO_OP;
        } else if (io->output_test == 0) {
            status = AK_PROTO_OUTPUT_TEST_NO_BOARD;
        } else {
            /* The gate, the range check and the timeout are the board's, and
             * all three are checked inside the callback at the moment of this
             * command. The dispatch has no opinion about whether the aircraft
             * is disarmed - `io->writable` is where that lives, and the board
             * already asks it. A second opinion here would be a second policy,
             * and the two would disagree the first time either changed. */
            status = io->output_test(io->ctx, op, kind, index, level, &driven);
        }

        at = append_u8(response, at, (uint8_t)status, capacity);
        at = append_u8(response, at, op, capacity);
        at = append_u8(response, at, kind, capacity);
        at = append_u8(response, at, index, capacity);
        at = append_u8(response, at, driven.level_pct, capacity);
        at = append_u16le(response, at, driven.remaining_ms, capacity);
        break;
    }

    case AK_PROTO_CMD_PREFLIGHT: {
        /* One line, or one part of one line, of the board's checklist.
         *
         * The shape is PARAM_HELP's with an index in front of it, and for the
         * same reason: a sentence that does not fit a frame is walked by offset
         * rather than cut. `detail_total` is what tells a client it has the
         * whole thing, and it is the *sentence's* length and not the part's -
         * a client that compared the part against the frame's capacity would
         * stop one byte short of the end and show a fault without its cause.
         *
         * A short request is refused rather than defaulted. An `index` that
         * named nothing would become line zero, and a client that asked a
         * malformed question would be shown the first check as though it had
         * asked for it. */
        if (args_length < 3u) {
            /* One shape for NO_INDEX wherever it comes from: the status, then
             * the index and the total. `0xFF` for both, because neither is
             * known - and out of range for both, so a client cannot read "you
             * did not say" as "the board has 255 lines". */
            at = append_u8(response, at, AK_PROTO_PREFLIGHT_NO_INDEX, capacity);
            at = append_u8(response, at, (uint8_t)0xFF, capacity);
            at = append_u8(response, at, (uint8_t)0xFF, capacity);
            break;
        }

        unsigned index = args[0];
        unsigned offset = (unsigned)(args[1] | ((unsigned)args[2] << 8));
        const ak_preflight_line_t *lines = 0;
        unsigned count = 0;

        /* The rebuild happens here, on the first request of a walk and only
         * that one, so the pages of one walk describe one moment. */
        if (io->preflight == 0 || !io->preflight(io->ctx, index, &lines, &count) ||
            count == 0u) {
            at = append_u8(response, at, AK_PROTO_PREFLIGHT_NONE, capacity);
            break;
        }
        if (index >= count) {
            at = append_u8(response, at, AK_PROTO_PREFLIGHT_NO_INDEX, capacity);
            at = append_u8(response, at, (uint8_t)index, capacity);
            at = append_u8(response, at, (uint8_t)count, capacity);
            break;
        }

        const char *name = lines[index].name != 0 ? lines[index].name : "";
        const char *detail = lines[index].detail != 0 ? lines[index].detail : "";
        unsigned total = ak_strlen(detail);
        if (offset > total) {
            offset = total;
        }

        /* Eight bytes of header, then the name, then as much of the sentence as
         * is left. Counted rather than written down as a constant so that
         * adding a field to the header is a change in one place. */
        unsigned header = 8u;
        unsigned name_len = ak_strlen(name);
        if (name_len > AK_PROTO_PREFLIGHT_NAME_MAX) {
            name_len = AK_PROTO_PREFLIGHT_NAME_MAX;
        }
        unsigned room = (capacity > at + header + name_len)
                            ? capacity - (at + header + name_len)
                            : 0u;
        unsigned part = total - offset;
        if (part > room) {
            part = room;
        }
        if (part > 0xFFu) {
            part = 0xFFu;
        }

        at = append_u8(response, at, AK_PROTO_PREFLIGHT_OK, capacity);
        at = append_u8(response, at, (uint8_t)index, capacity);
        at = append_u8(response, at, (uint8_t)count, capacity);
        at = append_u8(response, at, lines[index].verdict, capacity);
        at = append_u8(response, at, (uint8_t)name_len, capacity);
        for (unsigned i = 0; i < name_len; i++) {
            at = append_u8(response, at, (uint8_t)name[i], capacity);
        }
        at = append_u16le(response, at, (uint16_t)total, capacity);
        at = append_u8(response, at, (uint8_t)part, capacity);
        for (unsigned i = 0; i < part; i++) {
            at = append_u8(response, at, (uint8_t)detail[offset + i], capacity);
        }
        break;
    }

    case AK_PROTO_CMD_CALIBRATE: {
        /* One verb, and the session's whole state back whichever it was.
         *
         * The state is zeroed before the callback runs and again on every
         * refusal, so a refusal is a complete reply rather than a header with a
         * caller's stack behind it - MISSION's shape, for MISSION's reason.
         *
         * A frame that does not name a complete verb is refused rather than
         * defaulted, and here the harmful default is worse than MISSION's: verb
         * zero is STATUS, which is a read and would be survivable, but the
         * *missing argument* is the one that bites. VBAT's argument is the
         * voltage a person read off a multimeter, and a truncated frame
         * defaulted to zero would ask the board to calibrate a pack divider
         * against nothing at all and write the result to `vbat_ratio`, which is
         * the parameter that decides when the aircraft comes home. `0xFF` is
         * echoed as the verb, out of range for all six, so a client can tell
         * "you did not say" from "I do not know that one". */
        ak_proto_calibration_t state;
        state.active = 0;
        state.verb = AK_PROTO_CALIBRATE_STATUS;
        state.step = AK_PROTO_CALIBRATE_NO_STEP;
        state.faces = 0;
        state.samples = 0;
        state.rejected = 0;
        for (unsigned i = 0; i < AK_PROTO_CALIBRATE_RESULT; i++) {
            state.result[i] = 0;
        }

        uint8_t verb = args_length > 0u ? args[0] : (uint8_t)0xFF;
        uint8_t face = args_length > 1u ? args[1] : (uint8_t)0;
        uint32_t mv = 0;
        int status;

        if (args_length < 1u) {
            status = AK_PROTO_CALIBRATE_NO_VERB;
            verb = (uint8_t)0xFF;
        } else if (verb > AK_PROTO_CALIBRATE_ABORT) {
            status = AK_PROTO_CALIBRATE_NO_VERB;
        } else if (verb == AK_PROTO_CALIBRATE_VBAT && args_length < 5u) {
            /* The verb is known and its argument is not, which is still "you
             * did not say" - so it is answered the same way, with the verb left
             * out of range, rather than letting a zero reach a callback that
             * would take it for a measurement. */
            status = AK_PROTO_CALIBRATE_NO_VERB;
            verb = (uint8_t)0xFF;
        } else if (io->calibrate == 0) {
            /* Null is "this board has nothing to calibrate" - the same split
             * OUTPUT_INFO's null callback makes. The state stays zeroed, and a
             * client that got NOTHING should say the board cannot do this
             * rather than draw a session that never started. The verb byte says
             * NO_SESSION rather than echoing what was asked for: there is no
             * session on this board and never will be, and a client that read
             * its own request back would be looking at the one byte it is
             * supposed to be learning something from. The refusals above keep
             * their echo, which is a different fact - those name the input the
             * dispatch could not use. */
            status = AK_PROTO_CALIBRATE_NOTHING;
            verb = AK_PROTO_CALIBRATE_NO_SESSION;
        } else {
            /* The gate the write routes have, for the reason the header gives:
             * every one of these four verbs ends in a parameter write, and a
             * calibration run while armed is a hand near a live throttle. Asked
             * here so the refusal is a policy a client can render, and asked
             * again inside the callback so the guard does not depend on this
             * file - PARAM_SAVE's two gates, one route along. */
            if (verb != AK_PROTO_CALIBRATE_STATUS &&
                verb != AK_PROTO_CALIBRATE_ABORT && !write_allowed(io)) {
                status = AK_PROTO_CALIBRATE_ARMED;
            } else {
                if (verb == AK_PROTO_CALIBRATE_VBAT) {
                    mv = (uint32_t)args[1] | ((uint32_t)args[2] << 8) |
                         ((uint32_t)args[3] << 16) |
                         ((uint32_t)args[4] << 24);
                }
                /* The refusals that are the aircraft's - busy, no such face, not
                 * enough still samples, an implausible gravity - are the
                 * callback's. The dispatch has no opinion about any of them. */
                status = io->calibrate(io->ctx, verb, face, mv, &state);

                /* And the reply's verb byte is the *session's*, not the verb
                 * that was asked for. This is the one place the two differ, and
                 * taking the request's here is what made `status` useless: the
                 * verb beside the six result slots is the thing that says how to
                 * read them, so a client polling a gyro calibration would have
                 * been handed three bias numbers labelled `status`, and asked
                 * for `abort` would have been told a session it could not name
                 * had ended. A refusal is still owed the state of the session
                 * running - that is the same reading, one verb along. */
                verb = state.verb;
            }
        }

        at = append_u8(response, at, (uint8_t)status, capacity);
        at = append_u8(response, at, verb, capacity);
        at = append_u8(response, at, state.active, capacity);
        at = append_u8(response, at, state.step, capacity);
        at = append_u8(response, at, state.faces, capacity);
        at = append_u32le(response, at, state.samples, capacity);
        at = append_u32le(response, at, state.rejected, capacity);
        for (unsigned i = 0; i < AK_PROTO_CALIBRATE_RESULT; i++) {
            at = append_i32le(response, at, state.result[i], capacity);
        }
        break;
    }

    case AK_PROTO_CMD_MISSION: {
        /* One verb, and the mission's whole state back whichever it was.
         *
         * A short frame is refused rather than defaulted, and here the harmful
         * default is the one that *does* something: an op that named nothing
         * would otherwise become verb zero, and verb zero is the only one of
         * the five that is a read. A client that sent a truncated frame would
         * be answered as though it had asked a question, which is survivable -
         * but the same default on the next number along would start a mission,
         * and there is no reason for the two paths to be shaped differently.
         * `0xFF` is echoed as the op, out of range for all five, so a client
         * can tell "you did not say" from "I do not know that one".
         *
         * The state is zeroed before the callback runs and again on every
         * refusal, so a refusal is a complete reply rather than a header with a
         * caller's stack behind it. */
        ak_proto_mission_t state;
        state.active = 0;
        state.requested = 0;
        state.count = 0;
        state.index = AK_PROTO_MISSION_NO_INDEX;
        state.channel = 0;
        state.reached = 0;
        state.started = 0;
        state.cancelled = 0;
        state.hold_alt_mm = 0;

        uint8_t op = args_length > 0u ? args[0] : (uint8_t)0xFF;
        int status;

        if (args_length < 1u) {
            status = AK_PROTO_MISSION_NO_VERB;
            op = (uint8_t)0xFF;
        } else if (op > AK_PROTO_MISSION_HOME_CLEAR) {
            status = AK_PROTO_MISSION_NO_VERB;
        } else if (io->mission == 0) {
            /* Null is "this board has no navigator", not "this list is empty" -
             * the same split OUTPUT_INFO's null callback makes. The state stays
             * zeroed, and `count` of zero here is a client's to tell apart from
             * the same zero with a board that answered OK */
            status = AK_PROTO_MISSION_NO_NAV;
        } else {
            /* The refusals are the board's: an empty list, a home with no
             * usable fix, and whatever else only the aircraft knows. The
             * dispatch has no opinion about any of them. */
            status = io->mission(io->ctx, op, &state);
        }

        at = append_u8(response, at, (uint8_t)status, capacity);
        at = append_u8(response, at, op, capacity);
        at = append_u8(response, at, state.active, capacity);
        at = append_u8(response, at, state.requested, capacity);
        at = append_u8(response, at, state.count, capacity);
        at = append_u8(response, at, state.index, capacity);
        at = append_u8(response, at, state.channel, capacity);
        at = append_u16le(response, at, state.reached, capacity);
        at = append_u16le(response, at, state.started, capacity);
        at = append_u16le(response, at, state.cancelled, capacity);
        at = append_i32le(response, at, state.hold_alt_mm, capacity);
        break;
    }

    case AK_PROTO_CMD_PERF: {
        /* The profiler's window, which is the one reply here that asks nothing
         * of the aircraft: there is no verb, no argument and nothing to refuse.
         *
         * A null callback and a callback that says "nothing to measure" are the
         * same answer on the wire - AK_PROTO_PERF_NONE - and deliberately so.
         * The difference between them is a build-time fact, and a client can
         * tell it from the capability word in HELLO; sending both would be two
         * ways to say one thing.
         *
         * `window` is zeroed first so that a callback which returns zero without
         * touching anything still leads to a complete reply rather than a header
         * with a caller's stack behind it - the same rule MISSION and CALIBRATE
         * follow. */
        ak_proto_perf_t window;
        unsigned i;
        int status;

        window.loops = 0u;
        window.samples = 0u;
        window.nominal_us = 0u;
        window.period_last_us = 0u;
        window.period_min_us = 0u;
        window.period_max_us = 0u;
        window.late = 0u;
        window.jitter_p50_us = 0u;
        window.jitter_p99_us = 0u;
        window.jitter_max_us = 0u;
        window.jitter_over = 0u;
        window.load_permille = 0u;
        for (i = 0u; i < AK_PROTO_PERF_SECTIONS; i++) {
            window.section_avg_us_x10[i] = 0u;
            window.section_max_us[i] = 0u;
        }

        if (io->perf == 0 || !io->perf(io->ctx, &window)) {
            status = AK_PROTO_PERF_NONE;
        } else {
            status = AK_PROTO_PERF_OK;
        }

        at = append_u8(response, at, (uint8_t)status, capacity);
        at = append_u32le(response, at, window.loops, capacity);
        at = append_u32le(response, at, window.samples, capacity);
        at = append_u16le(response, at, window.nominal_us, capacity);
        at = append_u32le(response, at, window.period_last_us, capacity);
        at = append_u32le(response, at, window.period_min_us, capacity);
        at = append_u32le(response, at, window.period_max_us, capacity);
        at = append_u32le(response, at, window.late, capacity);
        at = append_u16le(response, at, window.jitter_p50_us, capacity);
        at = append_u16le(response, at, window.jitter_p99_us, capacity);
        at = append_u16le(response, at, window.jitter_max_us, capacity);
        at = append_u32le(response, at, window.jitter_over, capacity);
        for (i = 0u; i < AK_PROTO_PERF_SECTIONS; i++) {
            at = append_u16le(response, at, window.section_avg_us_x10[i],
                              capacity);
        }
        for (i = 0u; i < AK_PROTO_PERF_SECTIONS; i++) {
            at = append_u16le(response, at, window.section_max_us[i], capacity);
        }
        at = append_u16le(response, at, window.load_permille, capacity);
        break;
    }

    case AK_PROTO_CMD_MOTOR_TELEMETRY: {
        /* How fast each motor is turning, which is the one reading here that a
         * board in this tree cannot take yet.
         *
         * A null callback means "no telemetry path in this build", and the reply
         * is one byte and stops - the same refusal RC_CHANNELS makes about a
         * missing receiver port, and for the same reason: a header, a count and
         * four entries of zeros would read on a screen as four motors that are
         * stopped, when the truth is that nobody is listening to them.
         *
         * A callback that returns zero says the same thing and is the same
         * answer, deliberately: the difference between a build with no callback
         * and a build whose callback always declines is a build-time fact, and
         * the capability word in HELLO is where a client reads it.
         *
         * `telemetry` is cleared first so that a callback which returns nonzero
         * without filling every field still leads to a complete reply rather
         * than a header with a caller's stack behind it - the rule PERF,
         * MISSION and CALIBRATE already follow. */
        ak_proto_motor_telemetry_t telemetry;
        unsigned i;
        unsigned count;

        telemetry.count = 0u;
        telemetry.poles = 0u;
        for (i = 0u; i < AK_PROTO_MOTOR_MAX; i++) {
            telemetry.motor[i].flags = 0u;
            telemetry.motor[i].erpm = 0u;
            telemetry.motor[i].rpm = 0u;
            telemetry.motor[i].temperature = 0u;
            telemetry.motor[i].max_temperature = 0u;
            telemetry.motor[i].millivolts = 0u;
            telemetry.motor[i].milliamps = 0u;
            telemetry.motor[i].packets = 0u;
            telemetry.motor[i].invalid = 0u;
        }

        if (io->motor_telemetry == 0 ||
            !io->motor_telemetry(io->ctx, &telemetry)) {
            /* One byte and no more, as RC_CHANNELS does. */
            at = append_u8(response, at, AK_PROTO_MOTOR_NONE, capacity);
            break;
        }

        at = append_u8(response, at, AK_PROTO_MOTOR_OK, capacity);
        count = telemetry.count > AK_PROTO_MOTOR_MAX
                    ? (unsigned)AK_PROTO_MOTOR_MAX
                    : (unsigned)telemetry.count;
        at = append_u8(response, at, (uint8_t)count, capacity);
        at = append_u8(response, at, telemetry.poles, capacity);
        for (i = 0u; i < count; i++) {
            const ak_proto_motor_t *m = &telemetry.motor[i];
            at = append_u8(response, at, m->flags, capacity);
            at = append_u32le(response, at, m->erpm, capacity);
            at = append_u32le(response, at, m->rpm, capacity);
            at = append_u8(response, at, m->temperature, capacity);
            at = append_u8(response, at, m->max_temperature, capacity);
            at = append_u16le(response, at, m->millivolts, capacity);
            at = append_u16le(response, at, m->milliamps, capacity);
            at = append_u16le(response, at, m->packets, capacity);
            at = append_u16le(response, at, m->invalid, capacity);
        }
        break;
    }

    default:
        proto->unknown_commands++;
        at = append_u8(response, at, 0x7F, capacity);
        break;
    }

    return finish_response(response, at);
}

unsigned ak_proto_feed(ak_proto_t *proto, const ak_proto_io_t *io, uint8_t byte,
                       uint32_t now_ms, uint8_t *response, unsigned capacity)
{
    proto->bytes++;

    /* A frame that stops part way is abandoned after a gap. Bytes inside a frame
     * arrive back to back, so a gap that long means the sender stopped - and
     * without this the parser would swallow the start of the next frame. */
    if (proto->state != STATE_SYNC1 &&
        (uint32_t)(now_ms - proto->last_byte_ms) > AK_PROTO_GAP_MS) {
        proto->bad_length++;
        proto->state = STATE_SYNC1;
    }
    proto->last_byte_ms = now_ms;

    switch (proto->state) {
    case STATE_DRAIN:
        /* Deliberately the same bytes read the same way as STATE_SYNC1 below:
         * a sync pair inside the tail of an abandoned frame is far likelier to
         * be the next frame than a coincidence, and reading it as one is what
         * resynchronises a client that pipelined its requests. Falling through
         * rather than repeating the branch is the point - these two states
         * differ in what the caller does with the byte, and nowhere else. */
        /* fall through */
    case STATE_SYNC1:
        if (byte == AK_PROTO_SYNC1) {
            proto->state = STATE_SYNC2;
        }
        break;

    case STATE_SYNC2:
        if (byte == AK_PROTO_SYNC2) {
            proto->held = 0;
            /* The smallest frame is version, command and length; the length
             * byte then says how much more to expect. Leaving this at zero is
             * how every frame failed the first time: the first body byte was
             * already "as many bytes as expected". */
            proto->expected = 3;
            proto->state = STATE_BODY;
        } else if (byte != AK_PROTO_SYNC1) {
            proto->state = STATE_SYNC1;
        }
        /* A second 0xAA is the start of the pair, not noise before it: stay
         * here. Going back to SYNC1 dropped `AA AA 55 <frame>` - a stray byte
         * ahead of a real frame cost the frame. */
        break;

    case STATE_BODY:
        proto->buffer[proto->held++] = byte;

        /* Version, command and length arrive first; after that the length says
         * how much more belongs to this frame. */
        if (proto->held == 3u) {
            if (proto->buffer[2] > AK_PROTO_MAX_PAYLOAD) {
                proto->bad_length++;
                /* Drained rather than idle: this frame was cut off before its
                 * end, and the sender is still sending the rest of it. */
                proto->state = STATE_DRAIN;
                break;
            }
            proto->expected = (uint16_t)(3u + proto->buffer[2]);
        }
        if (proto->held >= proto->expected) {
            proto->state = STATE_CRC_LOW;
        }
        break;

    case STATE_CRC_LOW:
        proto->crc_received = byte;
        proto->state = STATE_CRC_HIGH;
        break;

    case STATE_CRC_HIGH: {
        proto->state = STATE_SYNC1;
        proto->crc_received = (uint16_t)(proto->crc_received | ((uint16_t)byte << 8));

        uint16_t crc = ak_proto_crc16(proto->buffer, proto->expected);
        if (proto->crc_received != crc) {
            proto->bad_crc++;
            break;
        }
        proto->frames++;

        unsigned written = handle_command(proto, io, proto->buffer, capacity,
                                          response);
        if (written > 0) {
            proto->responses++;
        }
        return written;
    }
    }

    return 0;
}

void ak_proto_report(const ak_proto_t *proto, ak_printf_fn out)
{
    out("protocol:  %u bytes, %u frames, %u responses, %u bad crc, %u bad "
        "length, %u unknown\n",
        proto->bytes, proto->frames, proto->responses, proto->bad_crc,
        proto->bad_length, proto->unknown_commands);
}

int ak_proto_idle_at(const ak_proto_t *proto, uint32_t now_ms)
{
    if (proto->state == STATE_SYNC1) {
        return 1;
    }
    /* The same test, against the same constant, that ak_proto_feed makes before
     * it abandons a frame - so the caller and the parser cannot disagree about
     * whose byte this is. They would drift apart on exactly the byte they
     * disagree about, because a byte the caller gives to the console never
     * reaches ak_proto_feed and so never advances last_byte_ms. */
    return (uint32_t)(now_ms - proto->last_byte_ms) > AK_PROTO_GAP_MS;
}
