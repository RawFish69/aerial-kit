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
            value_length = sizeof text - 1u;
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
        } else {
            proto->state = STATE_SYNC1;
        }
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
