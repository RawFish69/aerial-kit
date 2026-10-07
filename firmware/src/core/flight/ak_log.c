#include "ak_log.h"

void ak_log_init(ak_log_t *log)
{
    log->magic = AK_LOG_MAGIC;
    log->version = AK_LOG_VERSION;
    log->capacity = AK_LOG_CAPACITY;
    log->head = 0;
    log->count = 0;
    log->written = 0;
    log->overwritten = 0;
    log->decimation = 1;
}

int ak_log_resume(ak_log_t *log)
{
    /* Every field has to make sense before the records are believed: the magic
     * says this is a log, the version says it is this layout, the capacity says
     * the records fit the array they are in, and head and count say the ring
     * points somewhere real. Uninitialised RAM passes the magic test about once
     * in four billion boots, and fails one of the others far more often. */
    int valid = log->magic == AK_LOG_MAGIC &&
                log->version == AK_LOG_VERSION &&
                log->capacity == AK_LOG_CAPACITY &&
                log->count <= AK_LOG_CAPACITY &&
                log->head < AK_LOG_CAPACITY;

    if (valid) {
        return 1;
    }
    ak_log_init(log);
    return 0;
}

void ak_log_set_decimation(ak_log_t *log, uint16_t loops_per_record)
{
    log->decimation = loops_per_record == 0 ? 1 : loops_per_record;
}

void ak_log_push(ak_log_t *log, const ak_log_record_t *record)
{
    log->records[log->head] = *record;
    log->head = (uint16_t)((log->head + 1u) % AK_LOG_CAPACITY);
    if (log->count < AK_LOG_CAPACITY) {
        log->count++;
    } else {
        log->overwritten++;
    }
    log->written++;
}

void ak_log_reset(ak_log_t *log)
{
    ak_log_init(log);
}

int ak_log_get(const ak_log_t *log, uint16_t index, ak_log_record_t *out)
{
    if (index >= log->count) {
        return 0;
    }
    uint16_t start = log->count < AK_LOG_CAPACITY ? 0 : log->head;
    uint16_t at = (uint16_t)((start + index) % AK_LOG_CAPACITY);
    *out = log->records[at];
    return 1;
}

void ak_log_write_record(const ak_log_record_t *record, ak_printf_fn out)
{
    out("%u,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%u,%u,%u,%u,"
        "%u,%u,%d,%d,%d,%d,%d,%u,%u,%u,%u,%u,%u,"
        "%u,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%u\n",
        record->time_ms,
        record->gyro[0], record->gyro[1], record->gyro[2],
        record->accel[0], record->accel[1], record->accel[2],
        record->attitude[0], record->attitude[1], record->yaw, record->alt_mm,
        record->stick[0], record->stick[1], record->stick[2], record->stick[3],
        record->torque[0], record->torque[1], record->torque[2],
        record->motor[0], record->motor[1], record->motor[2], record->motor[3],
        record->state, record->flags, record->lat_e7, record->lon_e7,
        record->gyro_filtered[0], record->gyro_filtered[1],
        record->gyro_filtered[2],
        record->notch_hz[0], record->notch_hz[1], record->notch_hz[2],
        record->notch_engaged[0], record->notch_engaged[1],
        record->notch_engaged[2],
        record->time_us,
        record->rate_setpoint[0], record->rate_setpoint[1],
        record->rate_setpoint[2],
        record->pid_p[0], record->pid_p[1], record->pid_p[2],
        record->pid_i[0], record->pid_i[1], record->pid_i[2],
        record->pid_d[0], record->pid_d[1], record->pid_d[2],
        record->vbat_mv);
}

void ak_log_write_header(ak_printf_fn out)
{
    out("# units: gyro 0.1 dps, accel 0.001 g, attitude 0.1 deg, "
        "alt mm above the take-off reference, sticks per-mille, torque "
        "percent, motor 0..254\n");
    /* The two gyro triples are the same quantity either side of the chain, and
     * the header says which is which rather than leaving it to be worked out
     * from the column order. `notch_engaged_n` is a count and `notch_hz_n` is a
     * frequency, or 0 for an axis with no notch engaged. */
    out("# gyro_* is the driver's reading; gyro_f* is what the notch bank and the "
        "two low-passes made of it, and is the number the controller flew on\n");
    out("# notch_hz_* is 0 when no notch is engaged on that axis; notch_engaged_* "
        "is how many a measurement has put in place there\n");
    out("# setpoint_* is the rate loop's target in 0.1 dps; p_*, i_*, d_* are its "
        "terms in torque percent (P + I - D is the torque before its clamp, "
        "+/-127 is saturated); vbat_mv is 0 unless flags has 0x10; flags 0x08 is "
        "angle mode\n");
    out("time_ms,gyro_x,gyro_y,gyro_z,accel_x,accel_y,accel_z,roll,pitch,yaw,"
        "alt_mm,stick_roll,stick_pitch,stick_yaw,stick_throttle,torque_roll,"
        "torque_pitch,torque_yaw,motor1,motor2,motor3,motor4,state,flags,"
        "lat_e7,lon_e7,gyro_fx,gyro_fy,gyro_fz,notch_hz_x,notch_hz_y,notch_hz_z,"
        "notch_engaged_x,notch_engaged_y,notch_engaged_z,"
        "time_us,setpoint_roll,setpoint_pitch,setpoint_yaw,"
        "p_roll,p_pitch,p_yaw,i_roll,i_pitch,i_yaw,d_roll,d_pitch,d_yaw,"
        "vbat_mv\n");
}

void ak_log_dump(const ak_log_t *log, ak_printf_fn out)
{
    out("# aerialkit blackbox, %u records", log->count);
    if (log->overwritten > 0) {
        out(" (%u overwritten, so this starts part way through)", log->overwritten);
    }
    out("\n");
    out("# sampled every %u loop iterations\n", log->decimation);
    ak_log_write_header(out);

    /* Oldest first: when the ring has wrapped, the oldest is the one the head
     * is about to overwrite. */
    uint16_t start = log->count < AK_LOG_CAPACITY
                         ? 0
                         : log->head;
    for (uint16_t i = 0; i < log->count; i++) {
        uint16_t index = (uint16_t)((start + i) % AK_LOG_CAPACITY);
        ak_log_write_record(&log->records[index], out);
    }
}

static unsigned put_i16(uint8_t *out, unsigned at, int16_t value)
{
    out[at++] = (uint8_t)((uint16_t)value & 0xFFu);
    out[at++] = (uint8_t)(((uint16_t)value >> 8) & 0xFFu);
    return at;
}

unsigned ak_log_encode_record(const ak_log_record_t *record, uint8_t *out,
                              unsigned capacity)
{
    if (capacity < AK_LOG_WIRE_BYTES) {
        return 0;
    }

    unsigned at = 0;
    uint32_t time = record->time_ms;
    for (int i = 0; i < 4; i++) {
        out[at++] = (uint8_t)((time >> (8 * i)) & 0xFFu);
    }
    for (int i = 0; i < 3; i++) {
        at = put_i16(out, at, record->gyro[i]);
    }
    for (int i = 0; i < 3; i++) {
        at = put_i16(out, at, record->accel[i]);
    }
    for (int i = 0; i < 2; i++) {
        at = put_i16(out, at, record->attitude[i]);
    }
    at = put_i16(out, at, record->yaw);
    {
        uint32_t alt = (uint32_t)record->alt_mm;

        for (int i = 0; i < 4; i++) {
            out[at++] = (uint8_t)((alt >> (8 * i)) & 0xFFu);
        }
    }
    for (int i = 0; i < 4; i++) {
        at = put_i16(out, at, record->stick[i]);
    }
    for (int i = 0; i < 3; i++) {
        out[at++] = (uint8_t)record->torque[i];
    }
    for (int i = 0; i < 4; i++) {
        out[at++] = record->motor[i];
    }
    out[at++] = record->state;
    out[at++] = record->flags;
    {
        uint32_t lat = (uint32_t)record->lat_e7;
        uint32_t lon = (uint32_t)record->lon_e7;

        for (int i = 0; i < 4; i++) {
            out[at++] = (uint8_t)((lat >> (8 * i)) & 0xFFu);
        }
        for (int i = 0; i < 4; i++) {
            out[at++] = (uint8_t)((lon >> (8 * i)) & 0xFFu);
        }
    }
    /* Everything above this line is version 2's record, and it stays byte for
     * byte: this is where roadmap 2.4's fields begin, and where a client that
     * has never heard of them stops reading. */
    for (int i = 0; i < 3; i++) {
        at = put_i16(out, at, record->gyro_filtered[i]);
    }
    for (int i = 0; i < 3; i++) {
        uint16_t hz = record->notch_hz[i];

        out[at++] = (uint8_t)(hz & 0xFFu);
        out[at++] = (uint8_t)((hz >> 8) & 0xFFu);
    }
    for (int i = 0; i < 3; i++) {
        out[at++] = record->notch_engaged[i];
    }
    /* Version 3 ends here; roadmap 4.1's fields begin. */
    for (int i = 0; i < 4; i++) {
        out[at++] = (uint8_t)((record->time_us >> (8 * i)) & 0xFFu);
    }
    for (int i = 0; i < 3; i++) {
        at = put_i16(out, at, record->rate_setpoint[i]);
    }
    for (int i = 0; i < 3; i++) {
        out[at++] = (uint8_t)record->pid_p[i];
    }
    for (int i = 0; i < 3; i++) {
        out[at++] = (uint8_t)record->pid_i[i];
    }
    for (int i = 0; i < 3; i++) {
        out[at++] = (uint8_t)record->pid_d[i];
    }
    out[at++] = (uint8_t)(record->vbat_mv & 0xFFu);
    out[at++] = (uint8_t)((record->vbat_mv >> 8) & 0xFFu);
    return at;
}

static uint32_t get_u32(const uint8_t *in, unsigned at)
{
    return (uint32_t)in[at + 0u] | ((uint32_t)in[at + 1u] << 8) |
           ((uint32_t)in[at + 2u] << 16) | ((uint32_t)in[at + 3u] << 24);
}

static int16_t get_i16(const uint8_t *in, unsigned at)
{
    return (int16_t)((uint16_t)in[at + 0u] | ((uint16_t)in[at + 1u] << 8));
}

/* The inverse of the encoder, for the same reason it exists: what was written
 * to a wire or into flash is laid out explicitly, so reading it back means
 * decoding it explicitly rather than casting a buffer to a struct. This is
 * what a log in flash is read with, which means it is also what has to agree
 * with docs/16-protocol.md - and the round-trip test is what holds the two
 * together. */
int ak_log_decode_record(const uint8_t *in, unsigned len,
                         ak_log_record_t *out)
{
    if (in == 0 || out == 0 || len < AK_LOG_WIRE_BYTES) {
        return 0;
    }

    unsigned at = 0u;
    out->time_ms = get_u32(in, at);
    at += 4u;
    for (int i = 0; i < 3; i++) {
        out->gyro[i] = get_i16(in, at);
        at += 2u;
    }
    for (int i = 0; i < 3; i++) {
        out->accel[i] = get_i16(in, at);
        at += 2u;
    }
    for (int i = 0; i < 2; i++) {
        out->attitude[i] = get_i16(in, at);
        at += 2u;
    }
    out->yaw = get_i16(in, at);
    at += 2u;
    out->alt_mm = (int32_t)get_u32(in, at);
    at += 4u;
    for (int i = 0; i < 4; i++) {
        out->stick[i] = get_i16(in, at);
        at += 2u;
    }
    for (int i = 0; i < 3; i++) {
        out->torque[i] = (int8_t)in[at++];
    }
    for (int i = 0; i < 4; i++) {
        out->motor[i] = in[at++];
    }
    out->state = in[at++];
    out->flags = in[at++];
    out->lat_e7 = (int32_t)get_u32(in, at);
    at += 4u;
    out->lon_e7 = (int32_t)get_u32(in, at);
    at += 4u;
    for (int i = 0; i < 3; i++) {
        out->gyro_filtered[i] = get_i16(in, at);
        at += 2u;
    }
    for (int i = 0; i < 3; i++) {
        out->notch_hz[i] = (uint16_t)get_i16(in, at);
        at += 2u;
    }
    for (int i = 0; i < 3; i++) {
        out->notch_engaged[i] = in[at++];
    }
    out->time_us = get_u32(in, at);
    at += 4u;
    for (int i = 0; i < 3; i++) {
        out->rate_setpoint[i] = get_i16(in, at);
        at += 2u;
    }
    for (int i = 0; i < 3; i++) {
        out->pid_p[i] = (int8_t)in[at++];
    }
    for (int i = 0; i < 3; i++) {
        out->pid_i[i] = (int8_t)in[at++];
    }
    for (int i = 0; i < 3; i++) {
        out->pid_d[i] = (int8_t)in[at++];
    }
    out->vbat_mv = (uint16_t)get_i16(in, at);
    return 1;
}
