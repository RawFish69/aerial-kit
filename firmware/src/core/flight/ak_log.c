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
        "%u,%u,%d,%d\n",
        record->time_ms,
        record->gyro[0], record->gyro[1], record->gyro[2],
        record->accel[0], record->accel[1], record->accel[2],
        record->attitude[0], record->attitude[1], record->yaw, record->alt_mm,
        record->stick[0], record->stick[1], record->stick[2], record->stick[3],
        record->torque[0], record->torque[1], record->torque[2],
        record->motor[0], record->motor[1], record->motor[2], record->motor[3],
        record->state, record->flags, record->lat_e7, record->lon_e7);
}

void ak_log_dump(const ak_log_t *log, ak_printf_fn out)
{
    out("# aerialkit blackbox, %u records", log->count);
    if (log->overwritten > 0) {
        out(" (%u overwritten, so this starts part way through)", log->overwritten);
    }
    out("\n");
    out("# sampled every %u loop iterations\n", log->decimation);
    out("# units: gyro 0.1 dps, accel 0.001 g, attitude 0.1 deg, "
        "alt mm above the take-off reference, sticks per-mille, torque "
        "percent, motor 0..254\n");
    out("time_ms,gyro_x,gyro_y,gyro_z,accel_x,accel_y,accel_z,roll,pitch,yaw,"
        "alt_mm,stick_roll,stick_pitch,stick_yaw,stick_throttle,torque_roll,"
        "torque_pitch,torque_yaw,motor1,motor2,motor3,motor4,state,flags,"
        "lat_e7,lon_e7\n");

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
    return 1;
}
