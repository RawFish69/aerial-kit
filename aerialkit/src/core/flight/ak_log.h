#ifndef AK_FLIGHT_AK_LOG_H
#define AK_FLIGHT_AK_LOG_H

#include "ak_console.h"
#include "ak_types.h"

/*
 * The blackbox: what the loop saw and what it did about it.
 *
 * A fixed ring of records in RAM, written once per loop iteration and dumped
 * over the console as CSV. It exists because the interesting failures are the
 * ones that happen for 200 milliseconds and leave nothing behind: an armed
 * check on the bench, a first hover, a failsafe that triggered at the wrong
 * moment. A memory of what happened is not evidence; a file is.
 *
 * Everything is stored as integers in fixed units. That is not only for size:
 * the console formatter has no floating point on purpose, so a log of integers
 * dumps without one, and the units are in the header line of the CSV rather
 * than in the reader's head.
 *
 * When the ring fills it overwrites the oldest record, because the newest data
 * is the data you want after something went wrong. The count of what was
 * overwritten is kept, so "the log starts mid-flight" is visible rather than
 * puzzling.
 */

#ifndef AK_LOG_CAPACITY
#define AK_LOG_CAPACITY 384
#endif

/* Bumped when the record layout changes. A ring left behind by an older build
 * is not a ring this one can read, and reading it anyway is how a log becomes
 * a hallucination. */
#define AK_LOG_VERSION 2u
#define AK_LOG_MAGIC   0x414B4C47u /* "AKLG" */

typedef struct {
    uint32_t time_ms;    /* 1 ms */
    int16_t  gyro[3];    /* 0.1 deg/s */
    int16_t  accel[3];   /* 0.001 g */
    int16_t  attitude[2]; /* 0.1 deg, roll and pitch */
    /* The third angle, which the return depends on and the log did not carry:
     * a GPS-track-aligned yaw is what the quadrotor's return translates with,
     * and a log without it cannot show whether the alignment happened.
     *
     * A tenth of a degree, and *wrapped* to +/-180: the estimate itself is a
     * continuous angle (the hover subtracts two readings of it), and nine turns
     * of one is all this field can hold. It is written through
     * ak_attitude_ddeg(), which is also what the attitude telemetry goes
     * through, so the two agree. */
    int16_t  yaw;         /* 0.1 deg */
    /* And the height the navigator flies on, above the take-off reference -
     * the one number a bad landing or a return is judged by, and the one this
     * record was missing. Not metres above the sea: the question after a
     * flight is "how high above the ground it started from". */
    int32_t  alt_mm;      /* above the take-off reference */
    int16_t  stick[4];   /* -1000..1000 per-mille: roll, pitch, yaw, throttle */
    int8_t   torque[3];  /* -100..100 percent of the mix input */
    uint8_t  motor[4];   /* 0..254, or 255 for no output */
    uint8_t  state;      /* ak_flight_state_t */
    uint8_t  flags;      /* what was valid this iteration */
    /*
     * And where it was, in the module's own units: 1e-7 degrees, the same
     * numbers the fix carries and the same ones the config protocol speaks.
     *
     * This is the first question after a flight that ends somewhere
     * unexpected, and it was the one thing the log could not answer. The fast
     * ring lives in RAM and the console showed the position all along; the log
     * that survives the battery - the one that would still be there after a
     * crash in a field - had no position in it at all, so "where did it come
     * down" was a question for the search party's memory of the telemetry.
     */
    int32_t  lat_e7;
    int32_t  lon_e7;
} ak_log_record_t;

/* flags */
#define AK_LOG_RC_LIVE   0x01
#define AK_LOG_IMU_VALID 0x02
/* Whether lat_e7/lon_e7 hold a *fix* or the last one the module sent: a
 * position from a module that has stopped answering is a place the aircraft
 * was, not a place it is. */
#define AK_LOG_GPS_VALID 0x04

typedef struct {
    /* The header first, because a ring that outlives a reset has to be
     * checked before it is believed: uninitialised RAM looks exactly like a
     * log with an unlucky head and count. */
    uint32_t magic;
    uint16_t version;
    uint16_t capacity;   /* records this ring was built for */
    uint32_t written;    /* every record ever written */
    uint32_t overwritten;
    uint16_t head;
    uint16_t count;     /* how many are in the ring, up to capacity */
    uint16_t decimation; /* how many loop iterations each record covers */
    uint16_t reserved;
    ak_log_record_t records[AK_LOG_CAPACITY];
} ak_log_t;

void ak_log_init(ak_log_t *log);

/*
 * Prepare a ring that may already hold one.
 *
 * A log in memory startup does not clear arrives with a header from the run
 * before - if that run was this build, and if the RAM really was retained.
 * Anything else is a fresh ring. Returns 1 when it kept what was there, which
 * is what the caller prints at boot: "there is a log from last time" is worth
 * knowing before the aircraft is picked up.
 */
int ak_log_resume(ak_log_t *log);

void ak_log_push(ak_log_t *log, const ak_log_record_t *record);

void ak_log_reset(ak_log_t *log);

/* Reads one record, counted from the oldest the ring still holds. Returns 1
 * when there was one, 0 when the index is past the end - which is what a
 * caller streaming the log to something else needs, and is also how the ring's
 * wrap is proved: index 0 is never the newest record once it has wrapped. */
int ak_log_get(const ak_log_t *log, uint16_t index, ak_log_record_t *out);

/* Says how often a record is written, for the header. The log does not act on
 * it: the caller decides when to push, because the caller knows the loop
 * rate. */
void ak_log_set_decimation(ak_log_t *log, uint16_t loops_per_record);

/* Writes the ring oldest first, with a header line naming the units. The
 * caller supplies the output, so this is usable on a console, in a test, and
 * later over a wire protocol. */
void ak_log_dump(const ak_log_t *log, ak_printf_fn out);

/* One line per record, for a protocol that wants to stream them. */
void ak_log_write_record(const ak_log_record_t *record, ak_printf_fn out);

/* The same record as fixed-width little-endian bytes, for a wire.
 *
 * Not a memcpy of the struct: a C struct carries padding that differs between
 * compilers and architectures, and a protocol whose field offsets depend on
 * which machine built the firmware is a protocol that works until it does not.
 * The layout is written out here and in docs/16-protocol.md:
 *
 *   u32 time_ms, i16 gyro[3], i16 accel[3], i16 attitude[2], i16 yaw,
 *   i32 alt_mm, i16 stick[4], i8 torque[3], u8 motor[4], u8 state, u8 flags,
 *   i32 lat_e7, i32 lon_e7                                  = 51 bytes
 */
#define AK_LOG_WIRE_BYTES 51u

unsigned ak_log_encode_record(const ak_log_record_t *record, uint8_t *out,
                              unsigned capacity);

/* And back again: 1 when a record was decoded, 0 when the buffer is too short
 * for one. This is what reads a log out of flash. */
int ak_log_decode_record(const uint8_t *in, unsigned len,
                         ak_log_record_t *out);

#endif /* AK_FLIGHT_AK_LOG_H */
