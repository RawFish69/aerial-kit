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
#define AK_LOG_VERSION 4u
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
    /*
     * And the filtering itself (roadmap 2.4), because everything above is the
     * same record whether the gyro chain is doing anything or not: `gyro` is
     * the driver's reading and `torque` is what the controller made of it, and
     * a log of those two cannot tell "the chain removed the vibration" from
     * "there was no vibration to remove".
     *
     * Appended, not inserted, and that is the whole of the wire's compatibility
     * story. The first fifty-one bytes are byte-for-byte what version 2 wrote,
     * so a client built before this milestone decodes everything it knows and
     * ignores the tail rather than reading every field after a shift as noise.
     * A record layout may grow at the end or not at all.
     *
     * `gyro_filtered` is the same quantity as `gyro` in the same units, and it
     * is the number the controller actually flew on: the notch bank and both
     * low-passes (ak_flight.c, `flight->gyro`). Put the two columns side by
     * side and the chain's contribution is not an inference.
     */
    int16_t  gyro_filtered[3]; /* 0.1 deg/s, after the notch and both LPFs */
    /*
     * Where the notches are, which is the one number a tracked filter has to
     * report or it cannot be judged at all: the bank's centres move on their
     * own, and a log that showed the vibration going away without showing the
     * notch moving onto it would not distinguish a working tracker from a
     * fixed filter that happened to be in the right place.
     *
     * `notch_engaged[axis]` is how many slots a *measurement* has put in place
     * on that axis - the module engages a notch by measuring a peak, not by
     * being configured with one, so a bank that is running and has not yet
     * completed a window has zero of them and this is how a reader tells that
     * from a bank that has locked on (ak_dyn_notch.h, decision 7).
     *
     * `notch_hz[axis]` is the centre of the first engaged notch on that axis,
     * in whole Hz, or 0 when none is engaged - and zero is the honest value
     * rather than a placeholder, because a slot with no measurement behind it
     * is *bypassed*: it filters nothing, so there is no centre to report.
     *
     * One per axis rather than one per slot, and the price of the alternative
     * is what decides it - measured rather than estimated, from the two F405
     * images either side of this milestone: the record went from 56 bytes to
     * 72, and the image's RAM from 94,684 to 113,116, so these sixteen bytes
     * cost 18,432. That is 1,152 bytes of the part's 131,072 for every byte
     * here, because the record was counted **three** times over when that was
     * measured: the fast ring, the long ring, and a fallback ring the core kept
     * against a board with no retained RAM.
     *
     * **Three is now two.** The fallback is gone - no board this repository
     * ships could reach it, because all seven return a whole retained ring from
     * `ak_board_retained_ram`, so it was 27,672 bytes of `.bss` chosen against
     * `else` (see ak_board.h, and docs/evidence/phase-4.1-dead-ring-*.txt for
     * the measurement). The F405's RAM is 85,444 for that. So these sixteen
     * bytes cost 18,432 at the price the decision was taken at and **12,288**
     * now: the same decision, cheaper. Both numbers are here rather than the
     * smaller one alone because the larger is what was measured when the
     * choice was made, and a comment that quietly kept only the favourable
     * arithmetic would stop being a record of anything.
     *
     * At that price the four extra slots per axis the module can hold would be
     * 27,648 bytes at three rings and 18,432 at two - a fifth of the part, or
     * a seventh - for peaks the tracker already ranks below the first. So one
     * centre per axis: the first slot is the lowest-frequency peak the tracker
     * kept, the others are that same measurement's lesser peaks, and the
     * question this log answers - "does the centre follow the motor
     * fundamental?" - is asked of the first one.
     */
    uint16_t notch_hz[3];      /* whole Hz, 0 when nothing is engaged */
    uint8_t  notch_engaged[3];
    /*
     * Roadmap 4.1's fields that have a source in this firmware, appended after
     * version 3's tail for the same reason that tail was appended: a client
     * built before them decodes a correct prefix (docs/16-protocol.md).
     *
     * `time_us` is the IMU sample's own timestamp - the clock the estimator's
     * dt is measured on - so the loop's real cadence can be read from the log
     * rather than inferred from `time_ms`, which is the scheduler's.
     *
     * `rate_setpoint` is what the rate loop was asked to hold, in the gyro
     * columns' unit, and `pid_p`/`pid_i`/`pid_d` are the three terms it answered
     * with in the torque column's unit: P + I - D is the torque before its
     * clamp, so a saturated controller and a wound-up integrator are both
     * visible. See ak_flight_log_control.
     *
     * `vbat_mv` is the pack, 0 with AK_LOG_VBAT_VALID clear on a board that
     * cannot measure one (the Feather's divider ratio is unmeasured).
     *
     * **The rest of 4.1's list has no source and is not here**, rather than
     * written as zeros a reader would believe: battery current (no sensor on
     * any board), supervisor state and shadow output (phase 6 does not exist),
     * feed-forward (ak_pid.h has none, by design), per-motor eRPM (phase 3's
     * telemetry has never run on a wire). docs/11-blackbox.md lists them with
     * what each waits for.
     */
    uint32_t time_us;          /* the IMU sample's timestamp */
    int16_t  rate_setpoint[3]; /* 0.1 deg/s */
    int8_t   pid_p[3];         /* percent of the mix input, saturated +/-127 */
    int8_t   pid_i[3];
    int8_t   pid_d[3];         /* as subtracted */
    uint16_t vbat_mv;          /* 0 when AK_LOG_VBAT_VALID is clear */
} ak_log_record_t;

/* flags */
#define AK_LOG_RC_LIVE   0x01
#define AK_LOG_IMU_VALID 0x02
/* Whether lat_e7/lon_e7 hold a *fix* or the last one the module sent: a
 * position from a module that has stopped answering is a place the aircraft
 * was, not a place it is. */
#define AK_LOG_GPS_VALID 0x04
/* Version 4 (roadmap 4.1): the pilot's mode switch, and whether vbat_mv is a
 * measurement. */
#define AK_LOG_ANGLE_MODE 0x08
#define AK_LOG_VBAT_VALID 0x10

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

/* The units, what the two gyro triples mean, and the column line that names
 * every field `ak_log_write_record` writes.
 *
 * One writer for both dumps - the ring's and the flash log's - because they are
 * the same file to a reader, and when they were two copies of these lines the
 * flash one named twenty-four columns while its own rows carried twenty-six:
 * the ring's copy had gained `lat_e7,lon_e7` and the flash's had not. The rows
 * carry thirty-five since 2.4, so the same stale copy is eleven short. A
 * header a client cannot match to its row is a log that reads as a column of
 * the wrong numbers, which is worse than no log. */
void ak_log_write_header(ak_printf_fn out);

/* The same record as fixed-width little-endian bytes, for a wire.
 *
 * Not a memcpy of the struct: a C struct carries padding that differs between
 * compilers and architectures, and a protocol whose field offsets depend on
 * which machine built the firmware is a protocol that works until it does not.
 * The layout is written out here and in docs/16-protocol.md:
 *
 *   u32 time_ms, i16 gyro[3], i16 accel[3], i16 attitude[2], i16 yaw,
 *   i32 alt_mm, i16 stick[4], i8 torque[3], u8 motor[4], u8 state, u8 flags,
 *   i32 lat_e7, i32 lon_e7,                                  = 51 bytes
 *   i16 gyro_filtered[3], u16 notch_hz[3], u8 notch_engaged[3]  = 15 more
 *                                                  = 66 bytes (version 3)
 *   u32 time_us, i16 rate_setpoint[3], i8 pid_p[3], i8 pid_i[3], i8 pid_d[3],
 *   u16 vbat_mv                                              = 21 more
 *
 *                                                  = 87 bytes (AK_LOG_VERSION 4)
 *
 * The first fifty-one are version 2's and the next fifteen version 3's,
 * unmoved - each tail is appended, so a client that predates it decodes a
 * correct prefix instead of a shifted one.
 */
#define AK_LOG_WIRE_BYTES 87u
/* Where version 3's record ended: a reader of an older log stops here. */
#define AK_LOG_WIRE_BYTES_V3 66u

unsigned ak_log_encode_record(const ak_log_record_t *record, uint8_t *out,
                              unsigned capacity);

/* And back again: 1 when a record was decoded, 0 when the buffer is too short
 * for one. This is what reads a log out of flash. */
int ak_log_decode_record(const uint8_t *in, unsigned len,
                         ak_log_record_t *out);

#endif /* AK_FLIGHT_AK_LOG_H */
