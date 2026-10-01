#ifndef AK_TOOLS_AK_CONTROL_H
#define AK_TOOLS_AK_CONTROL_H

/*
 * The control core as a library, for a program that is not this firmware.
 *
 * Everything else in this repository that drives the flight core drives it
 * through `main()`: the host simulator implements a whole board and calls
 * `ak_firmware_main()`, and the tests call one module each. Both are the wrong
 * shape for the thing the assessment asks for next - a *Python* simulation
 * closing a loop around the real control core - because a simulation wants
 * three calls and a clock it owns, not a console, not a parameter table on
 * flash, and not a scheduler.
 *
 * So this is the narrow seam: `init`, `step`, `state`. It is deliberately not a
 * rewrite of `main()`. What runs behind it is the same object files the MCU
 * image is built from - `ak_flight.c`, `ak_estimator.c`, `ak_pid.c`,
 * `ak_mixer.c`, `ak_rc.c`, `ak_output.c` - which is possible because
 * `src/core/flight/` includes no MCU header and does no I/O, a property the
 * Makefile has relied on since the host build existed.
 *
 * **What crosses the boundary is stated in one place.** Every field below is
 * named, typed, unit-ed and frame-ed in `docs/31-contract.md`, and the contract
 * is what this file implements rather than something written beside it:
 *
 *   - C5: every sample carries `(t_ms, sequence)`, and the consumer applies an
 *     age bound. A sample that is merely *present* is not a sample that is
 *     *current*.
 *   - C6: the command is the stick shape, and a command carries the time it was
 *     formed. The firmware has no TTL on a command value; the boundary does,
 *     because the boundary is the consumer.
 *   - C7: actuators cross as motors 0..1 and servos -1..1. The airframe's own
 *     units - newtons, radians - stay on the airframe's side of the line.
 *   - C8: the configuration has an identity, and it is on the state.
 *
 * **There is no field here for truth.** The estimator's attitude is in
 * `akc_state_t`; the plant's attitude is not in any struct in this file, so a
 * simulation cannot accidentally fly the C core on the answer. A binding that
 * wants to compare the two has to keep the second one itself, which is what
 * makes "estimates separately from truth" a property of the interface rather
 * than a promise in a comment.
 */

#include <stdint.h>

#include "ak_flight.h"
#include "ak_params.h"
#include "ak_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Bumped whenever a struct below changes shape. A binding checks it and
 * refuses to run against a library that does not match, because a struct that
 * crossed a language boundary with one field moved is a transcription error
 * that produces plausible numbers rather than a crash.
 *
 * Version 2 added `akc_layout_t` and the field-by-field description of every
 * struct, which is the check that makes the version number worth having.
 */
#define AKC_ABI_VERSION 2u

/* How long a sample may be old before the core will not fly on it, and how long
 * a command may sit before it is refused. The first is a safety bound the
 * firmware already applies to its own links by other means (`rc_timeout_ms`);
 * the second has no counterpart inside the firmware, on purpose, and lives at
 * the boundary for the reason C6 gives. Both are here rather than in the
 * binding so that the C side is the one that decides. */
#define AKC_SAMPLE_MAX_AGE_MS 250u
#define AKC_COMMAND_MAX_AGE_MS 500u

/* The result of a step. Anything but OK means the outputs are the last ones the
 * core produced and the caller should not treat the step as having flown. */
typedef enum {
    AKC_OK = 0,
    AKC_NO_IMU = 1,        /* no usable inertial sample this step */
    AKC_SAMPLE_STALE = 2,  /* the sample is older than AKC_SAMPLE_MAX_AGE_MS */
    AKC_COMMAND_EXPIRED = 3, /* the command is older than AKC_COMMAND_MAX_AGE_MS */
    AKC_NOT_CONFIGURED = 4,
    AKC_BAD_ARGUMENT = 5,
} akc_result_t;

/*
 * One inertial sample. `t_ms` is when the sample was *taken*, which is not the
 * same as when the step that consumes it runs: an IMU that reports at 8 kHz and
 * a loop that runs at 1 kHz is exactly the case where the difference is real,
 * and it is the difference a stale-sample bug hides in.
 */
typedef struct {
    uint32_t t_ms;
    uint32_t sequence;  /* the producer's counter: a gap is a dropped sample */
    float    gyro[3];   /* rad/s, body FRD, per ak_types.h */
    float    accel[3];  /* g, body FRD */
    /* Validity is a field, not a convention about a sentinel value: the
     * difference between "the part said zero" and "the part said nothing" is
     * the whole of the `noimu` session, and a zero rate is a *plausible*
     * reading - a stationary aircraft gives one. */
    int      valid;
} akc_imu_t;

typedef struct {
    uint32_t t_ms;
    uint32_t sequence;
    float    pressure_pa;
    float    temperature_c;
    int      valid;
} akc_baro_t;

typedef struct {
    uint32_t t_ms;
    uint32_t sequence;
    int32_t  lat_e7;
    int32_t  lon_e7;
    int32_t  alt_msl_mm;
    int32_t  speed_mm_s;
    int32_t  course_e5;   /* 1e-5 degrees */
    uint8_t  fix_type;    /* u-blox 0-5 */
    uint8_t  satellites;
    int      valid;
} akc_gps_t;

/*
 * Everything the core is given for one step. The outer `t_ms`/`sequence` are
 * the *frame's*, because the three sensors do not run at one rate - a GPS at
 * 5 Hz inside a 1 kHz loop is the normal case, not an edge one - and a consumer
 * that could only see one timestamp would have to pretend they did.
 */
typedef struct {
    uint32_t  t_ms;
    uint32_t  sequence;
    akc_imu_t imu;
    akc_baro_t baro;
    akc_gps_t gps;
} akc_samples_t;

/*
 * The stick shape, per C6, plus the time it was formed. `angle_mode` and
 * `arm_request` are here as well as in the firmware's own command struct
 * because this is the *input* side: a simulation says what the switches are,
 * and the firmware decides what that means.
 */
typedef struct {
    uint32_t t_ms;
    float    roll;      /* -1..1 */
    float    pitch;     /* -1..1 */
    float    yaw;       /* -1..1 */
    float    throttle;  /* 0..1 */
    int      angle_mode;
    int      arm_request;
} akc_command_t;

/*
 * What the core produced, in the units C7 fixes for the crossing: normalised,
 * because the board's PWM range and the ESC's calibration are not the flight
 * controller's business. The encoded frame is carried too - it is what a timer
 * would actually emit, and the DShot field's zero-means-off rule is a decision
 * that belongs to this side of the line.
 */
typedef struct {
    uint32_t t_ms;
    uint32_t sequence;
    float    motor[AK_MAX_MOTORS];  /* 0..1, 0 = stopped */
    float    servo[AK_MAX_SERVOS];  /* -1..1, 0 = centre */
    uint16_t dshot[AK_MAX_MOTORS];
    uint16_t servo_us[AK_MAX_SERVOS];
} akc_outputs_t;

/*
 * What the core believes. Every number here is an *estimate* or a state; none
 * of it is a measurement of the world, and there is no field for one.
 */
typedef struct {
    uint32_t now_ms;
    uint32_t steps;
    int      state;        /* ak_flight_state_t: C9's vocabulary */
    int      link_live;    /* freshness, not presence: C5 */
    int      converged;    /* has the estimator been levelled by the accelerometer */
    int      arm_block;    /* ak_arm_block_t: why not, while it is not armed */
    float    arm_detail;   /* the number that goes with that reason */

    float    q_wxyz[4];    /* the estimator's attitude, body -> NED */
    float    roll, pitch, yaw;  /* rad, derived; yaw is from north, clockwise */
    float    gyro[3];      /* rad/s, the filtered rates the loop actually flew on */
    float    rate_setpoint[3];  /* rad/s, what the control law asked for */
    float    torque[3];    /* -1..1, what the mixer was handed */

    /* What the receiver decoded to, so a caller can see that the sticks it
     * asked for are the sticks the firmware got. */
    float    cmd_roll, cmd_pitch, cmd_yaw, cmd_throttle;
    int      cmd_angle_mode, cmd_arm_request;

    /* The ages and the verdicts of the last step's samples: C5 made visible
     * rather than assumed. */
    uint32_t imu_age_ms, baro_age_ms, gps_age_ms;
    int      imu_stale, baro_stale, gps_stale, command_expired;

    /* B4's bounded timing, which is the same struct the console prints. */
    uint32_t timing_gap_steps, timing_catchup_steps, timing_dropped_ms;
    uint32_t timing_duplicates, timing_long_loops, timing_max_loop_ms;

    uint32_t config_hash;  /* C8: which configuration this is */
    uint32_t mix_needed_motors, mix_needed_servos;
    uint32_t board_motors, board_servos;
} akc_state_t;

/*
 * And the description of all of the above, so that a binding can *check* its
 * own idea of the layout instead of asserting it.
 *
 * A struct crossing a language boundary fails silently. The C compiler pads,
 * the other language does not, and the result is a program that reads a
 * neighbour's field and produces numbers that look like numbers. The only
 * defence that works is to ask the side that has the compiler what the offsets
 * are and refuse to run if the answer is not what the other side wrote down.
 */
#define AKC_LAYOUT_MAX 64

typedef struct {
    const char *name;
    uint32_t    offset;
    uint32_t    size;
} akc_field_t;

typedef struct {
    const char *name;
    uint32_t    size;
    /* How many rows of `field` are filled. Counted on the way out of
     * akc_layout() rather than written down next to the rows, because a stale
     * count is the one error in this table that reads as agreement: too low and
     * the other side compares a prefix, matches it, and never sees the fields
     * that moved. */
    uint32_t    fields;
    akc_field_t field[AKC_LAYOUT_MAX];
} akc_layout_t;

/* --- the interface ------------------------------------------------------- */

uint32_t    akc_abi_version(void);
const char *akc_product(void);
const char *akc_board(void);
const char *akc_revision(void);
const char *akc_built(void);

/*
 * Configure the core from the firmware's own saved format - "name=value\n" per
 * line, the text `save` writes and `load` reads. Returns 0, or -1 with `msg`
 * set. The airframe comes from the `airframe` parameter in that text, exactly
 * as it does on a board; it is not a separate argument, because a second way to
 * say which aircraft this is would be a second answer.
 *
 * This is also what gives the configuration its identity: akc_config_hash()
 * is the hash of what this call left in the table.
 */
int akc_config(const char *text, char *msg, unsigned msg_len);

/* Tell the core how many outputs the board it is standing in for has. Without
 * it the arm gate refuses, which is the same answer a real board with no
 * timers gets. */
int akc_set_board_outputs(unsigned motors, unsigned servos);

uint32_t akc_config_hash(void);

/* The table as text, so an experiment can record what it ran. Returns the
 * number of bytes written, or 0 if the buffer is too small. */
unsigned akc_config_text(char *buf, unsigned len);

void akc_reset(void);

/*
 * One control step, at a clock the caller owns. `now_ms` is the caller's: the
 * simulated time the sample is being consumed at, which is what the age bounds
 * are measured against. Returns an akc_result_t.
 *
 * The outputs are written even when the result is not OK - they are the last
 * ones the core produced - so a caller that ignores the result still gets
 * something rather than a stale buffer it cannot distinguish from a fresh one.
 * `outputs` may be null, which a caller that only wants the state's answer to
 * "what did it do" can use.
 */
int akc_step(const akc_samples_t *samples, const akc_command_t *command,
             uint32_t now_ms, akc_outputs_t *outputs);

int akc_state(akc_state_t *state);

/* `which` names a struct: "imu", "baro", "gps", "samples", "command",
 * "outputs", "state". Returns 0 or -1. */
int akc_layout(const char *which, akc_layout_t *out);

#ifdef __cplusplus
}
#endif

#endif /* AK_TOOLS_AK_CONTROL_H */
