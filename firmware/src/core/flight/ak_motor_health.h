#ifndef AK_FLIGHT_MOTOR_HEALTH_H
#define AK_FLIGHT_MOTOR_HEALTH_H

#include <stdint.h>

#include "../filter/ak_filter.h"
#include "ak_dshot_edt.h"
#include "ak_types.h"

/*
 * Motor health: whether a motor that has been asked to turn is turning the way
 * this aircraft's own experience says it should.
 *
 * The roadmap asks for "RPM far from the command model". This file is that
 * sentence made into something a flight core can call, and the hard part is the
 * phrase *command model*: this firmware has no motor constants. It does not
 * know a kV, a propeller, or the pack's internal resistance, and every one of
 * those is in the true relation between a throttle fraction and an eRPM. A
 * model built from datasheet numbers would be a model of a motor nobody here
 * has measured, and phase 3's whole acceptance story is that nothing about an
 * ESC has been measured.
 *
 * So the model is **the motor's own history**: the ratio `eRPM / command` that
 * this motor has been producing, low-passed. Nothing here knows a number about
 * a motor; it knows what this one has been doing, and flags a departure from
 * that. The consequence to keep in mind is that the model is not available in
 * the first seconds of a flight, and the module says so rather than guessing.
 *
 * The twelve decisions below are the parts of that a reader would get wrong
 * from the code alone. They are numbered so the tests can name them.
 *
 *   1. **A motor that is not turning when nothing is commanded is not a fault.**
 *      Below `AK_MOTOR_HEALTH_COMMAND_FLOOR` the motor is idle, "not turning"
 *      is the expected answer, and the health test is *not asked* rather than
 *      passed - the flag is `AK_MOTOR_HEALTH_IDLE`, which is a different claim
 *      from `OK`. Most of a fixed wing's flight is below this floor.
 *
 *   2. **The model is the motor's own gain, learned in the band.** `gain` is
 *      `eRPM / command`, fed through a first-order low-pass at
 *      `AK_MOTOR_HEALTH_GAIN_CUTOFF_HZ`. It is fed *only* while the command is
 *      at or above the floor and only by a reply that carried a real speed, so
 *      an idle motor neither learns from its own zero nor forgets what it knew.
 *      It is not trusted until it has been fed for `AK_MOTOR_HEALTH_GAIN_MS`,
 *      and until then the off-model test is unavailable rather than passed.
 *
 *      **Only a sample that agrees with the model feeds it, and that half is
 *      not optional.** A model that keeps learning while the motor is wrong is
 *      a model that agrees with the fault: at the filter's time constant a
 *      sustained error of K drags the gain to K, the ratio comes back to one,
 *      and the module clears its own flag by having adopted the thing it was
 *      supposed to be comparing against.
 *
 *      The weaker rule - keep learning until the flag is up, then freeze - is
 *      not enough, and it is worth saying why, because it is the rule a reader
 *      would arrive at first. The flag cannot be up until the window is mostly
 *      fault, which takes `AK_MOTOR_HEALTH_BUCKET_MS` x `AK_MOTOR_HEALTH_
 *      BUCKETS` x (1 - 1/K) of samples to fill - around 400 ms at the constants
 *      below. Feeding during those 400 ms drags a factor-of-2.5 fault's gain far
 *      enough that the flag never sets at all, and it leaves a factor-of-6
 *      fault's model 20 % wrong for the rest of the flight, which shows up as
 *      the *healthy* samples reading as off model once the fault clears. Both
 *      of those were measured here, by the two cases in tests/test_motor_health.c
 *      that exist for it. Refusing the sample instead of the state costs
 *      nothing and has neither failure.
 *
 *      The cost that remains is deliberate: a motor whose real gain has
 *      genuinely changed by more than the band is flagged rather than
 *      re-learned, and the way to re-learn it is `ak_motor_health_forget`.
 *
 *   3. **"Far from the model" is two-sided.** A desynchronised motor, a
 *      thrown propeller and a motor whose load has gone away all *race* - the
 *      speed goes up for the same command, not down - so the test is
 *      `ratio < 1/K` **or** `ratio > K` around the learned gain. A one-sided
 *      test would catch a stalled motor, which decision 4 catches anyway, and
 *      miss the case this module exists for.
 *
 *      **Both sides of that ratio are eRPM per unit command**, and that is not
 *      a detail: `gain` is learned in those units (eRPM divided by command),
 *      so the test has to divide the *instantaneous eRPM by the same command*
 *      before comparing. Dividing the raw eRPM by the gain instead produces
 *      `command` back - a number in [0, 1] that is smaller than 1/K at every
 *      low throttle and would call a perfectly ordinary motor off model every
 *      time it was throttled back. The test caught exactly that, with a motor
 *      at a quarter command that was doing precisely what it had been doing.
 *
 *   4. **A commanded motor reporting "not turning" is its own flag and needs no
 *      model.** `AK_DSHOT_EDT_STOPPED` is the ESC saying, in the reply's own
 *      bits, that the motor is not turning. That is a fact rather than a
 *      deviation, so it is `AK_MOTOR_HEALTH_STALLED`, and it needs no model at
 *      all - it is the one verdict available before the gain has been learned.
 *      Keeping it apart from `OFF_MODEL` is the same distinction
 *      `ak_dshot_edt.h`'s decision 11 keeps between "not turning" and "no
 *      period". It is still a verdict over a window (decision 6) and not over
 *      one sample, because a real motor reports not-turning for a few frames
 *      while it spins up and one sample of it is not a fact.
 *
 *   5. **"No speed arrived" is not "the motor is not turning".** A reply that
 *      carried a temperature, a refused frame, and no reply at all are the same
 *      thing to this module - it was told no speed - and they are counted
 *      together. What comes out is `AK_MOTOR_HEALTH_NO_TELEMETRY`, which is a
 *      claim about the *wire or the ESC's answering*, not about the motor. The
 *      wire's own error rate is `ak_dshot_edt_quality_per_10k`'s business and
 *      this module does not duplicate it. This is `ak_dshot_edt.h`'s decision 9
 *      applied again: nothing arriving is not a reading of zero.
 *
 *   6. **Every verdict but `IDLE` is a count over a window, not a sample.**
 *      `AK_MOTOR_HEALTH_BUCKET_MS` x `AK_MOTOR_HEALTH_BUCKETS` is the
 *      window, counted the way `ak_dshot_edt.h`'s quality window is counted and
 *      with the same divergence from the reference: the buckets between the
 *      last one written and the new one are cleared, so a loop that stalls does
 *      not report its own history as the present.
 *
 *   7. **A motor is not judged for the first window after it is asked to turn.**
 *      Leaving `IDLE` for the band clears the motor's window. Without this, a
 *      wing's launch is judged on the half-second *before* the motor was
 *      commanded, which is a window of idleness, and every launch would flag.
 *      The cost is that a real fault in the first half second of a launch is
 *      not flagged, and that is the right way round.
 *
 *   8. **The flag is the window's verdict, and the hysteresis is a second,
 *      lower threshold rather than a latch.** Setting at
 *      `AK_MOTOR_HEALTH_BAD_PER_10K` and clearing at
 *      `AK_MOTOR_HEALTH_CLEAR_PER_10K` gives a flag that cannot flicker within
 *      a window, without needing a place to be cleared from - and nothing in
 *      this tree has an opinion about when a motor is believed again, so a
 *      latch would be inventing one.
 *
 *   9. **When more than one kind of bad is in the window, the flag names the
 *      worst one that is over the threshold, and `STALLED` outranks
 *      `OFF_MODEL`.** A window can hold both - a motor that stalls and then
 *      starts turning slowly - and the report must not depend on which sample
 *      happened to be last. Ordering the flags by severity and taking the
 *      highest over-threshold one makes the answer a property of the window.
 *
 *  10. **A bucket is sixteen bits wide.** A motor gets one reply per DShot
 *      frame and the tree's `dshot_khz` takes 150, 300 or 600, so 600 a second
 *      per motor is the ceiling and a 50 ms bucket holds at most thirty of
 *      them - against 65535. Sixteen bits and not
 *      thirty-two is 320 bytes of state for four motors against 640, and the
 *      counter cannot wrap at any rate this tree can generate. The overflow is
 *      clamped anyway rather than allowed to wrap, because a wrapped counter
 *      would read as a *clean* window, which is the failure direction that
 *      hides.
 *
 *  11. **A run-away is not detected, deliberately.** "The motor is turning and
 *      nothing commanded it" is the one fault a pilot would most want flagged,
 *      and the twelve bits cannot carry it: an ESC driving a motor and a
 *      propeller being turned by the air produce the same reply, and this
 *      aircraft is a wing that glides with its motor off. A windmilling
 *      propeller would flag on every descent. The refusal is recorded here
 *      rather than left as a gap, so the next reader does not add the check and
 *      discover the false positive in the air.
 *
 *  12. **Nothing here runs in the flight chain yet.** Same boundary as 3.2a and
 *      3.3: nothing in this firmware receives a per-motor eRPM - reading the
 *      pin is unwritten, and phase 3's receive side has no caller outside the
 *      console - so the module is reached by no path, `--gc-sections` drops it,
 *      and the image does not move. That is measured in the evidence file and
 *      not asserted here.
 */

/* The window: ten buckets of 50 ms is half a second. Half a second because the
 * verdict is phase 13's input for motor-failure tolerance and a second is too
 * slow to be worth having, and not shorter because a single eRPM reading is
 * noisy enough that a tenth of a second of them is not yet a fact. */
#define AK_MOTOR_HEALTH_BUCKET_MS 50u
#define AK_MOTOR_HEALTH_BUCKETS 10u

/* Below this the motor is idle and is not asked (decision 1). A fraction of
 * full command, so it does not move with the output protocol's range. It is a
 * constant rather than a parameter for the reason phase 2.2 recorded: a
 * parameter is a thing a pilot tunes, and nothing here has been on a bench to
 * say what it should be. What it *does* decide is where "idle" ends, and an
 * airframe whose idle sits above it would flag - so it is set low enough to be
 * below any idle this tree configures, and that is a choice to re-take when a
 * bench says where the real idle is. */
#define AK_MOTOR_HEALTH_COMMAND_FLOOR 0.10f

/* The learned gain: fed only in the band (decision 2), low-passed at 0.1 Hz -
 * a time constant of about 1.6 seconds, slow enough not to chase the fault it
 * is being compared against and fast enough to follow a pack's sag over a
 * flight. Not trusted until it has been fed this long. */
#define AK_MOTOR_HEALTH_GAIN_CUTOFF_HZ 0.1f
#define AK_MOTOR_HEALTH_GAIN_MS 1500u

/* How far from the learned gain counts as far (decision 3). Half and double:
 * wide enough that eRPM noise and a battery's sag do not reach it, narrow
 * enough that a motor which has lost a propeller's worth of load does. */
#define AK_MOTOR_HEALTH_RATIO 2.0f

/* The window's thresholds, in parts per ten thousand of the window's samples
 * (decision 8). Parts per ten thousand because ak_dshot_edt.h's quality window
 * uses them and the two readouts should be comparable; 8000 is 80 % and 4000 is
 * 40 %, and the gap between them is the hysteresis. */
#define AK_MOTOR_HEALTH_BAD_PER_10K 8000
#define AK_MOTOR_HEALTH_CLEAR_PER_10K 4000

/* The window must hold this many samples before it will flag anything, so that
 * a window holding two replies cannot be a verdict. At DShot150 - the slowest
 * rate the tree's `dshot_khz` takes - a 50 ms bucket holds about seven, so this
 * is one bucket at the slowest rate and a quarter of one at the fastest. */
#define AK_MOTOR_HEALTH_MIN_SAMPLES 8u

/*
 * What the module has to say about one motor, most severe last. `IDLE` sits
 * second rather than last because it is the answer for most of a wing's flight
 * and a reader scanning a status line should not find it filed with the faults.
 */
typedef enum {
    AK_MOTOR_HEALTH_OK = 0,           /* asked, turning as this motor does */
    AK_MOTOR_HEALTH_IDLE = 1,         /* commanded below the floor: not asked */
    AK_MOTOR_HEALTH_NO_TELEMETRY = 2, /* asked; no speed arrived (decision 5) */
    AK_MOTOR_HEALTH_OFF_MODEL = 3,    /* asked; a speed far from its own gain */
    AK_MOTOR_HEALTH_STALLED = 4,      /* asked; the ESC reports not turning */
} ak_motor_health_flag_t;

/* One reply, as the caller's decoder saw it. The fields are exactly what
 * ak_dshot_edt_update() and ak_dshot_edt_value_t already carry, so the caller
 * is not asked to re-decide anything - the module reads the reply's own terms
 * (decision 5) and nothing else. */
typedef struct {
    uint8_t replied;              /* a reply arrived since the last call */
    ak_dshot_edt_status_t status; /* ak_dshot_edt_update()'s answer, when one did */
    ak_dshot_edt_type_t type;     /* the stored type, when it stored one */
    uint32_t erpm;                /* the speed, when the type was AK_DSHOT_EDT_ERPM */
} ak_motor_health_reply_t;

/*
 * One motor's window. Three counters per bucket rather than one bad counter,
 * because decision 9 needs to know *which* kind of bad the window holds and a
 * single total cannot answer that.
 *
 * `answers` counts replies that carried a speed: it is the denominator, and it
 * is also what tells "the motor is answering and turning as it should" from
 * "nothing is answering" (decision 5) - the second is `answers == 0` over the
 * whole window, not a low fraction of anything.
 */
typedef struct {
    uint16_t answers[AK_MOTOR_HEALTH_BUCKETS];
    uint16_t stopped[AK_MOTOR_HEALTH_BUCKETS];
    uint16_t off_model[AK_MOTOR_HEALTH_BUCKETS];
    uint32_t answer_sum;
    uint32_t stopped_sum;
    uint32_t off_model_sum;
    uint32_t bucket; /* the last bucket written, as now_ms / BUCKET_MS */
} ak_motor_health_window_t;

/*
 * One motor. `gain` is decision 2's learned ratio and `gain_valid` is whether it
 * has been fed long enough to be used - a motor that has never been in the band
 * has a gain of zero and nothing may divide by it. `last_ms` is the last call's
 * timestamp, kept per motor because the modules that would drive this one are
 * not obliged to call it for every motor on every pass.
 */
typedef struct {
    ak_motor_health_window_t window;
    ak_filter_pt1_t gain_filter;
    float gain;
    float gain_dt;   /* the interval the filter's coefficients were built for */
    uint32_t gain_ms; /* how long the filter has been fed, in milliseconds */
    uint32_t last_ms;
    ak_motor_health_flag_t flag;
    uint8_t started;  /* a first call has established an interval to measure from */
    uint8_t gain_valid;
    uint8_t gain_fed; /* the filter has been seeded, rather than sitting at zero */
    uint8_t in_band;  /* the previous call was at or above the floor (decision 7) */
} ak_motor_health_motor_t;

typedef struct {
    ak_motor_health_motor_t motor[AK_MAX_MOTORS];
    unsigned motors;
} ak_motor_health_t;

/* `motors` is clamped to AK_MAX_MOTORS; zero is a state with no motors rather
 * than an error, and every call on it answers IDLE. */
void ak_motor_health_init(ak_motor_health_t *health, unsigned motors);

/* One loop's worth of one motor. `command` is the fraction of full command the
 * mixer wrote for this motor - the same number the DShot frame was built from,
 * not a throttle the caller reconstructed. Returns this motor's flag, which is
 * also what ak_motor_health_of() reads afterwards.
 *
 * `now_ms` is the loop's own millisecond clock (ak_millis(), or the scheduler's)
 * and the interval it implies is what the gain's filter is advanced by
 * (decision 2). Two calls in the same millisecond count two samples in the
 * window and advance the filter once, which is the honest reading of an
 * interval a millisecond clock can only answer as zero. */
ak_motor_health_flag_t ak_motor_health_update(ak_motor_health_t *health,
                                              unsigned motor, float command,
                                              const ak_motor_health_reply_t *reply,
                                              uint32_t now_ms);

/* The flag the last update left, or IDLE for a motor that does not exist. */
ak_motor_health_flag_t ak_motor_health_of(const ak_motor_health_t *health,
                                          unsigned motor);

/* The learned gain, or 0 when it is not trusted yet. For a readout: it is the
 * only number this module has that a person can compare against a tachometer. */
float ak_motor_health_gain(const ak_motor_health_t *health, unsigned motor);

/* How many of the window's samples carried a speed, and how many of those were
 * bad, split the way decision 9 splits them. For a readout and for a test that
 * wants the counts rather than the verdict. */
void ak_motor_health_counts(const ak_motor_health_t *health, unsigned motor,
                            uint32_t *answers, uint32_t *stopped,
                            uint32_t *off_model);

/* Forget one motor's window and its learned gain: the aircraft has been
 * reconfigured, or a motor was replaced, and this motor's history is not
 * evidence about this motor. */
void ak_motor_health_forget(ak_motor_health_t *health, unsigned motor);

/* The name of a flag, for a readout. Never null: an out-of-range flag is
 * reported as such rather than read past the table. */
const char *ak_motor_health_flag_name(ak_motor_health_flag_t flag);

#endif /* AK_FLIGHT_MOTOR_HEALTH_H */
