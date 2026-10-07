#ifndef AK_FLIGHT_H
#define AK_FLIGHT_H

#include "ak_estimator.h"
#include "ak_mixer.h"
#include "ak_params.h"
#include "ak_pid.h"
#include "ak_rc.h"
#include "ak_types.h"

#include "../filter/ak_dyn_notch.h"

/*
 * The flight loop. This is the whole of what runs at the loop rate:
 *
 *   rc frame + imu sample -> estimator -> control -> mixer -> outputs
 *
 * It touches no hardware and does no I/O, so the same code runs on the board,
 * in the host tests, and later in an ESP32 port.
 *
 * The states are deliberately few and the transitions deliberately dull. A
 * flight controller that surprises its owner is worse than one that does less.
 */

typedef enum {
    AK_FLIGHT_DISARMED = 0,
    AK_FLIGHT_ARMED = 1,
    AK_FLIGHT_FAILSAFE = 2, /* latched: needs the arm switch cycled off */
    AK_FLIGHT_RTH = 3,      /* the pilot's link is gone and a navigator has it */
    AK_FLIGHT_MANAGED = 4,  /* the pilot handed over and the navigator is flying */
    /*
     * The pilot's link is gone, there is no navigator to bring it home, and
     * this aircraft cannot stop: a fixed wing with the motor off and the
     * servos centred is a brick with wings. So it keeps flying - a bank it
     * holds, a nose a little down, enough throttle for the surfaces to have
     * air over them - and comes down in a circle instead of arriving in one.
     * See the wing's descend configuration in ak_flight_config_t, and
     * docs/04-flight-core.md for why this is not the quadrotor's answer.
     */
    AK_FLIGHT_DESCEND = 5,
} ak_flight_state_t;

typedef struct {
    /* Control */
    float rate_kp[3];
    float rate_ki[3];
    float rate_kd[3];
    float rate_i_limit;     /* clamp on the integral term */
    float torque_limit;     /* clamp on each axis before the mixer */

    /*
     * The gyro chain and the D-term chain (roadmap 2.2).
     *
     * Two first-order sections each, named after Betaflight's parameters so
     * that tuning knowledge transfers by name, and each chain's *first*
     * section has a cutoff that follows the throttle. The shape is
     * Betaflight's:
     *
     *     gyro  ->  LPF1 (dynamic)  ->  LPF2 (static)  ->  gyro[]
     *     D     ->  LPF1 (dynamic)  ->  LPF2 (static)  ->  the D term
     *
     * What "dynamic" means, exactly, is the reference's law and not an
     * approximation of it: `dynThrottle(t) = t * (1 - t*t/3) * 1.5`, and the
     * cutoff is `max(dynThrottle(t) * dyn_max, dyn_min)`. That curve is above
     * the identity across the whole stick range - at half throttle it answers
     * 0.6875 rather than 0.5 - so the filter opens sooner than a linear map
     * would, which is the point: the noise that has to be filtered is worst
     * at low throttle, and the phase lag is worst at high throttle where the
     * aircraft is moving fastest. Read from
     * upstream/betaflight-2026.6.1/src/main/sensors/gyro.c:670 and
     * src/main/flight/pid.c:1517 rather than from memory.
     *
     * Every stage is a PT1 - one pole and one zero - from
     * src/core/filter/ak_filter.c. **Neither chain has a type selector yet**:
     * Betaflight lets each of these four be a PT1, PT2, PT3 or SVF, and this
     * build offers the one. That is a deliberate simplification and it is
     * stated in docs/04-flight-core.md rather than left for a reader to
     * discover from a missing parameter.
     *
     * 0 disables a stage, which is also Betaflight's meaning. A disabled stage
     * passes its input through and its state stays where it is; a chain with
     * both stages at 0 is a wire, so an aircraft that wants no filtering gets
     * exactly the sample the driver read - which is what every build before
     * this one did.
     *
     * The defaults are Betaflight's 5-inch numbers, and **`gyro_lpf1` moving
     * from 0 to 250 is a change to what the aircraft flies on.** It is
     * measured, not assumed: see the closed-loop check in tests/, which is run
     * with these defaults the same way it is run with any other, and
     * docs/evidence/phase-2.2-*.txt for what it answered.
     */
    float gyro_lpf1_static_hz;
    float gyro_lpf1_dyn_min_hz;
    float gyro_lpf1_dyn_max_hz;
    float gyro_lpf2_static_hz;

    /*
     * The dynamic notch - roadmap 2.3. The peaks in the gyro's spectrum move
     * with the throttle, so a filter that is to take them out has to be told
     * where they are, and this is the configuration of the thing that finds
     * them.
     *
     * The names and the units are Betaflight's, deliberately and completely:
     * `dyn_notch_count`, `dyn_notch_q`, `dyn_notch_min_hz`, `dyn_notch_max_hz`,
     * with the same defaults (3, 300, 100, 600) and, awkwardly, the same unit
     * for Q. **A Q here is in hundredths** - 300 means Q 3 - because that is
     * what a number copied out of a Betaflight dump means to the person copying
     * it. The division happens once, in `dyn_notch_configure` below, and the
     * module itself only ever sees real Q. See decision 11 in ak_dyn_notch.h.
     *
     * `dyn_notch_count` is 0-disables, and it is a *count*: the module finds up
     * to that many peaks per axis and puts a notch on each. It is not the same
     * thing as the gate - a count above zero on a loop under 2 kHz finds
     * nothing and filters nothing, and the console says which of the two it is.
     *
     * The band is the reference's own default band. Below `min` is where a
     * control loop's own motion lives and above `max` is above every motor
     * fundamental this size of aircraft has; the transform zooms to fit
     * whatever is asked for here, so widening it costs resolution.
     */
    uint32_t dyn_notch_count;
    float    dyn_notch_q;
    float    dyn_notch_min_hz;
    float    dyn_notch_max_hz;

    float dterm_lpf1_static_hz;
    float dterm_lpf1_dyn_min_hz;
    float dterm_lpf1_dyn_max_hz;
    float dterm_lpf2_static_hz;

    /* First-order low pass on the accelerometer, in Hz; 0 disables it. It feeds
     * the ARMING GATE's innovation and nothing else - a second, separate copy of
     * the sample that no other consumer reads.
     *
     * Why it is here at all. The gate asks whether the sample's DIRECTION is
     * near the estimate's, and a lateral specific force d moves that direction
     * by atan(d) - first order, and independent of tilt - while moving the
     * sample's MAGNITUDE by sqrt(1+d^2)-1, which is second order. At 0.2 g the
     * direction is off by 11.3 degrees, more than five times the gate's own
     * threshold, while the magnitude has moved by 2 percent. So a vibration the
     * estimator does not care about at all is enough to hold the gate shut, and
     * measured on this tree it does: at 0.2 g and 25 Hz the gate refused 7650 of
     * 9000 settled samples.
     *
     * Why not the estimator's sample. The estimator is not corrupted by
     * vibration - its settled error under 0.2 g of oscillation measures lower
     * than the clean cell's - because it corrects toward the measured gravity
     * DIRECTION, which a perpendicular force does not change much. Filtering for
     * its sake would buy nothing and would move the convergence every other
     * measurement in this tree is calibrated against. Only the gate needs it.
     *
     * Why a filter is legitimate on a gate that must also refuse a genuinely
     * wrong attitude: a one-pole passes a constant unchanged. A mis-levelled
     * aircraft is a constant offset, so it produces the same innovation it did
     * before. What the filter removes is the part of the sample that is moving,
     * and the gate was never asked to measure a rate. */
    float arm_accel_lpf_hz;

    float angle_kp[2];      /* (rad/s) per rad of angle error */
    float angle_rate_limit; /* clamp on the angle loop's rate output */
    float max_rate_dps;     /* deg/s at full stick, rate mode */
    float max_tilt_deg;     /* clamp on the commanded angle, degrees */
    float yaw_rate_dps;     /* deg/s at full rudder */

    /* Arming and safety */
    float    throttle_low;  /* above this the arm switch will not arm */
    /* How far from level the aircraft may be and still be armed, in degrees,
     * measured as the tilt of the estimate from straight up. 180 is the whole
     * sky and disables the check, which is what Betaflight's own arithmetic
     * does with its small_angle. See ak_flight_arm_check(). */
    float    arm_max_tilt_deg;
    uint32_t arm_hold_ms;   /* switch on this long, throttle low, then armed */
    uint32_t rc_timeout_ms; /* no RC frame for this long is a failsafe */
    uint32_t max_dt_ms;     /* the longest interval one integration may span.
                             * A longer gap is not discarded - it is integrated
                             * in pieces of at most this, up to a bounded budget
                             * - see ak_flight_timing_t. */

    /*
     * The fixed wing's failsafe: a descending circle, flown by the attitude
     * loop, for the case the branch in ak_flight_step() describes. All three
     * are the airframe's to know and a pilot's to change, so they are settings
     * rather than constants - the bank is the circle's diameter, the pitch is
     * how fast it comes down, and the throttle is what keeps air over the
     * elevons, which is the difference between this and a stall.
     */
    float wing_descend_bank_deg;    /* how far it banks, degrees */
    float wing_descend_pitch_deg;   /* and how far nose-down, degrees */
    float wing_descend_throttle;    /* motor, as a fraction of full */
} ak_flight_config_t;

/*
 * The control rate, and it is the core's number rather than the port's.
 *
 * ak_flight_step() runs the rate loop once per call, so the interval the PID's
 * integral and derivative terms are given must be *this* period and not the
 * interval between gyro samples. Those are two different clocks and conflating
 * them is what made the old step_dt() wrong in both directions at once; see the
 * comment on plan_step(). main.c's loop gate and this constant have to agree,
 * and now main.c uses this one.
 */
#define AK_FLIGHT_LOOP_MS 1u

/* The same period in the unit the core actually counts in, because phase 1.5
 * moved the flight path's clock to microseconds - see ak_imu_sample_t.time_us
 * and ak_flight_step(). A millisecond cannot express the thing phase 1.4 is
 * for: at a four-kilohertz PID the loop's own period is 250 us, and an interval
 * that can only be 0 or 1 has no way to say so.
 *
 * Since 1.4 this pair is the *default* rather than the period: the loop's rate
 * is `gyro_rate_hz` over `pid_denom`, both of them parameters, and the live
 * value is `ak_flight_t.loop_period_us`. The constants survive because a
 * flight core that has not been told a rate has to have one, and one kilohertz
 * is what this firmware ran at before it could be told. */
#define AK_FLIGHT_LOOP_US (AK_FLIGHT_LOOP_MS * 1000u)

/* The narrowest and widest nominal period this firmware will hold, in
 * microseconds - 16 kHz and 31.25 Hz.
 *
 * The floor is a floor on the *arithmetic* rather than on any part: the loop's
 * period is the gyro's interval times `pid_denom`, and the IPC between the
 * control law, the outputs and the console all have to fit inside it. The
 * ceiling is the same argument from the other end - a loop slower than this is
 * an aircraft that is not being flown. Both ends are clamped rather than
 * refused here because this is the last line before a division, and the
 * parameter table is where a person is told: see `pid_denom`'s range in
 * main.c. */
#define AK_FLIGHT_LOOP_MIN_US 62u
#define AK_FLIGHT_LOOP_MAX_US 32000u

/*
 * The longest interval one integration step may span, in pieces, and the most
 * pieces one call may spend catching up.
 *
 * A gap is *not* clamped away - the aircraft really did fly through it and the
 * gyro was working the whole time, so the rotation is knowable and the estimate
 * must include it. What is bounded is the size of each integration step, so no
 * single step is large enough to trip the derivative term or to alias a fast
 * rotation into a slow one. A gap longer than the budget is integrated up to
 * the budget and the surplus is *counted* in dropped_ms, which is the
 * difference between this and the clamp it replaces: what the loop could not
 * use, it reports.
 */
#define AK_FLIGHT_MAX_CATCHUP 16u

/*
 * A timestamp this far in the future, or this far in the past, is not a gap -
 * it is a different clock: a counter that wrapped, a reboot, a second source
 * with its own epoch. Unsigned subtraction reports a hundred milliseconds
 * backwards as nearly 2^32 milliseconds forwards, so the two cases have to be
 * told apart *before* the arithmetic rather than after it.
 *
 * A minute, because it has to be longer than any stall worth integrating - a
 * thirty-second gap is a real thing that happened to a real aircraft and the
 * gyro was working the whole time - and shorter than any of the ways a clock
 * goes wrong, all of which land within a minute of 2^32.
 *
 * This bound moved from milliseconds to microseconds with the rest of the
 * flight path in phase 1.5, and the wrap arithmetic that goes with it changed
 * character, so it is worth stating rather than leaving to be re-derived:
 *
 *   - The old millisecond form sat far below the wrap. 2^32 ms is forty-nine
 *     days, so the subtraction that crossed it produced exactly the elapsed
 *     time and a genuine wrap was *correct*, which is why the comment here used
 *     to say a wrap must not be caught.
 *   - 2^32 us is **71.6 minutes**, so at this resolution the wrap is inside the
 *     range an aircraft can actually fly. A sample taken across it subtracts to
 *     nearly 2^32, which is far past this bound, and is therefore counted as a
 *     clock reset: one sample is not integrated and `clock_resets` goes up by
 *     one. That is the honest answer - the interval is not representable - and
 *     it is one sample in 4.3 million at a one-kilohertz loop, so it costs
 *     nothing. What it must not do is fall through as a gap, which is what a
 *     bound anywhere near 2^32 us would do.
 *
 * The bound is a minute rather than, say, ten seconds for the same reason as
 * before: a real stall is real flying and wants integrating.
 */
#define AK_FLIGHT_CLOCK_MAX_US 60000000u

/*
 * Everything the loop knows about its own timing, counted.
 *
 * The point of this struct is that "the timing is not observable" stops being a
 * sentence in a review and becomes a number in a `status` read. Before it, the
 * whole of the core's timing state was `last_imu_ms` and `have_last_imu`: an
 * operator whose aircraft flew badly because a bus stalled every few hundred
 * milliseconds had nothing to look at, and neither had anyone reading a log
 * afterwards.
 *
 * Every field is a count since ak_flight_init() and every one of them is
 * reachable from the flight struct, so a port can report them without the core
 * having to grow a console command for each.
 */
typedef struct {
    /* The intervals the loop was handed.
     *
     * gap_steps is how many iterations found a sensor interval longer than
     * max_dt_ms, and catchup_steps is the extra integration pieces those cost -
     * so a loop coping with a slow bus shows gap_steps climbing and
     * catchup_steps climbing with it, while one that is merely being *called*
     * slowly shows long_loops instead. Reading the two apart is the whole
     * reason they are separate counters.
     *
     * The iteration count itself is ak_flight_t's own `steps`, which the
     * console and the log cadence already read; counting it twice would be two
     * sources of truth for one number. */
    uint32_t gap_steps;
    uint32_t catchup_steps;

    /* Samples that were not new. duplicates is the same timestamp twice - a
     * frozen sensor, or a loop polling faster than its gyro. unusable is a
     * reading the board itself marked invalid, whose elapsed time is still
     * consumed and counted here because there is no rate to integrate. */
    uint32_t duplicates;
    uint32_t unusable;

    /* Time that could not be integrated: the surplus past the catch-up budget,
     * and the whole interval of every unusable sample. This is the number that
     * used to be thrown away in silence.
     *
     * Microseconds, like everything else the core now counts in. The console
     * and the Python ABI both still *print* it in milliseconds, because the
     * number they have always shown was in milliseconds and a reader comparing
     * two logs should not have to notice which firmware wrote them; the
     * division happens at those two boundaries rather than here, so the
     * accumulator does not lose a millisecond off every counted event. */
    uint32_t dropped_us;

    /* Timestamps that went backwards, and so were treated as a new clock
     * rather than as a very long gap. */
    uint32_t clock_resets;

    /* The control loop's own period: how many iterations took longer than
     * max_dt_ms, and the longest one seen. Microseconds, for the same reason
     * `dropped_us` is, and reported in milliseconds at the same two boundaries.
     *
     * This is the coarse health number, not the fine one. What the phase 1
     * acceptance test measures - p99 jitter against a nominal period - is
     * ak_perf's, which brackets the loop from the port's own counter and has
     * had microsecond resolution since phase 0.1. This counts iterations the
     * *core* can see were late. */
    uint32_t long_loops;
    uint32_t max_loop_us;
} ak_flight_timing_t;

typedef struct {
    ak_flight_config_t cfg;
    ak_rc_config_t     rc_cfg;
    ak_estimator_t     est;
    ak_pid_t           rate[3];
    const ak_mixer_t  *mixer;
    uint32_t           airframe; /* 0 quad-x, 1 elevon-wing */

    /*
     * What this airframe's mix needs, and what the board says it drives.
     *
     * The first two are counted from the mixer whenever the airframe changes,
     * so they cannot disagree with the table that is actually flying. The second
     * pair arrives from the board once, through `ak_flight_set_board_outputs`,
     * and until it does they are zero *and* unknown - which is not the same
     * answer as "a board with no outputs": an aircraft that cannot say what it
     * drives does not arm, because the one thing this check exists to prevent is
     * a wing flying a quadrotor's mix, and a check that a board can skip by not
     * answering is not a check.
     */
    uint8_t            needed_motors;
    uint8_t            needed_servos;
    uint8_t            board_motors;
    uint8_t            board_servos;
    uint8_t            board_outputs_known;

    /* Guidance from a navigator. With a live link it is used only when the
     * pilot has handed over - managed, below - and without one it is used
     * whenever it is there, which is the failsafe. Null means there is none and
     * a failed link stops the aircraft, which is the behaviour to have until a
     * return-to-home has been tested. The flight core knows nothing about GPS:
     * whoever owns that sets this, or sets nothing. */
    const ak_rc_command_t *guidance;

    /* The pilot has handed the aircraft to the navigator on purpose - a
     * mission, a loiter, anything that is not a failsafe. This is the only
     * thing that lets guidance override a live link, and it is the pilot's to
     * set: the flight core never decides it for itself. */
    int managed;

    /*
     * The aircraft is on the ground, as far as the measurements go.
     *
     * The core does not measure that: the navigator's descent is the only thing
     * that lands an aircraft in this firmware, and whoever runs the altitude
     * estimate is who can tell - so this is an input, like the guidance
     * pointer. The core has exactly one rule about it, and it is the narrowest
     * one it could be: an aircraft the navigator is flying, which has landed,
     * has finished flying, and its motors stop. A pilot holding the sticks
     * never sees this rule at all.
     */
    int landed;

    /*
     * The turn the airframe is already making, in radians a second.
     *
     * A wing that banks *turns*, and its yaw axis - differential thrust, on
     * this one - is the only thing that can either help or fight it. A yaw
     * rate setpoint of zero therefore asks the motors to hold the nose still
     * while the aircraft goes round a corner, which pegs them at opposite ends
     * of their range for the whole turn and, once a plant models what
     * differential thrust actually does, stops the aircraft turning far enough
     * to come home at all. The core cannot compute this: it is g tan(bank) over
     * the speed, and the speed is the GPS's business - so it is an input, the
     * way the guidance pointer and `landed` are, and the core adds it to the
     * yaw setpoint. Zero on an airframe whose yaw has nothing to do with its
     * bank, which is every quadrotor.
     */
    float yaw_rate_ff;

    ak_flight_state_t state;
    /* The command the last step decoded from the receiver. Kept because it is
     * the only place that knows whether the pilot's sticks are asking for an
     * angle or a rate, and the telemetry the handset sees says which - the
     * state alone cannot (armed is armed in either mode). */
    ak_rc_command_t   cmd;
    int               arm_hold_active;
    /* Whether the arm switch has been seen *off*, on a live link, since boot
     * or since the aircraft last armed. Arming wants an edge, not a level: a
     * board that powers up with the switch already on, or a return that lands
     * and disarms with the switch still on, must not arm itself half a second
     * later. Betaflight's ARMING_DISABLED_ARM_SWITCH is the same rule. */
    int               arm_released;
    /* The first moment of the current run of unusable IMU samples, and whether
     * there is one. One bad read is held through on the last good rate; a run
     * longer than AK_IMU_LOSS_MS is a lost sensor. */
    int               imu_bad;
    uint32_t          imu_bad_since_ms;
    uint32_t          arm_hold_started_ms;

    /* The arming gate's own hold: how long, in microseconds of *accounted-for
     * sensor time*, the innovation has been continuously inside
     * AK_ARM_INNOV_DEG. It lives here rather than in a file-scope static
     * because it is *this aircraft's* error - a static would be reset by
     * ak_flight_init() of any other instance, and the suite runs two aircraft
     * side by side on purpose.
     *
     * It was `arm_innov_ms` and counted *steps* until phase 1.4, which made the
     * gate's deadline a function of the loop's rate rather than of time: at a
     * four-kilohertz PID the same one-millisecond deadline would have been four
     * steps instead of one, and the gate would have opened four times sooner -
     * silently, because nothing about the aircraft changed. The deadline is a
     * millisecond of settling and is now accumulated as one. */
    uint32_t          arm_innov_us;

    float        rate_setpoint[3]; /* rad/s */

    /* The gyro chain's state, three axes on each of two sections. Here rather
     * than in the config because a filter's state is not a tunable: it is what
     * the aircraft has been reading, and it is reset when the estimator is,
     * not when somebody changes a cutoff. The coefficients stay in sync with
     * the config through `gyro_chain_update`, which moves them once per
     * integration piece - the rate a dynamic cutoff has to be re-derived at,
     * because it depends on the throttle of the piece being filtered. */
    ak_filter_pt1_t gyro_lpf1[3];
    ak_filter_pt1_t gyro_lpf2[3];
    /* What those coefficients are currently built for. Compared rather than
     * assumed, so the per-piece coefficient update costs three float compares
     * on the ordinary path instead of three sine-and-cosine pairs: the second
     * section's cutoff is static, and the sample interval only moves when the
     * plan subdivides. */
    float        gyro_chain_lpf1_hz;
    float        gyro_chain_lpf2_hz;
    float        gyro_chain_dt;
    float        gyro[3];          /* rad/s, after both gyro chain sections */

    /*
     * The dynamic notch, whole: the measurement, the tracked centres, and the
     * bank of biquads that are built from them. Roadmap 2.3.
     *
     * It is one member rather than several because it is one thing - see
     * ak_dyn_notch.h. Its slow half runs from a scheduler task in the slow
     * tier; its fast half, `ak_dyn_notch_filter`, runs here in the chain above
     * `gyro_lpf1`, on the *raw* sample, which is the ordering decision 2.3
     * makes and `ak_flight_step` states at the call site.
     *
     * `dyn_notch_cfg_*` is what the four parameters were the last time the
     * module was configured. Compared rather than re-applied every sample, for
     * the same reason the chain's three fields exist: `ak_dyn_notch_init`
     * defines every buffer and every filter's state, so calling it on a whim
     * would throw away a window the transform was half way through.
     */
    ak_dyn_notch_t dyn_notch;
    uint32_t     dyn_notch_cfg_count;
    float        dyn_notch_cfg_q;
    float        dyn_notch_cfg_min_hz;
    float        dyn_notch_cfg_max_hz;
    float        dyn_notch_cfg_loop_hz;

    /* The arming gate's filter, on the library rather than on the one-pole
     * solved form it used before roadmap 2.2 - the last user of that form. It
     * is still one pole and still per sample rather than per piece. */
    ak_filter_pt1_t arm_accel_lpf;
    float        arm_accel_lpf_dt;
    /* The gate's own accelerometer, after arm_accel_lpf_hz. Separate from the
     * sample the estimator is given on purpose: the two consumers want
     * different things from the same reading, and giving the estimator the
     * filtered copy would move its convergence for no benefit - see the config
     * field's comment. `arm_accel_primed` exists because the filter's DC value
     * is 1 g, not 0, so a filter started from zero would spend its first few
     * milliseconds reporting a direction that is not the aircraft's. */
    float        arm_accel[3];     /* g, after arm_accel_lpf_hz */
    int          arm_accel_primed;
    float        torque[3];        /* -1..1 per axis */
    ak_outputs_t out;

    uint32_t last_imu_us;
    int      have_last_imu;
    /*
     * Whether the sample handed to the last ak_flight_step() was one the board
     * could use. The step has always refused to arm on an unusable sample - it
     * returns before the arm logic - but ak_flight_arm_check(), which the
     * console and the preflight ask "would it arm?", did not know: an IMU that
     * answered at boot, let the estimate converge and then stopped left the
     * check saying yes while the step said no. Kept here so the gate and the
     * step read the same fact. Zero until a step has seen a usable sample.
     */
    int      imu_valid;
    uint32_t last_step_us;
    int      have_last_step;
    /* The interval the last step was given, in microseconds. Kept because the
     * port has a use for it that the core cannot make: main.c feeds the
     * estimator's GPS heading aid once per pass, and the interval that aid
     * should be told is this one. It used to be told AK_FLIGHT_LOOP_MS - the
     * nominal - which was defensible while the loop was a millisecond and is
     * not once the loop's rate is a thing the firmware is allowed to change. */
    uint32_t last_loop_us;
    /* The period the caller is running this at, in microseconds - phase 1.4's
     * `pid_denom` over the gyro's own rate. It is not the interval between
     * samples and not the interval this step is handed: it is the *nominal*
     * period, used in exactly two places, both of them places where a nominal
     * is the honest answer - the bound on one integration step when the
     * configuration has somehow been given a zero, and what the loop's timing
     * is measured against. Every interval that reaches the estimator still
     * comes from the sample's own timestamp. */
    uint32_t loop_period_us;
    uint32_t steps;
    int      link_live; /* as of the last step: is the pilot's link up? */

    /* What the loop's own timing did, since ak_flight_init(). See the struct's
     * comment: this is the half of B4 that makes the other half reportable. */
    ak_flight_timing_t timing;
} ak_flight_t;

/* How many entries ak_flight_param_table() writes. It is a count the host
 * suite checks against the table itself, so adding a parameter without moving
 * this is a failing test rather than a table that quietly got longer. */
#define AK_FLIGHT_PARAM_COUNT 45u

/*
 * Why an arm request would not be honoured.
 *
 * The gates are the ones the reference implementations have, in the order a
 * pilot can do something about them, and there is one function so the gate, the
 * console's answer and the preflight report cannot disagree about whether the
 * aircraft would arm. Betaflight keeps the same list as arming-disable flags
 * (`ARMING_DISABLED_THROTTLE` and `ARMING_DISABLED_ANGLE`, fc/core.c) and INAV
 * as `ARMING_DISABLED_NOT_LEVEL`, `_RC_LINK`, `_THROTTLE`.
 */
typedef enum {
    AK_ARM_OK = 0,
    /* The board does not drive what this airframe's mix needs: a wing's two
     * motors and two elevons on a board with two outputs and no servos, or a
     * quadrotor's four motors on one with two. It is first in the list because
     * it is the only one a pilot cannot act on - the others are switches and
     * sticks, and this one is a build. */
    AK_ARM_OUTPUTS,
    AK_ARM_NO_LINK,      /* no receiver, or none that has spoken recently */
    AK_ARM_FAILSAFE,     /* latched: the arm switch has to be cycled */
    AK_ARM_NOT_REQUESTED,/* the link is up and the arm channel is low */
    AK_ARM_THROTTLE,     /* the throttle stick is not down */
    AK_ARM_NOT_CONVERGED,/* the attitude estimate has not seen the accelerometer */
    AK_ARM_NOT_LEVEL,    /* tilted further than arm_max_tilt_deg */
    /* The last sample the core was handed was not one it could use: no IMU,
     * or one that has stopped answering. Checked second, after the outputs -
     * it is hardware, not a switch - and appended here rather than placed in
     * that order because these numbers are the gate's identity and a later
     * build must not renumber them. */
    AK_ARM_NO_IMU,
    /* The arm switch is on, and has not been seen off since boot or since the
     * aircraft last armed: arming needs the switch to go off and on again.
     * Appended, for the same reason as the one above. */
    AK_ARM_SWITCH_HELD,
} ak_arm_block_t;

/* How long a run of unusable IMU samples is held through, armed, on the last
 * good rate before it is a lost sensor and the aircraft fails safe. One bad
 * transaction on a bus is not a lost attitude; a tenth of a second of them
 * is. */
#define AK_IMU_LOSS_MS 100u

/*
 * `detail` (may be null) is the number behind the answer: the throttle, where
 * the throttle is the blocker, and the tilt in degrees, where the tilt is.
 */
ak_arm_block_t ak_flight_arm_check(const ak_flight_t *flight,
                                   const ak_rc_command_t *cmd, float *detail);

/*
 * What the board's outputs actually are - how many motors and how many servos
 * it drives - from the board, once, after its outputs are up.
 *
 * It is the flight core's business rather than the board's because the question
 * is a comparison: the mixer says what the airframe needs and the board says
 * what it has, and the core is the only place that knows both. A board that
 * never says leaves the aircraft unable to arm, which is the right direction for
 * a value nobody has stated - see the fields above.
 */
void ak_flight_set_board_outputs(ak_flight_t *flight, unsigned motors,
                                 unsigned servos);

/*
 * Tell the core the period its caller is running it at, in microseconds -
 * phase 1.4's `gyro_rate_hz` over `pid_denom`.
 *
 * The core does not schedule anything: `ak_flight_step` is called by whatever
 * the port drives it with, and it works this out for itself from the samples'
 * timestamps. What it cannot work out is the *nominal*, and there are exactly
 * two places that want one - the bound on a single integration step when the
 * configuration has somehow been given a zero, and what the loop's observed
 * period is measured against. Both are places where the truth is "what the
 * loop is set to run at", and only the caller knows that.
 *
 * A period outside AK_FLIGHT_LOOP_MIN_US..AK_FLIGHT_LOOP_MAX_US is not refused
 * here and is not silently accepted either: it is clamped, and the caller is
 * the one that gets to complain about it, because the caller is the one with a
 * parameter on it. `main.c` refuses at the parameter table, which is where a
 * person can read the reason.
 */
void ak_flight_set_loop_period_us(ak_flight_t *flight, uint32_t period_us);

/*
 * The throttle curve a dynamic low pass follows, and the cutoff it resolves
 * to. Exported because the two chains share them and because a host test can
 * then hold the curve to Betaflight's own values rather than to a copy of its
 * arithmetic - see docs/04-flight-core.md and tests/test_filter.c.
 */
float ak_flight_dyn_throttle(float throttle);
float ak_flight_dyn_cutoff(float throttle, float dyn_min_hz, float dyn_max_hz);
/* The whole rule, including which of the static and dynamic parameters is in
 * charge. This is what both chains call. */
float ak_flight_chain_cutoff(float throttle, float static_hz, float dyn_min_hz,
                             float dyn_max_hz);

/*
 * Which of the six chain cutoffs this loop rate cannot give the number that was
 * asked for, and what it gives instead.
 *
 * The library clamps a first-order cutoff to a quarter of the sample rate
 * (`AK_FILTER_CUTOFF_MAX_RATIO`) and reports that it did, and until this
 * existed nothing in the firmware read that report. It mattered the moment
 * roadmap 2.2 gave the chains Betaflight's 5-inch defaults: `gyro_lpf2_static_hz`
 * and `gyro_lpf1_dyn_max_hz` are both 500, and 500 Hz needs a loop running at
 * 2 kHz or faster. The F405 runs its gyro at 1600 Hz with `pid_denom` 1, so the
 * floor is 625 us and the ceiling is 400 Hz, and both of those parameters flew
 * as 400. On the simulator's 1 kHz loop the ceiling is 250 and they flew as
 * 250. Neither said anything: a parameter named 500 that is a 400 Hz filter is
 * a number in a document that does not describe the aircraft.
 *
 * So the flight core answers the question rather than a maintainer remembering
 * to. `out` is filled with one entry per cutoff *in force* - the dynamic pair
 * when a stage's `dyn_min_hz` is above zero, its `static_hz` otherwise, which
 * is `ak_flight_chain_cutoff`'s own rule - and only the ones that were moved.
 * A configuration the loop rate can express in full fills nothing and returns
 * zero, which is the ordinary answer on a quarter-of-a-megahertz loop and the
 * one a caller can print nothing for.
 *
 * The return is the number of entries *found*, not the number written: a caller
 * whose array is smaller than that is told so rather than shown a short list
 * that reads like a complete one - `ak_params_overflow` answers the same
 * question about the parameter table the same way. Passing `out` as null with
 * `max` of zero counts without filling anything.
 *
 * The interval used is `loop_period_us`, the nominal - a cutoff in a
 * configuration has to mean a frequency rather than the loop's jitter, and this
 * is a question about a configuration. The D-term chain is the one place that
 * is not exactly true: `ak_pid_update` builds its coefficients for the
 * *measured* interval of each sample, so its own clamp moves with the loop's
 * jitter by that same fraction and this reports the nominal it moves around.
 * The gyro chain does not - it is built for the nominal - and the difference is
 * recorded in docs/04-flight-core.md rather than papered over here.
 *
 * Read-only, no allocation, safe to call before the aircraft has ever been
 * stepped: a flight that has not run has `loop_period_us` at its default and
 * answers about the defaults.
 */
typedef struct {
    /* The parameter's own name, as the table spells it. */
    const char *name;
    /* What the configuration asked for, in Hz. */
    float requested_hz;
    /* What this loop rate gives it, in Hz. Equal to `requested_hz` when the
     * entry was not clamped, and the ceiling when it was. */
    float applied_hz;
} ak_flight_filter_clamp_t;

unsigned ak_flight_filter_clamps(const ak_flight_t *flight,
                                 ak_flight_filter_clamp_t *out,
                                 unsigned max);

/* How many cutoffs a chain can report - four parameters when every stage is
 * static, six when both dynamic pairs are in charge. The caller's array wants
 * this many entries to never be told about an overflow. */
#define AK_FLIGHT_FILTER_CLAMP_MAX 6u

/*
 * What the dynamic notch is doing, which is a question with two answers that
 * both have to be sayable.
 *
 * A notch bank that is running is a set of derived numbers the configuration
 * does not name - the analysis rate, the bin width, the band the search was
 * actually given after clamping - and every one of them is the kind of number
 * this project refuses to leave in a struct nobody reads. A bank that is *not*
 * running is worse: at the 1 kHz loop every board in this tree boots at it is
 * off, and a parameter named `dyn_notch_count` sitting at 3 while nothing is
 * notched is a document describing a different aircraft.
 *
 * So both are readable. `off` says which of the four states the module is in -
 * see ak_dyn_notch_off_t, including the AK_DYN_NOTCH_OFF_COUNT this layer sets
 * and the module never does, because turning the notch off by configuration is
 * a thing only the configuration knows.
 *
 * `min_hz` and `max_hz` are filled whenever there is a band at all - they are
 * the one part of the report a *refusal* has to name, since a crossed band is
 * refused in terms of the two numbers that crossed, and a sentence saying
 * "no band to search" without saying which band is a sentence a person cannot
 * act on. Everything else is meaningful only when `off` is
 * AK_DYN_NOTCH_RUNNING and is zeroed when it is not, so a caller that forgets
 * to branch prints zeroes rather than the last aircraft's numbers.
 *
 * `measured` is the honest edge of all of it: a bank that is running and
 * has not yet completed a window is filtering nothing, and the engaged counts
 * are how a caller tells "measuring" from "measured". It is not folded into
 * `off` because it is not a refusal - it is the first half second of every
 * flight.
 *
 * Cheap and allocation-free, and safe before the first step: a flight that has
 * never run reports the defaults and the state they resolve to. It is not
 * `const`, and that is deliberate: the call brings the module in line with the
 * configuration first (the same five compares a loop step does), because the
 * question is about what the aircraft is *configured* to do and not about what
 * the last step happened to see. A console asks it from the parameter path,
 * before any step has run at a new loop rate.
 */
typedef struct {
    ak_dyn_notch_off_t off;      /* AK_DYN_NOTCH_RUNNING when it is notching */
    uint8_t  asked;              /* dyn_notch_count, clamped to the module's max */
    uint8_t  engaged[AK_DYN_NOTCH_AXES]; /* notches a measurement put in place */
    uint8_t  measured;           /* a window has completed at least once */
    uint16_t decimation;         /* loop samples per analysis sample */
    float    q;                  /* the Q actually used, in real units */
    float    min_hz;             /* the band the search was given */
    float    max_hz;
    float    fs_hz;              /* the analysis rate, loop_hz / decimation */
    float    bin_hz;             /* what a transform bin is worth */
    uint32_t centre_clamps;      /* centres the filter library pulled down */
} ak_flight_notch_report_t;

void ak_flight_dyn_notch_report(ak_flight_t *flight,
                                ak_flight_notch_report_t *out);

/*
 * Where the notch on `axis` is actually filtering, in Hz, or 0 when none is.
 *
 * This is the blackbox's question (roadmap 2.4) and it is deliberately not the
 * report above. The report brings the module in line with the configuration
 * before answering, which is what a console asking "what is this aircraft
 * configured to do" wants and what a call made once per logged record must not
 * pay for. This one reads what the bank is *doing*: the last measurement's
 * answer. A log record is a statement about a moment, and a centre computed
 * from the configuration would be a statement about a document.
 *
 * Zero is an answer, not a placeholder. A slot with no measurement behind it is
 * *bypassed* - the sample goes through it untouched (ak_dyn_notch.h, decision
 * 7) - so an axis with nothing engaged has no centre to report, and writing the
 * last one it had would put a frequency in the log for a filter that is not
 * running.
 *
 * Slot 0 is the lowest-frequency peak the tracker kept, and the tracking
 * compacts into the low slots, so it is the freshest centre the axis has.
 */
float ak_flight_notch_centre_hz(const ak_flight_t *flight, uint8_t axis);

/* How many notches a measurement has put in place on `axis`, which is the half
 * that makes the centre above meaningful: a bank that is running and has not
 * yet completed a window has a centre of 0 and a count of 0, and one that has
 * locked on has both. They are one call and not two on purpose - the log writes
 * them side by side, and a reader that took only the first could not tell the
 * two states apart. */
unsigned ak_flight_notch_engaged(const ak_flight_t *flight, uint8_t axis);

/*
 * The blackbox record's gyro columns, filled from one sample: `gyro` as the
 * driver read it (the argument, in rad/s) and `gyro_filtered` as the chain
 * left it, both in tenths of a degree per second.
 *
 * The pair is the point. A record with only the first says what the aircraft
 * was shaken by; a record with only the second says what the controller flew
 * on and gives no way to tell a chain that removed the vibration from a
 * vibration that was never there. Written side by side in the same units, the
 * attenuation is a subtraction.
 *
 * The conversion lives here, and the logging path calls it, for one reason:
 * the test that judges the filtering *from a log* has to be measuring the same
 * arithmetic the firmware logs with. A test that wrote its own `rad2deg * 10`
 * would keep passing if main.c's column stopped being written at all, and that
 * is the failure this milestone exists to make visible.
 */
void ak_flight_log_gyro(const ak_flight_t *flight, const float gyro_raw[3],
                        int16_t gyro[3], int16_t gyro_filtered[3]);

/*
 * And the notch columns: where each axis's first engaged notch sits, in whole
 * hertz, and how many a measurement has put in place. Zero in both is a real
 * reading rather than a missing one - see ak_flight_notch_centre_hz.
 */
void ak_flight_log_notch(const ak_flight_t *flight, uint16_t notch_hz[3],
                         uint8_t notch_engaged[3]);

/*
 * And the controller itself (roadmap 4.1): the rate setpoint each loop was
 * asked to hold, and the three terms it answered with, per axis.
 *
 * `setpoint` is in tenths of a degree per second, the gyro columns' unit, so a
 * setpoint and the filtered gyro it was compared against subtract in a
 * spreadsheet. `p`, `i` and `d` are percent of the mix input - the torque
 * column's unit - so that P + I - D is the torque before its clamp; each is
 * saturated at +/-127 rather than wrapped, because a term larger than the mix
 * can take is exactly what a log of a wound-up integrator has to show. `d` is
 * the term as subtracted, kd times the filtered derivative of the measurement
 * (derivative on measurement, so a setpoint step does not kick it).
 */
void ak_flight_log_control(const ak_flight_t *flight, int16_t setpoint[3],
                           int8_t p[3], int8_t i[3], int8_t d[3]);

void ak_flight_default_config(ak_flight_config_t *cfg);
void ak_flight_init(ak_flight_t *flight, const ak_mixer_t *mixer);

/* One iteration. imu and rc are what the board collected; `now_us` is the
 * board's clock, at the same microsecond resolution as `imu->time_us` and from
 * the same counter (ak_time_us(), which every port derives as `ms * 1000 +
 * fraction`). Safe to call at any rate, including when the sensors have stopped
 * producing - which is exactly the case it has to survive.
 *
 * The two are not required to be equal: `now_us` is when the loop reached this
 * step and `imu->time_us` is when the sensor took the reading, and the interval
 * between the samples is deliberately not the interval between the calls. See
 * loop_dt() and plan_step(). Both being microseconds is what lets a four-
 * kilohertz loop state its own period at all.
 *
 * `now_ms` is the same instant in milliseconds, and it is a second argument
 * rather than a division for the reason the whole phase turns on: the two units
 * have different *range*. A 32-bit microsecond counter wraps every 71.6
 * minutes; a 32-bit millisecond counter wraps every 49 days. `now_us / 1000`
 * therefore equals the millisecond clock only until the wrap, and past it the
 * quotient is the millisecond reading modulo 71.6 minutes - so deriving a
 * millisecond deadline from a microsecond reading looks right for the first
 * hour and then fires every deadline at once. That was this function's first
 * phase-1.5 form and the simulator found it: see the loop's `gyro: 0 taken`
 * measurement in docs/29-timing.md.
 *
 * So the two rules that are genuinely millisecond rules - the receiver timeout
 * and the arm hold, both `_ms` parameters compared against a stamp the receiver
 * driver took from ak_time_ms() - are answered in milliseconds, and the
 * microsecond reading is used only for intervals the aircraft cannot be in the
 * air long enough to overflow. The caller reads both from one counter, so the
 * two cannot disagree about what time it is. */
void ak_flight_step(ak_flight_t *flight, const ak_imu_sample_t *imu,
                    const ak_rc_input_t *rc, uint32_t now_us, uint32_t now_ms);

/*
 * Where the phases inside that iteration begin.
 *
 * The firmware wants to know what each part of the loop costs, and the flight
 * core cannot tell it: no clock, no I/O, ground rule 7 - and it must not be
 * able to, because the same object is linked into the Python ABI's shared
 * library, where the profiler does not exist. So the core reports the
 * *transitions* and the port does the arithmetic: the callback is called as a
 * phase begins, and whatever is timing the loop turns that into a duration
 * using its own counter. `ak_perf_phase()` is the implementation the firmware
 * passes; see src/core/ak_perf.h.
 *
 * Three things about it are deliberate:
 *
 *  - **A phase is announced as it begins, not bracketed.** There is no "end"
 *    call to forget, which on a one-millisecond loop is a section that appears
 *    to have run for the rest of the mission.
 *  - **It costs one predictable branch per phase when no hook is set**, which
 *    is the case in the simulator, in the host tests and in the shared library.
 *    Three per iteration at 1 kHz is not a cost worth a build variant, and a
 *    measurement taken on a differently-compiled firmware is a measurement of a
 *    different firmware.
 *  - **Nothing in the flight path reads the result.** The profiler measures the
 *    control law; it takes no part in it.
 *
 * A phase is not announced on every path: the failsafe paths return before the
 * PID and the mixer, and a loop where they did not run must show them at zero
 * rather than at whatever ran instead. So the marks are inside the work, not at
 * the top of ak_flight_step(). */
typedef enum {
    AK_FLIGHT_PHASE_ESTIMATOR = 0,
    AK_FLIGHT_PHASE_PID,
    AK_FLIGHT_PHASE_MIXER
} ak_flight_phase_t;

typedef void (*ak_flight_phase_fn)(ak_flight_phase_t phase);

/* Null, which is the default, turns the marks into a single branch. */
void ak_flight_phase_hook(ak_flight_phase_fn fn);

void ak_flight_set_guidance(ak_flight_t *flight, const ak_rc_command_t *guidance);

/*
 * Hand the aircraft to the navigator, or take it back.
 *
 * With a live link the guidance is ignored unless this is set, so a navigator
 * can never fight a pilot who is flying - the rule that makes a return safe is
 * the same rule that makes a mission opt-in. Setting it while armed moves the
 * state to AK_FLIGHT_MANAGED; clearing it puts the sticks back in charge, and
 * the arm switch still disarms from either.
 */
void ak_flight_set_managed(ak_flight_t *flight, int on);

int ak_flight_managed(const ak_flight_t *flight);

/* Whether the pilot's link was up at the last step. The navigator needs to know
 * that to decide when to take over, and this is the one place that decides. */
int ak_flight_link_live(const ak_flight_t *flight);

/*
 * Sense the link: decode the frame, apply the receiver's timeout, and store the
 * answer. Every step does this, and anything that reads the link *before* the
 * step has to do it too - the navigator deciding whether to take over is the
 * reason this is callable on its own.
 *
 * It came apart without that: the navigator read the link one step behind the
 * flight core, so on the step the link went the navigator still thought it was
 * up and the core had already latched the failsafe - and the return-to-home
 * could never engage at all. `command` may be null if the caller only wants the
 * answer.
 */
int ak_flight_link_update(ak_flight_t *flight, const ak_rc_input_t *rc,
                          uint32_t now_ms, ak_rc_command_t *command);

const ak_outputs_t *ak_flight_outputs(const ak_flight_t *flight);
ak_flight_state_t   ak_flight_state(const ak_flight_t *flight);

/* The interval the last ak_flight_step() was given, in microseconds, or 0
 * before the first one. See the field's comment for why the port wants it. */
uint32_t            ak_flight_last_loop_us(const ak_flight_t *flight);

/*
 * Whether the configuration may be written to the board right now: disarmed,
 * and only disarmed.
 *
 * This is B3's armed-state policy and it is deliberately one function rather
 * than a condition each route writes for itself. The console's `save` and the
 * protocol's param save both persist, and before this they both did it in
 * every state - an aircraft could be told to write its flash with the motors
 * live. A recycled slot erases the whole config sector first, so that is a
 * tens-of-milliseconds stop in a control loop that is flying.
 *
 * The rule is here, on the flight core, because the flight core is what knows
 * the state. ak_params_save() takes the answer as an argument rather than
 * asking, so that a route cannot persist without having put the question.
 */
int ak_flight_config_writable(const ak_flight_t *flight);

/* "disarmed", "armed", "failsafe", "returning home". */
const char *ak_flight_state_name(ak_flight_state_t state);

/* Resolve flight->mixer from flight->airframe. Called at init and again after
 * a parameter change, so `set airframe 1` takes effect without a reboot. */
void ak_flight_apply_airframe(ak_flight_t *flight);

/*
 * Make everything *derived* from the configuration true again, after a
 * parameter change: the mixer, and the three rate loops' gains.
 *
 * This exists because "a parameter points straight at the field it controls"
 * was only true of the fields the control loop reads directly. Most of them it
 * does - `max_rate_dps`, the tilt limits, the angle gains and the yaw rate are
 * read out of `cfg` at the point of use - but the rate loop's six numbers per
 * axis are not: `ak_pid_init` copies them into an `ak_pid_t` once, and
 * `ak_pid_update` reads the copy. So `set rate_kp_roll 0.4` wrote a field
 * nothing flew on, and the console reported a gain the aircraft did not have.
 *
 * The copy is deliberate - `ak_pid_t` should not be reaching into the
 * configuration every iteration - so what was missing was the other half of
 * the arrangement: something that re-states the copy when the source changes.
 * That is this, and every route that changes a parameter already goes through
 * `parameters_changed()` to reach it.
 *
 * **A gain change resets the integrators**, because `ak_pid_init` is what
 * carries the gains and it also clears the accumulated state. An integral
 * accumulated under one gain is not the integral the new gain would have
 * produced, so carrying it over would be the same class of mistake as the one
 * this function fixes. The term is clamped to `rate_i_limit`, so the transient
 * is bounded.
 */
void ak_flight_apply_config(ak_flight_t *flight);

/* Fills `items` with the tunable subset of the configuration and returns how
 * many were written, or 0 if there is not room. Call it straight after
 * ak_flight_init: the values present at registration become the defaults that
 * ak_params_reset() restores. */
unsigned ak_flight_param_table(ak_flight_t *flight, ak_param_t *items,
                               unsigned max_items);

#endif /* AK_FLIGHT_H */
