#ifndef AK_FLIGHT_H
#define AK_FLIGHT_H

#include "ak_estimator.h"
#include "ak_mixer.h"
#include "ak_params.h"
#include "ak_pid.h"
#include "ak_rc.h"
#include "ak_types.h"

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
    float d_cutoff_hz;
    /* First-order low pass on the gyro, in Hz; 0 disables it. The rate loop's
     * proportional term sees this, not the driver's sample, so it is the one
     * filter that keeps the motor commands from carrying the sensor's noise.
     * The reference implementation flying this airframe runs the same 250 Hz
     * (`gyro_anti_aliasing_lpf_hz`), and the difference is that its filter is
     * a second-order one - see docs/04-flight-core.md. */
    float gyro_lpf_hz;

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
 * goes wrong, all of which land within a minute of 2^32. Note that a genuine
 * wrap is *not* caught here and must not be: 2^32 milliseconds is forty-nine
 * days and the subtraction that crosses it produces exactly the elapsed time,
 * which is the one case where the unsigned arithmetic is already right.
 */
#define AK_FLIGHT_CLOCK_MAX_MS 60000u

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
     * used to be thrown away in silence. */
    uint32_t dropped_ms;

    /* Timestamps that went backwards, and so were treated as a new clock
     * rather than as a very long gap. */
    uint32_t clock_resets;

    /* The control loop's own period: how many iterations took longer than
     * max_dt_ms, and the longest one seen. */
    uint32_t long_loops;
    uint32_t max_loop_ms;
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
    uint32_t          arm_hold_started_ms;

    /* The arming gate's own hold: how many consecutive steps the innovation has
     * been inside AK_ARM_INNOV_DEG. It lives here rather than in a file-scope
     * static because it is *this aircraft's* error - a static would be reset by
     * ak_flight_init() of any other instance, and the suite runs two aircraft
     * side by side on purpose. */
    uint32_t          arm_innov_ms;

    float        rate_setpoint[3]; /* rad/s */
    float        gyro[3];          /* rad/s, after gyro_lpf_hz */
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

    uint32_t last_imu_ms;
    int      have_last_imu;
    uint32_t last_step_ms;
    int      have_last_step;
    uint32_t steps;
    int      link_live; /* as of the last step: is the pilot's link up? */

    /* What the loop's own timing did, since ak_flight_init(). See the struct's
     * comment: this is the half of B4 that makes the other half reportable. */
    ak_flight_timing_t timing;
} ak_flight_t;

/* How many entries ak_flight_param_table() writes. */
#define AK_FLIGHT_PARAM_COUNT 33u

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
} ak_arm_block_t;

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

void ak_flight_default_config(ak_flight_config_t *cfg);
void ak_flight_init(ak_flight_t *flight, const ak_mixer_t *mixer);

/* One iteration. imu and rc are what the board collected; now_ms is the board's
 * millisecond clock. Safe to call at any rate, including when the sensors have
 * stopped producing - which is exactly the case it has to survive. */
void ak_flight_step(ak_flight_t *flight, const ak_imu_sample_t *imu,
                    const ak_rc_input_t *rc, uint32_t now_ms);

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
