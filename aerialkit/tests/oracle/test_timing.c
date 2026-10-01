/*
 * B4's baseline: what the control loop does with time it did not expect.
 *
 * The assessment's B4 asks for control timing that is observable and bounded -
 * source timestamps, sequence and age, execution duration and deadline
 * counters, and a *fallback* for a gap rather than clamping it away. Before any
 * of that is written this file measures what the loop does now, because "the
 * timing is not observable" is an adjective and this repository does not land
 * adjectives.
 *
 * What is there now is one function in ak_flight.c, `step_dt()`, and its caller:
 *
 *     static float step_dt(ak_flight_t *flight, const ak_imu_sample_t *imu)
 *     {
 *         float dt = 0.001f;
 *         if (flight->have_last_imu) {
 *             uint32_t delta = imu->time_ms - flight->last_imu_ms;
 *             if (delta > flight->cfg.max_dt_ms) {
 *                 delta = 1;              (1) the gap, clamped away
 *             }
 *             dt = (float)delta / 1000.0f;
 *             if (dt < 0.0005f) {
 *                 dt = 0.0005f;           (2) the duplicate, clamped up
 *             }
 *         }
 *         flight->last_imu_ms = imu->time_ms;
 *         flight->have_last_imu = 1;
 *         return dt;
 *     }
 *
 *     void ak_flight_step(ak_flight_t *flight, const ak_imu_sample_t *imu,
 *                         const ak_rc_input_t *rc, uint32_t now_ms)
 *     {
 *         flight->steps++;
 *         float dt = step_dt(flight, imu);        (3) before the validity gate
 *         ...
 *         if (imu->valid) {
 *             ak_estimator_update(&flight->est, &filtered, dt);
 *         }
 *
 * This file was written expecting to measure "a fifty-millisecond gap is
 * reported as one millisecond", and the first run said otherwise: the
 * comparison is a strict `>`, `max_dt_ms` defaults to 50, so a gap of exactly
 * 50 ms is believed (measured ratio 50.17) and the defect is not a bound but a
 * **cliff**. One millisecond more of gap and the time reported to the estimator
 * falls from 0.051 s to 0.001 s, with nothing in between and nothing recorded.
 * So the probe measures at the bound and one millisecond past it. Recording
 * that here rather than quietly editing the header is the point of measuring
 * first: the defect I had reasoned my way to was not the defect that is there.
 *
 * There are three distinct faults, and they are measured separately below:
 *
 *   (1) the cliff - a gap over `max_dt_ms` is reported as one millisecond, so
 *       the aircraft is told it rotated 1/51st of what it did, and the elapsed
 *       time is *discarded* rather than deferred: `last_imu_ms` is updated
 *       regardless, so the loss is permanent. A bus that is simply slow - every
 *       sample arriving over the bound - puts the estimate permanently at a
 *       fraction of the true rate (measured: 1/100th).
 *   (2) a duplicate sample becomes half a millisecond of rotation rather than
 *       none. This is the one that is wrong in the *other* direction, and it is
 *       the case the assessment names first: "test duplicate/stale samples".
 *   (3) `step_dt()` runs before the `if (imu->valid)` gate, so an *invalid*
 *       sample consumes the elapsed time and nothing integrates it. A gap that
 *       coincides with one unusable reading is lost without even reaching the
 *       clamp.
 *
 * None of the three was counted. `flight->steps` incremented per call and was
 * the only number in the core's timing state; there was no count of long gaps,
 * stale or duplicate samples, execution duration or deadline overruns, so an
 * operator whose aircraft flew badly because a bus stalled every few hundred
 * milliseconds had nothing to look at and neither had anyone reading a log
 * afterwards. That absence is the "observable" half of B4.
 *
 * The repair (B4.1) replaces the single clamped duration with a *plan*: the
 * elapsed time is integrated in pieces of at most max_dt_ms, up to a fixed
 * budget of them, so no integration step is large enough to trip the derivative
 * term - which is what the clamp was for, and it is still achieved - while the
 * whole of the elapsed time reaches the estimate. A duplicate integrates
 * nothing instead of half a millisecond. A timestamp that went backwards is
 * told apart from a long gap before the subtraction rather than after it. The
 * control law is given the *loop's* period rather than the sensor's interval,
 * because those are two clocks and the old code used one of them for both jobs.
 * And every one of those events is counted in `flight.timing`, which is what
 * sections E and H onwards below assert.
 *
 * The invariance that matters most is section A and B: a normal one-millisecond
 * loop produces *exactly* the numbers it produced before the repair, to the
 * last digit the estimator can represent. Changing the arithmetic of the
 * thousand-hertz path would have been a much larger change than B4 asked for.
 *
 * THE INSTRUMENT IS A RATIO, and it is measured on YAW. Both halves of that
 * matter:
 *
 *   - A ratio, because the estimator's own arithmetic then cancels. With a
 *     constant gyro rate, the advance across a gap of N milliseconds divided by
 *     the advance across one millisecond is N if the elapsed time was believed
 *     and 1 if it was clamped to a single millisecond. Nothing in this file
 *     needs to know how the estimator propagates a quaternion.
 *   - Yaw, because the accelerometer cannot observe heading. The estimator's
 *     complementary correction has nothing to say about this axis, so it
 *     integrates the gyro and nothing else. On roll or pitch the gravity
 *     correction pulls the estimate back toward level by an amount that depends
 *     on how long the filter has been running - which is the quantity under
 *     test, so the instrument would be measuring itself.
 *
 * This is a baseline in the shape of tests/oracle/test_config_policy.c and
 * test_config_recycle.c: it asserts the behaviour B4 is supposed to produce, so
 * it FAILS against the tree as it stands. It lives in tests/oracle/ rather than
 * tests/ because `TEST_OBJS` globs every C file in the tests directory into the
 * main test binary and a second main() would collide with it, and it is not in
 * ci.sh until the repair lands - a documented defect must not be a red suite.
 *
 * What is real here and what is a stand-in: the flight core is real
 * (ak_flight_init, the whole of ak_flight_step including step_dt, and the
 * estimator). There are no board seams in this file at all, because nothing
 * here touches a board: the question is what the core does with a timestamp,
 * and a timestamp is arithmetic.
 */

#include <math.h>
#include <stdint.h>
#include <stdio.h>

#include "ak_flight.h"
#include "ak_rc.h"

/* --- the measurements ---------------------------------------------------- */

static int failures;

static void expect(const char *name, int passed)
{
    printf("  %-6s %s\n", passed ? "ok" : "FAILED", name);
    if (!passed) {
        failures++;
    }
}

/* Degrees, for a report a person can read. */
static double deg(float rad)
{
    return (double)rad * 57.29577951308232;
}

/* --- the aircraft -------------------------------------------------------- */

static ak_flight_t flight;
static uint32_t    bound_ms;    /* the core's own max_dt_ms, not this file's */

/*
 * A fresh, level, disarmed aircraft. Nothing in the core is asked to do
 * anything except integrate the gyro: the throttle is shut, the arm switch is
 * down, the accelerometer says level. Every input but the timestamp is held
 * still, so the only thing that can move the estimate is the elapsed time the
 * core believes in - which is the quantity under test.
 */
static void reset(void)
{
    ak_flight_init(&flight, &ak_mixer_quad_x);
    ak_flight_set_board_outputs(&flight, 4u, 2u);
    bound_ms = flight.cfg.max_dt_ms;
}

/*
 * One step at time `t`, yawing at `rate_dps`, returning the estimate after it.
 * `valid` is the IMU's own validity flag, which is the assessment's "stale
 * sample": a reading the board could not use. See the header for why this axis
 * and not another.
 */
static float step_at(uint32_t t, float rate_dps, int valid)
{
    ak_imu_sample_t imu;
    ak_rc_input_t   rc;
    ak_rc_config_t  cfg;

    ak_rc_default_config(&cfg);

    imu.gyro[0] = 0.0f;
    imu.gyro[1] = 0.0f;
    imu.gyro[2] = rate_dps * 0.017453292519943295f;     /* deg/s -> rad/s */
    imu.accel[0] = 0.0f;
    imu.accel[1] = 0.0f;
    imu.accel[2] = 1.0f;
    imu.time_ms = t;
    imu.valid = valid;

    for (int i = 0; i < AK_RC_CHANNELS; i++) {
        rc.channel[i] = (uint16_t)cfg.mid;
    }
    rc.channel[AK_RC_THROTTLE] = (uint16_t)cfg.min;
    rc.channel[AK_RC_ARM] = (uint16_t)cfg.min;          /* disarmed */
    rc.channel[AK_RC_MODE] = (uint16_t)cfg.min;
    rc.last_update_ms = t;
    rc.valid = 1;

    ak_flight_step(&flight, &imu, &rc, t);
    return flight.est.yaw;
}

static float step_to(uint32_t t, float rate_dps)
{
    return step_at(t, rate_dps, 1);
}

/* --- the probe ----------------------------------------------------------- */

#define RATE_DPS 100.0f
#define PER_MS   ((double)RATE_DPS / 1000.0)

int main(void)
{
    printf("B4 baseline: what the control loop does with time it did not expect\n");
    printf("yaw at a constant %.0f deg/s, so the advance is the time believed\n",
           (double)RATE_DPS);

    /*
     * A. The control, and it is the measurement everything below is a ratio of.
     *    If this does not move, the rest of the file is a ratio of two zeroes
     *    and would report a confident 0/0 as a pass.
     */
    reset();
    printf("\nthe core's own bound, from ak_flight_init: max_dt_ms = %u\n", bound_ms);

    float t0 = step_to(1000u, RATE_DPS);
    float per_ms = step_to(1001u, RATE_DPS) - t0;

    printf("\nA. one millisecond apart\n");
    printf("   yaw advances    %.6f deg\n", deg(per_ms));
    printf("   truth           %.6f deg\n", PER_MS);
    expect("a millisecond of rotation is measured at all", per_ms > 1e-5f);
    expect("and it is the rotation a millisecond is worth",
           fabs(deg(per_ms) - PER_MS) < 0.02);

    /*
     * B. At the bound. The comparison in step_dt() is a strict `>`, so a gap of
     *    exactly max_dt_ms is still believed - the last value that is.
     */
    reset();
    float b0 = step_to(2000u, RATE_DPS);
    float b1 = step_to(2000u + bound_ms, RATE_DPS);
    double ratio_at = (double)(b1 - b0) / (double)per_ms;

    printf("\nB. a gap of exactly max_dt_ms (%u ms)\n", bound_ms);
    printf("   yaw advances    %.6f deg\n", deg(b1 - b0));
    printf("   truth           %.6f deg\n", PER_MS * (double)bound_ms);
    printf("   ratio to a 1 ms step  %.2f\n", ratio_at);
    expect("a gap at the core's own bound is believed",
           ratio_at > (double)bound_ms / 2.0);

    /*
     * C. One millisecond past it. This is the cliff: 2% more elapsed time and
     *    the estimator is told 51 times less happened.
     */
    reset();
    float c0 = step_to(3000u, RATE_DPS);
    float c1 = step_to(3000u + bound_ms + 1u, RATE_DPS);
    double ratio_over = (double)(c1 - c0) / (double)per_ms;

    printf("\nC. one millisecond past the bound (%u ms)\n", bound_ms + 1u);
    printf("   yaw advances    %.6f deg\n", deg(c1 - c0));
    printf("   truth           %.6f deg\n", PER_MS * (double)(bound_ms + 1u));
    printf("   ratio to a 1 ms step  %.2f  (%.0f if the time was believed, "
           "1 if clamped)\n", ratio_over, (double)(bound_ms + 1u));
    expect("one millisecond more of gap is not reported as one millisecond "
           "of flight",
           ratio_over > (double)bound_ms / 2.0);

    /*
     * D. A duplicate sample: the same timestamp twice, which is also what a
     *    frozen sensor looks like. Physically nothing happened between them, so
     *    the attitude must not advance. This is the assessment's "duplicate/
     *    stale samples" and it is the one fault that is wrong in the other
     *    direction - time is invented rather than discarded.
     */
    reset();
    (void)step_to(4000u, RATE_DPS);
    float d0 = flight.est.yaw;
    float d1 = step_to(4000u, RATE_DPS);
    double ratio_dup = (double)(d1 - d0) / (double)per_ms;

    printf("\nD. the same timestamp twice\n");
    printf("   yaw advances    %.6f deg\n", deg(d1 - d0));
    printf("   truth           0.000000 deg\n");
    printf("   ratio to a 1 ms step  %.2f  (0 if believed, 0.5 if clamped up)\n",
           ratio_dup);
    expect("a duplicate sample advances the attitude by nothing",
           fabs(ratio_dup) < 0.05);

    /*
     * E. An unusable reading. The old step_dt() ran before the `if (imu->valid)`
     *    gate, so this sample's timestamp was consumed and nothing integrated
     *    it: the second of flight between the two good samples was gone, and it
     *    did not even reach the clamp. Refusing to integrate a rate you do not
     *    have is still correct - an unusable reading is not evidence of
     *    anything - so the attitude does not advance. What B4 owes here is that
     *    the lost time is *counted*, and the first assertion below is the one
     *    that fails before the repair.
     */
    reset();
    float e0 = step_to(5000u, RATE_DPS);
    (void)step_at(6000u, RATE_DPS, 0);          /* unusable, one second later */
    float e1 = step_to(6001u, RATE_DPS);

    printf("\nE. an unusable sample landing inside a one-second gap\n");
    printf("   the aircraft turned   %.4f deg\n", PER_MS * 1001.0);
    printf("   yaw advances          %.4f deg\n", deg(e1 - e0));
    printf("   unusable samples      %u\n", flight.timing.unusable);
    printf("   dropped_ms            %u\n", flight.timing.dropped_ms);
    expect("the second with no usable reading is counted, not lost in silence",
           flight.timing.unusable == 1u && flight.timing.dropped_ms == 1000u);

    /*
     * F. The consequence worth reading out loud. A bus that is simply too slow
     *    - every sample arriving over the bound, the gyro valid every time, so
     *    the rotation is perfectly knowable - puts the estimate permanently at
     *    1/max_dt of the true rate. Ten samples, each bound+1 ms apart.
     */
    reset();
    float f0 = step_to(7000u, RATE_DPS);
    uint32_t at = 7000u;
    const int n = 10;

    for (int i = 0; i < n; i++) {
        at += bound_ms + 1u;
        (void)step_to(at, RATE_DPS);
    }

    double f_truth = PER_MS * (double)(bound_ms + 1u) * (double)n;
    double f_got   = deg(flight.est.yaw - f0);

    printf("\nF. a bus slower than the bound: %d samples, %u ms apart\n",
           n, bound_ms + 1u);
    printf("   the aircraft turned   %.4f deg\n", f_truth);
    printf("   yaw advances          %.4f deg\n", f_got);
    printf("   ratio to the truth    %.3f   (1.000 if every sample was believed)\n",
           f_got / f_truth);
    expect("a slow bus still reports the rotation that happened",
           f_got > f_truth / 2.0);

    /*
     * G. A timestamp that goes backwards - a clock reset, a wrap, a second
     *    source. Unsigned subtraction turns it into nearly 2^32, which is
     *    caught by the same clamp; what must not happen is the attitude running
     *    away. This one already holds, and it is here as a guard rather than as
     *    a defect: a repair for C, D and F must not lose it.
     */
    reset();
    (void)step_to(9000u, RATE_DPS);
    float g0 = flight.est.yaw;
    float g1 = step_to(8900u, RATE_DPS);

    printf("\nG. a timestamp a hundred milliseconds backwards\n");
    printf("   yaw advances    %.6f deg\n", deg(g1 - g0));
    expect("a timestamp that goes backwards does not run the attitude away",
           fabsf(g1 - g0) < 0.5f);

    /*
     * H. The observable half, which is the point of the whole struct. Before
     *    the repair this section could only print `steps`: the ten slow-bus
     *    steps below were ten, and the ten faults inside them were nothing at
     *    all. Now the loop's own account of what happened to it survives the
     *    aircraft, which is the difference between a log that can be read and
     *    one that cannot.
     */
    reset();
    {
        /* Primed, because the first sample after a reset has no predecessor and
         * is therefore never a gap: without this the ten gaps would be nine and
         * the count would be right for the wrong reason. */
        (void)step_to(1000u, RATE_DPS);

        uint32_t h_at = 1000u;
        const int h_n = 10;

        for (int i = 0; i < h_n; i++) {
            h_at += bound_ms + 1u;
            (void)step_to(h_at, RATE_DPS);
        }

        printf("\nH. what the core reports about the ten slow samples in F\n");
        printf("   loops           %u\n", flight.steps);
        printf("   gap_steps       %u  (sensor intervals past the bound)\n",
               flight.timing.gap_steps);
        printf("   catchup_steps   %u  (extra integration pieces they cost)\n",
               flight.timing.catchup_steps);
        printf("   dropped_ms      %u\n", flight.timing.dropped_ms);
        printf("   max_loop_ms     %u  (the control loop's own longest period)\n",
               flight.timing.max_loop_ms);

        expect("the ten long gaps are counted, one per step",
               flight.timing.gap_steps == (uint32_t)h_n);
        expect("and each cost one extra integration piece, not fifty",
               flight.timing.catchup_steps == (uint32_t)h_n);
        expect("and nothing inside the budget was dropped",
               flight.timing.dropped_ms == 0u);
        expect("and no duplicate or unusable sample was reported",
               flight.timing.duplicates == 0u && flight.timing.unusable == 0u);
    }

    /*
     * I. The budget, which is the "bounded" half. A gap far longer than one
     *    call may catch up on is integrated up to the budget and no further -
     *    integrating thirty seconds inside one iteration would make the
     *    iteration the stall - and the surplus is counted rather than hidden.
     *    This is the case the old clamp handled by pretending the gap was a
     *    millisecond; the difference is that nothing here is silent.
     */
    reset();
    {
        const uint32_t budget = bound_ms * 16u;     /* AK_FLIGHT_MAX_CATCHUP */
        const uint32_t jump = 30000u;               /* thirty seconds */

        (void)step_to(1000u, RATE_DPS);
        float i0 = flight.est.yaw;
        (void)step_to(1000u + jump, RATE_DPS);
        float i1 = flight.est.yaw;

        printf("\nI. a thirty-second gap, against a %u ms catch-up budget\n", budget);
        printf("   the aircraft turned   %.1f deg\n", PER_MS * (double)jump);
        printf("   yaw advances          %.1f deg  (%u ms of it)\n",
               deg(i1 - i0), budget);
        printf("   dropped_ms            %u  (the surplus, counted)\n",
               flight.timing.dropped_ms);

        expect("a long gap is integrated up to the budget, not clamped to "
               "one millisecond",
               fabs(deg(i1 - i0) - PER_MS * (double)budget) < 1.0);
        expect("and the time past the budget is counted, not silently dropped",
               flight.timing.dropped_ms == jump - budget);
    }

    /*
     * J. Each counter reaches the field it should, and none of them reaches a
     *    field it should not. The four faults are told apart because the four
     *    fallbacks are different, and a single "something went wrong" counter
     *    would not let an operator tell a slow bus from a rebooting one.
     */
    reset();
    {
        (void)step_to(1000u, RATE_DPS);
        (void)step_to(1000u, RATE_DPS);                 /* duplicate */
        (void)step_at(1001u, RATE_DPS, 0);              /* unusable */
        (void)step_to(500u, RATE_DPS);                  /* backwards */

        printf("\nJ. four faults, four counters\n");
        printf("   duplicates      %u\n", flight.timing.duplicates);
        printf("   unusable        %u\n", flight.timing.unusable);
        printf("   clock_resets    %u\n", flight.timing.clock_resets);
        printf("   gap_steps       %u\n", flight.timing.gap_steps);

        expect("a repeated timestamp counts as a duplicate",
               flight.timing.duplicates == 1u);
        expect("an invalid sample counts as unusable",
               flight.timing.unusable == 1u);
        expect("a timestamp that went backwards counts as a clock reset",
               flight.timing.clock_resets == 1u);
        expect("and none of the three is miscounted as a long gap",
               flight.timing.gap_steps == 0u);
    }

    printf("\n%d measurement(s) failed\n", failures);
    printf("%s\n", failures == 0
           ? "the control loop's timing is observable and bounded"
           : "this is the baseline, not a regression: B4 is not written yet");
    return failures == 0 ? 0 : 1;
}
