#include "ak_flight.h"

#include <stddef.h>
#include <string.h>

#include "ak_math.h"
#include "ak_params.h"

/* Defined with the gyro chain further down, and declared here because
 * `ak_flight_dyn_notch_report` - which sits above it - has to bring the module
 * in line before it can say what the module is doing. */
static void dyn_notch_configure(ak_flight_t *flight);

/*
 * The throttle curve a dynamic low pass follows, and the cutoff it resolves
 * to. Both chains use these, so the gyro's filter and the D term's filter open
 * up together instead of by two curves that were meant to be one.
 *
 * The curve is Betaflight's, copied rather than approximated:
 *
 *     dynThrottle(t) = t * (1 - t*t/3) * 1.5
 *
 * from upstream/betaflight-2026.6.1/src/main/sensors/gyro.c:670. It is not the
 * identity: it is above it over the whole range (at half throttle it answers
 * 0.6875, at a quarter 0.359) and it is exactly 1 at full throttle, which are
 * the two properties that matter. Above the identity means the filters open
 * sooner than a linear map would - the noise that has to be filtered is worst
 * at low throttle, and the phase lag is worst at high throttle where the
 * aircraft is moving fastest, so the trade wants to be biased that way.
 *
 * `throttle` is clamped into [0,1] first: the curve is negative below zero and
 * turns back down above one, so an out-of-range command would *close* the
 * filters rather than pin them. The mixer clamps the throttle it is given, but
 * this is called with the command and not with the mixer's output.
 */
float ak_flight_dyn_throttle(float throttle)
{
    if (throttle <= 0.0f) {
        return 0.0f;
    }
    if (throttle >= 1.0f) {
        return 1.0f;
    }
    return throttle * (1.0f - (throttle * throttle) / 3.0f) * 1.5f;
}

/*
 * `max(dynThrottle(t) * dyn_max, dyn_min)` - the reference's own form, in the
 * branch it takes when the curve expo is zero, which is its default
 * (gyro.c:682 and pid.c:1517).
 *
 * The `max` is not decoration. Without it a low `dyn_max` would drag the
 * cutoff *below* the minimum the person asked for at low throttle, which is
 * the opposite of what a minimum means. With it the rule reads: the cutoff is
 * `dyn_min` from zero throttle up to where the curve reaches it, and rises to
 * `dyn_max` at full.
 *
 * It does **not** fall back to the static cutoff when the two dynamic bounds
 * are equal, and that is deliberate rather than an omission: Betaflight's
 * defaults set `dyn_min` equal to the static value, so equal bounds are the
 * ordinary case and resolve to the same number either way. A person who wants
 * no dynamism sets the two bounds equal and gets a fixed filter at that
 * cutoff, which is the same field doing the same job.
 */
float ak_flight_dyn_cutoff(float throttle, float dyn_min_hz, float dyn_max_hz)
{
    float opened = ak_flight_dyn_throttle(throttle) * dyn_max_hz;

    return (opened > dyn_min_hz) ? opened : dyn_min_hz;
}

/*
 * Which of the two forms a chain's first section is using, and the cutoff it
 * resolves to. This is the reference's own rule and not an interpretation of
 * it: `gyro_lpf1_dyn_min_hz > 0` means the cutoff follows the throttle, and
 * otherwise the *static* parameter is the cutoff and the two dynamic bounds
 * are ignored entirely
 * (upstream/betaflight-2026.6.1/src/main/sensors/gyro_init.c:203 and
 * src/main/flight/pid_init.c:450).
 *
 * Without that branch there is a configuration that means two different things
 * in two firmwares: `dyn_min 0` with a static cutoff of 100 asks for a fixed
 * 100 Hz filter, and a chain that resolved only the dynamic bounds would read
 * it as "off" and hand the loop an unfiltered gyro. That is the kind of
 * difference a person finds by flying it.
 *
 * The D-term chain uses the same function with its own three numbers, because
 * the reference uses the same rule for it - the two chains differ in their
 * values, not in their shape.
 */
float ak_flight_chain_cutoff(float throttle, float static_hz, float dyn_min_hz,
                             float dyn_max_hz)
{
    if (dyn_min_hz > 0.0f) {
        return ak_flight_dyn_cutoff(throttle, dyn_min_hz, dyn_max_hz);
    }
    return static_hz;
}

/*
 * One stage's cutoffs, as the parameters that are actually in charge of it.
 *
 * `ak_flight_chain_cutoff` picks the dynamic pair over the static number on
 * `dyn_min_hz > 0`, so a check of what a configuration resolves to has to make
 * the same choice - reporting `gyro_lpf1_static_hz` on a chain that is running
 * the dynamic rule would be a sentence about a parameter nothing reads.
 *
 * A dynamic pair reports both ends: the minimum is what the stage runs at on
 * the stick and the maximum is the highest it can ever reach, so clamping the
 * maximum is a statement about full throttle and clamping the minimum is one
 * about every throttle. Both are worth a sentence and they are different
 * sentences, which is why the name goes out with the number.
 */
static unsigned stage_clamps(const char *static_name, float static_hz,
                             const char *dyn_min_name, float dyn_min_hz,
                             const char *dyn_max_name, float dyn_max_hz,
                             float dt, ak_flight_filter_clamp_t *out,
                             unsigned max, unsigned *written)
{
    unsigned found = 0;

    if (dyn_min_hz > 0.0f) {
        const float ends[2] = { dyn_min_hz, dyn_max_hz };
        const char *names[2] = { dyn_min_name, dyn_max_name };
        for (int i = 0; i < 2; i++) {
            float applied = 0.0f;
            if (ak_filter_pt1_cutoff(ends[i], dt, &applied) !=
                AK_FILTER_CLAMPED) {
                continue;
            }
            if (out != NULL && *written < max) {
                out[*written].name = names[i];
                out[*written].requested_hz = ends[i];
                out[*written].applied_hz = applied;
                (*written)++;
            }
            found++;
        }
        return found;
    }

    float applied = 0.0f;
    if (ak_filter_pt1_cutoff(static_hz, dt, &applied) == AK_FILTER_CLAMPED) {
        if (out != NULL && *written < max) {
            out[*written].name = static_name;
            out[*written].requested_hz = static_hz;
            out[*written].applied_hz = applied;
            (*written)++;
        }
        found++;
    }
    return found;
}

/*
 * The dynamic notch, as the console and the protocol want to see it. See the
 * header for what each field is for and why the two answers - running and
 * refused - are both worth saying.
 *
 * The order of the three refusals is the module's own, and it is walked here in
 * the order `dyn_notch_configure` reaches them, so a report can never name a
 * reason the module did not act on: the configuration's count first, because
 * that is decided in this file and before the module is ever called; then the
 * rate gate; then the band, which the module refuses second of its two.
 *
 * The numbers below come from the module rather than being recomputed here.
 * `fs_hz` and `bin_hz` are its own decimation law applied to its own loop rate,
 * and re-deriving them in this file would be a second implementation free to
 * drift from the one that runs.
 *
 * **It configures before it answers, which is why the flight is not const.**
 * The module is brought in line by `dyn_notch_configure` on every step, and a
 * console asks this question from `apply_loop_rate` *before* the next step has
 * run - so a report read without configuring first would answer with the
 * previous loop rate's verdict. A `set gyro_rate_hz` from 1 kHz to 8 kHz would
 * print "the loop is under the gate" about a loop that is no longer under it,
 * and nothing would print again to correct it. Configuring here is cheap (five
 * compares when nothing moved) and makes the answer a function of the
 * configuration rather than of how recently the loop happened to run.
 */
void ak_flight_dyn_notch_report(ak_flight_t *flight,
                                ak_flight_notch_report_t *out)
{
    if (out == NULL) {
        return;
    }
    memset(out, 0, sizeof *out);
    if (flight == NULL) {
        out->off = AK_DYN_NOTCH_OFF_COUNT;
        return;
    }

    dyn_notch_configure(flight);

    const ak_dyn_notch_t *d = &flight->dyn_notch;
    out->asked = (uint8_t)flight->dyn_notch_cfg_count;
    out->q     = flight->dyn_notch_cfg_q / 100.0f;

    if (flight->dyn_notch_cfg_count == 0u) {
        out->off = AK_DYN_NOTCH_OFF_COUNT;
        return;
    }
    /* Set before the refusal is read, because a refusal has to name the band it
     * refused on: `dyn_notch_min_hz` 250 over `dyn_notch_max_hz` 200 is only a
     * sentence if both numbers are in it. The module stores both before either
     * of its gates, so they are the configuration's own values and not a
     * default. */
    out->min_hz = d->min_hz;
    out->max_hz = d->max_hz;

    out->off = (ak_dyn_notch_off_t)d->off;
    if (out->off != AK_DYN_NOTCH_RUNNING) {
        return;
    }

    out->decimation = d->decim_by;
    out->fs_hz      = d->fs_hz;
    out->bin_hz     = d->bin_hz;
    out->centre_clamps = d->centre_clamps;
    /* The engaged count, not `have`: a slot whose peak has since dipped under
     * the floor keeps its notch (the module's decision 8), so `have` is what
     * the last transform *found* and this is what is *filtering*. The two
     * differ exactly when a resonance is intermittent, which is when a person
     * is most likely to be reading this. */
    for (int a = 0; a < AK_DYN_NOTCH_AXES; a++) {
        out->engaged[a] = (uint8_t)ak_dyn_notch_engaged_notches(d, (uint8_t)a);
        if (out->engaged[a] > 0u) {
            out->measured = 1u;
        }
    }
}

float ak_flight_notch_centre_hz(const ak_flight_t *flight, uint8_t axis)
{
    if (flight == NULL || axis >= (uint8_t)AK_DYN_NOTCH_AXES) {
        return 0.0f;
    }
    if (ak_dyn_notch_engaged_notches(&flight->dyn_notch, axis) == 0u) {
        return 0.0f;
    }
    return flight->dyn_notch.centre_hz[axis][0];
}

unsigned ak_flight_notch_engaged(const ak_flight_t *flight, uint8_t axis)
{
    if (flight == NULL || axis >= (uint8_t)AK_DYN_NOTCH_AXES) {
        return 0u;
    }
    return ak_dyn_notch_engaged_notches(&flight->dyn_notch, axis);
}

void ak_flight_log_gyro(const ak_flight_t *flight, const float gyro_raw[3],
                        int16_t gyro[3], int16_t gyro_filtered[3])
{
    if (flight == NULL || gyro_raw == NULL || gyro == NULL ||
        gyro_filtered == NULL) {
        return;
    }

    for (int i = 0; i < 3; i++) {
        /* The driver's reading as it came in, and the same quantity after the
         * notch bank and both low-passes. Same units, same scale, one line
         * apart, which is what makes the pair subtractable. */
        gyro[i] = (int16_t)(ak_rad2deg(gyro_raw[i]) * 10.0f);
        gyro_filtered[i] = (int16_t)(ak_rad2deg(flight->gyro[i]) * 10.0f);
    }
}

void ak_flight_log_notch(const ak_flight_t *flight, uint16_t notch_hz[3],
                         uint8_t notch_engaged[3])
{
    if (flight == NULL || notch_hz == NULL || notch_engaged == NULL) {
        return;
    }

    for (int i = 0; i < 3; i++) {
        const uint8_t axis = (uint8_t)i;
        const float centre = ak_flight_notch_centre_hz(flight, axis);
        const unsigned engaged = ak_flight_notch_engaged(flight, axis);

        notch_engaged[i] = (uint8_t)engaged;
        /* Rounded to whole hertz, not truncated: a tracker's centre is a
         * smoothed continuous quantity, and truncation would put a systematic
         * half-hertz bias into the one log whose purpose here is to be compared
         * against a measured motor fundamental. A centre of zero stays zero -
         * "no notch engaged" and "a notch at 0.4 Hz" are different claims. */
        notch_hz[i] = (uint16_t)(centre + 0.5f);
    }
}

static int8_t log_percent(float term)
{
    const float pct = term * 100.0f;
    if (pct >= 127.0f) {
        return 127;
    }
    if (pct <= -127.0f) {
        return -127;
    }
    return (int8_t)(pct >= 0.0f ? pct + 0.5f : pct - 0.5f);
}

static int16_t log_ddps(float rad_s)
{
    const float ddps = ak_rad2deg(rad_s) * 10.0f;
    if (ddps >= 32767.0f) {
        return 32767;
    }
    if (ddps <= -32767.0f) {
        return -32767;
    }
    return (int16_t)(ddps >= 0.0f ? ddps + 0.5f : ddps - 0.5f);
}

void ak_flight_log_control(const ak_flight_t *flight, int16_t setpoint[3],
                           int8_t p[3], int8_t i[3], int8_t d[3])
{
    if (flight == NULL || setpoint == NULL || p == NULL || i == NULL ||
        d == NULL) {
        return;
    }
    for (int axis = 0; axis < 3; axis++) {
        const ak_pid_t *pid = &flight->rate[axis];
        setpoint[axis] = log_ddps(flight->rate_setpoint[axis]);
        p[axis] = log_percent(pid->p_term);
        i[axis] = log_percent(pid->integral);
        d[axis] = log_percent(pid->kd * pid->d_filtered);
    }
}

unsigned ak_flight_filter_clamps(const ak_flight_t *flight,
                                 ak_flight_filter_clamp_t *out, unsigned max)
{
    if (flight == NULL) {
        return 0u;
    }

    /* The nominal, not the measured interval: this is a question about a
     * configuration, and the header says why the D-term chain is the one place
     * that is an approximation. A flight that has never been stepped has the
     * default period here, which is the honest answer about the defaults. */
    const float dt = (float)flight->loop_period_us / 1000000.0f;
    const ak_flight_config_t *cfg = &flight->cfg;

    unsigned written = 0u;
    unsigned found = 0u;

    found += stage_clamps("gyro_lpf1_static_hz", cfg->gyro_lpf1_static_hz,
                          "gyro_lpf1_dyn_min_hz", cfg->gyro_lpf1_dyn_min_hz,
                          "gyro_lpf1_dyn_max_hz", cfg->gyro_lpf1_dyn_max_hz,
                          dt, out, max, &written);
    found += stage_clamps("gyro_lpf2_static_hz", cfg->gyro_lpf2_static_hz,
                          NULL, 0.0f, NULL, 0.0f, dt, out, max, &written);
    found += stage_clamps("dterm_lpf1_static_hz", cfg->dterm_lpf1_static_hz,
                          "dterm_lpf1_dyn_min_hz", cfg->dterm_lpf1_dyn_min_hz,
                          "dterm_lpf1_dyn_max_hz", cfg->dterm_lpf1_dyn_max_hz,
                          dt, out, max, &written);
    found += stage_clamps("dterm_lpf2_static_hz", cfg->dterm_lpf2_static_hz,
                          NULL, 0.0f, NULL, 0.0f, dt, out, max, &written);

    return found;
}

void ak_flight_default_config(ak_flight_config_t *cfg)
{
    /* These gains are a starting point, taking the usual shape of a 5-inch
     * quad rate loop and rescaling it for rad/s. They are *not* tuned, and
     * nothing here has flown. Expect to change every number in this function
     * on a bench with the aircraft held down. */
    cfg->rate_kp[0] = 0.25f;
    cfg->rate_kp[1] = 0.25f;
    cfg->rate_kp[2] = 0.35f;
    cfg->rate_ki[0] = 0.15f;
    cfg->rate_ki[1] = 0.15f;
    cfg->rate_ki[2] = 0.20f;
    cfg->rate_kd[0] = 0.0025f;
    cfg->rate_kd[1] = 0.0025f;
    cfg->rate_kd[2] = 0.0f;
    cfg->rate_i_limit = 0.30f;
    cfg->torque_limit = 0.60f; /* leaves room for yaw inside the mix */
    /*
     * The gyro chain and the D-term chain, at Betaflight's 5-inch defaults.
     *
     * Read from the pinned source rather than remembered, because a default is
     * the one number a person will not look up:
     * `GYRO_LPF1_DYN_MIN_HZ_DEFAULT 250`, `GYRO_LPF1_DYN_MAX_HZ_DEFAULT 500`,
     * `GYRO_LPF2_HZ_DEFAULT 500`, `gyro_lpf1_static_hz = DYN_MIN` (250) — all
     * in upstream/betaflight-2026.6.1/src/main/sensors/gyro.c:127-133 — and
     * `DTERM_LPF1_DYN_MIN_HZ_DEFAULT 75`, `DTERM_LPF1_DYN_MAX_HZ_DEFAULT 150`,
     * `DTERM_LPF2_HZ_DEFAULT 150`, with `dterm_lpf1_static_hz = DYN_MIN` (75),
     * in src/main/flight/pid.c:169-178.
     *
     * **`gyro_lpf1` moving from 0 to 250 is a change to what the aircraft
     * flies on, and it is the one thing in this milestone that is.** The
     * previous default was off, and the reason recorded against it was that a
     * 250 Hz first-order filter in the proportional path made the loop's own
     * closed-loop check hunt. That reason is a measurement and it is kept -
     * what changed is *where the number is measured*. The chain's cutoff is
     * now clamped to a quarter of the sample rate, so at the closed-loop
     * check's 250 Hz loop a "250 Hz" filter is really a 62.5 Hz one, which is
     * 4x the lag of the aircraft's 250 Hz at its own 1 kHz loop. The check has
     * been moved to the loop rate the aircraft actually runs, and the result
     * is in docs/evidence/phase-2.2-*.txt rather than asserted here.
     *
     * The D-term default is a real change too, and a smaller one: 80 Hz
     * through one pole becomes 75 Hz through two, which is more attenuation
     * and more phase lag at the same nominal number. The reference's numbers
     * are kept because they are the ones a person tuning this aircraft will
     * arrive with, and because `dterm_lpf1_dyn_min_hz` at 75 with a max of 150
     * is what makes the D term open up at high throttle - where the noise is
     * worst and where the aircraft is moving fastest.
     */
    cfg->gyro_lpf1_static_hz = 250.0f;
    cfg->gyro_lpf1_dyn_min_hz = 250.0f;
    cfg->gyro_lpf1_dyn_max_hz = 500.0f;
    cfg->gyro_lpf2_static_hz = 500.0f;

    /*
     * The dynamic notch, at the reference's defaults: 3 notches over 100-600 Hz
     * at Q 3 (`upstream/betaflight-2025.12.2/src/main/pg/dyn_notch.c` - min 100, max
     * 600, q 300, count 3).
     *
     * It is off on the bench and off in the suite, and it is on for an aircraft,
     * which is a sentence that has to be true here rather than discovered:
     * **the module refuses to run at all below a 2 kHz loop rate**, and the
     * 1 kHz AK_FLIGHT_LOOP_US the boards boot at is under it. So a default of
     * 3 changes nothing about the aircraft this tree flies today - it is a
     * configuration a person who raises `gyro_rate_hz` and lowers `pid_denom`
     * already has, waiting. The alternative was a default of 0, which would
     * mean the notch was a feature only the person who read the roadmap could
     * reach. See ak_dyn_notch.h's decision 4 for the whole argument, and
     * `dyn_notch_configure` for what the aircraft prints when the gate is what
     * is holding it off.
     */
    cfg->dyn_notch_count  = 3u;
    cfg->dyn_notch_q      = 300.0f;
    cfg->dyn_notch_min_hz = AK_DYN_NOTCH_MIN_HZ_DEFAULT;
    cfg->dyn_notch_max_hz = AK_DYN_NOTCH_MAX_HZ_DEFAULT;

    cfg->dterm_lpf1_static_hz = 75.0f;
    cfg->dterm_lpf1_dyn_min_hz = 75.0f;
    cfg->dterm_lpf1_dyn_max_hz = 150.0f;
    cfg->dterm_lpf2_static_hz = 150.0f;

    /* The arming gate's filter is AerialKit's own - Betaflight has no
     * equivalent, because its gate is a different one - so it keeps this
     * tree's name and its default. Off, because the number that makes it worth
     * turning on is a vibration amplitude, and there is no board on the bus and
     * no measured spectrum to take it from. The 50 Hz this tree's records
     * sized is where a real airframe would start, not a value anyone has
     * flown. Off means the gate reads the sample itself, which is exactly what
     * it did before this field existed. */
    cfg->arm_accel_lpf_hz = 0.0f;

    cfg->max_rate_dps = 700.0f;
    cfg->angle_kp[0] = 6.0f;
    cfg->angle_kp[1] = 6.0f;
    cfg->angle_rate_limit = 6.0f;
    cfg->max_tilt_deg = 35.0f;
    cfg->yaw_rate_dps = 350.0f;

    cfg->throttle_low = 0.05f;
    /* The reference default, and the same number: Betaflight's DEFAULT_SMALL_
     * ANGLE is 25 degrees (flight/imu.c) and INAV's small_angle defaults to
     * it. It is the check that catches an aircraft somebody has picked up, and
     * - more usefully on a bench - one whose board alignment is wrong, because
     * a board that is mounted at 40 degrees reads 40 degrees of roll while
     * sitting still, and this is the gate that refuses to fly it. */
    cfg->arm_max_tilt_deg = 25.0f;
    cfg->arm_hold_ms = 500;
    cfg->rc_timeout_ms = 250;
    cfg->max_dt_ms = 50;

    /*
     * The wing's failsafe circle. Thirty degrees of bank is a normal turn -
     * enough to come down inside a field, far from the sixty a wing can hold -
     * eight degrees of nose-down makes the descent positive without turning it
     * into a dive, and fifteen per cent of throttle is air over the elevons.
     * That last one is the whole reason this is not "stop the motor": a wing
     * with no airflow stops responding, and a wing that stops responding is
     * not landing, it is falling.
     */
    cfg->wing_descend_bank_deg = 30.0f;
    cfg->wing_descend_pitch_deg = 8.0f;
    cfg->wing_descend_throttle = 0.15f;
}

static void outputs_safe(ak_outputs_t *out)
{
    for (int i = 0; i < AK_MAX_MOTORS; i++) {
        out->motor[i] = 0.0f;
    }
    for (int i = 0; i < AK_MAX_SERVOS; i++) {
        out->servo[i] = 0.0f;
    }
}

/* Where the phases are announced. See ak_flight.h for what this is for, why it
 * is a callback rather than a compile-time arm, and why the marks sit inside
 * the work they name. */
static ak_flight_phase_fn phase_hook;

static void phase(ak_flight_phase_t which)
{
    if (phase_hook != 0) {
        phase_hook(which);
    }
}

void ak_flight_phase_hook(ak_flight_phase_fn fn)
{
    phase_hook = fn;
}

static void control_reset(ak_flight_t *flight)
{
    for (int i = 0; i < 3; i++) {
        ak_pid_reset(&flight->rate[i]);
        flight->rate_setpoint[i] = 0.0f;
        flight->torque[i] = 0.0f;
    }
}

/*
 * How far from straight up the estimate says the aircraft is, as the cosine of
 * that angle: 1 is level. Betaflight compares the same quantity against the
 * cosine of its small_angle (flight/imu.c), and the cosine is what this needs -
 * the gate is a comparison, and the angle only exists for the sentence a person
 * reads.
 */
static float cos_tilt(const ak_estimator_t *est)
{
    return ak_cosf(est->roll) * ak_cosf(est->pitch);
}

ak_arm_block_t ak_flight_arm_check(const ak_flight_t *flight,
                                   const ak_rc_command_t *cmd, float *detail)
{
    if (detail != 0) {
        *detail = 0.0f;
    }

    /* Order matters, and it is the order a pilot can act on: a receiver that
     * is not talking is not a switch that is off, and a latched failsafe is not
     * a throttle problem. */
    if (!flight->board_outputs_known || flight->board_motors < flight->needed_motors ||
        flight->board_servos < flight->needed_servos) {
        /* First, because it is the one gate a pilot cannot clear from the
         * sticks: it is a build, and arming past it is a crash with a plausible
         * number in it. */
        return AK_ARM_OUTPUTS;
    }
    if (!flight->imu_valid) {
        /* Second, for the same reason as the first: no inertial sensor is
         * hardware, not a stick. The step has always refused on this - it
         * returns before the arm logic when the sample is unusable - and this
         * is the console's and the preflight's copy of that answer, so the
         * three cannot disagree. Without it an IMU that converged the estimate
         * and then stopped answering read as "would arm". */
        return AK_ARM_NO_IMU;
    }
    if (!flight->link_live) {
        return AK_ARM_NO_LINK;
    }
    if (flight->state == AK_FLIGHT_FAILSAFE) {
        return AK_ARM_FAILSAFE;
    }
    if (cmd == 0 || !cmd->arm_request) {
        return AK_ARM_NOT_REQUESTED;
    }
    if (!flight->arm_released) {
        return AK_ARM_SWITCH_HELD;
    }
    if (cmd->throttle > flight->cfg.throttle_low) {
        if (detail != 0) {
            *detail = cmd->throttle;
        }
        return AK_ARM_THROTTLE;
    }
    if (!flight->est.converged) {
        return AK_ARM_NOT_CONVERGED;
    }

    float c = cos_tilt(&flight->est);
    float limit = ak_cosf(ak_deg2rad(ak_clampf(flight->cfg.arm_max_tilt_deg,
                                               0.0f, 180.0f)));
    if (c < limit) {
        if (detail != 0) {
            /* The tilt itself, for the message: the same angle the comparison
             * is against, recovered from its cosine. Straight up is 0 and
             * upside down is 180, which is what a pilot expects to read. */
            float s = ak_sqrtf(ak_clampf(1.0f - c * c, 0.0f, 1.0f));
            *detail = ak_rad2deg(ak_atan2f(s, c));
        }
        return AK_ARM_NOT_LEVEL;
    }
    return AK_ARM_OK;
}

static void go_safe(ak_flight_t *flight, ak_flight_state_t state)
{
    control_reset(flight);
    outputs_safe(&flight->out);
    flight->state = state;
    flight->arm_hold_active = 0;
    /* Nothing that ends the flight leaves the navigator in charge: whoever
     * stopped it - the pilot, the failsafe, the ground - has the aircraft. */
    flight->managed = 0;
}

/* ---------------------------------------------------------------------------
 * The arming hold may not start until the estimate is *right*, and the
 * estimator's own error is what says so. `converged` counts accelerometer
 * updates: it says fifty samples arrived, not that the filter arrived, and the
 * difference is worth 6.7 degrees of attitude on a 20 degree aircraft. The
 * check in tests/test_aerialkit.c fails without this and passes with it.
 *
 * The innovation is the angle between the up the estimate predicts and the up
 * the accelerometer measures - exactly the vector est_gravity_correct() acts on
 * - and it is asked of the estimator rather than recomputed here, because it is
 * the estimator's error and only the estimator knows it.
 * ------------------------------------------------------------------------- */
#define AK_ARM_INNOV_DEG 2.0f

/* The deadline, in microseconds of sensor time - the same millisecond it has
 * always been, in the unit the loop now counts in. See the field's comment in
 * ak_flight.h for why counting steps made this a function of the loop's rate. */
#define AK_ARM_INNOV_US  1000u

/*
 * Bring the dynamic notch in line with the four parameters that configure it,
 * and with the loop rate they are to be resolved at. Called once per sample and
 * almost always does nothing but five compares.
 *
 * Three things here are decisions the module does not make for itself, because
 * the module has no parameters and no scheduler - it has a band, a count and a
 * rate, and somebody has to turn a person's table into those.
 *
 * **Count zero is this layer's, not the module's.** `ak_dyn_notch_init` clamps
 * its count up to one, because a module asked to find no peaks has nothing to
 * do and "off" is a state only the configuration can be in. So `dyn_notch_count
 * 0` zeroes the instance and leaves `enabled` at 0 - which is the same answer
 * the rate gate gives, reached from the other side.
 *
 * **Q is divided by a hundred here and nowhere else.** See decision 11 - a
 * `dyn_notch_q` of 300 is Q 3, because that is what the number means in the
 * firmware a person read it from.
 *
 * **The rate is the loop's, taken from `loop_period_us`.** Not the gyro's rate
 * and not the scheduler's period: the decimation is of the samples the loop
 * passes to the notch, one per step. It also means `pid_denom` is part of it -
 * a gyro at 8 kHz and a loop at 4 kHz is a 4 kHz notch, and the module's own
 * gate reads that number, not the sensor's.
 *
 * The re-init is deliberately heavy - it defines every buffer and every
 * filter's state - and that is why it is guarded by a comparison rather than
 * run: a table write while armed must not throw away a window the transform is
 * half way through. `dyn_notch_cfg_count` is seeded to a value no parameter can
 * take, so the first sample always configures.
 */
static void dyn_notch_configure(ak_flight_t *flight)
{
    const ak_flight_config_t *cfg = &flight->cfg;
    const float loop_hz = 1000000.0f / (float)flight->loop_period_us;
    const float q = cfg->dyn_notch_q / 100.0f;
    uint8_t     count = (uint8_t)(cfg->dyn_notch_count > (uint32_t)AK_DYN_NOTCH_COUNT_MAX
                                      ? (uint32_t)AK_DYN_NOTCH_COUNT_MAX
                                      : cfg->dyn_notch_count);

    if (count == flight->dyn_notch_cfg_count &&
        cfg->dyn_notch_q == flight->dyn_notch_cfg_q &&
        cfg->dyn_notch_min_hz == flight->dyn_notch_cfg_min_hz &&
        cfg->dyn_notch_max_hz == flight->dyn_notch_cfg_max_hz &&
        loop_hz == flight->dyn_notch_cfg_loop_hz) {
        return;
    }

    if (count == 0u) {
        memset(&flight->dyn_notch, 0, sizeof flight->dyn_notch);
        flight->dyn_notch.enabled = 0u;
    } else {
        ak_dyn_notch_init(&flight->dyn_notch, loop_hz,
                          cfg->dyn_notch_min_hz, cfg->dyn_notch_max_hz,
                          count, (uint16_t)AK_DYN_NOTCH_FFT_POINTS);
        ak_dyn_notch_set_q(&flight->dyn_notch, q);
    }

    flight->dyn_notch_cfg_count  = count;
    flight->dyn_notch_cfg_q      = cfg->dyn_notch_q;
    flight->dyn_notch_cfg_min_hz = cfg->dyn_notch_min_hz;
    flight->dyn_notch_cfg_max_hz = cfg->dyn_notch_max_hz;
    flight->dyn_notch_cfg_loop_hz = loop_hz;
}

void ak_flight_init(ak_flight_t *flight, const ak_mixer_t *mixer)
{
    ak_flight_default_config(&flight->cfg);    ak_rc_default_config(&flight->rc_cfg);
    ak_estimator_init(&flight->est, 0.5f);
    flight->arm_innov_us = 0;
    /* One kilohertz until a caller says otherwise, which is what this firmware
     * ran at before the rate was a parameter - see AK_FLIGHT_LOOP_US. */
    flight->loop_period_us = AK_FLIGHT_LOOP_US;

    /* Zeroed explicitly rather than left to the caller's memory, and marked
     * unprimed so the first usable sample loads the filter instead of being
     * averaged against a zero that no accelerometer would ever read. */
    flight->arm_accel_primed = 0;
    for (int i = 0; i < 3; i++) {
        flight->arm_accel[i] = 0.0f;
    }

    /*
     * The gyro chain's six sections, its record of what they are built for,
     * the gate's filter and the rate each of them is: all defined here, none
     * of them left to the caller's memory.
     *
     * Two things make this necessary rather than tidy. `ak_filter_pt1_set`
     * leaves the filter's state alone - correctly, because changing a cutoff
     * keeps the aircraft's motion - so a section that has never run has
     * whatever `s1` the struct came with, and `ak_filter_pt1_apply` reads it on
     * the very first sample: `y = b0*x + s1`. With the chain's default cutoffs
     * that is a large spike that decays in a few milliseconds; with the chain
     * turned off it never decays, and `flight->gyro` is not the driver's rate
     * at all. It reaches the proportional term.
     *
     * And the chain's change detection *compares* rather than assumes - it
     * rebuilds the coefficients when the cutoff or the interval it is built for
     * has moved - so those three fields must start somewhere no real interval
     * can be, or a configured filter could be skipped on the one sample it was
     * set for. `-1` microseconds is not an interval: the first piece always
     * rebuilds, and it rebuilds from `s1 = 0`, which is what a fresh boot is.
     */
    for (int i = 0; i < 3; i++) {
        ak_filter_pt1_init(&flight->gyro_lpf1[i], 0.0f, 0.0f);
        ak_filter_pt1_init(&flight->gyro_lpf2[i], 0.0f, 0.0f);
        flight->gyro[i] = 0.0f;
    }
    flight->gyro_chain_lpf1_hz = 0.0f;
    flight->gyro_chain_lpf2_hz = 0.0f;
    flight->gyro_chain_dt = -1.0f;

    /*
     * And the notch, which is defined by `dyn_notch_configure` rather than
     * here - one place that knows how a parameter becomes a module, instead of
     * two that have to agree. The guard is seeded to a count no parameter can
     * take so the first sample configures, and the instance is zeroed first so
     * that a core which has never been stepped is a notch that is off rather
     * than a struct full of whatever was on the stack.
     *
     * `dyn_notch_configure` runs the same init this would; calling it here
     * rather than leaving it to the first sample means a caller who reads
     * `flight->dyn_notch.enabled` before stepping anything gets the answer
     * about the *default* configuration, which is the honest one.
     */
    memset(&flight->dyn_notch, 0, sizeof flight->dyn_notch);
    flight->dyn_notch_cfg_count = 0xFFFFFFFFu;
    dyn_notch_configure(flight);

    ak_filter_pt1_init(&flight->arm_accel_lpf, 0.0f, 0.0f);
    flight->arm_accel_lpf_dt = -1.0f;

    for (int i = 0; i < 3; i++) {
        ak_pid_init(&flight->rate[i], flight->cfg.rate_kp[i],
                    flight->cfg.rate_ki[i], flight->cfg.rate_kd[i],
                    flight->cfg.rate_i_limit, flight->cfg.torque_limit,
                    flight->cfg.dterm_lpf1_static_hz);
    }

    flight->mixer = mixer;
    flight->airframe = 0;
    flight->guidance = 0;
    flight->state = AK_FLIGHT_DISARMED;
    /* An input, and one the caller re-states every pass - but zero until it
     * does, because "landed" must never be a guess from uninitialised memory. */
    flight->landed = 0;
    flight->yaw_rate_ff = 0.0f;
    flight->arm_hold_active = 0;
    flight->arm_hold_started_ms = 0;
    flight->arm_released = 0;
    flight->imu_bad = 0;
    flight->imu_bad_since_ms = 0;
    flight->last_imu_us = 0;
    flight->have_last_imu = 0;
    flight->imu_valid = 0;
    flight->last_step_us = 0;
    flight->have_last_step = 0;
    flight->last_loop_us = 0;
    flight->steps = 0;
    flight->timing = (ak_flight_timing_t){0};
    flight->link_live = 0;
    go_safe(flight, AK_FLIGHT_DISARMED);
    ak_flight_apply_airframe(flight);
}

void ak_flight_set_board_outputs(ak_flight_t *flight, unsigned motors,
                                 unsigned servos)
{
    flight->board_motors = (uint8_t)motors;
    flight->board_servos = (uint8_t)servos;
    flight->board_outputs_known = 1u;
}

void ak_flight_set_loop_period_us(ak_flight_t *flight, uint32_t period_us)
{
    if (period_us < AK_FLIGHT_LOOP_MIN_US) {
        period_us = AK_FLIGHT_LOOP_MIN_US;
    } else if (period_us > AK_FLIGHT_LOOP_MAX_US) {
        period_us = AK_FLIGHT_LOOP_MAX_US;
    }
    flight->loop_period_us = period_us;
}

void ak_flight_apply_airframe(ak_flight_t *flight)
{
    flight->mixer = ak_mixer_for_airframe(flight->airframe);

    /* What that mix needs, counted from the table itself rather than written
     * down beside it: a second copy of "a quad has four motors" is exactly the
     * kind of number that stops being true when a table changes. Only the rows
     * the mixer claims are counted - the arrays are longer than the aircraft. */
    unsigned motors = 0u;
    unsigned servos = 0u;

    if (flight->mixer != 0) {
        for (unsigned i = 0; i < flight->mixer->count; i++) {
            if (flight->mixer->kind[i] == AK_OUT_SERVO) {
                servos++;
            } else {
                motors++;
            }
        }
    }
    flight->needed_motors = (uint8_t)motors;
    flight->needed_servos = (uint8_t)servos;
}

void ak_flight_apply_config(ak_flight_t *flight)
{
    ak_flight_apply_airframe(flight);

    /* The same six arguments ak_flight_init passes, from the same fields, read
     * again - which is the whole point. Two places that build a rate loop out
     * of the configuration would be two places to forget, so this is the one
     * that a parameter change goes through and init is left as the one that
     * runs before any parameter has been read. */
    for (int i = 0; i < 3; i++) {
        ak_pid_init(&flight->rate[i], flight->cfg.rate_kp[i],
                    flight->cfg.rate_ki[i], flight->cfg.rate_kd[i],
                    flight->cfg.rate_i_limit, flight->cfg.torque_limit,
                    flight->cfg.dterm_lpf1_static_hz);
    }
}

/* The bound on one integration step, with a guard for a configuration that has
 * somehow been given zero: the parameter table will not take it, but a division
 * by a zero bound is a long way from where the mistake was made.
 *
 * `max_dt_ms` is a *parameter*, and it stays a millisecond parameter: it is in
 * the table, it is in every saved configuration, it is what `set max_dt_ms`
 * takes and what the console prints, and re-denominating it would change the
 * meaning of a number somebody has already written down without changing
 * anything they can see. The flight path's own clock is microseconds, so the
 * conversion happens here, once, at the one place the parameter is read. */
static uint32_t step_bound_us(const ak_flight_t *flight)
{
    return flight->cfg.max_dt_ms != 0u ? flight->cfg.max_dt_ms * 1000u
                                       : flight->loop_period_us;
}

/*
 * What one call to ak_flight_step() should integrate, and in how many pieces.
 *
 * This replaces a function that returned a single float and got three things
 * wrong at once, all of them measured in tests/oracle/test_timing.c:
 *
 *   - A gap longer than max_dt_ms was replaced by *one millisecond*. The
 *     aircraft had flown through the whole gap and the gyro had been working
 *     the entire time, so the rotation was knowable, and the estimator was told
 *     that a fifty-first of it had happened. A bus that was simply too slow put
 *     the estimate permanently at a fraction of the true rate - measured at
 *     1/51st, per sample, for a bus one millisecond over the bound.
 *   - The elapsed time was discarded rather than deferred: last_imu_ms was
 *     updated regardless, so the next sample saw a normal interval and the lost
 *     time never came back.
 *   - A duplicate sample was clamped *up* to half a millisecond, inventing
 *     rotation that had not happened, while a real gap was clamped down. The
 *     same expression was wrong in both directions.
 *
 * The replacement does not clamp the time; it bounds each *step*. A gap is
 * integrated in pieces of at most max_dt_ms, up to a fixed budget of them, so
 * no single integration is large enough to trip the derivative term or to alias
 * a fast rotation into a slow one - which is what the old clamp was for, and it
 * is still achieved - while the whole of the elapsed time reaches the estimate.
 * Anything past the budget is counted in dropped_us rather than hidden.
 *
 * The return is a plan rather than a duration because the caller has to run the
 * gyro filter and the estimator once per piece. substeps == 0 means "integrate
 * nothing this call", which is the honest answer both for a duplicate sample
 * and for a timestamp that went backwards.
 */
typedef struct {
    uint32_t elapsed_us;    /* what the source's timestamp says happened */
    uint32_t integrate_us;  /* how much of it this call will integrate */
    uint32_t substeps;      /* in how many pieces, each at most the bound */
} ak_step_plan_t;

static ak_step_plan_t plan_step(ak_flight_t *flight, const ak_imu_sample_t *imu)
{
    const uint32_t bound = step_bound_us(flight);
    const uint32_t budget = bound * AK_FLIGHT_MAX_CATCHUP;
    ak_step_plan_t plan;

    /*
     * No predecessor: this is the first sample, and there is no interval to
     * integrate over.
     *
     * This used to answer `AK_FLIGHT_LOOP_MS` - one nominal control period -
     * and phase 1.5 removed that, because it was the last place in the flight
     * path where a *nominal* period stood in for a measured one and it was
     * wrong in a way that is easy to miss. The interval before the first sample
     * is not one period: it is however long the board took to get from bringing
     * its sensors up to its first reading, and nothing measured it. Handing the
     * estimator a control period instead says "one millisecond of rotation
     * happened" about a span nobody looked at, and on a board that took a
     * hundred milliseconds to reach its first sample it says so a hundred times
     * too small.
     *
     * The honest answer is that there is no elapsed time to integrate, so
     * nothing is integrated and the estimate starts where the first sample says
     * it is. That costs nothing: there is no earlier attitude to propagate
     * *from*. `have_last_imu` is still latched here, which is the whole job of
     * this branch.
     */
    if (!flight->have_last_imu) {
        flight->last_imu_us = imu->time_us;
        flight->have_last_imu = 1;
        plan.elapsed_us = 0u;
        plan.integrate_us = 0u;
        plan.substeps = 0u;
        return plan;
    }

    const uint32_t delta = imu->time_us - flight->last_imu_us;

    /*
     * A timestamp that went backwards is a different clock, not a very long
     * gap. Unsigned subtraction reports a hundred microseconds backwards as
     * nearly 2^32 microseconds forwards, and the two cases want opposite
     * answers: a gap is time the aircraft really flew and must be integrated, a
     * clock change is time nobody can account for and must not be. Told apart
     * before the arithmetic rather than after it.
     *
     * At this resolution the 71.6-minute wrap lands here too, and that is the
     * intended reading rather than a compromise: the interval across a wrap is
     * not representable in 32 bits of microseconds, so it is counted as a reset
     * and one sample is not integrated. See AK_FLIGHT_CLOCK_MAX_US.
     */
    if (delta > AK_FLIGHT_CLOCK_MAX_US) {
        flight->timing.clock_resets++;
        flight->last_imu_us = imu->time_us;
        plan.elapsed_us = 0u;
        plan.integrate_us = 0u;
        plan.substeps = 0u;
        return plan;
    }

    flight->last_imu_us = imu->time_us;

    /*
     * A reading the board could not use. There is no rate to integrate, so the
     * whole interval is unrecoverable and is counted as such - the budget below
     * does not apply, because there is nothing to spend it on. It is still
     * *time*, though, and the next usable sample must not see a gap that has
     * already been accounted for, which is why it is counted here rather than
     * being left for the next call to discover.
     */
    if (!imu->valid) {
        flight->timing.unusable++;
        flight->timing.dropped_us += delta;
        plan.elapsed_us = delta;
        plan.integrate_us = 0u;
        plan.substeps = 0u;
        return plan;
    }

    /* The same timestamp twice: a frozen sensor, or a loop polling faster than
     * its gyro. Nothing elapsed, so nothing is integrated - and unlike the old
     * clamp, nothing is invented either. */
    if (delta == 0u) {
        flight->timing.duplicates++;
        plan.elapsed_us = 0u;
        plan.integrate_us = 0u;
        plan.substeps = 0u;
        return plan;
    }

    plan.elapsed_us = delta;
    plan.integrate_us = delta;

    if (delta > bound) {
        flight->timing.gap_steps++;
    }

    if (plan.integrate_us > budget) {
        /* More than one call can catch up on. Integrating it all would make
         * this iteration longer than the stall it is recovering from, which is
         * how a loop that is behind stays behind. The surplus is counted. */
        flight->timing.dropped_us += plan.integrate_us - budget;
        plan.integrate_us = budget;
    }

    plan.substeps = (plan.integrate_us + bound - 1u) / bound;
    if (plan.substeps == 0u) {
        plan.substeps = 1u;
    }
    if (plan.substeps > 1u) {
        flight->timing.catchup_steps += plan.substeps - 1u;
    }
    return plan;
}

/*
 * The control loop's own period, which is not the sensor's interval.
 *
 * These are two clocks and the old step_dt() used one of them for both jobs.
 * The estimator integrates the gyro, so it needs the interval between the
 * samples it is handed; the rate loop runs once per call to ak_flight_step(),
 * so its integral and derivative terms need the interval between calls. On a
 * board whose gyro runs at 8 kHz and whose loop runs at 1 kHz they differ by a
 * factor of eight, and on a board whose loop is late they differ by whatever it
 * was late by.
 *
 * This one is not clamped either. If an iteration took a hundred milliseconds
 * then the control law ran once over a hundred milliseconds and that is what it
 * should be told; long_loops and max_loop_us record that it happened.
 *
 * Phase 1.5 removed the two nominal-period answers that used to be here, and
 * the reason is worth keeping because the second one was not obviously wrong:
 *
 *   - With no previous step the interval is unknown, so the PID is told zero
 *     and ak_pid_update() returns without touching its integral or derivative
 *     terms. That is one iteration of proportional-only control on the first
 *     loop after a reset, which is the correct amount of authority to give a
 *     controller that does not yet know how long it has been since last time.
 *     It used to be told one millisecond, which is a *claim* about an interval
 *     nothing measured, and on the boot iteration - where the previous step is
 *     separated from this one by however long the boot took - it was a claim
 *     that was wrong by three orders of magnitude.
 *   - `measured == 0` used to fall through to the nominal as well, and at a
 *     millisecond that was doing real work: a loop polling its gyro faster than
 *     a kilohertz genuinely could not tell a duplicate step from a step one
 *     microsecond later. In microseconds the two are different numbers and the
 *     honest one is available, so a zero interval now means zero and the PID
 *     skips the iteration. This is the change that makes a four-kilohertz loop
 *     possible: at 250 us per iteration a millisecond clock reports 0 or 1 and
 *     the derivative term is computed over an interval that is wrong by the
 *     whole of its own period half the time.
 */
static float loop_dt(ak_flight_t *flight, uint32_t now_us)
{
    uint32_t delta = 0u;

    if (flight->have_last_step) {
        const uint32_t measured = now_us - flight->last_step_us;
        if (measured <= AK_FLIGHT_CLOCK_MAX_US) {
            delta = measured;
        }
    }
    flight->last_step_us = now_us;
    flight->have_last_step = 1;
    flight->last_loop_us = delta;

    if (delta > step_bound_us(flight)) {
        flight->timing.long_loops++;
    }
    if (delta > flight->timing.max_loop_us) {
        flight->timing.max_loop_us = delta;
    }
    return (float)delta / 1000000.0f;
}

/*
 * What a fixed wing flies when its pilot is gone and there is no navigator to
 * bring it home.
 *
 * The one thing it must not do is stop: with the throttle closed and the
 * servos centred a wing has no control at all, and its arrival is whatever
 * attitude it happened to be in. So this is a *descending circle* - a bank the
 * attitude loop holds, a nose a little down, and enough throttle for the
 * elevons to keep working - flown through the same control law the pilot's own
 * sticks go through, because a failsafe that flew a path of its own would be a
 * second autopilot to prove.
 *
 * The direction of the circle is arbitrary and deliberately so: which way a
 * lost aircraft turns is worth less than knowing that it will, and where it
 * comes down is decided by where it was when the link went.
 */
static void descend_command(const ak_flight_t *flight, ak_rc_command_t *out)
{
    float tilt = ak_deg2rad(flight->cfg.max_tilt_deg);
    float bank = tilt > 0.0f
                     ? ak_deg2rad(flight->cfg.wing_descend_bank_deg) / tilt
                     : 0.0f;
    float down = tilt > 0.0f
                     ? ak_deg2rad(flight->cfg.wing_descend_pitch_deg) / tilt
                     : 0.0f;

    out->roll = ak_clampf(bank, -1.0f, 1.0f);
    out->pitch = ak_clampf(-down, -1.0f, 1.0f); /* nose down is negative */
    out->yaw = 0.0f;
    out->throttle = ak_clampf(flight->cfg.wing_descend_throttle, 0.0f, 1.0f);
    out->angle_mode = 1;
    out->arm_request = 0; /* a failsafe does not decide whether to be armed */
}

/* The gyro this uses is the filtered one (`flight->gyro`), which is why it is
 * not handed the raw sample: the filter is applied once, in the step, before
 * anything reads a rate. */
static void control_armed(ak_flight_t *flight, const ak_rc_command_t *cmd,
                          float dt)
{
    /* The rate loop's phase begins here, at the first thing that is the control
     * law rather than the arming state. Everything before it in
     * ak_flight_step() - the link, the failsafes, the plan - is not stamped,
     * and shows up as the loop's un-owned time. */
    phase(AK_FLIGHT_PHASE_PID);

    const float dterm_lpf1_hz =
        ak_flight_chain_cutoff(cmd->throttle, flight->cfg.dterm_lpf1_static_hz,
                               flight->cfg.dterm_lpf1_dyn_min_hz,
                               flight->cfg.dterm_lpf1_dyn_max_hz);

    if (cmd->angle_mode) {
        float max_tilt = ak_deg2rad(flight->cfg.max_tilt_deg);
        float roll_target = ak_clampf(cmd->roll * max_tilt, -max_tilt, max_tilt);
        float pitch_target = ak_clampf(cmd->pitch * max_tilt, -max_tilt, max_tilt);
        flight->rate_setpoint[0] =
            ak_clampf(flight->cfg.angle_kp[0] * (roll_target - flight->est.roll),
                      -flight->cfg.angle_rate_limit, flight->cfg.angle_rate_limit);
        flight->rate_setpoint[1] =
            ak_clampf(flight->cfg.angle_kp[1] * (pitch_target - flight->est.pitch),
                      -flight->cfg.angle_rate_limit, flight->cfg.angle_rate_limit);
    } else {
        float max_rate = ak_deg2rad(flight->cfg.max_rate_dps);
        flight->rate_setpoint[0] = cmd->roll * max_rate;
        flight->rate_setpoint[1] = cmd->pitch * max_rate;
    }
    /* The pilot's rudder, plus the turn the airframe is already making: the
     * loop's job is to damp what the bank has started, not to argue with it. */
    flight->rate_setpoint[2] = cmd->yaw * ak_deg2rad(flight->cfg.yaw_rate_dps) +
                               flight->yaw_rate_ff;

    for (int i = 0; i < 3; i++) {
        if (!ak_mixer_has_axis(flight->mixer, (unsigned)i)) {
            /*
             * This aircraft has no output that can produce this axis, so the
             * loop is not run at all - not run-and-then-multiplied-by-zero.
             *
             * The difference is the integral. A PID handed a setpoint it can
             * never reach does not return zero: it returns a small torque that
             * the mixer discards, and it *keeps* the error it accumulated while
             * doing so. On a wing with no rudder a pilot holding rudder, or a
             * yaw feed-forward tracking a turn the aircraft cannot coordinate,
             * winds that term up without limit; it is invisible for as long as
             * the axis stays unauthorised and it is still there - full and
             * saturated - the moment one is. Resetting the state each tick is
             * what makes "no authority" a property of the loop rather than a
             * number multiplied by it.
             *
             * The setpoint goes to zero with it, because that is what the
             * controller is asking for: nothing. Leaving the pilot's demand
             * here would put a rate into the log that the aircraft was never
             * asked to fly and never could have flown, and the next person to
             * read that log would be looking for the fault in the mixer.
             *
             * The honest consequence: **on this airframe the rudder channel
             * does nothing at all**, and no amount of rudder will turn it.
             * Course changes are bank and pull, which is roll and pitch, which
             * this wing does have.
             */
            flight->rate_setpoint[i] = 0.0f;
            flight->torque[i] = 0.0f;
            ak_pid_reset(&flight->rate[i]);
            continue;
        }
        /*
         * The D-term chain's first cutoff, resolved from the same throttle the
         * gyro chain used and by the same rule. Set here, once per sample for
         * all three axes, rather than inside the loop above: the D term is
         * differentiated at the *control* rate - it is the rate loop's own
         * dt, not the integration piece's - so this chain moves once per
         * sample where the gyro chain moves once per piece, and that
         * difference is the reference's too.
         *
         * The second section is static, and both are only rebuilt when the
         * resolved cutoff moved. On a hover that is never; under a throttle
         * change it is every sample, which is why the change check exists.
         */
        ak_pid_set_dterm_lpf(&flight->rate[i], dterm_lpf1_hz,
                             flight->cfg.dterm_lpf2_static_hz, dt);
        flight->torque[i] = ak_pid_update(&flight->rate[i],
                                          flight->rate_setpoint[i],
                                          flight->gyro[i], dt);
    }

    /* Armed, so the mixer takes care of the authority limit: what the control
     * loop asked for, reduced only as far as it has to be to fit. */
    phase(AK_FLIGHT_PHASE_MIXER);
    ak_mixer_apply_limited(flight->mixer, cmd->throttle, flight->torque,
                           &flight->out);
}

int ak_flight_link_update(ak_flight_t *flight, const ak_rc_input_t *rc,
                          uint32_t now_ms, ak_rc_command_t *command)
{
    ak_rc_command_t scratch;
    ak_rc_command_t *decoded_into = command != 0 ? command : &scratch;

    int decoded = ak_rc_decode(rc, &flight->rc_cfg, decoded_into);
    flight->link_live =
        decoded &&
        (uint32_t)(now_ms - rc->last_update_ms) <= flight->cfg.rc_timeout_ms;
    return flight->link_live;
}

void ak_flight_step(ak_flight_t *flight, const ak_imu_sample_t *imu,
                    const ak_rc_input_t *rc, uint32_t now_us, uint32_t now_ms)
{
    flight->steps++;
    flight->imu_valid = imu != 0 && imu->valid;

    /* `now_ms` is the caller's, not `now_us / 1000u`. Both are readings of the
     * same counter at the same instant and they agree until the microsecond one
     * wraps, which it does every 71.6 minutes - and past that the quotient is
     * the millisecond clock modulo the wrap, so a deadline computed from it
     * fires on the first loop after the wrap and every loop thereafter. The
     * receiver timeout below is compared against a `last_update_ms` the
     * receiver driver stamped from ak_time_ms(), and the arm hold below is a
     * `_ms` parameter; both are millisecond rules and both are answered in
     * milliseconds. See ak_flight_step()'s declaration for the measurement. */

    /* The control law's interval, and the sensor's interval. Two intervals; see
     * loop_dt() for why they are not the same number. Both are microseconds
     * from the same counter, which is what lets a loop faster than a kilohertz
     * state its own period. */
    float dt = loop_dt(flight, now_us);
    ak_step_plan_t plan = plan_step(flight, imu);

    /*
     * The gyro, filtered once per integration piece, before anything reads it.
     *
     * Everything downstream - the estimator's propagation and the three rate
     * loops - uses `flight->gyro`, which is the driver's sample through the
     * gyro chain: two first-order sections, the first at a cutoff that follows
     * the throttle (`gyro_lpf1_*`) and the second fixed (`gyro_lpf2_static_hz`),
     * both 0-disabled. What that buys is not "smoother flight": it is that the
     * proportional term multiplies the *filtered* rate rather than the
     * sensor's noise, so the motor commands do not carry it. Before roadmap
     * 2.2 the only filter in the loop was on the derivative, which is the one
     * term whose filtering cannot help the others.
     *
     * It is deliberately *not* reset with the rest of the loop: a filter's
     * state is the aircraft's own motion, and zeroing it on every disarmed
     * step - which is where `control_reset` runs - would have the arm gate's
     * estimate see a rate of zero on an aircraft somebody is turning over in
     * their hands.
     *
     * A reading the board marked unusable is not filtered in at all, and the
     * plan says so by asking for no pieces. Feeding it through would put a
     * garbage rate straight into `flight->gyro`, which is what the rate loop's
     * proportional term reads, and the filter's own gain would then spread it
     * over the following samples. The previous good rate is held instead, and
     * the interval it covers is already counted in dropped_us - there is no
     * rate to integrate it with.
     */
    if (plan.substeps > 0u) {
        /*
         * One piece, or - when the sensor interval was longer than one
         * integration may span - several. Each piece is at most max_dt_ms, so
         * the filter and the estimator never take a step large enough to alias
         * a fast rotation into a slow one, while the whole of the elapsed time
         * still reaches them. On a normal one-millisecond loop this is one
         * iteration of a loop that runs once, and the arithmetic is what it
         * always was.
         */
        /*
         * The estimator's phase opens here, inside the branch rather than above
         * it: on a loop the plan gave no pieces to - a sensor interval nothing
         * can be integrated over - the gyro filter does not run and the
         * estimator is not stepped, and a mark above the branch would charge
         * that skipped work to the estimator anyway. A phase that did not run
         * has to read zero. The phase mark is before the filter because the
         * filter is the estimator's input conditioning, and the time between
         * the IMU read above and this line is the loop's un-owned time.
         */
        phase(AK_FLIGHT_PHASE_ESTIMATOR);

        ak_imu_sample_t filtered = *imu;
        const float sub_dt =
            (float)plan.integrate_us / (float)plan.substeps / 1000000.0f;

        /*
         * The chain runs once per *sample*, at the loop's own period, and the
         * pieces below then integrate the rate it produced. It ran once per
         * piece until tests/oracle/test_timing.c caught what that costs: a
         * piece is the estimator's integration step, not a filter sample
         * interval, and handing it to `ak_filter_pt1_set` made the chain's
         * cutoff a function of how late the loop had run. A thirty-second gap
         * arrives as sixteen fifty-millisecond pieces, the library clamps a
         * cutoff to a quarter of the sample rate, and the 250 Hz stage that
         * flies the aircraft became a 5 Hz one for the length of the catch-up -
         * a filter no configuration asked for, engaged by the loop stalling and
         * released by it catching up. The check that caught it is the one that
         * justifies the pieces: `yaw advances` read 75.0 deg where 800 ms of a
         * 100 deg/s rotation is 80.0, and the missing sixteenth is the settling
         * the chain ate on the piece it was re-tuned for.
         *
         * `loop_period_us` rather than the measured interval, because a cutoff
         * in the table has to mean a frequency rather than the loop's jitter.
         * The reference reads its filter coefficients off the configured loop
         * time for the same reason. The D-term chain has always worked this
         * way: `ak_pid_set_dterm_lpf` is handed the control loop's own dt, once
         * per sample, in `control_armed`.
         */
        const float chain_dt = (float)flight->loop_period_us / 1000000.0f;

        /*
         * The chain's first cutoff follows the throttle, so it is resolved
         * once per sample with it. The second section's cutoff is static;
         * both are only rebuilt when one of them or the interval actually
         * moved, which on a steady loop is never.
         *
         * **The throttle is the previous sample's, and it has to be.**
         * This block runs before `ak_flight_link_update` decodes the
         * receiver, so the command for *this* sample does not exist yet -
         * and moving the decode above the filter would put the whole
         * arming and failsafe state machine in front of the estimator's
         * input, which is a much larger change than one sample of throttle
         * is worth. `flight->cmd` is what the last pass decided, so the
         * cutoff is one loop period behind the stick: 1 ms at a kilohertz,
         * against a filter whose own lag at 250 Hz is 0.64 ms. The
         * reference has the same shape - its dynamic cutoff is computed
         * from the throttle the PID loop last produced and applied to the
         * gyro filter, not from the value in flight at the instant the
         * gyro was read.
         *
         * On the first sample after boot `flight->cmd` is zero, which
         * resolves to `dyn_min`: the most filtering the chain can apply,
         * for one millisecond, on an aircraft that is not armed yet.
         */
        const float lpf1_hz =
            ak_flight_chain_cutoff(flight->cmd.throttle,
                                   flight->cfg.gyro_lpf1_static_hz,
                                   flight->cfg.gyro_lpf1_dyn_min_hz,
                                   flight->cfg.gyro_lpf1_dyn_max_hz);

        const float lpf2_hz = flight->cfg.gyro_lpf2_static_hz;

        /*
         * A stage that has just been switched back on is primed rather than
         * reset, and it is primed with the value it is about to be given -
         * the sample itself for the first section, the first section's answer
         * for the second. A disabled filter passes its input through and
         * leaves its state alone - that is the library's own contract - so
         * without this, re-enabling a gyro filter would resume from the rate
         * the aircraft had seconds ago. The old answer was to zero the state,
         * which threw away that stale rate and the first sample's real one
         * with it: a first-order section takes two samples to reach its steady
         * state, so two cascaded sections starting from zero put a quarter of
         * the first sample through, and `make timing` read 0.024784 deg where
         * the rotation a millisecond is worth is 0.100000. Priming keeps the
         * output continuous with the input - no step into the proportional
         * term, which is the whole of what the zero was for - and loses
         * nothing. The arming gate's accelerometer filter has primed itself
         * this way since it was written (`arm_accel_primed`); this is the same
         * idea for the same reason.
         *
         * The flags are read before the block below stores the new cutoffs,
         * because "was it off" is a question about the last sample.
         */
        const int restart1 =
            (flight->gyro_chain_lpf1_hz <= 0.0f && lpf1_hz > 0.0f);
        const int restart2 =
            (flight->gyro_chain_lpf2_hz <= 0.0f && lpf2_hz > 0.0f);

        /*
         * The notch's slow half is fed here, once per *sample* - the same rate
         * the chain runs at, and the rate the decimation counts. The raw
         * sample, not the filtered one: the module is measuring the vibration
         * the airframe is making, and a copy that had been through this chain
         * would be a measurement of this chain.
         *
         * The configuration is brought into line first, because `push` and
         * `filter` both read `enabled` and a parameter written while the loop
         * was running must take effect on the next sample rather than on the
         * next boot. Almost always this is five compares.
         */
        dyn_notch_configure(flight);
        ak_dyn_notch_push(&flight->dyn_notch, imu->gyro);

        if (lpf1_hz != flight->gyro_chain_lpf1_hz ||
            lpf2_hz != flight->gyro_chain_lpf2_hz ||
            chain_dt != flight->gyro_chain_dt) {
            for (int i = 0; i < 3; i++) {
                ak_filter_pt1_set(&flight->gyro_lpf1[i], lpf1_hz, chain_dt);
                ak_filter_pt1_set(&flight->gyro_lpf2[i], lpf2_hz, chain_dt);
            }
            flight->gyro_chain_lpf1_hz = lpf1_hz;
            flight->gyro_chain_lpf2_hz = lpf2_hz;
            flight->gyro_chain_dt = chain_dt;
        }

        for (int i = 0; i < 3; i++) {
            /*
             * The dynamic notch, first in the chain - before the low-passes.
             *
             * It is worth being exact about what that ordering does and does
             * not buy, because the obvious argument for it is not true. A notch
             * and a first-order low-pass are both linear and time-invariant, so
             * their *steady-state response in series does not depend on their
             * order at all*: put the low-pass first, then the notch, and the
             * same 450 Hz tone comes out the same place at the same level. A
             * comment claiming the notch "takes out less of the real thing"
             * after a low-pass would be a comment claiming a property two LTI
             * sections cannot have.
             *
             * What the order does change is the two things that are not LTI:
             *
             *  - **The notch is time-varying.** Its centre moves with the
             *    measured peak every window, and a moving filter does not
             *    commute with a fixed one. What the notch is asked to remove is
             *    decided in the loop's own terms - where the peak is *now* -
             *    and putting the fixed low-passes after it keeps that decision
             *    the last word rather than something a later section smears.
             *  - **The priming reads the chain's input.** `restart1` primes the
             *    first low-pass when it is re-enabled, and priming a stage with
             *    anything but its own input is a step into the proportional
             *    term - so it is primed with `g`, the notch's output and the
             *    low-pass's real input, rather than with `imu->gyro[i]`.
             *
             * And it is where the reference puts it: `gyroFilter` runs the
             * dynamic notch on the raw sample and the low-passes after it
             * (upstream/betaflight-2025.12.2/src/main/sensors/gyro_filter_impl.h).
             *
             * The measurement is not affected by any of this, and that is a
             * decision of its own rather than luck: the call above hands
             * `ak_dyn_notch_push` the driver's reading, not `g`. A notch whose
             * centre is measured through its own filters chases its own tail,
             * and the push is where that is prevented.
             */
            float g = ak_dyn_notch_filter(&flight->dyn_notch, (uint8_t)i,
                                          imu->gyro[i]);
            if (restart1) {
                ak_filter_pt1_prime(&flight->gyro_lpf1[i], g);
            }
            g = ak_filter_pt1_apply(&flight->gyro_lpf1[i], g);
            if (restart2) {
                ak_filter_pt1_prime(&flight->gyro_lpf2[i], g);
            }
            g = ak_filter_pt1_apply(&flight->gyro_lpf2[i], g);
            flight->gyro[i] = g;
            filtered.gyro[i] = g;
        }

        /*
         * The pieces are the estimator's, and only the estimator's: each is at
         * most `max_dt_ms`, so no single propagation is large enough to alias a
         * fast rotation into a slow one, while the whole of the elapsed time
         * still reaches the estimate. The rate they integrate is the one
         * filtered above and held across them - there is one gyro sample here,
         * and a filter with one sample has no second one to advance to.
         */
        for (uint32_t piece = 0; piece < plan.substeps; piece++) {
            ak_estimator_update(&flight->est, &filtered, sub_dt);
        }

        /*
         * The arming gate's accelerometer. The estimator above is given the
         * driver's sample, deliberately - see the plan comment - and this is a
         * second path for the same reading, filtered once per sample, read by
         * the gate and by nothing else.
         *
         * Once per sample and not once per sub-step, because the gate makes one
         * decision per sample: filtering per piece would apply the same cutoff
         * more times the longer the gap was, which is a different filter at a
         * different cutoff, and the gate would then be reading a number that
         * depends on the loop's timing rather than on the aircraft.
         *
         * It is inside the `substeps > 0` branch with the estimator, so an
         * unusable reading never enters it - the previous good sample is held,
         * which is the same thing the gyro filter does and for the same reason.
         * The interval is `integrate_us`, the same interval the estimator
         * integrates, so a step the catch-up budget clamped filters over the
         * time that was actually accounted for rather than the time that
         * elapsed.
         */
        {
            const float adt = (float)plan.integrate_us / 1000000.0f;

            if (flight->arm_accel_lpf_dt != adt) {
                if (flight->arm_accel_lpf_dt <= 0.0f &&
                    flight->cfg.arm_accel_lpf_hz > 0.0f) {
                    ak_filter_pt1_reset(&flight->arm_accel_lpf);
                }
                ak_filter_pt1_set(&flight->arm_accel_lpf,
                                  flight->cfg.arm_accel_lpf_hz, adt);
                flight->arm_accel_lpf_dt = adt;
            }

            if (!flight->arm_accel_primed) {
                for (int i = 0; i < 3; i++) {
                    flight->arm_accel[i] = imu->accel[i];
                }
                flight->arm_accel_primed = 1;
            } else {
                for (int i = 0; i < 3; i++) {
                    flight->arm_accel[i] =
                        ak_filter_pt1_apply(&flight->arm_accel_lpf,
                                            imu->accel[i]);
                }
            }
        }

        if (ak_estimator_innovation_deg(&flight->est, flight->arm_accel) <=
            AK_ARM_INNOV_DEG) {
            /* `plan.integrate_us`, the same interval the filter just above was
             * run over, so the hold is measured in the time the gate and the
             * filter both accounted for rather than in the time that happened
             * to elapse. A step the catch-up budget clamped therefore adds less
             * than elapsed, which is the direction that opens the gate *later* -
             * the conservative one for a gate whose whole job is "has this
             * settled". */
            if (flight->arm_innov_us < 1000000u) {
                flight->arm_innov_us += plan.integrate_us;
            }
        } else {
            flight->arm_innov_us = 0;
        }
    }

    /*
     * Landed, under a navigator: the return is over. This is the only way this
     * firmware stops its own motors, and the two conditions are the whole
     * safety argument - a navigator is flying it, and something that measures
     * the height says the height has stopped falling at the ground.
     *
     * It comes first, before the link is even looked at, because a return is
     * usually a *failsafe* return: the version of this rule that sat further
     * down the function was never reached with the link down, which is exactly
     * when a landing happens.
     */
    if (flight->landed && (flight->state == AK_FLIGHT_RTH ||
                           flight->state == AK_FLIGHT_MANAGED ||
                           flight->state == AK_FLIGHT_DESCEND)) {
        go_safe(flight, AK_FLIGHT_DISARMED);
        return;
    }

    ak_rc_command_t cmd;
    int rc_live = ak_flight_link_update(flight, rc, now_ms, &cmd);
    flight->cmd = cmd;
    if (rc_live && !cmd.arm_request) {
        flight->arm_released = 1;
    }

    /*
     * One unusable sample is not a lost attitude: the plan above already holds
     * the last good rate through it, and a single failed bus transaction used
     * to latch a failsafe and stop the motors in flight - recoverable only by
     * the arm switch. Armed, a run of them is held through for AK_IMU_LOSS_MS;
     * past that, or on the ground, the sample is what it says.
     */
    int imu_lost = 0;
    if (imu->valid) {
        flight->imu_bad = 0;
    } else {
        if (!flight->imu_bad) {
            flight->imu_bad = 1;
            flight->imu_bad_since_ms = now_ms;
        }
        imu_lost = flight->state == AK_FLIGHT_DISARMED ||
                   (uint32_t)(now_ms - flight->imu_bad_since_ms) >= AK_IMU_LOSS_MS;
    }

    if (!rc_live || imu_lost) {
        /* A disarmed aircraft has nothing to fail safe from, whatever is
         * missing: no receiver, no attitude estimate, or neither. It sits
         * there, and calling that a failsafe would confuse the console and
         * latch a state that never happened. This comes first inside the
         * "something is missing" path - not before it, or a healthy aircraft
         * could never arm - and it is why a board with no IMU at all (the
         * ESP32 port, for now) reports disarmed rather than failsafe from its
         * first loop iteration. Found by running it there. */
        if (flight->state == AK_FLIGHT_DISARMED) {
            go_safe(flight, AK_FLIGHT_DISARMED);
            return;
        }

        /* No attitude is not a state to fly in: stop, whoever is talking. */
        if (imu_lost) {
            go_safe(flight, AK_FLIGHT_FAILSAFE);
            return;
        }

        /* The pilot's link is gone. If a navigator has something to say and the
         * aircraft was already flying, let it fly - that is the whole point of
         * carrying a GPS. Otherwise stop, and latch it: coming back from a
         * failsafe means the arm switch going off first, not the link
         * reappearing for one frame. */
        if (flight->guidance != 0 && (flight->state == AK_FLIGHT_ARMED ||
                                      flight->state == AK_FLIGHT_RTH ||
                                      flight->state == AK_FLIGHT_MANAGED)) {
            /* Which of the two it is does not change what flies the aircraft,
             * only what the console says about why: a mission the pilot
             * started keeps going when the link goes, because the navigator
             * was already flying it. */
            flight->state = flight->managed ? AK_FLIGHT_MANAGED
                                            : AK_FLIGHT_RTH;
            control_armed(flight, flight->guidance, dt);
            return;
        }

        /*
         * Nothing is flying it. What happens next belongs to the airframe: a
         * quadrotor stops, because thrust is what holds it up and there is
         * nothing to gain by holding - and a fixed wing circles down, because
         * an aircraft with wings that stops flying is not safer, it is a brick
         * with a battery in it. Which of the two this is comes from the mix
         * (ak_mixer_t.fixed_wing), so it cannot disagree with what the outputs
         * are actually driving.
         */
        if (flight->mixer != 0 && flight->mixer->fixed_wing) {
            ak_rc_command_t descend;

            flight->state = AK_FLIGHT_DESCEND;
            flight->managed = 0;
            descend_command(flight, &descend);
            control_armed(flight, &descend, dt);
            return;
        }
        go_safe(flight, AK_FLIGHT_FAILSAFE);
        return;
    }

    if (flight->state == AK_FLIGHT_FAILSAFE ||
        flight->state == AK_FLIGHT_DESCEND) {
        if (!cmd.arm_request && cmd.throttle <= flight->cfg.throttle_low) {
            go_safe(flight, AK_FLIGHT_DISARMED);
        } else if (flight->state == AK_FLIGHT_DESCEND) {
            /*
             * The link may have come back, but the switch is still on: the
             * aircraft is still coming down in its circle, and a pilot who
             * wants it has to take it back the way this firmware's failsafes
             * are always taken back - arm switch off, throttle down, then fly
             * it. A link that reappeared for a frame does not hand an
             * aircraft back in the middle of a descent.
             */
            ak_rc_command_t descend;

            descend_command(flight, &descend);
            control_armed(flight, &descend, dt);
        } else {
            go_safe(flight, AK_FLIGHT_FAILSAFE);
        }
        return;
    }

    if (flight->state == AK_FLIGHT_DISARMED) {
        outputs_safe(&flight->out);
        control_reset(flight);

        /*
         * Arming needs the switch, the throttle down, an attitude estimate
         * that has actually seen the accelerometer, and an aircraft that is
         * the right way up. The gates are in one function because the console
         * and the preflight report ask the same question - "would it arm if I
         * asked?" - and a second copy of this list is a second answer.
         */
        int ready = ak_flight_arm_check(flight, &cmd, 0) == AK_ARM_OK
                    && flight->arm_innov_us >= AK_ARM_INNOV_US;
        if (ready) {
            if (!flight->arm_hold_active) {
                flight->arm_hold_active = 1;
                flight->arm_hold_started_ms = now_ms;
            } else if ((uint32_t)(now_ms - flight->arm_hold_started_ms) >=
                       flight->cfg.arm_hold_ms) {
                flight->state = AK_FLIGHT_ARMED;
                flight->arm_hold_active = 0;
                flight->arm_released = 0;
            }
        } else {
            flight->arm_hold_active = 0;
        }
        return;
    }

    /* Armed, with the pilot's link up. The arm switch comes first, because
     * nothing below it is a reason to stay armed. */
    if (!cmd.arm_request) {
        go_safe(flight, AK_FLIGHT_DISARMED);
        return;
    }

    if (flight->state == AK_FLIGHT_RTH) {
        /* The link is back, so the pilot has it: the return is over. */
        flight->state = AK_FLIGHT_ARMED;
    }

    if (flight->managed && flight->guidance != 0) {
        /* Handed over on purpose, with the sticks still connected. This is the
         * difference between an autopilot and a failsafe: the link is up and
         * the navigator is flying anyway, because somebody asked it to. */
        flight->state = AK_FLIGHT_MANAGED;
        control_armed(flight, flight->guidance, dt);
        return;
    }

    if (flight->state == AK_FLIGHT_MANAGED) {
        /* The pilot has taken it back. */
        flight->state = AK_FLIGHT_ARMED;
    }

    control_armed(flight, &cmd, dt);
}

const ak_outputs_t *ak_flight_outputs(const ak_flight_t *flight)
{
    return &flight->out;
}

ak_flight_state_t ak_flight_state(const ak_flight_t *flight)
{
    return flight->state;
}

uint32_t ak_flight_last_loop_us(const ak_flight_t *flight)
{
    return flight->last_loop_us;
}

int ak_flight_config_writable(const ak_flight_t *flight)
{
    /* Disarmed is the only state in which the configuration may be written,
     * and the reason is not tidiness. The write goes to a flash sector, and a
     * ring that has been round erases the whole sector before programming the
     * new record - tens of milliseconds during which the core is stopped. An
     * aircraft in the air cannot afford that, and the configuration being
     * saved is the one currently holding it up.
     *
     * Every other state is flying or about to be: armed, failsafe, returning
     * home, on autopilot. A latched failsafe and an autopilot that has taken
     * over are both cases where the pilot is *not* in a position to be told
     * "saved 14 bytes" and expected to understand what that meant. */
    return flight->state == AK_FLIGHT_DISARMED;
}

/* One place spells the state, so the console, the preflight report and the
 * blackbox cannot disagree about what the aircraft was doing. */
const char *ak_flight_state_name(ak_flight_state_t state)
{
    switch (state) {
    case AK_FLIGHT_ARMED:
        return "armed";
    case AK_FLIGHT_FAILSAFE:
        return "failsafe";
    case AK_FLIGHT_RTH:
        return "returning home";
    case AK_FLIGHT_MANAGED:
        /* Not "on mission": a fence return is the navigator flying an aircraft
         * whose pilot is still there, and calling that a mission would be a
         * state name that lies about who decided. */
        return "on autopilot";
    case AK_FLIGHT_DESCEND:
        /* The pilot is gone and nobody is flying it, so what the console says
         * has to be what the aircraft is actually doing: circling down. */
        return "circling down";
    case AK_FLIGHT_DISARMED:
    default:
        return "disarmed";
    }
}

void ak_flight_set_guidance(ak_flight_t *flight, const ak_rc_command_t *guidance)
{
    flight->guidance = guidance;
}

void ak_flight_set_managed(ak_flight_t *flight, int on)
{
    flight->managed = on ? 1 : 0;
}

int ak_flight_managed(const ak_flight_t *flight)
{
    return flight->managed;
}

int ak_flight_link_live(const ak_flight_t *flight)
{
    return flight->link_live;
}

/* --- the tunable table ---------------------------------------------------- */

#define add_float(items, n, ...) ak_params_add_float(items, n, __VA_ARGS__)
#define add_u32(items, n, ...)   ak_params_add_u32(items, n, __VA_ARGS__)

unsigned ak_flight_param_table(ak_flight_t *flight, ak_param_t *items,
                               unsigned max_items)
{
    ak_flight_config_t *cfg = &flight->cfg;
    unsigned n = 0;

    /* Call this straight after ak_flight_init: whatever the fields hold then
     * becomes what `defaults` restores. */
    if (max_items < AK_FLIGHT_PARAM_COUNT) {
        return 0;
    }

    n = add_float(items, n, "rate_kp_roll", "rate loop P, roll",
                  &cfg->rate_kp[0], 3, 0.0f, 3.0f, AK_PARAM_GROUP_RATES);
    n = add_float(items, n, "rate_kp_pitch", "rate loop P, pitch",
                  &cfg->rate_kp[1], 3, 0.0f, 3.0f, AK_PARAM_GROUP_RATES);
    n = add_float(items, n, "rate_kp_yaw", "rate loop P, yaw",
                  &cfg->rate_kp[2], 3, 0.0f, 3.0f, AK_PARAM_GROUP_RATES);
    n = add_float(items, n, "rate_ki_roll", "rate loop I, roll",
                  &cfg->rate_ki[0], 3, 0.0f, 2.0f, AK_PARAM_GROUP_RATES);
    n = add_float(items, n, "rate_ki_pitch", "rate loop I, pitch",
                  &cfg->rate_ki[1], 3, 0.0f, 2.0f, AK_PARAM_GROUP_RATES);
    n = add_float(items, n, "rate_ki_yaw", "rate loop I, yaw",
                  &cfg->rate_ki[2], 3, 0.0f, 2.0f, AK_PARAM_GROUP_RATES);
    n = add_float(items, n, "rate_kd_roll", "rate loop D, roll",
                  &cfg->rate_kd[0], 4, 0.0f, 0.05f, AK_PARAM_GROUP_RATES);
    n = add_float(items, n, "rate_kd_pitch", "rate loop D, pitch",
                  &cfg->rate_kd[1], 4, 0.0f, 0.05f, AK_PARAM_GROUP_RATES);
    n = add_float(items, n, "rate_kd_yaw", "rate loop D, yaw",
                  &cfg->rate_kd[2], 4, 0.0f, 0.05f, AK_PARAM_GROUP_RATES);
    n = add_float(items, n, "rate_i_limit", "clamp on the integral term",
                  &cfg->rate_i_limit, 3, 0.0f, 1.0f, AK_PARAM_GROUP_RATES);
    n = add_float(items, n, "torque_limit", "clamp on each axis before the mix",
                  &cfg->torque_limit, 3, 0.05f, 1.0f, AK_PARAM_GROUP_RATES);
    /*
     * The gyro chain and the D-term chain. Named after Betaflight's parameters
     * so that tuning knowledge transfers by name - which costs a rename, and
     * `gyro_lpf_hz` and `d_cutoff_hz` are kept loadable by the migration table
     * in ak_params.c so that a configuration saved by an earlier build comes
     * up with its filters rather than without them.
     *
     * The ranges are the reference's: LPF_MAX_HZ 1000 for a static cutoff and
     * DYN_LPF_MAX_HZ 1000 for a dynamic bound (gyro.h:39-40 in the pinned
     * source). It is deliberately not the 500 the old `d_cutoff_hz` carried -
     * a D-term filter above 500 on an 8 kHz gyro is unusual but legal, and the
     * library clamps a cutoff that this sample rate cannot represent rather
     * than the table refusing it, so a person can see the clamp rather than a
     * range error they cannot explain.
     *
     * Every one of these is 0-disables. `*_dyn_min_hz` equal to
     * `*_dyn_max_hz` is how a person asks for no dynamism, which is also what
     * the reference means by it.
     */
    n = add_float(items, n, "gyro_lpf1_static_hz",
                  "gyro chain 1st filter, 0 disables",
                  &cfg->gyro_lpf1_static_hz, 0, 0.0f, 1000.0f,
                  AK_PARAM_GROUP_RATES);
    n = add_float(items, n, "gyro_lpf1_dyn_min_hz",
                  "gyro 1st filter cutoff at zero throttle",
                  &cfg->gyro_lpf1_dyn_min_hz, 0, 0.0f, 1000.0f,
                  AK_PARAM_GROUP_RATES);
    n = add_float(items, n, "gyro_lpf1_dyn_max_hz",
                  "gyro 1st filter cutoff at full throttle",
                  &cfg->gyro_lpf1_dyn_max_hz, 0, 0.0f, 1000.0f,
                  AK_PARAM_GROUP_RATES);
    n = add_float(items, n, "gyro_lpf2_static_hz",
                  "gyro chain 2nd filter, 0 disables",
                  &cfg->gyro_lpf2_static_hz, 0, 0.0f, 1000.0f,
                  AK_PARAM_GROUP_RATES);
    /*
     * The dynamic notch, named and ranged after the reference's four, because
     * a person tuning this aircraft has almost certainly tuned that one - and a
     * name that transfers tuning knowledge has to transfer its units too, which
     * is why `dyn_notch_q` is 300 for Q 3 here exactly as it is there. See
     * ak_dyn_notch.h's decision 11 for why the awkward unit is kept.
     *
     * The ranges are the reference's own settings ranges
     * (`cli/settings.c`: Q 1..1000, min 20..250, max 200..1000), except that
     * the count stops at AK_DYN_NOTCH_COUNT_MAX - the roadmap's 1..5 - rather
     * than the reference's 7.
     *
     * They are ranges of *requests*, and one combination inside them is not a
     * configuration: `dyn_notch_min_hz` 250 with `dyn_notch_max_hz` 200 is a
     * band with no width, which the module refuses. That is reachable, so it is
     * reported rather than silently ignored - see report_dyn_notch() in
     * main.c and `ak_flight_dyn_notch_report`, whose AK_DYN_NOTCH_OFF_BAND is
     * the sentence for it.
     *
     * The count's help says what the whole group rests on, because at the loop
     * rate every board in this tree boots at the answer is "nothing": a biquad
     * notch cannot sit above a quarter of its sample rate, so the bank is off
     * under a 2 kHz loop and the parameter keeps its value for the day a person
     * raises the loop to meet it.
     */
    n = add_u32(items, n, "dyn_notch_count",
                "tracked notches per axis, 0 disables; needs a loop at 2 kHz or "
                "faster",
                &cfg->dyn_notch_count, 0u, (uint32_t)AK_DYN_NOTCH_COUNT_MAX,
                AK_PARAM_GROUP_RATES);
    n = add_float(items, n, "dyn_notch_q",
                  "notch Q in hundredths: 300 is Q 3",
                  &cfg->dyn_notch_q, 0, 1.0f, 1000.0f, AK_PARAM_GROUP_RATES);
    n = add_float(items, n, "dyn_notch_min_hz",
                  "lowest frequency the peak search looks at",
                  &cfg->dyn_notch_min_hz, 0, 20.0f, 250.0f,
                  AK_PARAM_GROUP_RATES);
    n = add_float(items, n, "dyn_notch_max_hz",
                  "highest frequency the peak search looks at",
                  &cfg->dyn_notch_max_hz, 0, 200.0f, 1000.0f,
                  AK_PARAM_GROUP_RATES);
    n = add_float(items, n, "dterm_lpf1_static_hz",
                  "D-term chain 1st filter, 0 disables",
                  &cfg->dterm_lpf1_static_hz, 0, 0.0f, 1000.0f,
                  AK_PARAM_GROUP_RATES);
    n = add_float(items, n, "dterm_lpf1_dyn_min_hz",
                  "D-term 1st filter cutoff at zero throttle",
                  &cfg->dterm_lpf1_dyn_min_hz, 0, 0.0f, 1000.0f,
                  AK_PARAM_GROUP_RATES);
    n = add_float(items, n, "dterm_lpf1_dyn_max_hz",
                  "D-term 1st filter cutoff at full throttle",
                  &cfg->dterm_lpf1_dyn_max_hz, 0, 0.0f, 1000.0f,
                  AK_PARAM_GROUP_RATES);
    n = add_float(items, n, "dterm_lpf2_static_hz",
                  "D-term chain 2nd filter, 0 disables",
                  &cfg->dterm_lpf2_static_hz, 0, 0.0f, 1000.0f,
                  AK_PARAM_GROUP_RATES);
    n = add_float(items, n, "arm_accel_lpf_hz",
                  "accel low pass for the arming gate only, 0 disables",
                  &cfg->arm_accel_lpf_hz, 0, 0.0f, 1000.0f, AK_PARAM_GROUP_ARMING);
    n = add_float(items, n, "max_rate_dps", "rate mode, deg/s at full stick",
                  &cfg->max_rate_dps, 0, 10.0f, 1800.0f, AK_PARAM_GROUP_RATES);
    n = add_float(items, n, "angle_kp_roll", "angle loop P, roll",
                  &cfg->angle_kp[0], 2, 0.0f, 20.0f, AK_PARAM_GROUP_ANGLE);
    n = add_float(items, n, "angle_kp_pitch", "angle loop P, pitch",
                  &cfg->angle_kp[1], 2, 0.0f, 20.0f, AK_PARAM_GROUP_ANGLE);
    n = add_float(items, n, "angle_rate_lim", "clamp on the angle loop output",
                  &cfg->angle_rate_limit, 2, 0.5f, 20.0f, AK_PARAM_GROUP_ANGLE);
    n = add_float(items, n, "max_tilt_deg", "max commanded tilt in angle mode",
                  &cfg->max_tilt_deg, 0, 5.0f, 80.0f, AK_PARAM_GROUP_ANGLE);
    n = add_float(items, n, "yaw_rate_dps", "deg/s at full rudder",
                  &cfg->yaw_rate_dps, 0, 10.0f, 1000.0f, AK_PARAM_GROUP_ANGLE);
    n = add_float(items, n, "throttle_low", "arm only below this throttle",
                  &cfg->throttle_low, 3, 0.0f, 0.5f, AK_PARAM_GROUP_ARMING);
    n = add_float(items, n, "arm_max_tilt_deg",
                  "refuse to arm further than this from level; 180 is the sky",
                  &cfg->arm_max_tilt_deg, 1, 0.0f, 180.0f, AK_PARAM_GROUP_ARMING);
    n = add_u32(items, n, "arm_hold_ms", "switch held this long to arm",
                &cfg->arm_hold_ms, 0u, 5000u, AK_PARAM_GROUP_ARMING);
    n = add_u32(items, n, "rc_timeout_ms", "no RC frame this long is a failsafe",
                &cfg->rc_timeout_ms, 50u, 2000u, AK_PARAM_GROUP_RECEIVER);
    /* Not "longer than this is a sensor problem" any more, which is what it
     * said while a longer gap was replaced by a single millisecond. A longer
     * gap is now integrated in pieces of this size, up to a fixed budget, so
     * this bounds one integration *step* rather than the interval the loop will
     * accept - see ak_flight_timing_t. */
    n = add_u32(items, n, "max_dt_ms", "the longest one integration step may span",
                &cfg->max_dt_ms, 5u, 200u, AK_PARAM_GROUP_TIMING);
    /*
     * A fixed wing's lost-link descent: see descend_command(). The bank is how
     * wide the circle is, the pitch is how fast it comes down, and the
     * throttle is what keeps the elevons flying - a wing at zero throttle is a
     * wing with no control surfaces.
     */
    n = add_float(items, n, "wing_descend_bank_deg",
                  "fixed wing, lost link: bank held while circling down",
                  &cfg->wing_descend_bank_deg, 0, 0.0f, 60.0f, AK_PARAM_GROUP_FAILSAFE);
    n = add_float(items, n, "wing_descend_pitch_deg",
                  "fixed wing, lost link: nose-down held while circling down",
                  &cfg->wing_descend_pitch_deg, 0, 0.0f, 30.0f, AK_PARAM_GROUP_FAILSAFE);
    n = add_float(items, n, "wing_descend_throttle",
                  "fixed wing, lost link: throttle held while circling down",
                  &cfg->wing_descend_throttle, 2, 0.0f, 0.60f, AK_PARAM_GROUP_FAILSAFE);
    n = add_u32(items, n, "airframe",
                "0 quadx, 1 elevonwing, 2 quadx1234, 3 quadp, 4 y4, 5 vtail4, "
                "6 tri, 7 elevonwingsingle (no rudder) - see docs/07-outputs.md",
                &flight->airframe, 0u, AK_MIXER_AIRFRAMES - 1u, AK_PARAM_GROUP_AIRFRAME);
    /* Receiver calibration lives here rather than in a table of its own: it is
     * configuration, it has to survive a reboot, and the console, the stored
     * file and the protocol all already know how to read it. */
    n = add_u32(items, n, "rc_min", "receiver count at full low stick",
                &flight->rc_cfg.min, 0u, 500u, AK_PARAM_GROUP_RECEIVER);
    n = add_u32(items, n, "rc_mid", "receiver count with the sticks centred",
                &flight->rc_cfg.mid, 500u, 1500u, AK_PARAM_GROUP_RECEIVER);
    n = add_u32(items, n, "rc_max", "receiver count at full high stick",
                &flight->rc_cfg.max, 1200u, 2500u, AK_PARAM_GROUP_RECEIVER);
    n = add_float(items, n, "rc_deadband", "stick deadband, fraction of travel",
                  &flight->rc_cfg.deadband, 3, 0.0f, 0.2f, AK_PARAM_GROUP_RECEIVER);
    /* Which channel arms and which selects angle mode. ExpressLRS and the
     * custom ESP-NOW link both put the arm switch on CH5 (AUX1): set
     * `arm_channel 5` and `mode_channel 6` for either. See ak_rc.h. */
    n = add_u32(items, n, "arm_channel",
                "receiver channel that arms (5 = AUX1; ELRS and the custom "
                "radio arm on 5)",
                &flight->rc_cfg.arm_channel, AK_RC_FIRST_AUX_CHANNEL,
                AK_RC_CHANNELS, AK_PARAM_GROUP_RECEIVER);
    n = add_u32(items, n, "mode_channel",
                "receiver channel that selects angle mode (high = angle)",
                &flight->rc_cfg.mode_channel, AK_RC_FIRST_AUX_CHANNEL,
                AK_RC_CHANNELS, AK_PARAM_GROUP_RECEIVER);

    return n;
}
