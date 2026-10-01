#include "ak_flight.h"

#include "ak_math.h"
#include "ak_params.h"

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
    cfg->d_cutoff_hz = 80.0f;
    /*
     * **Off, and that is a decision the checks made.**
     *
     * The reference implementation flying this airframe runs its gyro through
     * a 250 Hz anti-aliasing low pass (`gyro_anti_aliasing_lpf_hz`, in
     * projects/twin-wings/ghf435-inav/), so 250 was this parameter's first
     * default. The loop's own closed-loop check said no: with the filter in
     * front of it, the aircraft in that check - a plant with an actuator time
     * constant and the gains below - stopped settling and started hunting,
     * because a first-order 250 Hz filter at a 1 kHz loop is about 0.6 ms of
     * lag in the *proportional* path. That is a real trade (the reference's
     * loop is tuned *with* its filter, and its filter is sharper), and the
     * honest default for an untuned aircraft is the one with more phase
     * margin: off, with `set gyro_lpf_hz 250` the first thing to try once a
     * flight's log shows the noise it would remove.
     */
    cfg->gyro_lpf_hz = 0.0f;

    /* Off by default, for the same reason the gyro filter is: the number that
     * makes this one worth turning on is a vibration amplitude, and there is no
     * board on the bus and no measured spectrum to take it from. The 50 Hz this
     * tree's records sized is where a real airframe would start, not a value
     * anyone has flown. Off means the gate reads the sample itself, which is
     * exactly what it did before this field existed. */
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
    if (!flight->link_live) {
        return AK_ARM_NO_LINK;
    }
    if (flight->state == AK_FLIGHT_FAILSAFE) {
        return AK_ARM_FAILSAFE;
    }
    if (cmd == 0 || !cmd->arm_request) {
        return AK_ARM_NOT_REQUESTED;
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
#define AK_ARM_INNOV_MS  1u

void ak_flight_init(ak_flight_t *flight, const ak_mixer_t *mixer)
{
    ak_flight_default_config(&flight->cfg);
    ak_rc_default_config(&flight->rc_cfg);
    ak_estimator_init(&flight->est, 0.5f);
    flight->arm_innov_ms = 0;

    /* Zeroed explicitly rather than left to the caller's memory, and marked
     * unprimed so the first usable sample loads the filter instead of being
     * averaged against a zero that no accelerometer would ever read. */
    flight->arm_accel_primed = 0;
    for (int i = 0; i < 3; i++) {
        flight->arm_accel[i] = 0.0f;
    }

    for (int i = 0; i < 3; i++) {
        ak_pid_init(&flight->rate[i], flight->cfg.rate_kp[i],
                    flight->cfg.rate_ki[i], flight->cfg.rate_kd[i],
                    flight->cfg.rate_i_limit, flight->cfg.torque_limit,
                    flight->cfg.d_cutoff_hz);
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
    flight->last_imu_ms = 0;
    flight->have_last_imu = 0;
    flight->last_step_ms = 0;
    flight->have_last_step = 0;
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
                    flight->cfg.d_cutoff_hz);
    }
}

/* The bound on one integration step, with a guard for a configuration that has
 * somehow been given zero: the parameter table will not take it, but a division
 * by a zero bound is a long way from where the mistake was made. */
static uint32_t step_bound_ms(const ak_flight_t *flight)
{
    return flight->cfg.max_dt_ms != 0u ? flight->cfg.max_dt_ms : AK_FLIGHT_LOOP_MS;
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
 * Anything past the budget is counted in dropped_ms rather than hidden.
 *
 * The return is a plan rather than a duration because the caller has to run the
 * gyro filter and the estimator once per piece. substeps == 0 means "integrate
 * nothing this call", which is the honest answer both for a duplicate sample
 * and for a timestamp that went backwards.
 */
typedef struct {
    uint32_t elapsed_ms;    /* what the source's timestamp says happened */
    uint32_t integrate_ms;  /* how much of it this call will integrate */
    uint32_t substeps;      /* in how many pieces, each at most the bound */
} ak_step_plan_t;

static ak_step_plan_t plan_step(ak_flight_t *flight, const ak_imu_sample_t *imu)
{
    const uint32_t bound = step_bound_ms(flight);
    const uint32_t budget = bound * AK_FLIGHT_MAX_CATCHUP;
    ak_step_plan_t plan;

    /*
     * No predecessor: this is the first sample, and the only honest answer for
     * the interval before it is one control period. There is no earlier
     * timestamp to subtract from, and inventing a long one would have the
     * estimator jump on power-up.
     */
    if (!flight->have_last_imu) {
        flight->last_imu_ms = imu->time_ms;
        flight->have_last_imu = 1;
        plan.elapsed_ms = AK_FLIGHT_LOOP_MS;
        plan.integrate_ms = AK_FLIGHT_LOOP_MS;
        plan.substeps = 1u;
        return plan;
    }

    const uint32_t delta = imu->time_ms - flight->last_imu_ms;

    /*
     * A timestamp that went backwards is a different clock, not a very long
     * gap. Unsigned subtraction reports a hundred milliseconds backwards as
     * nearly 2^32 milliseconds forwards, and the two cases want opposite
     * answers: a gap is time the aircraft really flew and must be integrated, a
     * clock change is time nobody can account for and must not be. Told apart
     * before the arithmetic rather than after it.
     */
    if (delta > AK_FLIGHT_CLOCK_MAX_MS) {
        flight->timing.clock_resets++;
        flight->last_imu_ms = imu->time_ms;
        plan.elapsed_ms = 0u;
        plan.integrate_ms = 0u;
        plan.substeps = 0u;
        return plan;
    }

    flight->last_imu_ms = imu->time_ms;

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
        flight->timing.dropped_ms += delta;
        plan.elapsed_ms = delta;
        plan.integrate_ms = 0u;
        plan.substeps = 0u;
        return plan;
    }

    /* The same timestamp twice: a frozen sensor, or a loop polling faster than
     * its gyro. Nothing elapsed, so nothing is integrated - and unlike the old
     * clamp, nothing is invented either. */
    if (delta == 0u) {
        flight->timing.duplicates++;
        plan.elapsed_ms = 0u;
        plan.integrate_ms = 0u;
        plan.substeps = 0u;
        return plan;
    }

    plan.elapsed_ms = delta;
    plan.integrate_ms = delta;

    if (delta > bound) {
        flight->timing.gap_steps++;
    }

    if (plan.integrate_ms > budget) {
        /* More than one call can catch up on. Integrating it all would make
         * this iteration longer than the stall it is recovering from, which is
         * how a loop that is behind stays behind. The surplus is counted. */
        flight->timing.dropped_ms += plan.integrate_ms - budget;
        plan.integrate_ms = budget;
    }

    plan.substeps = (plan.integrate_ms + bound - 1u) / bound;
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
 * should be told; long_loops and max_loop_ms record that it happened.
 */
static float loop_dt(ak_flight_t *flight, uint32_t now_ms)
{
    uint32_t delta = AK_FLIGHT_LOOP_MS;

    if (flight->have_last_step) {
        const uint32_t measured = now_ms - flight->last_step_ms;
        if (measured != 0u && measured <= AK_FLIGHT_CLOCK_MAX_MS) {
            delta = measured;
        }
    }
    flight->last_step_ms = now_ms;
    flight->have_last_step = 1;

    if (delta > step_bound_ms(flight)) {
        flight->timing.long_loops++;
    }
    if (delta > flight->timing.max_loop_ms) {
        flight->timing.max_loop_ms = delta;
    }
    return (float)delta / 1000.0f;
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
        flight->torque[i] = ak_pid_update(&flight->rate[i],
                                          flight->rate_setpoint[i],
                                          flight->gyro[i], dt);
    }

    /* Armed, so the mixer takes care of the authority limit: what the control
     * loop asked for, reduced only as far as it has to be to fit. */
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
                    const ak_rc_input_t *rc, uint32_t now_ms)
{
    flight->steps++;

    /* The control law's interval, and the sensor's interval. Two clocks; see
     * loop_dt() for why they are not the same number. */
    float dt = loop_dt(flight, now_ms);
    ak_step_plan_t plan = plan_step(flight, imu);

    /*
     * The gyro, filtered once per integration piece, before anything reads it.
     *
     * Everything downstream - the estimator's propagation and the three rate
     * loops - uses `flight->gyro`, which is the driver's sample through a
     * first-order low pass at `gyro_lpf_hz` (250 by default, the reference's
     * own number for this aircraft, and 0 disables it). What that buys is not
     * "smoother flight": it is that the proportional term multiplies the
     * *filtered* rate rather than the sensor's noise, so the motor commands do
     * not carry it. Without this the only filter in the loop was on the
     * derivative, which is the one term whose filtering cannot help the
     * others.
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
     * the interval it covers is already counted in dropped_ms - there is no
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
        ak_imu_sample_t filtered = *imu;
        const float sub_dt =
            (float)plan.integrate_ms / (float)plan.substeps / 1000.0f;

        for (uint32_t piece = 0; piece < plan.substeps; piece++) {
            if (flight->cfg.gyro_lpf_hz > 0.0f) {
                float alpha = ak_lpf_alpha(flight->cfg.gyro_lpf_hz, sub_dt);
                for (int i = 0; i < 3; i++) {
                    flight->gyro[i] += alpha * (imu->gyro[i] - flight->gyro[i]);
                    filtered.gyro[i] = flight->gyro[i];
                }
            } else {
                for (int i = 0; i < 3; i++) {
                    flight->gyro[i] = imu->gyro[i];
                }
            }

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
         * The interval is `integrate_ms`, the same interval the estimator
         * integrates, so a step the catch-up budget clamped filters over the
         * time that was actually accounted for rather than the time that
         * elapsed.
         */
        if (flight->cfg.arm_accel_lpf_hz > 0.0f) {
            const float adt = (float)plan.integrate_ms / 1000.0f;
            if (!flight->arm_accel_primed) {
                for (int i = 0; i < 3; i++) {
                    flight->arm_accel[i] = imu->accel[i];
                }
                flight->arm_accel_primed = 1;
            } else {
                const float alpha =
                    ak_lpf_alpha(flight->cfg.arm_accel_lpf_hz, adt);
                for (int i = 0; i < 3; i++) {
                    flight->arm_accel[i] +=
                        alpha * (imu->accel[i] - flight->arm_accel[i]);
                }
            }
        } else {
            for (int i = 0; i < 3; i++) {
                flight->arm_accel[i] = imu->accel[i];
            }
        }

        if (ak_estimator_innovation_deg(&flight->est, flight->arm_accel) <=
            AK_ARM_INNOV_DEG) {
            if (flight->arm_innov_ms < 1000000u) flight->arm_innov_ms++;
        } else {
            flight->arm_innov_ms = 0;
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

    if (!rc_live || !imu->valid) {
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
        if (!imu->valid) {
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
                    && flight->arm_innov_ms >= AK_ARM_INNOV_MS;
        if (ready) {
            if (!flight->arm_hold_active) {
                flight->arm_hold_active = 1;
                flight->arm_hold_started_ms = now_ms;
            } else if ((uint32_t)(now_ms - flight->arm_hold_started_ms) >=
                       flight->cfg.arm_hold_ms) {
                flight->state = AK_FLIGHT_ARMED;
                flight->arm_hold_active = 0;
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
    n = add_float(items, n, "d_cutoff_hz", "derivative low pass, 0 disables",
                  &cfg->d_cutoff_hz, 0, 0.0f, 500.0f, AK_PARAM_GROUP_RATES);
    n = add_float(items, n, "gyro_lpf_hz",
                  "gyro low pass, 0 disables - the reference's 250",
                  &cfg->gyro_lpf_hz, 0, 0.0f, 1000.0f, AK_PARAM_GROUP_RATES);
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

    return n;
}
