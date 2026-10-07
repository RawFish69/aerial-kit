#include "filter/ak_rpm_filter.h"

#include "flight/ak_math.h"

/*
 * The bank, its arithmetic, and the one law that turns a wire's number into a
 * frequency. The reasoning is in the header; what is here is the code that
 * makes it true, and the comments are about the parts of it that are easy to
 * get subtly wrong.
 *
 * There is no libm in the image (ground rule 8, `-nostdlib`), so there is no
 * `ceilf` here and no `atanf`: the two places that reach for one use integer
 * arithmetic and ak_atan2f instead.
 */

/*
 * The poles are the motor's, and the arithmetic that turns eRPM into a
 * mechanical frequency is the reverse of `ak_dshot_rpm`'s:
 *
 *     rpm = erpm * 2 / poles        (ak_dshot_gcr.h)
 *     hz  = rpm / 60 = erpm / (30 * poles)
 *
 * so this is 1 / (30 * poles), and not the arithmetically identical
 * 1 / (60 * poles / 2), which reads as though a factor of two might belong
 * somewhere it does not.
 *
 * Zero for a pole count below two: not a motor. That is `ak_dshot_rpm`'s own
 * rule and it is the same number - `ak_dshot_rpm` refuses a pole count below
 * two for the same reason, and the two must not disagree about what a motor is.
 */
static float erpm_to_hz_for(unsigned poles)
{
    if (poles < 2u) {
        return 0.0f;
    }
    return 1.0f / (30.0f * (float)poles);
}

/*
 * Build one section at `centre_hz` with depth `weight`.
 *
 * `f` is the reference's `sn / cs` from `sincosf_approx(M_PIf * filterFreq *
 * dt)`, `q` is `1 / Q` and not Q, and the three coefficients are its `a1`,
 * `a2` and `wq`.
 *
 * **The division by `cs` is safe because `ak_rpm_filter_init` refuses a band
 * with no width.** `centre_hz` is always inside `[min_hz, max_hz]` - `notch_aim`
 * is the only caller and it clamps into that band - and `max_hz` is
 * `0.48 / dt`, so the argument of the cosine is at most `0.48 * pi`: a cosine
 * of 0.0628 and a tangent of 15.89, at every loop rate, because the ceiling is
 * a fraction of the rate and the rate cancels. A `min_hz` at or above `max_hz`
 * would break that, which is why it is refused rather than clamped: see the
 * `AK_RPM_FILTER_OFF_BAND` branch below. Without that refusal a person could
 * configure a 5 kHz floor on an 8 kHz loop and every notch in the bank would be
 * built from a negative cosine - a filter that alternates instead of filtering,
 * with no part of the arithmetic reporting a problem.
 *
 * The state is deliberately *not* touched here: moving a notch's centre by a few
 * hertz must not restart it, or every update would put a step into the gyro.
 * That is the same contract `ak_filter_biquad_notch_set` keeps, and the reason
 * the reference has separate `rpmNotchInit` and `rpmNotchUpdate` rather than one
 * function. A section engaging for the first time is started from zero by
 * `ak_rpm_filter_init`, because there is nothing to preserve.
 */
static void notch_build(ak_rpm_notch_t *n, float centre_hz, float dt, float q_real,
                        float weight)
{
    const float angle = AK_PI * centre_hz * dt;
    const float f  = ak_sinf(angle) / ak_cosf(angle);
    const float q  = 1.0f / q_real;
    const float a1 = 1.0f / (1.0f + f * (f + q));

    n->f  = f;
    n->a1 = a1;
    n->a2 = f * a1;
    n->w  = weight;
    n->wq = q * weight;
}

static void notch_reset(ak_rpm_notch_t *n)
{
    for (int i = 0; i < 3; i++) {
        n->ic1[i] = 0.0f;
        n->ic2[i] = 0.0f;
    }
}

/*
 * Where one motor's `harmonic`-th notch goes, and how deep it is.
 *
 * The frequency law of decision 2 and the fade of decision 3, in one place
 * because they are one decision: the fade is only continuous if the frequency
 * that feeds it has already been clamped, and a caller that clamped afterwards
 * would get a depth that stepped from 0 to 1 the moment the motor crossed the
 * floor.
 *
 * `min_clamps` and `max_clamps` are counted here rather than at the call site
 * because this is the only place that knows a clamp happened.
 */
static void notch_aim(ak_rpm_filter_t *r, uint8_t motor, uint8_t harmonic,
                      float *centre_hz, float *weight)
{
    float f = r->motor_hz[motor] * (float)(harmonic + 1u);
    float w = 1.0f;

    if (f < r->min_hz) {
        f = r->min_hz;
        r->min_clamps++;
    } else if (f > r->max_hz) {
        f = r->max_hz;
        r->max_clamps++;
    }

    /* Decision 3. `f` is at or above `min_hz` by now, so the margin is never
     * negative and the depth never goes below zero on its way in. A `fade_hz` of
     * zero is a step at the floor, which is what a person who set it to zero
     * asked for - the module does not refuse it, it just has nothing to fade
     * over. */
    const float margin = f - r->min_hz;
    if (margin < r->fade_hz) {
        w = r->fade_hz > 0.0f ? (margin / r->fade_hz) : 0.0f;
    }

    *centre_hz = f;
    *weight = w * r->weights[harmonic];
}

void ak_rpm_filter_init(ak_rpm_filter_t *r, float loop_hz, uint8_t motors,
                        unsigned poles, uint8_t harmonics, float min_hz,
                        float fade_hz, float q, float lpf_hz)
{
    /* Zero everything, including every section's state, before deciding whether
     * any of it will be used: an init that left the state of a module it had
     * just disabled is one a caller could enable later with stale history. */
    for (int m = 0; m < AK_RPM_FILTER_MOTOR_MAX; m++) {
        for (int h = 0; h < AK_RPM_FILTER_HARMONIC_MAX; h++) {
            r->notch[m][h].f  = 0.0f;
            r->notch[m][h].a1 = 0.0f;
            r->notch[m][h].a2 = 0.0f;
            r->notch[m][h].wq = 0.0f;
            r->notch[m][h].w  = 0.0f;
            notch_reset(&r->notch[m][h]);
        }
        r->motor_hz[m] = 0.0f;
        r->have[m] = 0u;
        r->invalid[m] = 0u;
        ak_filter_pt1_init(&r->freq_lpf[m], 0.0f, 0.0f);
    }

    r->dt       = 0.0f;
    r->min_hz   = 0.0f;
    r->max_hz   = 0.0f;
    r->fade_hz  = 0.0f;
    r->q        = 0.0f;
    r->erpm_to_hz = 0.0f;
    r->motors   = 0u;
    r->harmonics = 0u;
    r->enabled  = 0u;
    r->off      = (uint8_t)AK_RPM_FILTER_OFF_COUNT;
    r->motor    = 0u;
    r->harmonic = 0u;
    r->updates_per_call = 0u;
    r->updates  = 0u;
    r->passes   = 0u;
    r->min_clamps = 0u;
    r->max_clamps = 0u;

    for (int h = 0; h < AK_RPM_FILTER_HARMONIC_MAX; h++) {
        r->weights[h] = 1.0f;
    }

    if (motors > AK_RPM_FILTER_MOTOR_MAX) {
        motors = AK_RPM_FILTER_MOTOR_MAX;
    }
    if (harmonics > AK_RPM_FILTER_HARMONIC_MAX) {
        harmonics = AK_RPM_FILTER_HARMONIC_MAX;
    }

    /* Every refusal is a separate branch with its own reason, because "why is
     * this not filtering" is the question a caller has and there is more than
     * one answer. `enabled` is the answer and `off` is the sentence. */
    if (motors == 0u || harmonics == 0u) {
        r->motors = motors;
        r->off = (uint8_t)AK_RPM_FILTER_OFF_COUNT;
        return;
    }

    r->dt = loop_hz > 0.0f ? (1.0f / loop_hz) : 0.0f;
    r->max_hz = AK_RPM_FILTER_MAX_RATIO * loop_hz;
    r->erpm_to_hz = erpm_to_hz_for(poles);

    /* `!(min_hz < max_hz)` rather than `min_hz >= max_hz`, so a NaN floor is
     * refused as well - and the `dt > 0` test is what keeps `max_hz` a number,
     * since both it and `dt` are zero when the rate is. A band with no width is
     * not a band, and the reason it is refused rather than squeezed is in
     * `notch_build`: a floor above the ceiling is a notch built from a negative
     * cosine, which is not a filter at all. `fade_hz` may be zero - that is a
     * step - but it may not be negative, and the `!(fade >= 0)` form refuses a
     * NaN fade the same way. */
    if (!(r->dt > 0.0f) || !(min_hz > 0.0f) || !(min_hz < r->max_hz) ||
        !(fade_hz >= 0.0f) || r->erpm_to_hz == 0.0f) {
        r->dt = 0.0f;
        r->max_hz = 0.0f;
        r->motors = motors;
        r->off = (uint8_t)AK_RPM_FILTER_OFF_BAND;
        return;
    }

    r->motors = motors;
    r->harmonics = harmonics;
    r->min_hz = min_hz;
    r->fade_hz = fade_hz;
    r->q = q;

    /* An `lpf_hz` at or below zero is the library's OFF, which is a
     * pass-through, so a person who does not want the frequency smoothed gets
     * the reading unfiltered rather than a filter at some arbitrary cutoff. */
    for (int m = 0; m < motors; m++) {
        ak_filter_pt1_init(&r->freq_lpf[m], lpf_hz, r->dt);
    }

    /* Decision 9. The reference's own arithmetic -
     * `notchUpdatesPerIteration = ceil(notches / (deadline / period))` - with
     * the ceiling written as a truncation and a correction, because there is no
     * `ceilf` in the image. The ceiling is what makes the bank finish *inside*
     * the deadline rather than on it, and on a fast loop the quotient is below
     * one, where the ceiling is what turns it into "one per call" rather than
     * "none, ever" - which is why the floor of one below is not a formality. */
    const unsigned notches = (unsigned)motors * (unsigned)harmonics;
    const float period_us = r->dt * 1000000.0f;
    const float per_deadline = period_us > 0.0f
        ? ((float)AK_RPM_FILTER_UPDATE_US / period_us)
        : (float)notches;
    unsigned per_call = (unsigned)((float)notches / per_deadline);
    if ((float)per_call * per_deadline < (float)notches) {
        per_call++;
    }
    if (per_call == 0u) {
        per_call = 1u;
    }
    if (per_call > notches) {
        per_call = notches;
    }
    r->updates_per_call = (uint8_t)per_call;

    /* Every section is built once at the floor with no depth, so that a bank
     * that has never been updated is a bank of bypassed sections in a defined
     * state rather than of uninitialised coefficients. `ak_rpm_filter_apply`
     * would pass its input through either way - the depth is zero - but the
     * state has to be a number. */
    for (int m = 0; m < motors; m++) {
        for (int h = 0; h < harmonics; h++) {
            notch_build(&r->notch[m][h], r->min_hz, r->dt, r->q, 0.0f);
            notch_reset(&r->notch[m][h]);
        }
    }

    r->enabled = 1u;
    r->off = (uint8_t)AK_RPM_FILTER_RUNNING;
}

void ak_rpm_filter_set_weight(ak_rpm_filter_t *r, uint8_t harmonic, float weight)
{
    if (harmonic >= AK_RPM_FILTER_HARMONIC_MAX) {
        return;
    }
    if (!(weight >= 0.0f) || weight > 1.0f) {
        /* NaN lands here too, and that is the point of writing the test as
         * `!(weight >= 0)` rather than `weight < 0`: a NaN compares false
         * either way, so only this form refuses it. A NaN weight would reach
         * `wq` and from there every axis's state. */
        return;
    }
    r->weights[harmonic] = weight;
}

void ak_rpm_filter_set_erpm(ak_rpm_filter_t *r, uint8_t motor, uint32_t erpm,
                            int valid)
{
    if (motor >= AK_RPM_FILTER_MOTOR_MAX) {
        return;
    }

    if (!valid) {
        /* Decision 5: hold. Counted even when the module is off, so that a
         * motor whose telemetry has gone quiet is visible from a build that was
         * configured not to filter - which is exactly the build someone is
         * looking at when they are asking whether filtering was worth having.
         * Nothing else is done with it, which is what "hold" means. */
        r->invalid[motor]++;
        return;
    }

    if (r->enabled == 0u) {
        /* Nothing to move, and nothing to record it against: a disabled module
         * has no `erpm_to_hz` to convert with. The reading is dropped rather
         * than kept, because there is no way to enable this module later that
         * does not go through `ak_rpm_filter_init` and start the bank over. */
        return;
    }

    const float hz = (float)erpm * r->erpm_to_hz;

    /* The first reading primes the low-pass instead of walking it up from zero.
     * The reference does not do this - its `motorFreqLpf` starts at zero and a
     * first reading ramps in over a few milliseconds - and the difference is
     * worth having: the ramp is not a filter doing its job, it is a filter
     * recovering from a value nobody measured, and at 100 Hz on a 1 kHz loop it
     * is 20 ms of the notch fading in from the wrong place. After the first
     * sample the low-pass is the low-pass. */
    if (r->have[motor] == 0u) {
        ak_filter_pt1_prime(&r->freq_lpf[motor], hz);
        r->motor_hz[motor] = hz;
        r->have[motor] = 1u;
        return;
    }

    r->motor_hz[motor] = ak_filter_pt1_apply(&r->freq_lpf[motor], hz);
}

unsigned ak_rpm_filter_update(ak_rpm_filter_t *r)
{
    if (r->enabled == 0u) {
        return 0u;
    }

    unsigned done = 0u;

    for (unsigned i = 0u; i < r->updates_per_call; i++) {
        const uint8_t m = r->motor;
        const uint8_t h = r->harmonic;

        /* Decision 8: a harmonic configured at zero weight is a harmonic nobody
         * asked for, and rebuilding its coefficients every deadline to multiply
         * by zero is work with no output. It is skipped here and in `apply`, and
         * both skips read the same field. */
        if (r->weights[h] > 0.0f) {
            float centre_hz;
            float weight;
            notch_aim(r, m, h, &centre_hz, &weight);
            notch_build(&r->notch[m][h], centre_hz, r->dt, r->q, weight);
            r->updates++;
            done++;
        }

        /* The pointer advances whether or not this one was updated, so a bank
         * with a zero-weight harmonic still walks all of its sections rather
         * than stopping on the skipped one. */
        r->harmonic = (uint8_t)((r->harmonic + 1u) % r->harmonics);
        if (r->harmonic == 0u) {
            r->motor = (uint8_t)((r->motor + 1u) % r->motors);
        }
    }

    return done;
}

void ak_rpm_filter_apply(ak_rpm_filter_t *r, float gyro[3])
{
    if (r->enabled == 0u) {
        return;
    }

    r->passes++;

    /* Harmonics outermost, motors inside - the reference's order. Every section
     * is linear and they are applied in series, so the composition commutes and
     * the order is a reading of the reference rather than a requirement; it is
     * worth saying so, because "does the order matter" is the first question a
     * reader of a filter bank asks. */
    for (uint8_t h = 0u; h < r->harmonics; h++) {
        if (r->weights[h] <= 0.0f) {
            continue;
        }
        for (uint8_t m = 0u; m < r->motors; m++) {
            ak_rpm_notch_t *n = &r->notch[m][h];
            const float a1 = n->a1;
            const float a2 = n->a2;
            const float f  = n->f;
            const float wq = n->wq;

            for (int i = 0; i < 3; i++) {
                const float ic1 = n->ic1[i];
                const float ic2 = n->ic2[i];
                const float v3 = gyro[i] - ic2;
                const float v1 = a1 * ic1 + a2 * v3;
                const float v2 = ic2 + f * v1;
                n->ic1[i] = 2.0f * v1 - ic1;
                n->ic2[i] = 2.0f * v2 - ic2;
                gyro[i] -= wq * v1;
            }
        }
    }
}

float ak_rpm_filter_notch_hz(const ak_rpm_filter_t *r, uint8_t motor,
                             uint8_t harmonic)
{
    /* `have` and not `enabled`: a bank that has been set up but has never been
     * told a speed has every section parked on the floor with no depth, because
     * that is the state `ak_rpm_filter_init` leaves them in - and reporting
     * "100 Hz" for a motor nobody has measured is a notch the caller does not
     * have. "No reading yet" is the answer to that question. */
    if (r->enabled == 0u || motor >= r->motors || harmonic >= r->harmonics ||
        r->have[motor] == 0u) {
        return 0.0f;
    }

    /* Recovered from the coefficient rather than kept in a field beside it, so
     * that the number reported is the number the section was actually built from
     * and the two cannot drift apart. `ak_atan2f(f, 1)` is `atan(f)`: the board
     * has no libm, and this is what ak_math.h is for. Its stated accuracy is
     * 2e-5 rad, which at a 1 kHz loop is 0.006 Hz and at 8 kHz is 0.05 Hz. */
    return ak_atan2f(r->notch[motor][harmonic].f, 1.0f) / (AK_PI * r->dt);
}

float ak_rpm_filter_notch_weight(const ak_rpm_filter_t *r, uint8_t motor,
                                 uint8_t harmonic)
{
    if (r->enabled == 0u || motor >= r->motors || harmonic >= r->harmonics) {
        return 0.0f;
    }
    return r->notch[motor][harmonic].w;
}
