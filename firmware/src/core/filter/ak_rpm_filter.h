#ifndef AK_CORE_AK_RPM_FILTER_H
#define AK_CORE_AK_RPM_FILTER_H

#include <stdint.h>

#include "filter/ak_filter.h"

/*
 * The notches the motors tell us where to put.
 *
 * ak_dyn_notch.h searches the gyro for vibration and puts a notch wherever it
 * finds a peak. This module does the opposite: it is *told* where the motors
 * are, from the eRPM a bidirectional ESC answers each frame with, and puts a
 * notch there. The two are complements rather than rivals - the dynamic notch
 * finds what nothing predicted, this one removes what the aircraft already
 * knows - and the difference decides everything else about the design.
 *
 * **It has no transform, no noise floor, no peak interpolation and no tracking
 * filter.** All four exist in the dynamic notch to answer "where is the peak",
 * and here the question is answered by a number arriving over a wire. What is
 * left is a frequency, a fold of that frequency into a bank of second-order
 * sections, and a weight.
 *
 * **And it runs at a 1 kHz loop, which the dynamic notch cannot.** That is the
 * practical reason it is worth having at all: the dynamic notch is gated off
 * below 2 kHz (ak_dyn_notch.h, decision 4), because its 64-point window at a
 * 1 kHz loop would resolve 15 Hz and cover 47 ms, and because a notch above a
 * quarter of the sample rate is not representable by the filter library. This
 * module needs neither a window nor the library's biquad - see decision 6 - so
 * a 1 kHz board can have motor notches. What it cannot do is find a resonance
 * the motors do not cause.
 *
 * Five decisions are the reference's, taken from the pinned checkout
 * (`betaflight-2026.6.1 @ 6dbc421`, `src/main/flight/rpm_filter.c` and
 * `src/main/common/filter.c`'s `rpmNotch*`) rather than remembered:
 *
 * 1. **One notch per motor per harmonic, applied to all three axes, not per
 *    axis.** A motor's vibration reaches every gyro axis at the same frequency,
 *    and the frequency comes from the motor, so there is one coefficient set
 *    per (motor, harmonic) and one *state* per axis underneath it. The bank is
 *    motors x harmonics with three states each; the dynamic notch's axes x
 *    slots shape is the other way round for the same reason - there the
 *    frequency is per axis, because the measurement is.
 *
 * 2. **The centre is `(harmonic + 1) * motor_hz`, clamped into the band.** The
 *    harmonics are the fundamental, twice it and three times it, and nothing
 *    searches for whether this particular airframe has all three: the weights
 *    are the control, and a harmonic set to zero weight is neither updated nor
 *    applied (decision 8).
 *
 * 3. **Below the band's floor the notch is faded out, not switched off.**
 *    `weight = (f - min_hz) / fade_hz`, so the notch is at full depth at
 *    `min_hz + fade_hz` and at no depth at `min_hz`. The frequency is clamped
 *    *up* to `min_hz` first, so a motor below the floor lands on the floor and
 *    is faded to nothing there - which is what makes the floor behave like a
 *    floor instead of like a step.
 *
 * 4. **The motor's frequency is low-passed before it moves a notch**, by
 *    `lpf_hz`. eRPM arrives in steps and with dropouts, and a notch that
 *    follows every step is a filter whose centre is modulated by the telemetry
 *    rather than by the motor. The reference's `rpm_filter_lpf_hz`.
 *
 * 5. **An unreadable reply holds the last good frequency rather than driving
 *    the notch to zero.** The reference keeps the motor's previous telemetry
 *    value when a frame decodes to nothing, and it is right to: a dropped frame
 *    is a gap in a measurement, not a stopped motor, and a notch that snapped
 *    to the floor and back every time a frame failed its checksum would put a
 *    step into the gyro at the CRC error rate. `invalid[motor]` counts the
 *    holds, so a motor whose telemetry has gone silent is visible rather than
 *    merely inaudible.
 *
 * And six belong to this module:
 *
 * 6. **The ceiling is the reference's 0.48 of the sample rate, and it is not
 *    the filter library's quarter.** `AK_FILTER_CUTOFF_MAX_RATIO` is a quarter,
 *    and its reason is a *biquad low-pass*: at half the sample rate its poles
 *    sit on the unit circle at z = -1 and its states drift for as long as the
 *    aircraft is in the air. A TPT state-variable section has no such failure -
 *    its poles are inside the circle for every finite frequency - and the case
 *    this filter exists for is the one a quarter-rate ceiling would refuse: a
 *    high-KV motor's third harmonic on an 8 kHz loop is 2.5 kHz, and 2.5 kHz is
 *    past the library's ceiling and inside this one. So the two ceilings differ
 *    deliberately, and the difference is *measured* rather than argued:
 *    tests/test_rpm_filter.c sweeps a notch to the 0.48 ceiling and shows it
 *    still attenuating at its own centre and still bounded.
 *
 *    What is above the ceiling *is* clamped, and every clamp is counted
 *    (`max_clamps`) rather than left to be discovered - the same discipline the
 *    dynamic notch's `centre_clamps` follows.
 *
 * 7. **The weight is a depth, not a switch.** The section is `x - wq * BP` with
 *    `wq = q * weight`, so at the centre the gain is `1 - weight`: a harmonic
 *    at 50 % is a -6 dB notch and not a bypassed one. That is the reference's
 *    meaning and it is why the parameters are percentages. It is also why the
 *    fade of decision 3 is continuous in *depth* - a fade written as an on/off
 *    would step the gyro by the notch's whole contribution.
 *
 * 8. **A harmonic whose configured weight is zero is neither updated nor
 *    applied**, which is the reference's own skip and is a pure configuration
 *    test. It is deliberately *not* extended to a notch whose fade has taken it
 *    to zero depth: that one is still applied, with `wq = 0`, so its state keeps
 *    running and it fades back in from a warm start rather than from rest. The
 *    cost is a section that computes a no-op; the alternative is a step in the
 *    gyro at every fade-in.
 *
 * 9. **The bank is spread across the loop, to a deadline.** motors x harmonics
 *    sections is up to twelve `sincos` pairs, and doing all of them in one
 *    iteration is a task overrun waiting for a four-motor aircraft. At most
 *    `updates_per_call` are updated per call, round-robin, chosen so the whole
 *    bank is refreshed inside AK_RPM_FILTER_UPDATE_US - the reference's
 *    RPM_FILTER_DURATION_S. Which notches were updated when is therefore not
 *    uniform, and the module says so rather than pretending otherwise: a notch
 *    is at most one deadline stale.
 *
 * 10. **Nothing here is gated on the loop rate.** The dynamic notch refuses
 *     below 2 kHz because its window and its biquads cannot work there; this
 *     module's ceiling is 0.48 of whatever rate it is given, so a 1 kHz loop
 *     gets notches up to 480 Hz and a 4 kHz loop up to 1920 Hz. `enabled` is 0
 *     only when there is nothing to filter: no motors, no harmonics, or a band
 *     with no width.
 *
 * 11. **The frequencies this drives are only as good as the eRPM under them,
 *     and there is no eRPM on this bench.** The input is `ak_dshot_gcr.h`'s
 *     electrical rpm, whose decode is host-tested and whose *wire* is not - the
 *     capture half of roadmap 3.1 does not exist and no bidirectional ESC has
 *     been measured. So every number this module's tests produce is a statement
 *     about the arithmetic and the filters, and none of them is a statement
 *     about an aircraft's vibration. The evidence file says which clauses of
 *     phase 3's acceptance this can and cannot reach.
 */

/* Four motors and three harmonics, which is the reference's own shape: its
 * RPM_FILTER_HARMONICS_MAX is 3 and its motor count is the mixer's. A hex or an
 * octo would raise the first; nothing in the arithmetic would change. */
#define AK_RPM_FILTER_MOTOR_MAX     4
#define AK_RPM_FILTER_HARMONIC_MAX  3

/* The reference's defaults, in the reference's units converted the same way
 * every other parameter of its kind is (see ak_dyn_notch.h decision 11): the
 * hundredths and the percentages are divided at the flight core's edge and the
 * module works in real Q and real weight. */
#define AK_RPM_FILTER_MIN_HZ_DEFAULT   100.0f
#define AK_RPM_FILTER_FADE_HZ_DEFAULT  100.0f
#define AK_RPM_FILTER_Q_DEFAULT          3.0f
#define AK_RPM_FILTER_LPF_HZ_DEFAULT   100.0f

/* The ceiling, as a fraction of the sample rate. The reference's
 * `0.48f * 1e6f / looptimeUs`, with the reason in decision 6. */
#define AK_RPM_FILTER_MAX_RATIO        0.48f

/* The whole bank is refreshed inside this, so a notch is at most this stale. */
#define AK_RPM_FILTER_UPDATE_US        1000u

/* The reference's ERPM_PER_LSB, and the /60 and the poles that turn it into a
 * mechanical frequency. Kept beside the law rather than inside it because the
 * same three numbers are `ak_dshot_rpm`'s. */
#define AK_RPM_FILTER_ERPM_PER_LSB     100.0f

/* One section of the bank: a TPT state-variable filter in notch form, with the
 * weight folded into the coefficient that scales the band-pass it subtracts.
 *
 * The four coefficients are the reference's, computed in rpmNotchUpdate:
 *
 *     f  = tan(pi * centre_hz * dt)
 *     q  = 1 / Q
 *     a1 = 1 / (1 + f * (f + q))
 *     a2 = f * a1
 *     wq = q * weight
 *
 * and one sample is
 *
 *     v3 = x - ic2 ; v1 = a1 * ic1 + a2 * v3 ; v2 = ic2 + f * v1
 *     ic1 = 2 * v1 - ic1 ; ic2 = 2 * v2 - ic2 ; y = x - wq * v1
 *
 * which is the TPT (topology-preserving transform) form: `v1` is the band-pass
 * output scaled by 1/q, and subtracting `q * weight` of it from the input is a
 * notch of depth `weight` at `centre_hz`.
 *
 * The state is per axis and the coefficients are not, which is decision 1.
 */
typedef struct {
    float f;
    float a1;
    float a2;
    float wq;
    /* And the weight `wq` was built from, kept rather than divided back out of
     * it: Q is a parameter a person can set, and `wq / q` would divide by zero
     * on a Q nobody meant. This is the number the readouts report and the one a
     * person comparing their configured percentage to the filter's behaviour
     * wants. */
    float w;
    float ic1[3];
    float ic2[3];
} ak_rpm_notch_t;

/* Why there is nothing to filter, when there is nothing. Shaped like
 * ak_dyn_notch_off_t so a caller that knows one knows the other. */
typedef enum {
    AK_RPM_FILTER_RUNNING   = 0,
    AK_RPM_FILTER_OFF_COUNT, /* no motors, or no harmonics: the caller asked */
    AK_RPM_FILTER_OFF_BAND,  /* no usable band: see ak_rpm_filter_init */
} ak_rpm_filter_off_t;

typedef struct {
    /* What it was set up with. `erpm_to_hz` is the one number that turns the
     * wire's unit into a mechanical frequency:
     * motor_hz = erpm * erpm_to_hz = erpm / (30 * poles). */
    float dt;
    float min_hz;
    float max_hz;
    float fade_hz;
    float q;
    float erpm_to_hz;
    float weights[AK_RPM_FILTER_HARMONIC_MAX];

    ak_rpm_notch_t notch[AK_RPM_FILTER_MOTOR_MAX][AK_RPM_FILTER_HARMONIC_MAX];

    /* The frequency each motor is running at, after the low-pass of decision 4,
     * in mechanical Hz. Zero until a reading arrives, which is a frequency the
     * fade turns into no filtering rather than a notch at zero Hz. */
    float    motor_hz[AK_RPM_FILTER_MOTOR_MAX];
    ak_filter_pt1_t freq_lpf[AK_RPM_FILTER_MOTOR_MAX];
    uint8_t  have[AK_RPM_FILTER_MOTOR_MAX];

    uint8_t  motors;
    uint8_t  harmonics;
    uint8_t  enabled;       /* 0 when there is nothing to filter */
    uint8_t  off;           /* ak_rpm_filter_off_t: why, when enabled is 0 */
    uint8_t  motor;         /* round-robin: the next notch to update */
    uint8_t  harmonic;
    uint8_t  updates_per_call;

    /* What happened, counted rather than inferred.
     *
     * `invalid[motor]` is decision 5's holds, per motor. `min_clamps` and
     * `max_clamps` count *rebuilds* whose aim the floor or the ceiling moved -
     * an event count and not a count of distinct frequencies, so a motor parked
     * below the floor for a second at 1 kHz adds a thousand to `min_clamps` and
     * that is the right number for "how often has this been happening".
     * `updates` counts section rebuilds and `passes` counts calls that applied
     * the bank, so the cost of decision 9 is two numbers rather than a claim. */
    uint32_t invalid[AK_RPM_FILTER_MOTOR_MAX];
    uint32_t min_clamps;
    uint32_t max_clamps;
    uint32_t updates;
    uint32_t passes;
} ak_rpm_filter_t;

/*
 * Set the module up for a loop running at `loop_hz`, for `motors` motors, of
 * `poles` magnetic poles each, with `harmonics` notches apiece (0 to
 * AK_RPM_FILTER_HARMONIC_MAX), searching no lower than `min_hz` and fading in
 * over `fade_hz`, at Q `q`, with the reported frequency low-passed at `lpf_hz`.
 *
 * `motors` is clamped to AK_RPM_FILTER_MOTOR_MAX and `harmonics` to
 * AK_RPM_FILTER_HARMONIC_MAX. Zero of either leaves `enabled` at 0 with `off`
 * saying `AK_RPM_FILTER_OFF_COUNT`; a `poles` below 2, a `min_hz` at or below
 * zero, a `min_hz` that is not *below* the ceiling, a negative `fade_hz`, or a
 * `loop_hz` at or below zero leaves `enabled` at 0 with `off` saying
 * `AK_RPM_FILTER_OFF_BAND`. Either way every other call is a no-op.
 * `poles` below 2 is not a motor rather than a division to guard, which is
 * `ak_dshot_rpm`'s own rule.
 *
 * The floor-versus-ceiling refusal is the one that is not obvious, and it is
 * load-bearing rather than tidy: the sections divide by `cos(pi * f * dt)`, and
 * `cos` is only positive while `f` is under half the sample rate. `min_hz` below
 * the ceiling is what keeps every centre - clamped *up* to `min_hz` as well as
 * down to the ceiling - inside that. A 5 kHz floor on an 8 kHz loop would
 * otherwise build all twelve sections from a negative cosine, and a section that
 * alternates instead of filtering reports nothing wrong to anything. So a band
 * with no width is refused, which is ak_dyn_notch.c's own answer to the same
 * question.
 *
 * The ceiling is `AK_RPM_FILTER_MAX_RATIO / loop_hz` and is *readable* as
 * `max_hz` afterwards, because a caller that has just been told a harmonic is
 * being clamped should be able to say what to. The weights start at 1 for every
 * harmonic and are moved by `ak_rpm_filter_set_weight`.
 *
 * Every state, every coefficient and every counter is defined by this call, so
 * it is safe to call on a module that has been running.
 */
void ak_rpm_filter_init(ak_rpm_filter_t *r, float loop_hz, uint8_t motors,
                        unsigned poles, uint8_t harmonics, float min_hz,
                        float fade_hz, float q, float lpf_hz);

/*
 * The depth of one harmonic, 0 to 1. Zero means the harmonic is neither updated
 * nor applied (decision 8); anything above it is a filter, at a gradient of
 * `20 * log10(1 - weight)` decibels at its own centre (decision 7).
 *
 * Out of range indices and weights are ignored rather than clamped, because
 * there is no sensible clamp for "harmonic 7 of 3" - the harmonics are a count,
 * not a quantity. A weight above 1 would make the notch's output the negative
 * of its band-pass, which is a filter nobody asked for.
 */
void ak_rpm_filter_set_weight(ak_rpm_filter_t *r, uint8_t harmonic, float weight);

/*
 * One motor's reply, as the ESC sent it: `erpm` in `ak_dshot_gcr.h`'s unit -
 * electrical rpm - and `valid` whether that number means anything.
 *
 * A valid reading moves the motor's frequency toward it through the low-pass of
 * decision 4; an invalid one is counted in `invalid[motor]` and changes nothing
 * (decision 5). On a module with `enabled` at 0 an invalid reading is still
 * counted, because "this motor's telemetry went quiet" is a fact about the
 * aircraft, and a valid one is dropped - there is no bank to move and no
 * conversion factor to move it with, and the only way back on is another
 * `ak_rpm_filter_init`, which starts the bank over anyway.
 *
 * `valid` is the verdict and not the number, which is exactly the
 * split `ak_dshot_erpm_from_value` makes: it leaves `*erpm` alone on
 * AK_DSHOT_ERPM_INVALID, so a caller that passed the number without the verdict
 * would hold a stale value by accident rather than by decision.
 *
 * A stopped motor (AK_DSHOT_ERPM_STOPPED, zero eRPM) is a valid reading of
 * zero, and drives the frequency down like any other - the fade takes the
 * notch out on its own, which is what the fade is for.
 */
void ak_rpm_filter_set_erpm(ak_rpm_filter_t *r, uint8_t motor, uint32_t erpm,
                            int valid);

/*
 * Move the bank on by one loop iteration. Updates at most `updates_per_call`
 * sections, round-robin, so the whole bank is refreshed inside
 * AK_RPM_FILTER_UPDATE_US (decision 9). Returns how many it updated, which is
 * what the test checks the deadline with. Does nothing when `enabled` is 0.
 */
unsigned ak_rpm_filter_update(ak_rpm_filter_t *r);

/*
 * The bank, applied to one gyro sample. `gyro` is three axes in whatever unit
 * the caller's gyro is in - the notch is unitless - and it is modified in
 * place, which is what the reference does and what the call site wants.
 *
 * Harmonics are walked outermost and motors inside, which is the reference's
 * order, and it does not matter: every section is linear and they are applied
 * in sequence, so the composition commutes. Written down because "does the
 * order matter" is the first question a reader of a filter bank asks.
 */
void ak_rpm_filter_apply(ak_rpm_filter_t *r, float gyro[3]);

/*
 * Where one motor's `harmonic`-th notch is, in Hz after the clamp, and how deep
 * it is: the harmonic weight times the fade of decision 3, so it is the number
 * the gyro is actually being filtered by rather than the one it was configured
 * with.
 *
 * Both report what the last `ak_rpm_filter_update` built, which is not the
 * module's configuration and not the motor's frequency: a notch is refreshed on
 * the round-robin of decision 9, so a section is up to one deadline stale, and
 * a notch pulled onto the floor or the ceiling reads as where it *went*. That is
 * the difference someone debugging a notch wants to see, and a readout that
 * recomputed the aim from `motor_hz` would hide it.
 *
 * Zero for an out-of-range motor or harmonic, and zero for everything while the
 * module is off, so a caller can print either without a guard of its own.
 */
float ak_rpm_filter_notch_hz(const ak_rpm_filter_t *r, uint8_t motor,
                             uint8_t harmonic);
float ak_rpm_filter_notch_weight(const ak_rpm_filter_t *r, uint8_t motor,
                                 uint8_t harmonic);

#endif /* AK_CORE_AK_RPM_FILTER_H */
