#ifndef AK_CORE_AK_FILTER_H
#define AK_CORE_AK_FILTER_H

#include <stdint.h>

/*
 * The filters the gyro and D-term chains are built from.
 *
 * Five of them, named for what they are rather than for where they are used:
 * PT1, PT2 and PT3 (one, two and three identical first-order poles), a biquad
 * low-pass, and a biquad notch. The names and the shapes are Betaflight's,
 * because the tuning knowledge a person brings to this aircraft is written in
 * those terms - `gyro_lpf1_static_hz` means what it means there - and inventing
 * a vocabulary for the same filter would make every number in doc 04 untranslatable.
 *
 * Four things about this file are decisions, and each is the reason a line of
 * it is written the way it is:
 *
 * 1. **Single precision, no libm.** Ground rule 8. The image links -nostdlib,
 *    so `sinf` and `cosf` are the project's own (`ak_math.h`), which also
 *    makes the coefficients the same on every build and every host. Nothing
 *    here is a `double` and `-Wdouble-promotion` is on for every build that
 *    compiles it.
 *
 * 2. **A cutoff is checked, not trusted.** Every setter reports whether the
 *    cutoff it was given is the cutoff it got. A number at or above a quarter
 *    of the sample rate is not a filter: the biquad's poles walk up to the unit
 *    circle (at half the rate they are *on* it, twice, at z = -1), and the PT
 *    section's pole walks out through the origin and comes back negative,
 *    which is a section that alternates instead of filtering. At the ceiling
 *    itself that pole is the origin to within 2.4e-07 - see the note in
 *    ak_filter.c about what the project's own `sinf`/`cosf` do at a quarter of
 *    pi - which is a section that has stopped filtering, not one that rings.
 *    So the library
 *    stops at a quarter and says `AK_FILTER_CLAMPED` rather than handing back a
 *    filter that rings. The firmware turns that into a sentence on the console;
 *    this file does not print. `AK_FILTER_OFF` is the other answer - a cutoff
 *    of zero or below, or a sample interval of zero or below, means the input
 *    passes through unchanged, which is what a person who asks for a disabled
 *    filter wants and is not the same thing as a filter with a very high
 *    cutoff.
 *
 * 3. **A coefficient update keeps the state.** These exist so a cutoff can
 *    move while the aircraft is flying (betaflight's dynamic low-pass does
 *    exactly that): `set` recomputes coefficients and touches no state, and a
 *    `reset` is a separate call for the moments that really do want the filter
 *    to start over - arming, or a gyro that has just been re-ranged. A setter
 *    that cleared the state would put a step into the gyro path every time the
 *    throttle moved.
 *
 * 4. **The response is checked against its own arithmetic.** A filter's
 *    magnitude and phase are properties a host test can measure exactly (see
 *    tests/test_filter.c): the difference equation is driven with a sine and
 *    the answer is compared with the transfer function that the *coefficients*
 *    describe, and separately with the continuous prototype the name promises.
 *    A filter whose -3 dB point is not where its name says is a filter that
 *    silently changes every tuning number in doc 04.
 *
 * 5. **Every section is the bilinear transform of the prototype it is named
 *    after**, with the cutoff prewarped so the -3 dB point lands on the number
 *    that was typed. That is one design method for all five filters, and it is
 *    what makes the phase right and not just the magnitude - and phase is the
 *    property a rate loop actually spends, as phase margin.
 *
 *    The alternative is worth naming, because this tree already contains it. A
 *    single pole whose gain is solved to put the -3 dB point in the right place
 *    (the shape `ak_lpf_alpha` produces, and the shape this firmware's earlier
 *    gyro and D-term filters are built from) hits that one number and misses
 *    the rest of the curve, because it has dropped the zero at z = -1 that a
 *    real digital low-pass has. Measured at a 101.5625 Hz corner on an 8 kHz
 *    sample rate: -42.76 degrees where the prototype answers -45.00, and at
 *    1000 Hz -62.01 where the prototype answers -84.20. The magnitude is right
 *    to 0.22 dB at 1000 Hz; it is the phase that is 22.2 degrees away.
 *
 *    Neither number is a defect on its own - a filter with *less* lag is a
 *    filter with more phase margin - and that is the point. It is a different
 *    filter from the one its cutoff names, by an amount that grows with
 *    frequency and is largest where the loop's phase margin is decided. Every
 *    tuning number this project inherits (doc 04, and Betaflight's names before
 *    it) was chosen against the prototype's phase, so the chains phase 2.2
 *    builds are built from this library and the older paths move here with it.
 *    tests/test_filter.c prints the two curves side by side; run it.
 *
 * The bilinear transform does not map frequency linearly, so the prototype is
 * only a claim about the filter inside a band. Measured, that band is a
 * sixteenth of the sample rate at 8 kHz, and it is a measurement rather than an
 * assumption: see the sweep in tests/test_filter.c, which prints the deviation
 * at every frequency it checks and holds the magnitude to the roadmap's 0.5 dB
 * and the phase to 0.5 degrees inside the band, and outside it reports what the
 * warping costs. At a sixteenth of the rate - 500 Hz on an 8 kHz gyro - the
 * worst of the five is 0.22 dB from its prototype.
 */

/*
 * How far a cutoff was from what was asked for.
 *
 * The three are not degrees of a single scale: `AS_ASKED` and `CLAMPED` are the
 * same filter with different histories, and `OFF` is not a filter at all.
 */
typedef enum {
    /* The cutoff is the one that was given. */
    AK_FILTER_AS_ASKED = 0,

    /* The cutoff was outside what this filter can be at this sample rate and
     * was moved to the nearest one that is. Both the PT cascade and the biquad
     * stop at a quarter of the sample rate; a Q below AK_FILTER_Q_MIN is moved
     * up to it, because the biquad divides by Q. */
    AK_FILTER_CLAMPED = 1,

    /* No filtering: the cutoff was zero or below, or the sample interval was.
     * The filter passes its input through unchanged and its state stays where
     * it is. */
    AK_FILTER_OFF = 2
} ak_filter_status_t;

/*
 * The highest cutoff any filter here will be given, as a fraction of the
 * sample rate.
 *
 * A quarter, not a half. At exactly half the sample rate a biquad low-pass
 * built the usual way has a double pole *on* the unit circle at z = -1 and a
 * double zero on top of it: mathematically a pass-through, and in single
 * precision a pair of marginally stable states that drift for as long as the
 * aircraft is in the air. A quarter keeps the poles at a distance where that
 * cannot happen, and it is above every cutoff a gyro or a D-term wants - a
 * 500 Hz filter on an 8 kHz gyro is at a sixteenth of the rate.
 */
#define AK_FILTER_CUTOFF_MAX_RATIO 0.25f

/* The smallest Q a biquad will be asked for. The low-pass of a second-order
 * section is underdamped below 0.5 and a notch narrower than this is not
 * tracking anything a motor does. */
#define AK_FILTER_Q_MIN 0.05f

/*
 * The cutoff a cascade of `order` PT1 stages must each sit at for the cascade's
 * own -3 dB point to be the cutoff that was asked for: 1 / sqrt(2^(1/n) - 1).
 * One stage needs no correction, two need 1.5538, three 1.9615 - which is why a
 * PT2 or PT3 is *not* simply the same cutoff applied three times.
 */
float ak_filter_pt_cutoff_correction(uint8_t order);

/* ------------------------------------------------------------------ PT1 --- */

/*
 * One first-order section, as a pole and a zero:
 *
 *     y[n] = b0*x[n] + b1*x[n-1] + p*y[n-1]        b0 = b1 = b,  p = 1 - 2b
 *
 * A first-order digital low-pass has both. The pole is what does the filtering
 * and the zero sits at z = -1, which is what makes the phase at the cutoff -45
 * degrees rather than the -43 a pole on its own would give and what carries the
 * response on down to nothing at Nyquist instead of flattening out there. One
 * `b` would describe a filter, but a *pass-through* is b0 = 1, b1 = 0, p = 0,
 * which is not one `b` and two of, so both are stored and `AK_FILTER_OFF` is an
 * ordinary coefficient set rather than a special case inside `apply`.
 *
 * The cascade types share the coefficients across their stages, because a PTn
 * is n *identical* sections: only the state differs per stage. They are three
 * types rather than an array because the count is fixed at build time by the
 * name of the filter a person asked for, and an array would be a loop in the
 * sample path to no purpose.
 */
typedef struct {
    float b0;
    float b1;
    float p;
    float s1;
} ak_filter_pt1_t;

/* ------------------------------------------------------------------ PT2 --- */

typedef struct {
    float b0;
    float b1;
    float p;
    float s1;
    float s2;
} ak_filter_pt2_t;

/* ------------------------------------------------------------------ PT3 --- */

typedef struct {
    float b0;
    float b1;
    float p;
    float s1;
    float s2;
    float s3;
} ak_filter_pt3_t;

void ak_filter_pt1_init(ak_filter_pt1_t *f, float cutoff_hz, float dt);
void ak_filter_pt2_init(ak_filter_pt2_t *f, float cutoff_hz, float dt);
void ak_filter_pt3_init(ak_filter_pt3_t *f, float cutoff_hz, float dt);

/* Move the cutoff. The state is kept: this is what a dynamic low-pass calls
 * when the throttle moves. */
ak_filter_status_t ak_filter_pt1_set(ak_filter_pt1_t *f, float cutoff_hz, float dt);
ak_filter_status_t ak_filter_pt2_set(ak_filter_pt2_t *f, float cutoff_hz, float dt);
ak_filter_status_t ak_filter_pt3_set(ak_filter_pt3_t *f, float cutoff_hz, float dt);

/* What a first-order cutoff is actually given at this sample interval, and
 * whether it had to move. `pt_set` already decides this; asking it as a
 * question is what lets a configuration be *checked* rather than only
 * discovered - a cutoff above a quarter of the sample rate comes back as the
 * quarter, so a parameter named 500 flies as 400 and nothing said so. The
 * firmware prints a sentence per clamp at boot off the back of this.
 *
 * `applied_hz` may be null; it is written 0 for `AK_FILTER_OFF`. */
ak_filter_status_t ak_filter_pt1_cutoff(float requested_hz, float dt,
                                        float *applied_hz);

void ak_filter_pt1_reset(ak_filter_pt1_t *f);
void ak_filter_pt2_reset(ak_filter_pt2_t *f);
void ak_filter_pt3_reset(ak_filter_pt3_t *f);

/* Start a section over *from the input it is about to be given*, so that its
 * next `apply` returns that input unchanged.
 *
 * `reset` says the filter has no history. This says the same thing and then
 * answers the question reset leaves open: a section with no history does not
 * know that the past was zero, and claiming it was costs a real fraction of
 * the first sample. A first-order section reaches its steady state over two
 * samples, so two cascaded sections starting from zero put a quarter of the
 * first sample through - which is what `make timing` caught, in a check whose
 * subject is the *interval* between samples and which read 0.024784 deg where
 * the rotation a millisecond is worth is 0.100000.
 *
 * Priming is the honest state rather than a fudge: `s1 = x(1 - b0)` is the
 * state that makes the section a fixed point for the constant `x`, so a
 * section primed with a steady input stays transparent until the input moves,
 * and a section primed with a rate the aircraft is already turning at puts
 * that rate through instead of a step down from it.
 *
 * Coefficients must be set first - the answer depends on `b0`. */
static inline void ak_filter_pt1_prime(ak_filter_pt1_t *f, float x)
{
    f->s1 = x * (1.0f - f->b0);
}

/* Transposed direct form II, which for a first-order section is one state:
 * the two input taps are multiplied by the same `b`, so `s1` carries what is
 * left of the previous output rather than a copy of the previous input. */
static inline float ak_filter_pt1_apply(ak_filter_pt1_t *f, float x)
{
    float y = f->b0 * x + f->s1;
    f->s1 = f->b1 * x + f->p * y;
    return y;
}

static inline float ak_filter_pt2_apply(ak_filter_pt2_t *f, float x)
{
    float y1 = f->b0 * x + f->s1;
    f->s1 = f->b1 * x + f->p * y1;
    float y2 = f->b0 * y1 + f->s2;
    f->s2 = f->b1 * y1 + f->p * y2;
    return y2;
}

static inline float ak_filter_pt3_apply(ak_filter_pt3_t *f, float x)
{
    float y1 = f->b0 * x + f->s1;
    f->s1 = f->b1 * x + f->p * y1;
    float y2 = f->b0 * y1 + f->s2;
    f->s2 = f->b1 * y1 + f->p * y2;
    float y3 = f->b0 * y2 + f->s3;
    f->s3 = f->b1 * y2 + f->p * y3;
    return y3;
}

/* --------------------------------------------------------------- biquad --- */

/*
 * A second-order section, in transposed direct form II.
 *
 * The coefficients are already divided by `a0`, and are in the convention the
 * difference equation below uses:
 *
 *     y[n] = b0*x[n] + b1*x[n-1] + b2*x[n-2] - a1*y[n-1] - a2*y[n-2]
 *
 * The transposed form is the one to keep at this word length: two states
 * instead of four, and the ones it does keep are scaled by the coefficients
 * rather than by the signal, so a biquad doing a 30 dB cut at 60 Hz on an 8 kHz
 * gyro does not lose the top of its input to rounding before the low-pass has
 * seen it.
 *
 * One type serves both sections because the shape is what is shared: a notch
 * and a low-pass differ only in where their zeros are. `apply` is the same for
 * both and there is no second one to keep in step.
 */
typedef struct {
    float b0;
    float b1;
    float b2;
    float a1;
    float a2;
    float s1;
    float s2;
} ak_filter_biquad_t;

/* A second-order low-pass with the given Q. The -3 dB point is exactly
 * `cutoff_hz` for any Q, which is a property of the design and is checked. */
void ak_filter_biquad_lowpass_init(ak_filter_biquad_t *f, float cutoff_hz, float q, float dt);
ak_filter_status_t ak_filter_biquad_lowpass_set(ak_filter_biquad_t *f, float cutoff_hz, float q, float dt);

/* A notch with its null exactly at `center_hz` and its width set by Q: the
 * higher the Q the narrower the band it takes out. Q is centre over width, so
 * a 300 Hz notch at Q 10 takes out about 30 Hz either side of it. */
void ak_filter_biquad_notch_init(ak_filter_biquad_t *f, float center_hz, float q, float dt);
ak_filter_status_t ak_filter_biquad_notch_set(ak_filter_biquad_t *f, float center_hz, float q, float dt);

void ak_filter_biquad_reset(ak_filter_biquad_t *f);

static inline float ak_filter_biquad_apply(ak_filter_biquad_t *f, float x)
{
    float y = f->b0 * x + f->s1;
    f->s1 = f->b1 * x - f->a1 * y + f->s2;
    f->s2 = f->b2 * x - f->a2 * y;
    return y;
}

#endif /* AK_CORE_AK_FILTER_H */
