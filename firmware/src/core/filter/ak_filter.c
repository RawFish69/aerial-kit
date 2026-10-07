/*
 * The coefficient side of the filter library. The sample side - the four
 * `apply` functions - is in the header, because it runs once per axis per
 * sample and everything else in it is a multiply and an add.
 *
 * Nothing here reads a register, a clock or a configuration: a cutoff and a
 * sample interval go in and coefficients come out. That is ground rule 7, and
 * it is what lets the host tests drive these with a sine at a rate this
 * machine can actually run.
 */

#include "ak_filter.h"

#include <stddef.h>

#include "ak_math.h"

/* 1 / sqrt(2^(1/n) - 1) for n = 1..3, indexed by the order. The one-stage
 * entry is the 1.0f that says a single pole needs no correction at all. */
static const float pt_correction[4] = { 1.0f, 1.0f, 1.55377397f, 1.96145918f };

float ak_filter_pt_cutoff_correction(uint8_t order)
{
    if (order < 1u || order > 3u) {
        return 1.0f;
    }
    return pt_correction[order];
}

/*
 * The coefficients for a cascade of `order` PT1 sections whose -3 dB point is
 * to be `cutoff_hz`, and how close that was to what was asked for.
 *
 * The design is one line of algebra once the frequency is prewarped. The
 * continuous prototype 1/(1 + s/wc) under the bilinear transform s =
 * (2/T)(1 - z^-1)/(1 + z^-1) becomes
 *
 *     H(z) = x(1 + z^-1) / ((1 + x) - (1 - x) z^-1),     x = tan(wc*T/2)
 *
 * so the pole is (1 - x)/(1 + x) and both input taps are x/(1 + x). Taking wc
 * to be tan(wc*T/2)/(T/2) is the prewarp, and it is what puts the -3 dB point
 * on the frequency that was asked for rather than on the one the transform
 * happens to land it on.
 *
 * The clamp is applied to the *corrected* cutoff rather than to the one that
 * was given, and that is the whole reason this is one function. A PT3 asked for
 * a cutoff near the ceiling needs its three stages above it, and clamping the
 * request instead of the stages would hand back a cascade that filters less
 * than the name says. This way the filter is as close to the request as this
 * form can be, and says it was moved.
 */
static ak_filter_status_t pt_set(uint8_t order, float *b0, float *b1, float *p,
                                 float cutoff_hz, float dt)
{
    if (!(cutoff_hz > 0.0f) || !(dt > 0.0f)) {
        *b0 = 1.0f; /* pass the input through: b1 and p are zero, so the */
        *b1 = 0.0f; /* state drains in one sample and the answer is x      */
        *p = 0.0f;
        return AK_FILTER_OFF;
    }

    ak_filter_status_t status = AK_FILTER_AS_ASKED;
    float corrected = cutoff_hz * pt_correction[order];
    float limit = AK_FILTER_CUTOFF_MAX_RATIO / dt;
    if (corrected > limit) {
        corrected = limit;
        status = AK_FILTER_CLAMPED;
    }

    /* tan of half the angle. The clamp above keeps wc*dt at or below a quarter,
     * so the half-angle stays inside a quarter of pi and neither the cosine nor
     * the 1 + x below can reach zero: the pole is at worst at the origin, which
     * is a section that has stopped filtering rather than one that rings.
     *
     * At worst *at* it rather than past it, with one caveat worth writing down
     * because it is measurable: the trig here is the project's own (ground rule
     * 8), and at a quarter of pi it gives sin/cos a hair above 1. At the
     * ceiling - 2 kHz on an 8 kHz gyro, where x is exactly 1 in real arithmetic
     * - that makes the pole -2.38e-07 rather than 0. A pole there is a section
     * whose time constant is four million samples: it is still a two-point
     * average for any purpose a gyro has, and tests/test_filter.c checks that
     * the clamped cascade attenuates rather than checking the sign of a number
     * this close to zero. */
    float half = 3.14159265f * corrected * dt;
    float x = ak_sinf(half) / ak_cosf(half);
    float inv = 1.0f / (1.0f + x);

    *b0 = x * inv;
    *b1 = *b0;
    *p = (1.0f - x) * inv;
    return status;
}

void ak_filter_pt1_init(ak_filter_pt1_t *f, float cutoff_hz, float dt)
{
    f->s1 = 0.0f;
    (void)ak_filter_pt1_set(f, cutoff_hz, dt);
}

void ak_filter_pt2_init(ak_filter_pt2_t *f, float cutoff_hz, float dt)
{
    f->s1 = 0.0f;
    f->s2 = 0.0f;
    (void)ak_filter_pt2_set(f, cutoff_hz, dt);
}

void ak_filter_pt3_init(ak_filter_pt3_t *f, float cutoff_hz, float dt)
{
    f->s1 = 0.0f;
    f->s2 = 0.0f;
    f->s3 = 0.0f;
    (void)ak_filter_pt3_set(f, cutoff_hz, dt);
}

ak_filter_status_t ak_filter_pt1_set(ak_filter_pt1_t *f, float cutoff_hz, float dt)
{
    return pt_set(1u, &f->b0, &f->b1, &f->p, cutoff_hz, dt);
}

ak_filter_status_t ak_filter_pt1_cutoff(float requested_hz, float dt,
                                        float *applied_hz)
{
    if (applied_hz != NULL) {
        *applied_hz = 0.0f;
    }
    if (!(requested_hz > 0.0f) || !(dt > 0.0f)) {
        return AK_FILTER_OFF;
    }

    /* Order one, so the correction table above multiplies by one and the
     * request reaches the clamp as it was typed. Kept as the same two lines
     * `pt_set` uses rather than one shared body, because the two answer
     * different questions: `pt_set` has coefficients to fill in, and this has
     * only the number the caller asked about. */
    const float limit = AK_FILTER_CUTOFF_MAX_RATIO / dt;
    if (requested_hz > limit) {
        if (applied_hz != NULL) {
            *applied_hz = limit;
        }
        return AK_FILTER_CLAMPED;
    }
    if (applied_hz != NULL) {
        *applied_hz = requested_hz;
    }
    return AK_FILTER_AS_ASKED;
}

ak_filter_status_t ak_filter_pt2_set(ak_filter_pt2_t *f, float cutoff_hz, float dt)
{
    return pt_set(2u, &f->b0, &f->b1, &f->p, cutoff_hz, dt);
}

ak_filter_status_t ak_filter_pt3_set(ak_filter_pt3_t *f, float cutoff_hz, float dt)
{
    return pt_set(3u, &f->b0, &f->b1, &f->p, cutoff_hz, dt);
}

void ak_filter_pt1_reset(ak_filter_pt1_t *f)
{
    f->s1 = 0.0f;
}

void ak_filter_pt2_reset(ak_filter_pt2_t *f)
{
    f->s1 = 0.0f;
    f->s2 = 0.0f;
}

void ak_filter_pt3_reset(ak_filter_pt3_t *f)
{
    f->s1 = 0.0f;
    f->s2 = 0.0f;
    f->s3 = 0.0f;
}

/*
 * Both biquad sections, from the same three numbers.
 *
 * This is the standard second-order design: the analogue prototype's frequency
 * is warped by the bilinear transform (which is what taking the sine and cosine
 * of `2*pi*f*dt` does), and `alpha` carries the Q. What makes the two sections
 * different is only where the zeros go - a low-pass puts them both at z = -1,
 * a notch puts them on the unit circle at the centre frequency, which is what
 * makes its null exact rather than merely deep.
 *
 * The upshot, and the reason a person can trust the numbers: the low-pass is
 * -3 dB at `cutoff_hz` for *any* Q, and the notch is zero at `center_hz`
 * exactly. Both are properties of this design and both are measured in the host
 * tests rather than asserted here.
 */
static ak_filter_status_t biquad_set(ak_filter_biquad_t *f, float freq_hz, float q, float dt, int notch)
{
    if (!(freq_hz > 0.0f) || !(dt > 0.0f)) {
        f->b0 = 1.0f;
        f->b1 = 0.0f;
        f->b2 = 0.0f;
        f->a1 = 0.0f;
        f->a2 = 0.0f;
        return AK_FILTER_OFF;
    }

    ak_filter_status_t status = AK_FILTER_AS_ASKED;

    float limit = AK_FILTER_CUTOFF_MAX_RATIO / dt;
    if (freq_hz > limit) {
        freq_hz = limit;
        status = AK_FILTER_CLAMPED;
    }
    if (!(q > AK_FILTER_Q_MIN)) {
        q = AK_FILTER_Q_MIN;
        status = AK_FILTER_CLAMPED;
    }

    float w = 6.28318531f * freq_hz * dt;
    float sn = ak_sinf(w);
    float cs = ak_cosf(w);
    float alpha = sn / (2.0f * q);
    float a0 = 1.0f + alpha;
    float inv = 1.0f / a0;

    /* y = b0*x + b1*x1 + b2*x2 - a1*y1 - a2*y2, with everything divided by a0
     * already. `-a1` is written out because that is the sign the difference
     * equation wants, and a coefficient stored with the other one is a filter
     * that runs away. */
    if (notch) {
        f->b0 = inv;
        f->b1 = -2.0f * cs * inv;
        f->b2 = inv;
    } else {
        float one_minus_cs = 1.0f - cs;
        f->b0 = 0.5f * one_minus_cs * inv;
        f->b1 = one_minus_cs * inv;
        f->b2 = f->b0;
    }
    f->a1 = -2.0f * cs * inv;
    f->a2 = (1.0f - alpha) * inv;

    return status;
}

void ak_filter_biquad_lowpass_init(ak_filter_biquad_t *f, float cutoff_hz, float q, float dt)
{
    f->s1 = 0.0f;
    f->s2 = 0.0f;
    (void)ak_filter_biquad_lowpass_set(f, cutoff_hz, q, dt);
}

ak_filter_status_t ak_filter_biquad_lowpass_set(ak_filter_biquad_t *f, float cutoff_hz, float q, float dt)
{
    return biquad_set(f, cutoff_hz, q, dt, 0);
}

void ak_filter_biquad_notch_init(ak_filter_biquad_t *f, float center_hz, float q, float dt)
{
    f->s1 = 0.0f;
    f->s2 = 0.0f;
    (void)ak_filter_biquad_notch_set(f, center_hz, q, dt);
}

ak_filter_status_t ak_filter_biquad_notch_set(ak_filter_biquad_t *f, float center_hz, float q, float dt)
{
    return biquad_set(f, center_hz, q, dt, 1);
}

void ak_filter_biquad_reset(ak_filter_biquad_t *f)
{
    f->s1 = 0.0f;
    f->s2 = 0.0f;
}
