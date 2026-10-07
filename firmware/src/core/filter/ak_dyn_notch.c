#include "filter/ak_dyn_notch.h"

#include <string.h>

#include "filter/ak_fft.h"
#include "flight/ak_math.h"

/*
 * Finding the gyro's vibration peaks. See ak_dyn_notch.h for what the decisions
 * are and which of them are the reference's.
 *
 * The shape: push() decimates at the loop rate into a per-axis window; update()
 * takes one axis whose window is full, windows it, transforms it, measures a
 * noise floor, picks the peaks above that floor, interpolates each one's true
 * frequency from its shape, and walks the tracked centre toward it.
 */

/* Two, as a float, so no `double` literal appears in this file - ground rule 8
 * and the -Wdouble-promotion the build carries. */
#define AK_DYN_NOTCH_TWO 2.0f

/* The reference weights a peak's own shoulders at 0.75 of a bin when it
 * averages them out of the floor: a shoulder does carry some of the peak. */
#define AK_DYN_NOTCH_SHOULDER 0.75f

/* The floor is doubled before it is used as a threshold. On *power*, which is
 * what this file measures in - the reference's own units, so the factor means
 * the same thing here. */
#define AK_DYN_NOTCH_FLOOR_FACTOR 2.0f

static float clampf(float v, float lo, float hi)
{
    if (v < lo) {
        return lo;
    }
    if (v > hi) {
        return hi;
    }
    return v;
}

/* Round to the nearest bin, as an int. */
static int nearest_bin(float hz, float bin_hz)
{
    return (int)(hz / bin_hz + 0.5f);
}

void ak_dyn_notch_init(ak_dyn_notch_t *d, float loop_hz, float min_hz,
                       float max_hz, uint8_t count, uint16_t points)
{
    if (d == 0) {
        return;
    }
    memset(d, 0, sizeof *d);

    if (count < 1u) {
        count = 1u;
    }
    if (count > AK_DYN_NOTCH_COUNT_MAX) {
        count = (uint8_t)AK_DYN_NOTCH_COUNT_MAX;
    }
    /* 64 or the window this module's arrays are sized for, and nothing else.
     * `ak_fft_size_ok` would accept 256, and every one of the buffers below
     * (`buf`, `fft_re`, `fft_im`, `power`) is AK_DYN_NOTCH_FFT_POINTS long:
     * accepting a longer transform here would write past all four. The check
     * is `!=` rather than `>` on purpose - a 128-point window is not this
     * module's window either, and a fallback that silently reshapes the
     * spectrum's resolution is worse than one that ignores the argument. */
    if (points != (uint16_t)AK_DYN_NOTCH_FFT_POINTS) {
        points = (uint16_t)AK_DYN_NOTCH_FFT_POINTS;
    }

    d->points = points;
    d->count  = count;
    d->min_hz = min_hz;
    d->max_hz = max_hz;
    d->q      = AK_DYN_NOTCH_Q_DEFAULT;

    /* The gate, before anything else: see decision 4 in the header. A caller
     * reads `enabled` and says so; nothing below is a frequency it could not
     * reach. */
    if (loop_hz < AK_DYN_NOTCH_UPDATE_MIN_HZ) {
        d->enabled = 0u;
        d->off     = (uint8_t)AK_DYN_NOTCH_OFF_RATE;
        return;
    }

    /* A band with no width is not a band. Refused here rather than clamped,
     * because every clamp available would be a choice the caller did not make:
     * pulling `max` up to `min` leaves a search one bin wide at a frequency
     * nobody asked for, and leaving them crossed makes `clampf(centre, min,
     * max)` below return `hi` - a notch *under* the band it was measured in.
     * A module that says "no" is the only honest answer. */
    if (!(max_hz > min_hz)) {
        d->enabled = 0u;
        d->off     = (uint8_t)AK_DYN_NOTCH_OFF_BAND;
        return;
    }
    d->enabled = 1u;
    d->off     = (uint8_t)AK_DYN_NOTCH_RUNNING;

    /* Decision 1: downsample by whatever makes the analysis Nyquist land just
     * above the top of the band being searched - the reference's
     * `MAX(1, nyquistHz / maxHz)`, which is a *truncation* and has to be.
     * Rounding is the trap here: at an 8 kHz loop with a 600 Hz ceiling the
     * quotient is 6.67, and rounding up to 7 would put the decimated Nyquist
     * at 571 Hz - below the ceiling the decimation exists to clear, so the top
     * of the band would be searching the mirror. Truncating gives 6, an
     * analysis rate of 1333 Hz and a Nyquist of 667 Hz, which clears it. */
    float nyquist = loop_hz * 0.5f;
    float by = 1.0f;

    if (max_hz > 0.0f) {
        by = nyquist / max_hz;
    }
    if (by < 1.0f) {
        by = 1.0f;
    }
    d->decim_by = (uint16_t)by;
    if (d->decim_by < 1u) {
        d->decim_by = 1u;
    }

    d->fs_hz  = loop_hz / (float)d->decim_by;
    d->bin_hz = d->fs_hz / (float)d->points;

    /* The biquads run at the loop rate, not at the analysis rate: they are in
     * the gyro chain, filtering the samples the driver read. A coefficient
     * built for the decimated rate would put every notch six times lower than
     * the measurement said. */
    d->notch_dt = 1.0f / loop_hz;

    /* The upper edge can never be past the decimated Nyquist, and it is clamped
     * *before* the band is derived from it - a max_hz above Nyquist would
     * otherwise put the search's top bin in the mirror, where it would report a
     * tone the search had already seen. */
    if (d->max_hz > d->fs_hz * 0.5f) {
        d->max_hz = d->fs_hz * 0.5f;
    }

    /* The band the search may look in. Bin 0 is the mean - the bias the
     * calibration already removed - and is never a vibration, so the low edge
     * is bin 1 at the earliest. The high edge stops one bin short of the
     * mirror, which is also what guarantees a peak's upper shoulder at bin+1
     * is a real bin and not a wrap. */
    int first = nearest_bin(d->min_hz, d->bin_hz);
    int last  = nearest_bin(d->max_hz, d->bin_hz);
    int top   = (int)(d->points / 2u) - 1;

    if (first < 1) {
        first = 1;
    }
    if (last > top) {
        last = top;
    }
    if (last < first) {
        /* A band narrower than a bin is not a band. Fall back to the bin the
         * low edge names, clamped into range, rather than searching nothing. */
        last = first;
    }
    d->first_bin = (uint16_t)first;
    d->last_bin  = (uint16_t)last;

    /* Start the tracked centres spread evenly across the band, the reference's
     * own choice ("makes notches stick to peaks quicker"): a notch that starts
     * at the far end of the band has further to travel to reach a peak that
     * appears next to a differently-seeded one. `have` stays 0 until a real
     * measurement lands in each slot. */
    for (int a = 0; a < AK_DYN_NOTCH_AXES; a++) {
        for (int p = 0; p < (int)count; p++) {
            d->centre_hz[a][p] =
                ((float)p + 0.5f) * (d->max_hz - d->min_hz) / (float)count + d->min_hz;
        }
    }
}

void ak_dyn_notch_push(ak_dyn_notch_t *d, const float gyro[3])
{
    int a;

    if (d == 0 || gyro == 0 || !d->enabled) {
        return;
    }

    for (a = 0; a < AK_DYN_NOTCH_AXES; a++) {
        d->accum[a] += gyro[a];
    }

    d->decim++;
    if (d->decim < d->decim_by) {
        return;
    }
    d->decim = 0u;

    /* Every axis gets the mean of the samples that made up this decimated one,
     * but each axis's window is its own: after an update resets one of them,
     * the axes are no longer in step, and an axis whose window is still full
     * counts the sample as dropped rather than overwriting the one it is
     * waiting to analyse. `dropped` is a count of axis-samples, so a healthy
     * run drops exactly the axes that have not been served yet. */
    float inv = 1.0f / (float)d->decim_by;

    for (a = 0; a < AK_DYN_NOTCH_AXES; a++) {
        if (d->fill[a] >= d->points) {
            d->dropped++;
        } else {
            d->buf[a][d->fill[a]] = d->accum[a] * inv;
            d->fill[a]++;
        }
        d->accum[a] = 0.0f;
    }
}

/* A bin is a peak if it is strictly above its lower shoulder and at least equal
 * to its upper - the strictness on one side only so a plateau reports once
 * rather than twice. */
static int is_local_max(const ak_dyn_notch_t *d, int k)
{
    int top = (int)(d->points / 2u);

    if (k <= 0 || k >= top) {
        return 0;
    }
    return d->power[k] > d->power[k - 1] && d->power[k] >= d->power[k + 1];
}

/*
 * Rebuild one axis's engaged sections from the centres the tracking holds.
 *
 * `ak_filter_biquad_notch_set` and not `..._init`: the library's contract is
 * that `set` re-derives the coefficients and leaves the state alone, which is
 * the whole reason a tracking notch can move by a few hertz without putting a
 * step into the gyro. See decision 10 in the header.
 *
 * Every engaged slot is rebuilt on every update rather than only the ones whose
 * centre moved - see decision 9 - so a change to Q reaches the filters on the
 * next window with no change-detection state to keep in step.
 */
static void bank_rebuild(ak_dyn_notch_t *d, int a)
{
    int p;

    for (p = 0; p < (int)d->count; p++) {
        if (!d->engaged[a][p]) {
            continue;
        }
        if (ak_filter_biquad_notch_set(&d->notch[a][p], d->centre_hz[a][p],
                                       d->q, d->notch_dt) ==
            AK_FILTER_CLAMPED) {
            d->centre_clamps++;
        }
        d->coefficient_sets++;
    }
}

void ak_dyn_notch_set_q(ak_dyn_notch_t *d, float q)
{
    if (d != 0) {
        d->q = q;
    }
}

float ak_dyn_notch_filter(ak_dyn_notch_t *d, uint8_t axis, float x)
{
    uint8_t p;

    if (d == 0 || !d->enabled || axis >= (uint8_t)AK_DYN_NOTCH_AXES) {
        return x;
    }

    /* Ascending in frequency, which is the order the tracking keeps them in, so
     * the chain is the same chain whatever the slots were named. A slot with no
     * measurement behind it is skipped rather than applied as a wire: skipping
     * leaves its state alone, which is what the library's `set` contract
     * expects, and applying it would mean its state advanced against a centre
     * nobody chose. */
    for (p = 0; p < d->count; p++) {
        if (d->engaged[axis][p]) {
            x = ak_filter_biquad_apply(&d->notch[axis][p], x);
        }
    }
    return x;
}

unsigned ak_dyn_notch_engaged_notches(const ak_dyn_notch_t *d, uint8_t axis)
{
    unsigned n = 0u;
    uint8_t  p;

    if (d == 0 || axis >= (uint8_t)AK_DYN_NOTCH_AXES) {
        return 0u;
    }
    for (p = 0; p < d->count; p++) {
        if (d->engaged[axis][p]) {
            n++;
        }
    }
    return n;
}

int ak_dyn_notch_update(ak_dyn_notch_t *d, uint32_t now_us)
{
    int a, k, p;

    if (d == 0 || !d->enabled) {
        return -1;
    }

    if (d->axis >= (uint8_t)AK_DYN_NOTCH_AXES) {
        d->axis = 0u;
    }
    a = (int)d->axis;

    if (d->fill[a] < d->points) {
        return -1;
    }

    /* Copy the window out and window it. Hann rather than rectangular: a motor
     * fundamental almost never sits on a bin, and a rectangular window's leak
     * from a strong harmonic is what a peak tracker follows by mistake. */
    for (k = 0; k < (int)d->points; k++) {
        d->fft_re[k] = d->buf[a][k];
        d->fft_im[k] = 0.0f;
    }
    ak_fft_window_apply(d->fft_re, d->points, AK_FFT_WINDOW_HANN);
    (void)ak_fft_forward(d->fft_re, d->fft_im, d->points);

    /* Power, not magnitude: the reference measures in power, and the floor
     * factor and the parabola below are both the reference's, so mixing units
     * would change what they mean. It also saves a square root per bin.
     *
     * Filled across the whole transform, mirror included, so that reading a
     * peak's upper shoulder at bin+1 can never leave the array regardless of
     * where the band's top edge landed. Only 0..points/2 is ever searched. */
    for (k = 0; k < (int)d->points; k++) {
        float re = d->fft_re[k];
        float im = d->fft_im[k];

        d->power[k] = re * re + im * im;
    }

    /* The strongest local maxima in the band, at least two bins apart. Two
     * apart because a single tone's main lobe is three bins wide under this
     * window: adjacent bins of one peak are not two peaks. */
    int   chosen[AK_DYN_NOTCH_COUNT_MAX];
    int   nchosen = 0;

    for (p = 0; p < (int)d->count; p++) {
        int   best  = -1;
        float bestv = 0.0f;

        for (k = (int)d->first_bin; k <= (int)d->last_bin; k++) {
            if (!is_local_max(d, k)) {
                continue;
            }
            int too_close = 0;

            for (int j = 0; j < nchosen; j++) {
                int gap = chosen[j] - k;

                if (gap < 0) {
                    gap = -gap;
                }
                if (gap < 2) {
                    too_close = 1;
                }
            }
            if (too_close) {
                continue;
            }
            if (best < 0 || d->power[k] > bestv) {
                best  = k;
                bestv = d->power[k];
            }
        }
        if (best < 0) {
            break;
        }
        chosen[nchosen++] = best;
    }

    /* The floor: the mean of the band with each chosen peak's own three bins
     * weighted out of the sum first, so a strong peak does not raise the floor
     * that decides whether it is a peak. Decision 5 in the header. */
    float total = 0.0f;
    int   bins  = (int)d->last_bin - (int)d->first_bin + 1;

    for (k = (int)d->first_bin; k <= (int)d->last_bin; k++) {
        total += d->power[k];
    }
    for (p = 0; p < nchosen; p++) {
        int b = chosen[p];

        total -= AK_DYN_NOTCH_SHOULDER * d->power[b - 1];
        total -= d->power[b];
        total -= AK_DYN_NOTCH_SHOULDER * d->power[b + 1];
    }
    bins -= nchosen;
    if (bins < 1) {
        bins = 1;
    }
    d->floor = (total / (float)bins) * AK_DYN_NOTCH_FLOOR_FACTOR;

    /* Sort ascending in frequency, so slot p is always the p'th lowest peak.
     * Without this the slots swap when two peaks cross and every notch jumps
     * the width of the band - an insertion sort over at most five entries. */
    for (p = 1; p < nchosen; p++) {
        int   key = chosen[p];
        int   j   = p - 1;

        while (j >= 0 && chosen[j] > key) {
            chosen[j + 1] = chosen[j];
            j--;
        }
        chosen[j + 1] = key;
    }

    /* The time step is measured, not assumed: the caller's clock differenced
     * against this axis's last transform. Unsigned subtraction, so the 32-bit
     * microsecond wrap is handled the way the scheduler handles it. A wrap
     * that produced an absurd interval would drive the alpha to 1 and snap the
     * centre to the measurement, which is the safe direction. */
    float dt = 0.0f;
    int   first_time = (d->last_us[a] == 0u);

    if (!first_time) {
        dt = (float)(now_us - d->last_us[a]) * 1e-6f;
    }

    int kept = 0;

    for (p = 0; p < nchosen; p++) {
        int b = chosen[p];

        if (d->power[b] <= d->floor) {
            continue;   /* at the floor: noise, not a vibration */
        }

        /* Parabolic interpolation through the peak and its two shoulders,
         * solving dy/dx = 0 - the reference's meanBin. A peak between two bins
         * is the normal case, not the exception. */
        float y0 = d->power[b - 1];
        float y1 = d->power[b];
        float y2 = d->power[b + 1];
        float mean_bin = (float)b;
        float denom = AK_DYN_NOTCH_TWO * (y0 - AK_DYN_NOTCH_TWO * y1 + y2);

        if (denom != 0.0f) {
            mean_bin += (y0 - y2) / denom;
        }

        float centre = clampf(mean_bin * d->bin_hz, d->min_hz, d->max_hz);

        if (first_time) {
            /* Nothing to smooth against yet. Snap, rather than walk in from
             * the even spread the init laid down. */
            d->centre_hz[a][kept] = centre;
        } else {
            /* Decision 6: how fast the centre may walk depends on how far above
             * the floor the peak is - up to ten times faster for a strong one. */
            float speedup = clampf(d->power[b] / d->floor, 1.0f,
                                   AK_DYN_NOTCH_MAX_SPEEDUP);
            float gain = ak_lpf_alpha(AK_DYN_NOTCH_SMOOTH_HZ * speedup, dt);

            d->centre_hz[a][kept] += gain * (centre - d->centre_hz[a][kept]);
        }

        /* A slot that has never held a measurement is put in place now - see
         * decision 7. `init` rather than `set`, because there is no state to
         * preserve: this notch has never filtered anything. Every other slot is
         * left to `bank_rebuild` below, which `set`s it and keeps its state. */
        if (!d->engaged[a][kept]) {
            ak_filter_biquad_notch_init(&d->notch[a][kept],
                                        d->centre_hz[a][kept], d->q,
                                        d->notch_dt);
            d->engaged[a][kept] = 1u;
        }
        kept++;
    }

    d->have[a] = (uint8_t)kept;
    if (kept == 0) {
        d->skipped++;
    }

    /* The whole bank for this axis, not only the slots this transform moved: a
     * slot whose peak dipped under the floor keeps its centre *and* its
     * coefficients, and a slot that never had a peak stays bypassed. */
    bank_rebuild(d, a);

    d->transforms++;
    d->last_us[a] = now_us;
    d->fill[a]    = 0u;
    d->axis       = (uint8_t)((a + 1) % AK_DYN_NOTCH_AXES);

    return a;
}
