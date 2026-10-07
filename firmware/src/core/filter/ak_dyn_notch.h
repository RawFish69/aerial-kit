#ifndef AK_CORE_AK_DYN_NOTCH_H
#define AK_CORE_AK_DYN_NOTCH_H

#include <stdint.h>

#include "filter/ak_filter.h"

/*
 * Finding the gyro's vibration peaks, and notching them out.
 *
 * The module is two halves that run at two different rates.
 *
 * **The measurement**, in the slow tier: a decimated copy of each gyro axis, a
 * transform of it (ak_fft.h), a noise floor, and a per-axis set of tracked
 * centre frequencies. `ak_dyn_notch_update` does this, one axis per call.
 *
 * **The notches themselves**, at the loop rate: one second-order section per
 * axis per slot, in this same struct, rebuilt from the tracked centres by the
 * slow half and applied to the raw gyro by `ak_dyn_notch_filter`. The split is
 * where the cost has to be, not a matter of taste - a transform is hundreds of
 * trig calls and a biquad is five multiplies, so the transform goes somewhere
 * that can afford it and the biquads go where the samples are.
 *
 * Both halves live in one struct and one file because the second has no meaning
 * without the first: a notch bank with no measurement is a set of filters at
 * frequencies nobody chose.
 *
 * Six things here are decisions, and four of them are the reference's, taken
 * from the pinned checkout rather than remembered:
 *
 * 1. **The decimation is derived from the band, not chosen.** Betaflight
 *    computes `sampleCount = MAX(1, nyquistHz / maxHz)` - downsample by
 *    whatever makes the analysis Nyquist land just above the top of the band
 *    being searched. The reason to decimate at all is to *zoom*: the same 64
 *    bins concentrate on 100-600 Hz instead of spreading over 0-4000, so the
 *    resolution lands where the motor noise is. Decimating for its own sake
 *    would throw resolution away exactly where the notches go.
 *
 * 2. **64 points, the roadmap's floor.** The window is `points / decimated
 *    rate` long, and that length is the whole trade: a longer window resolves
 *    closer tones and reacts more slowly to a throttle change. The motor's
 *    fundamental moves within tens of milliseconds, so the window is kept at
 *    the short end. At an 8 kHz loop with a 600 Hz ceiling: decimate by 6, a
 *    1333 Hz analysis rate, a 48 ms window, 20.8 Hz bins.
 *
 * 3. **One axis per call.** `ak_dyn_notch_update` transforms a single axis and
 *    moves on, round-robin - the reference's own spreading across
 *    `DYN_NOTCH_CALC_TICKS`, for the same reason: the transform must not
 *    overrun the task's period. It is also why this is safe without knowing the
 *    transform's cost in advance. A 64-point transform is 192 butterflies and
 *    384 calls to `ak_sinf`/`ak_cosf` (ak_fft.h computes its twiddles rather
 *    than tabulating them), and *that number is analytic, not measured* - the
 *    board's own profiler has to price it, and the phase 2.3 evidence says
 *    which half of that was done here.
 *
 * 4. **The gate is the reference's, and it is reported.** Below a 2 kHz loop
 *    rate the reference sets its notch count to zero and does nothing;
 *    `DYN_NOTCH_UPDATE_MIN_HZ` is 2000 in the pinned source. The same limit
 *    falls out of this project's own filter library independently: a biquad
 *    notch's centre is clamped to a quarter of its sample rate
 *    (`AK_FILTER_CUTOFF_MAX_RATIO`), so at the 1 kHz loop every board boots at,
 *    the highest notch representable is 250 Hz - and a 450 Hz vibration, which
 *    is the phase's own acceptance case, cannot be notched at all. So
 *    `enabled` is 0 below the gate and callers read it; nothing is silently
 *    clamped to a frequency it cannot reach.
 *
 * 5. **The floor is the reference's law**: twice the mean of the band's bins,
 *    with each peak's own three bins weighted out of the average first
 *    (0.75/1.0/0.75), so a strong peak does not raise the floor that decides
 *    whether it is a peak. A peak search with no floor at all reports the
 *    largest bin of a silent axis, which tests/test_fft.c demonstrates
 *    deliberately in `ak_fft_peak_bin`'s own terms.
 *
 * 6. **The centre is interpolated, then smoothed by a rate that depends on how
 *    strong the peak is.** The true peak rarely sits on a bin, so a parabola
 *    through the peak and its two shoulders estimates where it really is; and
 *    the PT1 that walks the tracked centre toward it runs up to ten times
 *    faster for a peak well above the floor than for a marginal one
 *    (`DYN_NOTCH_SMOOTH_HZ 4`, `DYN_NOTCH_MAX_SPEEDUP 10`). A notch that slews
 *    at one rate either lags a throttle punch or wanders on noise; this is how
 *    the reference gets both, and it is adopted rather than reinvented.
 *
 * The tracking's own time step is *measured, not assumed*: the caller passes
 * the microsecond time of each call and the module differences its own
 * successive transforms of the same axis. That is the timing doctrine phase 1.5
 * set for the gyro, applied to the slow tier.
 *
 * And six more belong to the bank, the half that filters:
 *
 * 7. **A notch is engaged by a measurement, not by the configuration.** The
 *    reference seeds every notch at an even spread across the band and filters
 *    with all of them from the first sample. Here a slot whose axis has never
 *    reported a peak is *bypassed* - the sample goes through untouched. A
 *    filter at a frequency nothing measured is a filter the configuration did
 *    not ask for, and an even spread across 100-600 Hz is three notches at
 *    frequencies chosen by arithmetic. The divergence is deliberate and it
 *    costs the reference's faster first lock: a slot engages on its first
 *    measurement instead of already being in place.
 *
 * 8. **A slot that has been engaged stays where it is until the next
 *    measurement moves it.** A peak that dips under the floor for one window
 *    does not release its notch - the centre is left alone and the coefficients
 *    are rebuilt from the same number. The alternative is a notch that jumps
 *    across the band every time a marginal peak flickers, which is a worse
 *    filter than one sitting where the resonance was a moment ago.
 *
 * 9. **The coefficients are rebuilt every update, unconditionally.** A
 *    change-detection guard - the shape the gyro chain uses, three float
 *    compares per sample - would be wrong here and the reason is worth stating:
 *    the chain's cutoffs are parameters that change when a person types, and
 *    this one's centres change on their own every window. Rebuilding is
 *    `count` sine-and-cosine pairs per axis per window: at 8 kHz with five
 *    notches, 312 pairs a second. The guard would cost more than it saves.
 *
 * 10. **`ak_filter_biquad_notch_set` preserves the filter's state and
 *     `..._init` clears it.** That is the library's contract and it is exactly
 *     what a tracking notch needs: moving a notch's centre by a few hertz must
 *     not restart it, or every update would put a step into the gyro. A slot
 *     engaging for the first time *is* started from nothing, because there is
 *     nothing to preserve.
 *
 * 11. **Q is configured in the reference's units: hundredths.** Betaflight's
 *     `dyn_notch_q` defaults to 300 and is divided by 100 before use, so a
 *     person's "300" means Q 3 everywhere. That is an awkward unit and it is
 *     kept anyway, because a name that transfers tuning knowledge has to
 *     transfer its units too - the alternative is a table where a number copied
 *     from a Betaflight dump tells this aircraft something else. The division
 *     happens once, at the flight core's edge, and never inside this module.
 *
 * 12. **A loop rate below the gate leaves the bank a wire.** `enabled` covers
 *     the whole module: when it is 0 there is no measurement *and* no
 *     filtering, so a board booted at 1 kHz passes its gyro through untouched
 *     rather than notching it at a frequency the sample rate cannot represent.
 */

#define AK_DYN_NOTCH_AXES        3
#define AK_DYN_NOTCH_COUNT_MAX   5    /* the roadmap's 1..5 */
#define AK_DYN_NOTCH_FFT_POINTS  64   /* the window this module chose; see
                                       * decision 2, and the arrays below are
                                       * sized by it */

/* Below this loop rate the dynamic notch cannot do its job - see decision 4
 * above. The reference's own DYN_NOTCH_UPDATE_MIN_HZ. */
#define AK_DYN_NOTCH_UPDATE_MIN_HZ 2000.0f

/* The tracking PT1's cutoff at the floor, and how much faster it is allowed to
 * run for a peak far above it. The reference's DYN_NOTCH_SMOOTH_HZ and its
 * constrainf(..., 1, 10). */
#define AK_DYN_NOTCH_SMOOTH_HZ     4.0f
#define AK_DYN_NOTCH_MAX_SPEEDUP  10.0f

/* What a notch is built at when nobody says otherwise, in real Q units. The
 * reference's `dyn_notch_q` default is 300 hundredths - see decision 11. */
#define AK_DYN_NOTCH_Q_DEFAULT     3.0f

/* The band the reference defaults to, and the one this test file's constants
 * agree with. Here rather than only in the flight core because a band with no
 * width is a case this module has to answer for on its own. */
#define AK_DYN_NOTCH_MIN_HZ_DEFAULT 100.0f
#define AK_DYN_NOTCH_MAX_HZ_DEFAULT 600.0f

/*
 * Why the module is not running, when it is not. `enabled` is the answer and
 * this is the sentence - a caller that only had the flag would have to guess
 * from the loop rate and the band, and guessing is exactly what a module that
 * refuses three different ways should not ask of its callers.
 *
 * AK_DYN_NOTCH_OFF_COUNT is the one value this module never sets: a count of
 * zero is clamped up to one here (a module asked to find no peaks has nothing
 * to do), so "the configuration asked for none" is a thing only the caller
 * knows. The caller sets it; the module leaves it alone everywhere else.
 */
typedef enum {
    AK_DYN_NOTCH_RUNNING  = 0,  /* enabled */
    AK_DYN_NOTCH_OFF_COUNT,     /* the caller asked for no notches */
    AK_DYN_NOTCH_OFF_RATE,      /* the loop is under AK_DYN_NOTCH_UPDATE_MIN_HZ */
    AK_DYN_NOTCH_OFF_BAND,      /* max_hz <= min_hz: no band to search */
} ak_dyn_notch_off_t;

typedef struct {
    /* The decimated gyro, per axis, newest last. A window is filled completely
     * and then handed to the transform; it does not slide. The consequence is
     * stated rather than hidden: a transform happens once per window, not once
     * per call, and `dropped` counts the axis-samples that arrived while a full
     * window was still waiting to be consumed. */
    float    buf[AK_DYN_NOTCH_AXES][AK_DYN_NOTCH_FFT_POINTS];
    float    accum[AK_DYN_NOTCH_AXES];

    /* The transform's scratch, in the struct for the same reason everything
     * else is: no allocation, and a caller cannot forget to provide it. */
    float    fft_re[AK_DYN_NOTCH_FFT_POINTS];
    float    fft_im[AK_DYN_NOTCH_FFT_POINTS];

    /* The spectrum in *power*, not magnitude - the reference's units, so its
     * floor factor and its parabola mean what they mean there. Sized to the
     * whole transform rather than to the mirror-symmetric half on purpose: a
     * peak at the top of the band reads its upper shoulder at bin+1, and a
     * half-sized array would make that read one past the end. */
    float    power[AK_DYN_NOTCH_FFT_POINTS];

    /* What a caller reads: where each axis's peaks are, ascending in frequency.
     * `have[axis]` is how many of the `count` slots hold a live estimate - the
     * rest are the spread-out starting guesses and must not be trusted as
     * measurements. */
    float    centre_hz[AK_DYN_NOTCH_AXES][AK_DYN_NOTCH_COUNT_MAX];
    uint8_t  have[AK_DYN_NOTCH_AXES];

    /*
     * The bank. `notch[axis][slot]` is the section that filters `axis` at
     * `centre_hz[axis][slot]`; `engaged[axis][slot]` says whether a real
     * measurement has ever landed in that slot, which is what decides whether
     * the section is applied at all - see decision 7.
     *
     * `q` is in real Q units here and not in the parameter's hundredths: the
     * division belongs at the flight core's edge, where a person's number
     * becomes this module's, and doing it anywhere else would leave two places
     * that know what unit a Q is in. `notch_dt` is the *loop's* interval, not
     * the transform's - the biquads run at the loop rate.
     */
    ak_filter_biquad_t notch[AK_DYN_NOTCH_AXES][AK_DYN_NOTCH_COUNT_MAX];
    uint8_t  engaged[AK_DYN_NOTCH_AXES][AK_DYN_NOTCH_COUNT_MAX];
    float    q;
    float    notch_dt;

    float    fs_hz;      /* the decimated sample rate */
    float    bin_hz;     /* fs_hz / points: what a bin is worth in Hz */
    float    floor;      /* the last measured noise floor, for reporting */
    float    min_hz;     /* the band the search may look in */
    float    max_hz;
    uint16_t first_bin;
    uint16_t last_bin;

    /* The last transform's time per axis, for differencing the PT1's dt. */
    uint32_t last_us[AK_DYN_NOTCH_AXES];

    uint16_t decim_by;               /* loop samples per analysis sample */
    uint16_t decim;                  /* the ones accumulated so far */
    uint16_t fill[AK_DYN_NOTCH_AXES];/* decimated samples in each axis's window */
    uint16_t points;

    uint8_t  axis;       /* which axis the next update will transform */
    uint8_t  count;      /* notches wanted, 1..AK_DYN_NOTCH_COUNT_MAX */
    uint8_t  enabled;    /* 0 when the loop rate is under the gate */
    uint8_t  off;        /* ak_dyn_notch_off_t: why, when enabled is 0 */

    uint32_t transforms;
    uint32_t dropped;    /* axis-samples discarded because that window was full */
    uint32_t skipped;    /* transforms that found no peak above the floor */

    /* What the bank did. `coefficient_sets` counts rebuilds, which is the
     * module's slow-tier cost made countable, and `centre_clamps` counts the
     * centres the filter library had to pull down because they were above a
     * quarter of the loop rate - a notch that is not where the measurement
     * said, reported rather than left to be discovered. */
    uint32_t coefficient_sets;
    uint32_t centre_clamps;
} ak_dyn_notch_t;

/*
 * Set the module up for a loop running at `loop_hz`, searching `min_hz` to
 * `max_hz` for up to `count` peaks, with `points`-point transforms.
 *
 * `loop_hz` at or under AK_DYN_NOTCH_UPDATE_MIN_HZ leaves `enabled` at 0 and
 * every other call a no-op - the caller is expected to read `enabled` and say
 * so rather than present a notch that is not running. So does a band with no
 * width (`max_hz <= min_hz`): the module refuses rather than parking a notch at
 * a frequency the caller did not choose.
 *
 * `min_hz` below one bin and `max_hz` above the decimated Nyquist are both
 * clamped, and `bin_hz` and the resulting band are readable afterwards. `points`
 * is accepted only when it is the window this module's arrays are sized for
 * (AK_DYN_NOTCH_FFT_POINTS); anything else falls back to it rather than
 * overflowing, which is why raising that constant resizes the window and the
 * buffers together.
 *
 * The notch's Q starts at AK_DYN_NOTCH_Q_DEFAULT and is moved by
 * `ak_dyn_notch_set_q`. Init is the one place every buffer and every filter's
 * state is defined, so it is safe to call on a module that has been running.
 */
void ak_dyn_notch_init(ak_dyn_notch_t *d, float loop_hz, float min_hz,
                       float max_hz, uint8_t count, uint16_t points);

/*
 * The Q the notches are built at, in real Q units. A value at or under
 * AK_FILTER_Q_MIN is clamped by the filter library when the coefficients are
 * built; a value below 1 is a filter wider than its own centre, which is a
 * shape this library will not make.
 *
 * Safe to call at any time. A rebuild happens on the next update of each axis,
 * and because the library preserves a filter's state across a coefficient
 * change, changing Q does not restart the notches - see decision 10.
 */
void ak_dyn_notch_set_q(ak_dyn_notch_t *d, float q);

/*
 * Hand the module one loop-rate gyro sample, per axis. Returns immediately when
 * the module is disabled. The decimation happens here: the sample is added to a
 * running sum, and every `decim_by`-th call the mean of those lands in the
 * window. A sample that arrives with the window already full is counted in
 * `dropped` and discarded.
 *
 * This is the *unfiltered* sample. A caller that had already filtered it would
 * be measuring its own filters - see the flight core, which pushes the driver's
 * reading and notches it before anything else touches it.
 */
void ak_dyn_notch_push(ak_dyn_notch_t *d, const float gyro[3]);

/*
 * One axis, through its engaged notches. `x` is the raw gyro and the return is
 * what is left of it, which is `x` itself when the module is disabled or when
 * this axis has never reported a peak. Slots are applied in ascending frequency
 * order, which is the order `centre_hz` holds them in.
 */
float ak_dyn_notch_filter(ak_dyn_notch_t *d, uint8_t axis, float x);

/*
 * How many notches are engaged on one axis. The count is per axis and it is a
 * reading, not a configuration: `count` is what was asked for, this is what a
 * measurement has actually put in place.
 */
unsigned ak_dyn_notch_engaged_notches(const ak_dyn_notch_t *d, uint8_t axis);

/*
 * Transform one axis, if a whole window is waiting. Returns the axis it
 * transformed, or -1 when there was nothing to do - disabled, a partial window,
 * or every peak sitting at the noise floor. `now_us` is the caller's clock; the
 * module differences it against the last transform of the same axis to get the
 * tracking filter's own time step, so nothing here assumes the caller's period.
 *
 * One axis per call, round-robin: see decision 3 in the header above.
 */
int ak_dyn_notch_update(ak_dyn_notch_t *d, uint32_t now_us);

#endif /* AK_CORE_AK_DYN_NOTCH_H */
