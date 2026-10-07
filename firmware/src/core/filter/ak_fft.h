#ifndef AK_CORE_AK_FFT_H
#define AK_CORE_AK_FFT_H

#include <stdint.h>

/*
 * The transform the dynamic notch reads the gyro's spectrum with.
 *
 * A radix-2 Cooley-Tukey FFT, in place, on two caller-owned arrays of `n`
 * floats - one real, one imaginary. 64 to 256 points: the low end is the
 * narrowest window that still resolves two adjacent motor harmonics, and the
 * high end is the widest that fits the slow tier's budget.
 *
 * Five things about this file are decisions:
 *
 * 1. **Radix-2, and ours.** The reference (Betaflight's `dyn_notch_filter.c`)
 *    does not use an FFT at all: it runs a sliding DFT over 72 samples through
 *    CMSIS-DSP, which is a library this project cannot link - the image is
 *    `-nostdlib` and the flight core is MCU-free (ground rule 7). The roadmap
 *    asks for radix-2 and 64-256 points. So this is a different length, a
 *    different algorithm and a different cost from the reference's, chosen
 *    here rather than inherited, and the notch RMS this feeds is not the same
 *    measurement the reference's is.
 *
 * 2. **Twiddles are computed, not tabulated.** `exp(-2*pi*i*k/len)` comes from
 *    the project's own `ak_cosf`/`ak_sinf`, so there is no static table, no
 *    init call, no hidden state, and the same bits on every build and every
 *    host - which is the reason `ak_math.h` exists at all (its `sinf` is a
 *    quadrant reduction and a polynomial, absolute error < 1e-6). The price is
 *    two trig calls per butterfly where a table would cost a load; that price
 *    is paid in the slow tier, not in the control loop, and a host test
 *    measures what it buys.
 *
 * 3. **The input is real, and this does not know that.** A gyro axis is a real
 *    signal, so half of every transform is redundant and a real-input packing
 *    would halve the work. It is not done: the packing trick is the kind of
 *    optimisation that has to be verified against the unpacked transform, and
 *    the slow tier is where the cycles are. `ak_fft_forward` takes an
 *    imaginary array of zeros and returns the conjugate-symmetric spectrum,
 *    and the helpers below only ever look at bins 1..n/2, which are the ones
 *    that carry information.
 *
 * 4. **The window is the caller's choice, and it matters.** A rectangular
 *    window on a tone that is not exactly on a bin leaks across the whole
 *    spectrum, which is how a peak tracker ends up following a sidelobe. Hann
 *    is offered beside it and is computed from the same `ak_cosf`. Which one
 *    the notch uses, and what each costs in peak accuracy, is measured in
 *    tests/test_fft.c rather than asserted here.
 *
 * 5. **No allocation, no failure path in the middle.** Everything is in place
 *    on the caller's arrays; the only thing that can be wrong is `n`, and that
 *    is checked once at the door. `n` is a power of two in
 *    [AK_FFT_MIN_POINTS, AK_FFT_MAX_POINTS] or the call does nothing and says
 *    so.
 */

/* The narrowest and widest transform this will do. */
#define AK_FFT_MIN_POINTS 64u
#define AK_FFT_MAX_POINTS 256u

/* The window a spectrum is looked at through. */
typedef enum {
    /* No window: the raw samples. Best bin resolution, worst leakage - a tone
     * between two bins appears in all of them. */
    AK_FFT_WINDOW_RECT = 0,

    /* Hann: cos^2 taper. The main lobe is three bins wide instead of one and
     * the sidelobes are 31 dB down instead of 13, which is the trade a peak
     * tracker wants. */
    AK_FFT_WINDOW_HANN = 1
} ak_fft_window_t;

/*
 * True if `n` is a power of two inside [AK_FFT_MIN_POINTS,
 * AK_FFT_MAX_POINTS] - the one thing `ak_fft_forward` can refuse.
 */
int ak_fft_size_ok(uint16_t n);

/*
 * Forward transform, in place. `re` and `im` are `n` floats each; on return
 * they hold the spectrum in the usual order (bin k at index k, bins above n/2
 * being the conjugate mirror of the ones below).
 *
 * Returns 0 and does the work, or -1 and touches nothing if `n` is not a size
 * `ak_fft_size_ok` accepts. A null array is also refused rather than
 * dereferenced.
 */
int ak_fft_forward(float *re, float *im, uint16_t n);

/*
 * Apply a window to `x` (n real samples) before the transform.
 *
 * The window is applied in place. This does not renormalise the result: a
 * windowed spectrum is smaller than an unwindowed one by the window's coherent
 * gain (0.5 for Hann), and a caller comparing magnitudes across windows has to
 * account for that itself rather than have it hidden here.
 */
void ak_fft_window_apply(float *x, uint16_t n, ak_fft_window_t window);

/*
 * The magnitude of one bin, `sqrt(re^2 + im^2)`, through the project's own
 * `ak_sqrtf`.
 */
float ak_fft_bin_magnitude(const float *re, const float *im, uint16_t bin);

/*
 * The bin with the largest magnitude in [first, last] inclusive, or -1 if the
 * range is empty, reversed, or entirely outside 1..n/2.
 *
 * `n` is the size the transform was taken at, and is needed here because the
 * range is clamped against it: a caller asking for bins up to 100 of a
 * 64-point transform is asking for the mirror of bins it has already looked
 * at, and this clamps rather than reporting a tone twice.
 *
 * Bin 0 is never returned even if the range names it: it is the mean of the
 * window, and a gyro's mean is the bias the calibration already removed, not a
 * vibration. The range is the caller's because the frequency band a notch may
 * look in is a decision, and the top of it is not n/2 - see the note in
 * ak_dyn_notch.h about what the upper bins are worth.
 */
int ak_fft_peak_bin(const float *re, const float *im, uint16_t n,
                    uint16_t first, uint16_t last);

#endif /* AK_CORE_AK_FFT_H */
