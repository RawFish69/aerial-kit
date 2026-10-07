#include "filter/ak_fft.h"

#include "flight/ak_math.h"

/*
 * A radix-2 Cooley-Tukey FFT. See ak_fft.h for what the decisions are.
 *
 * The shape is the ordinary one: permute the input into bit-reversed order,
 * then run log2(n) stages of butterflies, each stage doubling the transform
 * length. In place, no allocation, no state between calls.
 */

/* Two pi, as a float, so no `double` literal appears anywhere in this file -
 * ground rule 8 and the -Wdouble-promotion the build carries. */
#define AK_FFT_TWO_PI 6.283185307179586f

int ak_fft_size_ok(uint16_t n)
{
    if (n < AK_FFT_MIN_POINTS || n > AK_FFT_MAX_POINTS) {
        return 0;
    }
    /* A power of two has exactly one bit set, so `n & (n - 1)` is zero. */
    return (n & (uint16_t)(n - 1u)) == 0u;
}

/* log2 of a power of two, by counting the shifts it takes to get there. */
static uint8_t fft_log2(uint16_t n)
{
    uint8_t bits = 0u;

    while (((uint16_t)1u << bits) < n) {
        bits++;
    }
    return bits;
}

int ak_fft_forward(float *re, float *im, uint16_t n)
{
    int      i, k, len, half;
    uint8_t  bits;
    int      count;

    if (re == 0 || im == 0 || !ak_fft_size_ok(n)) {
        return -1;
    }

    count = (int)n;
    bits  = fft_log2(n);

    /* Bit-reversal permutation: element i moves to the position whose index is
     * i's bits read backwards. Swapping in pairs, walking i upward and taking
     * the reversed index only when it is larger, touches every element once. */
    for (i = 0; i < count; i++) {
        int j = 0;

        for (k = 0; k < (int)bits; k++) {
            j = (j << 1) | ((i >> k) & 1);
        }
        if (j > i) {
            float tr = re[i];
            float ti = im[i];
            re[i] = re[j];
            im[i] = im[j];
            re[j] = tr;
            im[j] = ti;
        }
    }

    /* The butterflies. Each stage combines two half-length transforms into one
     * of double the length; the twiddle for the k'th butterfly of a stage of
     * length `len` is exp(-2*pi*i*k/len), from the project's own sine and
     * cosine rather than a table. */
    for (len = 2; len <= count; len <<= 1) {
        half = len >> 1;
        for (i = 0; i < count; i += len) {
            for (k = 0; k < half; k++) {
                float ang = -AK_FFT_TWO_PI * (float)k / (float)len;
                float wr  = ak_cosf(ang);
                float wi  = ak_sinf(ang);
                int   a   = i + k;
                int   b   = a + half;
                float tr  = re[b] * wr - im[b] * wi;
                float ti  = re[b] * wi + im[b] * wr;

                re[b] = re[a] - tr;
                im[b] = im[a] - ti;
                re[a] = re[a] + tr;
                im[a] = im[a] + ti;
            }
        }
    }

    return 0;
}

void ak_fft_window_apply(float *x, uint16_t n, ak_fft_window_t window)
{
    int i;
    int count;

    if (x == 0 || !ak_fft_size_ok(n)) {
        return;
    }
    if (window != AK_FFT_WINDOW_HANN) {
        return; /* rectangular: the samples are already the window */
    }

    count = (int)n;

    /* The *periodic* Hann - denominator n, not n-1. The symmetric form is the
     * right one for filter design and the wrong one here: it puts a zero at
     * both ends of a window that is about to be treated as if it repeated,
     * which lowers the coherent gain off exactly 0.5 and tilts the main lobe.
     * The periodic form's gain is exactly 0.5, which is the number a caller
     * divides by. tests/test_fft.c measures both the gain and where the
     * sidelobes sit rather than taking this comment's word for it. */
    for (i = 0; i < count; i++) {
        float phase = AK_FFT_TWO_PI * (float)i / (float)count;
        x[i] = x[i] * 0.5f * (1.0f - ak_cosf(phase));
    }
}

float ak_fft_bin_magnitude(const float *re, const float *im, uint16_t bin)
{
    float r = re[bin];
    float i = im[bin];

    return ak_sqrtf(r * r + i * i);
}

int ak_fft_peak_bin(const float *re, const float *im, uint16_t n,
                    uint16_t first, uint16_t last)
{
    int   best = -1;
    float best_mag = 0.0f;
    int   i;
    int   top;

    if (re == 0 || im == 0 || !ak_fft_size_ok(n)) {
        return -1;
    }

    top = (int)(n / 2u);

    /* Bin 0 is the mean of the window and is never a vibration; a range that
     * names it starts at 1 instead. A range that ends above n/2 ends at n/2:
     * those bins are not empty, they are the mirror of the ones below, and
     * reporting a peak from them would report the same tone twice. */
    if (first < 1u) {
        first = 1u;
    }
    if (last > (uint16_t)top) {
        last = (uint16_t)top;
    }
    if (first > last) {
        return -1;
    }

    for (i = (int)first; i <= (int)last; i++) {
        float mag = ak_fft_bin_magnitude(re, im, (uint16_t)i);

        if (best < 0 || mag > best_mag) {
            best = i;
            best_mag = mag;
        }
    }

    return best;
}
