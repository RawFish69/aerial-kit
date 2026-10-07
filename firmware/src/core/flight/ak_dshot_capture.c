#include "ak_dshot_capture.h"

#include "ak_dshot_gcr.h"

/*
 * The reference's loop, over runs instead of over samples.
 *
 * A transcription note first, because this is one: the reference's
 * `decode_bb_bitband` walks a buffer one sample at a time and finds its edges
 * with unrolled scans. Everything it does with those edges is two lines -
 *
 *     const int len = MAX((p - oldP + 1) / 3, 1);
 *     bits += len;
 *     value <<= len;
 *     value |= 1 << (len - 1);
 *
 * - and those two lines are the whole of this file's arithmetic. What is dropped
 * is the walking: the scans exist to find where a run ends inside a flat sample
 * buffer, and a run list already knows. What is deliberately *not* dropped is
 * the `+ 1` and the `/ 3`, and the reason is in the header's decision 4: they
 * are not a rounding of n/3, they are the reference's own two off-by-ones
 * cancelling, and writing `samples / 3` here instead would slice every reply by
 * a bit at its head.
 *
 * The other thing kept is that the reference emits the bits of a run as a
 * leading one and then zeros, whatever the run's level was. A run that is high
 * and a run that is low of the same length carry the same bits, which is the
 * counter-intuitive heart of this codec (header, decision 1).
 */

unsigned ak_dshot_capture_run_bits(uint16_t samples)
{
    /* `MAX(..., 1)`: a run the capture recorded is a run that happened, and one
     * shorter than a bit - two samples of the three that a bit is worth - still
     * carries the one bit that the edge after it is the end of. The reference
     * clamps for the same reason and at the same place. */
    unsigned len = ((unsigned)samples + 1u) / AK_DSHOT_CAPTURE_OVERSAMPLE;

    return len > 0u ? len : 1u;
}

/* Puts `len` bits at `*at`: a one and then zeros, which is the shape the
 * reference's `value |= 1 << (len - 1)` produces read from the front. Returns
 * zero if that many bits would not fit, and writes nothing. */
static int put_run(uint8_t *bits, unsigned *at, unsigned len)
{
    if (len == 0u || *at + len > AK_DSHOT_GCR_BITS) {
        return 0;
    }

    bits[*at] = 1u;
    for (unsigned i = 1u; i < len; i++) {
        bits[*at + i] = 0u;
    }
    *at += len;
    return 1;
}

ak_dshot_capture_t ak_dshot_capture_slice(const ak_dshot_run_t *runs,
                                          unsigned count, uint8_t *bits)
{
    unsigned at = 0u;
    unsigned i = 0u;

    for (unsigned b = 0u; b < AK_DSHOT_GCR_BITS; b++) {
        bits[b] = 0u;
    }

    /* Decision 3: the pin is high while this firmware's own frame is going out
     * and stays high until the ESC pulls it, so the reply begins at the first
     * run that is low. Everything before is skipped whole rather than counted
     * and discarded, which matters: a leading high counted as a run would carry
     * bits and shift the whole frame. */
    while (i < count && runs[i].level != 0u) {
        i++;
    }

    if (i >= count) {
        /* The line was high for the whole capture. Either no frame was sent on
         * this wire or no ESC answered it - and this module cannot tell those
         * apart, which is why the verdict names the observation and not a
         * cause. */
        return AK_DSHOT_CAPTURE_NO_LOW;
    }

    for (; i < count; i++) {
        unsigned len = ak_dshot_capture_run_bits(runs[i].samples);

        if (at + len > AK_DSHOT_GCR_BITS) {
            /* More runs than a reply can hold. The reference reaches the same
             * verdict by a different road - its `nlen = 21 - bits` goes negative
             * - and the difference between the two is only what a describing
             * caller gets back, because both refuse. What bounds this here is
             * the bit count and what bounds it there is the sample window
             * (`MAX_VALID_BBSAMPLES`, 69 samples, which is twenty-three bits at
             * three to the bit): the window is a port's business, because it is
             * the port that decides how much buffer to hand over, and
             * AK_DSHOT_CAPTURE_RUNS_MAX is this side's equivalent. */
            return AK_DSHOT_CAPTURE_TOO_MANY_BITS;
        }

        (void)put_run(bits, &at, len);

        if (at == AK_DSHOT_GCR_BITS) {
            break;
        }
    }

    if (at < AK_DSHOT_CAPTURE_MIN_BITS) {
        return AK_DSHOT_CAPTURE_TOO_FEW_BITS;
    }

    /* Decision 5: the line is idle high after the reply, so where the last run
     * ends is not something a capture can show. The reference computes how many
     * bits are missing and appends them as another `1` and zeros, and that is
     * what happens here - so a capture cut short by the port's own buffer
     * decodes to the same reply as a whole one, and a stretched final run
     * decodes to the same thing too. */
    if (at < AK_DSHOT_GCR_BITS) {
        (void)put_run(bits, &at, AK_DSHOT_GCR_BITS - at);
    }

    return AK_DSHOT_CAPTURE_OK;
}
