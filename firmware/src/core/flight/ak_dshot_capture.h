#ifndef AK_FLIGHT_DSHOT_CAPTURE_H
#define AK_FLIGHT_DSHOT_CAPTURE_H

#include <stdint.h>

#include "ak_types.h"

/*
 * The reply's edges, to the twenty-one levels ak_dshot_gcr_decode() wants.
 *
 * ak_dshot_gcr.h names this file's job and then says why it is not there: "The
 * half that is *not* here is the one that needs a scope: turning edges into
 * those twenty-one levels (input capture, the DMA buffer, where the frame
 * starts, how many samples a bit lasts)." Half of that sentence is arithmetic,
 * and this is that half. What stays in the port is the DMA buffer and the
 * registers - it is a port's job to fill the run list below, and only a scope
 * can say whether it did.
 *
 * Eleven decisions, because this is a translation of somebody else's loop and
 * every one of them is a place a translation can quietly become a different
 * filter that still passes its own tests.
 *
 * 1. THE REPLY IS A DURATION CODE, NOT A LEVEL CODE. This is the thing to
 *    understand before reading any of it. The ESC does not send a long high for
 *    a '1' and a short one for a '0' the way this firmware's frames go out; it
 *    sends *runs*. A run of n samples at three samples to the bit carries
 *    (n + 1) / 3 bits, and those bits are a '1' followed by that many zeros.
 *    Three samples carry `1`, six carry `10`, nine carry `100` - and the *level*
 *    of the run does not matter at all: the reference counts positive edges and
 *    zero edges with the same two lines of code, and a run of high and a run of
 *    low of the same length carry the same bits. That is why every function here
 *    counts samples and never looks at which direction the line was in except to
 *    find where the frame begins.
 *
 * 2. THREE SAMPLES TO THE BIT. The reference samples the pin at three times the
 *    DShot bit rate, so one bit is three samples. This file takes that as the
 *    constant it is: AK_DSHOT_CAPTURE_OVERSAMPLE. It is not a parameter, because
 *    the arithmetic below is only the reference's arithmetic at three - the
 *    `(n + 1) / 3` is not a formula that generalises, and writing it as
 *    `(n + os - 1) / os` would look like one and be wrong.
 *
 * 3. THE LEADING HIGH IS NOT PART OF THE REPLY. The pin is driven high by this
 *    firmware's own frame and is idle high between frames; what arrives after is
 *    the ESC pulling it. The reference scans forward for the first *low* sample
 *    and starts the frame there, and so does this. A run of high at the front is
 *    skipped whole.
 *
 * 4. EVERY RUN COUNTS THE SAME, AND THAT IS A RESULT RATHER THAN AN
 *    ASSUMPTION. The first low run looks like it should count one short: the
 *    reference stops its scan *on* the first low sample, sets its baseline one
 *    sample past it, and then counts to the next positive edge with a `p++` that
 *    overruns by one. Both off-by-ones are the same one, so the count is
 *    `(n + 1) / 3` for the first run and for every run after it, and the
 *    arithmetic below has one branch fewer than it first looked like it needed.
 *    It is written down because the first draft of this file asserted the
 *    opposite - "the first low run counts n / 3" - from reading the baseline and
 *    not the overrun, and a translation that carries a phantom correction is
 *    wrong by a bit at the frame's head, which is a different number out of the
 *    fold and not a refusal. tests/test_dshot_capture.c pins it by slicing the
 *    same reply with and without a leading sample.
 *
 * 5. THE LAST RUN'S LENGTH IS INFERRED, NOT MEASURED, AND THE INFERENCE IS ONE
 *    GROUP WIDE. The line is idle high after the reply, so the reference cannot
 *    see where the last run ends - its edge scan finds no further edge and the
 *    run it was standing in is never measured. It computes `nlen = 21 - bits`
 *    and appends a '1' followed by `nlen - 1` zeros, and this does the same.
 *
 *    So the case it is for: a capture whose run list ends at the reply's last
 *    run decodes to the whole reply, and a capture whose last run is *absent*
 *    from the list - the buffer ran out, or the port stopped there - decodes to
 *    it too, because the missing bits are reconstructed as exactly the group a
 *    run carries. tests/test_dshot_capture.c measures both.
 *
 *    And what it does not cover, because a single inference cannot: a list that
 *    ends two or more groups early. Those missing bits are then reconstructed as
 *    one '1' and zeros where the wire carried several groups, and the answer is
 *    accepted - if eighteen bits arrived - and is a different reply. The same
 *    test walks every prefix of a real reply and counts it: of the prefixes
 *    carrying eighteen bits or more, the whole reply and the one-cut-short
 *    prefix reproduce it and the rest do not. A run list cannot tell a truncated
 *    capture from a short reply, which is decision 11's boundary again.
 *
 *    The same case found the one place this module is *more* permissive than the
 *    reference, which is a consequence of the same sentence rather than a
 *    choice: a reply whose final group carries four bits or more is refused by
 *    the reference - everything it did not measure is assumed to be one bit long
 *    - and accepted here, because a run list has the run in it and can measure
 *    it. The rate is asserted rather than described: over two thousand replies
 *    the reference refuses exactly those whose final group carries four bits or
 *    more, which is exactly the replies that end in three or more zeros. A real
 *    GCR frame cannot end that way - no code in the codebook ends in two zeros -
 *    so the two walks agree on every frame an ESC can send, and part company
 *    only on words that are not frames.
 *
 * 6. WHAT IS REFUSED, AND THE TWO NUMBERS. Fewer than eighteen bits of runs is
 *    not a reply (the reference's `bits < 18`), and more than twenty-one is not
 *    either (`nlen < 0`). Eighteen is not arbitrary: a reply is twenty-one bits
 *    and the reference allows a reply to lose the last three to a busy ESC CPU.
 *    Both refusals are separate verdicts here rather than one, because "the ESC
 *    sent too little" and "the capture is longer than a frame" are different
 *    findings about the bench.
 *
 *    A run list may also run *past* the frame, and a port that hands over the
 *    idle line after the reply will produce exactly that: the line stays high
 *    and the capture keeps making runs of it. So twenty-one bits ends the frame
 *    and the runs after it are not read, and TOO_MANY fires only when a single
 *    run would *cross* the boundary - which is a malformed capture rather than
 *    an idle line. The reference reaches its answer the other way round, by
 *    bounding its scan to MAX_VALID_BBSAMPLES past the start and refusing
 *    anything longer; that bound is a port's business, because it is the port
 *    that decides how much buffer to hand over, and AK_DSHOT_CAPTURE_RUNS_MAX is
 *    this side's equivalent. Ending the frame at twenty-one bits is the one place
 *    this file decides something the reference decided elsewhere, and the reason
 *    is that the final run merges into the idle high: where the reply stops is
 *    something no run list can show, so something has to say it.
 *
 * 7. NO ADAPTIVE PREAMBLE SKIP. The reference keeps `preambleSkip`, a margin it
 *    tunes every 500 ms so that its scan starts a fixed number of samples ahead
 *    of the leading edge. That is a fix for a *sample buffer* whose start is not
 *    aligned to the reply; a run list starts where the port says the frame
 *    starts, so there is nothing to tune and the mechanism is not reproduced.
 *    It is named here rather than omitted, because a reader comparing this file
 *    with the reference will find that loop and needs to know it was read.
 *
 * 8. THE LEVELS COME OUT IN WIRE ORDER. `bits[0]` is the first thing that
 *    arrived. This is ak_dshot_gcr.h's convention and the reason it exists -
 *    the reference assembles the frame with the first bit at the top of a word
 *    and reads its groups from there, and keeping the port's answer in arrival
 *    order takes "which end is which" away from the port.
 *
 * 9. THE START BIT IS NOT CHECKED HERE EITHER. The reference discards it
 *    (`value &= 0xfffff`) and so does the decoder above this file; this module
 *    hands all twenty-one levels on, start bit included, and does not judge it.
 *
 * 10. THE INPUT IS RUNS, NOT SAMPLES. A port with a PWM-input capture has
 *    (period, high time) per cycle; a port that bit-samples the pin has a bit
 *    per sample. Both are runs, so the input is runs and neither port has to
 *    build the other's buffer. `samples` is the run's length in sample periods
 *    at AK_DSHOT_CAPTURE_OVERSAMPLE to the bit, and a port whose capture counts
 *    in ticks divides by its ticks-per-sample before filling this in.
 *
 * 11. AND WHAT THIS CANNOT TELL YOU. Nothing here knows whether the pin was
 *    actually read at three times the bit rate. Every verdict above is about the
 *    shape of the run list, so a port that samples at the wrong rate produces a
 *    well-formed frame that decodes to a wrong number, and the fold catches only
 *    fifteen sixteenths of those. That is the same boundary ak_dshot_gcr.h draws
 *    around the fold, for the same reason, and the answer is the same: a real
 *    capture read back beside the ESC's own figure. There is no such capture in
 *    tests/fixtures/ and none is claimed.
 */

/* Samples to the bit, which is the reference's oversampling and not a knob. */
#define AK_DSHOT_CAPTURE_OVERSAMPLE 3u

/* The fewest bits of runs that can be a reply, and the most. */
#define AK_DSHOT_CAPTURE_MIN_BITS 18u

/* A run list long enough for any legal frame: twenty-one bits can be carried in
 * as few as one run and in as many as twenty-one, and a capture that ran long
 * needs a few more before the refusal above stops it. Sixty-four is four times
 * what a well-formed frame uses and is a bound, not a target. */
#define AK_DSHOT_CAPTURE_RUNS_MAX 64u

/* One level of the wire and how many samples it lasted. `level` is 0 or 1 and
 * is only read to find where the reply begins (decision 3); the bit count comes
 * from `samples` alone. */
typedef struct {
    uint8_t  level;
    uint16_t samples;
} ak_dshot_run_t;

/* How a capture was refused. Ordered so the good case is zero. */
typedef enum {
    AK_DSHOT_CAPTURE_OK = 0,
    AK_DSHOT_CAPTURE_NO_LOW,        /* the line never went low: no reply arrived */
    AK_DSHOT_CAPTURE_TOO_FEW_BITS,  /* under AK_DSHOT_CAPTURE_MIN_BITS of runs */
    AK_DSHOT_CAPTURE_TOO_MANY_BITS, /* more than the twenty-one a reply carries */
} ak_dshot_capture_t;

/* The bits one run carries, which is (n + 1) / 3 and never fewer than one: a
 * run this module was handed is a run that happened, and a run too short to
 * carry a bit still carries one. There is no first-run variant of this, which
 * is decision 4's whole content. */
unsigned ak_dshot_capture_run_bits(uint16_t samples);

/* Turns `count` runs into AK_DSHOT_GCR_BITS levels in `bits`, in wire order.
 *
 * On AK_DSHOT_CAPTURE_OK the levels are the reply and can go straight to
 * ak_dshot_gcr_decode(). On any other verdict `bits` holds whatever was
 * assembled before the refusal, which is the same courtesy ak_dshot_gcr_read()
 * extends: a caller describing a refusal gets the part that arrived rather than
 * an empty array that says nothing about where it stopped. */
ak_dshot_capture_t ak_dshot_capture_slice(const ak_dshot_run_t *runs,
                                          unsigned count, uint8_t *bits);

#endif /* AK_FLIGHT_DSHOT_CAPTURE_H */
