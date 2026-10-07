#include "ak_dshot_gcr.h"

/*
 * The 5-bit codes, indexed by the group of five levels as the reference reads
 * it, and the nibble each one carries. There are 32 groups of five bits and 16
 * of them are codes; a frame containing anything else is refused.
 *
 * Transcribed from Betaflight 2026.6.1 `src/main/drivers/dshot_bitbang_decode.c`
 * (`decode_bb_value`'s table) @ 6dbc4218 - docs/03-attribution.md carries the
 * revision and the licence note. The reference writes the sixteen invalid
 * entries as `0xffffffff` and catches them with a later `decodedValue > 0xffff`;
 * here they are a named byte and the refusal is explicit, which is the same
 * behaviour in a shape a reader can check.
 *
 * What the sixteen codes have in common: no code contains three zeros in a row,
 * and none begins or ends with two zeros, so a run of zeros longer than two is
 * a boundary a receiver can count. Fifteen of the sixteen are exactly the
 * groups with that property; `11111` is the one that has it and is still not a
 * code, and it is written down here rather than smoothed over because a rule
 * quoted as "the reason" and then found to have an exception is how a table
 * gets "fixed" by a later reader. `tests/test_dshot_gcr.c` checks the table
 * both ways and names the exception.
 */
static const uint8_t codes[32] = {
    AK_DSHOT_GCR_BAD, AK_DSHOT_GCR_BAD, AK_DSHOT_GCR_BAD, AK_DSHOT_GCR_BAD,
    AK_DSHOT_GCR_BAD, AK_DSHOT_GCR_BAD, AK_DSHOT_GCR_BAD, AK_DSHOT_GCR_BAD,
    AK_DSHOT_GCR_BAD, 9u, 10u, 11u, AK_DSHOT_GCR_BAD, 13u, 14u, 15u,
    AK_DSHOT_GCR_BAD, AK_DSHOT_GCR_BAD, 2u, 3u, AK_DSHOT_GCR_BAD, 5u, 6u, 7u,
    AK_DSHOT_GCR_BAD, 0u, 8u, 1u, AK_DSHOT_GCR_BAD, 4u, 12u, AK_DSHOT_GCR_BAD,
};

uint8_t ak_dshot_gcr_fold(uint16_t word)
{
    uint16_t csum = (uint16_t)(word ^ (word >> 8));
    csum = (uint16_t)(csum ^ (csum >> 4));
    return (uint8_t)(csum & 0x0Fu);
}

/* One group of five levels, to the index it makes in the table above.
 *
 * The first of the five is the top bit of the index, which is the reference's
 * arrangement rather than an obvious one: it assembles the frame into a word
 * with the first bit on the wire at the highest position, and reads the group
 * as `(word >> 15) & 0x1f`. Reversing this - taking the five levels in the
 * order they arrive and shifting them the other way - turns every group into a
 * different table entry, and for a table this sparse that means most frames
 * refused and the rest decoded to a wrong number. */
static uint8_t symbol_of(const uint8_t *five)
{
    uint8_t index = 0u;

    for (unsigned i = 0u; i < 5u; i++) {
        index = (uint8_t)((uint8_t)(index << 1) | (five[i] != 0u ? 1u : 0u));
    }
    return index;
}

void ak_dshot_gcr_read(const uint8_t *bits, ak_dshot_gcr_frame_t *frame)
{
    frame->word = 0u;
    frame->fold = 0u;
    frame->bad = 4u;
    for (unsigned s = 0u; s < 4u; s++) {
        /* Filled in with BAD first, so that a caller printing a refused frame
         * reads a refusal in every group after the one that stopped it rather
         * than whatever the struct held. */
        frame->nibbles[s] = AK_DSHOT_GCR_BAD;
    }

    for (unsigned s = 0u; s < 4u; s++) {
        frame->nibbles[s] = codes[symbol_of(&bits[1u + 5u * s])];

        if (frame->nibbles[s] == AK_DSHOT_GCR_BAD) {
            if (frame->bad == 4u) {
                frame->bad = (uint8_t)s;
            }
            /* Past the bad one the word would be a mixture of what arrived and
             * what is missing, and even the part before it is only a prefix of
             * one - five is not "the word so far" of anything - so it is
             * cleared here and not merely left alone: a caller describing a
             * refused frame gets the symbols and no number to misread. Written
             * because the first version of this returned with the prefix still
             * in `word`, which contradicted the header and read as a value to
             * everything except a caller who checked `bad` first. */
            frame->word = 0u;
            return;
        }
        /* The first symbol on the wire is the most significant nibble, which is
         * the reference's arrangement read from its word. */
        frame->word = (uint16_t)((uint16_t)(frame->word << 4) | frame->nibbles[s]);
    }

    frame->fold = ak_dshot_gcr_fold(frame->word);
}

ak_dshot_gcr_t ak_dshot_gcr_decode(const uint8_t *bits, uint16_t *value)
{
    ak_dshot_gcr_frame_t frame;

    ak_dshot_gcr_read(bits, &frame);

    if (frame.bad != 4u) {
        return AK_DSHOT_GCR_BAD_SYMBOL;
    }
    if (frame.fold != AK_DSHOT_GCR_FOLD_OK) {
        return AK_DSHOT_GCR_BAD_CRC;
    }

    *value = (uint16_t)(frame.word >> 4);
    return AK_DSHOT_GCR_OK;
}

uint32_t ak_dshot_period_us(uint16_t value)
{
    /* Anything wider than the twelve bits a reply carries is not a value from a
     * reply. Refusing it is not pedantry: the reference's masks are written for
     * a sixteen-bit variable, so a caller that hands in the whole decoded word -
     * whose top nibble is the frame's check, not part of the number - gets a
     * *plausible* answer rather than a refused one. 0x1234 read that way is a
     * period of 26624 us, which is 2300 eRPM: a number a motor could have. The
     * measurement is in tests/test_dshot_gcr.c. */
    if (value > AK_DSHOT_VALUE_MASK) {
        return 0u;
    }

    /* The one value that is not a period. Its bits are an ordinary one -
     * exponent 7, mantissa 511, 65408 microseconds - so this cannot be left to
     * the arithmetic below: as a period it decodes to 900 eRPM rather than to a
     * motor that has stopped. */
    if (value == 0x0FFFu) {
        return 0u;
    }

    /* The reference masks 0xfe00 of a sixteen-bit variable; on the twelve-bit
     * value of a reply that is exactly the top three bits, which is the mask
     * written here. */
    unsigned exponent = (unsigned)((value & 0x0E00u) >> 9);

    /* No ceiling on the shift is needed, and none is written: the field's
     * largest value is exponent 7 with mantissa 511, so the shift is at most
     * seven places of a nine-bit number and cannot overflow a uint32_t. The
     * reference's own comment says the packing is three exponent bits; its code
     * masks seven of a variable that is never wider than twelve, which is the
     * same thing until someone feeds it the whole word - see the guard above. */
    return (uint32_t)(value & 0x01FFu) << exponent;
}

ak_dshot_erpm_t ak_dshot_erpm_from_value(uint16_t value, uint32_t *erpm)
{
    if (value == 0x0FFFu) {
        *erpm = 0u;
        return AK_DSHOT_ERPM_STOPPED;
    }

    uint32_t period = ak_dshot_period_us(value);

    if (period == 0u) {
        return AK_DSHOT_ERPM_INVALID;
    }

    /* The reference's arithmetic, which is eRPM/100 rounded: (10^6 * 60 / 100 +
     * period / 2) / period. Written with that constant multiplied out, because
     * the intermediate does not fit anything and the compiler folds it anyway.
     *
     * There is no check here for this coming out zero, and that is a statement
     * with a reason: the field's widest period is 511 << 7 = 65408 us, where
     * this is 9, so a period the field can carry is always reportable. The
     * arithmetic's resolution runs out at about 1.2 seconds per electrical
     * revolution - past twenty times the widest the wire can say - so the only
     * way to reach it is to hand in a number that is not a reply, which the
     * guard above refuses first. tests/test_dshot_gcr.c sweeps all 4096 values
     * of the field and asserts every one of them is reportable. */
    uint32_t erpm100 = (600000u + period / 2u) / period;

    *erpm = erpm100 * 100u;
    return AK_DSHOT_ERPM_TURNING;
}

uint32_t ak_dshot_rpm(uint32_t erpm, unsigned poles)
{
    if (poles < 2u) {
        return 0u;
    }
    /* poles / 2 electrical revolutions per turn, so twice the eRPM over the
     * poles - exactly, which is why this is not written as eRPM / (poles / 2)
     * and does not need an odd count refused on arithmetic grounds. */
    return (erpm * 2u) / poles;
}
