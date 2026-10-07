#ifndef AK_FLIGHT_DSHOT_GCR_H
#define AK_FLIGHT_DSHOT_GCR_H

#include <stdint.h>

#include "ak_types.h"

/*
 * The reply half of bidirectional DShot: the levels off the wire, to an eRPM.
 *
 * DShot goes one way, and in the bidirectional variant the ESC answers after
 * every frame: it pulls the wire it was just driven on and sends twenty-one
 * levels - a start bit, then four 5-bit GCR symbols carrying a 16-bit word
 * whose low nibble is chosen so that the whole word folds to 0xF.
 *
 * This file is the arithmetic of that reply and nothing else: twenty-one levels
 * in, a telemetry value out. That is why it can be checked on a host, and it is
 * the same split ak_dshot_timing.h makes for the frame going the other way -
 * deciding the compare values is arithmetic, putting them in a register at the
 * right instant is the port's job and needs a scope. The half that is *not*
 * here is the one that needs a scope: turning edges into those twenty-one
 * levels (input capture, the DMA buffer, where the frame starts, how many
 * samples a bit lasts).
 *
 * The levels are in **wire order**: `bits[0]` is the first thing that arrived -
 * the start bit - and `bits[20]` the last. The reference assembles the same
 * frame into a 32-bit word with the start bit at bit 20 and reads it there;
 * taking the levels in the order they arrived keeps the convention a port can
 * get wrong - which end is which - out of the port.
 *
 * A note on what this cannot tell apart: a bidirectional ESC that has been
 * asked for extended telemetry mixes temperature, voltage and current frames
 * into the same field, and the 0x0fff that means "not turning" here is one of
 * that family's markers too. Nothing sends those frames until this firmware
 * asks for them (roadmap 3.3), so every value that arrives now is an eRPM - but
 * that is a statement about what was asked for, not something this file can
 * see, and it is written down here so a later reader does not mistake the
 * silence for a check.
 */

#define AK_DSHOT_GCR_BITS 21u /* one start bit and four 5-bit symbols */

/* A group of five levels that is not one of the sixteen codes. */
#define AK_DSHOT_GCR_BAD 0xFFu

/* A reply taken apart as far as it goes without judging it: what a caller needs
 * to describe a frame it is about to refuse, and to say which part was wrong.
 * A `nibbles` entry is 0..15 or AK_DSHOT_GCR_BAD. */
typedef struct {
    uint8_t  nibbles[4]; /* the four symbols, most significant first */
    uint8_t  bad;        /* the index of the first BAD one, or 4 for none */
    uint16_t word;       /* the sixteen bits they carry, 0 if one is not a code */
    uint8_t  fold;       /* the fold of `word`, 0 if there is no word */
} ak_dshot_gcr_frame_t;

void ak_dshot_gcr_read(const uint8_t *bits, ak_dshot_gcr_frame_t *frame);

/* How a reply was refused. Ordered so that the common case is zero. */
typedef enum {
    AK_DSHOT_GCR_OK = 0,
    AK_DSHOT_GCR_BAD_SYMBOL, /* a 5-bit group that is not one of the 16 codes */
    AK_DSHOT_GCR_BAD_CRC,    /* four legal symbols, and the fold is not 0xF */
} ak_dshot_gcr_t;

/* The fold of a 16-bit word: the two shifts the reference checks a frame with,
 * reduced to the low nibble. A reply is good when this is 0xF.
 *
 * The frame this firmware *sends* folds to 0 under the same function, which is
 * the "the crc is inverted when bidirectional telemetry is in use" that
 * docs/03-attribution.md records - so the two halves of this project's DShot
 * code agree about the fold and disagree about the constant, and
 * tests/test_dshot_gcr.c holds both to it with vectors that predate this file.
 */
uint8_t ak_dshot_gcr_fold(uint16_t word);

#define AK_DSHOT_GCR_FOLD_OK 0xFu

/* One reply: `bits` is AK_DSHOT_GCR_BITS levels in wire order. On
 * AK_DSHOT_GCR_OK, `*value` is the 12-bit telemetry value the ESC sent.
 *
 * The start bit is discarded and not checked, which is what the reference does.
 * A port that slices the frame one level late is usually caught by the fold but
 * not always - one word in sixteen folds to 0xF whatever it was - and what
 * catches the rest is a real capture read back beside the ESC's own number, not
 * a stricter function here. */
ak_dshot_gcr_t ak_dshot_gcr_decode(const uint8_t *bits, uint16_t *value);

/*
 * And the telemetry value's own packing, which is a period rather than a speed.
 *
 * The value is twelve bits, and that is the whole field: three of exponent and
 * nine of mantissa.
 *
 *   bit 11..9   exponent
 *   bit  8..0   mantissa
 *   period = mantissa << exponent, in microseconds per electrical revolution
 *
 * so a motor that turns faster sends a smaller number, and the value that falls
 * out of the arithmetic below is eRPM/100 - the unit the reference works in,
 * and the reason the figure reported here is multiplied by a hundred.
 *
 * The reference writes the packing out as `eeem mmmm mmmm`, and then masks
 * `0xfe00` and `0x01ff` of a *sixteen*-bit variable. On a twelve-bit value those
 * masks are exactly bits 11..9 and 8..0, so the two readings agree about
 * everything a reply can carry - and this file states the width, because a
 * sixteen-bit reading of the same masks is what a caller gets if it hands over
 * the whole decoded word instead of the value, and that fails silently rather
 * than loudly. `ak_dshot_period_us` therefore refuses anything wider than the
 * field: see the note in ak_dshot_gcr.c, where the arithmetic that makes the
 * refusal worth having is measured.
 */

/* The widest value a reply can carry. Everything above this is not a field of a
 * reply at all - most likely a caller that handed in the sixteen-bit word whose
 * top nibble is the frame's check rather than part of the number. */
#define AK_DSHOT_VALUE_MASK 0x0FFFu

/* The period the value carries, in microseconds per electrical revolution: 0
 * for a value that does not carry one. That is a mantissa of zero (the
 * reference's `if (!value) return INVALID`), the "not turning" marker - whose
 * bits are an ordinary period, 65408 us, and are not a period in fact, so it
 * has to be caught before the arithmetic rather than by it - or a number wider
 * than the twelve bits a reply carries, which is not a reply. */
uint32_t ak_dshot_period_us(uint16_t value);

typedef enum {
    AK_DSHOT_ERPM_TURNING = 0, /* *erpm is the electrical rpm */
    AK_DSHOT_ERPM_STOPPED,     /* the ESC said 0x0fff: not turning; *erpm is zero */
    /* Not a period at all, and *erpm is left as the caller had it: writing a
     * zero here would be this function claiming the motor had stopped, and the
     * verdict is the only thing that can say "this number means nothing". The
     * reference behaves the same way - it keeps the motor's last telemetry value
     * rather than overwriting it - which is why RPM-filter state must be driven
     * by the verdict and not by the number. */
    AK_DSHOT_ERPM_INVALID,
} ak_dshot_erpm_t;

ak_dshot_erpm_t ak_dshot_erpm_from_value(uint16_t value, uint32_t *erpm);

/* Electrical rpm to what a tachometer would read: a motor with `poles` magnetic
 * poles turns once per `poles / 2` electrical revolutions. Zero for a pole
 * count below two, which is not a motor rather than a division to guard. */
uint32_t ak_dshot_rpm(uint32_t erpm, unsigned poles);

#endif /* AK_FLIGHT_DSHOT_GCR_H */
