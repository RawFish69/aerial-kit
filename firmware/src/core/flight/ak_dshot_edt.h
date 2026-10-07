#ifndef AK_FLIGHT_DSHOT_EDT_H
#define AK_FLIGHT_DSHOT_EDT_H

#include <stdint.h>

#include "ak_types.h"
#include "ak_dshot_gcr.h"

/*
 * Extended DShot telemetry: what a bidirectional reply is when it is not an
 * eRPM, and how often the replies are wrong.
 *
 * ak_dshot_gcr.h decodes twenty-one levels into a twelve-bit value and stops,
 * because while nothing asks an ESC for extended telemetry every reply *is* an
 * eRPM. This file is the other half of that sentence: once a motor has been
 * asked, the same twelve bits carry one of eight things, and which one is not
 * visible in the bits alone - it is a property of what this motor has sent
 * before. That is why the state here is per motor and not a pure function.
 *
 * The law is transcribed from the reference's `dshot_decode_telemetry_value`,
 * and the thirteen decisions below are the parts of it a reader would get
 * wrong from the code alone. They are numbered so the tests can name them.
 *
 *   1. **The type field is four bits, not three, and one of them is a marker.**
 *      `type = (value & 0x0f00) >> 8`. On a sixteen-bit variable the reference
 *      masks the same way (0xfe00 for the period, and `& 0x0f00` here); the
 *      field is twelve bits wide (ak_dshot_gcr.h's decision), so bits 11..8 are
 *      the whole of it.
 *
 *   2. **eRPM is the marker set *or* the field zero** -
 *      `is_erpm = !edt || (type & 1) || (type == 0)`. This is the reference's
 *      most surprising rule and a naive reading gets it wrong twice. `type == 1`
 *      is both the lowest value with the marker set and, by the table below,
 *      the tag for temperature - and the law says eRPM, because the marker
 *      being set means "the whole field is a period, the rest of it is not a
 *      tag". The consequence is worth stating plainly: **temperature first
 *      arrives as `type == 2`, never 1**, and the decoded type is *half* the
 *      field, so the only field values that reach an extended type at all are
 *      2, 4, 6, 8, 10, 12 and 14.
 *
 *   3. **The payload is eight bits; the eRPM period is nine.** Two widths out
 *      of one field, decided by which branch ran: `value & 0x00ff` here,
 *      ak_dshot_gcr.h's three-exponent/nine-mantissa period there. A reader who
 *      reuses one width for both gets a plausible number either way.
 *
 *   4. **The index is `type >> 1`** into the reference's eight-entry table
 *      {eRPM, TEMPERATURE, VOLTAGE, CURRENT, DEBUG1, DEBUG2, DEBUG3,
 *      STATE_EVENTS} - dropping the marker bit, which is the low bit of the
 *      type field and therefore the *last* one shifted out.
 *
 *   5. **Only four of the eight have a unit this firmware can state**: eRPM
 *      (ak_dshot_gcr.h's), temperature in whole degrees Celsius, voltage in
 *      250 mV counts and current in 1 A counts - 0..63.75 V and 0..255 A, which
 *      is why both use the whole byte. DEBUG1..3 and STATE_EVENTS are the ESC's
 *      own numbers: the byte is returned with that said about it and no unit is
 *      invented for it. That is the same refusal ak_dshot_gcr.h makes about a
 *      pole count, for the same reason.
 *
 *   6. **A temperature of zero means none has arrived.** The reference fills
 *      `dest->temperature` with 0 when the type has never been seen
 *      (`edt && (telemetryTypes & ...) ? data : 0`), so zero is this encoding's
 *      "not sent" as well as a real reading of 0 degrees. `max_temp` carries
 *      the same ambiguity and is kept as the reference keeps it: a session
 *      maximum of the raw byte, 0 until the first temperature frame.
 *
 *   7. **Extended telemetry is a property of the motor's history, not of the
 *      reply.** `edt = edt_always || (seen & ~1) != 0`: once any extended type
 *      has arrived, every later reply to that motor is judged by the marker
 *      instead of being read as an eRPM. So the same twelve bits decode
 *      differently before and after the first such frame, and a test that
 *      decodes one reply in isolation is testing the case where the answer is
 *      "eRPM" whatever the bits say. This is the reference's law and it is
 *      kept, not simplified.
 *
 *   8. **The quality window is a rolling second of ten 100 ms buckets** - the
 *      reference's shape - **with one deliberate divergence.** The reference
 *      clears only the bucket it enters, so a loop that stalls for more than
 *      one bucket leaves counts from buckets the window claims to have
 *      dropped: the rate then reports history. This module clears every bucket
 *      between the last one and the new one. The divergence is here because
 *      phase 3's acceptance is an error *rate over five minutes*, and a
 *      five-minute measurement is exactly where a stall happens.
 *
 *   9. **No packets is not a rate of zero.** `ak_dshot_edt_quality_per_10k`
 *      answers -1 when the window holds nothing, because "0 % errors" and
 *      "nothing arrived" are different claims about the aircraft.
 *
 *  10. **The counters count replies, not values**, and the arbitration stays
 *      with the caller. `ak_dshot_edt_update` takes the verdict of the caller's
 *      own law (ak_dshot_gcr_decode's) as `valid` and does not re-decide it -
 *      one call per reply, whatever it carried. A frame that folds and carries
 *      no period - the 0x0fff "not turning" marker, or a zero mantissa - is a
 *      *valid packet* carrying a fact about the motor, and is not a wire error.
 *      The reference defines `updateDshotTelemetryQuality` and this tree's copy
 *      of it has no call site at all, so what counts as invalid is not settled
 *      there; this is AerialKit's arbitration and it is written down as such.
 *
 *  11. **Not every reply is stored, and which ones are is the reference's
 *      asymmetry rather than a tidy rule.** The reference stores a value only
 *      when it is not `DSHOT_TELEMETRY_INVALID` (0xffff), and only one path
 *      returns that: a zero period. So "not turning" - which the eRPM decoder
 *      returns as a *zero* - is stored, and sets the motor's eRPM-seen bit,
 *      while a reply carrying no period is stored nowhere and leaves `seen`
 *      alone. The two are one number apart in the field and are kept apart
 *      here (`AK_DSHOT_EDT_STOPPED` against `AK_DSHOT_EDT_BAD_ERPM`) for that
 *      reason. An extended payload of zero *is* stored, for the same reason.
 *
 *  12. **The stored value is thirty-two bits wide, and that is a divergence
 *      from the reference rather than a transcription of it.** The reference's
 *      `telemetryData[]` is `uint16_t` and takes the eRPM decoder's answer
 *      straight into it, and that answer runs to 30 000 000 for the fastest
 *      period the field can carry (ak_dshot_gcr.h's exponent boundary) - so the
 *      reference truncates silently above 65 535 eRPM. A truncated speed is
 *      not a refused one: it is a plausible wrong number handed to whatever
 *      uses it, which is the failure this project refuses everywhere else. The
 *      array is `uint32_t` here for that reason, at 128 bytes of state for four
 *      motors against the reference's 64. The extended payloads are eight bits
 *      and are unaffected either way.
 *
 *  13. **Nothing here runs in the flight chain yet.** Same boundary as 3.2a:
 *      reaching the module needs an ESC that answers a frame, and there is none
 *      on this bench, so the image is unchanged and the evidence says so.
 */

/* The four bits of the field that carry the type, and the one marker inside
 * them. AK_DSHOT_EDT_MARKER is bit 8 - `value & 0x0100` - and a set marker is
 * what decision 2 calls "odd". */
#define AK_DSHOT_EDT_TYPE_MASK 0x0F00u
#define AK_DSHOT_EDT_TYPE_SHIFT 8u
#define AK_DSHOT_EDT_MARKER 0x0100u

/* The payload of an extended type: eight bits, where the eRPM branch of
 * ak_dshot_gcr.h reads nine. */
#define AK_DSHOT_EDT_PAYLOAD_MASK 0x00FFu

/* The eight decoded types, in the reference's order. */
#define AK_DSHOT_EDT_TYPE_COUNT 8u

/* Bits that mean "this motor has sent extended telemetry": every type but
 * eRPM, which is the reference's DSHOT_EXTENDED_TELEMETRY_MASK. */
#define AK_DSHOT_EDT_EXTENDED_MASK 0xFEu

/* Units, each as the count the ESC sends. */
#define AK_DSHOT_EDT_MV_PER_COUNT 250u
#define AK_DSHOT_EDT_MA_PER_COUNT 1000u

/* The rolling window: ten buckets of 100 ms, a second of replies. */
#define AK_DSHOT_EDT_BUCKET_MS 100u
#define AK_DSHOT_EDT_BUCKETS 10u

typedef enum {
    /* The reference's table, indexed by `type >> 1`. */
    AK_DSHOT_EDT_ERPM = 0,
    AK_DSHOT_EDT_TEMPERATURE = 1,
    AK_DSHOT_EDT_VOLTAGE = 2,
    AK_DSHOT_EDT_CURRENT = 3,
    AK_DSHOT_EDT_DEBUG1 = 4,
    AK_DSHOT_EDT_DEBUG2 = 5,
    AK_DSHOT_EDT_DEBUG3 = 6,
    AK_DSHOT_EDT_STATE_EVENTS = 7,
} ak_dshot_edt_type_t;

/* Whether the value that came back has a unit this firmware can state. Ordered
 * with the answer it can act on first. */
typedef enum {
    AK_DSHOT_EDT_KNOWN_UNIT = 0,   /* eRPM, temperature, voltage, current */
    AK_DSHOT_EDT_ESC_DEFINED = 1,  /* debug1..3, state/events: the ESC's own byte */
} ak_dshot_edt_units_t;

/* What ak_dshot_edt_update() did, ordered with the good case zero. */
typedef enum {
    AK_DSHOT_EDT_OK = 0,      /* stored; the type has a unit (decision 5) */
    AK_DSHOT_EDT_NO_UNIT = 1, /* stored; the byte is the ESC's to explain */
    AK_DSHOT_EDT_STOPPED = 2, /* the eRPM field carried 0x0fff: not turning */
    AK_DSHOT_EDT_BAD_ERPM = 3, /* the eRPM field carried no period; not stored */
    AK_DSHOT_EDT_BAD_FRAME = 4, /* the caller's own law refused the reply */
    AK_DSHOT_EDT_BAD_MOTOR = 5, /* no such motor in this state */
} ak_dshot_edt_status_t;

/* One reply, decoded. */
typedef struct {
    ak_dshot_edt_type_t type;
    ak_dshot_edt_units_t units;
    uint8_t payload;  /* the low eight bits, always, whatever the type */
    uint8_t marker;   /* the field's bit 8 as it arrived */
    uint32_t erpm;    /* type == AK_DSHOT_EDT_ERPM and the period was legal */
    uint32_t celsius;        /* type == AK_DSHOT_EDT_TEMPERATURE */
    uint32_t millivolts;     /* type == AK_DSHOT_EDT_VOLTAGE */
    uint32_t milliamps;      /* type == AK_DSHOT_EDT_CURRENT */
} ak_dshot_edt_value_t;

/*
 * The rolling window of one motor's replies. `bucket` is the quotient
 * `now_ms / AK_DSHOT_EDT_BUCKET_MS` and not an index into the arrays - it is
 * kept whole so that "how many buckets went by" is a subtraction rather than a
 * subtraction modulo ten, which is where the reference's version of this
 * (decision 8) loses the buckets it never entered.
 */
typedef struct {
    uint32_t packets[AK_DSHOT_EDT_BUCKETS];
    uint32_t invalid[AK_DSHOT_EDT_BUCKETS];
    uint32_t packet_sum;
    uint32_t invalid_sum;
    uint32_t bucket; /* the last bucket written, as now_ms / BUCKET_MS */
} ak_dshot_edt_quality_t;

/* One motor's telemetry. The arrays are the reference's `telemetryData` and
 * `telemetryTypes`; `value[i]` is meaningful only when bit i of `seen` is set,
 * which is the whole of how "a temperature of zero" is told from "no
 * temperature yet" (decision 6). */
typedef struct {
    ak_dshot_edt_quality_t quality;
    uint32_t value[AK_DSHOT_EDT_TYPE_COUNT]; /* decision 12: wider than the reference's */
    uint8_t seen;
    uint8_t max_temp;
    uint8_t last_type; /* the last ak_dshot_edt_type_t decoded, or 0xFF */
} ak_dshot_edt_motor_t;

/* The thing a flight core would own: one entry per motor, and whether extended
 * telemetry was asked for rather than having to wait to be told (the
 * reference's `edtAlwaysDecode`). */
typedef struct {
    ak_dshot_edt_motor_t motor[AK_MAX_MOTORS];
    unsigned motors;
    uint8_t edt_always;
} ak_dshot_edt_t;

/* The type field of a reply, as the reference's lookup reads it: index
 * `type >> 1`. */
ak_dshot_edt_type_t ak_dshot_edt_type_of(uint16_t value);

/* Decision 2: is this reply an eRPM rather than an extended type? */
int ak_dshot_edt_is_erpm(uint16_t value, int edt_enabled);

/* Decision 5: does this type have a unit this firmware can state? */
ak_dshot_edt_units_t ak_dshot_edt_units_of(ak_dshot_edt_type_t type);

/* The name of a type, for a readout. Never null: an out-of-range type is
 * reported as such rather than read past the table. */
const char *ak_dshot_edt_type_name(ak_dshot_edt_type_t type);

/* One reply, to a value. `edt_enabled` is decision 7's answer for this motor -
 * ak_dshot_edt_enabled(). The value is filled for every status this can return:
 * OK, NO_UNIT, STOPPED and BAD_ERPM, the last of which fills the type and the
 * payload and leaves `erpm` at zero rather than guessing a speed. */
ak_dshot_edt_status_t ak_dshot_edt_decode(uint16_t value, int edt_enabled,
                                          ak_dshot_edt_value_t *out);

void ak_dshot_edt_quality_init(ak_dshot_edt_quality_t *quality);

/* One reply's verdict, whatever it turned out to carry. The window's own
 * operation, public because the window is worth testing on its own; a reply
 * loop goes through ak_dshot_edt_update() instead, which calls this. */
void ak_dshot_edt_quality_update(ak_dshot_edt_quality_t *quality, int valid,
                                 uint32_t now_ms);

/* The window's error rate in parts per ten thousand of the replies, or -1 when
 * the window holds none (decision 9). Parts per ten thousand and not a
 * percentage because phase 3's acceptance is *under one percent*, and 1 % is
 * 100 here - the resolution is kept where the threshold is. */
int32_t ak_dshot_edt_quality_per_10k(const ak_dshot_edt_quality_t *quality);

void ak_dshot_edt_init(ak_dshot_edt_t *edt, unsigned motors, int edt_always);

/* Decision 7, for one motor. */
int ak_dshot_edt_enabled(const ak_dshot_edt_t *edt, unsigned motor);

/* One reply, all the way in: counted into the motor's window, classified, and
 * stored under its type. `valid` is the caller's own law's verdict for this
 * frame (decision 10) - a refused reply is counted and nothing else, and
 * `value` is not read in that case. This is the one call a reply loop makes. */
ak_dshot_edt_status_t ak_dshot_edt_update(ak_dshot_edt_t *edt, unsigned motor,
                                          uint16_t value, int valid,
                                          uint32_t now_ms);

#endif /* AK_FLIGHT_DSHOT_EDT_H */
