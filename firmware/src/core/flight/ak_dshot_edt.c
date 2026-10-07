#include "ak_dshot_edt.h"

/*
 * The one table in this file that the reference also has, in its order. Indexed
 * by `type >> 1`, which is decision 4: the marker is the field's low bit, so it
 * is the last one shifted out and the index is the field halved.
 */
static const char *const type_names[AK_DSHOT_EDT_TYPE_COUNT] = {
    "erpm",
    "temperature",
    "voltage",
    "current",
    "debug1",
    "debug2",
    "debug3",
    "state/events",
};

static uint16_t type_field(uint16_t value)
{
    return (uint16_t)((value & AK_DSHOT_EDT_TYPE_MASK) >> AK_DSHOT_EDT_TYPE_SHIFT);
}

ak_dshot_edt_type_t ak_dshot_edt_type_of(uint16_t value)
{
    /* The field is four bits, so this cannot reach 8 and the guard is a
     * statement about the mask rather than about a caller: `type >> 1` over
     * 0..15 is 0..7 and every one of those is in the table. It is here so that
     * widening AK_DSHOT_EDT_TYPE_MASK without widening the table refuses to
     * read past the end instead of doing it. */
    uint16_t index = (uint16_t)(type_field(value) >> 1);

    if (index >= AK_DSHOT_EDT_TYPE_COUNT) {
        return AK_DSHOT_EDT_ERPM;
    }
    return (ak_dshot_edt_type_t)index;
}

int ak_dshot_edt_is_erpm(uint16_t value, int edt_enabled)
{
    uint16_t field = type_field(value);

    /* Decision 7: until something has been asked for, there is no question to
     * ask - every reply is a period whatever its bits say. This is the branch
     * that makes the same twelve bits mean two different things. */
    if (!edt_enabled) {
        return 1;
    }

    /* Decision 2. `field & 1` is the marker (AK_DSHOT_EDT_MARKER, bit 8), and
     * the zero test is the reference's own second clause - a field of zero has
     * the marker clear and is an eRPM anyway. */
    return (field & 1u) != 0u || field == 0u;
}

ak_dshot_edt_units_t ak_dshot_edt_units_of(ak_dshot_edt_type_t type)
{
    switch (type) {
    case AK_DSHOT_EDT_ERPM:
    case AK_DSHOT_EDT_TEMPERATURE:
    case AK_DSHOT_EDT_VOLTAGE:
    case AK_DSHOT_EDT_CURRENT:
        return AK_DSHOT_EDT_KNOWN_UNIT;
    default:
        /* Decision 5: debug1..3 and state/events carry a byte whose meaning is
         * the ESC's documentation's, not this firmware's. Returning the byte
         * and saying so is the honest answer; inventing a unit is not. */
        return AK_DSHOT_EDT_ESC_DEFINED;
    }
}

const char *ak_dshot_edt_type_name(ak_dshot_edt_type_t type)
{
    unsigned index = (unsigned)type;

    if (index >= AK_DSHOT_EDT_TYPE_COUNT) {
        return "unknown";
    }
    return type_names[index];
}

ak_dshot_edt_status_t ak_dshot_edt_decode(uint16_t value, int edt_enabled,
                                          ak_dshot_edt_value_t *out)
{
    out->payload = (uint8_t)(value & AK_DSHOT_EDT_PAYLOAD_MASK);
    out->marker = (value & AK_DSHOT_EDT_MARKER) != 0u ? 1u : 0u;
    out->erpm = 0u;
    out->celsius = 0u;
    out->millivolts = 0u;
    out->milliamps = 0u;

    if (ak_dshot_edt_is_erpm(value, edt_enabled)) {
        uint32_t erpm = 0u;
        ak_dshot_erpm_t verdict;

        out->type = AK_DSHOT_EDT_ERPM;
        out->units = AK_DSHOT_EDT_KNOWN_UNIT;

        /* Decision 3: this branch reads the field's low *nine* bits, through
         * ak_dshot_gcr.h's period law - not the eight this file masks for an
         * extended type. The marker bit is inside the period, and that is not
         * an accident of the reference: it is the same twelve bits, read two
         * ways, and the branch is chosen by whether a period is what arrived.
         *
         * ak_dshot_erpm_from_value() answers for 0x0fff itself (it is
         * ak_dshot_gcr.h's STOPPED) so the marker is not repeated here. */
        verdict = ak_dshot_erpm_from_value(value, &erpm);
        if (verdict == AK_DSHOT_ERPM_STOPPED) {
            return AK_DSHOT_EDT_STOPPED;
        }
        if (verdict != AK_DSHOT_ERPM_TURNING) {
            return AK_DSHOT_EDT_BAD_ERPM;
        }
        out->erpm = erpm;
        return AK_DSHOT_EDT_OK;
    }

    out->type = ak_dshot_edt_type_of(value);
    out->units = ak_dshot_edt_units_of(out->type);

    switch (out->type) {
    case AK_DSHOT_EDT_TEMPERATURE:
        /* Whole degrees Celsius: the byte is the temperature, which is why
         * there is no scale here to get wrong. Zero means none has arrived
         * (decision 6) and the motor's `seen` bit is what tells the two
         * apart. */
        out->celsius = out->payload;
        break;
    case AK_DSHOT_EDT_VOLTAGE:
        out->millivolts = (uint32_t)out->payload * AK_DSHOT_EDT_MV_PER_COUNT;
        break;
    case AK_DSHOT_EDT_CURRENT:
        out->milliamps = (uint32_t)out->payload * AK_DSHOT_EDT_MA_PER_COUNT;
        break;
    default:
        /* AK_DSHOT_EDT_ERPM cannot arrive here - the index is `field >> 1` and
         * the branch above has taken every odd and zero field, which leaves
         * 1..7. It is left to the default rather than given a case, so that a
         * field of 1 reaching this arm would be visible as "no unit" instead
         * of as a speed of zero. */
        break;
    }

    return out->units == AK_DSHOT_EDT_KNOWN_UNIT ? AK_DSHOT_EDT_OK
                                                : AK_DSHOT_EDT_NO_UNIT;
}

void ak_dshot_edt_quality_init(ak_dshot_edt_quality_t *quality)
{
    unsigned i;

    for (i = 0u; i < AK_DSHOT_EDT_BUCKETS; i++) {
        quality->packets[i] = 0u;
        quality->invalid[i] = 0u;
    }
    quality->packet_sum = 0u;
    quality->invalid_sum = 0u;
    quality->bucket = 0u;
}

/*
 * Drop every bucket the window has left behind.
 *
 * This is decision 8, and it is the one place this file deliberately does not
 * do what the reference does. The reference clears the single bucket it is
 * about to enter and moves on, so if a second goes by without a call - a stalled
 * loop, a bus that stopped answering, this task preempted across a whole window
 * - the counts from the buckets in between stay in the sums for ever. The
 * window then reports replies it claims to have dropped, and phase 3's
 * acceptance is a rate over five minutes, which is long enough for that to
 * happen and quiet about it when it does.
 *
 * `bucket` is kept as the whole quotient `now_ms / AK_DSHOT_EDT_BUCKET_MS` and
 * not as an index, so "how many went by" is one unsigned subtraction rather
 * than a subtraction modulo ten - which is the same bug in a different shape,
 * because a modulo difference cannot tell ten buckets from none.
 */
static void quality_rotate(ak_dshot_edt_quality_t *quality, uint32_t now_ms)
{
    uint32_t bucket = now_ms / AK_DSHOT_EDT_BUCKET_MS;
    uint32_t steps = bucket - quality->bucket;
    unsigned i;

    if (steps == 0u) {
        return;
    }

    if (steps >= AK_DSHOT_EDT_BUCKETS) {
        /* A whole window or more: every bucket is behind now, and clearing all
         * ten is not the same as clearing `steps` of them - which is why this
         * is a test and not the loop below with a larger bound. */
        for (i = 0u; i < AK_DSHOT_EDT_BUCKETS; i++) {
            quality->packet_sum -= quality->packets[i];
            quality->invalid_sum -= quality->invalid[i];
            quality->packets[i] = 0u;
            quality->invalid[i] = 0u;
        }
    } else {
        uint32_t step;

        for (step = 1u; step <= steps; step++) {
            unsigned b = (unsigned)((quality->bucket + step) % AK_DSHOT_EDT_BUCKETS);

            quality->packet_sum -= quality->packets[b];
            quality->invalid_sum -= quality->invalid[b];
            quality->packets[b] = 0u;
            quality->invalid[b] = 0u;
        }
    }

    quality->bucket = bucket;
}

void ak_dshot_edt_quality_update(ak_dshot_edt_quality_t *quality, int valid,
                                 uint32_t now_ms)
{
    unsigned b;

    quality_rotate(quality, now_ms);
    b = (unsigned)(quality->bucket % AK_DSHOT_EDT_BUCKETS);

    quality->packet_sum++;
    quality->packets[b]++;

    if (!valid) {
        quality->invalid_sum++;
        quality->invalid[b]++;
    }
}

int32_t ak_dshot_edt_quality_per_10k(const ak_dshot_edt_quality_t *quality)
{
    if (quality->packet_sum == 0u) {
        /* Decision 9. Not zero: "no reply arrived" and "every reply was good"
         * are different statements about the aircraft, and a caller that has
         * only the number cannot tell them apart afterwards. */
        return -1;
    }

    /* Thirty-two-bit arithmetic on purpose. The target has no 64-bit divide and
     * would link one in for this; the product stays inside 32 bits because the
     * denominator is one second of replies at this firmware's own maximum rate
     * (8 kHz on four motors is 32 000, so the product is about 3.2e8). */
    return (int32_t)(quality->invalid_sum * 10000u / quality->packet_sum);
}

void ak_dshot_edt_init(ak_dshot_edt_t *edt, unsigned motors, int edt_always)
{
    unsigned m;
    unsigned i;

    if (motors > AK_MAX_MOTORS) {
        motors = AK_MAX_MOTORS;
    }
    edt->motors = motors;
    edt->edt_always = edt_always ? 1u : 0u;

    /* Every motor is cleared, not just the first `motors` of them: a caller
     * that later grows its motor count must not find the new motor holding a
     * previous aircraft's telemetry. */
    for (m = 0u; m < AK_MAX_MOTORS; m++) {
        ak_dshot_edt_motor_t *motor = &edt->motor[m];

        ak_dshot_edt_quality_init(&motor->quality);
        for (i = 0u; i < AK_DSHOT_EDT_TYPE_COUNT; i++) {
            motor->value[i] = 0u;
        }
        motor->seen = 0u;
        motor->max_temp = 0u;
        motor->last_type = 0xFFu;
    }
}

int ak_dshot_edt_enabled(const ak_dshot_edt_t *edt, unsigned motor)
{
    if (motor >= edt->motors) {
        return 0;
    }
    if (edt->edt_always != 0u) {
        return 1;
    }
    /* Decision 7: the motor's own history. `& ~1` is eRPM's bit out of the
     * mask, so a motor that has only ever sent periods is still reading them
     * as periods. */
    return (edt->motor[motor].seen & AK_DSHOT_EDT_EXTENDED_MASK) != 0u;
}

ak_dshot_edt_status_t ak_dshot_edt_update(ak_dshot_edt_t *edt, unsigned motor,
                                          uint16_t value, int valid,
                                          uint32_t now_ms)
{
    ak_dshot_edt_motor_t *state;
    ak_dshot_edt_value_t decoded;
    ak_dshot_edt_status_t status;

    if (motor >= edt->motors) {
        /* Not a reply to this aircraft at all, so it is not counted: the error
         * rate is about a wire, and a motor that is not on it has no wire. */
        return AK_DSHOT_EDT_BAD_MOTOR;
    }

    state = &edt->motor[motor];

    /* Decision 10: the caller's law decided this, and every reply is counted -
     * the good ones below and the refused ones here. Counting only the good
     * ones would understate the rate, which is the direction that hides a bad
     * wire. */
    ak_dshot_edt_quality_update(&state->quality, valid, now_ms);

    if (!valid) {
        return AK_DSHOT_EDT_BAD_FRAME;
    }

    /* Decision 7 is asked here and not before the counter: what a reply *is*
     * depends on what this motor sent before it, and the reply being decoded
     * is not part of its own history. */
    status = ak_dshot_edt_decode(value, ak_dshot_edt_enabled(edt, motor),
                                 &decoded);

    if (status == AK_DSHOT_EDT_BAD_ERPM) {
        /* Decision 11: the reference stores nothing here. The field carried no
         * period, so there is no speed to remember and no type to claim - the
         * motor's `seen` bit is left as it was. */
        return status;
    }

    state->last_type = (uint8_t)decoded.type;
    state->seen |= (uint8_t)(1u << decoded.type);

    if (decoded.type == AK_DSHOT_EDT_ERPM) {
        /* Decision 11's other half: not turning is stored as the zero the
         * reference's decoder returned for it, and it does set the bit.
         * Decision 12: this is the store the reference truncates - the answer
         * runs to 30 000 000 and its own array is sixteen bits wide. */
        state->value[AK_DSHOT_EDT_ERPM] = decoded.erpm;
    } else {
        state->value[decoded.type] = decoded.payload;

        if (decoded.type == AK_DSHOT_EDT_TEMPERATURE &&
            decoded.payload > state->max_temp) {
            /* Decision 6: a session maximum of the raw byte, as the reference
             * keeps it - including its ambiguity about zero. */
            state->max_temp = decoded.payload;
        }
    }

    return status;
}
