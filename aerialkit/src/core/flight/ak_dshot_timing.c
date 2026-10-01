#include "ak_dshot_timing.h"

uint16_t ak_dshot_ccr_zero(uint16_t period)
{
    return (uint16_t)(((uint32_t)period * AK_DSHOT_DUTY_ZERO_NUM) / 100u);
}

uint16_t ak_dshot_ccr_one(uint16_t period)
{
    return (uint16_t)(((uint32_t)period * AK_DSHOT_DUTY_ONE_NUM) / 100u);
}

void ak_dshot_fill(uint16_t *entries, const uint16_t frames[AK_MAX_MOTORS],
                   uint16_t ccr_zero, uint16_t ccr_one)
{
    for (unsigned bit = 0; bit < AK_DSHOT_BITS; bit++) {
        /* MSB first: bit 15 of the frame goes out first. */
        uint16_t mask = (uint16_t)(0x8000u >> bit);
        for (unsigned motor = 0; motor < AK_MAX_MOTORS; motor++) {
            uint16_t value = (frames[motor] & mask) != 0 ? ccr_one : ccr_zero;
            entries[bit * AK_MAX_MOTORS + motor] = value;
        }
    }

    for (unsigned group = AK_DSHOT_BITS; group < AK_DSHOT_GROUPS; group++) {
        for (unsigned motor = 0; motor < AK_MAX_MOTORS; motor++) {
            entries[group * AK_MAX_MOTORS + motor] = 0;
        }
    }
}

uint16_t ak_dshot_ticks_per_bit(uint32_t khz, uint32_t tick_ns)
{
    uint32_t bit_ns;

    switch (khz) {
    case 150u:
        bit_ns = 6667u;
        break;
    case 300u:
        bit_ns = 3333u;
        break;
    case 600u:
        bit_ns = 1667u;
        break;
    default:
        return 0u; /* a rate DShot does not have */
    }
    if (tick_ns == 0u) {
        return 0u;
    }
    return (uint16_t)((bit_ns + tick_ns / 2u) / tick_ns);
}

void ak_dshot_edges(uint16_t frame, uint16_t ticks_per_bit,
                    ak_dshot_edge_t *out)
{
    uint16_t high_zero = (uint16_t)(((uint32_t)ticks_per_bit *
                                     AK_DSHOT_DUTY_ZERO_NUM) / 100u);
    uint16_t high_one = (uint16_t)(((uint32_t)ticks_per_bit *
                                    AK_DSHOT_DUTY_ONE_NUM) / 100u);
    unsigned at = 0u;

    if (ticks_per_bit == 0u) {
        /* Nothing to send: a list of zero-length lows, which the caller can
         * hand to a transmitter without a special case for "no frame". */
        for (unsigned i = 0u; i < AK_DSHOT_EDGES; i++) {
            out[i].level = 0u;
            out[i].ticks = 0u;
        }
        return;
    }

    for (unsigned bit = 0u; bit < AK_DSHOT_BITS; bit++) {
        uint16_t high = (frame & (uint16_t)(0x8000u >> bit)) != 0u ? high_one
                                                                   : high_zero;
        out[at].level = 1u;
        out[at].ticks = high;
        at++;
        out[at].level = 0u;
        out[at].ticks = (uint16_t)(ticks_per_bit - high);
        at++;
    }

    /* The gap between frames, in the same units, clamped rather than wrapped:
     * a bit so long that twice it does not fit in sixteen bits is a transmitter
     * that has been asked for something absurd. */
    uint32_t gap = (uint32_t)ticks_per_bit * AK_DSHOT_GAP_GROUPS;
    out[at].level = 0u;
    out[at].ticks = gap > 0xFFFFu ? 0xFFFFu : (uint16_t)gap;
}
