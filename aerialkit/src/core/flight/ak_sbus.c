#include "ak_sbus.h"

/*
 * The channel unpacking is the whole of the arithmetic, and it is worth
 * stating once: channel n starts at bit 11*n of the twenty-two bytes after the
 * header, little-endian. Reading a channel therefore means taking the three
 * bytes that bracket those bits, shifting the pair into place, and masking.
 *
 * The third byte of that window is past the end of the channel data for the
 * last channel or two, which is why the frame is passed whole and not the
 * twenty-two bytes: the bits the mask keeps are the last of the real ones, and
 * the extra byte is shifted out above them. Reading one byte too many is
 * cheaper than a special case that is only wrong for channel sixteen.
 */
void ak_sbus_unpack(const uint8_t *frame, uint16_t channels[AK_SBUS_CHANNELS])
{
    const uint8_t *data = frame + 1u;

    for (unsigned n = 0; n < AK_SBUS_CHANNELS; n++) {
        unsigned bit = 11u * n;
        unsigned index = bit / 8u;
        unsigned shift = bit % 8u;
        uint32_t window = (uint32_t)data[index] |
                          ((uint32_t)data[index + 1u] << 8) |
                          ((uint32_t)data[index + 2u] << 16);

        channels[n] = (uint16_t)((window >> shift) & 0x7FFu);
    }
}

void ak_sbus_init(ak_sbus_t *sbus)
{
    for (unsigned i = 0; i < AK_SBUS_FRAME; i++) {
        sbus->frame[i] = 0u;
    }
    sbus->held = 0u;
    sbus->frames = 0u;
    sbus->failsafe_frames = 0u;
    sbus->lost_frames = 0u;
    sbus->rejected = 0u;
    sbus->last_byte_ms = 0u;
    sbus->failsafe = 0;
    for (unsigned i = 0; i < AK_SBUS_CHANNELS; i++) {
        sbus->channel[i] = 0u;
    }
}

int ak_sbus_feed(ak_sbus_t *sbus, uint8_t byte, ak_rc_input_t *out,
                 uint32_t now_ms)
{
    /* A frame that stopped half way. The gap is measured between bytes, so a
     * stream that pauses for longer than one frame period starts again from
     * whatever comes next rather than completing a frame out of two. */
    if (sbus->held > 0u && sbus->held < AK_SBUS_FRAME &&
        (uint32_t)(now_ms - sbus->last_byte_ms) > AK_SBUS_GAP_MS) {
        sbus->held = 0u;
        sbus->rejected++;
    }
    sbus->last_byte_ms = now_ms;

    if (sbus->held == 0u) {
        if (byte != AK_SBUS_HEADER) {
            /* Bytes between frames, or a stream that is not SBUS at all. Count
             * it so `rc` can tell a wrong baud rate from a dead receiver. */
            sbus->rejected++;
            return 0;
        }
    }

    sbus->frame[sbus->held++] = byte;
    if (sbus->held < AK_SBUS_FRAME) {
        return 0;
    }
    sbus->held = 0u;

    if (sbus->frame[AK_SBUS_FRAME - 1u] != AK_SBUS_FOOTER) {
        sbus->rejected++;
        return 0;
    }

    uint8_t flags = sbus->frame[AK_SBUS_FRAME - 2u];
    ak_sbus_unpack(sbus->frame, sbus->channel);

    if (flags & AK_SBUS_FLAG_SIGNAL_LOSS) {
        /* The receiver dropped frames. The channels in this one are still the
         * pilot's, so it counts and it is still a command - but the count is
         * worth having, because a link that is losing frames is a link that is
         * about to stop. */
        sbus->lost_frames++;
    }

    if (flags & AK_SBUS_FLAG_FAILSAFE) {
        /*
         * The receiver has lost its transmitter. These channels are whatever
         * the failsafe was configured to be - often throttle down and neutral,
         * sometimes the last position, and there is no telling which.
         *
         * So: remembered for the console, not returned as a command, and the
         * caller's `last_update_ms` is left alone. The flight core's timeout
         * then does what it does for a receiver that went quiet, which is the
         * behaviour a pilot has already tested.
         */
        sbus->failsafe_frames++;
        sbus->failsafe = 1;
        return 0;
    }

    sbus->failsafe = 0;
    sbus->frames++;
    for (unsigned i = 0; i < AK_RC_CHANNELS && i < AK_SBUS_CHANNELS; i++) {
        out->channel[i] = sbus->channel[i];
    }
    out->last_update_ms = now_ms;
    out->valid = 1;
    return 1;
}
