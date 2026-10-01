#ifndef AK_FLIGHT_AK_SBUS_H
#define AK_FLIGHT_AK_SBUS_H

#include "ak_types.h"

/*
 * SBUS receive: the protocol most receivers a person buys are already speaking.
 *
 * Wire format, byte by byte:
 *
 *   [0x0F][22 bytes of channels][flags][0x00]     25 bytes, every 7 or 14 ms
 *
 * Sixteen channels are packed into those 22 bytes, eleven bits each and
 * little-endian, so channel n starts at bit 11*n. The counts are 172..1811
 * with 992 in the middle - the same scale CRSF uses, which is why nothing
 * downstream of this file has to care which protocol the receiver speaks.
 *
 * The frame has no checksum, which changes what a parser can promise. CRSF
 * carries a crc8 so a bad frame is *known* to be bad and dropped; SBUS has a
 * fixed first byte and a fixed last byte, and everything in between is a
 * number. So a frame that is corrupt in the middle is credible - the two ends
 * are the only thing that can be checked, and the inter-byte gap is the rest
 * of it. That is the protocol's weakness rather than this file's, and it is
 * why the receiver's own failsafe flags are worth reading rather than
 * ignoring.
 *
 * The flags byte is the part that matters:
 *
 *   bit 2   the receiver has lost frames
 *   bit 3   the receiver's own failsafe is active - it has lost its
 *           transmitter, and the channels it is sending are not a pilot
 *
 * A frame with bit 3 set is *not* a new command, and this parser does not
 * pretend otherwise: it reports the frame, does not return it as live
 * channels, and leaves the caller's timestamp alone. The flight core detects a
 * dead link by time, so a receiver shouting "I have lost the transmitter"
 * while still sending frames must not keep the link looking alive - that is
 * the one way an SBUS link fails that a timeout alone cannot see.
 */

#define AK_SBUS_FRAME    25u
#define AK_SBUS_CHANNELS 16u

#define AK_SBUS_HEADER 0x0Fu
#define AK_SBUS_FOOTER 0x00u

#define AK_SBUS_FLAG_SIGNAL_LOSS 0x04u
#define AK_SBUS_FLAG_FAILSAFE    0x08u

/* A frame that stops mid-way is abandoned after this much silence, the same
 * way and for the same reason CRSF's is: a receiver rebooting in the middle of
 * a frame must not leave the parser waiting for bytes that are never coming. */
#define AK_SBUS_GAP_MS 5

typedef struct {
    uint8_t  frame[AK_SBUS_FRAME];
    uint8_t  held;    /* bytes collected for the frame in progress */
    uint32_t frames;  /* frames with a header, a footer and live channels */
    uint32_t failsafe_frames;
    uint32_t lost_frames;
    uint32_t rejected; /* bad header, bad footer, or a gap mid-frame */
    uint32_t last_byte_ms;
    uint16_t channel[AK_SBUS_CHANNELS];
    int      failsafe; /* as of the last frame the receiver sent */
} ak_sbus_t;

void ak_sbus_init(ak_sbus_t *sbus);

/* The sixteen channels, unpacked. Exposed because it is the piece of the
 * protocol most likely to be got wrong and the cheapest to test on its own. */
void ak_sbus_unpack(const uint8_t *frame, uint16_t channels[AK_SBUS_CHANNELS]);

/* Feed one byte at a time, as they come off the UART. Returns 1 when that byte
 * completed a frame carrying live channels, in which case `out` holds them and
 * has been stamped with now_ms. A failsafe frame returns 0: the channels are
 * not a pilot's. */
int ak_sbus_feed(ak_sbus_t *sbus, uint8_t byte, ak_rc_input_t *out,
                 uint32_t now_ms);

#endif /* AK_FLIGHT_AK_SBUS_H */
