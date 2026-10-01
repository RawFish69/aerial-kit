#ifndef AK_FLIGHT_CRSF_H
#define AK_FLIGHT_CRSF_H

#include "ak_types.h"

/*
 * CRSF receive: the protocol the twin-wings receiver already speaks, and the
 * one an ELRS link speaks by default.
 *
 * Wire format, byte by byte:
 *
 *   [address][len][type][payload ...][crc8]
 *
 *   len counts the type, the payload and the crc - everything after itself -
 *   so a frame is len + 2 bytes on the wire. The crc is CRC-8/DVB-S2
 *   (polynomial 0xD5, init 0, no reflection) over the type and payload.
 *
 * RC channels arrive as type 0x16: sixteen 11-bit channels packed
 * little-endian, so channel n starts at bit 11*n. Counts are 172..1811 with 992
 * in the middle, which is what ak_rc_default_config() already expects.
 *
 * No failsafe flag rides in an RC frame. A silent link is detected by time -
 * ak_flight.c already treats a stale last_update_ms as a failsafe - and the
 * link statistics frame (0x14) is where a signal-loss *warning* would come
 * from, when something has a use for it.
 */

#define AK_CRSF_MAX_FRAME 64
#define AK_CRSF_CHANNELS  16
#define AK_CRSF_TYPE_RC_CHANNELS 0x16

/* A frame that stops mid-way is abandoned after this much silence. Without it,
 * a receiver that reboots in the middle of a frame leaves the parser waiting
 * for bytes that will never come, and the link stays dead until the aircraft
 * is power cycled - which is exactly the failure a parser has to survive. */
#define AK_CRSF_GAP_MS 5

typedef struct {
    uint8_t  frame[AK_CRSF_MAX_FRAME];
    uint8_t  held;      /* bytes collected for the frame in progress */
    uint8_t  expected;  /* total bytes this frame will be, 0 until known */
    uint32_t frames;    /* frames that passed the crc */
    uint32_t crc_errors;
    uint32_t rejected;  /* bad address, bad length */
    /* Device pings (type 0x28) addressed to this flight controller: what a
     * handset sends when it wants to know who is on the link. Counted here
     * rather than answered here, because the answer is telemetry and this file
     * is only the receiver. */
    uint32_t pings;
    uint8_t  last_ping_from;
    uint32_t last_byte_ms;
    uint16_t channel[AK_CRSF_CHANNELS];
} ak_crsf_t;

void ak_crsf_init(ak_crsf_t *crsf);

uint8_t ak_crsf_crc8(const uint8_t *data, uint8_t len);

/* Feed one byte at a time, as they come off the UART. Returns 1 when that byte
 * completed an RC channel frame, in which case `out` holds those channels and
 * has been stamped with now_ms. Byte-at-a-time on purpose: it is the shape a
 * UART interrupt gives you, and it makes a truncated frame a nothing rather
 * than a wrong answer. */
int ak_crsf_feed(ak_crsf_t *crsf, uint8_t byte, ak_rc_input_t *out,
                 uint32_t now_ms);

/* The 11-bit channels, unpacked, for callers that want more than the eight the
 * flight core uses. */
void ak_crsf_unpack(const uint8_t *payload, uint16_t channels[AK_CRSF_CHANNELS]);

#endif /* AK_FLIGHT_CRSF_H */
