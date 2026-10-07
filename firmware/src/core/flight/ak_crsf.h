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
 * from. It is decoded below: see ak_crsf_link_stats_t.
 */

#define AK_CRSF_MAX_FRAME 64
#define AK_CRSF_CHANNELS  16
#define AK_CRSF_TYPE_RC_CHANNELS 0x16
#define AK_CRSF_TYPE_LINK_STATISTICS 0x14

/* What the receiver says about its own link, from frame type 0x14.
 *
 * Ten bytes, in this order, and the layout is copied rather than reconstructed
 * from the numbers: it is `crsfPayloadLinkstatistics_s` beside the frame-type
 * comment in Betaflight's `src/main/rx/crsf.c`, which is the implementation the
 * receivers on this link were built against. The payload length that goes with
 * it is `CRSF_FRAME_LINK_STATISTICS_PAYLOAD_SIZE` = 10, so the frame's `len`
 * field is 12 - type, ten bytes, crc - the same arithmetic as the 22-byte
 * channel frame above.
 *
 * **Two of these fields do not mean what their types suggest, which is why the
 * struct is quoted and not paraphrased.** Both RSSIs are dBm *negated*: the
 * wire's 70 is -70 dBm, and a *larger* number is a weaker signal - the opposite
 * of the direction the name reads in. Both SNRs are signed. Nothing here
 * converts either: the wire's units are what the receiver meant, and turning
 * them into something else in this file would be this file inventing a scale.
 * A reader that wants dBm negates; a reader that wants a percentage has to say
 * where its scale came from.
 *
 * The last three fields of the frame's own enumeration - `active_antenna`,
 * `rf_mode`, `uplink_tx_power` - are carried as raw numbers for the same
 * reason. Betaflight documents them in a comment block ("ant. 1 = 0, ant. 2",
 * "4fps = 0, 50fps, 150hz", "0mW = 0, 10mW, 25mW, ...") and does not name them
 * in code, so naming them here would be this file's vocabulary rather than the
 * wire's. */
typedef struct {
    uint8_t uplink_rssi_1;   /* dBm * -1, antenna 1 */
    uint8_t uplink_rssi_2;   /* dBm * -1, antenna 2 */
    uint8_t uplink_lq;       /* package success rate, per cent */
    int8_t  uplink_snr;      /* dB */
    uint8_t active_antenna;
    uint8_t rf_mode;
    uint8_t uplink_tx_power;
    uint8_t downlink_rssi;   /* dBm * -1 */
    uint8_t downlink_lq;     /* per cent */
    int8_t  downlink_snr;    /* dB */
} ak_crsf_link_stats_t;

#define AK_CRSF_LINK_STATS_BYTES 10u

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
    /* The last link-statistics frame (0x14), and the two numbers that date it.
     *
     * **`stats_frames == 0` is the line between "this receiver has not said"
     * and "this receiver said zero".** A zeroed struct with no count is a
     * receiver that never sent the frame, and it must not read as a link with
     * 0 % quality beside a 0 dBm signal - 0 % is what a dead link looks like,
     * and 0 dBm on a *negated* field is the strongest reading it can express,
     * so the two together are not a reading at all. A UI that cannot tell the
     * two cases apart would dress a receiver that is simply quiet as a dead
     * link. Every
     * reader tests the count first; the payload and the count are written in the
     * same branch so they cannot disagree.
     *
     * `last_stats_ms` is the arrival time of that frame, so a reader can ask
     * how stale it is - the same question ak_flight.c already asks of
     * `last_update_ms` for the channels themselves. Nothing here decides what
     * staleness means. */
    ak_crsf_link_stats_t stats;
    uint32_t stats_frames;
    uint32_t last_stats_ms;
    uint16_t channel[AK_CRSF_CHANNELS];
} ak_crsf_t;

void ak_crsf_init(ak_crsf_t *crsf);

uint8_t ak_crsf_crc8(const uint8_t *data, uint8_t len);

/* Feed one byte at a time, as they come off the UART. Returns 1 when that byte
 * completed an RC channel frame, in which case `out` holds those channels and
 * has been stamped with now_ms. Byte-at-a-time on purpose: it is the shape a
 * UART interrupt gives you, and it makes a truncated frame a nothing rather
 * than a wrong answer.
 *
 * A link-statistics frame (0x14) returns 0 and is not an RC update - it updates
 * `crsf->stats` in place and is read from there. A 0x14 whose length is not
 * `2 + AK_CRSF_LINK_STATS_BYTES` is not decoded at all and is not counted
 * anywhere; it is one of the frames "we understand the envelope of but not the
 * body". Should a receiver be found in the field sending one, that is where the
 * counter would go - deliberate as it stands, because this file does not invent
 * a counter nobody reads. */
int ak_crsf_feed(ak_crsf_t *crsf, uint8_t byte, ak_rc_input_t *out,
                 uint32_t now_ms);

/* The 11-bit channels, unpacked, for callers that want more than the eight the
 * flight core uses. */
void ak_crsf_unpack(const uint8_t *payload, uint16_t channels[AK_CRSF_CHANNELS]);

#endif /* AK_FLIGHT_CRSF_H */
