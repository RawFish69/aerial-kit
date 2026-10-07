#include "ak_crsf.h"

/* Addresses seen on this link: the flight controller, the handset, and the
 * transmitter module. The crc is what actually protects the frame, so this is a
 * sanity filter rather than a security check. */
static int address_ok(uint8_t address)
{
    return address == 0xC0 || address == 0xC8 || address == 0xEA ||
           address == 0xEE;
}

void ak_crsf_init(ak_crsf_t *crsf)
{
    crsf->held = 0;
    crsf->expected = 0;
    crsf->frames = 0;
    crsf->crc_errors = 0;
    crsf->rejected = 0;
    crsf->pings = 0;
    crsf->last_ping_from = 0;
    crsf->last_byte_ms = 0;
    /* The stats are zeroed *and* their frame count is zeroed. The count is what
     * says whether the zeros mean anything: see the field comment in the
     * header. */
    crsf->stats_frames = 0;
    crsf->last_stats_ms = 0;
    crsf->stats = (ak_crsf_link_stats_t){0};
    for (int i = 0; i < AK_CRSF_CHANNELS; i++) {
        crsf->channel[i] = 0;
    }
}

uint8_t ak_crsf_crc8(const uint8_t *data, uint8_t len)
{
    uint8_t crc = 0;
    for (uint8_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int bit = 0; bit < 8; bit++) {
            crc = (crc & 0x80u) != 0 ? (uint8_t)((crc << 1) ^ 0xD5u)
                                     : (uint8_t)(crc << 1);
        }
    }
    return crc;
}

void ak_crsf_unpack(const uint8_t *payload, uint16_t channels[AK_CRSF_CHANNELS])
{
    for (int n = 0; n < AK_CRSF_CHANNELS; n++) {
        unsigned bit = (unsigned)n * 11u;
        unsigned byte = bit / 8u;
        unsigned shift = bit % 8u;

        /* 11 bits little-endian. The window is three bytes wide only when the
         * field actually reaches into the third: at shift 6 or 7 it ends on bit
         * 16 or 17, and at shift 5 it ends exactly on bit 15. Reading the third
         * byte unconditionally reads one past the end of the 22-byte channel
         * payload CRSF defines - the read tools/fuzz_parsers.c means when it
         * says the unpacker must not read outside the payload it is given, and
         * what the sanitize stage caught at tests/test_custom_radio.c:181.
         *
         * It never produced a wrong channel, and neither of the two reasons is
         * a reason to keep it: the bits above 15 are masked off by the `& 0x7FF`
         * below, and in the frame buffer the byte past the payload is the crc,
         * which happens to be mapped - so the only caller that passes exactly
         * 22 bytes (the test) is the one that reads out of bounds. */
        uint32_t window = (uint32_t)payload[byte] |
                          ((uint32_t)payload[byte + 1] << 8);
        if (shift > 5u) {
            window |= (uint32_t)payload[byte + 2] << 16;
        }
        channels[n] = (uint16_t)((window >> shift) & 0x7FFu);
    }
}

int ak_crsf_feed(ak_crsf_t *crsf, uint8_t byte, ak_rc_input_t *out,
                 uint32_t now_ms)
{
    if (crsf->held > 0 &&
        (uint32_t)(now_ms - crsf->last_byte_ms) > AK_CRSF_GAP_MS) {
        crsf->rejected++;
        crsf->held = 0;
        crsf->expected = 0;
    }
    crsf->last_byte_ms = now_ms;

    if (crsf->held == 0) {
        if (!address_ok(byte)) {
            crsf->rejected++;
            return 0;
        }
        crsf->frame[0] = byte;
        crsf->held = 1;
        crsf->expected = 0;
        return 0;
    }

    crsf->frame[crsf->held++] = byte;

    if (crsf->held == 2) {
        /* len counts type + payload + crc, so the frame is len + 2 bytes and
         * the smallest sensible frame carries a type and a crc. */
        uint8_t len = crsf->frame[1];
        if (len < 2 || len > AK_CRSF_MAX_FRAME - 2) {
            crsf->rejected++;
            crsf->held = 0;
            return 0;
        }
        crsf->expected = (uint8_t)(len + 2u);
        return 0;
    }

    if (crsf->expected == 0 || crsf->held < crsf->expected) {
        return 0;
    }

    uint8_t len = crsf->frame[1];
    uint8_t type = crsf->frame[2];
    uint8_t crc = crsf->frame[crsf->expected - 1];
    uint8_t want = ak_crsf_crc8(&crsf->frame[2], (uint8_t)(len - 1u));

    crsf->held = 0;
    crsf->expected = 0;

    if (crc != want) {
        crsf->crc_errors++;
        return 0;
    }
    crsf->frames++;

    if (type == 0x28u && len >= 4u) {
        /* A device ping: [dest][origin][...]. Answered only when it is for this
         * flight controller - a ping the receiver sends to somebody else is
         * not ours to reply to. */
        uint8_t dest = crsf->frame[3];

        if (dest == 0xC8u || dest == 0x00u) {
            crsf->pings++;
            crsf->last_ping_from = crsf->frame[4];
        }
        return 0;
    }

    if (type == AK_CRSF_TYPE_LINK_STATISTICS &&
        len == 2u + AK_CRSF_LINK_STATS_BYTES) {
        /* What the receiver says about the link it is holding. Copied field by
         * field rather than as a struct, so the byte order is visible here and
         * a change to the struct's layout cannot silently re-map the wire.
         *
         * Returns 0 on purpose. This is not a channel update and must not be
         * mistaken for one: `out` is untouched and `last_update_ms` is not
         * stamped, so a receiver that keeps talking about a link it can no
         * longer carry sticks on never looks like a live stick frame. The
         * numbers are left on the parser for whoever asks. */
        const uint8_t *p = &crsf->frame[3];

        crsf->stats.uplink_rssi_1   = p[0];
        crsf->stats.uplink_rssi_2   = p[1];
        crsf->stats.uplink_lq       = p[2];
        crsf->stats.uplink_snr      = (int8_t)p[3];
        crsf->stats.active_antenna  = p[4];
        crsf->stats.rf_mode         = p[5];
        crsf->stats.uplink_tx_power = p[6];
        crsf->stats.downlink_rssi   = p[7];
        crsf->stats.downlink_lq     = p[8];
        crsf->stats.downlink_snr    = (int8_t)p[9];

        crsf->stats_frames++;
        crsf->last_stats_ms = now_ms;
        return 0;
    }

    if (type != AK_CRSF_TYPE_RC_CHANNELS || len != 2 + 22) {
        return 0; /* a frame we understand the envelope of but not the body */
    }

    ak_crsf_unpack(&crsf->frame[3], crsf->channel);

    if (out != 0) {
        for (int i = 0; i < AK_RC_CHANNELS && i < AK_CRSF_CHANNELS; i++) {
            out->channel[i] = crsf->channel[i];
        }
        out->last_update_ms = now_ms;
        out->valid = 1;
    }
    return 1;
}
