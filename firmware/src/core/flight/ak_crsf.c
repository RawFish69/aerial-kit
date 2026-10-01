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

        /* 11 bits little-endian, spanning at most three bytes of the payload. */
        uint32_t window = (uint32_t)payload[byte] |
                          ((uint32_t)payload[byte + 1] << 8) |
                          ((uint32_t)payload[byte + 2] << 16);
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
