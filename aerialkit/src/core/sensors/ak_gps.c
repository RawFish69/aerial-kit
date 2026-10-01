#include "ak_gps.h"

/*
 * UBX framing, byte at a time:
 *
 *   B5 62  class  id  length(2, little endian)  payload  checksum(2)
 *
 * The checksum is the 8-bit Fletcher-style pair: two running sums over
 * everything from the class byte to the end of the payload. A frame that does
 * not add up is counted and dropped, and the parser looks for the next sync
 * pair rather than trusting what it was in the middle of.
 *
 * NAV-PVT (class 1, id 7) is 92 bytes and carries the lot. The offsets live in
 * ak_gps.h, where a test pins each of them against INAV's own ubx_nav_pvt
 * struct at the pinned revision (see docs/03-attribution.md); the *framing* is
 * checked against a real captured configuration frame that INAV's own unit test
 * carries. That split matters: the first version of this file claimed the
 * cross-check and had every offset two bytes too high, which nothing caught
 * because the unit frame and the simulator had been built from the same wrong
 * numbers. See docs/13-gps.md.
 */

enum {
    AK_GPS_STATE_SYNC1 = 0,
    AK_GPS_STATE_SYNC2,
    AK_GPS_STATE_CLASS,
    AK_GPS_STATE_ID,
    AK_GPS_STATE_LEN1,
    AK_GPS_STATE_LEN2,
    AK_GPS_STATE_PAYLOAD,
    AK_GPS_STATE_CK_A,
    AK_GPS_STATE_CK_B,
};

#define AK_GPS_CLASS_NAV 0x01u
#define AK_GPS_MSG_PVT   0x07u

#define UBX_FLAGS_FIX_OK 0x01u

static uint32_t read_u32(const uint8_t *data)
{
    return (uint32_t)data[0] | ((uint32_t)data[1] << 8) |
           ((uint32_t)data[2] << 16) | ((uint32_t)data[3] << 24);
}

static int32_t read_i32(const uint8_t *data)
{
    return (int32_t)read_u32(data);
}

static uint16_t read_u16(const uint8_t *data)
{
    return (uint16_t)((uint16_t)data[0] | ((uint16_t)data[1] << 8));
}

void ak_gps_ubx_checksum(const uint8_t *data, uint16_t length, uint8_t *a,
                         uint8_t *b)
{
    uint8_t sum_a = 0;
    uint8_t sum_b = 0;

    for (uint16_t i = 0; i < length; i++) {
        sum_a = (uint8_t)(sum_a + data[i]);
        sum_b = (uint8_t)(sum_b + sum_a);
    }
    *a = sum_a;
    *b = sum_b;
}

void ak_gps_init(ak_gps_t *gps)
{
    for (unsigned i = 0; i < sizeof gps->fix; i++) {
        ((uint8_t *)&gps->fix)[i] = 0;
    }
    gps->state = AK_GPS_STATE_SYNC1;
    gps->held = 0;
    gps->expected = 0;
    gps->messages = 0;
    gps->fix_messages = 0;
    gps->checksum_errors = 0;
    gps->bad_length = 0;
    gps->other_messages = 0;
    gps->bytes = 0;
    gps->last_byte_ms = 0;
    gps->have_fix = 0;
    gps->min_sats = 6;
}

static void decode_pvt(ak_gps_t *gps, uint32_t now_ms)
{
    const uint8_t *payload = gps->buffer;

    gps->fix.fix_type = payload[AK_GPS_PVT_FIX_TYPE];
    gps->fix.fix_ok = (payload[AK_GPS_PVT_FLAGS] & UBX_FLAGS_FIX_OK) != 0;
    gps->fix.satellites = payload[AK_GPS_PVT_SATELLITES];
    gps->fix.lon_e7 = read_i32(&payload[AK_GPS_PVT_LON]);
    gps->fix.lat_e7 = read_i32(&payload[AK_GPS_PVT_LAT]);
    gps->fix.alt_msl_mm = read_i32(&payload[AK_GPS_PVT_ALT_MSL]);
    gps->fix.hacc_mm = read_u32(&payload[AK_GPS_PVT_HACC]);
    gps->fix.speed_mm_s = read_i32(&payload[AK_GPS_PVT_SPEED]);
    gps->fix.course_e5 = read_i32(&payload[AK_GPS_PVT_COURSE]);
    gps->fix.pdop_e2 = read_u16(&payload[AK_GPS_PVT_PDOP]);
    gps->fix.last_fix_ms = now_ms;
    gps->have_fix = 1;
    gps->fix_messages++;
}

int ak_gps_feed(ak_gps_t *gps, uint8_t byte, uint32_t now_ms)
{
    gps->bytes++;

    if (gps->state != AK_GPS_STATE_SYNC1 &&
        (uint32_t)(now_ms - gps->last_byte_ms) > AK_GPS_GAP_MS) {
        gps->bad_length++;
        gps->state = AK_GPS_STATE_SYNC1;
    }
    gps->last_byte_ms = now_ms;

    switch (gps->state) {
    case AK_GPS_STATE_SYNC1:
        if (byte == AK_GPS_UBX_SYNC1) {
            gps->state = AK_GPS_STATE_SYNC2;
        }
        break;

    case AK_GPS_STATE_SYNC2:
        /* A second sync byte that is itself a sync byte is the start of a new
         * frame, not noise: stay where we are rather than dropping it. */
        gps->state = byte == AK_GPS_UBX_SYNC2 ? AK_GPS_STATE_CLASS
                                              : AK_GPS_STATE_SYNC1;
        break;

    case AK_GPS_STATE_CLASS:
        gps->msg_class = byte;
        gps->checksum_a = byte;
        gps->checksum_b = byte;
        gps->state = AK_GPS_STATE_ID;
        break;

    case AK_GPS_STATE_ID:
        gps->msg_id = byte;
        gps->checksum_a = (uint8_t)(gps->checksum_a + byte);
        gps->checksum_b = (uint8_t)(gps->checksum_b + gps->checksum_a);
        gps->state = AK_GPS_STATE_LEN1;
        break;

    case AK_GPS_STATE_LEN1:
        gps->expected = byte;
        gps->checksum_a = (uint8_t)(gps->checksum_a + byte);
        gps->checksum_b = (uint8_t)(gps->checksum_b + gps->checksum_a);
        gps->state = AK_GPS_STATE_LEN2;
        break;

    case AK_GPS_STATE_LEN2:
        gps->expected = (uint16_t)(gps->expected | ((uint16_t)byte << 8));
        gps->checksum_a = (uint8_t)(gps->checksum_a + byte);
        gps->checksum_b = (uint8_t)(gps->checksum_b + gps->checksum_a);
        gps->held = 0;
        if (gps->expected > sizeof gps->buffer) {
            /* Too long to be ours. Counted, and the parser starts again rather
             * than storing a frame it cannot hold. */
            gps->bad_length++;
            gps->state = AK_GPS_STATE_SYNC1;
        } else {
            gps->state = AK_GPS_STATE_PAYLOAD;
        }
        break;

    case AK_GPS_STATE_PAYLOAD:
        gps->buffer[gps->held++] = byte;
        gps->checksum_a = (uint8_t)(gps->checksum_a + byte);
        gps->checksum_b = (uint8_t)(gps->checksum_b + gps->checksum_a);
        if (gps->held >= gps->expected) {
            gps->payload_a = gps->checksum_a;
            gps->payload_b = gps->checksum_b;
            gps->state = AK_GPS_STATE_CK_A;
        }
        break;

    case AK_GPS_STATE_CK_A:
        gps->checksum_a = byte;
        gps->state = AK_GPS_STATE_CK_B;
        break;

    case AK_GPS_STATE_CK_B:
        gps->state = AK_GPS_STATE_SYNC1;
        if (byte != gps->payload_b || gps->checksum_a != gps->payload_a) {
            gps->checksum_errors++;
            break;
        }

        gps->messages++;
        if (gps->msg_class == AK_GPS_CLASS_NAV && gps->msg_id == AK_GPS_MSG_PVT &&
            gps->expected == AK_GPS_UBX_NAV_PVT_LENGTH) {
            decode_pvt(gps, now_ms);
            return 1;
        }
        if (gps->msg_class == AK_GPS_CLASS_NAV && gps->msg_id == AK_GPS_MSG_PVT) {
            /* The right message with the wrong length is a receiver speaking a
             * version we do not know, and worth telling apart from noise. */
            gps->bad_length++;
        } else {
            gps->other_messages++;
        }
        break;
    }

    return 0;
}

int ak_gps_fix_valid(const ak_gps_t *gps, uint32_t now_ms, uint32_t max_age_ms)
{
    if (!gps->have_fix || !gps->fix.fix_ok) {
        return 0;
    }
    /*
     * Three dimensions, not two.
     *
     * A 2D fix has a position and no height, and the fused altitude takes its
     * absolute reference from the first fix the aircraft sees on the ground:
     * anchoring that to whatever a 2D fix reports as "altitude" is how a return
     * descends to the wrong ground level. INAV requires 3D for the same reason.
     */
    if (gps->fix.fix_type < AK_GPS_FIX_3D) {
        return 0;
    }
    /*
     * And enough satellites to believe it. A receiver with four in view can
     * report a fix and be tens of metres out; the navigator would fly that
     * error home. The floor is a parameter so that a part which reports fewer
     * satellites than it is using can be accommodated - the default is INAV's.
     */
    if (gps->fix.satellites < gps->min_sats) {
        return 0;
    }
    return (uint32_t)(now_ms - gps->fix.last_fix_ms) <= max_age_ms;
}

void ak_gps_report(const ak_gps_t *gps, ak_printf_fn out)
{
    out("gps:       %u bytes, %u messages, %u NAV-PVT, %u bad checksums, "
        "%u bad lengths, %u other\n",
        gps->bytes, gps->messages, gps->fix_messages, gps->checksum_errors,
        gps->bad_length, gps->other_messages);

    if (!gps->have_fix) {
        out("fix:       none yet\n");
        return;
    }

    out("fix:       type %u%s, %u satellites, pdop %u.%02u, hacc %u mm\n",
        gps->fix.fix_type, gps->fix.fix_ok ? " (ok)" : " (not ok)",
        gps->fix.satellites, gps->fix.pdop_e2 / 100u, gps->fix.pdop_e2 % 100u,
        gps->fix.hacc_mm);
    /* Whether the navigator would use it, and why not when it would not: the
     * numbers above are for a person to look at, and this is the one the
     * firmware actually decides on. */
    if (gps->fix.fix_type < AK_GPS_FIX_3D) {
        out("usable:    no - a 2D fix has no height to anchor an altitude "
            "to\n");
    } else if (gps->fix.satellites < gps->min_sats) {
        out("usable:    no - %u satellites, and the navigator wants %u\n",
            gps->fix.satellites, gps->min_sats);
    } else {
        out("usable:    yes (needs %u satellites, has %u)\n", gps->min_sats,
            gps->fix.satellites);
    }
    out("position:  %d.%07d, %d.%07d\n", gps->fix.lat_e7 / 10000000,
        gps->fix.lat_e7 % 10000000, gps->fix.lon_e7 / 10000000,
        gps->fix.lon_e7 % 10000000);
    out("altitude:  %d mm msl\n", gps->fix.alt_msl_mm);
    out("motion:    %d mm/s, course %d.%05d deg\n", gps->fix.speed_mm_s,
        gps->fix.course_e5 / 100000, gps->fix.course_e5 % 100000);
}
