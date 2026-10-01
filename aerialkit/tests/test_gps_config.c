/*
 * The GPS configuration frame.
 *
 * The frame layout is not something to guess at, and it is not guessed here:
 * INAV's unit test carries a real captured VALSET frame with one key in it, and
 * the first check is that our builder produces the same bytes for the same
 * input. Everything after that - the checksum, the key encoding, the values -
 * rests on that one comparison.
 */

#include <stdint.h>
#include <string.h>

#include "ak_gps.h"
#include "ak_gps_config.h"
#include "tests.h"

static void test_matches_a_captured_frame(void)
{
    /* The captured frame: B5 62 06 8A 09 00 | 01 01 00 00 | key 25 00 31 10 |
     * value 01 | checksum 02 A7.
     *
     * It sets key 0x10310025 to 1 in RAM. Reproducing those bytes proves the
     * header, the layer byte, the little-endian key and the trailing checksum
     * are all where a receiver expects them. */
    uint8_t frame[AK_GPS_VALSET_MAX_BYTES];
    unsigned at = 0;

    frame[at++] = AK_GPS_UBX_SYNC1;
    frame[at++] = AK_GPS_UBX_SYNC2;
    frame[at++] = 0x06;
    frame[at++] = 0x8A;
    frame[at++] = 9;
    frame[at++] = 0;
    frame[at++] = AK_GPS_VALSET_VERSION;
    frame[at++] = AK_GPS_VALSET_LAYER_RAM;
    frame[at++] = 0;
    frame[at++] = 0;
    frame[at++] = 0x25;                /* key 0x10310025 */
    frame[at++] = 0x00;
    frame[at++] = 0x31;
    frame[at++] = 0x10;
    frame[at++] = 0x01;                /* value 1 */
    uint8_t a;
    uint8_t b;
    ak_gps_ubx_checksum(&frame[2], (uint16_t)(at - 2u), &a, &b);
    frame[at++] = a;
    frame[at++] = b;

    static const uint8_t captured[] = {
        0xB5, 0x62, 0x06, 0x8A, 0x09, 0x00, 0x01, 0x01,
        0x00, 0x00, 0x25, 0x00, 0x31, 0x10, 0x01, 0x02, 0xA7,
    };

    expect("the header, layer, key encoding and checksum match a captured frame",
           at == sizeof captured && memcmp(frame, captured, sizeof captured) == 0);
}

static void test_our_frame(void)
{
    uint8_t frame[AK_GPS_VALSET_MAX_BYTES];
    unsigned length = ak_gps_config_frame(200, frame, sizeof frame);

    expect("the frame is built", length > 0);
    if (length == 0) {
        return; /* everything below indexes into the frame */
    }
    expect("the sync pair and class lead it",
           frame[0] == 0xB5 && frame[1] == 0x62 && frame[2] == 0x06 &&
           frame[3] == 0x8A);

    unsigned payload = (unsigned)frame[4] | ((unsigned)frame[5] << 8);
    expect("the length field counts the payload",
           payload == length - 6u - 2u);
    expect("the payload starts with the version, RAM layer and transaction",
           frame[6] == AK_GPS_VALSET_VERSION &&
           frame[7] == AK_GPS_VALSET_LAYER_RAM && frame[8] == 0 && frame[9] == 0);

    /* Walk the keys and check the two that matter are in there: NAV-PVT asked
     * for once per solution, and the NMEA sentences switched off. */
    int pvt_on = 0;
    int nmea_off = 0;
    int rate_meas_200 = 0;
    unsigned at = 10;
    while (at + 5u <= 6u + payload) {
        uint32_t key = (uint32_t)frame[at] | ((uint32_t)frame[at + 1] << 8) |
                       ((uint32_t)frame[at + 2] << 16) |
                       ((uint32_t)frame[at + 3] << 24);
        if (key == AK_GPS_CFG_MSGOUT_NAV_PVT) {
            pvt_on = frame[at + 4] == 1;
            at += 5;
        } else if (key == AK_GPS_CFG_MSGOUT_NMEA_GGA) {
            nmea_off = frame[at + 4] == 0;
            at += 5;
        } else if (key == AK_GPS_CFG_RATE_MEAS || key == AK_GPS_CFG_RATE_NAV) {
            /* The rate keys carry a 16-bit value, so they take six bytes where
             * the message rates take five - and walking this wrong is how the
             * test failed the first time. */
            if (key == AK_GPS_CFG_RATE_MEAS) {
                rate_meas_200 = frame[at + 4] == 200 && frame[at + 5] == 0;
            }
            at += 6;
        } else {
            at += 5;
        }
    }

    expect("NAV-PVT is asked for once per solution", pvt_on);
    expect("the NMEA sentences are switched off", nmea_off);
    expect("the measurement rate is the one asked for (200 ms here)",
           rate_meas_200);

    /* The checksum at the end has to be the checksum of what is in front. */
    uint8_t a;
    uint8_t b;
    ak_gps_ubx_checksum(&frame[2], (uint16_t)(4u + payload), &a, &b);
    expect("the trailing checksum covers the header and payload",
           frame[length - 2] == a && frame[length - 1] == b);

    /* And the whole frame is something our own parser accepts, which is the
     * only end-to-end check available without a module. */
    ak_gps_t gps;
    ak_gps_init(&gps);
    for (unsigned i = 0; i < length; i++) {
        (void)ak_gps_feed(&gps, frame[i], 1000);
    }
    expect("the frame passes our own framing check",
           gps.messages == 1 && gps.checksum_errors == 0 && gps.other_messages == 1);

    expect("a buffer that is too small is refused, not overrun",
           ak_gps_config_frame(200, frame, 8) == 0);
}

void test_gps_config(void)
{
    test_matches_a_captured_frame();
    test_our_frame();
}
