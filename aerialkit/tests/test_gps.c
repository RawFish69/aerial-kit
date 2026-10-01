/*
 * GPS: UBX framing and the NAV-PVT decode.
 *
 * Two kinds of check here, and the difference matters. The *framing* is checked
 * against a real captured UBX frame - the one INAV's own unit test carries, a
 * configuration message with its bytes spelled out - so the sync pair, the
 * length byte order and the checksum are confirmed by something other than this
 * code. The NAV-PVT decode is checked against a payload this test builds from
 * the field offsets, which pins the parser's behaviour but cannot prove the
 * offsets are the ones a receiver uses; a receiver is the only thing that can.
 *
 * That second half used to be circular: the payload was built from the parser's
 * own constants, so the test agreed with the parser about a layout that was two
 * bytes wrong, and the only thing that could have caught it - the offsets
 * themselves - was the one thing not checked. Now the payload is built from
 * ubx_reference.h, which is INAV's ubx_nav_pvt copied field for field, and the
 * constants are checked against offsetof() of it. The decode is still a claim
 * about a payload; the layout is now a claim about the upstream.
 */

#include <stdint.h>
#include <stddef.h>
#include <string.h>

#include "ak_gps.h"
#include "tests.h"
#include "ubx_reference.h"

/*
 * Every offset the parser reads, checked against the layout a receiver uses.
 * If INAV's struct and the u-blox interface description ever disagree, this
 * breaks here rather than at a bench with a module that answers but never
 * gives a fix.
 */
_Static_assert(offsetof(ak_ubx_nav_pvt_t, fix_type) == AK_GPS_PVT_FIX_TYPE,
               "NAV-PVT fix type offset");
_Static_assert(offsetof(ak_ubx_nav_pvt_t, flags) == AK_GPS_PVT_FLAGS,
               "NAV-PVT flags offset");
_Static_assert(offsetof(ak_ubx_nav_pvt_t, satellites) == AK_GPS_PVT_SATELLITES,
               "NAV-PVT satellite count offset");
_Static_assert(offsetof(ak_ubx_nav_pvt_t, longitude) == AK_GPS_PVT_LON,
               "NAV-PVT longitude offset");
_Static_assert(offsetof(ak_ubx_nav_pvt_t, latitude) == AK_GPS_PVT_LAT,
               "NAV-PVT latitude offset");
_Static_assert(offsetof(ak_ubx_nav_pvt_t, altitude_msl) == AK_GPS_PVT_ALT_MSL,
               "NAV-PVT altitude offset");
_Static_assert(offsetof(ak_ubx_nav_pvt_t, horizontal_accuracy) ==
                   AK_GPS_PVT_HACC,
               "NAV-PVT horizontal accuracy offset");
_Static_assert(offsetof(ak_ubx_nav_pvt_t, speed_2d) == AK_GPS_PVT_SPEED,
               "NAV-PVT ground speed offset");
_Static_assert(offsetof(ak_ubx_nav_pvt_t, heading_2d) == AK_GPS_PVT_COURSE,
               "NAV-PVT heading offset");
_Static_assert(offsetof(ak_ubx_nav_pvt_t, position_dop) == AK_GPS_PVT_PDOP,
               "NAV-PVT position DOP offset");

static void put_u16(uint8_t *dst, uint16_t value)
{
    dst[0] = (uint8_t)(value & 0xFFu);
    dst[1] = (uint8_t)((value >> 8) & 0xFFu);
}

/* Builds a NAV-PVT frame the way a receiver does, from the offsets. */
static uint16_t build_pvt(uint8_t *frame, const uint8_t *payload)
{
    uint8_t a;
    uint8_t b;

    frame[0] = AK_GPS_UBX_SYNC1;
    frame[1] = AK_GPS_UBX_SYNC2;
    frame[2] = 0x01; /* class NAV */
    frame[3] = 0x07; /* id PVT */
    put_u16(&frame[4], AK_GPS_UBX_NAV_PVT_LENGTH);
    memcpy(&frame[6], payload, AK_GPS_UBX_NAV_PVT_LENGTH);
    ak_gps_ubx_checksum(&frame[2], (uint16_t)(4u + AK_GPS_UBX_NAV_PVT_LENGTH), &a, &b);
    frame[6 + AK_GPS_UBX_NAV_PVT_LENGTH] = a;
    frame[7 + AK_GPS_UBX_NAV_PVT_LENGTH] = b;
    return (uint16_t)(8u + AK_GPS_UBX_NAV_PVT_LENGTH);
}

static int feed_all(ak_gps_t *gps, const uint8_t *data, uint16_t length,
                    uint32_t now_ms)
{
    int decoded = 0;
    for (uint16_t i = 0; i < length; i++) {
        decoded = ak_gps_feed(gps, data[i], now_ms) || decoded;
    }
    return decoded;
}

/* The reference layout into the payload a receiver sends. The last eight bytes
 * of the message - four reserved, then headVeh and the magnetic declination -
 * are fields this firmware does not read, so the struct stops short and the
 * rest stays zero. */
static void payload_from(const ak_ubx_nav_pvt_t *pvt, uint8_t *payload)
{
    memset(payload, 0, AK_GPS_UBX_NAV_PVT_LENGTH);
    memcpy(payload, pvt, sizeof *pvt);
}

static void test_framing_against_a_captured_frame(void)
{
    ak_gps_t gps;
    ak_gps_init(&gps);

    /* A real UBX-CFG-VALSET frame, byte for byte, from INAV's unit test
     * (src/test/unit/gps_ublox_unittest.cc at the pinned revision). If our
     * checksum or byte order were wrong, this frame would be rejected. */
    static const uint8_t captured[] = {
        0xB5, 0x62, 0x06, 0x8A, 0x09, 0x00, 0x01, 0x01,
        0x00, 0x00, 0x25, 0x00, 0x31, 0x10, 0x01, 0x02, 0xA7,
    };

    expect("a captured frame passes the checksum",
           feed_all(&gps, captured, sizeof captured, 1000) == 0 &&
           gps.messages == 1 && gps.checksum_errors == 0);
    expect("and is counted as a message we do not use",
           gps.other_messages == 1 && gps.fix_messages == 0);

    /* Corrupt one payload byte: the checksum has to catch it. */
    uint8_t damaged[sizeof captured];
    memcpy(damaged, captured, sizeof captured);
    damaged[10] ^= 0x01;
    ak_gps_init(&gps);
    feed_all(&gps, damaged, sizeof damaged, 1000);
    expect("a corrupted frame is rejected",
           gps.messages == 0 && gps.checksum_errors == 1);
}

static void test_nav_pvt(void)
{
    ak_gps_t gps;
    ak_ubx_nav_pvt_t pvt;
    uint8_t payload[AK_GPS_UBX_NAV_PVT_LENGTH];
    uint8_t frame[8 + AK_GPS_UBX_NAV_PVT_LENGTH];

    ak_gps_init(&gps);

    /* 52.1234567 N, 4.9876543 E, 1234 m above sea level, 3 m/s, heading
     * 270.12345 deg, 11 satellites, DOP 1.23, accuracy 1.5 m. */
    memset(&pvt, 0, sizeof pvt);
    pvt.fix_type = AK_GPS_FIX_3D;
    pvt.flags = 0x01; /* gnssFixOK */
    pvt.satellites = 11;
    pvt.longitude = 49876543;         /* 4.9876543 E  */
    pvt.latitude = 521234567;         /* 52.1234567 N */
    pvt.altitude_msl = 1234000;       /* mm           */
    pvt.horizontal_accuracy = 1500;   /* mm           */
    pvt.speed_2d = 3000;              /* mm/s         */
    pvt.heading_2d = 27012345;        /* 1e-5 deg     */
    pvt.position_dop = 123;           /* 1.23         */
    payload_from(&pvt, payload);

    uint16_t length = build_pvt(frame, payload);
    expect("a NAV-PVT frame decodes", feed_all(&gps, frame, length, 5000) == 1);
    expect("and is counted as a fix", gps.fix_messages == 1 && gps.have_fix == 1);

    expect("the fix type and satellites come through",
           gps.fix.fix_type == AK_GPS_FIX_3D && gps.fix.fix_ok == 1 &&
           gps.fix.satellites == 11);
    expect("position is kept at 1e-7 degrees",
           gps.fix.lat_e7 == 521234567 && gps.fix.lon_e7 == 49876543);
    expect("altitude, accuracy and DOP keep their units",
           gps.fix.alt_msl_mm == 1234000 && gps.fix.hacc_mm == 1500 &&
           gps.fix.pdop_e2 == 123);
    expect("motion keeps its units too",
           gps.fix.speed_mm_s == 3000 && gps.fix.course_e5 == 27012345);
    expect("the fix is stamped with the time it arrived",
           gps.fix.last_fix_ms == 5000);

    /* A positive latitude and a negative longitude, because a sign error here
     * puts an aircraft in the sea. */
    memset(&pvt, 0, sizeof pvt);
    pvt.fix_type = AK_GPS_FIX_3D;
    pvt.flags = 0x01;
    pvt.satellites = 11;
    pvt.longitude = -1224194300; /* -122.4194300 */
    pvt.latitude = -337000000;   /* -33.7000000  */
    payload_from(&pvt, payload);
    length = build_pvt(frame, payload);
    feed_all(&gps, frame, length, 6000);
    expect("southern and western positions keep their signs",
           gps.fix.lat_e7 == -337000000 && gps.fix.lon_e7 == -1224194300);

    /* Validity: a fix that is stale, not ok, or has no position is not valid,
     * and the navigation code needs all three to be true. */
    expect("a fresh, ok, 3D fix is valid",
           ak_gps_fix_valid(&gps, 6000, 1000) == 1);
    expect("an old fix is not", ak_gps_fix_valid(&gps, 9000, 1000) == 0);

    pvt.flags = 0x00; /* the receiver says not ok */
    payload_from(&pvt, payload);
    length = build_pvt(frame, payload);
    feed_all(&gps, frame, length, 7000);
    expect("a fix the receiver does not trust is not valid",
           ak_gps_fix_valid(&gps, 7000, 1000) == 0);

    pvt.fix_type = AK_GPS_FIX_NONE;
    pvt.flags = 0x01;
    payload_from(&pvt, payload);
    length = build_pvt(frame, payload);
    feed_all(&gps, frame, length, 8000);
    expect("and neither is one with no position",
           ak_gps_fix_valid(&gps, 8000, 1000) == 0);
}

static void test_recovery(void)
{
    ak_gps_t gps;
    ak_ubx_nav_pvt_t pvt;
    uint8_t payload[AK_GPS_UBX_NAV_PVT_LENGTH];
    uint8_t frame[8 + AK_GPS_UBX_NAV_PVT_LENGTH];

    ak_gps_init(&gps);
    memset(&pvt, 0, sizeof pvt);
    pvt.fix_type = AK_GPS_FIX_3D;
    pvt.flags = 0x01;
    payload_from(&pvt, payload);
    uint16_t length = build_pvt(frame, payload);

    /* Noise, a truncated frame, and then a real one: the parser has to find the
     * sync pair in the middle of the mess rather than staying lost. */
    for (int i = 0; i < 40; i++) {
        ak_gps_feed(&gps, (uint8_t)(0x11 + i), 1000);
    }
    for (uint16_t i = 0; i < 30; i++) {
        ak_gps_feed(&gps, frame[i], 2000);
    }
    /* The rest of that frame never arrives. The next one is the first chance to
     * recover, and the parser has to take it rather than staying lost. */
    expect("a truncated frame is abandoned after a gap, and the next decodes",
           feed_all(&gps, frame, length, 2100) == 1);

    /* A frame with a length we cannot hold is counted, not stored. */
    ak_gps_init(&gps);
    uint8_t long_frame[] = { 0xB5, 0x62, 0x01, 0x07, 0xFF, 0x00 };
    feed_all(&gps, long_frame, sizeof long_frame, 1000);
    expect("an over-long frame is rejected by length", gps.bad_length == 1);

    /* The right message with the wrong length is a different problem from
     * noise, and worth its own counter. */
    ak_gps_init(&gps);
    uint8_t short_pvt[8 + 4 + 2];
    memset(short_pvt, 0, sizeof short_pvt);
    short_pvt[0] = 0xB5;
    short_pvt[1] = 0x62;
    short_pvt[2] = 0x01;
    short_pvt[3] = 0x07;
    put_u16(&short_pvt[4], 4);
    uint8_t a;
    uint8_t b;
    /* The checksum goes after the payload, not inside it - which is the mistake
     * this test made first, and the reason the parser saw a bad checksum
     * instead of a bad length. */
    ak_gps_ubx_checksum(&short_pvt[2], 8, &a, &b);
    short_pvt[10] = a;
    short_pvt[11] = b;
    feed_all(&gps, short_pvt, sizeof short_pvt, 1000);
    expect("a NAV-PVT of the wrong length is counted as a bad length",
           gps.bad_length == 1 && gps.fix_messages == 0);
}

/*
 * What makes a fix worth navigating on.
 *
 * The parser used to accept any fresh 2D-or-better fix the receiver called ok,
 * and the numbers a receiver reports about its own quality - the satellite
 * count, the DOP, the horizontal accuracy - were shown on the console and used
 * to decide nothing. That is the wrong way round for the one thing they are
 * for: a navigator that flies to a position is only as right as the position,
 * and a fix with four satellites can be tens of metres out. INAV's rule is
 * `fixType == 3D && numSat >= gpsMinSats` with a default of six, and its reason
 * is a property of the receivers rather than of the firmware: "some GPS
 * receivers appeared to be very inaccurate with low satellite count".
 *
 * 3D matters for a different reason here: a 2D fix has no height, and the
 * fused altitude takes its absolute reference from the first fix the aircraft
 * sees on the ground.
 */
static void test_fix_quality(void)
{
    ak_gps_t gps;
    ak_ubx_nav_pvt_t pvt;
    uint8_t payload[AK_GPS_UBX_NAV_PVT_LENGTH];
    uint8_t frame[8 + AK_GPS_UBX_NAV_PVT_LENGTH];

    ak_gps_init(&gps);
    expect("the floor is INAV's", gps.min_sats == 6u);

    /* A plenty-good fix: 3D, ok, eleven satellites, fresh. */
    memset(&pvt, 0, sizeof pvt);
    pvt.fix_type = AK_GPS_FIX_3D;
    pvt.flags = 0x01;
    pvt.satellites = 11;
    pvt.latitude = 521234567;
    pvt.longitude = 49876543;
    payload_from(&pvt, payload);
    uint16_t length = build_pvt(frame, payload);
    feed_all(&gps, frame, length, 1000);
    expect("a 3D fix with satellites to spare is worth navigating on",
           ak_gps_fix_valid(&gps, 1000, 1000) == 1);

    /* The same fix with fewer satellites than the floor. */
    pvt.satellites = 4;
    payload_from(&pvt, payload);
    length = build_pvt(frame, payload);
    feed_all(&gps, frame, length, 2000);
    expect("and one with four satellites is not, however ok it says it is",
           ak_gps_fix_valid(&gps, 2000, 1000) == 0);

    /* And the floor is a parameter, so a part that reports fewer satellites
     * than it uses can be accommodated. */
    gps.min_sats = 8u;
    pvt.satellites = 7;
    payload_from(&pvt, payload);
    length = build_pvt(frame, payload);
    feed_all(&gps, frame, length, 3000);
    expect("a satellite floor of eight refuses seven",
           ak_gps_fix_valid(&gps, 3000, 1000) == 0);
    gps.min_sats = 6u;

    /* A 2D fix has a position and no height: it is not something to anchor a
     * ground level to, which is what the fused altitude does with it. */
    pvt.fix_type = AK_GPS_FIX_2D;
    pvt.satellites = 11;
    payload_from(&pvt, payload);
    length = build_pvt(frame, payload);
    feed_all(&gps, frame, length, 4000);
    expect("and a 2D fix is not valid even with every satellite in the sky",
           ak_gps_fix_valid(&gps, 4000, 1000) == 0);
}

void test_gps(void)
{
    test_framing_against_a_captured_frame();
    test_nav_pvt();
    test_recovery();
    test_fix_quality();
}
