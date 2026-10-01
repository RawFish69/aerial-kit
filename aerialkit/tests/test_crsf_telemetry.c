/*
 * CRSF telemetry out: the frames a handset reads.
 *
 * Three things are worth checking here, and they are different kinds of check.
 *
 * 1. The crc, against the *catalogue* value for CRC-8/DVB-S2 rather than
 *    against itself: a telemetry frame a handset drops is a frame that was
 *    never sent, and the crc covers one span of the frame that is easy to get
 *    wrong.
 * 2. The fields, decoded back the way the handset decodes them - volts to a
 *    tenth, attitude to a ten-thousandth of a radian, a position in degrees
 *    times ten million, a speed in a unit nobody would invent.
 * 3. That the firmware's own receive parser accepts what the transmit path
 *    builds: one protocol, two halves, checked against each other.
 */

#include <stdint.h>
#include <string.h>

#include "ak_crsf.h"
#include "ak_crsf_telemetry.h"
#include "tests.h"

static uint16_t get_u16(const uint8_t *at)
{
    return (uint16_t)(((uint16_t)at[0] << 8) | at[1]);
}

static uint32_t get_u32(const uint8_t *at)
{
    return ((uint32_t)at[0] << 24) | ((uint32_t)at[1] << 16) |
           ((uint32_t)at[2] << 8) | at[3];
}

static void test_crc(void)
{
    /* The standard check value: the crc of "123456789" for CRC-8/DVB-S2 is
     * 0xBC. It is in every crc catalogue, which makes it the one number here
     * that does not come from this repository. */
    expect("the crc is CRC-8/DVB-S2, by its catalogue check value",
           ak_crsf_crc8((const uint8_t *)"123456789", 9) == 0xBCu);
    expect("and an empty span crcs to zero", ak_crsf_crc8(0, 0) == 0u);
}

/* The envelope every frame shares: address, length, type, payload, crc. */
static void test_envelope(void)
{
    ak_crsf_tlm_t tlm;
    ak_crsf_telemetry_t in;
    uint8_t frame[AK_CRSF_TLM_MAX_FRAME];
    unsigned len;
    ak_crsf_t receiver;
    ak_rc_input_t channels;

    ak_crsf_tlm_init(&tlm);
    memset(&in, 0, sizeof in);
    in.attitude_valid = 1;
    in.roll_rad = 0.5f;
    in.pitch_rad = -0.25f;
    in.yaw_rad = 1.0f;

    len = ak_crsf_tlm_next(&tlm, &in, 0u, frame, sizeof frame);
    expect("an attitude frame is ten bytes", len == 10u);
    expect("with the flight controller's address first",
           frame[0] == AK_CRSF_ADDRESS_FLIGHT_CONTROLLER);
    expect("a length that counts the type, the payload and the crc",
           frame[1] == 8u && len == (unsigned)frame[1] + 2u);
    expect("and the attitude type", frame[2] == AK_CRSF_TYPE_ATTITUDE);
    expect("the crc covers the type and payload, and matches",
           frame[len - 1u] == ak_crsf_crc8(&frame[2], (uint8_t)(len - 3u)));

    /* The same span is the one the receive path checks, so feeding the frame
     * back in must be a frame the firmware would accept from a receiver. */
    ak_crsf_init(&receiver);
    memset(&channels, 0, sizeof channels);
    for (unsigned i = 0; i < len; i++) {
        (void)ak_crsf_feed(&receiver, frame[i], &channels, 1u);
    }
    expect("and the receiver's own parser accepts it",
           receiver.frames == 1u && receiver.crc_errors == 0u);
}

static void test_fields(void)
{
    ak_crsf_tlm_t tlm;
    ak_crsf_telemetry_t in;
    uint8_t frame[AK_CRSF_TLM_MAX_FRAME];
    unsigned len;

    /* Battery: volts in tenths, the two counters zero because there is no
     * current sensor, and the percentage. */
    ak_crsf_tlm_init(&tlm);
    memset(&in, 0, sizeof in);
    in.volts_valid = 1;
    in.volts = 12.44f;
    in.percent_valid = 1;
    in.percent = 77;
    len = ak_crsf_tlm_next(&tlm, &in, 0u, frame, sizeof frame);
    expect("a battery frame is twelve bytes on the wire",
           len == 12u && frame[2] == AK_CRSF_TYPE_BATTERY);
    expect("the pack voltage is in tenths of a volt",
           get_u16(&frame[3]) == 124u);
    expect("current and capacity are zero, not invented",
           get_u16(&frame[5]) == 0u && (get_u32(&frame[7]) >> 8) == 0u);
    expect("and the percentage is the one it was given", frame[10] == 77u);

    /* Attitude: three int16, radians times ten thousand, big endian. */
    ak_crsf_tlm_init(&tlm);
    memset(&in, 0, sizeof in);
    in.attitude_valid = 1;
    in.roll_rad = 0.5236f; /* 30 degrees, in the estimator's units */
    in.pitch_rad = -0.25f;
    in.yaw_rad = 3.0f;
    len = ak_crsf_tlm_next(&tlm, &in, 0u, frame, sizeof frame);
    expect("the attitude frame carries pitch, roll and yaw in that order",
           (int16_t)get_u16(&frame[3]) == -2500 &&
               (int16_t)get_u16(&frame[5]) == 5236 &&
               (int16_t)get_u16(&frame[7]) == 30000);

    /* The same frame, handed a yaw the estimator has carried past a turn - it
     * is the continuous angle, and the frame is not. Ten radians is 572.96
     * degrees, which is -147.04 as a heading, and the two things this pins are
     * that it is that heading and not the 100000 the field cannot hold. */
    ak_crsf_tlm_init(&tlm);
    memset(&in, 0, sizeof in);
    in.attitude_valid = 1;
    in.yaw_rad = 10.0f;
    len = ak_crsf_tlm_next(&tlm, &in, 0u, frame, sizeof frame);
    expect("a yaw past a whole turn is wrapped into the attitude frame",
           len == 10u && (int16_t)get_u16(&frame[7]) > -25668 &&
               (int16_t)get_u16(&frame[7]) < -25660);

    /* GPS: degrees times ten million, speed in km/h times ten, course in
     * degrees times ten, altitude offset by a thousand metres, satellites. */
    ak_crsf_tlm_init(&tlm);
    memset(&in, 0, sizeof in);
    in.fix_valid = 1;
    in.lat_e7 = 521234567;
    in.lon_e7 = 49876543;
    in.alt_m = 123.4f;
    in.speed_mm_s = 10000.0f; /* 10 m/s is 36 km/h */
    in.course_deg = 359.9f;
    in.satellites = 11u;
    len = ak_crsf_tlm_next(&tlm, &in, 0u, frame, sizeof frame);
    expect("a gps frame is nineteen bytes on the wire",
           len == 19u && frame[2] == AK_CRSF_TYPE_GPS);
    expect("the position is degrees times ten million",
           (int32_t)get_u32(&frame[3]) == 521234567 &&
               (int32_t)get_u32(&frame[7]) == 49876543);
    expect("the ground speed is km/h times ten", get_u16(&frame[11]) == 360u);
    expect("the course is degrees times ten", get_u16(&frame[13]) == 3599u);
    expect("the altitude is metres above mean sea level, plus a thousand",
           get_u16(&frame[15]) == 1123u);
    expect("and the satellites are the module's count", frame[17] == 11u);

    /* And a position in the southern and western hemispheres, which is where
     * a sign error would show up. */
    in.lat_e7 = -338654321;
    in.lon_e7 = -701234567;
    ak_crsf_tlm_init(&tlm);
    (void)ak_crsf_tlm_next(&tlm, &in, 0u, frame, sizeof frame);
    expect("a southern, western position keeps its signs",
           (int32_t)get_u32(&frame[3]) == -338654321 &&
               (int32_t)get_u32(&frame[7]) == -701234567);

    /* Flight mode: the string, null terminated. */
    ak_crsf_tlm_init(&tlm);
    memset(&in, 0, sizeof in);
    in.flight_mode = "RTH";
    len = ak_crsf_tlm_next(&tlm, &in, 0u, frame, sizeof frame);
    expect("the flight mode travels as text, null terminated",
           len == 8u && frame[2] == AK_CRSF_TYPE_FLIGHT_MODE &&
               memcmp(&frame[3], "RTH", 4u) == 0);
}

static void test_device_info(void)
{
    ak_crsf_tlm_t tlm;
    uint8_t frame[AK_CRSF_TLM_MAX_FRAME];
    unsigned len;

    ak_crsf_tlm_init(&tlm);
    len = ak_crsf_tlm_device_info(&tlm, "aerialkit-f405", frame, sizeof frame);

    expect("a device info reply names the addresses both ways",
           len > 4u && frame[2] == AK_CRSF_TYPE_DEVICE_INFO &&
               frame[3] == AK_CRSF_ADDRESS_RADIO_TRANSMITTER &&
               frame[4] == AK_CRSF_ADDRESS_FLIGHT_CONTROLLER);
    expect("the name is null terminated inside the payload",
           memcmp(&frame[5], "aerialkit-f405", 14u) == 0 && frame[19] == 0u);
    expect("twelve reserved bytes follow it",
           frame[19 + 12u] == 0u && frame[20] == 0u);
    expect("no parameters over CRSF, and parameter version 1",
           frame[len - 3u] == 0u && frame[len - 2u] == 1u);
    expect("the crc covers the whole payload",
           frame[len - 1u] == ak_crsf_crc8(&frame[2], (uint8_t)(len - 3u)));

    /* A name that fills the buffer must not overflow it or lose its
     * terminator: this is a wire format with a length byte, and a long name is
     * how one frame becomes two. */
    len = ak_crsf_tlm_device_info(&tlm, "a-name-far-too-long-for-this-frame",
                                  frame, sizeof frame);
    expect("a name that does not fit is truncated, not fatal",
           len > 0u && len <= AK_CRSF_TLM_MAX_FRAME &&
               frame[len - 1u] ==
                   ak_crsf_crc8(&frame[2], (uint8_t)(len - 3u)));
    expect("and it still counts the reply", tlm.device_infos == 2u);
}

static void test_schedule(void)
{
    ak_crsf_tlm_t tlm;
    ak_crsf_telemetry_t in;
    uint8_t frame[AK_CRSF_TLM_MAX_FRAME];
    unsigned counts[4] = { 0u, 0u, 0u, 0u };
    unsigned nothing = 0u;

    ak_crsf_tlm_init(&tlm);
    memset(&in, 0, sizeof in);
    in.volts_valid = 1;
    in.volts = 12.0f;
    in.percent_valid = 1;
    in.percent = 50;
    in.attitude_valid = 1;
    in.fix_valid = 1;
    in.lat_e7 = 1;
    in.lon_e7 = 2;
    in.satellites = 9u;
    in.flight_mode = "ACRO";

    /* Two seconds at the rate the flight loop ticks. */
    for (uint32_t now = 0; now <= 2000u; now += AK_CRSF_TLM_TICK_MS) {
        unsigned len = ak_crsf_tlm_next(&tlm, &in, now, frame, sizeof frame);

        if (len == 0u) {
            continue;
        }
        switch (frame[2]) {
        case AK_CRSF_TYPE_ATTITUDE:
            counts[0]++;
            break;
        case AK_CRSF_TYPE_BATTERY:
            counts[1]++;
            break;
        case AK_CRSF_TYPE_GPS:
            counts[2]++;
            break;
        case AK_CRSF_TYPE_FLIGHT_MODE:
            counts[3]++;
            break;
        default:
            counts[0] += 100u; /* an unknown frame fails the check below */
            break;
        }
    }

    /* The first tick sends what is already known; after that each type runs at
     * its own period, to within the tick it is scheduled on. */
    expect("attitude goes out at about ten a second",
           counts[0] >= 20u && counts[0] <= 22u);
    expect("the battery at about five", counts[1] >= 10u && counts[1] <= 12u);
    expect("the position at about two", counts[2] >= 4u && counts[2] <= 6u);
    expect("and the flight mode at about one",
           counts[3] >= 2u && counts[3] <= 4u);

    /* A frame whose data is missing is skipped rather than sent as zeros, and
     * a board with nothing to say sends nothing - which is the difference
     * between a handset showing no attitude and a handset showing level. */
    ak_crsf_tlm_init(&tlm);
    memset(&in, 0, sizeof in);
    for (uint32_t now = 0; now <= 1000u; now += AK_CRSF_TLM_TICK_MS) {
        if (ak_crsf_tlm_next(&tlm, &in, now, frame, sizeof frame) > 0u) {
            nothing++;
        }
    }
    expect("a board with no data sends nothing at all",
           nothing == 0u && tlm.frames == 0u);
}

static void test_ping(void)
{
    ak_crsf_t receiver;
    ak_rc_input_t channels;
    uint8_t ping[6];
    uint8_t elsewhere[6];

    /* [address][len][type][destination][origin][crc] */
    ping[0] = AK_CRSF_ADDRESS_FLIGHT_CONTROLLER;
    ping[1] = 4u;
    ping[2] = AK_CRSF_TYPE_DEVICE_PING;
    ping[3] = AK_CRSF_ADDRESS_FLIGHT_CONTROLLER;
    ping[4] = AK_CRSF_ADDRESS_RADIO_TRANSMITTER;
    ping[5] = ak_crsf_crc8(&ping[2], 3u);

    memcpy(elsewhere, ping, sizeof ping);
    elsewhere[3] = 0xC2u; /* a gps, by the crsf address table */
    elsewhere[5] = ak_crsf_crc8(&elsewhere[2], 3u);

    ak_crsf_init(&receiver);
    memset(&channels, 0, sizeof channels);
    for (unsigned i = 0; i < sizeof ping; i++) {
        (void)ak_crsf_feed(&receiver, ping[i], &channels, 1u);
    }
    expect("a device ping addressed to the flight controller is counted",
           receiver.pings == 1u &&
               receiver.last_ping_from == AK_CRSF_ADDRESS_RADIO_TRANSMITTER);
    expect("and it is not mistaken for a channel frame",
           channels.valid == 0 && receiver.frames == 1u);

    for (unsigned i = 0; i < sizeof elsewhere; i++) {
        (void)ak_crsf_feed(&receiver, elsewhere[i], &channels, 1u);
    }
    expect("a ping for somebody else's address is not ours to answer",
           receiver.pings == 1u);
}

static void test_modes(void)
{
    expect("the handset is told the mode, not this firmware's state name",
           strcmp(ak_crsf_flight_mode(0, 0, 0, 0, 0), "DISARM") == 0 &&
               strcmp(ak_crsf_flight_mode(1, 0, 0, 0, 1), "ANGLE") == 0 &&
               strcmp(ak_crsf_flight_mode(1, 0, 0, 0, 0), "ACRO") == 0 &&
               strcmp(ak_crsf_flight_mode(1, 0, 1, 0, 1), "RTH") == 0 &&
               strcmp(ak_crsf_flight_mode(1, 0, 0, 1, 1), "RTH") == 0 &&
               strcmp(ak_crsf_flight_mode(1, 1, 0, 0, 1), "!FS!") == 0);
    expect("and a failsafe outranks everything else",
           strcmp(ak_crsf_flight_mode(1, 1, 1, 1, 1), "!FS!") == 0);
}

void test_crsf_telemetry(void)
{
    test_crc();
    test_envelope();
    test_fields();
    test_device_info();
    test_schedule();
    test_ping();
    test_modes();
}
