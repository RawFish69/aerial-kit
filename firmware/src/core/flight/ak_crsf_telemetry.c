#include "ak_crsf_telemetry.h"

#include "ak_crsf.h"
#include "ak_math.h"

/*
 * The frames, byte by byte, with the layout Betaflight writes as the oracle.
 * Nothing here reads hardware, knows a UART or allocates: it is arithmetic and
 * a byte order, which is what makes the whole of it testable on a host and
 * what makes the same code run on both targets.
 */

/* The three periods, in milliseconds, and the order they are tried in. The
 * index into `next_ms` and the counters is this order. */
enum {
    TLM_ATTITUDE = 0,
    TLM_BATTERY,
    TLM_GPS,
    TLM_MODE,
    TLM_SLOTS
};

static const uint32_t tlm_period_ms[TLM_SLOTS] = {
    100u, /* attitude, 10 Hz */
    200u, /* battery, 5 Hz */
    500u, /* gps, 2 Hz */
    1000u /* flight mode, 1 Hz */
};

static void put_u16(uint8_t *at, uint16_t value)
{
    at[0] = (uint8_t)(value >> 8);
    at[1] = (uint8_t)value;
}

static void put_u24(uint8_t *at, uint32_t value)
{
    at[0] = (uint8_t)(value >> 16);
    at[1] = (uint8_t)(value >> 8);
    at[2] = (uint8_t)value;
}

static void put_u32(uint8_t *at, uint32_t value)
{
    at[0] = (uint8_t)(value >> 24);
    at[1] = (uint8_t)(value >> 16);
    at[2] = (uint8_t)(value >> 8);
    at[3] = (uint8_t)value;
}

/* Round half away from zero, without libm: this firmware does not link one. */
static int32_t round_to_int(float value)
{
    return (int32_t)(value >= 0.0f ? value + 0.5f : value - 0.5f);
}

static uint16_t clamp_u16(int32_t value, int32_t low, int32_t high)
{
    if (value < low) {
        value = low;
    }
    if (value > high) {
        value = high;
    }
    return (uint16_t)value;
}

/*
 * One frame: address, length, type, payload, crc.
 *
 * The length byte counts the type, the payload *and* the crc, which is why it
 * is one more than the body. The crc covers the type and payload only - the
 * receiver's own parser (ak_crsf.c) checks the same span, so a frame this file
 * builds is a frame that parser accepts, and that is checked on the host by
 * feeding one back in.
 */
static unsigned build(uint8_t *out, unsigned cap, uint8_t type,
                      const uint8_t *payload, unsigned payload_len)
{
    unsigned body = 1u + payload_len; /* type + payload */
    unsigned total = body + 3u;       /* + address + length + crc */

    if (payload_len > 0xFFu - 2u || cap < total) {
        return 0;
    }
    out[0] = AK_CRSF_ADDRESS_FLIGHT_CONTROLLER;
    out[1] = (uint8_t)(body + 1u);
    out[2] = type;
    for (unsigned i = 0; i < payload_len; i++) {
        out[3u + i] = payload[i];
    }
    out[3u + payload_len] = ak_crsf_crc8(&out[2], (uint8_t)body);
    return total;
}

/*
 * 0x08 Battery sensor, 8 bytes:
 *
 *   uint16 voltage      tenths of a volt, the whole pack
 *   uint16 current      tenths of an amp - zero here, and there is no sensor
 *   uint24 capacity     mAh drawn - zero, and there is no counter
 *   uint8  remaining    percent, from the thresholds the pilot configured
 *
 * The percentage is the same estimate Betaflight sends: a straight line from
 * the ceiling a cell can be detected at down to the critical threshold. It is
 * an estimate of charge from a voltage, and the honest way to send it is with
 * the voltage next to it, which is what the frame does - a handset shows both.
 */
static unsigned battery_frame(const ak_crsf_telemetry_t *in, uint8_t *out,
                              unsigned cap)
{
    uint8_t payload[8];

    put_u16(&payload[0], clamp_u16(round_to_int(in->volts * 10.0f), 0, 65535));
    put_u16(&payload[2], 0u);
    put_u24(&payload[4], 0u);
    payload[7] = in->percent_valid
                     ? (uint8_t)clamp_u16(round_to_int((float)in->percent), 0,
                                          100)
                     : 0u;
    return build(out, cap, AK_CRSF_TYPE_BATTERY, payload, sizeof payload);
}

/*
 * 0x1E Attitude, 6 bytes: three int16, radians * 10000, big endian. The
 * handset does the trigonometry; what it wants from us is the aircraft's
 * attitude in the units the frame defines, and nothing else.
 */
static unsigned attitude_frame(const ak_crsf_telemetry_t *in, uint8_t *out,
                               unsigned cap)
{
    uint8_t payload[6];

    /*
     * The frame holds three angles of at most 3.2767 radians - less than a
     * turn - and the estimator's are carried unwrapped and keep counting past
     * it. A yaw that has been through more than one turn therefore has to be
     * reduced before it is scaled, or the int16 it lands in is a number that
     * means something else: 200 degrees reported as -187, and past 344 degrees
     * as a small positive one. Wrapped, it is the same heading the handset
     * would have drawn anyway. See ak_wrap_pi().
     */
    put_u16(&payload[0],
            (uint16_t)(int16_t)round_to_int(ak_wrap_pi(in->pitch_rad) * 10000.0f));
    put_u16(&payload[2],
            (uint16_t)(int16_t)round_to_int(ak_wrap_pi(in->roll_rad) * 10000.0f));
    put_u16(&payload[4],
            (uint16_t)(int16_t)round_to_int(ak_wrap_pi(in->yaw_rad) * 10000.0f));
    return build(out, cap, AK_CRSF_TYPE_ATTITUDE, payload, sizeof payload);
}

/*
 * 0x02 GPS, 15 bytes:
 *
 *   int32 latitude, longitude      degrees * 1e7, as the module reports them
 *   uint16 ground speed            km/h * 10
 *   uint16 course                  degrees * 10
 *   uint16 altitude                metres above mean sea level, + 1000
 *   uint8  satellites
 *
 * The altitude offset is not a mistake: the field has no sign, so the wire
 * carries "metres + 1000" and a handset subtracts it. Anything below -1000 m
 * or above 4000 m is clamped, which is where Betaflight clamps it too.
 *
 * The speed conversion is the one arithmetic here worth writing down: the
 * module gives millimetres a second and the field wants tenths of a km/h, so
 * 1 mm/s is 0.0036 km/h and 10 mm/s is 0.036 of the field's unit.
 */
static unsigned gps_frame(const ak_crsf_telemetry_t *in, uint8_t *out,
                          unsigned cap)
{
    uint8_t payload[15];
    int32_t speed_units = round_to_int(in->speed_mm_s * 36.0f / 1000.0f);
    int32_t course_units = round_to_int(in->course_deg * 10.0f);
    int32_t alt_units = round_to_int(in->alt_m) + 1000;

    put_u32(&payload[0], (uint32_t)in->lat_e7);
    put_u32(&payload[4], (uint32_t)in->lon_e7);
    put_u16(&payload[8], clamp_u16(speed_units, 0, 65535));
    put_u16(&payload[10], clamp_u16(course_units, 0, 3600));
    put_u16(&payload[12], clamp_u16(alt_units, 0, 5000));
    payload[14] = (uint8_t)(in->satellites > 255u ? 255u : in->satellites);
    return build(out, cap, AK_CRSF_TYPE_GPS, payload, sizeof payload);
}

/* 0x21 Flight mode: the mode string, null terminated. */
static unsigned mode_frame(const ak_crsf_telemetry_t *in, uint8_t *out,
                           unsigned cap)
{
    uint8_t payload[16];
    unsigned n = 0;

    while (in->flight_mode[n] != '\0' && n + 1u < sizeof payload) {
        payload[n] = (uint8_t)in->flight_mode[n];
        n++;
    }
    payload[n++] = 0u;
    return build(out, cap, AK_CRSF_TYPE_FLIGHT_MODE, payload, n);
}

void ak_crsf_tlm_init(ak_crsf_tlm_t *tlm)
{
    tlm->frames = 0;
    tlm->attitude_frames = 0;
    tlm->battery_frames = 0;
    tlm->gps_frames = 0;
    tlm->mode_frames = 0;
    tlm->device_infos = 0;
    tlm->started = 0;
}

unsigned ak_crsf_tlm_next(ak_crsf_tlm_t *tlm,
                          const ak_crsf_telemetry_t *in, uint32_t now_ms,
                          uint8_t *out, unsigned cap)
{
    if (!tlm->started) {
        /* The first tick sends what is already known rather than waiting a
         * period for each type: a handset that has just been switched on shows
         * nothing until these arrive, and a pilot reads them before take-off.
         */
        tlm->started = 1;
        for (unsigned i = 0; i < TLM_SLOTS; i++) {
            tlm->next_ms[i] = now_ms;
        }
    }

    for (unsigned i = 0; i < TLM_SLOTS; i++) {
        unsigned len;

        if ((int32_t)(now_ms - tlm->next_ms[i]) < 0) {
            continue;
        }
        /* Due. If the data behind it is not there yet, push the deadline and
         * let the next type have this tick - a missing barometer must not stop
         * the battery from being reported. */
        switch (i) {
        case TLM_ATTITUDE:
            if (!in->attitude_valid) {
                len = 0;
                break;
            }
            len = attitude_frame(in, out, cap);
            if (len > 0u) {
                tlm->attitude_frames++;
            }
            break;
        case TLM_BATTERY:
            if (!in->volts_valid) {
                len = 0;
                break;
            }
            len = battery_frame(in, out, cap);
            if (len > 0u) {
                tlm->battery_frames++;
            }
            break;
        case TLM_GPS:
            if (!in->fix_valid) {
                len = 0;
                break;
            }
            len = gps_frame(in, out, cap);
            if (len > 0u) {
                tlm->gps_frames++;
            }
            break;
        default:
            if (in->flight_mode == 0) {
                len = 0;
                break;
            }
            len = mode_frame(in, out, cap);
            if (len > 0u) {
                tlm->mode_frames++;
            }
            break;
        }

        tlm->next_ms[i] = now_ms + tlm_period_ms[i];
        if (len > 0u) {
            tlm->frames++;
            return len;
        }
    }
    return 0;
}

unsigned ak_crsf_tlm_device_info(ak_crsf_tlm_t *tlm, const char *name,
                                 uint8_t *out, unsigned cap)
{
    uint8_t payload[AK_CRSF_TLM_MAX_FRAME];
    unsigned room;   /* payload bytes that still fit in the caller's buffer */
    unsigned n = 0;
    unsigned len;

    /* Three bytes of the frame are not payload: the address, the length and
     * the crc. The name is cut to fit what is left rather than the frame being
     * refused - a handset wants to know *something* answered. */
    room = cap >= 3u ? cap - 3u : 0u;
    if (room > sizeof payload) {
        room = sizeof payload;
    }
    if (room < 17u) {
        return 0; /* not enough room for the addresses and the version bytes */
    }

    payload[n++] = AK_CRSF_ADDRESS_RADIO_TRANSMITTER; /* destination */
    payload[n++] = AK_CRSF_ADDRESS_FLIGHT_CONTROLLER; /* origin */

    /* The name is null terminated on the wire, and the twelve zero bytes after
     * it are where a serial number would go. */
    while (name[n - 2u] != '\0' && n + 16u < room) {
        payload[n] = (uint8_t)name[n - 2u];
        n++;
    }
    payload[n++] = 0u;
    for (unsigned i = 0; i < 12u; i++) {
        payload[n++] = 0u;
    }
    /* The parameters this flight controller exposes over CRSF: none. Its own
     * protocol is where the parameter table lives, and saying 255 here would
     * invite a handset to start a conversation this firmware does not answer.
     */
    payload[n++] = 0u;
    payload[n++] = 0x01u; /* parameter version, as the protocol defines it */

    len = build(out, cap, AK_CRSF_TYPE_DEVICE_INFO, payload, n);
    if (len > 0u) {
        tlm->device_infos++;
    }
    return len;
}

const char *ak_crsf_flight_mode(int state_armed, int failsafe, int returning,
                                int managed, int angle_mode)
{
    /* The names a handset has room for: four characters or so, because the
     * field is a short text line on a small screen. They are not this
     * firmware's state names - "on autopilot" is a console sentence, not
     * something a transmitter can show.
     */
    if (failsafe) {
        return "!FS!";
    }
    if (returning || managed) {
        return "RTH";
    }
    if (!state_armed) {
        return "DISARM";
    }
    return angle_mode ? "ANGLE" : "ACRO";
}
