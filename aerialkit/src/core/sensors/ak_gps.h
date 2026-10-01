#ifndef AK_SENSORS_AK_GPS_H
#define AK_SENSORS_AK_GPS_H

#include <stdint.h>

#include "ak_console.h"

/*
 * GPS, for the fixed wing: a position, a velocity and an idea of how much to
 * trust them.
 *
 * u-blox modules speak UBX, and NAV-PVT is the message that carries everything
 * a navigation loop wants in one frame: fix type, satellites, position,
 * altitude, ground speed, course and the accuracy figures. This parses that,
 * byte at a time, the way a UART delivers it.
 *
 * Every number is kept in the integer unit the protocol uses - 1e-7 degrees,
 * millimetres, millimetres per second, 1e-5 degrees of course - so nothing is
 * lost to a float round trip on the way in, and the console formatter (which
 * has no floating point) can print all of it.
 */

#define AK_GPS_UBX_SYNC1 0xB5u
#define AK_GPS_UBX_SYNC2 0x62u
#define AK_GPS_UBX_NAV_PVT_LENGTH 92u

/*
 * NAV-PVT field offsets, from the u-blox interface description.
 *
 * These are exported because a test derives them from INAV's own ubx_nav_pvt
 * struct instead of restating the numbers, and the simulator builds its frames
 * from that same layout. The first version of this parser had every one of them
 * two bytes too high, and both the unit frame and the simulator had been
 * written to the same wrong number - so the parser, the test and the simulation
 * agreed with each other and none of them agreed with a receiver.
 */
#define AK_GPS_PVT_FIX_TYPE    20u
#define AK_GPS_PVT_FLAGS       21u
#define AK_GPS_PVT_SATELLITES  23u
#define AK_GPS_PVT_LON         24u
#define AK_GPS_PVT_LAT         28u
#define AK_GPS_PVT_ALT_MSL     36u
#define AK_GPS_PVT_HACC        40u
#define AK_GPS_PVT_SPEED       60u
#define AK_GPS_PVT_COURSE      64u
#define AK_GPS_PVT_PDOP        76u

#define AK_GPS_MAX_PAYLOAD 100u

/* A frame that stops part way is abandoned after this much silence. Bytes
 * inside a frame arrive back to back, so a gap this long means the sender
 * stopped - a receiver reboot, a pulled wire, a UART that dropped bytes.
 * Without it the parser swallows the start of the next frame as the end of the
 * last one, and one dropped byte costs two frames. */
#define AK_GPS_GAP_MS 10u

/* Fix types, as u-blox numbers them, kept rather than translated: a translation
 * layer here would only hide which one the receiver reported. */
enum {
    AK_GPS_FIX_NONE = 0,
    AK_GPS_FIX_DEAD_RECKONING = 1,
    AK_GPS_FIX_2D = 2,
    AK_GPS_FIX_3D = 3,
    AK_GPS_FIX_GNSS_DR = 4,
    AK_GPS_FIX_TIME_ONLY = 5,
};

typedef struct {
    uint8_t  fix_type;
    uint8_t  fix_ok;      /* the receiver's own "this fix is usable" bit */
    uint8_t  satellites;
    int32_t  lat_e7;      /* degrees * 1e7 */
    int32_t  lon_e7;
    int32_t  alt_msl_mm;
    int32_t  speed_mm_s;  /* ground speed */
    int32_t  course_e5;   /* heading of motion, degrees * 1e5 */
    uint16_t pdop_e2;     /* position DOP * 100 */
    uint32_t hacc_mm;
    uint32_t last_fix_ms; /* when this message arrived */
} ak_gps_fix_t;

typedef struct {
    uint8_t  buffer[AK_GPS_MAX_PAYLOAD];
    uint8_t  held;
    uint8_t  state;
    uint16_t expected;
    uint8_t  checksum_a;
    uint8_t  checksum_b;
    uint8_t  payload_a;
    uint8_t  payload_b;
    uint8_t  msg_class;
    uint8_t  msg_id;

    ak_gps_fix_t fix;
    uint32_t messages;        /* frames that passed the checksum */
    uint32_t fix_messages;    /* NAV-PVT messages decoded */
    uint32_t checksum_errors;
    uint32_t bad_length;
    uint32_t other_messages;  /* valid UBX we do not use */
    uint32_t bytes;
    uint32_t last_byte_ms;
    int      have_fix;
    /* How many satellites a fix needs before it is worth navigating on. INAV's
     * rule and INAV's reason: `gpsMinSats` defaults to six because "some GPS
     * receivers appeared to be very inaccurate with low satellite count". Ours
     * used to have no floor at all and to accept a 2D fix, which has no
     * altitude - and the fused height takes its absolute reference from the
     * first fix it sees. */
    uint32_t min_sats;
} ak_gps_t;

void ak_gps_init(ak_gps_t *gps);

/* One byte from the port. Returns 1 when that byte completed a NAV-PVT frame,
 * in which case `fix` is fresh and stamped with now_ms. */
int ak_gps_feed(ak_gps_t *gps, uint8_t byte, uint32_t now_ms);

/* A fix is only useful if it is recent and the receiver believed it. */
int ak_gps_fix_valid(const ak_gps_t *gps, uint32_t now_ms, uint32_t max_age_ms);

/* The u-blox checksum, exposed because it is the one piece of the framing that
 * can be checked against a captured frame. */
void ak_gps_ubx_checksum(const uint8_t *data, uint16_t length, uint8_t *a,
                         uint8_t *b);

void ak_gps_report(const ak_gps_t *gps, ak_printf_fn out);

#endif /* AK_SENSORS_AK_GPS_H */
