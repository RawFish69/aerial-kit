#include "ak_gps_config.h"

#include "ak_gps.h"

typedef struct {
    uint32_t key;
    uint8_t  value;
} valset_u1_t;

/* One byte per key: everything here is a U1 message rate or a layer flag. The
 * frame layout was checked against a real captured VALSET frame - INAV's unit
 * test carries one, and the test for this builder compares our bytes to it. */
static const valset_u1_t messages[] = {
    { AK_GPS_CFG_MSGOUT_NAV_PVT, 1 },   /* the one we read */
    { AK_GPS_CFG_MSGOUT_NAV_SAT, 0 },
    { AK_GPS_CFG_MSGOUT_NAV_SIG, 0 },
    { AK_GPS_CFG_MSGOUT_NAV_POSLLH, 0 },
    { AK_GPS_CFG_MSGOUT_NAV_STATUS, 0 },
    { AK_GPS_CFG_MSGOUT_NAV_VELNED, 0 },
    { AK_GPS_CFG_MSGOUT_NAV_TIMEUTC, 0 },
    { AK_GPS_CFG_MSGOUT_NMEA_GGA, 0 },
    { AK_GPS_CFG_MSGOUT_NMEA_GLL, 0 },
    { AK_GPS_CFG_MSGOUT_NMEA_GSA, 0 },
    { AK_GPS_CFG_MSGOUT_NMEA_RMC, 0 },
    { AK_GPS_CFG_MSGOUT_NMEA_VTG, 0 },
};

static unsigned append_u1(uint8_t *frame, unsigned at, uint32_t key,
                          uint8_t value)
{
    frame[at + 0] = (uint8_t)(key & 0xFFu);
    frame[at + 1] = (uint8_t)((key >> 8) & 0xFFu);
    frame[at + 2] = (uint8_t)((key >> 16) & 0xFFu);
    frame[at + 3] = (uint8_t)((key >> 24) & 0xFFu);
    frame[at + 4] = value;
    return at + 5u;
}

static unsigned append_u2(uint8_t *frame, unsigned at, uint32_t key,
                          uint16_t value)
{
    frame[at + 0] = (uint8_t)(key & 0xFFu);
    frame[at + 1] = (uint8_t)((key >> 8) & 0xFFu);
    frame[at + 2] = (uint8_t)((key >> 16) & 0xFFu);
    frame[at + 3] = (uint8_t)((key >> 24) & 0xFFu);
    frame[at + 4] = (uint8_t)(value & 0xFFu);
    frame[at + 5] = (uint8_t)((value >> 8) & 0xFFu);
    return at + 6u;
}

unsigned ak_gps_config_frame(uint16_t measurement_ms, uint8_t *frame,
                             unsigned capacity)
{
    unsigned keys = (unsigned)(sizeof messages / sizeof messages[0]);
    unsigned payload = 4u + keys * 5u + 12u; /* header, rates, message rates */

    if (capacity < payload + 8u) {
        return 0;
    }

    frame[0] = AK_GPS_UBX_SYNC1;
    frame[1] = AK_GPS_UBX_SYNC2;
    frame[2] = 0x06; /* class CFG */
    frame[3] = 0x8A; /* id VALSET */
    frame[4] = (uint8_t)(payload & 0xFFu);
    frame[5] = (uint8_t)((payload >> 8) & 0xFFu);

    unsigned at = 6;
    frame[at++] = AK_GPS_VALSET_VERSION;
    frame[at++] = AK_GPS_VALSET_LAYER_RAM;
    frame[at++] = 0; /* transaction */
    frame[at++] = 0; /* reserved */

    /* How often to measure and how many measurements per navigation solution.
     * One means a fix every measurement. */
    at = append_u2(frame, at, AK_GPS_CFG_RATE_MEAS, measurement_ms);
    at = append_u2(frame, at, AK_GPS_CFG_RATE_NAV, 1);

    for (unsigned i = 0; i < keys; i++) {
        at = append_u1(frame, at, messages[i].key, messages[i].value);
    }

    uint8_t a;
    uint8_t b;
    ak_gps_ubx_checksum(&frame[2], (uint16_t)(at - 2u), &a, &b);
    frame[at++] = a;
    frame[at++] = b;
    return at;
}
