#ifndef AK_SENSORS_AK_GPS_CONFIG_H
#define AK_SENSORS_AK_GPS_CONFIG_H

#include <stdint.h>

/*
 * Telling the GPS what to say.
 *
 * A u-blox module out of the box speaks NMEA at 9600 baud, which this parser
 * cannot read - so without configuring it, a working module looks like a stream
 * of noise. The fix is a UBX-CFG-VALSET frame at boot: ask for NAV-PVT once per
 * navigation solution, and switch off the NMEA sentences and the UBX messages
 * nobody reads. That is the same set INAV sends at the same stage, and the keys
 * below are copied from its header at the pinned revision.
 *
 * The frame itself is built here, in the portable core, so the bytes are
 * testable: VALSET is a four-byte header, then key/value pairs where the key is
 * little-endian and the value is as many bytes as the key's type needs.
 */

#define AK_GPS_VALSET_MAX_KEYS 12
/* 6 (sync, class, id, length) + 4 (version, layer, transaction, reserved) +
 * 12 (two 16-bit rate keys) + 5 per message key + 2 (checksum), with room to
 * spare, because a buffer one byte short fails in a way that looks like
 * something else. */
#define AK_GPS_VALSET_MAX_BYTES \
    (6u + 4u + 12u + AK_GPS_VALSET_MAX_KEYS * 5u + 2u + 8u)

/* The header version a real tool sends. Confirmed rather than recalled: the
 * captured frame in INAV's unit test starts `01 01 00 00` - version 1, RAM
 * layer, no transaction, reserved - and the test for this builder compares
 * against it byte for byte. */
#define AK_GPS_VALSET_VERSION 0x01u

/* Configuration keys (u-blox CFG interface), from INAV's gps_ublox.h. */
#define AK_GPS_CFG_UART1_BAUDRATE      0x40520001u
#define AK_GPS_CFG_RATE_MEAS           0x30210001u
#define AK_GPS_CFG_RATE_NAV            0x30210002u
#define AK_GPS_CFG_MSGOUT_NAV_PVT      0x20910007u
#define AK_GPS_CFG_MSGOUT_NAV_SAT      0x20910016u
#define AK_GPS_CFG_MSGOUT_NAV_POSLLH   0x2091002Au
#define AK_GPS_CFG_MSGOUT_NAV_STATUS   0x2091001Bu
#define AK_GPS_CFG_MSGOUT_NAV_VELNED   0x20910043u
#define AK_GPS_CFG_MSGOUT_NAV_TIMEUTC  0x2091005Cu
#define AK_GPS_CFG_MSGOUT_NAV_SIG      0x20910346u
#define AK_GPS_CFG_MSGOUT_NMEA_GGA     0x209100BBu
#define AK_GPS_CFG_MSGOUT_NMEA_GLL     0x209100CAu
#define AK_GPS_CFG_MSGOUT_NMEA_GSA     0x209100C0u
#define AK_GPS_CFG_MSGOUT_NMEA_RMC     0x209100ACu
#define AK_GPS_CFG_MSGOUT_NMEA_VTG     0x209100B1u

/* Layers: which of the module's stores the settings go into. RAM only, because
 * the firmware sends this at every boot and a module that keeps a setting it
 * was given once is a module nobody can reason about later. */
#define AK_GPS_VALSET_LAYER_RAM 0x01u

/* Builds a VALSET frame asking for NAV-PVT once per solution, no NMEA, and no
 * other UBX messages. Returns the frame length in bytes, or 0 if the buffer is
 * too small. A single U1 value per key, which is what every key here takes. */
unsigned ak_gps_config_frame(uint16_t measurement_ms, uint8_t *frame,
                             unsigned capacity);

/* The default ask: 5 Hz measurements, NAV-PVT every solution. */
#define AK_GPS_MEASUREMENT_MS_DEFAULT 200u

#endif /* AK_SENSORS_AK_GPS_CONFIG_H */
