#ifndef AK_FLIGHT_CRSF_TELEMETRY_H
#define AK_FLIGHT_CRSF_TELEMETRY_H

#include <stdint.h>

/*
 * CRSF telemetry: what the handset shows while the aircraft flies.
 *
 * The receiver half of CRSF is in ak_crsf.c - bytes in, sticks out. This is
 * the other direction, and it exists for one reason that is easy to state: a
 * pilot cannot fly a pack they cannot see, and on the wing the only link to
 * the pilot is the receiver's own wire. The F405's board file has said "PA9,
 * wired for telemetry later" since it was written; this is the later.
 *
 * The frames are the ones a handset and an ELRS/Crossfire receiver already
 * understand, and each one is built to the byte layout Betaflight writes
 * (oracle: `upstream/betaflight/src/main/telemetry/crsf.c`), which is in turn
 * what the CRSF protocol document says:
 *
 *   [0xC8][len][type][payload ...][crc8]
 *
 *   len counts the type, the payload and the crc. The crc is CRC-8/DVB-S2
 *   over the type and the payload - *not* over the address or the length -
 *   which is the same crc the receive side already checks, so there is one
 *   implementation of it in the firmware and this file does not get its own.
 *   0xC8 is the flight controller's own address, which is also the byte a
 *   receiver uses to sync.
 *
 * What is sent, and how often:
 *
 *   0x1E attitude     10 Hz   the estimator's roll, pitch and yaw
 *   0x08 battery       5 Hz   pack volts, and the percentage the thresholds imply
 *   0x02 gps           2 Hz   position, ground speed, course, altitude, sats
 *   0x21 flight mode   1 Hz   "ANGLE", "ACRO", "RTH", "!FS!" ...
 *   0x29 device info   on     what a device ping (0x28) asks for
 *
 * The rates are the point of the scheduler below rather than four calls in the
 * flight loop: at 420000 baud there is room for far more than this, but a
 * receiver forwards telemetry to the handset on a much slower schedule, and a
 * flight controller that floods it makes the handset drop the frames that
 * matter.
 *
 * What it does *not* send: current, mAh used (there is no current sensor on
 * any board here, and a zero is honest only because the frame says so), and
 * link statistics (that is the receiver's own frame - it is the one thing on
 * this wire the flight controller does not own). Parameter-over-CRSF (the
 * handset's own configuration menu, frame types 0x2B-0x2D) is not implemented:
 * AerialKit configures itself over its own protocol, and two ways to write the
 * parameter table is one way too many.
 */

/* The two addresses on this wire. The others (a GPS's, a current sensor's) are
 * not ours to use. */
#define AK_CRSF_ADDRESS_BROADCAST          0x00u
#define AK_CRSF_ADDRESS_FLIGHT_CONTROLLER  0xC8u
#define AK_CRSF_ADDRESS_RADIO_TRANSMITTER  0xEAu

#define AK_CRSF_TYPE_GPS         0x02u
#define AK_CRSF_TYPE_BATTERY     0x08u
#define AK_CRSF_TYPE_ATTITUDE    0x1Eu
#define AK_CRSF_TYPE_FLIGHT_MODE 0x21u
#define AK_CRSF_TYPE_DEVICE_PING 0x28u
#define AK_CRSF_TYPE_DEVICE_INFO 0x29u

/* The longest frame this file builds is the device info reply: type, two
 * addresses, a name, twelve zero bytes and two version bytes. */
#define AK_CRSF_TLM_MAX_FRAME 48u

/* How often the flight loop should ask. Each frame type has its own period
 * inside the scheduler; this is the tick it is asked on. */
#define AK_CRSF_TLM_TICK_MS 20u

typedef struct {
    /* Everything is optional, and the frame that needs it is simply not sent
     * while it is missing: a handset showing nothing is honest, and a handset
     * showing zero volts is a report that the pack is empty. */
    int      volts_valid;    float volts;
    int      percent_valid;  int percent;      /* 0..100 */
    int      attitude_valid; float roll_rad, pitch_rad, yaw_rad;
    int      fix_valid;
    int32_t  lat_e7, lon_e7; /* degrees * 1e7, as the gps hands them over */
    float    alt_m;          /* metres above mean sea level */
    float    speed_mm_s;
    float    course_deg;
    unsigned satellites;
    const char *flight_mode; /* null while there is nothing to say */
} ak_crsf_telemetry_t;

typedef struct {
    uint32_t frames;
    uint32_t attitude_frames, battery_frames, gps_frames, mode_frames;
    uint32_t device_infos;
    uint32_t next_ms[4];
    int      started;
} ak_crsf_tlm_t;

void ak_crsf_tlm_init(ak_crsf_tlm_t *tlm);

/*
 * The next frame that is due, written into `out` (at most `cap` bytes).
 * Returns its length, or 0 when nothing is due - which is most calls, because
 * the flight loop runs at a kilohertz and the handset needs a few frames a
 * second.
 */
unsigned ak_crsf_tlm_next(ak_crsf_tlm_t *tlm,
                          const ak_crsf_telemetry_t *in, uint32_t now_ms,
                          uint8_t *out, unsigned cap);

/* The reply to a device ping (0x28): the flight controller's name, the
 * addresses, and the two bytes that say how many parameters it has over CRSF,
 * which is zero - see the note at the top about 0x2B. */
unsigned ak_crsf_tlm_device_info(ak_crsf_tlm_t *tlm, const char *name,
                                 uint8_t *out, unsigned cap);

/* The CRSF flight-mode string for a flight state and whether the pilot's
 * sticks are commanding an angle or a rate. Kept here rather than in the
 * flight core: it is a *wire* format, and the strings a handset shows are not
 * the names this firmware uses for its own states. */
const char *ak_crsf_flight_mode(int state_armed, int failsafe, int returning,
                                int managed, int angle_mode);

#endif /* AK_FLIGHT_CRSF_TELEMETRY_H */
