#ifndef AK_FLIGHT_RC_H
#define AK_FLIGHT_RC_H

#include "ak_types.h"

/*
 * Raw receiver channels in, sticks and switches out.
 *
 * Counts are CRC/CRSF-shaped by default (172..1811 with 992 in the middle,
 * which is what the twin-wings receiver carries), so nothing here assumes
 * milliseconds. A receiver that speaks SBUS or PWM gets its own calibration in
 * the config rather than its own decoder.
 */

typedef struct {
    /* uint32_t rather than uint16_t so these can be parameters: the table
     * points at plain integers, and a type that only exists as a wire format
     * should not leak into the firmware's configuration. */
    uint32_t min;
    uint32_t mid;
    uint32_t max;
    float    deadband;        /* fraction of full stick travel */
    uint32_t mode_threshold;  /* above this, the mode channel selects angle */
    uint32_t arm_threshold;   /* above this, the arm channel requests armed */
    /*
     * Which receiver channel arms and which selects the mode, 1-based the way
     * a transmitter labels them, so `arm_channel 5` is "AUX1".
     *
     * They were fixed at 6 and 5 until 2026-10-05, and that order fits neither
     * radio the fixed wing flies: ExpressLRS puts the arm switch on CH5 (AUX1)
     * by convention, and the custom ESP-NOW link sends its arm button there
     * too (aerial-kit firmware/legacy/espnow, `rcChannels[4]`). With the old
     * fixed order, either radio's arm switch toggled angle mode and nothing
     * armed. The defaults are still 6 and 5, so no saved configuration means
     * something different; the parameters are what a pilot changes. Channels
     * 1-4 are the sticks (AETR), so both are held to 5..AK_RC_CHANNELS, and a
     * value outside that - or the two set equal - is read as "no arm request"
     * and "rate mode" rather than guessed at.
     */
    uint32_t arm_channel;
    uint32_t mode_channel;
} ak_rc_config_t;

/* The aux channels a switch may be on: the four sticks come first. */
#define AK_RC_FIRST_AUX_CHANNEL 5u

typedef struct {
    float roll;       /* -1..1 */
    float pitch;      /* -1..1 */
    float yaw;        /* -1..1 */
    float throttle;   /* 0..1 */
    int   angle_mode; /* 1 = angle/stabilized, 0 = rate */
    int   arm_request;
} ak_rc_command_t;

void ak_rc_default_config(ak_rc_config_t *cfg);

/*
 * Measuring where a receiver's sticks actually sit.
 *
 * The defaults are what CRSF is specified to send, and a real receiver is
 * usually a few counts away from them - which the flight core reads as a
 * permanent stick offset, so the aircraft drifts on the bench and nobody can
 * tell that from a bad accelerometer. Centre calibration is the half that
 * matters for a first flight; the endpoints are close to spec and the stick
 * travel is what tells you they are not.
 */
typedef struct {
    uint32_t sum[AK_RC_CHANNELS];
    uint32_t samples;
    uint32_t wanted;
    uint32_t first[AK_RC_CHANNELS]; /* the frame it started from, for a report */
    int      running;
    int      done;
} ak_rc_cal_t;

void ak_rc_cal_init(ak_rc_cal_t *cal, uint32_t wanted);
void ak_rc_cal_start(ak_rc_cal_t *cal);
void ak_rc_cal_feed(ak_rc_cal_t *cal, const ak_rc_input_t *input);

/* Writes the measured centres into the configuration and returns how far each
 * of the four stick channels was from where the configuration thought it was.
 * Returns 0 if no calibration was done. */
int ak_rc_cal_apply(ak_rc_cal_t *cal, ak_rc_config_t *cfg,
                    int32_t offset[AK_RC_CHANNELS]);

/* Returns 0 and leaves the command zeroed when the frame cannot be used. */
int ak_rc_decode(const ak_rc_input_t *input, const ak_rc_config_t *cfg,
                 ak_rc_command_t *out);

#endif /* AK_FLIGHT_RC_H */
