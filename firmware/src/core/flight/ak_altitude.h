#ifndef AK_FLIGHT_AK_ALTITUDE_H
#define AK_FLIGHT_AK_ALTITUDE_H

#include <stdint.h>

/*
 * Height above the take-off point, from a barometer and a GPS.
 *
 * The two measurements fail in opposite directions and that is the whole
 * reason to have both. A barometer is fast - tens of centimetres, tens of times
 * a second - and its reference drifts: the weather, and the inside of a
 * fuselage warming up, move the pressure it reads with no height involved. A
 * GPS altitude is slow and noisy to a few metres, and it is an *absolute*
 * height, so it does not drift at all.
 *
 * So: the barometer says what the aircraft is doing right now, the GPS says
 * where it has ended up, and a slow leak from the GPS into the barometer's
 * reference keeps the two from disagreeing over a long flight. That is the same
 * complementary filter as the attitude estimate, with the same time constant
 * argument - one number for where the trust crosses over.
 *
 * Both references are captured while the aircraft is standing on the ground:
 * the pressure it is sitting in, and the GPS altitude it is sitting at. That is
 * what makes every number here a *change*, which is what a pilot and a
 * navigator both want.
 */

typedef struct {
    /* How much of the GPS's disagreement to take per second, and how much of
     * the barometer's movement to believe immediately. */
    float gps_correction_per_s;

    /* Each sensor anchors itself, and neither needs the other to be useful: a
     * barometer with no GPS still knows how far it has climbed since take-off,
     * and a GPS with no barometer is the answer this firmware flew on before. */
    float reference_pa;      /* the pressure on the ground */
    float reference_msl_m;   /* the GPS altitude on the ground */
    float offset_m;          /* the leak: what the gps says minus what we say */
    float height_m;          /* the answer: above the take-off point */

    int   have_baro_reference;
    int   have_gps_reference;
    int   have_baro;
    int   have_gps;
    /* A barometer that has stopped answering: the caller says so, because only
     * it knows whether the part is still being polled successfully. The height
     * then comes from the GPS, which is the same answer a board with no
     * barometer fitted gives - and a sample puts the barometer back. */
    int   baro_lost;
    float gps_height_m;      /* the last GPS height above the reference */
    uint32_t baro_samples;
    uint32_t gps_samples;
} ak_altitude_t;

void ak_altitude_init(ak_altitude_t *alt);

/* The ground pressure, captured once while disarmed and still. A second call
 * is ignored: a reference taken in the air is a height nobody wants. */
void ak_altitude_set_baro_reference(ak_altitude_t *alt, float pressure_pa);

/* The ground altitude, the same way and for the same reason. */
void ak_altitude_set_gps_reference(ak_altitude_t *alt, float gps_msl_m);

int ak_altitude_have_reference(const ak_altitude_t *alt);

/* One barometer reading, in pascals. */
void ak_altitude_baro(ak_altitude_t *alt, float pressure_pa);

/* One GPS altitude, in metres above mean sea level, and how long since the
 * last one: the leak is applied per second, not per sample, so a GPS that
 * arrives at 5 Hz and a barometer at 32 Hz do not fight over the gain. */
void ak_altitude_gps(ak_altitude_t *alt, float gps_msl_m, float dt_s);

/* The barometer has stopped answering - its reads are failing, or it has been
 * gone long enough that the last sample means nothing. Only the caller can
 * know that, which is why it is told rather than guessed: from here the height
 * comes from the GPS, and the next barometer sample puts it back. */
void ak_altitude_baro_lost(ak_altitude_t *alt);

/* Metres above the take-off point, or 0 before there is anything to say. */
float ak_altitude_height_m(const ak_altitude_t *alt);

#endif /* AK_FLIGHT_AK_ALTITUDE_H */
