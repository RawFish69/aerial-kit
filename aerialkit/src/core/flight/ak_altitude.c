#include "ak_altitude.h"

#include "ak_baro.h"
#include "ak_math.h"

/*
 * Two sensors, one height.
 *
 * The barometer arrives at 32 Hz and the GPS at 5, and the two are worth
 * trusting for different things, so neither is simply preferred:
 *
 *   - a barometer reading moves the *estimate* by as much as it moved, so the
 *     fast part of the answer is never delayed by a filter;
 *   - a GPS reading moves the *offset* between the two, slowly, which is what
 *     stops a barometer's drift from becoming the aircraft's idea of the
 *     ground.
 *
 * Read the other way round, this is the same complementary filter as the
 * attitude estimate: one path that is fast and drifts, one that is slow and
 * absolute, and a time constant where the trust crosses over.
 *
 * With no barometer it degenerates to "the GPS, as it arrives" - which is what
 * this firmware flew on before there was one, and is the behaviour a board with
 * no barometer fitted should have. With no GPS it is the barometer's own
 * change since take-off, which drifts with the weather but holds a *relative*
 * height perfectly well.
 */

void ak_altitude_init(ak_altitude_t *alt)
{
    /*
     * Where the two errors cross.
     *
     * Leaking the GPS in with a time constant tau leaves a steady error of
     * (barometer drift rate x tau) and a noise error of about
     * (gps noise x sqrt(2 / (tau x gps rate))). The first grows with tau and the
     * second shrinks, so the sum has a minimum: for a few centimetres per
     * second of drift and a few metres of GPS noise at five hertz, it is around
     * fifteen seconds. That is a fourteenth of the disagreement per second,
     * which on a five-minute flight holds the GPS's noise to well under a metre
     * while following the barometer's drift instead of accumulating it.
     */
    alt->gps_correction_per_s = 0.07f;
    alt->reference_pa = 0.0f;
    alt->reference_msl_m = 0.0f;
    alt->offset_m = 0.0f;
    alt->height_m = 0.0f;
    alt->have_baro_reference = 0;
    alt->have_gps_reference = 0;
    alt->have_baro = 0;
    alt->have_gps = 0;
    alt->baro_lost = 0;
    alt->gps_height_m = 0.0f;
    alt->baro_samples = 0;
    alt->gps_samples = 0;
}

void ak_altitude_set_baro_reference(ak_altitude_t *alt, float pressure_pa)
{
    if (alt->have_baro_reference || pressure_pa <= 0.0f) {
        return;
    }
    alt->reference_pa = pressure_pa;
    alt->offset_m = 0.0f;
    alt->height_m = 0.0f;
    alt->have_baro_reference = 1;
}

void ak_altitude_set_gps_reference(ak_altitude_t *alt, float gps_msl_m)
{
    if (alt->have_gps_reference) {
        return;
    }
    alt->reference_msl_m = gps_msl_m;
    if (!alt->have_baro_reference) {
        alt->height_m = 0.0f;
    }
    alt->have_gps_reference = 1;
}

int ak_altitude_have_reference(const ak_altitude_t *alt)
{
    return alt->have_baro_reference || alt->have_gps_reference;
}

void ak_altitude_baro(ak_altitude_t *alt, float pressure_pa)
{
    if (pressure_pa <= 0.0f) {
        return;
    }
    alt->baro_samples++;
    alt->have_baro = 1;

    if (!alt->have_baro_reference) {
        /* No ground reference yet, so there is no height - but the sample is
         * counted, and the caller captures the reference from the first one it
         * sees while the aircraft is disarmed. */
        return;
    }

    float baro_height_m = ak_baro_altitude_m(pressure_pa, alt->reference_pa);

    /*
     * A part that is answering again after being silent.
     *
     * The leak has been moving the *offset* the whole time the barometer was
     * gone - that is what made the frozen height crawl towards the GPS - so
     * adding the first good sample to that offset gives a height nobody
     * measured: measured in the host test, a return to 20 m of pressure came
     * back as -60 m. Re-seat the offset instead, so the height the GPS has
     * been carrying is where the barometer picks up, and the barometer's own
     * movement takes over from there without a jump.
     */
    if (alt->baro_lost) {
        if (alt->have_gps_reference) {
            alt->offset_m = alt->gps_height_m - baro_height_m;
        }
        alt->baro_lost = 0;
    }

    /* What the barometer says the height is, plus the correction the GPS has
     * talked us into. The barometer's own movement goes into the answer
     * immediately: that is the half of this filter that is fast. */
    alt->height_m = baro_height_m + alt->offset_m;
}

void ak_altitude_gps(ak_altitude_t *alt, float gps_msl_m, float dt_s)
{
    alt->gps_samples++;
    alt->have_gps = 1;

    if (!alt->have_gps_reference) {
        return;
    }

    /* The GPS, expressed the way the barometer is: above the take-off point.
     * An absolute altitude is only useful against the one it started at. */
    float gps_height_m = gps_msl_m - alt->reference_msl_m;

    /* Kept whether or not the barometer is answering, because it is the whole
     * answer when the barometer has stopped. */
    alt->gps_height_m = gps_height_m;

    if (!alt->have_baro || !alt->have_baro_reference) {
        /* No barometer: the GPS is the whole answer, with no filter in the way
         * of it. */
        alt->height_m = gps_height_m;
        return;
    }

    /* How wrong we are, according to the one sensor that cannot drift. The
     * correction is a leak rather than a step, because a few metres of GPS
     * noise stepping into the height every 200 ms is a height that twitches. */
    float error = gps_height_m - alt->height_m;
    float correction = error * alt->gps_correction_per_s * dt_s;
    alt->offset_m += correction;
    alt->height_m += correction;
}

float ak_altitude_height_m(const ak_altitude_t *alt)
{
    /*
     * A barometer that has stopped answering is not a barometer: the height
     * comes from the GPS, relative to the same ground reference, which is what
     * this module gives on a board with no barometer fitted. Without a GPS
     * reference either there is nothing left to say, and the last height is
     * the honest answer - the caller is expected to know it is stale (see the
     * landing rule in main.c, which refuses to decide on a height that is not
     * being measured).
     */
    if (alt->baro_lost && alt->have_gps_reference) {
        return alt->gps_height_m;
    }
    return alt->height_m;
}

void ak_altitude_baro_lost(ak_altitude_t *alt)
{
    alt->baro_lost = 1;
}
