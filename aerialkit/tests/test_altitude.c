/*
 * The altitude estimate.
 *
 * The property a complementary filter is supposed to have is not "it runs": it
 * is that the answer is *better than either input*. So this flies the same
 * synthetic flight past all three - a barometer that drifts, a GPS that is
 * noisy, and the filter - and compares each against a height both of them are
 * trying to measure. If somebody changes the leak rate, or swaps the two
 * around, one of these numbers gets worse and says so.
 */

#include <math.h>
#include <stdio.h>

#include "ak_altitude.h"
#include "ak_baro.h"
#include "tests.h"

#define HOME_ALT_MSL_M 120.0f
#define SEA_LEVEL_PA   101325.0f

/* The flight: a climb, then a hold that breathes a few metres, over two
 * minutes. Everything below is a function of this. */
static float truth_m(float t)
{
    if (t < 20.0f) {
        return 3.0f * t; /* 3 m/s up to 60 m */
    }
    return 60.0f + 2.0f * sinf((t - 20.0f) * 0.2f);
}

/* The pressure the standard atmosphere has at a height, with the weather
 * moving the reference: three centimetres a second of apparent climb - under
 * two metres a minute - is the sort of drift a fuselage warming in the sun
 * produces, and over a five-minute flight it is metres. */
static float baro_pressure(float t)
{
    float apparent = truth_m(t) + 0.03f * t;
    return (float)((double)SEA_LEVEL_PA *
                   pow(1.0 - (double)apparent / 44330.0, 5.25500));
}

/* The GPS, in metres above sea level: the truth, a few metres of noise that is
 * deterministic (a test that fails one run in five is not a test), and a
 * five-hertz update. */
static float gps_noise(float t)
{
    return 2.5f * sinf(t * 2.7f) + 1.5f * sinf(t * 11.3f);
}

typedef struct {
    float fused_rms;
    float gps_rms;
    float baro_rms;
} errors_t;

static errors_t fly(int with_baro, int with_gps)
{
    ak_altitude_t alt;
    errors_t out = { 0.0f, 0.0f, 0.0f };
    float fused2 = 0.0f, gps2 = 0.0f, baro2 = 0.0f;
    int counted = 0;
    float last_gps_t = 0.0f;

    ak_altitude_init(&alt);
    /* The aircraft on the ground, still, before anything else happens: that is
     * where both references come from. */
    ak_altitude_set_baro_reference(&alt, baro_pressure(0.0f));
    ak_altitude_set_gps_reference(&alt, HOME_ALT_MSL_M);

    for (int step = 0; step < 300 * 32; step++) {
        float t = (float)step / 32.0f;

        if (with_baro) {
            ak_altitude_baro(&alt, baro_pressure(t));
        }
        if (with_gps && ((step % 6) == 0)) {
            ak_altitude_gps(&alt, HOME_ALT_MSL_M + truth_m(t) + gps_noise(t),
                            t - last_gps_t);
            last_gps_t = t;
        }
        if (t < 30.0f) {
            continue; /* let the filter settle before the marks are counted */
        }

        float reference = truth_m(t);
        float fused = ak_altitude_height_m(&alt) - reference;
        float baro = ak_baro_altitude_m(baro_pressure(t), alt.reference_pa) -
                     reference;
        float gps = truth_m(t) + gps_noise(t) - reference;

        fused2 += fused * fused;
        baro2 += baro * baro;
        gps2 += gps * gps;
        counted++;
    }

    out.fused_rms = sqrtf(fused2 / (float)counted);
    out.gps_rms = sqrtf(gps2 / (float)counted);
    out.baro_rms = sqrtf(baro2 / (float)counted);
    return out;
}

static void test_fusion_beats_both(void)
{
    errors_t errors = fly(1, 1);

    printf("        rms error: fused %.2f m, gps %.2f m, baro %.2f m\n",
           (double)errors.fused_rms, (double)errors.gps_rms,
           (double)errors.baro_rms);

    expect("the filter is better than the gps alone",
           errors.fused_rms < errors.gps_rms * 0.6f);
    expect("and much better than the barometer alone",
           errors.fused_rms < errors.baro_rms * 0.3f);
    expect("and it is a height, not a number that happens to be small",
           errors.fused_rms < 1.5f);
}

static void test_one_sensor_alone(void)
{
    /* No barometer: the GPS is the whole answer, unfiltered, which is what
     * this firmware flew on before there was one. */
    errors_t gps_only = fly(0, 1);
    expect("without a barometer the answer is the gps, and only as good",
           fabsf(gps_only.fused_rms - gps_only.gps_rms) < 0.001f);

    /* No GPS: the barometer's own change since take-off, which drifts and does
     * not pretend otherwise. */
    errors_t baro_only = fly(1, 0);
    expect("without a gps the answer is the barometer's own change",
           fabsf(baro_only.fused_rms - baro_only.baro_rms) < 0.001f);
    expect("which drifts, and that is what the gps is for",
           baro_only.fused_rms > 1.0f);
}

static void test_the_reference(void)
{
    ak_altitude_t alt;

    ak_altitude_init(&alt);
    expect("there is no height before a reference",
           !ak_altitude_have_reference(&alt));

    ak_altitude_baro(&alt, SEA_LEVEL_PA);
    expect("and a barometer sample before one says nothing",
           ak_altitude_height_m(&alt) == 0.0f && alt.baro_samples == 1);

    ak_altitude_set_baro_reference(&alt, SEA_LEVEL_PA);
    ak_altitude_set_gps_reference(&alt, HOME_ALT_MSL_M);
    expect("the reference is the ground",
           ak_altitude_have_reference(&alt) &&
           ak_altitude_height_m(&alt) == 0.0f);

    /* The next sample is a height above it. A hundred metres of pressure is a
     * hundred metres of height, to well under a metre. */
    float high = (float)((double)SEA_LEVEL_PA *
                         pow(1.0 - 100.0 / 44330.0, 5.25500));
    ak_altitude_baro(&alt, high);
    expect("and a hundred metres of pressure is a hundred metres",
           fabsf(ak_altitude_height_m(&alt) - 100.0f) < 0.5f);

    /* A reference taken in the air would make every later height a lie, so a
     * second one is ignored. */
    ak_altitude_set_baro_reference(&alt, high);
    ak_altitude_set_gps_reference(&alt, HOME_ALT_MSL_M + 100.0f);
    expect("a second reference is refused rather than believed",
           fabsf(ak_altitude_height_m(&alt) - 100.0f) < 0.5f);

    /* Nonsense in: a pressure of zero is not a height. */
    ak_altitude_baro(&alt, 0.0f);
    expect("and a pressure of zero does not move it",
           fabsf(ak_altitude_height_m(&alt) - 100.0f) < 0.5f);
}

/*
 * The barometer that stops answering.
 *
 * A part that dies in flight leaves the estimate *still*, and still is what a
 * landing looks like: measured in the loop, with the barometer failing as a
 * return's descent began, the navigator's height froze at twenty metres, the
 * landing rule never fired - it needs the hover height first - and the aircraft
 * sat on the ground with its motors still running.
 *
 * The answer is the one this module already gives a board with no barometer at
 * all: the GPS, relative to the same ground reference. The caller is the one
 * who knows the part has stopped, so it says so, and the next good sample puts
 * it back.
 */
static void test_the_barometer_stops_answering(void)
{
    ak_altitude_t alt;
    float high = (float)((double)SEA_LEVEL_PA *
                         pow(1.0 - 100.0 / 44330.0, 5.25500));
    float low = (float)((double)SEA_LEVEL_PA *
                        pow(1.0 - 20.0 / 44330.0, 5.25500));

    ak_altitude_init(&alt);
    ak_altitude_set_baro_reference(&alt, SEA_LEVEL_PA);
    ak_altitude_set_gps_reference(&alt, HOME_ALT_MSL_M);

    /* Flying at a hundred metres, with the barometer and the GPS agreeing. */
    ak_altitude_baro(&alt, high);
    ak_altitude_gps(&alt, HOME_ALT_MSL_M + 100.0f, 0.2f);
    expect("a hundred metres of pressure and a hundred metres of gps agree",
           fabsf(ak_altitude_height_m(&alt) - 100.0f) < 0.5f);

    /* The part goes quiet and the aircraft descends. The GPS follows it, and
     * the height has to follow the GPS - a frozen height is what the landing
     * rule cannot be allowed to see. It descends to twenty metres, where the
     * barometer will answer again, so the two can be compared at the moment of
     * recovery. */
    ak_altitude_baro_lost(&alt);
    for (int i = 0; i < 25; i++) {
        float msl = HOME_ALT_MSL_M + 100.0f - 3.2f * (float)(i + 1);

        ak_altitude_gps(&alt, msl, 0.2f);
    }
    expect("with the barometer silent the gps is the height",
           fabsf(ak_altitude_height_m(&alt) - 20.0f) < 1.0f);

    /* While it is silent, nothing the barometer would have said matters - and
     * the next sample is what puts it back, *without a jump*: the offset the
     * leak had been dragging is re-seated on the GPS's answer, so the
     * barometer picks up where the estimate already is. */
    ak_altitude_baro(&alt, low);
    expect("and the next barometer sample takes it back where it was",
           fabsf(ak_altitude_height_m(&alt) - 20.0f) < 0.5f);

    /* A barometer that is silent with no GPS either leaves the last height,
     * because there is nothing else to say: the caller is the one that has to
     * refuse to decide anything on it (the landing rule does, by requiring the
     * height to be live). */
    ak_altitude_t no_gps;
    ak_altitude_init(&no_gps);
    ak_altitude_set_baro_reference(&no_gps, SEA_LEVEL_PA);
    ak_altitude_baro(&no_gps, high);
    ak_altitude_baro_lost(&no_gps);
    expect("with no gps to fall back on, the last height is what there is",
           fabsf(ak_altitude_height_m(&no_gps) - 100.0f) < 0.5f);
}

void test_altitude(void)
{
    test_fusion_beats_both();
    test_one_sensor_alone();
    test_the_reference();
    test_the_barometer_stops_answering();
}
