/*
 * The flight pack.
 *
 * Three pieces of arithmetic in a row, and each one has a way of being wrong
 * that is quiet rather than loud: a divider that multiplies by the wrong
 * number, a cell count that rounds the wrong way, a threshold that is
 * compared with < when it should be <=. None of them can be checked by running
 * the firmware, because they all produce a number that looks like a number.
 *
 * So the values here were worked out separately - a multimeter's arithmetic,
 * not this file's - and pinned. The wiring itself is a bench question and is
 * in docs/19-battery.md.
 */

#include <math.h>
#include <stdio.h>

#include "ak_battery.h"
#include "tests.h"

/* A 12-bit ADC with the board's 3.3 volt reference, and the board's divider. */
#define VREF     3.3f
#define FULL     4095u
#define DIVIDER  11.0f

/* A pack voltage, as the pin sees it. */
static float pin_of(float pack_v)
{
    return pack_v / DIVIDER;
}

/* One reading, through the whole chain, and the state that came out.
 *
 * A whole second between readings, which the filter takes as the reading
 * itself - so these are readings of a pack that is holding still, and the
 * filter's own behaviour is the subject of test_the_filter() rather than
 * something every other check has to allow for. */
static ak_battery_state_t read_pack(ak_battery_t *b, float pack_v)
{
    ak_battery_sample(b, pin_of(pack_v), 1.0f);
    return b->state;
}

/* A connector coming out: the pin is at zero and stays there. */
static void unplug(ak_battery_t *b)
{
    for (int i = 0; i < 3; i++) {
        ak_battery_sample(b, 0.0f, 0.1f);
    }
}

static void test_counts_to_volts(void)
{
    expect("full scale counts is the reference voltage",
           fabsf(ak_battery_pin_volts(FULL, VREF, FULL) - VREF) < 0.0005f);
    expect("zero counts is zero volts",
           ak_battery_pin_volts(0u, VREF, FULL) == 0.0f);
    expect("half scale is half the reference",
           fabsf(ak_battery_pin_volts(FULL / 2u, VREF, FULL) - 1.65f) < 0.001f);
    /* 2048 counts of 4095 at 3.3 volts is 1.6505 volts, which the next digit
     * of the sum is in: the point of the check is that it is a division and
     * not a shift. */
    expect("and the last count counts",
           ak_battery_pin_volts(FULL, VREF, FULL) >
               ak_battery_pin_volts(FULL - 1u, VREF, FULL));
    expect("a full scale of zero divides by nothing, not by zero",
           ak_battery_pin_volts(1234u, VREF, 0u) == 0.0f);
}

static void test_counting_cells(void)
{
    /* A cell cannot hold more than the detect voltage, so the count is the
     * ceiling of the pack over one cell. These are the packs somebody would
     * actually plug into one of these boards. */
    expect("a charged 1S is one cell", ak_battery_cell_count(4.20f, 4.30f) == 1u);
    expect("a charged 2S is two cells", ak_battery_cell_count(8.40f, 4.30f) == 2u);
    expect("a charged 3S is three cells", ak_battery_cell_count(12.60f, 4.30f) == 3u);
    expect("a charged 4S is four cells", ak_battery_cell_count(16.80f, 4.30f) == 4u);
    expect("a charged 6S is six cells", ak_battery_cell_count(25.20f, 4.30f) == 6u);

    /* And the ones in the middle of a flight, which is where a count that
     * rounds down turns into a reassurance. */
    expect("a half-empty 3S is still three cells",
           ak_battery_cell_count(10.50f, 4.30f) == 3u);
    expect("a nearly-empty 3S is still three cells",
           ak_battery_cell_count(9.60f, 4.30f) == 3u);
    expect("a nearly-empty 4S is still four cells",
           ak_battery_cell_count(13.20f, 4.30f) == 4u);

    /*
     * The trap the 4.30 volt detect voltage exists for. A 3S that came off a
     * charger hot reads 12.65, and 12.65 over the 4.20 a cell is nominally
     * full to is 3.01 - a ceil() against that says four cells, and four cells
     * at 3.16 volts is a pack that is fine, which is the wrong answer about a
     * 3S that is full.
     */
    expect("a 3S above 4.20 a cell is not read as a 4S",
           ak_battery_cell_count(12.65f, 4.30f) == 3u);
    expect("and 12.65 volts is 4 cells against the wrong threshold",
           ak_battery_cell_count(12.65f, 4.20f) == 4u);

    /* Nothing at all, and more than this firmware believes in. */
    expect("no volts is no cells", ak_battery_cell_count(0.0f, 4.30f) == 0u);
    expect("a negative voltage is no cells",
           ak_battery_cell_count(-1.0f, 4.30f) == 0u);
    expect("a threshold of zero counts nothing",
           ak_battery_cell_count(12.0f, 0.0f) == 0u);
    expect("a 14S is capped at what this firmware believes in",
           ak_battery_cell_count(60.0f, 4.30f) == 8u);
}

static void test_the_chain(void)
{
    ak_battery_t b;
    ak_battery_init(&b);
    expect("the default divider is the 10k/1k pair", b.divider_ratio == 11.0f);
    expect("and before anything is read there is nothing to report",
           b.state == AK_BATTERY_ABSENT && b.samples == 0u);

    /* A 3S at its nominal 3.7 volts a cell, read through a real ADC's
     * counts: 1.009 volts at the pin is 1252 counts, which is 1.0086 volts,
     * which is 11.095 volts of pack. The whole chain, one number at a time. */
    uint16_t counts = (uint16_t)(pin_of(11.1f) / VREF * (float)FULL);
    expect("1.009 volts at the pin is 1252 counts", counts == 1252u);
    ak_battery_sample(&b, ak_battery_pin_volts(counts, VREF, FULL), 0.1f);
    expect("and those counts are 11.09 volts of pack",
           fabsf(b.volts - 11.09f) < 0.02f);
    expect("which is three cells", b.cells == 3u);
    expect("at 3.70 volts a cell", fabsf(b.volts_per_cell - 3.697f) < 0.01f);
    expect("and that is a healthy pack", b.state == AK_BATTERY_OK);

    /* The thresholds, and the side of them. Exactly on the warning voltage is
     * not yet a warning; a hundredth under it is. */
    ak_battery_init(&b);
    expect("3.50 volts a cell is exactly at the warning and still ok",
           read_pack(&b, 10.50f) == AK_BATTERY_OK);
    ak_battery_init(&b);
    expect("3.49 volts a cell is low", read_pack(&b, 10.47f) == AK_BATTERY_WARN);
    ak_battery_init(&b);
    expect("3.33 volts a cell is still only low",
           read_pack(&b, 10.0f) == AK_BATTERY_WARN);
    ak_battery_init(&b);
    expect("3.00 volts a cell is critical",
           read_pack(&b, 9.0f) == AK_BATTERY_CRITICAL);

    expect("the states have words", 
           ak_battery_state_name(AK_BATTERY_OK)[0] == 'o' &&
           ak_battery_state_name(AK_BATTERY_ABSENT)[0] == 'n');
}

static void test_a_pack_never_loses_a_cell(void)
{
    ak_battery_t b;
    ak_battery_init(&b);

    expect("a 3S reads as three cells", read_pack(&b, 12.60f) == AK_BATTERY_OK);
    expect("and the count is three", b.cells == 3u);

    /* Now it sags. 8.4 volts divided by the 4.30 a cell is 1.95, so the
     * arithmetic on its own says two cells - and two cells at 4.20 volts is a
     * pack that is fine. It is not: it is a 3S at 2.8 volts a cell, which is
     * past the end of its usable range. The count is a high-water mark for
     * exactly this reason. */
    expect("a 3S sagging to 8.4 volts is critical, not a healthy 2S",
           read_pack(&b, 8.40f) == AK_BATTERY_CRITICAL);
    expect("because the count stayed at three", b.cells == 3u);
    expect("and 2.80 volts a cell is what it says",
           fabsf(b.volts_per_cell - 2.80f) < 0.02f);

    /* Unplugging it is the one thing that resets the count. */
    unplug(&b);
    expect("unplugging the pack is what clears the count",
           b.state == AK_BATTERY_ABSENT && b.cells == 0u);
    expect("and a 4S plugged in afterwards is counted afresh",
           read_pack(&b, 16.80f) == AK_BATTERY_OK && b.cells == 4u);
}

static void test_an_override(void)
{
    ak_battery_t b;
    ak_battery_init(&b);
    b.cells_override = 4u;

    /* A pilot who says four cells is believed, even when the arithmetic would
     * have counted three - four cells at 3.15 volts is a pack to land, and
     * that is the point of being able to say so. */
    read_pack(&b, 12.60f);
    expect("an overridden count is used as given", b.cells == 4u);
    expect("and the level is computed from it",
           fabsf(b.volts_per_cell - 3.15f) < 0.01f &&
               b.state == AK_BATTERY_CRITICAL);
}

static void test_the_traps(void)
{
    ak_battery_t b;

    /*
     * The trap that would cost an aircraft: an ADC that is not configured
     * reads zero, and zero volts a cell is the most alarming number there is.
     * It is not a flat battery, it is no battery, and the two want different
     * things done about them.
     */
    ak_battery_init(&b);
    read_pack(&b, 12.60f);
    ak_battery_sample(&b, 0.0f, 0.1f);
    expect("one reading of nothing is a wire, not an empty battery",
           b.state == AK_BATTERY_OK && b.cells == 3u && b.volts > 12.5f);

    /* Two more and it is an unplugged pack - and the estimate snaps to what
     * the pin says rather than easing down to it, because the pin is the one
     * telling the truth. */
    ak_battery_sample(&b, 0.0f, 0.1f);
    ak_battery_sample(&b, 0.0f, 0.1f);
    expect("three in a row is a pack that is not there",
           b.state == AK_BATTERY_ABSENT);
    expect("and it does not invent a cell count", b.cells == 0u);
    expect("or a voltage per cell", b.volts_per_cell == 0.0f);
    expect("and the estimate is what the pin reads, not a guess",
           b.volts == 0.0f);

    /* A pin sitting at the rail is not a pack either. 3.3 volts at the pin is
     * 36.3 volts of pack, which is more than this firmware believes in, and a
     * reading like that is counted and dropped rather than turned into nine
     * confident cells. */
    ak_battery_init(&b);
    ak_battery_sample(&b, 3.3f, 0.1f);
    expect("a reading above anything this firmware knows is rejected",
           b.rejected == 1u && b.samples == 0u);
    expect("and leaves the answer where it was",
           b.state == AK_BATTERY_ABSENT && b.volts == 0.0f);

    /* A board with no ADC at all says so with a negative voltage, which is not
     * the same answer as zero and does not move the estimate. */
    ak_battery_init(&b);
    read_pack(&b, 12.60f);
    float held = b.volts;
    uint32_t samples = b.samples;
    ak_battery_sample(&b, -1.0f, 0.1f);
    expect("a board that cannot read its own ADC does not become an empty one",
           b.volts == held && b.samples == samples &&
               b.state == AK_BATTERY_OK);
}

static void test_the_filter(void)
{
    ak_battery_t b;
    ak_battery_init(&b);

    /* The first reading is the estimate rather than a step towards it: a pack
     * that reads 12 volts should say 12 volts, not creep up to it while the
     * cell count is decided from a voltage that is still wrong. */
    ak_battery_sample(&b, pin_of(12.0f), 0.1f);
    expect("the first reading is the answer, not a step towards it",
           b.volts == 12.0f && b.samples == 1u);

    /* Then a step down to 6 volts, sampled at ten hertz. Taking a fifth of the
     * difference each time is what a two-per-second filter is at that rate. */
    float previous = b.volts;
    ak_battery_sample(&b, pin_of(6.0f), 0.1f);
    expect("and the next one takes a fifth of the difference",
           fabsf(b.volts - 10.8f) < 0.001f && b.volts < previous);

    int monotone = 1;
    for (int i = 0; i < 40; i++) {
        float before = b.volts;
        ak_battery_sample(&b, pin_of(6.0f), 0.1f);
        monotone = monotone && b.volts <= before && b.volts >= 6.0f;
    }
    expect("and it converges on the new pack without overshooting",
           monotone && fabsf(b.volts - 6.0f) < 0.01f);

    /* A slow sample rate must not step past the reading it is filtering. */
    ak_battery_init(&b);
    ak_battery_sample(&b, pin_of(12.0f), 0.1f);
    ak_battery_sample(&b, pin_of(8.0f), 5.0f);
    expect("a five-second gap takes the reading whole rather than past it",
           b.volts == 8.0f);
}

void test_battery(void)
{
    test_counts_to_volts();
    test_counting_cells();
    test_the_chain();
    test_a_pack_never_loses_a_cell();
    test_an_override();
    test_the_traps();
    test_the_filter();
}
