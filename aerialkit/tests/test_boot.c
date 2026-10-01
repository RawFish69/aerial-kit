/*
 * Where the boot has got to, and the blink that says so.
 *
 * The instrument exists because a board whose console is the thing that is
 * broken has one output left: its status pin. What is worth testing is not
 * that a pin can be toggled - the board's own test covers that - but that the
 * *pattern* is the one a person reads off it: N blinks for stage N, three
 * rounds of them, and a fault pattern that cannot be mistaken for a stage.
 * Those are the parts a mistake turns into a bench session spent counting
 * blinks that mean something else, and they are pure arithmetic, so they are
 * checkable here rather than on the board.
 */

#include <stdio.h>
#include <string.h>

#include "ak_boot.h"
#include "tests.h"

/* What a person would write down, so the test says it the way the bench reads
 * it: the levels the pin is driven to, and the waits between them. */
static int led_levels = 0;
static unsigned led_beats = 0;
static uint32_t waited_ms = 0;
static int last_level = -1;
static int rising = 0;

static void note_led(int on)
{
    led_beats++;
    if (on && last_level != 1) {
        rising++;
    }
    last_level = on;
}

static void note_wait(uint32_t ms)
{
    waited_ms += ms;
}

static void run_plan(const ak_boot_beat_t *beats, int count)
{
    led_beats = 0;
    rising = 0;
    last_level = -1;
    waited_ms = 0;
    (void)led_levels;
    ak_boot_run(beats, count, note_led, note_wait);
}

/* The number of times the pin goes from dark to lit: the blinks a person
 * counts, which is what the pattern is for. */
static int blinks_for(int stage)
{
    ak_boot_beat_t beats[AK_BOOT_BEATS_MAX];
    int count = ak_boot_blink_plan(stage, beats, AK_BOOT_BEATS_MAX);

    if (count <= 0) {
        return -1;
    }
    run_plan(beats, count);
    return rising;
}

void test_boot(void)
{
    ak_boot_beat_t beats[AK_BOOT_BEATS_MAX];
    ak_boot_beat_t short_buffer[4];
    int count;

    /*
     * Nothing has been marked at the point a test gets here if the firmware's
     * own boot has not run - and when it has, the marks are what the simulator
     * reads. Either way the record starts at something the plan below can
     * move.
     */
    expect("a boot that has not been marked says so rather than guessing",
           ak_boot_stage_reached() == 0 || ak_boot_stage_reached() >= 1);

    ak_boot_mark(AK_BOOT_TICK);
    expect("the stage the boot is at is the last one marked",
           ak_boot_stage_reached() == AK_BOOT_TICK);
    ak_boot_mark(AK_BOOT_PREFLIGHT);
    expect("and it moves forward with the boot",
           ak_boot_stage_reached() == AK_BOOT_PREFLIGHT);
    expect("with a name for every stage",
           strcmp(ak_boot_stage_name(AK_BOOT_CLOCK), "the clock") == 0 &&
               strcmp(ak_boot_stage_name(AK_BOOT_PREFLIGHT),
                      "the network and the preflight") == 0 &&
               ak_boot_stage_name(AK_BOOT_STAGE_COUNT + 40) != 0);

    /* And a name of its own for each of the thirteen: a list that says "the
     * clock" twice is a list a person reading a blink pattern cannot use. */
    int named = 1;
    for (int stage = 1; stage <= AK_BOOT_STAGE_COUNT; stage++) {
        const char *name = ak_boot_stage_name(stage);

        if (name == 0 || name[0] == '\0' ||
            strcmp(name, ak_boot_stage_name(AK_BOOT_STAGE_COUNT + 40)) == 0) {
            named = 0;
        }
        if (stage > 1 &&
            strcmp(name, ak_boot_stage_name(stage - 1)) == 0) {
            named = 0;
        }
    }
    expect("every stage has a name of its own", named);

    /*
     * The pattern itself: stage N is N blinks, and the *three* rounds are the
     * same three every time, so the number a person counts is the stage and the
     * repeats are what let them start counting late.
     */
    for (int stage = 1; stage <= AK_BOOT_STAGE_COUNT; stage++) {
        int blinks = blinks_for(stage);
        expect("a stage blinks its own number, three times over",
               blinks == stage * AK_BOOT_BLINK_ROUNDS);
    }
    expect("a stage that is not a stage has no pattern",
           ak_boot_blink_plan(0, beats, AK_BOOT_BEATS_MAX) == 0 &&
               ak_boot_blink_plan(AK_BOOT_STAGE_COUNT + 1, beats,
                                  AK_BOOT_BEATS_MAX) == 0);
    expect("and a buffer too small is refused rather than overrun",
           ak_boot_blink_plan(AK_BOOT_PREFLIGHT, short_buffer, 4) == -1);

    /*
     * The plan's beat count is what the board's array is sized for, and the
     * count is exact rather than merely large: a plan that wrote one beat past
     * the end would be a tell-tale that crashes the thing it is explaining.
     */
    count = ak_boot_blink_plan(AK_BOOT_PREFLIGHT, beats, AK_BOOT_BEATS_MAX);
    expect("the longest pattern fits the array the boards declare",
           count == AK_BOOT_BEATS_MAX && count > 0);
    run_plan(beats, count);
    expect("and it is thirteen blinks, three times, with the gap after each",
           rising == 3 * AK_BOOT_STAGE_COUNT && led_beats == (unsigned)count &&
               waited_ms > 0u);

    /*
     * And the crash. Ten quick blinks is not a stage - no stage is ten - the
     * pause after them is longer than any gap inside a stage, and the number
     * that follows is the stage the boot had reached, which is the whole point
     * of the pattern.
     */
    count = ak_boot_fault_plan(AK_BOOT_USB, beats, AK_BOOT_BEATS_MAX);
    expect("a fault blinks its ten and then the stage", count > 0);
    run_plan(beats, count);
    expect("which is ten blinks before the pause and the stage after it",
           rising == 10 + AK_BOOT_USB);

    count = ak_boot_fault_plan(AK_BOOT_FAULT, beats, AK_BOOT_BEATS_MAX);
    run_plan(beats, count);
    expect("a fault before any stage has been marked is ten blinks and no "
           "number", rising == 10 && count > 0);

    expect("and the fault pattern into a buffer that is too small is refused",
           ak_boot_fault_plan(AK_BOOT_PREFLIGHT, short_buffer, 4) == -1);
    expect("and a fault pattern with nowhere to put it is nothing rather than "
           "an overrun", ak_boot_fault_plan(AK_BOOT_PREFLIGHT, 0, 0) == 0);

    /*
     * And the walk itself, with neither a pin nor a clock to drive: a caller
     * that has neither is how a port that is being brought up calls it, and
     * the plan has to be readable without being run.
     */
    count = ak_boot_blink_plan(AK_BOOT_CLOCK, beats, AK_BOOT_BEATS_MAX);
    ak_boot_run(beats, count, 0, 0);
    expect("a plan walks with no pin and no clock of its own", count == 9);

    printf("  ----      the last mark here was %d, %s\n",
           ak_boot_stage_reached(),
           ak_boot_stage_name(ak_boot_stage_reached()));
}
