#ifndef AK_CORE_AK_BOOT_H
#define AK_CORE_AK_BOOT_H

#include <stdint.h>

/*
 * Where the boot has got to.
 *
 * The F405 on the bench on 2026-09-17 put its USB device on the bus, refused
 * every request the host made, and lit its LED and left it lit. Nothing ever
 * said which of the two halves of that was the boot and which was the port:
 * the console only existed as a USB device nobody could enumerate, and the LED
 * was the only instrument the board had. This is that instrument, written down
 * once instead of in a build nobody can reproduce.
 *
 * Every image keeps the record - `ak_boot_mark()` is a store and a call, and
 * `ak_boot_stage_reached()` is what a test or a report reads. A board built
 * with `AK_BOOT_STAGE=1` *also* blinks it, on the status pin, which is what a
 * board with no console at all needs:
 *
 *   stage 4   four blinks, three times over, and then the next stage
 *   fault     ten quick blinks, a pause, then the stage the boot had reached
 *
 * The pattern is three rounds because it is read off an LED by a person who
 * may look up late: one blink of stage 3 and one stage of "three blinks" are
 * not the same thing if you missed the start. The cost is that a *healthy*
 * tell-tale boot takes about a minute and a half to blink its way past all
 * thirteen stages, which is the price of an instrument nobody has to guess at
 * - and a boot that stops tells you where in the first ten seconds.
 *
 * What the blink is *not* good at is a board whose LED is itself in doubt
 * (it says nothing about the pin, the polarity or the resistor), which is why
 * the record exists in every image as well, and why the first bench question
 * stays "is the lit thing the status LED or the power LED".
 */

/* The thirteen steps, numbered in the order they run. The first four are the
 * board's own (which clock, which bus, which console, which USB core) and the
 * rest are the core's, so a board that shares none of those still has one
 * list to read. */
enum {
    AK_BOOT_CLOCK = 1,
    AK_BOOT_BARO_BUS,
    AK_BOOT_CONSOLE,
    AK_BOOT_USB,
    AK_BOOT_TICK,
    AK_BOOT_BANNER,
    AK_BOOT_SELFTEST,
    AK_BOOT_PARAMETERS,
    AK_BOOT_OUTPUTS,
    AK_BOOT_BATTERY,
    AK_BOOT_IMU,
    AK_BOOT_ALTITUDE,
    AK_BOOT_PREFLIGHT,
    AK_BOOT_STAGE_COUNT = AK_BOOT_PREFLIGHT,

    /* Not a step: the CPU has stopped, and the number after the ten quick
     * blinks is how far the boot had got. Only ever passed to a board's
     * tell-tale, never recorded as the stage reached. */
    AK_BOOT_FAULT = 0,
};

#define AK_BOOT_BLINK_ROUNDS 3

/* The longest pattern the blink plans can ask for: stage 13, three rounds of
 * thirteen blinks and the gap after each round. */
#define AK_BOOT_BEATS_MAX \
    (AK_BOOT_BLINK_ROUNDS * (2 * AK_BOOT_STAGE_COUNT + 1))

/* One beat of a plan: what the pin does, and for how long. Ordered by size so
 * that a plan of eighty-one of them is 324 bytes rather than 648 - the
 * tell-tale keeps one in RAM, and a diagnostic that wastes half of what it
 * asks for is the sort of thing a person notices on the smallest part. */
typedef struct {
    uint16_t hold_ms;
    uint8_t  on;
} ak_boot_beat_t;

typedef void (*ak_boot_led_fn)(int on);
typedef void (*ak_boot_wait_fn)(uint32_t ms);

/* Says where the boot is. Called by the core between its own steps and by a
 * board between its two or three, and it is the one thing that must never be
 * able to fail: a store, and a call that most boards do not implement. */
void ak_boot_mark(int stage);

/* The last stage marked, 0 before the first one. */
int ak_boot_stage_reached(void);

/* A stage's name, for a report or a test. Never null. */
const char *ak_boot_stage_name(int stage);

/* Fill `beats` with the plan for a stage: N blinks, three times over, with the
 * gap between rounds. Returns the number of beats, 0 for a stage that has no
 * pattern, and -1 when `max` cannot hold it. */
int ak_boot_blink_plan(int stage, ak_boot_beat_t *beats, int max);

/* And the crash's: ten quick blinks, a pause, then the stage reached. */
int ak_boot_fault_plan(int stage, ak_boot_beat_t *beats, int max);

/* Run a plan against a pin and a wait. The board supplies both - the LED
 * because that is what it has, and a wait because a native delay cannot be
 * used before the tick exists. */
void ak_boot_run(const ak_boot_beat_t *beats, int count,
                 ak_boot_led_fn led, ak_boot_wait_fn wait);

/*
 * The board's half, and it is optional: the core supplies an empty one, so a
 * board that has a console from its first instruction (either ESP32, QEMU, the
 * host) says nothing and needs no code. The two ARM boards implement it under
 * `#if AK_BOOT_STAGE`, which is off in a flight image and on in the diagnostic
 * one - `make EXTRA_CFLAGS=-DAK_BOOT_STAGE=1`.
 */
void ak_board_boot_mark(int stage);

#ifndef AK_BOOT_STAGE
#define AK_BOOT_STAGE 0
#endif

#endif /* AK_CORE_AK_BOOT_H */
