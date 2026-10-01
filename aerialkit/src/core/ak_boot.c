#include "ak_boot.h"

/*
 * The boot's record, and the two blink plans the tell-tale build drives.
 *
 * Nothing here touches a pin: the pattern is data, and running it is four
 * lines, which is what makes the part a person depends on testable on the host
 * (tests/test_boot.c) rather than only on the board it is meant to explain.
 */

static int boot_stage;

void ak_boot_mark(int stage)
{
    if (stage >= AK_BOOT_CLOCK) {
        boot_stage = stage;
    }
    ak_board_boot_mark(stage);
}

int ak_boot_stage_reached(void)
{
    return boot_stage;
}

const char *ak_boot_stage_name(int stage)
{
    switch (stage) {
    case AK_BOOT_CLOCK:      return "the clock";
    case AK_BOOT_BARO_BUS:   return "the barometer's bus";
    case AK_BOOT_CONSOLE:    return "the console's uart";
    case AK_BOOT_USB:        return "the usb core";
    case AK_BOOT_TICK:       return "the millisecond tick";
    case AK_BOOT_BANNER:     return "the banner";
    case AK_BOOT_SELFTEST:   return "the selftest";
    case AK_BOOT_PARAMETERS: return "the parameter table";
    case AK_BOOT_OUTPUTS:    return "the outputs";
    case AK_BOOT_BATTERY:    return "the pack";
    case AK_BOOT_IMU:        return "the inertial sensor";
    case AK_BOOT_ALTITUDE:   return "the altitude and rangefinder";
    case AK_BOOT_PREFLIGHT:  return "the network and the preflight";
    default:                 return "a step this build does not know";
    }
}

/*
 * A board that does not implement the tell-tale gets this, so the contract is
 * "a board *may* instrument its boot" rather than a function every one of the
 * four ESP32 boards and the host stand-in has to write out as empty.
 */
__attribute__((weak)) void ak_board_boot_mark(int stage)
{
    (void)stage;
}

int ak_boot_blink_plan(int stage, ak_boot_beat_t *beats, int max)
{
    if (stage < AK_BOOT_CLOCK || stage > AK_BOOT_STAGE_COUNT || beats == 0) {
        return 0;
    }

    int needed = AK_BOOT_BLINK_ROUNDS * (2 * stage + 1);
    if (max < needed) {
        return -1;
    }

    int n = 0;
    for (int round = 0; round < AK_BOOT_BLINK_ROUNDS; round++) {
        for (int i = 0; i < stage; i++) {
            beats[n].on = 1;
            beats[n].hold_ms = (uint16_t)120u;
            n++;
            beats[n].on = 0;
            beats[n].hold_ms = (uint16_t)160u;
            n++;
        }
        /* The gap that makes one round's last blink the end of a round rather
         * than the start of the next pattern. */
        beats[n].on = 0;
        beats[n].hold_ms = (uint16_t)700u;
        n++;
    }
    return n;
}

int ak_boot_fault_plan(int stage, ak_boot_beat_t *beats, int max)
{
    int steps = stage >= AK_BOOT_CLOCK && stage <= AK_BOOT_STAGE_COUNT ? stage : 0;
    int needed = 20 + 1 + 2 * steps;

    if (beats == 0 || max < needed) {
        return beats == 0 ? 0 : -1;
    }

    int n = 0;
    for (int i = 0; i < 10; i++) {
        beats[n].on = 1;
        beats[n].hold_ms = (uint16_t)100u;
        n++;
        beats[n].on = 0;
        beats[n].hold_ms = (uint16_t)100u;
        n++;
    }
    /* The pause is what makes the ten distinct from a stage pattern: no stage
     * has ten blinks, and none of them ends in silence this long. */
    beats[n].on = 0;
    beats[n].hold_ms = (uint16_t)1500u;
    n++;
    for (int i = 0; i < steps; i++) {
        beats[n].on = 1;
        beats[n].hold_ms = (uint16_t)250u;
        n++;
        beats[n].on = 0;
        beats[n].hold_ms = (uint16_t)250u;
        n++;
    }
    return n;
}

void ak_boot_run(const ak_boot_beat_t *beats, int count,
                 ak_boot_led_fn led, ak_boot_wait_fn wait)
{
    for (int i = 0; i < count; i++) {
        if (led != 0) {
            led(beats[i].on);
        }
        if (wait != 0 && beats[i].hold_ms > 0u) {
            wait(beats[i].hold_ms);
        }
    }
}
