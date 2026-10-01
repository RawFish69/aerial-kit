#ifndef AK_CORE_AK_MAIN_H
#define AK_CORE_AK_MAIN_H

/*
 * The firmware's entry point, named rather than being `main`.
 *
 * Each platform has its own way in - IDF wants app_main(), a host build wants
 * main(), a board hands over from its startup code - and all of them should
 * arrive at the same function. Calling it `ak_firmware_main` is what lets a
 * simulator link the whole firmware without fighting over the name.
 */

int ak_firmware_main(void);

/*
 * How many bytes one pass of the main loop may take off one link before the
 * rest of the loop runs.
 *
 * This is what stops a flood on a link from deciding how often the control
 * loop runs. 32 bytes at a kilohertz is 32 kB/s per link, comfortably above any
 * of them when they are behaving - CRSF is 13 kB/s at 500 Hz - and the quota is
 * only ever reached by a link that is not.
 *
 * It lives here rather than in main.c because the simulator has to be able to
 * say whether the firmware held to it: `tools/fw_sim.c` counts the console
 * bytes taken in a single pass and compares them against this number, and a
 * check against a copy of a constant is a check against the wrong number
 * (trap 59's family, and the reason `AK_FLIGHT_LOOP_MS` moved to
 * `ak_flight.h`).
 */
#define AK_DRAIN_QUOTA 32u

/*
 * The links the quota applies to, one at a time.
 *
 * Named rather than numbered because the counter below is read per link: the
 * flood session is a claim about the *console*, and a maximum across all four
 * would be satisfied by the receiver legitimately taking its quota's worth in
 * one pass while the console took a hundred thousand. That is exactly what the
 * first version of this did - it read 32 with the console unbounded, and the
 * check passed. A number that cannot tell the faulty link from the healthy one
 * is not a measurement of either (trap 60).
 */
typedef enum {
    AK_LINK_CONSOLE = 0,
    AK_LINK_NET,
    AK_LINK_RC,
    AK_LINK_GPS,
    AK_LINK_COUNT
} ak_link_t;

/*
 * The most bytes that link has taken in a single pass since boot.
 *
 * The quota's own number, counted where a pass is defined, so that something
 * outside the file can check it. Exported for the same reason
 * `ak_flight_timing()` is: a claim about the loop that only the loop can see is
 * a claim nobody can falsify. It is a maximum and never decreases - a pass that
 * took a hundred bytes stays visible after the link goes quiet, which is what
 * makes it readable from a console after the fact rather than only during.
 */
uint32_t ak_main_drain_max(ak_link_t link);

#endif /* AK_CORE_AK_MAIN_H */
