#ifndef AK_FLIGHT_AK_BATTERY_H
#define AK_FLIGHT_AK_BATTERY_H

#include <stdint.h>

/*
 * The flight pack, as a voltage.
 *
 * A flight controller that does not know how much battery it has left is a
 * flight controller that finds out in the air. What it takes to know is small:
 * a divider of two resistors, an ADC pin, and the arithmetic in this file.
 *
 * The shape of the arithmetic is worth stating, because each step is a place
 * where a wrong answer is quiet rather than loud:
 *
 *   counts  ->  volts at the pin     what the ADC measured, scaled by Vref
 *   pin     ->  pack volts           multiplied by the divider's ratio
 *   pack    ->  cell count           divided by what one cell can hold
 *   pack    ->  volts per cell       the only number a pilot reads
 *
 * The cell count is *counted*, not configured, because a pack is plugged in by
 * hand and the firmware is the thing that should notice. The count only ever
 * goes up while a pack stays plugged in: a 3S that sags to 8.4 volts under load
 * is a 3S at 2.8 volts a cell - nearly empty - and reading it as a 2S at 4.2
 * volts would turn the last minute of a flight into a reassurance.
 *
 * Nothing here talks to hardware. The board measures volts at its own ADC pin
 * and says so; everything above that is this file, which is why it runs in the
 * host tests and in the simulator as well as on the board.
 */

typedef enum {
    /* Nothing plugged in, or nothing to read it with. The two are deliberately
     * the same answer: an unconfigured ADC reads zero, and zero is also what a
     * divider with no pack on it reads, and neither one is "flat battery". */
    AK_BATTERY_ABSENT = 0,
    AK_BATTERY_OK = 1,
    AK_BATTERY_WARN = 2,
    AK_BATTERY_CRITICAL = 3,
} ak_battery_state_t;

typedef struct {
    /* What the divider does to the pack before the ADC sees it: pack volts per
     * volt at the pin. Two equal resistors give 2.0; the 10k/1k pair this
     * board is drawn for gives 11.0, which puts a 4S pack at 1.5 volts - far
     * enough below the 3.3 volt rail that a fully charged pack cannot clip. */
    float divider_ratio;

    /* Per cell, because that is what a pack has in common with another pack of
     * a different size. The three numbers are the same three the reference
     * implementations use; see docs/19-battery.md for where they come from. */
    float cell_detect_v; /* a cell cannot hold more than this, so ceil() counts */
    float warn_cell_v;
    float critical_cell_v;

    /* Below this the pack is not there; above the ceiling it is not a pack
     * this firmware knows about, and a reading like that is counted and
     * dropped rather than believed - a divider that is missing, or a ratio
     * that is wrong, otherwise arrives as a confident nonsense cell count. */
    float absent_below_v;
    float reject_above_v;

    /* How much of a new reading to take per second. A pack sags under throttle
     * and comes back when it stops, and the number worth warning on is neither
     * the instantaneous one nor a slowly-recovering average. */
    float filter_per_s;

    /* How many readings in a row have to be below the floor before a pack
     * counts as unplugged. "Nothing is connected" is a statement about the pin
     * rather than about the pack's voltage, so it is decided from the raw
     * reading - but a single reading of nothing is a wire that moved, not a
     * battery that left, and resetting the cell count on one of those is how a
     * sagging pack gets re-counted as a smaller one that is doing fine. */
    uint32_t absent_readings;

    /* 0 counts the cells from the voltage. A number overrides it, for a pack
     * whose count the arithmetic gets wrong - and the override is the pilot's
     * answer, so it is used as given rather than latched. */
    uint32_t cells_override;

    /* The last reading and the answer. */
    float pin_volts;      /* what the board measured; negative if it could not */
    float volts;          /* the pack, filtered */
    float raw_volts;      /* the pack, this instant */
    float volts_per_cell;
    uint32_t cells;
    uint32_t samples;     /* readings taken into the filter */
    uint32_t rejected;    /* readings dropped as implausible */
    ak_battery_state_t state;
} ak_battery_t;

void ak_battery_init(ak_battery_t *battery);

/* Counts to volts at the pin: the one piece of the chain that is the ADC's
 * rather than the divider's, and the one a host test can check exactly. */
float ak_battery_pin_volts(uint32_t counts, float vref, uint32_t full_scale);

/* How many cells a pack voltage holds. Zero for a voltage that is not a pack
 * at all. Exposed because it is the step that is easiest to get wrong and
 * cheapest to test on its own. */
uint32_t ak_battery_cell_count(float volts, float detect_v);

/* One reading, in volts at the ADC pin, and how long since the last one.
 * A negative voltage means the board had no reading, which is not a pack that
 * is not there - but it is not a number either, so nothing moves. */
void ak_battery_sample(ak_battery_t *battery, float pin_volts, float dt_s);

const char *ak_battery_state_name(ak_battery_state_t state);

#endif /* AK_FLIGHT_AK_BATTERY_H */
