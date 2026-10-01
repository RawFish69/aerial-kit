#include "ak_battery.h"

/*
 * How many cells this firmware is willing to believe in. Eight, which is what
 * the reference implementation caps its own auto-detection at; a pack bigger
 * than that is not a pack either of the two airframes this targets can lift.
 */
#define AK_BATTERY_MAX_CELLS 8u

/* Readings of nothing in a row before the pack is called unplugged. Three is
 * a third of a second at the rate the core samples at: long enough that a
 * wire moving or one bad conversion is not an unplug, short enough that
 * somebody who has just pulled a connector does not have to wait to see it. */
#define AK_BATTERY_ABSENT_READINGS 3u

void ak_battery_init(ak_battery_t *battery)
{
    /*
     * The three thresholds are the reference implementations' defaults, in
     * volts per cell: warn at 3.50, complain loudly at 3.30, and count the
     * cells against 4.30 rather than the 4.20 a cell is nominally full to.
     *
     * The last of those is the one that matters. Counting with ceil() is what
     * makes a 3S that has sagged to 9.9 volts still read as three cells, but
     * ceil() against 4.20 would read a *fully charged* 3S at 12.65 volts as
     * four cells, because 12.65 / 4.20 is 3.01. Against 4.30 there is a tenth
     * of a volt a cell of headroom for a pack that came off the charger hot,
     * and the count stays right over the whole range a 3S is used over.
     */
    battery->divider_ratio = 11.0f; /* 10k over 1k */
    battery->cell_detect_v = 4.30f;
    battery->warn_cell_v = 3.50f;
    battery->critical_cell_v = 3.30f;
    battery->absent_below_v = 2.0f;
    battery->reject_above_v = 4.30f * (float)AK_BATTERY_MAX_CELLS;
    /* Half a second. Fast enough that a pack is reported within a second of
     * being plugged in, slow enough that the sag of one throttle punch does
     * not read as an empty pack and the recovery afterwards does not read as
     * a fresh one. */
    battery->filter_per_s = 2.0f;
    battery->cells_override = 0u;
    battery->absent_readings = 0u;

    battery->pin_volts = 0.0f;
    battery->volts = 0.0f;
    battery->raw_volts = 0.0f;
    battery->volts_per_cell = 0.0f;
    battery->cells = 0u;
    battery->samples = 0u;
    battery->rejected = 0u;
    battery->state = AK_BATTERY_ABSENT;
}

float ak_battery_pin_volts(uint32_t counts, float vref, uint32_t full_scale)
{
    if (full_scale == 0u) {
        return 0.0f;
    }
    return (float)counts * vref / (float)full_scale;
}

uint32_t ak_battery_cell_count(float volts, float detect_v)
{
    if (detect_v <= 0.0f || volts <= 0.0f) {
        return 0u;
    }

    uint32_t cells = (uint32_t)(volts / detect_v);

    /* The ceiling, with a millivolt of slack for the last bit of a float. A
     * pack that is 12.9000 volts against a 4.3000 threshold is three cells,
     * and this is the line that decides it rather than the rounding of
     * `volts / detect_v` one instruction earlier. */
    if ((float)cells * detect_v < volts - 0.001f) {
        cells++;
    }
    if (cells == 0u) {
        /* Any pack at all is at least one cell, even one that is deep in the
         * wrong end of its discharge curve. */
        cells = 1u;
    }
    if (cells > AK_BATTERY_MAX_CELLS) {
        cells = AK_BATTERY_MAX_CELLS;
    }
    return cells;
}

void ak_battery_sample(ak_battery_t *battery, float pin_volts, float dt_s)
{
    battery->pin_volts = pin_volts;

    if (pin_volts < 0.0f) {
        /* The board could not read its own ADC. That is not a pack that is not
         * there, and it is not a voltage either, so the last good reading
         * stands - a sensor that stops answering is not an empty battery. */
        return;
    }

    float raw = pin_volts * battery->divider_ratio;
    battery->raw_volts = raw;

    if (raw > battery->reject_above_v) {
        battery->rejected++;
        return;
    }

    if (raw < battery->absent_below_v) {
        /*
         * Below the floor. These readings do not go into the filter at all:
         * a wire that came loose would otherwise drag a twelve-volt estimate
         * down by a fifth and take seconds to recover, and the pack's voltage
         * is not what changed. What has changed is whether there is a pack.
         */
        if (battery->absent_readings < AK_BATTERY_ABSENT_READINGS) {
            battery->absent_readings++;
        }
        if (battery->absent_readings >= AK_BATTERY_ABSENT_READINGS) {
            battery->volts = raw;
            battery->volts_per_cell = 0.0f;
            battery->cells = 0u;
            battery->state = AK_BATTERY_ABSENT;
        }
        return;
    }
    battery->absent_readings = 0u;

    int was_absent = battery->state == AK_BATTERY_ABSENT;
    if (battery->samples == 0u || was_absent) {
        /* The first reading is the estimate, not a step towards it. Starting
         * from zero and creeping up would take three seconds to say "12
         * volts", and during those three seconds the cell count would be
         * counted from the wrong voltage and latched - which is exactly the
         * bug the latch below is there to make impossible. A pack that has
         * just been plugged into a board that was reading nothing gets the
         * same treatment, for the same reason. */
        battery->volts = raw;
    } else {
        float alpha = battery->filter_per_s * dt_s;
        if (alpha > 1.0f) {
            alpha = 1.0f;
        }
        if (alpha < 0.0f) {
            alpha = 0.0f;
        }
        battery->volts += alpha * (raw - battery->volts);
    }
    battery->samples++;

    uint32_t cells;
    if (battery->cells_override != 0u) {
        cells = battery->cells_override;
    } else {
        cells = ak_battery_cell_count(battery->volts, battery->cell_detect_v);
        if (cells < battery->cells) {
            /* A pack never loses a cell in flight. The count is a high-water
             * mark, so a sag reads as a low cell voltage rather than as a
             * smaller pack that is doing fine. */
            cells = battery->cells;
        }
    }

    battery->cells = cells;
    battery->volts_per_cell = battery->volts / (float)cells;

    if (battery->volts_per_cell < battery->critical_cell_v) {
        battery->state = AK_BATTERY_CRITICAL;
    } else if (battery->volts_per_cell < battery->warn_cell_v) {
        battery->state = AK_BATTERY_WARN;
    } else {
        battery->state = AK_BATTERY_OK;
    }
}

const char *ak_battery_state_name(ak_battery_state_t state)
{
    switch (state) {
    case AK_BATTERY_ABSENT:
        return "nothing connected";
    case AK_BATTERY_OK:
        return "ok";
    case AK_BATTERY_WARN:
        return "low";
    case AK_BATTERY_CRITICAL:
        return "critical";
    }
    return "unknown";
}
