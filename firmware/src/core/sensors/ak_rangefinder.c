#include "ak_rangefinder.h"

/*
 * The rangefinder service: one part on a bus, one reading at a time, and the
 * filter between the part and the two things that use the number.
 *
 * There is deliberately nothing here that decides anything about the flight.
 * Whether "0.2 metres" means the aircraft is on the ground is a question about
 * the aircraft and the navigator, and it is answered where the navigator and
 * the flight core can be seen together (main.c). What lives here is the part
 * of the answer that is about the *sensor*: is it fitted, is it answering, was
 * that reading possible, and how long ago was it.
 */

const ak_rangefinder_driver_t *ak_rangefinder_detect(const ak_bus_t *bus)
{
    if (bus == 0) {
        return 0;
    }
    for (unsigned i = 0; ak_range_drivers[i] != 0; i++) {
        if (ak_range_drivers[i]->probe(bus) == 0) {
            return ak_range_drivers[i];
        }
    }
    return 0;
}

int ak_rangefinder_open(ak_rangefinder_t *rgf, const ak_bus_t *bus,
                        ak_printf_fn out)
{
    const ak_rangefinder_driver_t *driver = ak_rangefinder_detect(bus);

    rgf->bus = bus;
    rgf->driver = driver;
    rgf->present = driver != 0;
    rgf->distance_mm = AK_RANGE_NONE;
    rgf->last_ms = 0;
    rgf->poll_ms = 0;
    rgf->polled = 0;
    rgf->samples = 0;
    rgf->out_of_range = 0;
    rgf->faults = 0;
    rgf->rejected = 0;

    if (driver != 0 && out != 0) {
        out("rangefinder: %s at 0x%02x, up to %d.%02d m\r\n", driver->name,
            (unsigned)driver->address, (int)(driver->max_mm / 1000),
            (int)((driver->max_mm % 1000) / 10));
    }
    return driver != 0 ? 0 : -1;
}

int ak_rangefinder_read(ak_rangefinder_t *rgf, uint32_t now_ms,
                        int32_t *distance_mm)
{
    int32_t read_mm = AK_RANGE_NONE;
    int got;

    if (!rgf->present) {
        return AK_RANGE_IDLE;
    }
    /* The part is asked at its own rate and no faster, and the answer to "was
     * it asked too soon" is about the *asking*, not about whether the last
     * answer was kept: a part whose readings are being rejected is still a
     * part being polled. */
    if (rgf->polled &&
        (uint32_t)(now_ms - rgf->poll_ms) < rgf->driver->period_ms) {
        return AK_RANGE_IDLE;
    }
    rgf->poll_ms = now_ms;
    rgf->polled = 1;

    got = rgf->driver->read(rgf->bus, &read_mm);
    if (got < 0) {
        rgf->faults++;
        /* A part that stops answering is not a part saying "the ground is
         * close". The last distance goes with it: a landing may not be
         * decided on a reading the part is no longer making. */
        rgf->distance_mm = AK_RANGE_NONE;
        return AK_RANGE_FAULT;
    }
    if (got == 0) {
        rgf->out_of_range++;
        rgf->distance_mm = AK_RANGE_NONE;
        return AK_RANGE_NOTHING;
    }

    /*
     * The spike filter: how fast the ground would have had to move to explain
     * this reading, over the time since the last one. See the header for why
     * it is a rate and not a distance.
     */
    if (rgf->distance_mm >= 0 && now_ms != rgf->last_ms) {
        int32_t step = read_mm - rgf->distance_mm;
        uint32_t elapsed_ms = now_ms - rgf->last_ms;
        int32_t limit_mm;

        if (step < 0) {
            step = -step;
        }
        limit_mm = (int32_t)((int64_t)AK_RANGE_SPIKE_MM_S *
                             (int64_t)elapsed_ms / 1000);
        if (step > limit_mm) {
            rgf->rejected++;
            return AK_RANGE_NOTHING;
        }
    }

    if (read_mm < rgf->driver->min_mm) {
        /* Below the part's own minimum the number is the inside of the case,
         * not the ground. Reported as the minimum rather than as a fault: a
         * part that is right about "very close" is answering the landing
         * question, which is the only one being asked this close. */
        read_mm = rgf->driver->min_mm;
    }

    rgf->distance_mm = read_mm;
    rgf->last_ms = now_ms;
    rgf->samples++;
    if (distance_mm != 0) {
        *distance_mm = read_mm;
    }
    return AK_RANGE_READING;
}

int ak_rangefinder_valid(const ak_rangefinder_t *rgf, uint32_t now_ms,
                         uint32_t hold_ms)
{
    if (!rgf->present || rgf->distance_mm < 0) {
        return 0;
    }
    return (uint32_t)(now_ms - rgf->last_ms) <= hold_ms;
}
