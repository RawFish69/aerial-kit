#include "ak_rangefinder.h"

/*
 * TOF10120: a small time-of-flight part that answers two bytes and a
 * question, "how far to the ground below me", for about the price of a
 * receiver. It is the first rangefinder this firmware knows, and the one whose
 * protocol was read out of the reference implementation rather than recalled:
 *
 *   the register map, the two-byte big-endian distance, the configuration
 *   register that has to be written before the part answers, the address
 *   register used as the "is anything there" answer, and the 100 ms task
 *   period:
 *     INAV 9.1.0 src/main/drivers/rangefinder/rangefinder_tof10120_i2c.c @
 *     e519b69
 *
 * docs/03-attribution.md carries the revision. No code was copied: the same
 * transactions are written here against the part's own register numbers, with
 * the names and the error handling this repository uses. INAV's driver reads
 * the *raw* distance register, which is the one this reads too, and does its
 * own filtering above the driver - so the spike filter in ak_rangefinder.c is
 * standing where INAV's rangefinder task stands, and not on top of a filtered
 * number that is not the one being measured.
 *
 * A two-metre part is the whole of the first question. It cannot fly a descent
 * - the barometer does that, and has since the return was written - and it is
 * the last two metres of one, which is where a filtered barometer with a
 * leaking reference is worth less than a tape measure.
 */

#define TOF10120_ADDRESS             0x52u

/* "Sending method config": what has to be written before the part will answer
 * at all. INAV writes it in both its detect and its init. */
#define TOF10120_REG_SENDING_METHOD  0x09u
#define TOF10120_SENDING_METHOD_AUTO 0x01u
/* The address register, read back as the "something is there" answer. Its
 * value is the part's own I2C address, which is never zero. */
#define TOF10120_REG_ADDRESS         0x0Fu
/* The raw distance, two bytes, millimetres, big endian first. */
#define TOF10120_REG_DISTANCE        0x00u

#define TOF10120_MAX_RANGE_MM        2000
/* The part's blind zone. Inside it the reading is its own case rather than the
 * ground, and the service reports the minimum instead of a number nobody
 * measured. Datasheet rather than INAV: INAV's driver has no minimum, which is
 * why this one is written down here as its own fact. */
#define TOF10120_MIN_RANGE_MM        30
#define TOF10120_PERIOD_MS           100u

static int tof10120_write(const ak_bus_t *bus, uint8_t reg, uint8_t value)
{
    return ak_bus_write(bus, reg, value) == 0 ? 0 : -1;
}

static int tof10120_read_regs(const ak_bus_t *bus, uint8_t reg, uint8_t *buf,
                              unsigned len)
{
    return ak_bus_read(bus, reg, buf, len) == 0 ? 0 : -1;
}

static int tof10120_configure(const ak_bus_t *bus)
{
    if (tof10120_write(bus, TOF10120_REG_SENDING_METHOD,
                       TOF10120_SENDING_METHOD_AUTO) != 0) {
        return -1;
    }
    ak_bus_delay_ms(bus, 100);
    return 0;
}

static int tof10120_probe(const ak_bus_t *bus)
{
    uint8_t answer = 0;

    if (tof10120_configure(bus) != 0) {
        return -1;
    }
    if (tof10120_read_regs(bus, TOF10120_REG_ADDRESS, &answer, 1) != 0) {
        return -1;
    }
    /* Zero is what a floating bus reads, and no TOF10120 answers with it. */
    return answer != 0 ? 0 : -1;
}

static int tof10120_read(const ak_bus_t *bus, int32_t *distance_mm)
{
    uint8_t bytes[2];
    int32_t distance;

    if (tof10120_read_regs(bus, TOF10120_REG_DISTANCE, bytes, 2) != 0) {
        return -1;
    }
    distance = ((int32_t)bytes[0] << 8) | (int32_t)bytes[1];
    /* INAV's rule, and the part's: at or past the end of its range there is
     * nothing to report, and a number there is the part guessing. */
    if (distance >= TOF10120_MAX_RANGE_MM) {
        return 0;
    }
    *distance_mm = distance;
    return 1;
}

const ak_rangefinder_driver_t ak_range_tof10120 = {
    .name = "tof10120",
    .address = TOF10120_ADDRESS,
    .period_ms = TOF10120_PERIOD_MS,
    .max_mm = TOF10120_MAX_RANGE_MM,
    .min_mm = TOF10120_MIN_RANGE_MM,
    .probe = tof10120_probe,
    .read = tof10120_read,
};

/* One part, and room for the next one to be a line rather than a rewrite: the
 * probe order is the table order, exactly as the barometers are ordered. */
const ak_rangefinder_driver_t *const ak_range_drivers[] = {
    &ak_range_tof10120,
    0,
};
