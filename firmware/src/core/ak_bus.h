#ifndef AK_CORE_AK_BUS_H
#define AK_CORE_AK_BUS_H

#include <stdint.h>

/*
 * What a sensor driver is allowed to assume about the wire.
 *
 * A driver asks for a register by its number in the datasheet and never learns
 * whether it is on SPI or I2C, or which pins, or which chip select. Everything
 * transport-shaped lives behind these three functions:
 *
 *   read(ctx, reg, buf, len)   `len` bytes starting at `reg`
 *   write(ctx, reg, value)     one byte
 *   write_burst(ctx, reg, buf, len)   `len` bytes starting at `reg`, or null
 *   delay_ms(ctx, ms)          a wait, so a driver can follow a datasheet
 *
 * `write_burst` is optional and exists for one driver: the BMI270 takes eight
 * kilobytes of its own firmware through a single register, and writing that a
 * byte at a time is eight thousand transactions where the transport can do one
 * that never lifts the chip select. A bus that cannot do it says so with a
 * null pointer, and a driver that wants it falls back to the byte at a time -
 * which is what a fake bus in a test does, and which is how the two paths are
 * shown to agree.
 *
 * The read-flag convention belongs to the transport, not the driver: an MPU
 * style SPI device wants bit 7 set for a read, and the SPI bus layer sets it.
 * That is why the driver passes a plain 7-bit register number and why a fake
 * bus in a test needs to know nothing about SPI.
 */

typedef struct {
    int (*read)(void *ctx, uint8_t reg, uint8_t *buf, unsigned len);
    int (*write)(void *ctx, uint8_t reg, uint8_t value);
    /* Optional. Null means "one byte at a time, please". */
    int (*write_burst)(void *ctx, uint8_t reg, const uint8_t *buf,
                       unsigned len);
    void (*delay_ms)(void *ctx, unsigned ms);
    void *ctx;
    /*
     * Optional. The 7-bit I2C address this bus is talking to, for a report to
     * print - null on SPI, and on an I2C bus whose address nobody needs to see.
     *
     * A driver never calls it: which address a part answers on is the board's
     * business, the same as which pins. It exists because one board probes for
     * its IMU's address rather than stating it (the Feather, where a BNO055
     * can be strapped to 0x28 or 0x29 and an LSM6DSO to 0x6A or 0x6B), and the
     * address the probe settled on is the one fact a bench session needs to
     * see before it checks a strap or a cable. A function rather than a field
     * because that address is chosen at run time and the bus is const.
     */
    uint8_t (*address)(void *ctx);
} ak_bus_t;

static inline int ak_bus_read(const ak_bus_t *bus, uint8_t reg, uint8_t *buf,
                              unsigned len)
{
    return bus->read(bus->ctx, reg, buf, len);
}

static inline int ak_bus_write(const ak_bus_t *bus, uint8_t reg, uint8_t value)
{
    return bus->write(bus->ctx, reg, value);
}

/* A burst if the transport has one, byte at a time if it does not. The return
 * is the same either way, so a driver never has to know which it got. */
static inline int ak_bus_write_burst(const ak_bus_t *bus, uint8_t reg,
                                     const uint8_t *buf, unsigned len)
{
    if (bus->write_burst != 0) {
        return bus->write_burst(bus->ctx, reg, buf, len);
    }
    for (unsigned i = 0; i < len; i++) {
        if (bus->write(bus->ctx, reg, buf[i]) != 0) {
            return -1;
        }
    }
    return 0;
}

static inline void ak_bus_delay_ms(const ak_bus_t *bus, unsigned ms)
{
    if (bus->delay_ms != 0) {
        bus->delay_ms(bus->ctx, ms);
    }
}

/* The address the bus reports, or 0 when it reports none (0 is the general
 * call address, which no part in this firmware is strapped to). */
static inline uint8_t ak_bus_address(const ak_bus_t *bus)
{
    if (bus->address != 0) {
        return bus->address(bus->ctx);
    }
    return 0;
}

#endif /* AK_CORE_AK_BUS_H */
