#ifndef AK_HOST_SPI_MODEL_H
#define AK_HOST_SPI_MODEL_H

#include <stdint.h>

/*
 * The sensor bus behind the two ARM ports' SPI, modelled, for the host build of
 * tests/test_arch.c and tests/test_arch_at32.c.
 *
 * `spi.c` routes every register access through two functions when it is
 * compiled with `-DAK_HOST_SPI` / `-DAK_HOST_SPI_AT32` - the same seam the F405
 * and AT32 I2C ports have - so the *real* driver runs its *real* sequence
 * against a shift register that answers rather than a page of memory that does
 * not. That matters because of what is on the other end of this bus on the
 * wing's board: the ICM-42688-P, which is the sensor the aircraft is flown on.
 * Before this, the board's own IMU bus - its read, its write and its burst -
 * had never been executed anywhere: the board declares the part fitted, the
 * host has no part, and the port's register block answers nothing, so the
 * first thing that bus would ever have done is the first thing it did on the
 * bench.
 *
 * What is modelled: a register file that answers reads and takes writes, a
 * register address that auto-increments as a sensor does, the read flag in the
 * address byte, and the three shapes the drivers use - an address on its own
 * followed by data (`imu_bus_read`), an address and data in one transfer
 * (`imu_bus_write`), and an address followed by chunks (`write_burst` for the
 * BMI270's eight-kilobyte upload).
 *
 * What is not: timing, mode or clock polarity - the port's own register tests
 * pin those against the mapped block, including its loop and its error paths -
 * and the chip select, which is a GPIO the model cannot see. A transfer
 * boundary is therefore the *shape* of the bytes, which is what the drivers
 * here produce; the .c file says why it is a rule rather than a fact.
 */

void     host_spi_reset(void);

/* The register file behind the device. */
void     host_spi_set_register(uint8_t reg, uint8_t value);
uint8_t  host_spi_register(uint8_t reg);

/* What has gone out, for the checks that are about the wire rather than the
 * part: the last transfer's bytes and how many there have been. */
unsigned host_spi_transfers(void);
unsigned host_spi_last_len(void);
const uint8_t *host_spi_last_tx(void);

/*
 * Whether a part is on the bus at all. The default is *no*: a transfer still
 * shifts - the port puts a pull-up on MISO, so an empty bus reads 0xFF, which
 * is what a person's spare board does - and what says "no part" is the driver's
 * own who-am-i check rather than a transfer that fails. A test that wants a
 * sensor calls this with 1 and fills the register file.
 */
void host_spi_set_present(int present);

/* Or the voltage on MISO is the voltage on MOSI, which is one jumper wire and
 * the board's own loopback check. */
void host_spi_set_echo(int echo);

/* The device half, called by spi.c's transfer when the host build is asked for
 * one. Returns 0 for the same reason the port does. */
int host_spi_transfer(uint32_t base, const uint8_t *tx, uint8_t *rx,
                      unsigned len);

#endif /* AK_HOST_SPI_MODEL_H */
