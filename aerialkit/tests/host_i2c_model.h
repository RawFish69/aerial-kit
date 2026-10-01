#ifndef AK_HOST_I2C_MODEL_H
#define AK_HOST_I2C_MODEL_H

/*
 * An I2C bus, modelled, for the host build of tests/test_arch.c.
 *
 * i2c.c routes every register access through host_i2c_read() and
 * host_i2c_write() when it is compiled with -DAK_HOST_I2C. That seam is what
 * makes the *transaction sequences* runnable without a part: the start, the
 * address byte, the register number, the repeated start, the acknowledge that
 * moves for a two-byte read, the STOP. Those are the lines of RM0090 27.3.3
 * that a scope would otherwise have to confirm, and the driver has never been
 * able to run them.
 *
 * What is modelled, and why each piece is load-bearing:
 *
 *   - the flags the driver waits on: SB after a start, ADDR after an address
 *     that answered, TXE and BTF while transmitting, RXNE and BTF while
 *     receiving. A driver that waits for a flag the part never sets is a
 *     driver that hangs, and this is where that shows up.
 *   - ADDR being cleared by reading SR1 and then SR2, in that order, which is
 *     the one register interaction in the file nobody would guess.
 *   - AF, set when an address nobody answers goes out, because that is the
 *     error path the driver has to clear by hand.
 *   - one or more slave devices with a register file and a register pointer
 *     that auto-increments, which is what every sensor on this bus is.
 *
 * What it is not: a bus with timing. Bits do not take time, a slave that
 * stretches the clock is not modelled, and nothing here can say whether the
 * CCR and TRISE values produce 400 kHz on a real wire. The port page says so
 * too (docs/21-port-on-the-host.md).
 */

#include <stdint.h>

#define HOST_I2C_SLAVES 4u
#define HOST_I2C_REGISTERS 256u

/* The model, from the peripheral's side. The test supplies both ends of every
 * transaction: the driver under test writes registers, and the slaves below
 * answer as a sensor would. */
uint32_t host_i2c_read(uint32_t base, uint32_t offset);
void     host_i2c_write(uint32_t base, uint32_t offset, uint32_t value);

void host_i2c_model_reset(void);

/* Attach a slave at a 7-bit address with a register file. Returns its index, or
 * -1 when the bus is full. The registers are the device's contents, indexed by
 * the register number the master sends. */
int host_i2c_attach(uint8_t address, const uint8_t *registers, unsigned count);

/* What the last transaction did, for a test to assert on: the address byte the
 * master sent (7 bits), the register it named, and how many data bytes it
 * wrote. `host_i2c_last_read` counts the bytes the master read. */
uint8_t  host_i2c_last_address(void);
uint8_t  host_i2c_last_register(void);
unsigned host_i2c_last_written(void);
unsigned host_i2c_last_read(void);

/* The bytes one attached slave has now, for the checks that are about what
 * arrived rather than what the master thinks it did. */
uint8_t host_i2c_register(unsigned index, uint8_t reg);
int     host_i2c_addressed(unsigned index);

/*
 * A bus that stops answering, which is the failure the port's guards exist
 * for.
 *
 * Every wait in i2c.c is bounded, and reaching the bound resets the peripheral
 * rather than returning half a reading. That promise is worth nothing if the
 * bound is never reached, and nothing on this machine can hold SDA low - so the
 * model stops answering after a chosen number of status flags. Zero is a bus
 * that never comes up; each step beyond it lets one more wait succeed and then
 * dies, which is how the start, the address, the register number, the repeated
 * start and the byte a read is waiting for are each asked in turn.
 */
#define HOST_I2C_ANSWERS_ALL 0xFFu
void host_i2c_model_stall_after(unsigned flags);

/*
 * The bus *wires*, which are not the peripheral.
 *
 * Everything above is the part's own registers. Bus recovery - i2c.c's
 * ak_i2c_bus_recover - is the one thing in that file that cannot be reached
 * through them, because a slave holding SDA down is exactly what SWRST does
 * not touch: the slave is holding the *wire*, so the driver drives the wire.
 * A host build that could not hold a line down could not run that path at all,
 * and the path would then only ever run on a bench with a broken sensor on it.
 *
 * So: the test says which pins are SCL and SDA and which line a slave is
 * holding, and the model reports what the master drove and how many clocks it
 * sent. Open drain, as on the wire: a line is low if anyone is pulling it
 * down, and high only if nobody is.
 */
void host_i2c_gpio_attach(uint32_t scl_port, uint8_t scl_pin,
                          uint32_t sda_port, uint8_t sda_pin);

/*
 * A slave holding SDA down, mid-byte. `release_after` is how many SCL falling
 * edges it takes before it lets go, or 0 for never - which is what tells the
 * two failures apart: a slave that can be clocked free, and a wire that is
 * shorted to ground, where nine clocks change nothing and the honest answer is
 * -1 rather than a confident "recovered".
 */
void host_i2c_gpio_hold_sda(unsigned release_after);

/* The seam i2c.c's bus_line/bus_drive are defined against under AK_HOST_I2C. */
int  host_i2c_gpio_line(uint32_t port, uint8_t pin);
void host_i2c_gpio_drive(uint32_t port, uint8_t pin, int level);

/* What the recovery did, for a test to assert on rather than infer. */
unsigned host_i2c_gpio_clocks(void);
int      host_i2c_gpio_saw_stop(void);

#endif /* AK_HOST_I2C_MODEL_H */
