#ifndef AK_HOST_I2C_MODEL_AT32_H
#define AK_HOST_I2C_MODEL_AT32_H

/*
 * An I2C bus, modelled, for the host build of tests/test_arch_at32.c.
 *
 * src/arch/at32f435/i2c.c routes every register access through
 * host_i2c_at32_read() and host_i2c_at32_write() when it is compiled with
 * -DAK_HOST_I2C_AT32. That seam is what makes the *transaction sequences*
 * runnable without a part, and on this peripheral they are not the F405's
 * sequences at all: the address, the direction and the byte count are one
 * register write, the address phase is performed by the hardware rather than by
 * a byte the driver feeds it, and the acknowledge for a read is a bit in that
 * same ctrl2 register rather than a ctrl1 flag the driver toggles as the bytes
 * arrive. A driver written by pattern-matching the F405's file would produce
 * something that compiles, runs against a modelled F405, and never moves this
 * bus at all.
 *
 * What is modelled, and why each piece is load-bearing:
 *
 *   - the flags the driver waits on: TDIS before each byte it sends, TDC when
 *     the register number has left the shift register, RDBF before each byte it
 *     receives, STOPF at the end of a transfer, and BUSYF while one is running.
 *     A driver that waits for a flag this part never raises is a driver that
 *     hangs, and this is where that shows up;
 *   - the address phase: a transfer to an address nobody answers sets ACKFAIL
 *     and nothing else, and the driver has to notice, clear it and put ctrl2
 *     back - which is the path a board with no barometer fitted takes at every
 *     boot;
 *   - auto-stop: the count written into ctrl2 decides how many bytes move, and
 *     the stop condition arrives by itself when it runs out. That is what makes
 *     the count load-bearing: too short and the last byte is cut off, too long
 *     and the driver waits for a byte no slave was going to send;
 *   - one or more slaves with a register file and a register pointer that
 *     auto-increments, which is what every sensor on this bus is;
 *   - a peripheral that is *off* until ctrl1 says otherwise: a transfer on a bus
 *     nobody enabled does nothing, and the driver's bounded waits expire. That
 *     is how a bring-up that forgot to enable the port looks.
 *
 * What it is not: a bus with timing. Bits do not take time, a slave that
 * stretches the clock is not modelled, arbitration and bus errors are flags a
 * test can only set by hand, and nothing here can say whether the CLKCTRL value
 * the timing arithmetic produced is 400 kHz on a real wire. [21-port-on-the-host.md]
 * says the same about the F405's model.
 */

#include <stdint.h>

#define HOST_I2C_AT32_SLAVES 4u
#define HOST_I2C_AT32_REGISTERS 256u

/* The model, from the peripheral's side. The test supplies both ends of every
 * transaction: the driver under test writes registers, and the slaves below
 * answer as a sensor would.
 *
 * One controller is modelled and `base` is taken but not used: it is there so
 * that the seam reads like the register macros it stands in for. */
uint32_t host_i2c_at32_read(uint32_t base, uint32_t offset);
void     host_i2c_at32_write(uint32_t base, uint32_t offset, uint32_t value);

void host_i2c_at32_reset(void);

/* Attach a slave at a 7-bit address with a register file. Returns its index, or
 * -1 when the bus is full. */
int host_i2c_at32_attach(uint8_t address, const uint8_t *registers,
                         unsigned count);

/* The two registers ak_i2c_init() writes, for the checks that assert what
 * reached the part: the control register (the enable and the digital filter)
 * and the timing register the arithmetic produced. */
uint32_t host_i2c_at32_ctrl1(void);
uint32_t host_i2c_at32_clkctrl(void);

/* What the last transaction did: the address byte the master sent (7 bits), the
 * register the slave's pointer ended on, and how many data bytes moved each
 * way. */
uint8_t  host_i2c_at32_last_address(void);
uint8_t  host_i2c_at32_last_register(void);
unsigned host_i2c_at32_last_written(void);
unsigned host_i2c_at32_last_read(void);

/* One attached slave's register file, and whether it was addressed. */
uint8_t host_i2c_at32_register(unsigned index, uint8_t reg);
int     host_i2c_at32_addressed(unsigned index);

/*
 * A bus that stops answering, which is what this port's bound is for.
 *
 * The same seam as the F405's model, for the same reason: the wing's barometer
 * is on this bus, and a sensor that dies must cost a byte rather than the
 * flight loop. Zero is a bus that never comes up; each step beyond it answers
 * one more status change and then dies, which walks the sequence - the start,
 * the address phase, each byte's flag, the stop - one wait at a time.
 */
#define HOST_I2C_AT32_ANSWERS_ALL 0xFFu
void host_i2c_at32_stall_after(unsigned flags);

/*
 * The bus is busy, and stays busy: a slave holding the clock down, or another
 * master in the middle of a transfer. Nothing software can do about it, which
 * is why `wait_idle()` at the top of every transfer is bounded - the answer is
 * to give up and abort rather than to wait for ever. It is the one failure a
 * stall cannot produce: a stall is a bus that dies *during* a transfer, and
 * this is a bus that never became free to start one.
 */
void host_i2c_at32_bus_busy(int busy);

#endif /* AK_HOST_I2C_MODEL_AT32_H */
