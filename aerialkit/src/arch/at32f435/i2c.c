#include "arch.h"

#include "i2c_timing.h"

/*
 * The barometer's bus, as far as this port has it: the pins, the clock, and the
 * timing register that decides how fast it runs - and, now, the transfers.
 *
 * Two things are this part's rather than the F405's, and both are larger than a
 * renaming. The timing is its own arithmetic, in i2c_timing.h, because this
 * is the newer I2C peripheral: one register describes the whole bus rather than
 * the F405's clock control and rise-time pair. A wrong one does not fail - it
 * runs the bus at the wrong speed, and a bus too fast looks exactly like a
 * barometer that is not answering.
 *
 * And the transfers are a different shape altogether, which is the part a port
 * written by pattern-matching the F405 would get wrong in a way that compiles:
 *
 *   - **the address, the direction and the byte count are one register write.**
 *     `ctrl2` carries all three, and `genstart` in the same value starts the
 *     transfer and performs the address phase in hardware. The F405 is fed an
 *     address byte through the data register; this part is told who it is
 *     talking to and how many bytes are moving, and drives the rest itself;
 *   - **the acknowledge is not the driver's to move.** On the F405 the master
 *     has to drop ACK before the second-to-last byte, and the manual gives a
 *     different rule for one byte, two bytes and more. Here the count decides:
 *     the peripheral acknowledges every byte but the last by itself and, with
 *     auto-stop on, ends the transfer when the count runs out;
 *   - **the flags are this part's**: room to send is `tdis`, a byte has arrived
 *     is `rdbf`, the last byte has left the shift register is `tdc`, and the
 *     transfer is over is `stopf`. The register number needs `tdc` before the
 *     repeated start, which is the one ordering here that is not obvious.
 *
 * The sequences are Artery's own master routines read as the oracle
 * (upstream/inav-9.1.0/src/main/drivers/i2c_application.c, `i2c_memory_write`
 * and `i2c_memory_read`, which are the two this bus actually needs), and the
 * host model in tests/host_i2c_model_at32.c executes them against a device that
 * answers - which is where a wrong flag, a missing clear or a count one byte out
 * shows up instead of on the bench.
 */

#define AK_I2C_GUARD 200000u

/*
 * Every register access in this file goes through these two, so that a host
 * build can put a *modelled bus* behind them and run the sequences rather than
 * leaving them to a scope. The accesses are spelled out rather than hidden
 * behind the register macros because half of them are read-modify-writes, and a
 * model that only saw the read would never learn that a start condition went
 * out.
 */
#ifdef AK_HOST_I2C_AT32
#include "host_i2c_model_at32.h"
#define i2c_read(base, offset)         host_i2c_at32_read((base), (offset))
#define i2c_write(base, offset, value) host_i2c_at32_write((base), (offset), (value))
#else
static uint32_t i2c_read(uint32_t base, uint32_t offset)
{
    return AK_REG32(base + offset);
}

static void i2c_write(uint32_t base, uint32_t offset, uint32_t value)
{
    AK_REG32(base + offset) = value;
}
#endif

/* Is this one of the three controllers on the part? The same guard the F405
 * port grew after a wrong base address wrote a start bit into the ADC's status
 * register: every caller here passes a board constant, and it is cheap to make
 * that a rule rather than a habit. */
static int known_controller(uint32_t i2c)
{
    return i2c == I2C1_BASE || i2c == I2C2_BASE || i2c == I2C3_BASE;
}

static int clock_enable(uint32_t i2c)
{
    switch (i2c) {
    case I2C1_BASE:
        CRM_APB1EN |= 1u << 21; /* CRM_I2C1_PERIPH_CLOCK = MAKE_VALUE(0x40, 21) */
        return 1;
    case I2C2_BASE:
        CRM_APB1EN |= 1u << 22; /* MAKE_VALUE(0x40, 22) */
        return 1;
    default:
        return 0;
    }
}

void ak_i2c_init(uint32_t i2c, ak_pin_t scl, ak_pin_t sda, uint8_t af,
                 uint32_t speed_hz)
{
    if (!clock_enable(i2c)) {
        return;
    }

    /*
     * Both lines are open drain, which on this part is the output-type bit per
     * pin - and it is not optional: I2C is a wired-and bus, every device pulls
     * it low and the pull-ups raise it, so a pin that drives it high fights
     * every other device on the wire. That is why the F405 port has a function
     * of its own for this and why this one uses the same idea with this part's
     * register.
     */
    ak_pin_af_open_drain(scl, af, AK_GPIO_PULL_UP);
    ak_pin_af_open_drain(sda, af, AK_GPIO_PULL_UP);

    /* Off while it is configured: the timing register may not be written with
     * the peripheral enabled. */
    i2c_write(i2c, AK_I2C_CTRL1_OFF, 0u);

    uint32_t clkctrl = ak_at32_i2c_clkctrl(ak_clk_apb1_hz(),
                                           speed_hz / 1000u);

    if (clkctrl == 0u) {
        return; /* the clock and the rate cannot be reconciled: leave it off */
    }
    i2c_write(i2c, AK_I2C_CLKCTRL_OFF, clkctrl);

    /* The digital noise filter, one peripheral clock of it: a bus with the
     * pull-ups this one has picks up short spikes, and the filter is the part's
     * own answer to them. */
    i2c_write(i2c, AK_I2C_CTRL1_OFF,
              (1u << AK_I2C_CTRL1_DFLT_SHIFT) | AK_I2C_CTRL1_I2CEN);
}

/* --- the transfers --------------------------------------------------------
 *
 * The shape is the F405 port's, because the *contract* is: a register number
 * and then the bytes, with a repeated start between the two halves of a read.
 * What the two ports do not share is how any of that reaches the part.
 */

/*
 * Wait for a flag, giving up after a bounded number of reads.
 *
 * The bound is the same kind of number the F405 port uses and for the same
 * reason: reaching it means the bus is stuck rather than slow, and a stuck bus
 * must cost a byte rather than the flight loop. This part's error flags are
 * checked here as well, which is what Artery's own `i2c_wait_flag()` does with
 * `I2C_EVENT_CHECK_ACKFAIL`: a device that stops answering sets ackfail, and
 * waiting for a flag that is never coming would waste the whole bound.
 */
static int wait_set(uint32_t i2c, uint32_t mask)
{
    uint32_t guard = AK_I2C_GUARD;

    while (guard-- > 0u) {
        uint32_t sts = i2c_read(i2c, AK_I2C_STS_OFF);

        if ((sts & mask) != 0u) {
            return 0;
        }
        if ((sts & AK_I2C_STS_ERR) != 0u) {
            return -1;
        }
    }
    return -1;
}

/* And the one flag that is waited on to go away: the bus being busy. */
static int wait_idle(uint32_t i2c)
{
    uint32_t guard = AK_I2C_GUARD;

    while (guard-- > 0u) {
        if ((i2c_read(i2c, AK_I2C_STS_OFF) & AK_I2C_STS_BUSYF) == 0u) {
            return 0;
        }
    }
    return -1;
}

/* Every flag this driver can leave set, cleared by name - this part has a
 * register for exactly that. */
static void clear_flags(uint32_t i2c)
{
    i2c_write(i2c, AK_I2C_CLR_OFF,
              AK_I2C_CLR_ADDRF | AK_I2C_CLR_ACKFAIL | AK_I2C_CLR_STOPF |
                  AK_I2C_CLR_BUSERR | AK_I2C_CLR_ARLOST);
}

/* Put ctrl2 back the way a transfer finds it: the address, the direction, the
 * count and the two condition bits are a transfer's, not the bus's. */
static void reset_ctrl2(uint32_t i2c)
{
    uint32_t value = i2c_read(i2c, AK_I2C_CTRL2_OFF);

    i2c_write(i2c, AK_I2C_CTRL2_OFF,
              value & ~(AK_I2C_CTRL2_KEEP | AK_I2C_CTRL2_NACKEN));
}

/*
 * A transfer that failed part way through leaves the bus in the middle of a
 * frame, where the next start would be read as a continuation of it. So: the
 * stop condition, the flags, and ctrl2 - and the bus is usable again. A slave
 * that is holding the clock low is not fixed by this (nothing in software can
 * be), which is why the wait above is bounded rather than patient.
 */
static void transfer_abort(uint32_t i2c)
{
    i2c_write(i2c, AK_I2C_CTRL2_OFF,
              i2c_read(i2c, AK_I2C_CTRL2_OFF) | AK_I2C_CTRL2_GENSTOP);
    (void)wait_set(i2c, AK_I2C_STS_STOPF);

    clear_flags(i2c);
    reset_ctrl2(i2c);
}

/*
 * Start a transfer: the address, the direction and the byte count in one write
 * to ctrl2, with genstart. `read` picks the direction bit; `bytes` is the count
 * the peripheral will move before it stops by itself.
 */
static void start_transfer(uint32_t i2c, uint8_t address, unsigned bytes,
                           int read, int auto_stop)
{
    uint32_t value = i2c_read(i2c, AK_I2C_CTRL2_OFF) & ~AK_I2C_CTRL2_KEEP;

    value |= ((uint32_t)address << 1) & AK_I2C_CTRL2_SADDR_MASK;
    value |= ((uint32_t)bytes & AK_I2C_CTRL2_CNT_MASK)
             << AK_I2C_CTRL2_CNT_SHIFT;
    value |= AK_I2C_CTRL2_GENSTART;
    if (read) {
        value |= AK_I2C_CTRL2_DIR;
    }
    if (auto_stop) {
        value |= AK_I2C_CTRL2_ASTOPEN;
    }

    i2c_write(i2c, AK_I2C_CTRL2_OFF, value);
}

static int finish_transfer(uint32_t i2c)
{
    if (wait_set(i2c, AK_I2C_STS_STOPF) != 0) {
        transfer_abort(i2c);
        return -1;
    }
    clear_flags(i2c);
    reset_ctrl2(i2c);
    return 0;
}

int ak_i2c_write_reg(uint32_t i2c, uint8_t address, uint8_t reg, uint8_t value)
{
    if (!known_controller(i2c)) {
        return -1;
    }
    if (wait_idle(i2c) != 0) {
        transfer_abort(i2c);
        return -1;
    }

    /* The register number and the value are one transfer of two bytes: this
     * part is told the count up front, so there is no "and now send another
     * byte" state to get wrong. */
    start_transfer(i2c, address, 2u, 0, 1);

    if (wait_set(i2c, AK_I2C_STS_TDIS) != 0) {
        transfer_abort(i2c);
        return -1;
    }
    i2c_write(i2c, AK_I2C_TXDT_OFF, reg);

    if (wait_set(i2c, AK_I2C_STS_TDIS) != 0) {
        transfer_abort(i2c);
        return -1;
    }
    i2c_write(i2c, AK_I2C_TXDT_OFF, value);

    /* Auto-stop: when the second byte has gone the peripheral ends the
     * transfer itself, and stopf is how that is known. */
    return finish_transfer(i2c);
}

int ak_i2c_read_reg(uint32_t i2c, uint8_t address, uint8_t reg, uint8_t *buf,
                    unsigned len)
{
    if (!known_controller(i2c)) {
        return -1;
    }
    if (len == 0u) {
        return 0;
    }
    if (len > AK_I2C_CTRL2_CNT_MASK) {
        /* The count is eight bits. Artery's own driver chains transfers through
         * the reload bit for anything longer; nothing on this bus is longer
         * than a few bytes, and a half-written reload path would be a reading
         * that stops early and looks like a sensor. */
        return -1;
    }
    if (wait_idle(i2c) != 0) {
        transfer_abort(i2c);
        return -1;
    }

    /* The register number, as a one-byte write with no stop at the end of it:
     * that is what makes the start below a repeated start rather than a new
     * transaction, and without it a sensor waits for data that never comes. */
    start_transfer(i2c, address, 1u, 0, 0);

    if (wait_set(i2c, AK_I2C_STS_TDIS) != 0) {
        transfer_abort(i2c);
        return -1;
    }
    i2c_write(i2c, AK_I2C_TXDT_OFF, reg);

    /* tdc, not tdis: tdis says the buffer is free, and the repeated start that
     * followed it would arrive while the register number is still in the shift
     * register - which is a read of whatever register the part had before. */
    if (wait_set(i2c, AK_I2C_STS_TDC) != 0) {
        transfer_abort(i2c);
        return -1;
    }

    /* And the read: the same address with the direction bit, and the count.
     * The peripheral acknowledges every byte but the last by itself and stops
     * when the count runs out, which is the whole of the difference from the
     * F405's manual acknowledge dance. */
    start_transfer(i2c, address, len, 1, 1);

    for (unsigned i = 0; i < len; i++) {
        if (wait_set(i2c, AK_I2C_STS_RDBF) != 0) {
            transfer_abort(i2c);
            return -1;
        }
        buf[i] = (uint8_t)(i2c_read(i2c, AK_I2C_RXDT_OFF) & 0xFFu);
    }

    return finish_transfer(i2c);
}
