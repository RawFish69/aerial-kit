/*
 * The modelled I2C bus behind the seam in src/arch/at32f435/i2c.c. See
 * host_i2c_model_at32.h for what it is, what it is not, and why each flag it
 * raises is load-bearing.
 *
 * The register offsets and bit positions come from the port's own regs.h - they
 * are the hardware contract. What this file owns is the *behaviour*: what a
 * write of genstart does, where the byte count takes effect, when the stop
 * condition arrives, and which read clears which flag.
 */

#include <string.h>

#include "../src/arch/at32f435/regs.h"
#include "host_i2c_model_at32.h"

typedef struct {
    int      attached;
    uint8_t  address;                    /* 7 bits */
    uint8_t  registers[HOST_I2C_AT32_REGISTERS];
    uint8_t  pointer;                    /* where the next byte comes from */
    int      addressed;                  /* it answered this transfer */
} host_slave_t;

static host_slave_t slaves[HOST_I2C_AT32_SLAVES];

/* The master, as the driver sees it. Everything the driver writes lives here
 * rather than in memory, so a check can ask what the port was configured with
 * and what a transfer did. */
static uint32_t ctrl1;
static uint32_t ctrl2;
static uint32_t clkctrl;
static uint32_t sts;

/* The transfer in progress: which direction, how many bytes are left, and
 * whether the register number has been sent yet. */
enum { BUS_IDLE = 0, BUS_WRITE, BUS_READ };
static int      mode;
static unsigned remaining;
static int      register_named;

static host_slave_t *current;

static uint8_t  last_address;
static unsigned last_written;
static unsigned last_read;

/* How many status *changes* this bus answers before it stops. Counting the
 * driver's reads instead counts a wait that is timing out two hundred thousand
 * times as two hundred thousand answers - see the same counter in
 * host_i2c_model.c, where that mistake was made on the F405's bus first. */
static unsigned stall_after = HOST_I2C_AT32_ANSWERS_ALL;
static unsigned flags_answered;
static uint32_t last_sts_seen;
static int bus_busy;

void host_i2c_at32_stall_after(unsigned flags)
{
    stall_after = flags;
    flags_answered = 0u;
    last_sts_seen = sts;
}

void host_i2c_at32_bus_busy(int busy)
{
    bus_busy = busy;
}

void host_i2c_at32_reset(void)
{
    memset(slaves, 0, sizeof slaves);
    ctrl1 = 0u;
    ctrl2 = 0u;
    clkctrl = 0u;
    sts = 0u;
    mode = BUS_IDLE;
    remaining = 0u;
    register_named = 0;
    current = 0;
    last_address = 0u;
    last_written = 0u;
    last_read = 0u;
    stall_after = HOST_I2C_AT32_ANSWERS_ALL;
    flags_answered = 0u;
    last_sts_seen = 0u;
    bus_busy = 0;
}

int host_i2c_at32_attach(uint8_t address, const uint8_t *registers,
                         unsigned count)
{
    for (unsigned i = 0u; i < HOST_I2C_AT32_SLAVES; i++) {
        if (!slaves[i].attached) {
            slaves[i].attached = 1;
            slaves[i].address = address;
            if (count > HOST_I2C_AT32_REGISTERS) {
                count = HOST_I2C_AT32_REGISTERS;
            }
            memset(slaves[i].registers, 0, sizeof slaves[i].registers);
            if (registers != 0 && count > 0u) {
                memcpy(slaves[i].registers, registers, count);
            }
            slaves[i].pointer = 0u;
            slaves[i].addressed = 0;
            return (int)i;
        }
    }
    return -1;
}

uint32_t host_i2c_at32_ctrl1(void) { return ctrl1; }
uint32_t host_i2c_at32_clkctrl(void) { return clkctrl; }
uint8_t  host_i2c_at32_last_address(void) { return last_address; }

uint8_t host_i2c_at32_last_register(void)
{
    return current != 0 ? current->pointer : 0u;
}

unsigned host_i2c_at32_last_written(void) { return last_written; }
unsigned host_i2c_at32_last_read(void) { return last_read; }

uint8_t host_i2c_at32_register(unsigned index, uint8_t reg)
{
    return index < HOST_I2C_AT32_SLAVES ? slaves[index].registers[reg] : 0u;
}

int host_i2c_at32_addressed(unsigned index)
{
    return index < HOST_I2C_AT32_SLAVES ? slaves[index].addressed : 0;
}

/* The stop condition, when the count runs out or when the driver asks for one
 * by hand. The bus goes idle and the modules that were mid-transfer stop being
 * mid-transfer - which is what makes a transfer that was cut short visible to
 * the next one rather than leaving it talking to a slave that is still
 * listening. */
static void complete_stop(void)
{
    sts |= AK_I2C_STS_STOPF;
    sts &= ~(AK_I2C_STS_BUSYF | AK_I2C_STS_TDIS | AK_I2C_STS_RDBF);
    mode = BUS_IDLE;
    remaining = 0u;
}

static host_slave_t *slave_at(uint8_t address)
{
    for (unsigned i = 0u; i < HOST_I2C_AT32_SLAVES; i++) {
        if (slaves[i].attached && slaves[i].address == address) {
            return &slaves[i];
        }
    }
    return 0;
}

/*
 * The address phase, which this peripheral performs itself: writing ctrl2 with
 * genstart is the whole of it. The direction and the count come from the same
 * register, which is why a driver that sets them up separately (as the F405's
 * does, byte by byte) cannot drive this part.
 */
static void begin_transfer(uint32_t value)
{
    if ((ctrl1 & AK_I2C_CTRL1_I2CEN) == 0u) {
        /* The peripheral is off. Hardware does nothing at all here, so the
         * driver's waits are what has to notice - and they are bounded. */
        return;
    }

    unsigned wanted = (value & AK_I2C_CTRL2_SADDR_MASK) >> 1;
    int reading = (value & AK_I2C_CTRL2_DIR) != 0u;

    last_address = (uint8_t)wanted;
    last_written = 0u;
    last_read = 0u;
    register_named = 0;
    remaining = (value >> AK_I2C_CTRL2_CNT_SHIFT) & AK_I2C_CTRL2_CNT_MASK;

    current = slave_at((uint8_t)wanted);
    if (current == 0) {
        /* Nobody answered: the flag the driver has to notice and clear, and no
         * room to send or receive anything. */
        sts |= AK_I2C_STS_ACKFAIL;
        sts &= ~(AK_I2C_STS_TDIS | AK_I2C_STS_RDBF | AK_I2C_STS_ADDRF);
        mode = BUS_IDLE;
        remaining = 0u;
        return;
    }

    current->addressed = 1;
    sts |= AK_I2C_STS_ADDRF | AK_I2C_STS_BUSYF;
    mode = reading ? BUS_READ : BUS_WRITE;

    if (reading) {
        sts &= ~AK_I2C_STS_TDIS;
        if (remaining > 0u) {
            sts |= AK_I2C_STS_RDBF;
        }
    } else {
        /* Room for the first byte to send - the register number. */
        sts |= AK_I2C_STS_TDIS | AK_I2C_STS_TDBE;
    }
}

static void write_txdt(uint32_t value)
{
    uint8_t byte = (uint8_t)(value & 0xFFu);

    if (mode != BUS_WRITE || current == 0 || remaining == 0u) {
        return; /* nothing was set up to send: a write the part ignores */
    }

    if (!register_named) {
        /* The first byte of a write names the register, which is the shape
         * every sensor on this bus has. */
        current->pointer = byte;
        register_named = 1;
    } else {
        current->registers[current->pointer] = byte;
        current->pointer = (uint8_t)(current->pointer + 1u);
    }
    last_written++;
    remaining--;

    /* The buffer is busy until the byte has left, and then there is room for
     * the next one - or, if that was the last, the transfer is over and the
     * stop condition goes out by itself when auto-stop is on. */
    sts &= ~AK_I2C_STS_TDIS;
    if (remaining > 0u) {
        sts |= AK_I2C_STS_TDIS;
    } else {
        sts |= AK_I2C_STS_TDC;
        if ((ctrl2 & AK_I2C_CTRL2_ASTOPEN) != 0u) {
            complete_stop();
        }
    }
}

static uint32_t read_rxdt(void)
{
    if (mode != BUS_READ || current == 0 || remaining == 0u) {
        return 0u;
    }

    uint8_t byte = current->registers[current->pointer];

    current->pointer = (uint8_t)(current->pointer + 1u);
    last_read++;
    remaining--;

    if (remaining > 0u) {
        sts |= AK_I2C_STS_RDBF;
    } else {
        sts &= ~AK_I2C_STS_RDBF;
        sts |= AK_I2C_STS_TDC;
        if ((ctrl2 & AK_I2C_CTRL2_ASTOPEN) != 0u) {
            complete_stop();
        }
    }
    return (uint32_t)byte;
}

static void write_ctrl2(uint32_t value)
{
    ctrl2 = value;

    if ((value & AK_I2C_CTRL2_GENSTART) != 0u) {
        /* The start bit is cleared by the hardware once it is on the wire, and
         * the driver depends on that: `transfer_abort()` sets the stop bit on
         * whatever ctrl2 holds, and a start bit still sitting in there made
         * that write look like another transfer rather than a stop - which
         * left the model mid-transfer with the bus busy, so the next
         * transaction timed out. The F405's model had already learned the same
         * thing about its own start bit. */
        ctrl2 &= ~AK_I2C_CTRL2_GENSTART;
        begin_transfer(value);
    } else if ((value & AK_I2C_CTRL2_GENSTOP) != 0u) {
        complete_stop();
        ctrl2 &= ~AK_I2C_CTRL2_GENSTOP;
    }
}

uint32_t host_i2c_at32_read(uint32_t base, uint32_t offset)
{
    (void)base;

    switch (offset) {
    case 0x00u:
        return ctrl1;
    case 0x04u:
        return ctrl2;
    case 0x10u:
        return clkctrl;
    case 0x18u:
        if (bus_busy) {
            /* Somebody else has the bus: the flag `wait_idle()` is waiting to
             * see clear never does. */
            return sts | AK_I2C_STS_BUSYF;
        }
        if (flags_answered >= stall_after) {
            /* The bus has stopped answering: nothing this register used to say
             * is true any more, so the port's wait runs to its bound and gives
             * up - which is the difference between a sensor that died and an
             * aircraft that stops flying because of it. */
            return 0u;
        }
        if (sts != last_sts_seen) {
            last_sts_seen = sts;
            flags_answered++;
        }
        return sts;
    case 0x24u:
        return read_rxdt();
    default:
        return 0u;
    }
}

void host_i2c_at32_write(uint32_t base, uint32_t offset, uint32_t value)
{
    (void)base;

    switch (offset) {
    case 0x00u:
        ctrl1 = value;
        break;
    case 0x04u:
        write_ctrl2(value);
        break;
    case 0x10u:
        clkctrl = value;
        break;
    case 0x18u:
        /* A one written to a flag bit sets it, which is what this part's own
         * recovery code does to put the send status back after an error. */
        sts |= value;
        break;
    case 0x1Cu:
        /* And the clear register is the other convention: a one clears. Both
         * exist on this part, one register apart, which is worth knowing
         * before reading a driver that writes a flag by name. */
        sts &= ~value;
        break;
    case 0x28u:
        write_txdt(value);
        break;
    default:
        break;
    }
}
