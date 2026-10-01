/*
 * The modelled I2C bus behind the seam in src/arch/stm32f405/i2c.c. See
 * host_i2c_model.h for what it is, what it is not, and why each flag it raises
 * is load-bearing.
 *
 * The register bit positions come from the port's own regs.h - they are the
 * hardware contract, and tests/test_regs.c is what pins them. What this file
 * owns is the *behaviour*: what a write of the start bit does, where the
 * acknowledge moves, which read clears the address flag.
 */

#include <string.h>

#include "../src/arch/stm32f405/regs.h"
#include "host_i2c_model.h"

typedef struct {
    int      attached;
    uint8_t  address;                    /* 7 bits */
    uint8_t  registers[HOST_I2C_REGISTERS];
    uint8_t  pointer;                    /* where a read starts */
    unsigned written;                    /* data bytes written this transfer */
    int      addressed;                  /* it is the one being talked to */
} host_slave_t;

static host_slave_t slaves[HOST_I2C_SLAVES];

/* The master, as the driver sees it. */
static uint32_t cr1;
static uint32_t cr2;
static uint32_t ccr;
static uint32_t trise;
static uint32_t sr1;
static uint32_t sr2;
static uint32_t data;        /* the byte the master last wrote, or may read */

static int      addressed;   /* a slave answered the address */
static int      receiving;   /* this transfer is a read */
static int      address_phase; /* the next byte written is the address */
static int      sr1_was_read;  /* for the SR1-then-SR2 clear of ADDR */
static int      register_named; /* a data byte has named the register */
static unsigned next_in;     /* the next register byte the slave will supply */
static int      flushed;     /* the byte that was in flight when the ACK stopped */

static uint8_t  last_address;
static unsigned last_written;
static unsigned last_read;

static host_slave_t *current;

/* How many times this bus will answer before it stops: see the header. The
 * count is of *answers*, not of reads. Counting reads is what the first draft
 * did, and it broke the test next door: a wait that times out polls this
 * register two hundred thousand times with the same byte in it, so one bus
 * that does not answer - an address nobody acknowledges - spent the whole
 * budget and left the *next*, healthy transaction reading zero for ever. An
 * answer is a change the driver can see. */
static unsigned stall_after = HOST_I2C_ANSWERS_ALL;
static unsigned flags_answered;
static uint32_t last_sr1_seen;

void host_i2c_model_stall_after(unsigned flags)
{
    stall_after = flags;
    flags_answered = 0u;
    last_sr1_seen = sr1;
}

/* --- the wires ------------------------------------------------------------
 *
 * See host_i2c_model.h for why the wires are modelled separately from the
 * peripheral. Open drain throughout, because that is the property the whole
 * recovery path is written around: a line is low if *anyone* is pulling it
 * down, and high only if nobody is. A model that let the master drive a line
 * high would let a push-pull driver pass here and short two devices together
 * on the bench.
 */
static uint32_t wire_scl_port;
static uint32_t wire_sda_port;
static uint8_t  wire_scl_pin;
static uint8_t  wire_sda_pin;
static int      wire_known;

static int      sda_held;            /* a slave is pulling it down */
static unsigned sda_release_after;   /* clocks until it lets go; 0 = never */
static int      sda_driven_low;      /* the master is pulling it down */
static int      scl_driven_low;

static unsigned clocks;              /* SCL falling edges the master drove */
static int      stop_seen;

void host_i2c_gpio_attach(uint32_t scl_port, uint8_t scl_pin,
                          uint32_t sda_port, uint8_t sda_pin)
{
    wire_scl_port = scl_port;
    wire_scl_pin = scl_pin;
    wire_sda_port = sda_port;
    wire_sda_pin = sda_pin;
    wire_known = 1;
}

void host_i2c_gpio_hold_sda(unsigned release_after)
{
    sda_held = 1;
    sda_release_after = release_after;
}

int host_i2c_gpio_line(uint32_t port, uint8_t pin)
{
    if (!wire_known) {
        /* No wires attached: an unheld bus reads released, which is what a
         * board with nothing plugged into it looks like to the driver's
         * stuck-bus check, and keeps every test that predates this one from
         * taking a recovery path it never asked for. */
        return 1;
    }

    if (port == wire_sda_port && pin == wire_sda_pin) {
        if (sda_driven_low) {
            return 0;
        }
        if (sda_held &&
            (sda_release_after == 0u || clocks < sda_release_after)) {
            return 0;
        }
        return 1;
    }
    if (port == wire_scl_port && pin == wire_scl_pin) {
        return scl_driven_low ? 0 : 1;
    }
    return 1;
}

void host_i2c_gpio_drive(uint32_t port, uint8_t pin, int level)
{
    if (!wire_known) {
        return;
    }

    if (port == wire_sda_port && pin == wire_sda_pin) {
        /* STOP is SDA being released while SCL is high - the one edge in the
         * protocol that is not a data bit, and the thing the recovery has to
         * send for a slave to see the frame end rather than run on into the
         * next one. Detected rather than asserted so that a recovery that
         * forgot it fails a test instead of shipping. */
        if (level != 0 && !sda_driven_low && !scl_driven_low && clocks > 0u) {
            stop_seen = 1;
        }
        sda_driven_low = (level == 0);
        return;
    }
    if (port == wire_scl_port && pin == wire_scl_pin) {
        if (level == 0 && !scl_driven_low) {
            clocks++;
        }
        scl_driven_low = (level == 0);
        return;
    }
}

unsigned host_i2c_gpio_clocks(void)
{
    return clocks;
}

int host_i2c_gpio_saw_stop(void)
{
    return stop_seen;
}

void host_i2c_model_reset(void)
{
    wire_known = 0;
    wire_scl_port = 0u;
    wire_sda_port = 0u;
    wire_scl_pin = 0u;
    wire_sda_pin = 0u;
    sda_held = 0;
    sda_release_after = 0u;
    sda_driven_low = 0;
    scl_driven_low = 0;
    clocks = 0u;
    stop_seen = 0;

    memset(slaves, 0, sizeof slaves);
    cr1 = 0u;
    cr2 = 0u;
    ccr = 0u;
    trise = 0u;
    sr1 = 0u;
    sr2 = 0u;
    data = 0u;
    addressed = 0;
    receiving = 0;
    address_phase = 0;
    sr1_was_read = 0;
    register_named = 0;
    next_in = 0u;
    flushed = 0;
    last_address = 0u;
    last_written = 0u;
    last_read = 0u;
    current = 0;
    stall_after = HOST_I2C_ANSWERS_ALL;
    flags_answered = 0u;
    last_sr1_seen = 0u;
}

int host_i2c_attach(uint8_t address, const uint8_t *registers, unsigned count)
{
    for (unsigned i = 0u; i < HOST_I2C_SLAVES; i++) {
        if (!slaves[i].attached) {
            slaves[i].attached = 1;
            slaves[i].address = address;
            if (count > HOST_I2C_REGISTERS) {
                count = HOST_I2C_REGISTERS;
            }
            memset(slaves[i].registers, 0, sizeof slaves[i].registers);
            if (registers != 0 && count > 0u) {
                memcpy(slaves[i].registers, registers, count);
            }
            slaves[i].pointer = 0u;
            slaves[i].written = 0u;
            slaves[i].addressed = 0;
            return (int)i;
        }
    }
    return -1;
}

uint8_t host_i2c_last_address(void) { return last_address; }
uint8_t host_i2c_last_register(void) { return current != 0 ? current->pointer : 0u; }
unsigned host_i2c_last_written(void) { return last_written; }
unsigned host_i2c_last_read(void) { return last_read; }

uint8_t host_i2c_register(unsigned index, uint8_t reg)
{
    return index < HOST_I2C_SLAVES ? slaves[index].registers[reg] : 0u;
}

int host_i2c_addressed(unsigned index)
{
    return index < HOST_I2C_SLAVES ? slaves[index].addressed : 0;
}

/* The address byte: the slave that answers, or AF when none does. */
static void address_phase_write(uint32_t value)
{
    unsigned wanted = (value >> 1) & 0x7Fu;

    last_address = (uint8_t)wanted;
    last_written = 0u;
    last_read = 0u;
    sr1 &= ~I2C_SR1_SB;
    address_phase = 0;
    receiving = (value & 1u) != 0u;
    register_named = 0;

    current = 0;
    for (unsigned i = 0u; i < HOST_I2C_SLAVES; i++) {
        slaves[i].addressed = 0;
        slaves[i].written = 0u;
        if (slaves[i].attached && slaves[i].address == wanted) {
            current = &slaves[i];
        }
    }
    if (current == 0) {
        /* Nobody is at that address: the part answers with a not-acknowledge
         * and the flag the driver has to clear by hand. */
        sr1 |= I2C_SR1_AF;
        addressed = 0;
        return;
    }
    addressed = 1;
    current->addressed = 1;
    sr1 |= I2C_SR1_ADDR;

    /* A transmit transfer has an empty data register the moment the address is
     * acknowledged, which is what the driver waits for before it sends the
     * register number. A receive transfer gets its first byte instead - the one
     * the code below hands over when the master clears ADDR. */
    if (!receiving) {
        sr1 |= I2C_SR1_TXE;
    }
}

/* A data byte in a transfer the master is sending. The first one names the
 * register; the rest are written into it, auto-incrementing as a sensor
 * does. */
static void transmit(uint32_t value)
{
    if (current == 0) {
        return;
    }
    if (!register_named) {
        current->pointer = (uint8_t)value;
        register_named = 1;
    } else {
        current->registers[current->pointer] = (uint8_t)value;
        if (current->pointer < HOST_I2C_REGISTERS - 1u) {
            current->pointer++;
        }
    }
    current->written++;
    last_written = current->written;
    /* The data register is free again the moment the byte is on the wire, and
     * BTF says both the register and the shift register hold one. Both are set
     * here because neither takes time in a model. */
    sr1 |= I2C_SR1_TXE | I2C_SR1_BTF;
}

/* A byte in a transfer the master is receiving: the slave's register file from
 * wherever the write phase left the pointer. */
static void receive_prepare(void)
{
    if (current != 0) {
        next_in = current->pointer;
    }
}

static void read_data(void)
{
    if (current != 0) {
        data = current->registers[next_in % HOST_I2C_REGISTERS];
        next_in++;
        /* The register pointer walks as the master reads, which is what makes
         * "read four bytes from here" one transaction on every sensor that has
         * an auto-incrementing address. */
        current->pointer++;
    }
    last_read++;
    sr1 &= ~I2C_SR1_BTF;

    /*
     * What the slave does next. While the master is acknowledging, every byte
     * read is followed by another. When the master stops acknowledging there
     * is still one byte already clocked in behind this one - which is the
     * whole reason the last two bytes of a long read come out together - and
     * after that the slave stops.
     */
    if ((cr1 & I2C_CR1_ACK) != 0u) {
        sr1 |= I2C_SR1_RXNE;
    } else if (!flushed) {
        flushed = 1;
        sr1 |= I2C_SR1_RXNE;
    } else {
        sr1 &= ~I2C_SR1_RXNE;
    }
}

static uint32_t register_read(uint32_t offset)
{
    switch (offset) {
    case I2C_OFF_SR1:
        sr1_was_read = 1;
        if (flags_answered >= stall_after) {
            /* The bus has stopped: none of what this register used to say is
             * true any more, so the driver's wait runs to its own bound - and
             * gives up, which is the difference between a sensor that died and
             * an aircraft that stops flying because of it. */
            return 0u;
        }
        if (sr1 != last_sr1_seen) {
            last_sr1_seen = sr1;
            flags_answered++;
        }
        return sr1;
    case I2C_OFF_SR2:
        /* Reading SR1 and then SR2 is what clears ADDR, in that order. The
         * first byte of a read transfer arrives at the same moment. */
        if (sr1_was_read && (sr1 & I2C_SR1_ADDR) != 0u) {
            sr1 &= ~I2C_SR1_ADDR;
            if (receiving) {
                receive_prepare();
                flushed = 0;
                sr1 |= I2C_SR1_RXNE;
            }
        }
        return sr2;
    case I2C_OFF_DR:
        read_data();
        return data;
    case I2C_OFF_CR1:
        return cr1;
    case I2C_OFF_CR2:
        return cr2;
    case I2C_OFF_CCR:
        return ccr;
    case I2C_OFF_TRISE:
        return trise;
    default:
        return 0u;
    }
}

static void cr1_write(uint32_t value)
{
    if ((value & I2C_CR1_SWRST) != 0u) {
        /* The part's own reset: the peripheral comes back with nothing set,
         * which is why the driver rewrites CR1 afterwards. */
        cr1 = 0u;
        sr1 = 0u;
        sr2 = 0u;
        addressed = 0;
        receiving = 0;
        address_phase = 0;
        current = 0;
        return;
    }

    cr1 = value;

    if ((cr1 & I2C_CR1_PE) == 0u) {
        return; /* a peripheral that is off does nothing */
    }
    if ((value & I2C_CR1_START) != 0u) {
        /* A start bit on the wire: SB says the address may be written, and the
         * part clears START itself once it has gone out. Leaving it set is
         * what made the *next* control write look like another start - the
         * driver's one-byte read writes STOP while that bit is still there,
         * and the transaction was thrown away before its byte was read. */
        sr1 |= I2C_SR1_SB;
        cr1 &= ~I2C_CR1_START;
        address_phase = 1;
        addressed = 0;
        receiving = 0;
        current = 0;
    }
    if ((value & I2C_CR1_STOP) != 0u) {
        /* The stop ends the transfer on the wire, but a byte that has already
         * been clocked in is still in the data register to be read - which is
         * the one-byte read, where the stop goes out before the read does. The
         * part clears STOP on its own for the same reason it clears START. */
        cr1 &= ~I2C_CR1_STOP;
        address_phase = 0;
    }
    if (receiving && addressed) {
        if ((value & I2C_CR1_POS) != 0u) {
            /* POS is the two-byte read: by the time it is set, both bytes are
             * in the master's registers and BTF says so - which is what the
             * driver waits for. */
            sr1 |= I2C_SR1_BTF | I2C_SR1_RXNE;
        }
        if ((value & I2C_CR1_ACK) == 0u) {
            /* The acknowledge that will not come: one byte is still on its
             * way, and then the slave stops. */
            flushed = 0;
        }
    }
}

static void register_write(uint32_t offset, uint32_t value)
{
    switch (offset) {
    case I2C_OFF_CR1:
        cr1_write(value);
        return;
    case I2C_OFF_CR2:
        cr2 = value;
        return;
    case I2C_OFF_CCR:
        ccr = value;
        return;
    case I2C_OFF_TRISE:
        trise = value;
        return;
    case I2C_OFF_DR:
        data = value;
        if (address_phase) {
            address_phase_write(value);
        } else if (receiving) {
            /* The master writing during a read transfer is the repeated start
             * plus its address, which is handled above. */
            return;
        } else {
            transmit(value);
        }
        return;
    case I2C_OFF_SR1:
        /* Write zero to clear: AF goes when software writes 0 to it. */
        sr1 &= value;
        return;
    default:
        return;
    }
}

uint32_t host_i2c_read(uint32_t base, uint32_t offset)
{
    (void)base;
    return register_read(offset);
}

void host_i2c_write(uint32_t base, uint32_t offset, uint32_t value)
{
    (void)base;
    register_write(offset, value);
}
