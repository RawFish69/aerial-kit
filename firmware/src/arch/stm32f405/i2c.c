#include "arch.h"

#include "i2c_timing.h"

/*
 * I2C master, polled, 7-bit addresses, register-oriented reads and writes.
 *
 * Register-oriented because that is what every sensor on this bus is: a byte
 * for the register, then the bytes. The bus interface the drivers use
 * (ak_bus.h) is exactly that shape, so a barometer written against it works
 * over SPI with a chip select or over I2C with an address and never knows
 * which - which is the point of the seam, and the thing this file is here to
 * demonstrate.
 *
 * Polled rather than interrupt-driven, for the same reason the console's UART
 * is: a barometer is read thirty times a second at most, and an interrupt for
 * that is an interrupt to get wrong. Every wait has a bound, and a bound that
 * is reached resets the peripheral rather than returning half a reading - a
 * sensor that stops answering must not be able to stop the aircraft.
 *
 * Nothing here has been run against a part. There is no barometer on the bench
 * board and no I2C device of any kind wired to it, so what is verified is the
 * timing arithmetic (tests/test_i2c.c, against RM0090) and the register bit
 * positions (tests/test_regs.c). The transaction sequences below are RM0090's
 * own master sequences, including the NACK position for a two-byte read, and
 * they are the part a scope will have to confirm.
 */

/* A wait bound in loops rather than in microseconds: the delays that matter
 * here are microseconds and this core's loop is a few cycles, so the number is
 * chosen to be far longer than any real transaction and far shorter than a
 * human notices. Reaching it means the bus is stuck, not slow. */
#define AK_I2C_GUARD 200000u

/*
 * Every register access in this file goes through these two, so that a host
 * build can put a *modelled bus* behind them (tests/host_i2c_model.c) and run
 * the transaction sequences - the start, the address, the register number, the
 * repeated start, the acknowledge that moves for a two-byte read - instead of
 * leaving them to a scope and a confident comment.
 *
 * The seam is spelled out rather than hidden behind a redefined register
 * macro because half of these accesses are read-modify-writes: `CR1 |= START`
 * is a read and a write, and a model that only saw the read would never learn
 * that a start bit went out. On the target the two are a load and a store the
 * compiler sees through, and the image is the same size without them.
 */
#ifdef AK_HOST_I2C
#include "host_i2c_model.h"
#define i2c_read(base, offset)         host_i2c_read((base), (offset))
#define i2c_write(base, offset, value) host_i2c_write((base), (offset), (value))
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

/*
 * The bus *wires*, for the recovery further down.
 *
 * Everything else in this file talks to the peripheral, and the peripheral is
 * exactly what cannot help with a slave holding SDA down: that is not a
 * peripheral state, and SWRST - the part's own reset, used everywhere above -
 * does not touch it. The slave is holding the wire, so the wire is where the
 * fix has to happen, which means reading IDR and driving the pins directly.
 *
 * Seamed for the same reason the register accesses are: the only other place
 * this path runs is a bench with a stuck sensor on it, and a host build that
 * cannot hold a line down cannot run it at all. See tests/host_i2c_model.h.
 */
#ifdef AK_HOST_I2C
static int bus_line(ak_pin_t pin)
{
    return host_i2c_gpio_line(pin.port, pin.pin);
}

static void bus_drive(ak_pin_t pin, int level)
{
    host_i2c_gpio_drive(pin.port, pin.pin, level);
}
#else
/* IDR, not ak_pin_get(): that reads ODR, which for an open-drain
 * alternate-function pin is what *we* are driving, and the entire question
 * here is what somebody else is doing to the wire. */
static int bus_line(ak_pin_t pin)
{
    return (int)((GPIO_IDR(pin.port) >> pin.pin) & 1u);
}

static void bus_drive(ak_pin_t pin, int level)
{
    ak_pin_set(pin, level);
}
#endif

/*
 * Is this one of the three controllers on the part?
 *
 * The read and write functions used to skip this, which meant a caller that
 * passed the wrong base address wrote a start bit and an address byte into
 * whatever lived there - the ADC's status register, the SPI control register -
 * and then failed on a timeout, leaving the wreckage behind. Every caller in
 * this firmware passes a board constant, so nothing was broken; it was a
 * footgun with the safety off, and it is cheap to remove.
 */
static int ak_i2c_known_controller(uint32_t i2c)
{
    return i2c == I2C1_BASE || i2c == I2C2_BASE || i2c == I2C3_BASE;
}

static int wait_set(uint32_t i2c, uint32_t mask)
{
    uint32_t guard = AK_I2C_GUARD;

    while (guard-- > 0u) {
        if ((i2c_read(i2c, I2C_OFF_SR1) & mask) != 0u) {
            return 0;
        }
    }
    return -1;
}

/*
 * Clearing ADDR is not a write of zero to it: the flag is cleared by reading
 * SR1 and then SR2, in that order, which is why this is two volatile reads
 * nobody would guess at. RM0090 27.6.6.
 */
static void clear_address(uint32_t i2c)
{
    (void)i2c_read(i2c, I2C_OFF_SR1);
    (void)i2c_read(i2c, I2C_OFF_SR2);
}

/*
 * Put the peripheral back to where it can start again.
 *
 * A transaction that timed out leaves the bus somewhere in the middle of a
 * frame, and the next START would be taken as a continuation of it. SWRST is
 * the part's own reset for exactly this; the pins stay where they are, which
 * matters because they are open-drain and a slave holding SDA low will still
 * be holding it after this.
 */
static void bus_reset(uint32_t i2c, uint32_t ccr, uint32_t trise, uint32_t freq)
{
    i2c_write(i2c, I2C_OFF_CR1, I2C_CR1_SWRST);
    i2c_write(i2c, I2C_OFF_CR1, 0u);
    i2c_write(i2c, I2C_OFF_CR2, freq);
    i2c_write(i2c, I2C_OFF_CCR, ccr);
    i2c_write(i2c, I2C_OFF_TRISE, trise);
    i2c_write(i2c, I2C_OFF_CR1, I2C_CR1_PE | I2C_CR1_ACK);
}

/* --- freeing a bus a slave is holding -------------------------------------
 *
 * What this is for, in one paragraph, because the failure it addresses looks
 * exactly like a dead sensor and is not one.
 *
 * I2C has no reset line. A slave is left part-way through a byte - because the
 * master was reset mid-transfer, because power came up in the wrong order,
 * because a cable moved - and it sits there driving SDA low waiting for a
 * clock edge that will never come. Nothing the master does to its own
 * peripheral reaches it: SWRST, which every timeout above calls, resets the
 * *master*, and the master was never the problem. The bus reads SCL high, SDA
 * low, for ever, and the board reports `nothing answered on the bus` about a
 * sensor that is plugged in and working.
 *
 * The cure is the one in every I2C application note: clock the bus until the
 * slave gives up. Nine clocks is one more than a byte, so whatever state it
 * was in, it finishes it; the STOP after them is what tells it the frame is
 * over rather than continuing into the next one. Then the peripheral comes
 * back up and the transaction that found the bus stuck runs normally.
 *
 * Measured on the Feather's LSM6DSO at 0x6B, 2026-10-01: SDA held low, no
 * address answered, all read lengths -1; recovery was not in the firmware yet
 * and the honest reading was a sensor that looked dead. This is what turned
 * that reading into a fix rather than a bin.
 */

#define AK_I2C_RECOVER_CLOCKS 9u

/* Roughly five microseconds, which is a half period a little under 100 kHz.
 * Approximate on purpose: the loop is a few cycles and the core is at 168 MHz,
 * and the slave being clocked does not care about the rate - it cares that
 * there are edges. A slave that needed an exact 100 kHz to let go would not be
 * recoverable by any master whose clock was in doubt, which is the situation
 * this runs in. */
#define AK_I2C_RECOVER_HALF_PERIOD_LOOPS 200u

static void bus_half_period(void)
{
    for (volatile unsigned i = 0; i < AK_I2C_RECOVER_HALF_PERIOD_LOOPS; i++) {
    }
}

/*
 * Where each controller's two wires are.
 *
 * ak_i2c_init is the only thing that is ever told the pins, and recovery
 * happens later, on a bus that has since been held down - so the pins are
 * recorded there and looked up here. Recording rather than adding them to
 * ak_i2c_read_reg's arguments is what keeps the other twenty-odd call sites in
 * the tree from each having to carry a fact only the board knows.
 */
typedef struct {
    uint32_t base;
    ak_pin_t scl;
    ak_pin_t sda;
    uint8_t  af;
    uint32_t speed_hz;
    int      known;
} ak_i2c_bus_t;

static ak_i2c_bus_t buses[3];

static ak_i2c_bus_t *bus_of(uint32_t i2c)
{
    for (unsigned i = 0; i < 3u; i++) {
        if (buses[i].known && buses[i].base == i2c) {
            return &buses[i];
        }
    }
    return 0;
}

static ak_i2c_bus_t *bus_record(uint32_t i2c)
{
    ak_i2c_bus_t *slot = bus_of(i2c);

    if (slot == 0) {
        for (unsigned i = 0; i < 3u; i++) {
            if (!buses[i].known) {
                slot = &buses[i];
                break;
            }
        }
    }
    if (slot != 0) {
        slot->base = i2c;
        slot->known = 1;
    }
    return slot;
}

int ak_i2c_bus_recover(uint32_t i2c, ak_pin_t scl, ak_pin_t sda, uint8_t af,
                       uint32_t speed_hz)
{
    /*
     * Only one condition is recoverable by clocking, and being exact about
     * which is what keeps this from being a thing that fires on a healthy bus:
     *
     *   SCL high, SDA low  - a slave mid-byte. Nine clocks free it. This.
     *   SCL low           - nobody can clock anything, because the line that
     *                       would carry the clocks is the one being held. A
     *                       short, or a slave holding both. Not this.
     *   SCL high, SDA high - idle. Not this.
     */
    if (bus_line(scl) == 0 || bus_line(sda) != 0) {
        return 0;
    }

    /* Both pins stop being the peripheral's and become plain open-drain
     * outputs. Open drain because the slave is driving SDA too, and a
     * push-pull output here would be two devices shorting one wire the moment
     * the slave pulled it down. */
    ak_pin_output(scl, 1, GPIO_SPEED_HIGH);
    ak_pin_output(sda, 1, GPIO_SPEED_HIGH);

    /* Let go of data: the slave is the one holding it, and it can only let go
     * when it sees clocks. */
    bus_drive(sda, 1);

    for (unsigned i = 0; i < AK_I2C_RECOVER_CLOCKS; i++) {
        bus_drive(scl, 0);
        bus_half_period();
        bus_drive(scl, 1);
        bus_half_period();

        /* Stop early if it has already let go: a slave that releases on the
         * first clock has said what it needed to, and the remaining clocks
         * would be eight edges of noise on a bus that is now fine. */
        if (bus_line(sda) != 0) {
            break;
        }
    }

    /*
     * STOP: SDA low, then released, while SCL is high. Without it the slave
     * has finished the byte it was on and is waiting for the next one - the
     * next START would be taken as a continuation of a frame it never saw the
     * end of. Same reason the transaction paths above end with one.
     */
    bus_drive(sda, 0);
    bus_half_period();
    bus_drive(scl, 1);
    bus_half_period();
    bus_drive(sda, 1);
    bus_half_period();

    int freed = bus_line(sda) != 0;

    /* Back to the bus. ak_i2c_init puts both pins back to alternate function
     * and re-arms the peripheral from scratch, which the direct driving above
     * left detached. */
    ak_i2c_init(i2c, scl, sda, af, speed_hz);

    return freed ? 1 : -1;
}

/*
 * Called before every transaction, so that a bus a slave has hold of is freed
 * by the first access that meets it rather than by a caller who has to know to
 * ask.
 *
 * It costs two register reads when the bus is healthy, which is every
 * transaction but the first one after a fault, and it does nothing at all
 * unless the bus is in the one condition recovery can fix.
 */
static void bus_unstick(uint32_t i2c)
{
    ak_i2c_bus_t *bus = bus_of(i2c);

    if (bus == 0 || bus_line(bus->scl) == 0 || bus_line(bus->sda) != 0) {
        return;
    }
    (void)ak_i2c_bus_recover(bus->base, bus->scl, bus->sda, bus->af,
                             bus->speed_hz);
}

static uint32_t ccr_of(uint32_t i2c)
{
    return i2c_read(i2c, I2C_OFF_CCR);
}

static void reset_keeping_settings(uint32_t i2c)
{
    bus_reset(i2c, ccr_of(i2c), i2c_read(i2c, I2C_OFF_TRISE),
              i2c_read(i2c, I2C_OFF_CR2) & I2C_CR2_FREQ_MASK);
}

static int start(uint32_t i2c, uint8_t address, int read)
{
    i2c_write(i2c, I2C_OFF_CR1,
              i2c_read(i2c, I2C_OFF_CR1) | I2C_CR1_START);
    if (wait_set(i2c, I2C_SR1_SB) != 0) {
        return -1;
    }

    i2c_write(i2c, I2C_OFF_DR,
              (uint32_t)((address << 1) | (read ? 1u : 0u)));
    if (wait_set(i2c, I2C_SR1_ADDR) != 0) {
        /* The address did not answer. AF is the flag for that and it has to be
         * cleared by hand, or the next transaction inherits it. */
        if ((i2c_read(i2c, I2C_OFF_SR1) & I2C_SR1_AF) != 0u) {
            i2c_write(i2c, I2C_OFF_SR1,
                      i2c_read(i2c, I2C_OFF_SR1) & ~I2C_SR1_AF);
        }
        return -1;
    }
    clear_address(i2c);
    return 0;
}

int ak_i2c_read_reg(uint32_t i2c, uint8_t address, uint8_t reg, uint8_t *buf,
                    unsigned len)
{
    if (!ak_i2c_known_controller(i2c)) {
        return -1;
    }
    if (len == 0u) {
        return 0;
    }

    bus_unstick(i2c);

    i2c_write(i2c, I2C_OFF_CR1,
              (i2c_read(i2c, I2C_OFF_CR1) | I2C_CR1_ACK) & ~I2C_CR1_POS);

    if (start(i2c, address, 0) != 0) {
        reset_keeping_settings(i2c);
        return -1;
    }

    /* The register number, then a repeated start, which is what separates this
     * from a write: the same transaction without it would leave the part
     * waiting for more data. */
    if (wait_set(i2c, I2C_SR1_TXE) != 0) {
        reset_keeping_settings(i2c);
        return -1;
    }
    i2c_write(i2c, I2C_OFF_DR, reg);
    if (wait_set(i2c, I2C_SR1_TXE) != 0) {
        reset_keeping_settings(i2c);
        return -1;
    }

    i2c_write(i2c, I2C_OFF_CR1,
              i2c_read(i2c, I2C_OFF_CR1) | I2C_CR1_START);
    if (wait_set(i2c, I2C_SR1_SB) != 0) {
        reset_keeping_settings(i2c);
        return -1;
    }
    i2c_write(i2c, I2C_OFF_DR, (uint32_t)((address << 1) | 1u));
    if (wait_set(i2c, I2C_SR1_ADDR) != 0) {
        reset_keeping_settings(i2c);
        return -1;
    }
    clear_address(i2c);

    /*
     * Where the acknowledge turns into a NACK decides where the slave stops,
     * and the manual gives a different answer for each length. All three are
     * here because all three happen to a sensor: a who-am-i is one byte, a
     * sixteen-bit coefficient is two, and a pressure or an acceleration is
     * more. RM0090 27.3.3, master receiver.
     */
    if (len == 1u) {
        /* One byte: the NACK and the STOP go out before it is read, or the
         * slave is acknowledged and starts a second one nobody wants. */
        i2c_write(i2c, I2C_OFF_CR1,
                  (i2c_read(i2c, I2C_OFF_CR1) & ~I2C_CR1_ACK) | I2C_CR1_STOP);

        if (wait_set(i2c, I2C_SR1_RXNE) != 0) {
            reset_keeping_settings(i2c);
            return -1;
        }
        buf[0] = (uint8_t)(i2c_read(i2c, I2C_OFF_DR) & 0xFFu);
    } else if (len == 2u) {
        /* Two bytes: POS moves the NACK one byte later, and both bytes arrive
         * before anything is read - BTF says so. Without POS the acknowledge
         * lands after the first byte and the master waits for a third that the
         * slave has already been told not to send. */
        i2c_write(i2c, I2C_OFF_CR1,
                  (i2c_read(i2c, I2C_OFF_CR1) & ~I2C_CR1_ACK) | I2C_CR1_POS);

        if (wait_set(i2c, I2C_SR1_BTF) != 0) {
            reset_keeping_settings(i2c);
            return -1;
        }
        buf[0] = (uint8_t)(i2c_read(i2c, I2C_OFF_DR) & 0xFFu);
        i2c_write(i2c, I2C_OFF_CR1,
                  i2c_read(i2c, I2C_OFF_CR1) & ~I2C_CR1_POS);
        buf[1] = (uint8_t)(i2c_read(i2c, I2C_OFF_DR) & 0xFFu);
    } else {
        /* Three or more: NACK the last byte and let the STOP follow it.
         *
         * The NACK has to be the one sampled during the LAST byte's reception,
         * so ACK is cleared immediately before waiting for that byte - which
         * on a polled loop is well before the byte's ninth clock, the loop
         * being far faster than 400 kHz. Clearing it a byte earlier, when two
         * were still outstanding, was the first version and it does not work:
         * the loop reaches that point while the second-to-last byte is still
         * shifting in, so the NACK lands on *that* one, the part stops, and
         * the read dies in wait_set with the peripheral still waiting for a
         * byte the slave was told not to send. Measured on the Feather's
         * LSM6DSO at 0x6B: length 1 and 2 return 0, length 3, 6 and 12 all
         * returned -1 before this line changed. */
        for (unsigned i = 0; i < len; i++) {
            if (i + 1u == len) {
                i2c_write(i2c, I2C_OFF_CR1,
                          i2c_read(i2c, I2C_OFF_CR1) & ~I2C_CR1_ACK);
            }
            if (wait_set(i2c, I2C_SR1_RXNE) != 0) {
                reset_keeping_settings(i2c);
                return -1;
            }
            buf[i] = (uint8_t)(i2c_read(i2c, I2C_OFF_DR) & 0xFFu);
        }
        i2c_write(i2c, I2C_OFF_CR1,
                  i2c_read(i2c, I2C_OFF_CR1) | I2C_CR1_STOP);
    }

    i2c_write(i2c, I2C_OFF_CR1, i2c_read(i2c, I2C_OFF_CR1) | I2C_CR1_ACK);
    return 0;
}

int ak_i2c_write_reg(uint32_t i2c, uint8_t address, uint8_t reg, uint8_t value)
{
    if (!ak_i2c_known_controller(i2c)) {
        return -1;
    }

    bus_unstick(i2c);

    if (start(i2c, address, 0) != 0) {
        reset_keeping_settings(i2c);
        return -1;
    }

    if (wait_set(i2c, I2C_SR1_TXE) != 0) {
        reset_keeping_settings(i2c);
        return -1;
    }
    i2c_write(i2c, I2C_OFF_DR, reg);
    if (wait_set(i2c, I2C_SR1_TXE) != 0) {
        reset_keeping_settings(i2c);
        return -1;
    }
    i2c_write(i2c, I2C_OFF_DR, value);

    /* BTF, not TXE: TXE only says the data register is free, and the STOP
     * that followed it would end the transaction while the last byte is still
     * on the wire. */
    if (wait_set(i2c, I2C_SR1_BTF) != 0) {
        reset_keeping_settings(i2c);
        return -1;
    }
    i2c_write(i2c, I2C_OFF_CR1,
              i2c_read(i2c, I2C_OFF_CR1) | I2C_CR1_STOP);
    return 0;
}

void ak_i2c_init(uint32_t i2c, ak_pin_t scl, ak_pin_t sda, uint8_t af,
                 uint32_t speed_hz)
{
    uint32_t pclk1 = ak_clk_apb1_hz();
    uint32_t freq = ak_i2c_freq_range(pclk1);

    if (i2c == I2C1_BASE) {
        RCC_APB1ENR |= RCC_APB1ENR_I2C1EN;
    } else if (i2c == I2C2_BASE) {
        RCC_APB1ENR |= RCC_APB1ENR_I2C2EN;
    } else if (i2c == I2C3_BASE) {
        RCC_APB1ENR |= RCC_APB1ENR_I2C3EN;
    } else {
        return;
    }
    (void)RCC_APB1ENR;

    /* Where the wires are, before anything below can fail: a transaction that
     * meets a bus a slave is holding has to be able to find the pins to free
     * it, and this is the only place they are ever named. */
    {
        ak_i2c_bus_t *slot = bus_record(i2c);

        /* The three controllers are the only bases that reach here, so there
         * is always a slot; the check is so that adding a fourth one day is a
         * bus without recovery rather than a null dereference. */
        if (slot != 0) {
            slot->scl = scl;
            slot->sda = sda;
            slot->af = af;
            slot->speed_hz = speed_hz;
        }
    }

    /* Both pins are open-drain and nobody drives them high: the bus is pulled
     * up by resistors and every participant can only pull it down. A
     * push-pull pin here would short two devices together the first time both
     * of them spoke. */
    ak_pin_af_open_drain(scl, af, GPIO_PUPD_NONE);
    ak_pin_af_open_drain(sda, af, GPIO_PUPD_NONE);

    bus_reset(i2c, ak_i2c_ccr(pclk1, speed_hz, 1),
              ak_i2c_trise(pclk1, speed_hz > 100000u), freq);
}
