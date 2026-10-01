#include "host_spi_model.h"

#include <string.h>

/*
 * A device on the modelled SPI bus: see host_spi_model.h for why the seam is
 * at the transfer rather than at the registers.
 *
 * The two ARM ports drive SPI identically, and the *port's* own loop - write
 * the data register, wait for the buffer, read what came back - is already
 * pinned against the mapped register block in the arch tests. What a page of
 * memory cannot be is a part that answers, and that is what this adds: the
 * bytes of one transfer go to a device with a register file, and its answer
 * comes back in `rx`.
 *
 * The one rule worth stating, because the model cannot see the chip select (a
 * GPIO the driver sets directly): a transfer is classified by its shape. One
 * byte is an address, which moves the part's register pointer and starts a read
 * or a write; two or more bytes are an address and data in one frame; and
 * anything that follows a one-byte address is more data for that address -
 * which is how a read's bytes come back and how a burst is written. That is
 * exactly the byte stream both boards and every driver in `src/core/sensors`
 * produce, and `host_spi_transfers()` is what the checks hold it to.
 */

#define MODEL_BUSES 4u

typedef struct {
    uint8_t regs[256];
    int     have_address; /* a one-byte address is waiting for its data */
    uint8_t address;
    int     reading;
} device_t;

static device_t devices[MODEL_BUSES];
static uint32_t seen_bases[MODEL_BUSES];

static unsigned transfers;
static unsigned last_len;
static uint8_t  last_tx[1024];
static int      present;
static int      echo;

static unsigned bus_index(uint32_t base)
{
    for (unsigned i = 0; i < MODEL_BUSES; i++) {
        if (seen_bases[i] == base) {
            return i;
        }
        if (seen_bases[i] == 0u) {
            seen_bases[i] = base;
            return i;
        }
    }
    return 0u;
}

void host_spi_reset(void)
{
    memset(devices, 0, sizeof devices);
    memset(seen_bases, 0, sizeof seen_bases);
    memset(last_tx, 0, sizeof last_tx);
    transfers = 0u;
    last_len = 0u;
    present = 0;
    echo = 0;
}

void host_spi_set_register(uint8_t reg, uint8_t value)
{
    for (unsigned i = 0; i < MODEL_BUSES; i++) {
        devices[i].regs[reg] = value;
    }
}

uint8_t host_spi_register(uint8_t reg)
{
    return devices[0].regs[reg];
}

unsigned host_spi_transfers(void)
{
    return transfers;
}

unsigned host_spi_last_len(void)
{
    return last_len;
}

const uint8_t *host_spi_last_tx(void)
{
    return last_tx;
}

void host_spi_set_present(int on)
{
    present = on;
}

void host_spi_set_echo(int on)
{
    echo = on;
}

int host_spi_transfer(uint32_t base, const uint8_t *tx, uint8_t *rx,
                      unsigned len)
{
    device_t *dev = &devices[bus_index(base)];
    unsigned first_data = 0u;

    if (len == 0u) {
        return 0; /* nothing to shift, and nothing to answer */
    }

    /* What the wires carried, for the checks that are about the framing. */
    for (unsigned i = 0; i < len && last_len < sizeof last_tx; i++) {
        last_tx[last_len++] = tx != 0 ? tx[i] : 0xFFu;
    }
    transfers++;

    if (echo || !present) {
        for (unsigned i = 0; i < len; i++) {
            uint8_t out = tx != 0 ? tx[i] : 0xFFu;
            if (rx != 0) {
                rx[i] = echo ? out : 0xFFu; /* nothing drives MISO */
            }
        }
        return 0;
    }

    if (dev->have_address) {
        /* The bytes after a one-byte address, which is a read's data or a
         * burst's payload. */
        dev->have_address = 0;
        first_data = 0u;
    } else if (len >= 2u) {
        /* An address and data in one frame: the shape a two-byte write takes. */
        uint8_t address = tx[0];

        dev->reading = (address & 0x80u) != 0u;
        dev->address = (uint8_t)(address & 0x7Fu);
        if (rx != 0) {
            rx[0] = 0xFFu;
        }
        first_data = 1u;
    } else {
        /* One byte on its own: an address, and the data follows in the next
         * transfer with the select still down. */
        dev->reading = (tx[0] & 0x80u) != 0u;
        dev->address = (uint8_t)(tx[0] & 0x7Fu);
        dev->have_address = 1;
        if (rx != 0) {
            rx[0] = 0xFFu;
        }
        return 0;
    }

    for (unsigned i = first_data; i < len; i++) {
        if (dev->reading) {
            uint8_t value = dev->regs[dev->address];
            dev->address++;
            if (rx != 0) {
                rx[i] = value;
            }
        } else {
            dev->regs[dev->address] = tx[i];
            dev->address++;
            if (rx != 0) {
                rx[i] = 0xFFu;
            }
        }
    }
    return 0;
}
