/*
 * The modelled SPI and I2C peripherals, and the IDF calls the ESP32's two bus
 * files make. Compiled *instead of* IDF, so the real `src/arch/esp32/spi.c`
 * and `i2c.c` run against it.
 *
 * Faithfulness, where it matters:
 *
 *   - `spi_transaction_t::length` is in BITS, so the model divides by eight
 *     and the check reads the same number the peripheral would have clocked.
 *   - a device with a chip select asserts it per transaction unless the
 *     transaction carries SPI_TRANS_CS_KEEP_ACTIVE - which is how a burst that
 *     is one conversation is told from one that bounces the select between
 *     chunks, and the model counts the assertions.
 *   - I2C reads come out of the register file from the pointer the last write
 *     left, in order, because the parts auto-increment and that is what makes
 *     "register byte, repeated start, N bytes" one transaction.
 */

#include <string.h>

#include "driver/i2c_master.h"
#include "driver/spi_master.h"

#include "host_esp32_bus_model.h"

#define MODEL_SPI_BUSES 2u
#define MODEL_SPI_DEVICES 4u
#define MODEL_I2C_BUSES 2u
#define MODEL_I2C_DEVICES 4u
/* The port's own ceiling (AK_ESP_SPI_FRAME_MAX), because a burst arrives here
 * in chunks of a kilobyte and a model that refused them would be refusing the
 * thing the burst path exists for. */
#define MODEL_FRAME 2048u
#define MODEL_REGISTERS 256u

struct host_spi_bus {
    int up;
    int host;
    int sclk;
    int miso;
    int mosi;
    int max_transfer;
    int dma;
};

struct host_spi_device {
    int added;
    int bus;
    int cs;
    int clock_hz;
    int mode;
    int queue_size;
    unsigned transfers;
    unsigned cs_asserts;
    int cs_active; /* the select is down: a chain of kept transactions */
    int pending; /* a transfer the peripheral accepted and never finished */
    unsigned last_bits;
    unsigned last_bytes;
    int last_kept_cs;
    /* The select window's operation: a read or a write, to which register,
     * and how far into its data the last chunk got. A burst that holds the
     * select open is several transfers of *one* window. */
    int op_read;
    uint8_t op_reg;
    unsigned op_at;
    uint8_t tx[MODEL_FRAME];
    uint8_t rx[MODEL_FRAME];
};

struct host_i2c_bus {
    int up;
    int port;
    int sda;
    int scl;
    int pullup;
};

struct host_i2c_device {
    int added;
    uint16_t address;
    uint32_t speed_hz;
    unsigned transmits;
    unsigned receives;
    unsigned last_write_bytes;
    unsigned last_read_bytes;
    int last_address;
    uint8_t write[MODEL_FRAME];
};

static struct host_spi_bus spi_buses[MODEL_SPI_BUSES];
static struct host_spi_device spi_devices[MODEL_SPI_DEVICES];
static struct host_i2c_bus i2c_buses[MODEL_I2C_BUSES];
static struct host_i2c_device i2c_devices[MODEL_I2C_DEVICES];

static uint8_t registers[MODEL_REGISTERS];
static uint8_t i2c_registers[MODEL_REGISTERS];
static uint8_t i2c_pointer;

static int spi_silent;
static int spi_echo;
static int i2c_present = 1;
static unsigned i2c_probes;

/* The refusals a test can ask for: see the header. Each is a one-shot or a
 * flag, because the interesting case is the *first* call failing. */
static int spi_refuse_bus;
static int spi_refuse_device;
static int spi_refuse_transfer;
static int i2c_refuse_bus;
static int i2c_refuse_device;

void ak_host_esp_bus_reset(void)
{
    spi_refuse_bus = 0;
    spi_refuse_device = 0;
    spi_refuse_transfer = 0;
    i2c_refuse_bus = 0;
    i2c_refuse_device = 0;
    memset(spi_buses, 0, sizeof spi_buses);
    memset(spi_devices, 0, sizeof spi_devices);
    memset(i2c_buses, 0, sizeof i2c_buses);
    memset(i2c_devices, 0, sizeof i2c_devices);
    memset(registers, 0, sizeof registers);
    memset(i2c_registers, 0, sizeof i2c_registers);
    i2c_pointer = 0;
    spi_silent = 0;
    spi_echo = 0;
    i2c_present = 1;
    i2c_probes = 0;
}

/* --- SPI ------------------------------------------------------------------ */

void ak_host_spi_refuse_bus(int refuse) { spi_refuse_bus = refuse; }
void ak_host_spi_refuse_device(int refuse) { spi_refuse_device = refuse; }
void ak_host_spi_refuse_transfer(int refuse) { spi_refuse_transfer = refuse; }
void ak_host_i2c_refuse_bus(int refuse) { i2c_refuse_bus = refuse; }
void ak_host_i2c_refuse_device(int refuse) { i2c_refuse_device = refuse; }

esp_err_t spi_bus_initialize(spi_host_device_t host_id,
                             const spi_bus_config_t *bus_config,
                             spi_dma_chan_t dma_chan)
{
    if (bus_config == NULL) {
        return ESP_FAIL;
    }
    if (spi_refuse_bus) {
        return ESP_FAIL; /* the host is already claimed */
    }
    for (unsigned i = 0; i < MODEL_SPI_BUSES; i++) {
        if (!spi_buses[i].up) {
            spi_buses[i].up = 1;
            spi_buses[i].host = host_id;
            spi_buses[i].sclk = bus_config->sclk_io_num;
            spi_buses[i].miso = bus_config->miso_io_num;
            spi_buses[i].mosi = bus_config->mosi_io_num;
            spi_buses[i].max_transfer = bus_config->max_transfer_sz;
            spi_buses[i].dma = dma_chan;
            return ESP_OK;
        }
    }
    /* Torn down when the list is full: the port treats this as "already up". */
    return ESP_ERR_INVALID_STATE;
}

esp_err_t spi_bus_add_device(spi_host_device_t host_id,
                             const spi_device_interface_config_t *dev_config,
                             spi_device_handle_t *handle)
{
    if (dev_config == NULL || handle == NULL) {
        return ESP_FAIL;
    }
    if (spi_refuse_device) {
        return ESP_FAIL; /* no device slots left */
    }
    for (unsigned i = 0; i < MODEL_SPI_DEVICES; i++) {
        if (!spi_devices[i].added) {
            spi_devices[i].added = 1;
            spi_devices[i].bus = host_id;
            spi_devices[i].cs = dev_config->spics_io_num;
            spi_devices[i].clock_hz = dev_config->clock_speed_hz;
            spi_devices[i].mode = dev_config->mode;
            spi_devices[i].queue_size = dev_config->queue_size;
            *handle = &spi_devices[i];
            return ESP_OK;
        }
    }
    return ESP_FAIL;
}

esp_err_t spi_device_queue_trans(spi_device_handle_t handle,
                                 spi_transaction_t *trans_desc,
                                 TickType_t ticks_to_wait)
{
    (void)ticks_to_wait;

    if (spi_refuse_transfer) {
        return ESP_FAIL; /* the transaction queue will not take it */
    }
    if (handle == NULL || trans_desc == NULL) {
        return ESP_FAIL;
    }
    if (spi_silent) {
        /* What a peripheral with no device on it does: the *queue call*
         * succeeds - the transaction is accepted and the peripheral starts it -
         * and nothing ever completes, so the driver's *collection* is what
         * times out. Returning the failure from the queue call instead, which
         * this did, is a different fault (a peripheral that refuses the
         * transaction) and it left the port's collection timeout unreached -
         * see the coverage map, which is what found it. */
        handle->pending = 1;
        return ESP_OK;
    }

    unsigned bytes = (unsigned)(trans_desc->length / 8u);
    if (bytes > MODEL_FRAME) {
        return ESP_FAIL;
    }
    handle->pending = 0;
    handle->last_bits = (unsigned)trans_desc->length;
    handle->last_bytes = bytes;
    handle->last_kept_cs =
        (trans_desc->flags & SPI_TRANS_CS_KEEP_ACTIVE) != 0u;
    /* Whether this transfer opens a select window or carries on inside one -
     * read before the accounting below changes it, because the first chunk of
     * a kept-open burst is still the chunk that carries the address. */
    int continuation = handle->cs_active;
    /* Counting the *assertions* is the point: a burst that keeps the select
     * down is one conversation with the part, and a burst that lifts it
     * between chunks is several. */
    if (handle->cs >= 0) {
        if (!handle->cs_active) {
            handle->cs_asserts++;
            handle->cs_active = 1;
        }
        if (!handle->last_kept_cs) {
            handle->cs_active = 0;
        }
    }
    handle->transfers++;

    const uint8_t *tx = trans_desc->tx_buffer;
    for (unsigned i = 0; i < bytes; i++) {
        handle->tx[i] = tx != NULL ? tx[i] : 0u;
    }

    /*
     * The device's answer, and the reason the model is worth having: the first
     * byte of a *window* is the register address, bit 7 set means read, and
     * everything after it is data - which is what an InvenSense part expects
     * and what the port has to get right. A window that is already open (the
     * select was kept down) carries on with data only.
     */
    unsigned first = 0u;
    if (!continuation) {
        handle->op_read = (handle->tx[0] & 0x80u) != 0u;
        handle->op_reg = (uint8_t)(handle->tx[0] & 0x7Fu);
        handle->op_at = 0u;
        first = 1u;
    }
    for (unsigned i = 0; i < bytes; i++) {
        handle->rx[i] = 0xFFu;
    }
    if (spi_echo) {
        for (unsigned i = 0; i < bytes; i++) {
            handle->rx[i] = handle->tx[i];
        }
    } else if (handle->op_read) {
        for (unsigned i = first; i < bytes; i++) {
            handle->rx[i] = registers[(handle->op_reg + handle->op_at) & 0xFFu];
            handle->op_at++;
        }
    } else {
        for (unsigned i = first; i < bytes; i++) {
            registers[(handle->op_reg + handle->op_at) & 0xFFu] = handle->tx[i];
            handle->op_at++;
        }
    }

    /* What the peripheral's DMA does with the received bytes: they land in the
     * caller's buffer, which is the only place the port looks. */
    if (trans_desc->rx_buffer != NULL && bytes > 0u) {
        memcpy(trans_desc->rx_buffer, handle->rx, bytes);
    }
    return ESP_OK;
}

esp_err_t spi_device_get_trans_result(spi_device_handle_t handle,
                                      spi_transaction_t **trans_desc,
                                      TickType_t ticks_to_wait)
{
    (void)ticks_to_wait;
    if (handle == NULL) {
        return ESP_FAIL;
    }
    if (spi_silent || handle->pending) {
        /* The bound was reached and nothing came back: the transfer stays
         * pending, which is what a peripheral with no device on it looks
         * like. */
        return ESP_FAIL;
    }
    if (trans_desc != NULL) {
        *trans_desc = NULL;
    }
    return ESP_OK;
}

unsigned ak_host_spi_buses(void)
{
    unsigned count = 0u;
    for (unsigned i = 0; i < MODEL_SPI_BUSES; i++) {
        count += spi_buses[i].up ? 1u : 0u;
    }
    return count;
}

int ak_host_spi_sclk(unsigned bus)
{
    return bus < MODEL_SPI_BUSES ? spi_buses[bus].sclk : -1;
}

int ak_host_spi_miso(unsigned bus)
{
    return bus < MODEL_SPI_BUSES ? spi_buses[bus].miso : -1;
}

int ak_host_spi_mosi(unsigned bus)
{
    return bus < MODEL_SPI_BUSES ? spi_buses[bus].mosi : -1;
}

int ak_host_spi_max_transfer(unsigned bus)
{
    return bus < MODEL_SPI_BUSES ? spi_buses[bus].max_transfer : -1;
}

int ak_host_spi_dma(unsigned bus)
{
    return bus < MODEL_SPI_BUSES ? spi_buses[bus].dma : -1;
}

unsigned ak_host_spi_devices(void)
{
    unsigned count = 0u;
    for (unsigned i = 0; i < MODEL_SPI_DEVICES; i++) {
        count += spi_devices[i].added ? 1u : 0u;
    }
    return count;
}

int ak_host_spi_device_cs(unsigned device)
{
    return device < MODEL_SPI_DEVICES ? spi_devices[device].cs : -1;
}

int ak_host_spi_device_clock(unsigned device)
{
    return device < MODEL_SPI_DEVICES ? spi_devices[device].clock_hz : 0;
}

int ak_host_spi_device_mode(unsigned device)
{
    return device < MODEL_SPI_DEVICES ? spi_devices[device].mode : -1;
}

int ak_host_spi_device_queue(unsigned device)
{
    return device < MODEL_SPI_DEVICES ? spi_devices[device].queue_size : 0;
}

unsigned ak_host_spi_transfers(unsigned device)
{
    return device < MODEL_SPI_DEVICES ? spi_devices[device].transfers : 0u;
}

unsigned ak_host_spi_last_bits(unsigned device)
{
    return device < MODEL_SPI_DEVICES ? spi_devices[device].last_bits : 0u;
}

unsigned ak_host_spi_last_bytes(unsigned device)
{
    return device < MODEL_SPI_DEVICES ? spi_devices[device].last_bytes : 0u;
}

const uint8_t *ak_host_spi_last_tx(unsigned device)
{
    return device < MODEL_SPI_DEVICES ? spi_devices[device].tx : NULL;
}

const uint8_t *ak_host_spi_last_rx(unsigned device)
{
    return device < MODEL_SPI_DEVICES ? spi_devices[device].rx : NULL;
}

int ak_host_spi_last_kept_cs(unsigned device)
{
    return device < MODEL_SPI_DEVICES ? spi_devices[device].last_kept_cs : 0;
}

unsigned ak_host_spi_cs_asserts(unsigned device)
{
    return device < MODEL_SPI_DEVICES ? spi_devices[device].cs_asserts : 0u;
}

void ak_host_spi_set_register(uint8_t reg, uint8_t value)
{
    registers[reg] = value;
}

void ak_host_spi_set_burst(uint8_t reg, const uint8_t *bytes, unsigned len)
{
    for (unsigned i = 0; i < len; i++) {
        registers[(reg + i) & 0xFFu] = bytes[i];
    }
}

uint8_t ak_host_spi_register(uint8_t reg)
{
    return registers[reg];
}

void ak_host_spi_set_silent(int silent)
{
    spi_silent = silent != 0;
}

void ak_host_spi_set_echo(int echo)
{
    spi_echo = echo != 0;
}

/* --- I2C ------------------------------------------------------------------ */

esp_err_t i2c_new_master_bus(const i2c_master_bus_config_t *bus_config,
                             i2c_master_bus_handle_t *ret_bus_handle)
{
    if (bus_config == NULL || ret_bus_handle == NULL) {
        return ESP_FAIL;
    }
    if (i2c_refuse_bus) {
        return ESP_FAIL; /* the port is already in use */
    }
    for (unsigned i = 0; i < MODEL_I2C_BUSES; i++) {
        if (!i2c_buses[i].up) {
            i2c_buses[i].up = 1;
            i2c_buses[i].port = bus_config->i2c_port;
            i2c_buses[i].sda = bus_config->sda_io_num;
            i2c_buses[i].scl = bus_config->scl_io_num;
            i2c_buses[i].pullup = (int)bus_config->flags.enable_internal_pullup;
            *ret_bus_handle = &i2c_buses[i];
            return ESP_OK;
        }
    }
    return ESP_FAIL;
}

esp_err_t i2c_master_bus_add_device(i2c_master_bus_handle_t bus_handle,
                                    const i2c_device_config_t *dev_config,
                                    i2c_master_dev_handle_t *ret_handle)
{
    (void)bus_handle;
    if (dev_config == NULL || ret_handle == NULL) {
        return ESP_FAIL;
    }
    if (i2c_refuse_device) {
        return ESP_FAIL; /* no room for another address on the bus */
    }
    for (unsigned i = 0; i < MODEL_I2C_DEVICES; i++) {
        if (!i2c_devices[i].added) {
            i2c_devices[i].added = 1;
            i2c_devices[i].address = dev_config->device_address;
            i2c_devices[i].speed_hz = dev_config->scl_speed_hz;
            *ret_handle = &i2c_devices[i];
            return ESP_OK;
        }
    }
    return ESP_FAIL;
}

esp_err_t i2c_master_bus_rm_device(i2c_master_dev_handle_t handle)
{
    if (handle == NULL) {
        return ESP_FAIL;
    }
    handle->added = 0;
    return ESP_OK;
}

esp_err_t i2c_master_transmit(i2c_master_dev_handle_t i2c_dev,
                              const uint8_t *write_buffer, size_t write_size,
                              int xfer_timeout_ms)
{
    (void)xfer_timeout_ms;
    if (i2c_dev == NULL || write_buffer == NULL || !i2c_present) {
        return ESP_FAIL;
    }
    i2c_dev->transmits++;
    i2c_dev->last_write_bytes = (unsigned)write_size;
    i2c_dev->last_address = i2c_dev->address;
    if (write_size > MODEL_FRAME) {
        return ESP_FAIL;
    }
    for (unsigned i = 0; i < write_size; i++) {
        i2c_dev->write[i] = write_buffer[i];
    }
    /* A write is "this register, then these bytes". */
    if (write_size >= 1u) {
        i2c_pointer = write_buffer[0];
        for (unsigned i = 1; i < write_size; i++) {
            i2c_registers[(i2c_pointer + i - 1u) & 0xFFu] = write_buffer[i];
        }
    }
    return ESP_OK;
}

esp_err_t i2c_master_transmit_receive(i2c_master_dev_handle_t i2c_dev,
                                      const uint8_t *write_buffer,
                                      size_t write_size, uint8_t *read_buffer,
                                      size_t read_size, int xfer_timeout_ms)
{
    (void)xfer_timeout_ms;
    if (i2c_dev == NULL || write_buffer == NULL || read_buffer == NULL ||
        !i2c_present) {
        return ESP_FAIL;
    }
    i2c_dev->receives++;
    i2c_dev->last_write_bytes = (unsigned)write_size;
    i2c_dev->last_read_bytes = (unsigned)read_size;
    i2c_dev->last_address = i2c_dev->address;
    if (write_size > MODEL_FRAME) {
        return ESP_FAIL;
    }
    for (unsigned i = 0; i < write_size; i++) {
        i2c_dev->write[i] = write_buffer[i];
    }
    /* The register byte moves the pointer and the part auto-increments from
     * there, which is the whole reason a register read is one transaction. */
    i2c_pointer = write_buffer[0];
    for (unsigned i = 0; i < read_size; i++) {
        read_buffer[i] = i2c_registers[(i2c_pointer + i) & 0xFFu];
    }
    return ESP_OK;
}

esp_err_t i2c_master_probe(i2c_master_bus_handle_t bus_handle,
                           uint16_t address, int xfer_timeout_ms)
{
    (void)xfer_timeout_ms;
    if (bus_handle == NULL) {
        return ESP_FAIL;
    }
    i2c_probes++;
    for (unsigned i = 0; i < MODEL_I2C_DEVICES; i++) {
        if (i2c_devices[i].added && i2c_devices[i].address == address) {
            return i2c_present ? ESP_OK : ESP_FAIL;
        }
    }
    return ESP_FAIL;
}

unsigned ak_host_i2c_buses(void)
{
    unsigned count = 0u;
    for (unsigned i = 0; i < MODEL_I2C_BUSES; i++) {
        count += i2c_buses[i].up ? 1u : 0u;
    }
    return count;
}

unsigned ak_host_i2c_devices(void)
{
    unsigned count = 0u;
    for (unsigned i = 0; i < MODEL_I2C_DEVICES; i++) {
        count += i2c_devices[i].added ? 1u : 0u;
    }
    return count;
}

int ak_host_i2c_device_present(unsigned device)
{
    return device < MODEL_I2C_DEVICES ? i2c_devices[device].added : 0;
}

uint16_t ak_host_i2c_device_address(unsigned device)
{
    return device < MODEL_I2C_DEVICES ? i2c_devices[device].address : 0u;
}

uint32_t ak_host_i2c_device_speed(unsigned device)
{
    return device < MODEL_I2C_DEVICES ? i2c_devices[device].speed_hz : 0u;
}

unsigned ak_host_i2c_transmits(unsigned device)
{
    return device < MODEL_I2C_DEVICES ? i2c_devices[device].transmits : 0u;
}

unsigned ak_host_i2c_receives(unsigned device)
{
    return device < MODEL_I2C_DEVICES ? i2c_devices[device].receives : 0u;
}

unsigned ak_host_i2c_probes(void)
{
    return i2c_probes;
}

const uint8_t *ak_host_i2c_last_write(unsigned device)
{
    return device < MODEL_I2C_DEVICES ? i2c_devices[device].write : NULL;
}

unsigned ak_host_i2c_last_write_bytes(unsigned device)
{
    return device < MODEL_I2C_DEVICES ? i2c_devices[device].last_write_bytes : 0u;
}

unsigned ak_host_i2c_last_read_bytes(unsigned device)
{
    return device < MODEL_I2C_DEVICES ? i2c_devices[device].last_read_bytes : 0u;
}

int ak_host_i2c_last_address(unsigned device)
{
    return device < MODEL_I2C_DEVICES ? i2c_devices[device].last_address : -1;
}

void ak_host_i2c_set_present(int present)
{
    i2c_present = present != 0;
}

void ak_host_i2c_set_register(uint8_t reg, uint8_t value)
{
    i2c_registers[reg] = value;
}

void ak_host_i2c_set_burst(uint8_t reg, const uint8_t *bytes, unsigned len)
{
    for (unsigned i = 0; i < len; i++) {
        i2c_registers[(reg + i) & 0xFFu] = bytes[i];
    }
}

uint8_t ak_host_i2c_register(uint8_t reg)
{
    return i2c_registers[reg];
}

uint8_t ak_host_i2c_pointer(void)
{
    return i2c_pointer;
}
