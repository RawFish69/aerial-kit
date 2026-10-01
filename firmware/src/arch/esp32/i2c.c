#include "esp.h"

#include "driver/i2c_master.h"

/*
 * The barometer's I2C bus.
 *
 * Register-oriented, 7-bit address, polled - the same shape the F405's bus has
 * and the same one every driver in src/core/sensors expects, because ak_bus.h
 * hides the transport and this is the other transport. On this chip a driver
 * owns the peripheral and every call carries its own timeout, which is the
 * difference worth writing down: there is no wait here that is not bounded, so
 * a barometer that stops answering costs a failed read rather than a boot that
 * hangs on it. That is the same rule the outputs learned on this port, from
 * the other direction - IDF calls that *can* wait forever must not be given
 * the chance.
 *
 * The bus is only created when the board says a part is fitted. A probe
 * against an empty bus is not dangerous here, but it is not useful either, and
 * this chip has no way to tell "no part" from "a part that is not answering"
 * at boot. ak_esp_i2c_probe() is the deliberate version of that question, and
 * the console's bus check calls it: on a bare devkit it prints nothing
 * answered, which is the honest answer and the one a wiring mistake needs.
 *
 * Nothing here has been on a board. QEMU's ESP32 does not model this
 * peripheral, so the emulator's answer is not evidence about a real part.
 */

/* A sensor read is a register byte and a handful of bytes back at 400 kHz,
 * which is about a hundred microseconds. Fifty milliseconds is a bus that is
 * not answering rather than one that is slow. */
#define AK_ESP_I2C_TIMEOUT_MS 50

static int bus_state; /* 0 = not tried, 1 = up, -1 = refused */
static i2c_master_bus_handle_t bus_handle;
static i2c_master_dev_handle_t dev_handle;
static uint16_t dev_address;
static uint32_t dev_speed_hz;

static int bus_up(int port, int scl_gpio, int sda_gpio)
{
    i2c_master_bus_config_t bus = { 0 };
    esp_err_t status;

    if (bus_state != 0) {
        return bus_state == 1 ? 0 : -1;
    }

    bus.i2c_port = (i2c_port_num_t)port;
    bus.sda_io_num = (gpio_num_t)sda_gpio;
    bus.scl_io_num = (gpio_num_t)scl_gpio;
    bus.clk_source = I2C_CLK_SRC_DEFAULT;
    bus.glitch_ignore_cnt = 7;
    /* Weak, and said so: a devkit has no pull-ups on these pins, and a sensor
     * that is fitted on a board gets a real pair of resistors. The internal
     * ones are what makes a bare devkit's bus idle high instead of floating. */
    bus.flags.enable_internal_pullup = 1;

    status = i2c_new_master_bus(&bus, &bus_handle);
    if (status != ESP_OK) {
        bus_state = -1;
        return -1;
    }
    bus_state = 1;
    return 0;
}

static int device_up(uint16_t address, uint32_t speed_hz)
{
    i2c_device_config_t dev = { 0 };

    if (dev_handle != 0 && dev_address == address && dev_speed_hz == speed_hz) {
        return 0;
    }
    if (dev_handle != 0) {
        (void)i2c_master_bus_rm_device(dev_handle);
        dev_handle = 0;
    }

    dev.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    dev.device_address = address;
    dev.scl_speed_hz = speed_hz;
    if (i2c_master_bus_add_device(bus_handle, &dev, &dev_handle) != ESP_OK) {
        dev_handle = 0;
        return -1;
    }
    dev_address = address;
    dev_speed_hz = speed_hz;
    return 0;
}

/* --- the bus the barometer driver sees ------------------------------------ */

static int baro_read(void *ctx, uint8_t reg, uint8_t *buf, unsigned len)
{
    (void)ctx;
    if (dev_handle == 0) {
        return -1;
    }
    /* Register byte, repeated start, then the bytes: one transaction, which is
     * what the parts expect and what the F405's bus does by hand. */
    return i2c_master_transmit_receive(dev_handle, &reg, 1, buf, len,
                                       AK_ESP_I2C_TIMEOUT_MS) == ESP_OK
               ? 0
               : -1;
}

static int baro_write(void *ctx, uint8_t reg, uint8_t value)
{
    uint8_t bytes[2] = { reg, value };

    (void)ctx;
    if (dev_handle == 0) {
        return -1;
    }
    return i2c_master_transmit(dev_handle, bytes, sizeof bytes,
                               AK_ESP_I2C_TIMEOUT_MS) == ESP_OK
               ? 0
               : -1;
}

static void baro_delay(void *ctx, unsigned ms)
{
    (void)ctx;
    ak_delay_ms(ms);
}

static const ak_bus_t baro_bus = {
    .read = baro_read,
    .write = baro_write,
    /* No burst: every register a barometer takes is one or two bytes, and the
     * interface says a null here means "one byte at a time, please". */
    .write_burst = 0,
    .delay_ms = baro_delay,
    .ctx = 0,
};

#ifdef AK_HOST_I2C_ESP
/* And the same seam for the barometer's bus: see the SPI port's. */
void ak_esp_i2c_forget_for_host(void)
{
    bus_state = 0;
    bus_handle = 0;
    dev_handle = 0;
    dev_address = 0;
    dev_speed_hz = 0;
}
#endif

const ak_bus_t *ak_esp_i2c_baro_bus(int port, int scl_gpio, int sda_gpio,
                                    uint32_t speed_hz, uint32_t address)
{
    if (bus_up(port, scl_gpio, sda_gpio) != 0) {
        return 0;
    }
    if (device_up((uint16_t)address, speed_hz) != 0) {
        return 0;
    }
    return &baro_bus;
}

int ak_esp_i2c_probe(int port, int scl_gpio, int sda_gpio, uint32_t speed_hz,
                     uint32_t address)
{
    if (bus_up(port, scl_gpio, sda_gpio) != 0) {
        return -1;
    }
    if (device_up((uint16_t)address, speed_hz) != 0) {
        return -1;
    }
    return i2c_master_probe(bus_handle, (uint16_t)address,
                            AK_ESP_I2C_TIMEOUT_MS) == ESP_OK
               ? 1
               : 0;
}
