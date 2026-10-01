#include "esp.h"

#include "driver/gpio.h"
#include "driver/spi_master.h"

#include "freertos/FreeRTOS.h"

/*
 * The IMU's SPI bus, and the loopback that is the only way to check it with no
 * sensor on the bench.
 *
 * The F405 drives SPI with its own registers: write the data register, wait
 * for the flags, hold the chip select down itself. Here a driver owns the
 * peripheral, the chip select moves per transaction, and the interesting
 * decisions are all about what happens when a device does *not* answer.
 *
 * That is the second half of the lesson the outputs taught on this port. An
 * SPI transfer whose peripheral never completes leaves the driver waiting on a
 * semaphore, and IDF's own convenience call - spi_device_transmit() - waits
 * forever for it. This file therefore never uses it: every transfer is queued
 * and collected with a bound, exactly the way rmt_transmit() had to be told
 * not to queue-block. A bus that stops answering costs a failed read and a
 * counter, not the aircraft.
 *
 * The frame shape is the F405's, deliberately: an InvenSense part wants the
 * register byte and the data in one chip-select window, so the read is one
 * transaction with the address first and a byte of padding per byte read,
 * rather than two transactions with the select bouncing between them. A driver
 * that never learns which chip it is on is the point of ak_bus.h, and this is
 * the same register protocol through different hardware.
 *
 * Nothing here has been on a board. There is no IMU on a devkit and QEMU's
 * ESP32 does not model this peripheral, so what the emulator checks is that
 * the port comes up, reports honestly when it cannot, and leaves the flight
 * loop running either way.
 */

#define AK_ESP_SPI_HOST       SPI2_HOST
#define AK_ESP_SPI_CLOCK_HZ   1000000 /* 1 MHz: a bring-up clock, not a limit */
#define AK_ESP_SPI_QUEUE      4
#define AK_ESP_SPI_WAIT_TICKS pdMS_TO_TICKS(50)
#define AK_ESP_SPI_FRAME_MAX  2048u
#define AK_ESP_SPI_BURST      1024u

/* 0 = not tried, 1 = up, -1 = the bus refused to come up. A refusal is not
 * retried: a peripheral that did not initialise will not initialise on the
 * next call either, and a retry per read is a bus that costs the loop time to
 * say the same thing again. */
static int bus_state;

static spi_device_handle_t imu_dev;
static spi_device_handle_t bare_dev; /* no chip select: the loopback's device */

/* One transaction descriptor and two frame buffers, module-scope rather than
 * on the stack: a queued transaction is the driver's until it is collected,
 * and the flight task's stack is not the place to keep either one. */
static spi_transaction_t trans;
static uint8_t frame_tx[AK_ESP_SPI_FRAME_MAX];
static uint8_t frame_rx[AK_ESP_SPI_FRAME_MAX];

static uint32_t transfers;
static uint32_t timeouts;

static int bus_up(int sck_gpio, int miso_gpio, int mosi_gpio)
{
    spi_bus_config_t bus = { 0 };
    esp_err_t status;

    if (bus_state != 0) {
        return bus_state == 1 ? 0 : -1;
    }

    bus.mosi_io_num = mosi_gpio;
    bus.miso_io_num = miso_gpio;
    bus.sclk_io_num = sck_gpio;
    bus.quadwp_io_num = -1;
    bus.quadhd_io_num = -1;
    bus.max_transfer_sz = AK_ESP_SPI_FRAME_MAX;

    status = spi_bus_initialize(AK_ESP_SPI_HOST, &bus, SPI_DMA_CH_AUTO);
    if (status != ESP_OK && status != ESP_ERR_INVALID_STATE) {
        bus_state = -1;
        return -1;
    }
    bus_state = 1;
    return 0;
}

/*
 * One frame, with a bound on the wait.
 *
 * spi_device_queue_trans() is called with a zero tick timeout - a full queue
 * is a fact, not something to wait for - and the result is collected with a
 * real but finite one. Reaching that bound means the peripheral stopped
 * answering, which is counted and reported rather than waited on.
 */
static int frame(spi_device_handle_t dev, const uint8_t *tx, uint8_t *rx,
                 unsigned len, int keep_selected)
{
    spi_transaction_t *done = 0;

    if (dev == 0 || len == 0u || len > AK_ESP_SPI_FRAME_MAX) {
        return -1;
    }

    trans = (spi_transaction_t){ 0 };
    trans.length = len * 8u;
    trans.tx_buffer = tx;
    trans.rx_buffer = rx;
    if (keep_selected) {
        trans.flags = SPI_TRANS_CS_KEEP_ACTIVE;
    }

    if (spi_device_queue_trans(dev, &trans, 0) != ESP_OK) {
        timeouts++;
        return -1;
    }
    if (spi_device_get_trans_result(dev, &done, AK_ESP_SPI_WAIT_TICKS) !=
        ESP_OK) {
        timeouts++;
        return -1;
    }
    transfers++;
    return 0;
}

/* --- the IMU's bus --------------------------------------------------------
 *
 * The same three functions the F405 hands the driver: a register read, a
 * register write, and the burst the BMI270's firmware upload needs (which is
 * a chip select held down across many frames, so the chunks here are chained
 * with SPI_TRANS_CS_KEEP_ACTIVE rather than being separate transactions). */

static int imu_read(void *ctx, uint8_t reg, uint8_t *buf, unsigned len)
{
    uint8_t address = (uint8_t)(reg | 0x80u);

    (void)ctx;
    if (imu_dev == 0 || len + 1u > AK_ESP_SPI_FRAME_MAX) {
        return -1;
    }

    frame_tx[0] = address;
    for (unsigned i = 0; i < len; i++) {
        frame_tx[i + 1u] = 0xFFu; /* what a bus reads with nothing to say */
    }
    if (frame(imu_dev, frame_tx, frame_rx, len + 1u, 0) != 0) {
        return -1;
    }
    for (unsigned i = 0; i < len; i++) {
        buf[i] = frame_rx[i + 1u];
    }
    return 0;
}

static int imu_write(void *ctx, uint8_t reg, uint8_t value)
{
    (void)ctx;
    if (imu_dev == 0) {
        return -1;
    }
    frame_tx[0] = (uint8_t)(reg & 0x7Fu);
    frame_tx[1] = value;
    return frame(imu_dev, frame_tx, 0, 2u, 0);
}

static int imu_write_burst(void *ctx, uint8_t reg, const uint8_t *buf,
                           unsigned len)
{
    unsigned first;
    unsigned at;

    (void)ctx;
    /* Two different reasons, said separately: nothing to write is not a
     * transaction, and a device that was never opened is an error. Written as
     * one expression, the compiler folds the second `len == 0` test into the
     * first and the coverage map loses the line - which is how this was found
     * (the map, not a bug). */
    if (imu_dev == 0) {
        return -1;
    }
    if (len == 0u) {
        return 0;
    }

    /* The register byte goes out once, at the head of the first frame, and
     * every frame after it is more data for that register - which is what
     * holding the select down across the chunks means. The chunking is a bound
     * on the buffer, not a change of transaction. */
    first = len < AK_ESP_SPI_BURST - 1u ? len : AK_ESP_SPI_BURST - 1u;
    frame_tx[0] = (uint8_t)(reg & 0x7Fu);
    for (unsigned i = 0; i < first; i++) {
        frame_tx[i + 1u] = buf[i];
    }
    at = first;
    if (frame(imu_dev, frame_tx, 0, first + 1u, at < len) != 0) {
        return -1;
    }

    while (at < len) {
        unsigned chunk = len - at;
        if (chunk > AK_ESP_SPI_BURST) {
            chunk = AK_ESP_SPI_BURST;
        }
        for (unsigned i = 0; i < chunk; i++) {
            frame_tx[i] = buf[at + i];
        }
        if (frame(imu_dev, frame_tx, 0, chunk, at + chunk < len) != 0) {
            return -1;
        }
        at += chunk;
    }
    return 0;
}

static void imu_delay(void *ctx, unsigned ms)
{
    (void)ctx;
    ak_delay_ms(ms);
}

static const ak_bus_t imu_bus = {
    .read = imu_read,
    .write = imu_write,
    .write_burst = imu_write_burst,
    .delay_ms = imu_delay,
    .ctx = 0,
};

const ak_bus_t *ak_esp_spi_imu_bus(int sck_gpio, int miso_gpio, int mosi_gpio,
                                   int cs_gpio)
{
    spi_device_interface_config_t dev = { 0 };

    if (bus_up(sck_gpio, miso_gpio, mosi_gpio) != 0) {
        return 0;
    }
    if (imu_dev == 0) {
        dev.clock_speed_hz = AK_ESP_SPI_CLOCK_HZ;
        dev.mode = 3; /* the InvenSense parts, and the F405 port's choice */
        dev.spics_io_num = cs_gpio;
        dev.queue_size = AK_ESP_SPI_QUEUE;
        if (spi_bus_add_device(AK_ESP_SPI_HOST, &dev, &imu_dev) != ESP_OK) {
            imu_dev = 0;
            return 0;
        }
    }
    return &imu_bus;
}

/* --- the loopback ---------------------------------------------------------
 *
 * The same check the F405's console offers, for the same reason: with no part
 * on the bus, one jumper from MOSI to MISO is the only thing that can tell a
 * working port from a port that never moves a clock. The device used here has
 * no chip select at all, because this is a test of the wires.
 */

#ifdef AK_HOST_SPI_ESP
/*
 * The host build's way to ask again.
 *
 * `bus_state` is deliberately one attempt per boot - 0 untried, 1 up, -1 failed
 * and never retried - which is what a board does. A test that has just made the
 * peripheral refuse needs that forgotten, or its next call would be testing the
 * cache rather than the refusal. Compiled only for the host build (the Makefile
 * passes -DAK_HOST_SPI_ESP on this object alone, the same seam the ADC's and the
 * F405's flash and I2C tests use), so the image never contains it.
 */
void ak_esp_spi_forget_for_host(void)
{
    bus_state = 0;
    imu_dev = 0;
    bare_dev = 0;
    transfers = 0;
    timeouts = 0;
}
#endif

void ak_esp_spi_loopback(int sck_gpio, int miso_gpio, int mosi_gpio,
                         ak_printf_fn out)
{
    static const uint8_t pattern[6] = { 0x55, 0xAA, 0x00, 0xFF, 0x5A, 0xA5 };
    spi_device_interface_config_t dev = { 0 };

    if (bus_up(sck_gpio, miso_gpio, mosi_gpio) != 0) {
        out("spi:       the spi bus did not come up on this chip\n");
        return;
    }
    if (bare_dev == 0) {
        dev.clock_speed_hz = AK_ESP_SPI_CLOCK_HZ;
        dev.mode = 3;
        dev.spics_io_num = -1; /* no select: nothing is being addressed */
        dev.queue_size = AK_ESP_SPI_QUEUE;
        if (spi_bus_add_device(AK_ESP_SPI_HOST, &dev, &bare_dev) != ESP_OK) {
            bare_dev = 0;
            out("spi:       no device could be added to the bus\n");
            return;
        }
    }

    for (unsigned i = 0; i < sizeof pattern; i++) {
        frame_rx[i] = 0;
    }
    if (frame(bare_dev, pattern, frame_rx, sizeof pattern, 0) != 0) {
        out("spi:       transfer timed out after %u transfers\n", transfers);
        return;
    }

    int same = 1;
    for (unsigned i = 0; i < sizeof pattern; i++) {
        same = same && frame_rx[i] == pattern[i];
    }

    out("spi:       sent    ");
    for (unsigned i = 0; i < sizeof pattern; i++) {
        out("%02x ", pattern[i]);
    }
    out("\nspi:       read    ");
    for (unsigned i = 0; i < sizeof pattern; i++) {
        out("%02x ", frame_rx[i]);
    }
    out("\nspi:       %s\n",
        same ? "loopback matches - MOSI reaches MISO"
             : "no match - what a floating MISO does; jumper GPIO23 to GPIO19");
    out("spi:       %u transfers, %u timed out\n", transfers, timeouts);
}
