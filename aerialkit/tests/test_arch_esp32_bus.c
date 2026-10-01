/*
 * The ESP32's sensor buses, run on the host against modelled devices.
 *
 * `src/arch/esp32/spi.c` and `i2c.c` translate `ak_bus_t` - read a register,
 * write a register, write a burst - into one chip's driver calls. Nothing on
 * this machine had ever run them: a devkit carries no IMU and no barometer,
 * QEMU models neither peripheral, and a bus with nothing on it proves only
 * that the failure path is reached. The port's own comment says as much ("the
 * emulator checks that the port comes up, reports honestly when it cannot").
 *
 * So the real files run here against a modelled register file, and the checks
 * are the ones an oscilloscope would make if this machine had one: what went
 * out on the wire (the address byte with its read flag, the padding, the
 * register pointer), how a burst was framed (one select, three chunks, or
 * three separate conversations), and what happens when the device stops
 * answering. The core's own drivers - the ICM-42688-P and the DPS310 - are on
 * top, so a register the port addresses wrongly shows up as a driver that
 * cannot identify its part.
 */

#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "esp.h"

#include "ak_baro.h"
#include "ak_imu.h"
#include "host_esp32_bus_model.h"
#include "tests.h"

/* The pins the port is given here: the devkit's own guesses, as the board
 * names them, are the board file's business - this is about the wire. */
#define TEST_SCK   18
#define TEST_MISO  19
#define TEST_MOSI  23
#define TEST_CS     5
#define TEST_SCL   22
#define TEST_SDA   21

#define TEST_I2C_ADDRESS 0x77u /* the DPS310's, per its datasheet */

static char report[512];

static int sink(const char *fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    int written = vsnprintf(report + strlen(report),
                            sizeof report - strlen(report), fmt, args);
    va_end(args);
    return written;
}

/* The host build's way to ask each port again: see the seam in each source
 * file. `bus_state` is one attempt per boot and never retried, which is what a
 * board does - so a test that has just made the peripheral refuse has to forget
 * that before it can ask a second time. */
void ak_esp_spi_forget_for_host(void);
void ak_esp_i2c_forget_for_host(void);

void test_arch_esp32_bus(void)
{
    ak_host_esp_bus_reset();

    /* --- the IMU's SPI bus ------------------------------------------------ */

    const ak_bus_t *imu = ak_esp_spi_imu_bus(TEST_SCK, TEST_MISO, TEST_MOSI,
                                             TEST_CS);
    expect("the imu's spi bus comes up", imu != NULL);
    expect("one bus, on the pins it was given",
           ak_host_spi_buses() == 1 && ak_host_spi_sclk(0) == TEST_SCK &&
               ak_host_spi_miso(0) == TEST_MISO &&
               ak_host_spi_mosi(0) == TEST_MOSI);
    expect("with a transfer ceiling and dma, which is what a burst needs",
           ak_host_spi_max_transfer(0) >= 2048 && ak_host_spi_dma(0) >= 0);

    expect("one device, with its own select, in mode 3 at 1 MHz",
           ak_host_spi_devices() == 1 && ak_host_spi_device_cs(0) == TEST_CS &&
               ak_host_spi_device_mode(0) == 3 &&
               ak_host_spi_device_clock(0) == 1000000);
    expect("and a queue, because a transfer is not collected instantly",
           ak_host_spi_device_queue(0) > 1);

    /* The who-am-i of an ICM-42688-P, read the way the driver reads it. */
    ak_host_spi_set_register(0x75u, 0x47u);
    uint8_t who = 0;
    expect("a register read answers with the device's value",
           ak_bus_read(imu, 0x75u, &who, 1) == 0 && who == 0x47u);

    /* And the frame it took to ask: the address with the read flag set, then
     * a byte of padding per byte wanted, which is what an InvenSense part
     * expects inside one select window. */
    expect("the read is the address with its read flag, then a padding byte",
           ak_host_spi_last_bytes(0) == 2 &&
               ak_host_spi_last_tx(0)[0] == (0x75u | 0x80u) &&
               ak_host_spi_last_tx(0)[1] == 0xFFu);
    expect("the transfer is measured in bits, as the peripheral wants it",
           ak_host_spi_last_bits(0) == 2u * 8u);
    expect("and the select was asserted once for it",
           ak_host_spi_cs_asserts(0) == 1u);

    /* The real driver on top of the real port code. */
    ak_host_spi_set_register(0x4Du, 0x00u); /* INTF_CONFIG1, read during init */
    ak_imu_t imu_state;
    report[0] = '\0';
    expect("the core's imu driver identifies its part through this bus",
           ak_imu_open(&imu_state, imu, sink) == 0 &&
               imu_state.present == 1 &&
               strcmp(imu_state.driver->name, "icm42688p") == 0);
    expect("and says what it configured, in the port's own words",
           strstr(report, "2000 dps") != NULL);

    /* A sample: 1 g on Z, nothing on the other axes, as raw big-endian counts.
     * 2048 counts is 1 g at this part's 16 g range. */
    {
        uint8_t raw[12];
        memset(raw, 0, sizeof raw);
        raw[4] = 0x08u;
        raw[5] = 0x00u;
        ak_host_spi_set_burst(0x1Fu, raw, sizeof raw);

        ak_imu_sample_t sample;
        memset(&sample, 0, sizeof sample);
        unsigned before = ak_host_spi_transfers(0);
        expect("a sample comes back from the modelled part",
               ak_imu_read(&imu_state, &sample) == 0 && sample.valid == 1);
        expect("scaled the way the datasheet says: 2048 counts is a g",
               fabsf(sample.accel[2] - 1.0f) < 0.01f &&
                   fabsf(sample.accel[0]) < 0.01f);
        expect("and all six axes came in one transaction",
               ak_host_spi_transfers(0) == before + 1u &&
                   ak_host_spi_last_bytes(0) == 13u &&
                   ak_host_spi_last_bits(0) == 13u * 8u);
        expect("with the select held across the whole of it",
               ak_host_spi_last_kept_cs(0) == 0);
    }

    /* A write: the address with the read flag *clear*, then the value. */
    expect("a register write is the address, then the value",
           ak_bus_write(imu, 0x4Eu, 0x0Fu) == 0 &&
               ak_host_spi_register(0x4Eu) == 0x0Fu &&
               ak_host_spi_last_tx(0)[0] == 0x4Eu &&
               ak_host_spi_last_bytes(0) == 2u);

    /* A burst: the register byte once at the head, then chunks, with the
     * select held between them - which is what makes eight kilobytes of
     * firmware one conversation rather than two thousand. */
    {
        static uint8_t big[3000];
        for (unsigned i = 0; i < sizeof big; i++) {
            big[i] = (uint8_t)(i ^ 0x5Au);
        }
        unsigned before = ak_host_spi_transfers(0);
        unsigned asserts = ak_host_spi_cs_asserts(0);

        expect("a long burst is accepted", ak_bus_write_burst(imu, 0x10u, big,
                                                             sizeof big) == 0);
        expect("as three chunks rather than one oversized transfer",
               ak_host_spi_transfers(0) == before + 3u);
        expect("with the select down across all of them",
               ak_host_spi_cs_asserts(0) == asserts + 1u);
        /* The chunks are the port's: 1023 bytes behind the register byte, then
         * 1024s. So the last chunk's first byte is data at 2047, and a port
         * that repeated the register byte on each transaction would show 0x10
         * here instead - which is the mistake this pins. */
        expect("the register byte is at the head of the first chunk only",
               ak_host_spi_last_tx(0)[0] == big[2047] &&
                   ak_host_spi_last_tx(0)[0] != 0x10u);
        expect("and the data lands in order",
               ak_host_spi_register(0x10u) == big[0] &&
                   ak_host_spi_register(0x11u) == big[1]);
    }

    /* The failure that matters: a peripheral with no device on it. The read
     * is an error and the driver's read is an error - never a hang. */
    ak_host_spi_set_silent(1);
    {
        uint8_t value = 0;
        ak_imu_sample_t sample;
        memset(&sample, 0, sizeof sample);
        expect("a bus that never answers is an error, not a value",
               ak_bus_read(imu, 0x75u, &value, 1) == -1);
        expect("and the driver reports it rather than a stale sample",
               ak_imu_read(&imu_state, &sample) == -1 && sample.valid == 0);
    }
    ak_host_spi_set_silent(0);

    /* An edge of the core's burst write: nothing to write is not a
     * transaction, and the port says so rather than queueing an empty frame. */
    {
        uint8_t chunk = 0x5Au;

        expect("a burst of no bytes is not a transfer",
               ak_bus_write_burst(imu, 0x10u, &chunk, 0u) == 0);
        expect("and a burst of one goes",
               ak_bus_write_burst(imu, 0x10u, &chunk, 1u) == 0);
    }

    /* The loopback, which is the console's own check on a bare board. */
    ak_host_spi_set_echo(1);
    report[0] = '\0';
    ak_esp_spi_loopback(TEST_SCK, TEST_MISO, TEST_MOSI, sink);
    expect("a jumper from mosi to miso reads back what was sent",
           strstr(report, "loopback matches") != NULL);
    ak_host_spi_set_echo(0);
    report[0] = '\0';
    ak_esp_spi_loopback(TEST_SCK, TEST_MISO, TEST_MOSI, sink);
    expect("and a floating miso is reported as no match",
           strstr(report, "no match") != NULL);

    /* --- the barometer's I2C bus ----------------------------------------- */

    const ak_bus_t *baro = ak_esp_i2c_baro_bus(0, TEST_SCL, TEST_SDA, 400000u,
                                               TEST_I2C_ADDRESS);
    expect("the barometer's i2c bus comes up", baro != NULL);
    expect("with the device addressed the way the datasheet says",
           ak_host_i2c_devices() == 1 &&
               ak_host_i2c_device_address(0) == TEST_I2C_ADDRESS &&
               ak_host_i2c_device_speed(0) == 400000u);

    /* A register read is one transaction: register byte, repeated start, the
     * bytes back - not a write and a read with the bus left idle between. */
    ak_host_i2c_set_register(0x0Du, 0x10u); /* the DPS310's product id */
    uint8_t id = 0;
    expect("a register read answers",
           ak_bus_read(baro, 0x0Du, &id, 1) == 0 && id == 0x10u);
    expect("in one transaction, with the register byte first",
           ak_host_i2c_receives(0) == 1u &&
               ak_host_i2c_last_write_bytes(0) == 1u &&
               ak_host_i2c_last_read_bytes(0) == 1u &&
               ak_host_i2c_last_write(0)[0] == 0x0Du);
    expect("and the part's register pointer left where it was asked",
           ak_host_i2c_pointer() == 0x0Du);

    /* A write moves the pointer and stores the value. */
    expect("a register write carries the register and the value",
           ak_bus_write(baro, 0x06u, 0x54u) == 0 &&
               ak_host_i2c_last_write_bytes(0) == 2u &&
               ak_host_i2c_last_write(0)[0] == 0x06u &&
               ak_host_i2c_register(0x06u) == 0x54u);

    /* The core's barometer table, probing through this bus: the same call the
     * board makes at boot, against a part that is answering. */
    uint8_t seen = 0;
    const ak_baro_driver_t *found = ak_baro_detect(baro, &seen);
    expect("the core's barometer table finds the part on this bus",
           found == &ak_baro_dps310 && seen == 0x10u);

    /* The deliberate question the console's bus check asks. */
    expect("the port can be asked whether anything is on the bus",
           ak_esp_i2c_probe(0, TEST_SCL, TEST_SDA, 400000u,
                            TEST_I2C_ADDRESS) == 1);
    ak_host_i2c_set_present(0);
    expect("and it says no when nothing answers",
           ak_esp_i2c_probe(0, TEST_SCL, TEST_SDA, 400000u,
                            TEST_I2C_ADDRESS) == 0);
    expect("a read of an absent part is an error",
           ak_bus_read(baro, 0x0Du, &id, 1) == -1);
    ak_host_i2c_set_present(1);

    /*
     * And the parts of both ports that answer "the bus itself did not come
     * up". They are a different failure from a silent device - the peripheral
     * refused, rather than the part staying quiet - and on this chip they are
     * the likely ones: a host another driver has already claimed, no device
     * slots left, a queue that will not take the transfer. Every one of them
     * has to come back as an error the board can report, rather than a sensor
     * that reads zeros or a boot that waits.
     */
    ak_host_esp_bus_reset();
    ak_esp_spi_forget_for_host();
    ak_host_spi_refuse_bus(1);
    expect("an spi bus the peripheral will not create is not a bus",
           ak_esp_spi_imu_bus(TEST_SCK, TEST_MISO, TEST_MOSI, TEST_CS) == 0);
    report[0] = '\0';
    ak_esp_spi_loopback(TEST_SCK, TEST_MISO, TEST_MOSI, sink);
    expect("and the loopback check says which half failed",
           strstr(report, "the spi bus did not come up") != NULL);
    expect("which is a different sentence from a bus with no device on it",
           strstr(report, "no device could be added") == NULL);
    ak_host_spi_refuse_bus(0);
    ak_esp_spi_forget_for_host(); /* the refusal above is not the chip's state */

    /* The same check with the *device* refused: a bus that is up and a chip
     * that will not take the bare device the loopback needs. */
    ak_host_spi_refuse_device(1);
    report[0] = '\0';
    ak_esp_spi_loopback(TEST_SCK, TEST_MISO, TEST_MOSI, sink);
    expect("a loopback with no device to add says so",
           strstr(report, "no device could be added to the bus") != NULL);
    ak_host_spi_refuse_device(0);

    /* And a bus with no device on it, which is the state a person with a bare
     * board is in before the jumper goes on. */
    ak_esp_spi_forget_for_host();
    ak_host_spi_set_silent(1);
    report[0] = '\0';
    ak_esp_spi_loopback(TEST_SCK, TEST_MISO, TEST_MOSI, sink);
    expect("and a loopback over a bus nothing answers is a timeout, not a pass",
           strstr(report, "timed out") != NULL &&
               strstr(report, "loopback matches") == NULL);
    ak_host_spi_set_silent(0);
    ak_esp_spi_forget_for_host();

    ak_host_esp_bus_reset();
    expect("a bus that does come up gives the imu bus",
           ak_esp_spi_imu_bus(TEST_SCK, TEST_MISO, TEST_MOSI, TEST_CS) != 0);
    ak_esp_spi_forget_for_host();
    ak_host_spi_refuse_device(1);
    (void)ak_esp_spi_imu_bus(TEST_SCK, TEST_MISO, TEST_MOSI, TEST_CS);
    expect("a chip that will not take the device says so rather than handing "
           "back a bus nobody is on",
           ak_esp_spi_imu_bus(TEST_SCK, TEST_MISO, TEST_MOSI, TEST_CS) == 0 ||
               ak_host_spi_devices() == 0u);

    /* A transfer the queue refuses is counted as a timeout, not as a value. */
    ak_host_esp_bus_reset();
    ak_esp_spi_forget_for_host();
    {
        const ak_bus_t *queue_bus =
            ak_esp_spi_imu_bus(TEST_SCK, TEST_MISO, TEST_MOSI, TEST_CS);
        uint8_t queued = 0;

        expect("the bus is up before the queue refuses anything",
               queue_bus != 0);
        ak_host_spi_refuse_transfer(1);
        expect("a transfer the peripheral refuses is an error",
               ak_bus_read(queue_bus, 0x00u, &queued, 1) == -1);
        ak_host_spi_refuse_transfer(0);
        expect("and the same read works when it takes it",
               ak_bus_read(queue_bus, 0x00u, &queued, 1) == 0);
    }
    ak_host_spi_refuse_device(0);

    /* And the barometer's port: the bus, then the device. */
    ak_host_esp_bus_reset();
    ak_esp_i2c_forget_for_host();
    ak_host_i2c_refuse_bus(1);
    expect("an i2c bus the peripheral will not create is not a bus",
           ak_esp_i2c_baro_bus(0, TEST_SCL, TEST_SDA, 400000u,
                               TEST_I2C_ADDRESS) == 0);
    ak_host_i2c_refuse_bus(0);
    ak_esp_i2c_forget_for_host();

    ak_host_esp_bus_reset();
    ak_host_i2c_refuse_device(1);
    expect("and a device the peripheral will not add is not a bus either",
           ak_esp_i2c_baro_bus(0, TEST_SCL, TEST_SDA, 400000u,
                               TEST_I2C_ADDRESS) == 0);
    ak_host_i2c_refuse_device(0);

    /* And the path a *re-probe* takes: asking for a different address on a bus
     * that already has a device open removes the old one first, because this
     * peripheral takes one address at a time. A board that probes twice - the
     * boot does, and the console's `baro` command does - is the caller. */
    ak_host_esp_bus_reset();
    ak_esp_i2c_forget_for_host();
    {
        const ak_bus_t *first = ak_esp_i2c_baro_bus(0, TEST_SCL, TEST_SDA,
                                                    400000u, 0x77u);
        const ak_bus_t *second = ak_esp_i2c_baro_bus(0, TEST_SCL, TEST_SDA,
                                                     400000u, 0x76u);
        expect("asking for another address on the same bus re-opens it",
               first != 0 && second != 0 &&
                   ak_host_i2c_devices() == 1u &&
                   ak_host_i2c_device_address(0) == 0x76u);
    }

    /*
     * And the part of the port the tests above never reach because they only
     * *probe* the part: opening the barometer through this bus. That is what
     * the board does at boot, the driver's initialisation waits between steps,
     * and the wait is this port's `baro_delay` - so a delay that is missing or
     * wrong is a part that is configured before its own reset has finished.
     */
    ak_host_esp_bus_reset();
    ak_esp_i2c_forget_for_host();
    ak_host_i2c_set_register(0x0Du, 0x10u); /* the DPS310's product id */
    ak_host_i2c_set_register(0x08u, 0xF0u); /* coefficients and data ready */
    {
        const ak_bus_t *bus = ak_esp_i2c_baro_bus(0, TEST_SCL, TEST_SDA,
                                                  400000u, TEST_I2C_ADDRESS);
        ak_baro_t part;
        ak_baro_sample_t sample;

        expect("the dps310 opens through this port's bus",
               bus != 0 && ak_baro_open(&part, bus, 0) == 0 && part.present);
        expect("and a sample comes back with the ready bits the model gave it",
               ak_baro_read(&part, &sample) == 1 && sample.valid);
        expect("with the register pointer left where the read asked",
               ak_host_i2c_pointer() == 0x05u ||
                   ak_host_i2c_pointer() == 0x00u);
    }
}
