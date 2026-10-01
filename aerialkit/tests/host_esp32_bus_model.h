#ifndef AK_HOST_ESP32_BUS_MODEL_H
#define AK_HOST_ESP32_BUS_MODEL_H

#include <stdint.h>

/*
 * The ESP32's two sensor buses, run on the host against modelled SPI and I2C.
 *
 * The same method the F405's buses were made believable with
 * (tests/host_i2c_model.c, docs/21-port-on-the-host.md), for the same reason:
 * `src/arch/esp32/spi.c` and `i2c.c` are translations of a bus protocol into
 * another chip's driver calls, and nothing on this machine could run them -
 * a devkit has no IMU or barometer on it, and QEMU models neither peripheral.
 * "It compiles and the board boots" was the whole claim.
 *
 * So the real files are compiled for the host against these stand-ins, the
 * core's own sensor drivers talk to them through the same `ak_bus_t` they use
 * on either target, and what the model records is what the port actually put
 * on the wire: the address byte, the padding, the register pointer, how many
 * transactions a burst became, and whether the select stayed down between
 * them.
 *
 * What is modelled is a *device*, not a peripheral: a 256-byte register file
 * that answers reads in order (the parts auto-increment, which is what makes a
 * twelve-byte burst one transaction), a register pointer that a write moves,
 * and a silent mode that answers nothing - which is the failure path that
 * matters, because a bus that never answers must cost a failed read and not a
 * hung boot.
 */

/* Fresh model, as if the chip had just come out of reset. */
void ak_host_esp_bus_reset(void);

/* --- the bus the SPI port brought up -------------------------------------- */

unsigned ak_host_spi_buses(void);
int      ak_host_spi_sclk(unsigned bus);
int      ak_host_spi_miso(unsigned bus);
int      ak_host_spi_mosi(unsigned bus);
int      ak_host_spi_max_transfer(unsigned bus);
int      ak_host_spi_dma(unsigned bus);

/* The devices added to it, in order. */
unsigned ak_host_spi_devices(void);
int      ak_host_spi_device_cs(unsigned device);
int      ak_host_spi_device_clock(unsigned device);
int      ak_host_spi_device_mode(unsigned device);
int      ak_host_spi_device_queue(unsigned device);

/* What has gone out on a device, and how it was framed. */
unsigned ak_host_spi_transfers(unsigned device);
unsigned ak_host_spi_last_bits(unsigned device);
unsigned ak_host_spi_last_bytes(unsigned device);
const uint8_t *ak_host_spi_last_tx(unsigned device);
const uint8_t *ak_host_spi_last_rx(unsigned device);
int      ak_host_spi_last_kept_cs(unsigned device);
unsigned ak_host_spi_cs_asserts(unsigned device);

/* --- the register file behind it ------------------------------------------ */

void ak_host_spi_set_register(uint8_t reg, uint8_t value);
void ak_host_spi_set_burst(uint8_t reg, const uint8_t *bytes, unsigned len);
uint8_t ak_host_spi_register(uint8_t reg);

/* Nothing answers: a transfer is queued and never completes, which is what a
 * peripheral with no device on it looks like from the driver's side. */
void ak_host_spi_set_silent(int silent);

/*
 * And the peripheral itself saying no, which is a different failure from a
 * silent device and one this chip is likely to have: a bus that another driver
 * has already claimed, a host out of device slots, a queue that will not take
 * the transfer. Each of them has to end in an error the port reports rather
 * than a boot that hangs or a sensor that reads zeros.
 */
void ak_host_spi_refuse_bus(int refuse);      /* spi_bus_initialize fails */
void ak_host_spi_refuse_device(int refuse);   /* spi_bus_add_device fails */
void ak_host_spi_refuse_transfer(int refuse); /* queue_trans fails */
void ak_host_i2c_refuse_bus(int refuse);      /* i2c_new_master_bus fails */
void ak_host_i2c_refuse_device(int refuse);   /* add_device fails */

/* The loopback: whatever goes out comes back, which is the one thing a jumper
 * from MOSI to MISO does. */
void ak_host_spi_set_echo(int echo);

/* --- the barometer's bus -------------------------------------------------- */

unsigned ak_host_i2c_buses(void);
unsigned ak_host_i2c_devices(void);
int      ak_host_i2c_device_present(unsigned device);
uint16_t ak_host_i2c_device_address(unsigned device);
uint32_t ak_host_i2c_device_speed(unsigned device);

unsigned ak_host_i2c_transmits(unsigned device);
unsigned ak_host_i2c_receives(unsigned device);
unsigned ak_host_i2c_probes(void);
const uint8_t *ak_host_i2c_last_write(unsigned device);
unsigned ak_host_i2c_last_write_bytes(unsigned device);
unsigned ak_host_i2c_last_read_bytes(unsigned device);
int      ak_host_i2c_last_address(unsigned device);

void ak_host_i2c_set_present(int present);
void ak_host_i2c_set_register(uint8_t reg, uint8_t value);
void ak_host_i2c_set_burst(uint8_t reg, const uint8_t *bytes, unsigned len);
uint8_t ak_host_i2c_register(uint8_t reg);
uint8_t ak_host_i2c_pointer(void);

#endif /* AK_HOST_ESP32_BUS_MODEL_H */
