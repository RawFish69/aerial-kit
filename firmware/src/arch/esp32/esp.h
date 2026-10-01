#ifndef AK_ARCH_ESP32_ESP_H
#define AK_ARCH_ESP32_ESP_H

#include <stdint.h>

#include "ak_console.h"
#include "ak_flashlog.h"
#include "ak_bus.h"
#include "ak_output.h"
#include "ak_params.h"

/*
 * The ESP32 port.
 *
 * Different shape of problem from the STM32: the chip does not start the way
 * this firmware expects (the ROM bootloader loads an app image with a header, a
 * segment table and a checksum), and the reason this target exists at all is
 * the radio, which is a vendor blob rather than a register map. So ESP-IDF
 * supplies the platform - boot chain, image format, drivers, FreeRTOS - and
 * this directory supplies what IDF does not: the board contract that the
 * portable core already talks to.
 *
 * See docs/12-esp32.md for the decision and docs/17-esp32-port.md for what the
 * port does and does not do yet.
 */

/* Console (console.c): UART0, the one IDF leaves alone for us. */
void     ak_esp_console_init(uint32_t baud);
void     ak_console_attach(uint32_t port);
uint32_t ak_console_port(void);
int      ak_esp_console_poll_rx(uint8_t *byte);
uint32_t AK_ESP_CONSOLE_UART(void);

/* Time (time.c): the portable contract, satisfied by esp_timer. */
void     ak_time_init(void);
uint32_t ak_time_ms(void);
void     ak_delay_ms(uint32_t ms);

/* System (system.c) */
void     ak_arch_reset(void);
uint32_t ak_esp_cpu_mhz(void);
void     ak_esp_note_reset_reason(void);

/* LED (led.c) */
void ak_esp_led_init(int gpio);
void ak_esp_led_set(int on);
int  ak_esp_led_state(void);

/* Configuration in NVS (nvs.c) */
int ak_esp_config_read(void *buf, uint32_t len);
int ak_esp_config_write(const void *buf, uint32_t len);

/* The network (net.c): bring it up, and hand the core bytes rather than
 * sockets. ak_*_net_ready() is 0 when this board has no network at all, which
 * is a different thing from nobody being connected to it. */
void     ak_esp_net_start(void);
/* The board's own parameters: the Wi-Fi credentials, and which role the radio
 * plays. Registered into the one table so that `set`, `save` and the config
 * protocol all reach them the way they reach everything else. */
unsigned ak_esp_net_params(ak_param_t *items, unsigned count);
/* Which medium the build chose: "wifi" or "ethernet". A link that is up but
 * unreachable and a link that is up on a medium nobody expected are different
 * problems, and the address alone does not tell them apart. */
const char *ak_esp_net_medium(void);
int      ak_esp_net_poll_rx(uint8_t *byte);
unsigned ak_esp_net_write(const uint8_t *data, unsigned length);
int      ak_esp_net_connected(void);
int      ak_esp_net_ready(void);
void     ak_esp_net_report(ak_printf_fn out);

/* The blackbox in flash (flashlog.c): the partition this build reserved for it,
 * or 0 for a build whose partition table has none. The core's ak_flashlog.c
 * does the logging; this is the chip half of it. */
const ak_flashlog_store_t *ak_esp_log_store(void);

/* Motors and servos (output.c): DShot out of the RMT, one channel per motor,
 * and the servos on LEDC channels. The board's own output functions are thin
 * wrappers over these, the way its network functions are wrappers over net.c.
 *
 * The counts come from the board because the *chip* is what limits them: DShot
 * here is one RMT transmit channel per motor, and those are eight on the ESP32,
 * four on the S2 and S3 and two on the C3. A board whose chip has two says two
 * and drives two - the preflight is what tells its owner a quadrotor needs
 * four. */
void     ak_esp_output_init(const int *motor_gpios, unsigned motors,
                            const int *servo_gpios, unsigned servos);
int      ak_esp_output_ready(void);
void     ak_esp_output_write(const ak_output_frame_t *frame);
void     ak_esp_output_set_rate(uint32_t khz);
void     ak_esp_output_report(ak_printf_fn out);
uint32_t ak_esp_output_dshot_hz(void);
uint32_t ak_esp_output_dshot_period(void);
uint32_t ak_esp_output_ccr_zero(void);
uint32_t ak_esp_output_ccr_one(void);
uint32_t ak_esp_output_frames_sent(void);

/* Receiver and GPS (uart.c). Same contract as the F405's UARTs: the board
 * names the port, the pins and the baud; this layer owns the driver, the ring
 * inside it and the counters the console prints. `protocol` is the core's
 * enum: 0 CRSF (420000 8N1), 1 SBUS (100000 8E2, inverted - which this chip
 * can do in its GPIO matrix, so no transistor is needed). */
void     ak_esp_rc_init(int num, int tx_gpio, int rx_gpio, int baud,
                        int sbus_baud);
void     ak_esp_rc_set_protocol(uint32_t protocol);
int      ak_esp_rc_poll(uint8_t *byte);
uint32_t ak_esp_rc_dropped(void);
int      ak_esp_rc_inverted(void);
int      ak_esp_rc_send(const char *data, unsigned len);
void     ak_esp_gps_init(int num, int tx_gpio, int rx_gpio, int baud);
int      ak_esp_gps_poll(uint8_t *byte);
uint32_t ak_esp_gps_dropped(void);
int      ak_esp_gps_send(const char *data, unsigned len);

/* The sensor buses (i2c.c, spi.c). Both hand the core the same ak_bus_t the
 * F405 does, so the same driver code runs on either chip; both return 0 when
 * the bus or the device on it could not be brought up, which the board reports
 * as "none fitted" rather than as a fault. */
const ak_bus_t *ak_esp_i2c_baro_bus(int port, int scl_gpio, int sda_gpio,
                                    uint32_t speed_hz, uint32_t address);
int  ak_esp_i2c_probe(int port, int scl_gpio, int sda_gpio, uint32_t speed_hz,
                      uint32_t address);
const ak_bus_t *ak_esp_spi_imu_bus(int sck_gpio, int miso_gpio, int mosi_gpio,
                                   int cs_gpio);
void ak_esp_spi_loopback(int sck_gpio, int miso_gpio, int mosi_gpio,
                         ak_printf_fn out);

/* The flight pack's converter (adc.c). Unlike the F405's, which hands back
 * counts for the core to convert, this one hands back millivolts: the
 * calibration on this part is a curve in eFuse and the only honest conversion
 * is the one IDF does. The board turns that into volts at its pin and the core
 * does the rest - the ratio, the cells and the thresholds are the same code on
 * both chips. -1 means "no reading", never zero volts. */
int  ak_esp_adc_init(int unit, int channel);
int  ak_esp_adc_mv(int unit, int channel);
void ak_esp_adc_report(ak_printf_fn out);

#endif /* AK_ARCH_ESP32_ESP_H */
