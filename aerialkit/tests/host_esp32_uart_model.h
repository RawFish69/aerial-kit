#ifndef AK_HOST_ESP32_UART_MODEL_H
#define AK_HOST_ESP32_UART_MODEL_H

#include <stdint.h>

/*
 * The ESP32's two UARTs, run on the host against a modelled IDF driver.
 *
 * `src/arch/esp32/uart.c` is the receiver's and the GPS's port, and the two
 * things it exists to get right are both about the *driver* rather than the
 * part: the receive ring lives inside IDF, and so does the only statement
 * about bytes it could not keep. Nothing on this machine had ever run it - no
 * receiver and no GPS have been wired to this port, and QEMU's ESP32 has no
 * serial device on those pins - so "the port comes up and the firmware runs"
 * was the whole claim, which is exactly the claim the outputs and the sensor
 * buses turned out not to be enough for.
 *
 * So the file is compiled against a stand-in for IDF's UART driver and this
 * model behind it: a ring the test fills byte by byte (the same "reach in
 * where the hardware would" the register-block tests do with a peripheral's
 * flags), an event queue the port is supposed to drain, a transmit buffer that
 * records what went out, and failure switches for the paths a bench cannot
 * arrange on purpose - an install that is refused, a write that goes out
 * short, a transmitter that never finishes.
 */

void ak_host_uart_reset(void);

/* What the port configured, as the driver would have seen it. */
int ak_host_uart_installed(unsigned port);
int ak_host_uart_rx_buffer(unsigned port);
int ak_host_uart_tx_buffer(unsigned port);
int ak_host_uart_event_depth(unsigned port);
int ak_host_uart_baud(unsigned port);
int ak_host_uart_parity(unsigned port);
int ak_host_uart_stop_bits(unsigned port);
int ak_host_uart_data_bits(unsigned port);
int ak_host_uart_inverted(unsigned port);
int ak_host_uart_tx_pin(unsigned port);
int ak_host_uart_rx_pin(unsigned port);

/* What has happened to it. */
unsigned ak_host_uart_flushes(unsigned port);
unsigned ak_host_uart_reads(unsigned port);
const uint8_t *ak_host_uart_tx(unsigned port);
unsigned ak_host_uart_tx_bytes(unsigned port);

/* The device on the other end. */
void ak_host_uart_feed(unsigned port, const uint8_t *bytes, unsigned len);
unsigned ak_host_uart_pending(unsigned port);

/* The driver's own event queue: the port is supposed to drain it, and an
 * event left in it is an overflow the console never hears about. */
void ak_host_uart_push_event(unsigned port, int type);
unsigned ak_host_uart_events(unsigned port);

/* The failure paths. */
void ak_host_uart_set_install_refused(int refused);
/* Or the driver installs and the *parameters* are refused - a pin the GPIO
 * matrix cannot route, which is a wiring mistake rather than a busy port, and
 * the port's answer to it is to tear the driver down again and say so. */
void ak_host_uart_set_param_refused(int refused);
void ak_host_uart_set_write_short(int bytes_short);
void ak_host_uart_set_tx_stuck(int stuck);

#endif /* AK_HOST_ESP32_UART_MODEL_H */
