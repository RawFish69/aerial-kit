/*
 * The modelled UART driver, and the IDF calls `src/arch/esp32/uart.c` makes.
 * Compiled *instead of* IDF, so the port's own file runs against it.
 *
 * The two calls worth being careful about are the ones the port's design turns
 * on: `uart_read_bytes()` is asked for one byte with a zero timeout (the
 * caller is the flight loop, and a port with nothing on it must not cost a
 * pass), and `xQueueReceive()` is how the driver's events - the only statement
 * about a buffer overflow - reach the port at all.
 */

#include <string.h>

#include "driver/uart.h"
#include "freertos/queue.h"

#include "host_esp32_uart_model.h"

#define MODEL_PORTS 4u
#define MODEL_RING 2048u
#define MODEL_TX 1024u
#define MODEL_EVENTS 32u

struct host_queue {
    uart_event_t items[MODEL_EVENTS];
    unsigned head;
    unsigned tail;
};

struct host_uart {
    int installed;
    int rx_buffer;
    int tx_buffer;
    int event_depth;
    int baud;
    int data_bits;
    int parity;
    int stop_bits;
    int inverted;
    int tx_pin;
    int rx_pin;
    unsigned flushes;
    unsigned reads;
    uint8_t ring[MODEL_RING];
    unsigned head;
    unsigned tail;
    uint8_t tx[MODEL_TX];
    unsigned tx_len;
    struct host_queue events;
    struct host_queue *queue_ptr;
};

static struct host_uart uarts[MODEL_PORTS];

static int install_refused;
static int param_refused;
static int write_short;
static int tx_stuck;

void ak_host_uart_reset(void)
{
    memset(uarts, 0, sizeof uarts);
    install_refused = 0;
    param_refused = 0;
    write_short = 0;
    tx_stuck = 0;
}

int xQueueReceive(QueueHandle_t queue, void *item, uint32_t ticks_to_wait)
{
    (void)ticks_to_wait; /* the port always passes zero, and there is no clock */
    if (queue == 0 || item == 0 || queue->head == queue->tail) {
        return pdFALSE;
    }
    *(uart_event_t *)item = queue->items[queue->head];
    queue->head = (queue->head + 1u) % MODEL_EVENTS;
    return pdTRUE;
}

esp_err_t uart_driver_install(uart_port_t uart_num, int rx_buffer_size,
                              int tx_buffer_size, int queue_size,
                              QueueHandle_t *uart_queue, int intr_alloc_flags)
{
    (void)intr_alloc_flags;
    if (uart_num < 0 || (unsigned)uart_num >= MODEL_PORTS) {
        return ESP_FAIL;
    }
    if (install_refused) {
        return ESP_FAIL; /* what a port whose pins another driver owns does */
    }
    struct host_uart *u = &uarts[uart_num];
    u->installed = 1;
    u->rx_buffer = rx_buffer_size;
    u->tx_buffer = tx_buffer_size;
    u->event_depth = queue_size;
    u->head = 0;
    u->tail = 0;
    u->tx_len = 0;
    u->events.head = 0;
    u->events.tail = 0;
    if (uart_queue != 0) {
        u->queue_ptr = &u->events;
        *uart_queue = &u->events;
    }
    return ESP_OK;
}

esp_err_t uart_driver_delete(uart_port_t uart_num)
{
    if (uart_num < 0 || (unsigned)uart_num >= MODEL_PORTS) {
        return ESP_FAIL;
    }
    uarts[uart_num].installed = 0;
    uarts[uart_num].queue_ptr = 0;
    return ESP_OK;
}

void ak_host_uart_set_param_refused(int refused)
{
    param_refused = refused;
}

esp_err_t uart_param_config(uart_port_t uart_num,
                            const uart_config_t *uart_config)
{
    if (uart_config == 0 || uart_num < 0 || (unsigned)uart_num >= MODEL_PORTS) {
        return ESP_FAIL;
    }
    if (param_refused) {
        return ESP_FAIL; /* a pin the matrix will not route */
    }
    struct host_uart *u = &uarts[uart_num];
    u->baud = uart_config->baud_rate;
    u->data_bits = uart_config->data_bits;
    u->parity = uart_config->parity;
    u->stop_bits = uart_config->stop_bits;
    return ESP_OK;
}

esp_err_t uart_set_pin(uart_port_t uart_num, int tx_io_num, int rx_io_num,
                       int rts_io_num, int cts_io_num)
{
    (void)rts_io_num;
    (void)cts_io_num;
    if (uart_num < 0 || (unsigned)uart_num >= MODEL_PORTS) {
        return ESP_FAIL;
    }
    uarts[uart_num].tx_pin = tx_io_num;
    uarts[uart_num].rx_pin = rx_io_num;
    return ESP_OK;
}

esp_err_t uart_set_baudrate(uart_port_t uart_num, uint32_t baudrate)
{
    if (uart_num < 0 || (unsigned)uart_num >= MODEL_PORTS) {
        return ESP_FAIL;
    }
    uarts[uart_num].baud = (int)baudrate;
    return ESP_OK;
}

esp_err_t uart_set_parity(uart_port_t uart_num, uart_parity_t parity_mode)
{
    if (uart_num < 0 || (unsigned)uart_num >= MODEL_PORTS) {
        return ESP_FAIL;
    }
    uarts[uart_num].parity = parity_mode;
    return ESP_OK;
}

esp_err_t uart_set_stop_bits(uart_port_t uart_num, uart_stop_bits_t stop_bits)
{
    if (uart_num < 0 || (unsigned)uart_num >= MODEL_PORTS) {
        return ESP_FAIL;
    }
    uarts[uart_num].stop_bits = stop_bits;
    return ESP_OK;
}

esp_err_t uart_set_line_inverse(uart_port_t uart_num, uint32_t inverse_mask)
{
    if (uart_num < 0 || (unsigned)uart_num >= MODEL_PORTS) {
        return ESP_FAIL;
    }
    uarts[uart_num].inverted = (int)inverse_mask;
    return ESP_OK;
}

esp_err_t uart_flush_input(uart_port_t uart_num)
{
    if (uart_num < 0 || (unsigned)uart_num >= MODEL_PORTS) {
        return ESP_FAIL;
    }
    struct host_uart *u = &uarts[uart_num];
    u->head = 0;
    u->tail = 0;
    u->flushes++;
    return ESP_OK;
}

int uart_read_bytes(uart_port_t uart_num, void *buf, uint32_t length,
                    TickType_t ticks_to_wait)
{
    (void)ticks_to_wait;
    if (buf == 0 || uart_num < 0 || (unsigned)uart_num >= MODEL_PORTS) {
        return -1;
    }
    struct host_uart *u = &uarts[uart_num];
    if (!u->installed) {
        return -1;
    }
    uint8_t *out = buf;
    unsigned got = 0;
    while (got < length && u->head != u->tail) {
        out[got++] = u->ring[u->head];
        u->head = (u->head + 1u) % MODEL_RING;
    }
    if (got > 0u) {
        u->reads++;
    }
    return (int)got;
}

int uart_write_bytes(uart_port_t uart_num, const void *src, size_t size)
{
    if (src == 0 || uart_num < 0 || (unsigned)uart_num >= MODEL_PORTS) {
        return -1;
    }
    struct host_uart *u = &uarts[uart_num];
    if (!u->installed) {
        return -1;
    }
    if (size > MODEL_TX) {
        return -1;
    }
    if (write_short > 0 && (size_t)write_short < size) {
        size -= (size_t)write_short; /* a driver that took part of the frame */
    }
    memcpy(u->tx, src, size);
    u->tx_len = (unsigned)size;
    return (int)size;
}

esp_err_t uart_wait_tx_done(uart_port_t uart_num, TickType_t ticks_to_wait)
{
    (void)ticks_to_wait;
    if (uart_num < 0 || (unsigned)uart_num >= MODEL_PORTS) {
        return ESP_FAIL;
    }
    return tx_stuck ? ESP_FAIL : ESP_OK;
}

/* --- what the test can see ------------------------------------------------ */

static struct host_uart *at(unsigned port)
{
    return port < MODEL_PORTS ? &uarts[port] : 0;
}

int ak_host_uart_installed(unsigned port)
{
    struct host_uart *u = at(port);
    return u != 0 ? u->installed : 0;
}

int ak_host_uart_rx_buffer(unsigned port)
{
    struct host_uart *u = at(port);
    return u != 0 ? u->rx_buffer : 0;
}

int ak_host_uart_tx_buffer(unsigned port)
{
    struct host_uart *u = at(port);
    return u != 0 ? u->tx_buffer : 0;
}

int ak_host_uart_event_depth(unsigned port)
{
    struct host_uart *u = at(port);
    return u != 0 ? u->event_depth : 0;
}

int ak_host_uart_baud(unsigned port)
{
    struct host_uart *u = at(port);
    return u != 0 ? u->baud : 0;
}

int ak_host_uart_parity(unsigned port)
{
    struct host_uart *u = at(port);
    return u != 0 ? u->parity : -1;
}

int ak_host_uart_stop_bits(unsigned port)
{
    struct host_uart *u = at(port);
    return u != 0 ? u->stop_bits : -1;
}

int ak_host_uart_data_bits(unsigned port)
{
    struct host_uart *u = at(port);
    return u != 0 ? u->data_bits : -1;
}

int ak_host_uart_inverted(unsigned port)
{
    struct host_uart *u = at(port);
    return u != 0 ? u->inverted : -1;
}

int ak_host_uart_tx_pin(unsigned port)
{
    struct host_uart *u = at(port);
    return u != 0 ? u->tx_pin : -1;
}

int ak_host_uart_rx_pin(unsigned port)
{
    struct host_uart *u = at(port);
    return u != 0 ? u->rx_pin : -1;
}

unsigned ak_host_uart_flushes(unsigned port)
{
    struct host_uart *u = at(port);
    return u != 0 ? u->flushes : 0u;
}

unsigned ak_host_uart_reads(unsigned port)
{
    struct host_uart *u = at(port);
    return u != 0 ? u->reads : 0u;
}

const uint8_t *ak_host_uart_tx(unsigned port)
{
    struct host_uart *u = at(port);
    return u != 0 ? u->tx : 0;
}

unsigned ak_host_uart_tx_bytes(unsigned port)
{
    struct host_uart *u = at(port);
    return u != 0 ? u->tx_len : 0u;
}

void ak_host_uart_feed(unsigned port, const uint8_t *bytes, unsigned len)
{
    struct host_uart *u = at(port);
    if (u == 0) {
        return;
    }
    for (unsigned i = 0; i < len; i++) {
        unsigned next = (u->tail + 1u) % MODEL_RING;
        if (next == u->head) {
            return; /* the ring is full: the driver would post an overflow */
        }
        u->ring[u->tail] = bytes[i];
        u->tail = next;
    }
}

unsigned ak_host_uart_pending(unsigned port)
{
    struct host_uart *u = at(port);
    if (u == 0) {
        return 0u;
    }
    return (u->tail + MODEL_RING - u->head) % MODEL_RING;
}

void ak_host_uart_push_event(unsigned port, int type)
{
    struct host_uart *u = at(port);
    if (u == 0) {
        return;
    }
    unsigned next = (u->events.tail + 1u) % MODEL_EVENTS;
    if (next == u->events.head) {
        return; /* a full event queue, which is what the port exists to avoid */
    }
    u->events.items[u->events.tail].type = (uart_event_type_t)type;
    u->events.items[u->events.tail].size = 0;
    u->events.items[u->events.tail].timeout_flag = 0;
    u->events.tail = next;
}

unsigned ak_host_uart_events(unsigned port)
{
    struct host_uart *u = at(port);
    if (u == 0) {
        return 0u;
    }
    return (u->events.tail + MODEL_EVENTS - u->events.head) % MODEL_EVENTS;
}

void ak_host_uart_set_install_refused(int refused)
{
    install_refused = refused != 0;
}

void ak_host_uart_set_write_short(int bytes_short)
{
    write_short = bytes_short;
}

void ak_host_uart_set_tx_stuck(int stuck)
{
    tx_stuck = stuck != 0;
}
