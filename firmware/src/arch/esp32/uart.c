#include "esp.h"

#include "driver/uart.h"

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

/*
 * The receiver's and the GPS's UARTs.
 *
 * The F405 talks to these ports through its own registers: poll a status bit,
 * read a data register, count what the interrupt had to drop. This chip has a
 * driver instead, and the shape of the port is different in exactly one way
 * that matters - the ring buffer lives *inside* the driver, and IDF posts an
 * event when it cannot keep what arrived. So "dropped" here is a number the
 * driver reports rather than one this file keeps, and a framing error is
 * visible at all, which it is not on the F405 (whose receive interrupt looks
 * only at "receive not empty").
 *
 * Three things are deliberate.
 *
 * - **The read path never waits.** ak_esp_rc_poll() asks for one byte with a
 *   zero tick timeout, because the caller is the flight loop, and a UART with
 *   nothing on it must not cost a millisecond a pass. It is the same rule as
 *   rmt_transmit()'s queue_nonblocking flag, learned the same way: a vendor
 *   call inside a superloop that *can* wait is a vendor call that can end the
 *   loop.
 * - **The events are drained on that same path**, not by a task of their own.
 *   An event queue that nobody empties fills up, and a full event queue is
 *   exactly how the overflow it was installed to report goes missing.
 * - **A framing error is said once and then only counted.** It is the symptom
 *   of the one mistake this port cannot detect for the pilot - the wrong
 *   protocol, or a receiver on the wrong side of an inverter - and repeating
 *   it a thousand times a second makes the console useless in the situation
 *   where somebody is reading it.
 *
 * Nothing here has been on a board: no receiver and no GPS have ever been
 * wired to this port, and QEMU's ESP32 has no serial device on those pins to
 * wire one to. What the emulator can say is that the port comes up and that
 * the firmware runs with it in the loop, and that is what it is used for.
 */

/* The ring the driver keeps on the receive side. CRSF is 420000 baud, about
 * 42 kB/s, and the flight loop drains it every pass; a kilobyte is a fifth of
 * a second of slack, which is enough to cover a log erase and small enough
 * that an overflow is noticed rather than hidden. */
#define AK_ESP_UART_RX_BYTES 1024
#define AK_ESP_UART_TX_BYTES 512
#define AK_ESP_UART_EVENTS   16

/* A write to the GPS is a configuration frame, tens of bytes, and it is sent
 * from the loop that also flies the aircraft - so it gets a bound, not a
 * wait. Twenty milliseconds is a UBX frame at 9600 baud with room to spare. */
#define AK_ESP_UART_TX_WAIT_TICKS pdMS_TO_TICKS(20)

typedef struct {
    int           num;          /* the UART number, -1 before init */
    int           installed;
    int           inverted;
    QueueHandle_t events;
    uint32_t      dropped;      /* bytes the driver's ring could not keep */
    uint32_t      framing;      /* framing or parity errors seen */
    int           said_framing; /* the one line about them has been printed */
    int           baud;         /* the rate the board named for this port */
    int           sbus_baud;    /* the receiver's second rate, for SBUS */
} ak_esp_uart_t;

static ak_esp_uart_t rc_uart  = { .num = -1 };
static ak_esp_uart_t gps_uart = { .num = -1 };

static int uart_setup(ak_esp_uart_t *u, int num, int tx_gpio, int rx_gpio,
                      int baud)
{
    uart_config_t cfg = {
        .baud_rate = baud,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    if (u->installed) {
        (void)uart_driver_delete((uart_port_t)u->num);
        u->installed = 0;
    }
    u->events = 0;
    u->inverted = 0;

    /* The event queue is part of the install, and it is what makes an overflow
     * visible instead of silent. UART_DATA events land in it too - IDF posts
     * one every time it moves data from the FIFO into the ring, which is its
     * default threshold of 120 bytes - so the queue is made deeper than the
     * events a busy receiver can produce between two passes of the loop. A
     * queue full of "some bytes arrived" is a queue with no room left for "the
     * buffer overflowed", and that one is the one worth keeping. */
    if (uart_driver_install((uart_port_t)num, AK_ESP_UART_RX_BYTES,
                            AK_ESP_UART_TX_BYTES, AK_ESP_UART_EVENTS,
                            &u->events, 0) != ESP_OK) {
        return -1;
    }
    if (uart_param_config((uart_port_t)num, &cfg) != ESP_OK ||
        uart_set_pin((uart_port_t)num, tx_gpio, rx_gpio, UART_PIN_NO_CHANGE,
                     UART_PIN_NO_CHANGE) != ESP_OK) {
        (void)uart_driver_delete((uart_port_t)num);
        u->events = 0;
        return -1;
    }
    u->num = num;
    u->installed = 1;
    u->baud = baud;
    u->dropped = 0;
    u->framing = 0;
    u->said_framing = 0;
    return 0;
}

/* Drain what the driver has to say, and turn it into the two numbers the
 * console can act on. Called from the poll path; never waits. */
static void uart_drain(ak_esp_uart_t *u)
{
    uart_event_t event;

    if (!u->installed || u->events == 0) {
        return;
    }
    while (xQueueReceive(u->events, &event, 0) == pdTRUE) {
        switch (event.type) {
        case UART_BUFFER_FULL:
        case UART_FIFO_OVF:
            /* Both mean bytes were lost. The driver asks to be flushed after
             * either one, or it keeps reporting the same condition. */
            u->dropped++;
            (void)uart_flush_input((uart_port_t)u->num);
            break;
        case UART_FRAME_ERR:
        case UART_PARITY_ERR:
            u->framing++;
            break;
        default:
            break; /* UART_DATA, break, pattern, and everything else */
        }
    }
}

static void uart_say_framing(ak_esp_uart_t *u, const char *what)
{
    if (u->framing == 0u || u->said_framing) {
        return;
    }
    u->said_framing = 1;
    /* The console is UART0 and this is one line, once. */
    ak_console_printf("uart%d:     %s is arriving with framing errors - the "
                      "line settings do not match it (protocol, or the "
                      "inverter)\r\n",
                      u->num, what);
}

void ak_esp_rc_init(int num, int tx_gpio, int rx_gpio, int baud,
                    int sbus_baud)
{
    if (uart_setup(&rc_uart, num, tx_gpio, rx_gpio, baud) != 0) {
        /* Louder than a counter, because "no receiver" and "the port never
         * came up" look identical from the console otherwise. */
        ak_console_printf("rc:        uart%d did not come up - no receiver\r\n",
                          num);
        return;
    }
    rc_uart.sbus_baud = sbus_baud;
}

void ak_esp_rc_set_protocol(uint32_t protocol)
{
    uart_port_t port;

    if (!rc_uart.installed) {
        return;
    }
    port = (uart_port_t)rc_uart.num;

    if (protocol == 1u) {
        /* SBUS: 100000 baud, even parity, two stop bits, and inverted. This
         * chip can invert the receive line in its GPIO matrix, which the F405
         * cannot - so the transistor the F405's board file asks for is not
         * needed here, and ak_esp_rc_inverted() says so. */
        (void)uart_set_baudrate(port, (uint32_t)rc_uart.sbus_baud);
        (void)uart_set_parity(port, UART_PARITY_EVEN);
        (void)uart_set_stop_bits(port, UART_STOP_BITS_2);
        (void)uart_set_line_inverse(port, UART_SIGNAL_RXD_INV);
        rc_uart.inverted = 1;
    } else {
        /* CRSF: 420000 baud, 8N1, not inverted. */
        /* Back to the rate the board named for CRSF. It is the port's own
         * default, kept here rather than remembered: a receiver that was
         * switched from SBUS to CRSF has to get the CRSF rate back. */
        (void)uart_set_baudrate(port, (uint32_t)rc_uart.baud);
        (void)uart_set_parity(port, UART_PARITY_DISABLE);
        (void)uart_set_stop_bits(port, UART_STOP_BITS_1);
        (void)uart_set_line_inverse(port, UART_SIGNAL_INV_DISABLE);
        rc_uart.inverted = 0;
    }

    /* Whatever was in flight was at the old line settings: half a frame read
     * at the wrong baud is worse than half a frame thrown away. */
    (void)uart_flush_input(port);
    rc_uart.said_framing = 0;
}

int ak_esp_rc_poll(uint8_t *byte)
{
    if (!rc_uart.installed) {
        return 0;
    }
    uart_drain(&rc_uart);
    if (uart_read_bytes((uart_port_t)rc_uart.num, byte, 1, 0) == 1) {
        return 1;
    }
    uart_say_framing(&rc_uart, "the receiver");
    return 0;
}

uint32_t ak_esp_rc_dropped(void)
{
    uart_drain(&rc_uart);
    return rc_uart.dropped;
}

int ak_esp_rc_inverted(void)
{
    /* Asked by the core when the protocol is SBUS: "does this port invert the
     * signal it is given?" On this chip the answer is the port's own, not a
     * transistor's - 1 means the GPIO matrix is doing it. A port that never
     * came up answers 0, because nothing is being inverted by anything. */
    return rc_uart.installed ? 1 : 0;
}

/* Telemetry out. Bounded like the GPS's writes: the caller is the flight loop,
 * and a receiver that is not reading must cost a failed write rather than a
 * loop that stops. */
int ak_esp_rc_send(const char *data, unsigned len)
{
    int written;

    if (!rc_uart.installed) {
        return -1;
    }
    written = uart_write_bytes((uart_port_t)rc_uart.num, data, len);
    if (written != (int)len) {
        return -1;
    }
    return uart_wait_tx_done((uart_port_t)rc_uart.num,
                             AK_ESP_UART_TX_WAIT_TICKS) == ESP_OK ? 0 : -1;
}

void ak_esp_gps_init(int num, int tx_gpio, int rx_gpio, int baud)
{
    if (uart_setup(&gps_uart, num, tx_gpio, rx_gpio, baud) != 0) {
        ak_console_printf("gps:       uart%d did not come up - no gps\r\n",
                          num);
    }
}

int ak_esp_gps_poll(uint8_t *byte)
{
    if (!gps_uart.installed) {
        return 0;
    }
    uart_drain(&gps_uart);
    return uart_read_bytes((uart_port_t)gps_uart.num, byte, 1, 0) == 1 ? 1 : 0;
}

uint32_t ak_esp_gps_dropped(void)
{
    uart_drain(&gps_uart);
    return gps_uart.dropped;
}

int ak_esp_gps_send(const char *data, unsigned len)
{
    int written;

    if (!gps_uart.installed) {
        return -1;
    }
    written = uart_write_bytes((uart_port_t)gps_uart.num, data, len);
    if (written != (int)len) {
        return -1;
    }
    /* Queued, not sent: wait - with a bound - so that a caller which sends a
     * second configuration frame does not reorder them. */
    return uart_wait_tx_done((uart_port_t)gps_uart.num,
                             AK_ESP_UART_TX_WAIT_TICKS) == ESP_OK ? 0 : -1;
}
