#include "esp.h"

#include "driver/uart.h"
#include "esp_log.h"

/* UART0, which IDF has already configured for its own console. We take it over
 * for output and read it for the CLI: one port, two users, exactly as on the
 * STM32 board. */
#define AK_ESP_CONSOLE_PORT UART_NUM_0

static uint32_t console_port;
static int      console_attached;

uint32_t AK_ESP_CONSOLE_UART(void)
{
    return AK_ESP_CONSOLE_PORT;
}

void ak_esp_console_init(uint32_t baud)
{
    /* The driver has to be installed before uart_write_bytes or uart_read_bytes
     * will do anything, and IDF does not install it for us on the default
     * console configuration - which is how the first version of this port
     * booted, ran, and printed nothing at all. ESP_ERR_INVALID_STATE means
     * somebody else got there first, which is fine. */
    esp_err_t status = uart_driver_install(AK_ESP_CONSOLE_PORT, 256, 0, 0, NULL, 0);
    if (status != ESP_OK && status != ESP_ERR_INVALID_STATE) {
        return;
    }
    uart_set_baudrate(AK_ESP_CONSOLE_PORT, baud);
}

void ak_console_attach(uint32_t port)
{
    console_port = port;
    console_attached = 1;
}

uint32_t ak_console_port(void)
{
    /* Not zero for "nothing attached": UART_NUM_0 *is* zero on this chip, and
     * using the same value for both is how the first version of this port
     * booted, ran the whole firmware, and printed nothing at all. */
    return console_attached ? console_port : 0xFFFFFFFFu;
}

void ak_console_write_raw(const char *data, unsigned len)
{
    if (!console_attached) {
        return;
    }
    /* Straight to the driver: the console formatter assembles a whole line
     * before calling this, so one write per line is also the cheapest. */
    (void)uart_write_bytes(AK_ESP_CONSOLE_PORT, data, len);
}

int ak_esp_console_poll_rx(uint8_t *byte)
{
    return uart_read_bytes(AK_ESP_CONSOLE_PORT, byte, 1, 0) == 1 ? 1 : 0;
}
