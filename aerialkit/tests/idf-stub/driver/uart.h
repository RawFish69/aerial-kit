#ifndef AK_HOST_IDF_UART_H
#define AK_HOST_IDF_UART_H

#include <stddef.h>
#include <stdint.h>

#include "driver/rmt_types.h"  /* esp_err_t */
#include "freertos/FreeRTOS.h" /* TickType_t */
#include "freertos/queue.h"    /* QueueHandle_t */

/*
 * The UART surface `src/arch/esp32/uart.c` calls, declared the way IDF
 * declares it - including that the receive ring and the event queue belong to
 * the *driver*, not to the port. That is the whole difference from the F405:
 * there the interrupt counts what it dropped in a register this firmware owns;
 * here the driver posts an event and the port has to drain it on the poll
 * path, or the event queue fills and the overflow it was installed to report
 * goes missing.
 *
 * `uart_read_bytes()` takes a tick timeout and the port passes zero, because
 * the caller is the flight loop - the same rule the outputs learned on this
 * chip from `rmt_transmit()`: a vendor call that *can* wait must not be given
 * the chance.
 */

typedef int uart_port_t;
typedef int uart_word_length_t;
typedef int uart_parity_t;
typedef int uart_stop_bits_t;
typedef int uart_hw_flowcontrol_t;
typedef int uart_sclk_t;

typedef struct {
    int baud_rate;
    uart_word_length_t data_bits;
    uart_parity_t parity;
    uart_stop_bits_t stop_bits;
    uart_hw_flowcontrol_t flow_ctrl;
    uint8_t rx_flow_ctrl_thresh;
    uart_sclk_t source_clk;
    struct {
        uint32_t allow_pd : 1;
        uint32_t backup_before_sleep : 1;
    } flags;
} uart_config_t;

typedef enum {
    UART_DATA = 0,
    UART_BREAK,
    UART_BUFFER_FULL,
    UART_FIFO_OVF,
    UART_FRAME_ERR,
    UART_PARITY_ERR,
    UART_EVENT_MAX,
} uart_event_type_t;

typedef struct {
    uart_event_type_t type;
    size_t size;
    uint32_t timeout_flag;
} uart_event_t;

#define UART_NUM_0 0
#define UART_NUM_1 1
#define UART_NUM_2 2

#define UART_DATA_8_BITS 3
#define UART_PARITY_DISABLE 0
#define UART_PARITY_EVEN 2
#define UART_STOP_BITS_1 1
#define UART_STOP_BITS_2 3
#define UART_HW_FLOWCTRL_DISABLE 0
#define UART_SCLK_DEFAULT 0
#define UART_PIN_NO_CHANGE (-1)

#define UART_SIGNAL_RXD_INV (1u << 5)
#define UART_SIGNAL_INV_DISABLE 0u

esp_err_t uart_driver_install(uart_port_t uart_num, int rx_buffer_size,
                              int tx_buffer_size, int queue_size,
                              QueueHandle_t *uart_queue, int intr_alloc_flags);
esp_err_t uart_driver_delete(uart_port_t uart_num);
esp_err_t uart_param_config(uart_port_t uart_num,
                            const uart_config_t *uart_config);
esp_err_t uart_set_pin(uart_port_t uart_num, int tx_io_num, int rx_io_num,
                       int rts_io_num, int cts_io_num);
esp_err_t uart_set_baudrate(uart_port_t uart_num, uint32_t baudrate);
esp_err_t uart_set_parity(uart_port_t uart_num, uart_parity_t parity_mode);
esp_err_t uart_set_stop_bits(uart_port_t uart_num, uart_stop_bits_t stop_bits);
esp_err_t uart_set_line_inverse(uart_port_t uart_num, uint32_t inverse_mask);
esp_err_t uart_flush_input(uart_port_t uart_num);
int uart_read_bytes(uart_port_t uart_num, void *buf, uint32_t length,
                    TickType_t ticks_to_wait);
int uart_write_bytes(uart_port_t uart_num, const void *src, size_t size);
esp_err_t uart_wait_tx_done(uart_port_t uart_num, TickType_t ticks_to_wait);

#endif /* AK_HOST_IDF_UART_H */
