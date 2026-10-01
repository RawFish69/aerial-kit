#ifndef AK_HOST_IDF_FREERTOS_QUEUE_H
#define AK_HOST_IDF_FREERTOS_QUEUE_H

/*
 * The one queue call the ESP32's UART port makes: `xQueueReceive()` with a
 * zero tick timeout, draining the driver's events on the poll path.
 *
 * On a target this is FreeRTOS's queue. On the host it is the model's event
 * ring (tests/host_esp32_uart_model.c), which the test fills directly - the
 * same "reach in where the hardware would" that the register-block tests do
 * with a peripheral's flags.
 */

#include <stdint.h>

typedef struct host_queue *QueueHandle_t;

#define pdTRUE 1
#define pdFALSE 0

int xQueueReceive(QueueHandle_t queue, void *item, uint32_t ticks_to_wait);

#endif /* AK_HOST_IDF_FREERTOS_QUEUE_H */
