#ifndef AK_HOST_IDF_FREERTOS_H
#define AK_HOST_IDF_FREERTOS_H

/*
 * Just the FreeRTOS pieces the ESP32 port uses on the host.
 *
 * The bus files take `pdMS_TO_TICKS()`, as the bounded wait a transfer is
 * collected with. On the host there is no scheduler and no tick, so a tick is a
 * millisecond and the timeout is a number the model can report. It is
 * deliberately very nearly the *only* thing here: a port file that reached for
 * vTaskDelay or a queue would fail to compile, which is the right way for the
 * host build to say "this is not portable code".
 *
 * The one exception is the network, which is not a bus file and is not
 * portable by construction: it is the Wi-Fi build's task and the two stream
 * buffers that join it to the flight loop. Those are declared for the same
 * reason the rest of this directory exists - so that the *real* net.c runs on
 * this machine in the configuration a board is in - and the model behind them
 * is a byte stream rather than a stub, because "what the client sent reached
 * the loop" is one of the things the checks are about.
 */

#include <stddef.h>

#define configTICK_RATE_HZ 1000
#define pdMS_TO_TICKS(ms)  ((unsigned)(ms))

/* The type IDF's blocking calls take. On the host a tick is a millisecond. */
typedef unsigned TickType_t;
typedef int      BaseType_t;

#define pdTRUE  1
#define pdFALSE 0

typedef void *StreamBufferHandle_t;
typedef void *TaskHandle_t;

StreamBufferHandle_t xStreamBufferCreate(size_t buffer_size, size_t trigger_level);
size_t xStreamBufferSend(StreamBufferHandle_t buffer, const void *data,
                         size_t length, TickType_t wait);
size_t xStreamBufferReceive(StreamBufferHandle_t buffer, void *data,
                            size_t length, TickType_t wait);
BaseType_t xStreamBufferReset(StreamBufferHandle_t buffer);

#endif /* AK_HOST_IDF_FREERTOS_H */
