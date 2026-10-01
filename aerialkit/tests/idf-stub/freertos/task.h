#ifndef AK_HOST_IDF_FREERTOS_TASK_H
#define AK_HOST_IDF_FREERTOS_TASK_H

/* The task API the net port uses: create the socket task, and delete it when it
 * cannot start. The model keeps the function the port created instead of
 * starting it, so a check can run the task's own body and stop it at a point
 * of the test's choosing - which is the only way to run a `for (;;)` on a host
 * and look at what it did. */

#include "FreeRTOS.h"

typedef void (*TaskFunction_t)(void *argument);

BaseType_t xTaskCreate(TaskFunction_t task, const char *name,
                       unsigned stack_depth, void *argument,
                       unsigned priority, TaskHandle_t *created);
void vTaskDelete(TaskHandle_t task);

#endif /* AK_HOST_IDF_FREERTOS_TASK_H */
