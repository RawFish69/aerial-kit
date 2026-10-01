#ifndef AK_HOST_IDF_ESP_SYSTEM_H
#define AK_HOST_IDF_ESP_SYSTEM_H

/*
 * The two things `src/arch/esp32/` asks of IDF's system layer: why the chip
 * restarted, and how to restart it.
 *
 * The enum is IDF's own (`components/esp_system/include/esp_system.h`), values
 * and order, because `src/arch/esp32/fault.c` stores the reason it is handed
 * in the record's fault-status field - so a model with the values in a
 * different order would check a different machine.
 */

typedef enum {
    ESP_RST_UNKNOWN = 0,   /* the reason could not be determined */
    ESP_RST_POWERON = 1,   /* power-on */
    ESP_RST_EXT = 2,       /* an external pin (not applicable to the ESP32) */
    ESP_RST_SW = 3,        /* software reset, esp_restart() */
    ESP_RST_PANIC = 4,     /* software reset after an exception or panic */
    ESP_RST_INT_WDT = 5,   /* the interrupt watchdog */
    ESP_RST_TASK_WDT = 6,  /* the task watchdog */
    ESP_RST_WDT = 7,       /* another watchdog */
    ESP_RST_DEEPSLEEP = 8,
    ESP_RST_BROWNOUT = 9,
    ESP_RST_SDIO = 10,
} esp_reset_reason_t;

void esp_restart(void);
esp_reset_reason_t esp_reset_reason(void);

#endif /* AK_HOST_IDF_ESP_SYSTEM_H */
