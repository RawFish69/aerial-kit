#ifndef AK_HOST_IDF_RMT_TYPES_H
#define AK_HOST_IDF_RMT_TYPES_H

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

#include "esp_err.h" /* esp_err_t, ESP_OK and ESP_FAIL, in one place */

/*
 * Just enough of ESP-IDF's RMT types to compile the ESP32's output layer on
 * the host, and no more.
 *
 * The fields are IDF's, copied from `hal/rmt_types.h` in the pinned ESP-IDF
 * checkout, because the point of this stub is that the *real* code builds
 * against something with the real layout: `output.c` packs durations into
 * these bitfields and the check unpacks them, so a stub with the fields in a
 * different order would check a different machine. Where this file and IDF
 * disagree, the ESP32 build - which uses IDF's own headers - is the one that
 * fails, and that is the check on the stub.
 */

typedef int gpio_num_t;
typedef int rmt_clock_source_t;

/* On a target this places a function in IRAM so an interrupt can call it.
 * Here it is what it is on the host: nothing. */
#define IRAM_ATTR
/* IDF's own values: the port tests a status against ESP_ERR_INVALID_STATE to
 * mean "someone else already brought this bus up", which is not a failure. */
#define ESP_ERR_INVALID_STATE 0x103

/* One RMT symbol: two level/duration pairs, 15 bits of duration each. */
typedef union {
    struct {
        uint16_t duration0 : 15;
        uint16_t level0 : 1;
        uint16_t duration1 : 15;
        uint16_t level1 : 1;
    };
    uint32_t val;
} rmt_symbol_word_t;

/* Opaque to the caller, which is how IDF treats them too: the port passes
 * these back and never looks inside. */
typedef struct host_rmt_channel *rmt_channel_handle_t;
typedef struct host_rmt_encoder *rmt_encoder_handle_t;

#define RMT_CLK_SRC_DEFAULT 0

#endif /* AK_HOST_IDF_RMT_TYPES_H */
