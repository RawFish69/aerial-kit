#ifndef AK_HOST_IDF_ESP_EVENT_H
#define AK_HOST_IDF_ESP_EVENT_H

/*
 * IDF's event loop, as much of it as `src/arch/esp32/net.c` names: two event
 * bases, five events, a handler registration and the default loop.
 *
 * The events an event handler is *called* with are handled by the model: it
 * keeps every registration and the test raises an event through it, which is
 * how the port's two handlers - the address report and the reconnect - are
 * reached here at all. On a chip the radio and the netif raise them.
 *
 * The bases are string pointers compared by address, which is IDF's own shape
 * (`ESP_EVENT_DEFINE_BASE`): the port compares the base it is handed with the
 * one it registered for.
 */

#include "esp_err.h"

#include <stdint.h>

typedef const char *esp_event_base_t;

extern const char *IP_EVENT;
extern const char *WIFI_EVENT;

/* IDF's own order (components/esp_netif/include/esp_netif_types.h). Nothing in
 * the port stores these values, so only their distinctness is load-bearing -
 * but a model that renumbers them is a model that would stop noticing if the
 * port ever started to. */
typedef enum {
    IP_EVENT_STA_GOT_IP = 0,
    IP_EVENT_STA_LOST_IP,
    IP_EVENT_AP_STAIPASSIGNED,
    IP_EVENT_GOT_IP6,
    IP_EVENT_ETH_GOT_IP,
    IP_EVENT_ETH_LOST_IP,
    IP_EVENT_PPP_GOT_IP,
    IP_EVENT_PPP_LOST_IP,
} ip_event_t;

/* And the Wi-Fi stations', from components/esp_wifi/include/esp_wifi_types.h:
 * the values in between are the access-point events, which this port names
 * none of. */
typedef enum {
    WIFI_EVENT_WIFI_READY = 0,
    WIFI_EVENT_SCAN_DONE,
    WIFI_EVENT_STA_START,
    WIFI_EVENT_STA_STOP,
    WIFI_EVENT_STA_CONNECTED,
    WIFI_EVENT_STA_DISCONNECTED,
} wifi_event_stand_in_t;

typedef void (*esp_event_handler_t)(void *argument, esp_event_base_t base,
                                    int32_t id, void *data);

esp_err_t esp_event_loop_create_default(void);
esp_err_t esp_event_handler_register(esp_event_base_t base, int32_t id,
                                     esp_event_handler_t handler, void *argument);

#endif /* AK_HOST_IDF_ESP_EVENT_H */
