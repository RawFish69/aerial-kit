#ifndef AK_HOST_IDF_ESP_NETIF_H
#define AK_HOST_IDF_ESP_NETIF_H

/*
 * IDF's netif layer: the object the port keeps a pointer to, the address it
 * prints, and the two "default Wi-Fi netif" helpers.
 *
 * The address structure is IDF's own shape
 * (components/esp_netif/include/esp_netif_ip_addr.h), including that the port
 * reads it a byte at a time out of a little-endian word - which is what makes
 * the `a.b.c.d` in a console line a thing worth checking: a model that handed
 * back the address in another order would print a different one.
 */

#include "esp_err.h"

#include <stdint.h>

typedef struct esp_netif_obj esp_netif_t;

typedef struct {
    uint32_t addr;
} esp_ip4_addr_t;

typedef struct {
    esp_ip4_addr_t ip;
    esp_ip4_addr_t netmask;
    esp_ip4_addr_t gw;
} esp_netif_ip_info_t;

/* Ethernet builds call this and Wi-Fi builds do not; it is here so the type a
 * netif is built from is named in one place. See tests/idf-stub/esp_eth.h. */
typedef struct {
    const void *base;
    int         if_key;
} esp_netif_config_t;

esp_err_t esp_netif_init(void);

/* What the port uses the object for. The model's netif is not IDF's: it is a
 * handle the test can point at an address, so the console line can be checked
 * with an address and without one. */
esp_netif_t *esp_netif_create_default_wifi_sta(void);
esp_netif_t *esp_netif_create_default_wifi_ap(void);
esp_netif_t *esp_netif_new(const esp_netif_config_t *config);
esp_err_t esp_netif_attach(esp_netif_t *netif, void *io_driver);
esp_err_t esp_netif_get_ip_info(esp_netif_t *netif, esp_netif_ip_info_t *info);

#endif /* AK_HOST_IDF_ESP_NETIF_H */
