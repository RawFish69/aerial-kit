#ifndef AK_HOST_IDF_ESP_WIFI_H
#define AK_HOST_IDF_ESP_WIFI_H

/*
 * IDF's Wi-Fi driver, as much of it as `src/arch/esp32/net.c` touches - which
 * is the half of that file no machine here can run: QEMU's ESP32 has no radio,
 * so the end-to-end check runs the Ethernet medium and every line below the
 * `CONFIG_AK_NET_WIFI` in the port is reached only on a board.
 *
 * The structures are subsets of IDF v5.5's
 * (components/esp_wifi/include/esp_wifi_types_generic.h): the fields the port
 * *writes*, with IDF's own types, order and array sizes, because the sizes are
 * load-bearing - the port copies the SSID with `strncpy(..., sizeof ssid - 1)`
 * and a model with a smaller array would be checking different code. Every
 * other field is left out rather than guessed at, and the ESP32 build is what
 * checks the port against the whole thing.
 *
 * WIFI_INIT_CONFIG_DEFAULT() is IDF's macro however the fields are spelled;
 * the port passes the value straight to esp_wifi_init() and reads none of it,
 * so what the model records is the call, not the configuration.
 */

#include "esp_err.h"

#include <stdint.h>

typedef enum {
    WIFI_MODE_NULL = 0,
    WIFI_MODE_STA,
    WIFI_MODE_AP,
    WIFI_MODE_APSTA,
} wifi_mode_t;

typedef enum {
    ESP_IF_WIFI_STA = 0,
    ESP_IF_WIFI_AP = 1,
} esp_interface_t;

typedef esp_interface_t wifi_interface_t;

#define WIFI_IF_STA ESP_IF_WIFI_STA
#define WIFI_IF_AP  ESP_IF_WIFI_AP

typedef enum {
    WIFI_STORAGE_FLASH = 0,
    WIFI_STORAGE_RAM,
} wifi_storage_t;

/* The auth modes, in IDF's order, which is a strength order: the port's
 * "WPA2 or better" is a threshold against it. */
typedef enum {
    WIFI_AUTH_OPEN = 0,
    WIFI_AUTH_WEP,
    WIFI_AUTH_WPA_PSK,
    WIFI_AUTH_WPA2_PSK,
    WIFI_AUTH_WPA_WPA2_PSK,
    WIFI_AUTH_ENTERPRISE,
    WIFI_AUTH_WPA2_ENTERPRISE = WIFI_AUTH_ENTERPRISE,
    WIFI_AUTH_WPA3_PSK,
    WIFI_AUTH_WPA2_WPA3_PSK,
    WIFI_AUTH_WAPI_PSK,
    WIFI_AUTH_OWE,
} wifi_auth_mode_t;

typedef struct {
    int8_t           rssi;
    wifi_auth_mode_t authmode;
} wifi_scan_threshold_t;

typedef struct {
    uint8_t               ssid[32];
    uint8_t               password[64];
    wifi_scan_threshold_t threshold;
} wifi_sta_config_t;

typedef struct {
    uint8_t          ssid[32];
    uint8_t          password[64];
    uint8_t          ssid_len;
    wifi_auth_mode_t authmode;
    uint8_t          ssid_hidden;
    uint8_t          max_connection;
} wifi_ap_config_t;

typedef union {
    wifi_ap_config_t  ap;
    wifi_sta_config_t sta;
} wifi_config_t;

typedef struct {
    int static_rx_buf_num;
    int static_tx_buf_num;
    int dynamic_rx_buf_num;
    int dynamic_tx_buf_num;
} wifi_init_config_t;

#define WIFI_INIT_CONFIG_DEFAULT() { 0, 0, 0, 0 }

esp_err_t esp_wifi_init(const wifi_init_config_t *config);
esp_err_t esp_wifi_set_storage(wifi_storage_t storage);
esp_err_t esp_wifi_set_mode(wifi_mode_t mode);
esp_err_t esp_wifi_set_config(wifi_interface_t interface, wifi_config_t *config);
esp_err_t esp_wifi_start(void);
esp_err_t esp_wifi_connect(void);

#endif /* AK_HOST_IDF_ESP_WIFI_H */
