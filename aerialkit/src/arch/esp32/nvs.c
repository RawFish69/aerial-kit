#include "esp.h"

#include "nvs.h"
#include "nvs_flash.h"

/*
 * Saved configuration, in NVS.
 *
 * The STM32 writes a sector of flash and has to erase it first; the ESP32 has
 * a key-value store with wear levelling underneath the whole flash, so the same
 * contract is a blob read and a blob write. Nothing above this line can tell
 * the difference, which is the point of having put the contract there.
 */

#define AK_ESP_NVS_NAMESPACE "aerialkit"
#define AK_ESP_NVS_KEY "config"

static int ensure_init(void)
{
    static int ready;
    if (!ready) {
        esp_err_t status = nvs_flash_init();
        if (status == ESP_ERR_NVS_NO_FREE_PAGES ||
            status == ESP_ERR_NVS_NEW_VERSION_FOUND) {
            (void)nvs_flash_erase();
            status = nvs_flash_init();
        }
        ready = status == ESP_OK;
    }
    return ready;
}

int ak_esp_config_read(void *buf, uint32_t len)
{
    if (!ensure_init()) {
        return -1;
    }

    nvs_handle_t handle;
    if (nvs_open(AK_ESP_NVS_NAMESPACE, NVS_READONLY, &handle) != ESP_OK) {
        return 0; /* nothing stored yet, which is not an error */
    }

    size_t length = len;
    esp_err_t status = nvs_get_blob(handle, AK_ESP_NVS_KEY, buf, &length);
    nvs_close(handle);

    if (status == ESP_ERR_NVS_NOT_FOUND) {
        return 0;
    }
    if (status == ESP_ERR_NVS_INVALID_LENGTH) {
        /* "The record does not fit the buffer" is a caller problem, not a
         * damaged record - and NVS reports it as an error, which the core
         * would otherwise read as a checksum failure. */
        return -1;
    }
    if (status != ESP_OK) {
        return -1;
    }
    return (int)length;
}

int ak_esp_config_write(const void *buf, uint32_t len)
{
    /* The same bound the F405's record has, for the same reason: a stored
     * record has to leave room for the terminator a reader puts after it. NVS
     * would take more, and then nothing could read it back with the buffer the
     * console and the preflight keep. */
    if (len == 0u || len >= AK_PARAMS_TEXT_MAX) {
        return -1;
    }
    if (!ensure_init()) {
        return -1;
    }

    nvs_handle_t handle;
    if (nvs_open(AK_ESP_NVS_NAMESPACE, NVS_READWRITE, &handle) != ESP_OK) {
        return -1;
    }
    esp_err_t status = nvs_set_blob(handle, AK_ESP_NVS_KEY, buf, len);
    if (status == ESP_OK) {
        status = nvs_commit(handle);
    }
    nvs_close(handle);
    return status == ESP_OK ? 0 : -1;
}
