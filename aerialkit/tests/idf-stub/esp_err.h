#ifndef AK_HOST_IDF_ESP_ERR_H
#define AK_HOST_IDF_ESP_ERR_H

/*
 * IDF's error type, and the two calls this port makes with it.
 *
 * `ESP_ERROR_CHECK` is the one that is not a plain stand-in, and the reason is
 * in what it does on a chip: it prints and then restarts the part. A host test
 * that obeyed that would take the process down in the middle of a check, and
 * the thing a check wants to know is *that the call failed* - so the model
 * counts it and the port carries on, which is what makes the lines after a
 * failed `esp_netif_init()` reachable here. Nothing is claimed about the
 * board's behaviour after that: on a board there is no "after that".
 */

typedef int esp_err_t;

#define ESP_OK   0
#define ESP_FAIL (-1)

void ak_host_idf_error_check(esp_err_t err, int line);

#define ESP_ERROR_CHECK(x)                              \
    do {                                                \
        ak_host_idf_error_check((x), __LINE__);          \
    } while (0)

#endif /* AK_HOST_IDF_ESP_ERR_H */
