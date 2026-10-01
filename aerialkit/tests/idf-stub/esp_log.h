#ifndef AK_HOST_IDF_ESP_LOG_H
#define AK_HOST_IDF_ESP_LOG_H

/*
 * IDF's log macro, pointed at the model's counter instead of a console.
 *
 * The port logs a line every time a driver call fails, and on the host a test
 * wants to know *that* it happened without a wall of text in the middle of the
 * check's own output - so the model counts them and the check reads the count.
 * The message text is kept for the cases where the check wants to print it.
 */

void ak_host_idf_log(const char *tag, const char *fmt, ...)
    __attribute__((format(printf, 2, 3)));

unsigned ak_host_idf_log_count(void);
const char *ak_host_idf_log_text(unsigned index);

#define ESP_LOGW(tag, fmt, ...) ak_host_idf_log(tag, fmt, ##__VA_ARGS__)
#define ESP_LOGE(tag, fmt, ...) ak_host_idf_log(tag, fmt, ##__VA_ARGS__)
#define ESP_LOGI(tag, fmt, ...) ak_host_idf_log(tag, fmt, ##__VA_ARGS__)

#endif /* AK_HOST_IDF_ESP_LOG_H */
