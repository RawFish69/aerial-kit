#ifndef AK_HOST_IDF_RMT_TX_H
#define AK_HOST_IDF_RMT_TX_H

#include "driver/rmt_types.h"

/*
 * The RMT transmit API `src/arch/esp32/output.c` calls, declared the way IDF
 * declares it - including the one signature the port got wrong.
 *
 * `rmt_transmit`'s fourth argument is `payload_bytes`. The copy encoder treats
 * the payload as an array of `rmt_symbol_word_t`, so a caller handing it a
 * *symbol count* sends a quarter of the symbols it meant to: a DShot frame cut
 * off after four bits, which an ESC discards as a bad frame and which nothing
 * on this machine could see, because QEMU's ESP32 has no RMT to transmit with
 * and the port's own report says "no frame has completed" either way.
 *
 * Writing the parameter's name here is the whole reason this header is a copy
 * of IDF's declarations rather than something made up: the host model reads
 * what the caller passed, and the check compares it with what the symbols
 * need. See tests/host_esp32_output_model.h.
 */

typedef struct {
    gpio_num_t gpio_num;
    rmt_clock_source_t clk_src;
    uint32_t resolution_hz;
    size_t mem_block_symbols;
    size_t trans_queue_depth;
    int intr_priority;
    struct {
        uint32_t invert_out : 1;
        uint32_t with_dma : 1;
        uint32_t io_loop_back : 1;
        uint32_t io_od_mode : 1;
        uint32_t allow_pd : 1;
    } flags;
} rmt_tx_channel_config_t;

typedef struct {
    int loop_count;
    struct {
        uint32_t eot_level : 1;
        uint32_t queue_nonblocking : 1;
    } flags;
} rmt_transmit_config_t;

/* IDF's copy encoder takes an empty configuration: the symbols *are* the
 * payload, so there is nothing to configure. */
typedef struct {
} rmt_copy_encoder_config_t;

typedef struct {
    int dummy;
} rmt_tx_done_event_data_t;

typedef bool (*rmt_tx_done_callback_t)(rmt_channel_handle_t channel,
                                       const rmt_tx_done_event_data_t *event,
                                       void *user_data);

typedef struct {
    rmt_tx_done_callback_t on_trans_done;
} rmt_tx_event_callbacks_t;

esp_err_t rmt_new_tx_channel(const rmt_tx_channel_config_t *config,
                             rmt_channel_handle_t *ret_chan);
esp_err_t rmt_new_copy_encoder(const rmt_copy_encoder_config_t *config,
                               rmt_encoder_handle_t *ret_encoder);
esp_err_t rmt_tx_register_event_callbacks(rmt_channel_handle_t channel,
                                          const rmt_tx_event_callbacks_t *callbacks,
                                          void *user_data);
esp_err_t rmt_enable(rmt_channel_handle_t channel);
esp_err_t rmt_transmit(rmt_channel_handle_t channel,
                       rmt_encoder_handle_t encoder,
                       const void *payload, size_t payload_bytes,
                       const rmt_transmit_config_t *config);

#endif /* AK_HOST_IDF_RMT_TX_H */
