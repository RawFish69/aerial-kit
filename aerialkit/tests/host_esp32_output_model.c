/*
 * The modelled RMT and LEDC, and the IDF calls the ESP32's output layer makes.
 *
 * This file is compiled *instead of* IDF: `src/arch/esp32/output.c` is built
 * for the host with `tests/idf-stub` on its include path, so the calls in it
 * land here. Everything in this file is about being faithful to what IDF does
 * with those arguments; what the port does with the answers is the check's
 * business (tests/test_arch_esp32_output.c).
 */

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "driver/ledc.h"
#include "driver/rmt_tx.h"

#include "host_esp32_output_model.h"

#define MODEL_CHANNELS 8u
#define MODEL_LEDC_CHANNELS 8u
#define MODEL_LEDC_TIMERS 8u
#define MODEL_PAYLOAD_BYTES 512u
#define MODEL_LOGS 16u

struct host_rmt_channel {
    int created;
    int enabled;
    int held;
    gpio_num_t gpio;
    uint32_t resolution_hz;
    size_t mem_symbols;
    size_t queue_depth;
    rmt_tx_done_callback_t on_done;
    void *user_data;
    unsigned transmits;
    unsigned last_bytes;
    uint8_t payload[MODEL_PAYLOAD_BYTES];
};

static struct host_rmt_channel channels[MODEL_CHANNELS];
static struct host_rmt_encoder {
    int created;
} encoder;

/* The refusals a test can ask for: see the header. REFUSE_NEVER is past every
 * channel this model has. */
#define REFUSE_NEVER 0xFFFFFFFFu
static unsigned rmt_refuse_from = REFUSE_NEVER;
static unsigned ledc_refuse_from = REFUSE_NEVER;
static int      ledc_timer_refuses;
static unsigned rmt_created;
static unsigned ledc_created;
static int      rmt_encoder_refuses;
static unsigned rmt_enable_refuse_from = REFUSE_NEVER;
static unsigned rmt_transmit_refuse_from = REFUSE_NEVER;
static unsigned rmt_callback_refuse_from = REFUSE_NEVER;

struct host_ledc_timer {
    int configured;
    uint32_t freq_hz;
    unsigned resolution;
};

struct host_ledc_channel {
    int configured;
    int gpio;
    unsigned timer;
    uint32_t duty;
};

static struct host_ledc_timer ledc_timers[MODEL_LEDC_TIMERS];
static struct host_ledc_channel ledc_channels[MODEL_LEDC_CHANNELS];

static char logs[MODEL_LOGS][160];
static unsigned log_count;

void ak_host_idf_log(const char *tag, const char *fmt, ...)
{
    if (log_count >= MODEL_LOGS) {
        return;
    }

    char text[160];
    int at = snprintf(text, sizeof text, "[%s] ", tag);
    if (at < 0) {
        at = 0;
    }
    va_list args;
    va_start(args, fmt);
    vsnprintf(text + at, sizeof text - (size_t)at, fmt, args);
    va_end(args);
    snprintf(logs[log_count], sizeof logs[0], "%s", text);
    log_count++;
}

unsigned ak_host_idf_log_count(void)
{
    return log_count;
}

const char *ak_host_idf_log_text(unsigned index)
{
    return index < log_count ? logs[index] : "";
}

void ak_host_idf_reset(void)
{
    memset(channels, 0, sizeof channels);
    memset(&encoder, 0, sizeof encoder);
    memset(ledc_timers, 0, sizeof ledc_timers);
    memset(ledc_channels, 0, sizeof ledc_channels);
    memset(logs, 0, sizeof logs);
    log_count = 0;
    rmt_refuse_from = REFUSE_NEVER;
    ledc_refuse_from = REFUSE_NEVER;
    ledc_timer_refuses = 0;
    rmt_created = 0u;
    ledc_created = 0u;
    rmt_encoder_refuses = 0;
    rmt_enable_refuse_from = REFUSE_NEVER;
    rmt_transmit_refuse_from = REFUSE_NEVER;
    rmt_callback_refuse_from = REFUSE_NEVER;
}

void ak_host_rmt_refuse(unsigned from)
{
    rmt_refuse_from = from;
}

void ak_host_ledc_refuse(unsigned from)
{
    ledc_refuse_from = from;
}

void ak_host_ledc_timer_refuse(int refuse)
{
    ledc_timer_refuses = refuse;
}

void ak_host_rmt_encoder_refuse(int refuse)
{
    rmt_encoder_refuses = refuse;
}

void ak_host_rmt_enable_refuse(unsigned from)
{
    rmt_enable_refuse_from = from;
}

void ak_host_rmt_transmit_refuse(unsigned channel)
{
    rmt_transmit_refuse_from = channel;
}

void ak_host_rmt_callback_refuse(unsigned from)
{
    rmt_callback_refuse_from = from;
}

esp_err_t rmt_new_copy_encoder(const rmt_copy_encoder_config_t *config,
                               rmt_encoder_handle_t *ret_encoder)
{
    (void)config;
    if (ret_encoder == NULL) {
        return -1;
    }
    if (rmt_encoder_refuses > 0) {
        rmt_encoder_refuses--;
        return -1;
    }
    encoder.created = 1;
    *ret_encoder = &encoder;
    return ESP_OK;
}

esp_err_t rmt_new_tx_channel(const rmt_tx_channel_config_t *config,
                             rmt_channel_handle_t *ret_chan)
{
    if (config == NULL || ret_chan == NULL) {
        return -1;
    }
    /* The channels a test has asked this chip not to have: a resource that is
     * already allocated, which is what this fails as on real silicon. */
    if (rmt_created >= rmt_refuse_from) {
        return -1;
    }
    for (unsigned i = 0; i < MODEL_CHANNELS; i++) {
        if (!channels[i].created) {
            channels[i].created = 1;
            rmt_created++;
            channels[i].gpio = config->gpio_num;
            channels[i].resolution_hz = config->resolution_hz;
            channels[i].mem_symbols = config->mem_block_symbols;
            channels[i].queue_depth = config->trans_queue_depth;
            *ret_chan = &channels[i];
            return ESP_OK;
        }
    }
    return -1; /* no free channel, the way a part with all of them taken is */
}

esp_err_t rmt_tx_register_event_callbacks(rmt_channel_handle_t channel,
                                          const rmt_tx_event_callbacks_t *callbacks,
                                          void *user_data)
{
    if (channel == NULL || callbacks == NULL) {
        return -1;
    }
    for (unsigned i = 0; i < MODEL_CHANNELS; i++) {
        if (&channels[i] == channel) {
            if (i >= rmt_callback_refuse_from) {
                return -1;
            }
            channels[i].on_done = callbacks->on_trans_done;
            channels[i].user_data = user_data;
            return ESP_OK;
        }
    }
    return -1;
}

esp_err_t rmt_enable(rmt_channel_handle_t channel)
{
    if (channel == NULL) {
        return -1;
    }
    for (unsigned i = 0; i < MODEL_CHANNELS; i++) {
        if (&channels[i] == channel) {
            if (i >= rmt_enable_refuse_from) {
                return -1;
            }
            channels[i].enabled = 1;
            return ESP_OK;
        }
    }
    return -1;
}

esp_err_t rmt_transmit(rmt_channel_handle_t channel,
                       rmt_encoder_handle_t encoder_in,
                       const void *payload, size_t payload_bytes,
                       const rmt_transmit_config_t *config)
{
    /* There is one encoder in this model (the copy encoder), and the port has
     * one too - but the parameter is IDF's, so it is accepted and ignored
     * rather than dropped from the signature. */
    (void)encoder_in;
    if (channel == NULL || payload == NULL || config == NULL) {
        return -1;
    }

    for (unsigned i = 0; i < MODEL_CHANNELS; i++) {
        if (&channels[i] != channel) {
            continue;
        }
        if (!channels[i].enabled) {
            return -1; /* IDF: a disabled channel is ESP_ERR_INVALID_STATE */
        }
        if (i >= rmt_transmit_refuse_from) {
            /* A channel this test has asked to refuse the call - which IDF does
             * with ESP_ERR_INVALID_STATE when the queue cannot take it. */
            return -1;
        }
        if (payload_bytes > MODEL_PAYLOAD_BYTES) {
            return -1;
        }
        channels[i].transmits++;
        channels[i].last_bytes = (unsigned)payload_bytes;
        /* IDF's copy encoder walks the payload as symbols, four bytes at a
         * time - so this is exactly what would reach an ESC. */
        memcpy(channels[i].payload, payload, payload_bytes);
        return ESP_OK;
    }
    return -1;
}

void ak_host_rmt_complete(unsigned channel)
{
    if (channel >= MODEL_CHANNELS || !channels[channel].created) {
        return;
    }
    if (channels[channel].held) {
        return;
    }
    if (channels[channel].on_done != NULL) {
        rmt_tx_done_event_data_t event = { 0 };
        channels[channel].on_done(&channels[channel], &event,
                                  channels[channel].user_data);
    }
}

void ak_host_rmt_hold(unsigned channel, int hold)
{
    if (channel < MODEL_CHANNELS) {
        channels[channel].held = hold;
    }
}

unsigned ak_host_rmt_channels(void)
{
    unsigned count = 0u;
    for (unsigned i = 0; i < MODEL_CHANNELS; i++) {
        count += channels[i].created ? 1u : 0u;
    }
    return count;
}

int ak_host_rmt_gpio(unsigned channel)
{
    return channel < MODEL_CHANNELS ? channels[channel].gpio : -1;
}

uint32_t ak_host_rmt_resolution_hz(unsigned channel)
{
    return channel < MODEL_CHANNELS ? channels[channel].resolution_hz : 0u;
}

unsigned ak_host_rmt_mem_symbols(unsigned channel)
{
    return channel < MODEL_CHANNELS ? (unsigned)channels[channel].mem_symbols : 0u;
}

unsigned ak_host_rmt_queue_depth(unsigned channel)
{
    return channel < MODEL_CHANNELS ? (unsigned)channels[channel].queue_depth : 0u;
}

int ak_host_rmt_enabled(unsigned channel)
{
    return channel < MODEL_CHANNELS ? channels[channel].enabled : 0;
}

unsigned ak_host_rmt_transmits(unsigned channel)
{
    return channel < MODEL_CHANNELS ? channels[channel].transmits : 0u;
}

unsigned ak_host_rmt_last_bytes(unsigned channel)
{
    return channel < MODEL_CHANNELS ? channels[channel].last_bytes : 0u;
}

unsigned ak_host_rmt_last_symbols(unsigned channel)
{
    /* What IDF's copy encoder would get through: the payload is symbols. */
    return channel < MODEL_CHANNELS
               ? channels[channel].last_bytes / sizeof(rmt_symbol_word_t)
               : 0u;
}

const uint8_t *ak_host_rmt_last_payload(unsigned channel)
{
    return channel < MODEL_CHANNELS ? channels[channel].payload : NULL;
}

esp_err_t ledc_timer_config(const ledc_timer_config_t *timer_conf)
{
    if (timer_conf == NULL || (unsigned)timer_conf->timer_num >= MODEL_LEDC_TIMERS) {
        return -1;
    }
    if (ledc_timer_refuses > 0) {
        ledc_timer_refuses--;
        return -1;
    }
    ledc_timers[timer_conf->timer_num].configured = 1;
    ledc_timers[timer_conf->timer_num].freq_hz = timer_conf->freq_hz;
    ledc_timers[timer_conf->timer_num].resolution =
        (unsigned)timer_conf->duty_resolution;
    return ESP_OK;
}

esp_err_t ledc_channel_config(const ledc_channel_config_t *ledc_conf)
{
    if (ledc_conf == NULL ||
        (unsigned)ledc_conf->channel >= MODEL_LEDC_CHANNELS) {
        return -1;
    }
    if (ledc_created >= ledc_refuse_from) {
        return -1;
    }
    ledc_channels[ledc_conf->channel].configured = 1;
    ledc_created++;
    ledc_channels[ledc_conf->channel].gpio = ledc_conf->gpio_num;
    ledc_channels[ledc_conf->channel].timer = (unsigned)ledc_conf->timer_sel;
    ledc_channels[ledc_conf->channel].duty = ledc_conf->duty;
    return ESP_OK;
}

esp_err_t ledc_set_duty(ledc_mode_t speed_mode, ledc_channel_t channel,
                        uint32_t duty)
{
    (void)speed_mode;
    if ((unsigned)channel >= MODEL_LEDC_CHANNELS) {
        return -1;
    }
    ledc_channels[channel].duty = duty;
    return ESP_OK;
}

esp_err_t ledc_update_duty(ledc_mode_t speed_mode, ledc_channel_t channel)
{
    (void)speed_mode;
    (void)channel;
    return ESP_OK;
}

uint32_t ak_host_ledc_freq_hz(unsigned timer)
{
    return timer < MODEL_LEDC_TIMERS ? ledc_timers[timer].freq_hz : 0u;
}

unsigned ak_host_ledc_resolution(unsigned timer)
{
    return timer < MODEL_LEDC_TIMERS ? ledc_timers[timer].resolution : 0u;
}

unsigned ak_host_ledc_channels(void)
{
    unsigned count = 0u;
    for (unsigned i = 0; i < MODEL_LEDC_CHANNELS; i++) {
        count += ledc_channels[i].configured ? 1u : 0u;
    }
    return count;
}

int ak_host_ledc_gpio(unsigned channel)
{
    return channel < MODEL_LEDC_CHANNELS ? ledc_channels[channel].gpio : -1;
}

unsigned ak_host_ledc_timer_of(unsigned channel)
{
    return channel < MODEL_LEDC_CHANNELS ? ledc_channels[channel].timer : 0u;
}

uint32_t ak_host_ledc_duty(unsigned channel)
{
    return channel < MODEL_LEDC_CHANNELS ? ledc_channels[channel].duty : 0u;
}
