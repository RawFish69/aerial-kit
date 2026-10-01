#ifndef AK_HOST_ESP32_OUTPUT_MODEL_H
#define AK_HOST_ESP32_OUTPUT_MODEL_H

#include <stdint.h>

#include "driver/rmt_types.h"

/*
 * The ESP32's output layer, run on the host against a modelled RMT and LEDC.
 *
 * Why this exists is the same reason the F405's flash and I2C drivers have
 * models: the *real* code runs, and what is checked is what it did to the
 * peripheral rather than what it says it did. This port needed it more than
 * most, because the only environment it could run in has no RMT at all - QEMU
 * does not complete a transmission, so `output.c`'s success path had never
 * executed anywhere, and its own report says so ("no frame has completed").
 *
 * The model is deliberately literal about one thing: `rmt_transmit` takes
 * **bytes**, as IDF's own header says, and stores that many bytes of the
 * payload. A caller passing a symbol count therefore sends a quarter of the
 * frame, and this model reproduces that rather than papering over it - the
 * check that reads the symbols back is what finds it.
 *
 * What it cannot model: anything electrical, and anything about time. A
 * transmission completes when the test says it does, not when the hardware
 * would have finished, so the interrupt path is exercised on purpose rather
 * than raced.
 */

/* Fresh model, as if the chip had just come out of reset. */
void ak_host_idf_reset(void);

/* What the RMT channels were created with. */
unsigned ak_host_rmt_channels(void);
int      ak_host_rmt_gpio(unsigned channel);
uint32_t ak_host_rmt_resolution_hz(unsigned channel);
unsigned ak_host_rmt_mem_symbols(unsigned channel);
unsigned ak_host_rmt_queue_depth(unsigned channel);
int      ak_host_rmt_enabled(unsigned channel);

/* The last transmission handed to a channel: how many bytes the caller said
 * the payload was, and the bytes themselves. IDF's copy encoder turns the
 * payload into symbols at four bytes each, which is what
 * `ak_host_rmt_last_symbols()` counts. */
unsigned ak_host_rmt_transmits(unsigned channel);
unsigned ak_host_rmt_last_bytes(unsigned channel);
unsigned ak_host_rmt_last_symbols(unsigned channel);
const uint8_t *ak_host_rmt_last_payload(unsigned channel);

/* Finish whatever a channel is sending, the way the peripheral's done
 * interrupt would: the registered callback runs with its own user data. */
void ak_host_rmt_complete(unsigned channel);

/* Hold a channel busy: transmissions still "queue", but nothing completes
 * until the test lets go - which is how a test sees the driver's behaviour on
 * a channel it cannot transmit on yet. */
void ak_host_rmt_hold(unsigned channel, int hold);

/*
 * And the chips the board does not have: the peripheral refusing to create
 * something. `ak_host_rmt_refuse(n)` makes the *n*th channel request fail from
 * then on (0 is the first), and `ak_host_ledc_refuse(n)` does the same for the
 * servo channels. This is the failure the ESP32 is most likely to have in
 * practice - RMT channels are a scarce resource and they are allocated, not
 * configured - and the one that has to end in an aircraft that refuses to arm
 * rather than one whose motors never turn.
 */
void ak_host_rmt_refuse(unsigned from);
void ak_host_ledc_refuse(unsigned from);
void ak_host_ledc_timer_refuse(int refuse);
/* The three other ways the RMT says no: the encoder, a channel that exists but
 * will not start, and a transmission the peripheral refuses at the call. */
void ak_host_rmt_encoder_refuse(int refuse);
void ak_host_rmt_enable_refuse(unsigned from);
void ak_host_rmt_transmit_refuse(unsigned channel);
void ak_host_rmt_callback_refuse(unsigned from);

/* The LEDC side, which is one timer and a duty per servo channel. */
uint32_t ak_host_ledc_freq_hz(unsigned timer);
unsigned ak_host_ledc_resolution(unsigned timer);
unsigned ak_host_ledc_channels(void);
int      ak_host_ledc_gpio(unsigned channel);
unsigned ak_host_ledc_timer_of(unsigned channel);
uint32_t ak_host_ledc_duty(unsigned channel);

#endif /* AK_HOST_ESP32_OUTPUT_MODEL_H */
