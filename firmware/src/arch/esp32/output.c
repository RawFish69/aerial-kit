#include "esp.h"

#include "ak_dshot_timing.h"

#include "driver/ledc.h"
#include "driver/rmt_tx.h"
#include "esp_log.h"

/*
 * Motors and servos, on an ESP32.
 *
 * The F405 emits DShot with a timer and a DMA burst: one timer period per bit,
 * a compare value per bit, four motors interleaved into one burst. This chip
 * has nothing like that - it has the RMT, which is a small engine for saying
 * "high for this many ticks, low for this many, next" - so the *shape* of the
 * problem is different and the arithmetic is not: the levels and durations
 * come from ak_dshot_edges() in the core, which emits the same frame the timer
 * encoder does and is checked on a host against it.
 *
 * Two things are worth writing down about the numbers.
 *
 * The tick: the RMT's clock is the APB at 80 MHz and its divider is whole
 * (1..256), so the shortest tick is 12.5 ns and this uses 100 ns. DShot600 is
 * 1667 ns a bit, which is 16.67 ticks - not a whole number of anything - so a
 * bit is 17 ticks, 0.2% fast, and the duty is 35% or 70% of *that*. Every ESC
 * measures the high time against its own clock and accepts a margin; what none
 * of them accepts is a frame that is a *wrong* bit, which is why the encoder
 * is a function with tests rather than a loop in here.
 *
 * The queue, and this is the one thing that had to be found by watching it:
 * rmt_transmit() waits *forever* for a free transaction descriptor unless it
 * is told not to (rmt_transmit_config_t's queue_nonblocking flag). On a chip
 * whose RMT completes transmissions that is invisible, because a frame is
 * thirty microseconds against a millisecond loop; on a chip whose RMT never
 * completes - QEMU's, which has no such peripheral - the flight loop stops at
 * the first frame it cannot hand over, and the console stops with it. With the
 * flag set, a frame that will not fit is counted and dropped, which is what a
 * motor command is worth.
 *
 * Nothing here has driven an ESC. QEMU's ESP32 model has no RMT, so the
 * channels do not come up in the only environment this port has - which the
 * board reports and carries on from, and which is why the outputs are not
 * `ready` under emulation. A board is the next thing this needs.
 *
 * What the file *has* had since is a host model: the real code below runs
 * against a modelled RMT and LEDC (tests/idf-stub, tests/
 * host_esp32_output_model.c), and the check reads the symbols back out and
 * decodes them into the frame an ESC would see. That is what found that every
 * frame this file sent was a quarter of a frame - `rmt_transmit()` takes
 * bytes, not symbols - and it is the reason the line that hands them over says
 * `sizeof(rmt_symbol_word_t)` out loud.
 */

#define ESP_OUTPUT_TICK_NS    100u  /* one RMT tick, in nanoseconds */
#define ESP_OUTPUT_DSHOT_KHZ  300u  /* DShot300 to start with */
#define ESP_OUTPUT_RMT_MEM    64u   /* symbols per channel: 17 fit easily */

/*
 * The widest duty resolution *this chip's* LEDC has, which is not the same
 * across the family: the original ESP32 runs to 20 bits, the S2 and S3 stop at
 * 14 (and IDF hides the wider entries behind `SOC_LEDC_TIMER_BIT_WIDTH`, so
 * asking for 16 bits by name does not compile for them at all). Fourteen bits
 * of a 20 ms servo period is 1.2 microseconds a step, which is finer than any
 * servo's deadband - the resolution that matters for a servo is its pulse
 * width, not this. The S3 build found this, the same way it found the ADC's
 * calibration scheme.
 *
 * The duty the servo write computes is a fraction of the period, so its full
 * scale has to be the *selected* timer's full scale and not a constant. A duty
 * worked out against 16 bits and handed to a 14-bit timer is four times the
 * pulse it means - a 1500 us command arrives as 6 ms, which is outside the
 * range any hobby servo is meant to be given. What a servo does with that is
 * not measured here and is not claimed. The resolution and its bit count are
 * therefore chosen together, here, from the one capability, and the two cannot
 * drift apart.
 */
#if SOC_LEDC_TIMER_BIT_WIDTH > 14
#define ESP_OUTPUT_SERVO_RES  LEDC_TIMER_16_BIT
#define ESP_OUTPUT_SERVO_BITS 16u
#else
#define ESP_OUTPUT_SERVO_RES  LEDC_TIMER_14_BIT
#define ESP_OUTPUT_SERVO_BITS 14u
#endif
#define ESP_OUTPUT_QUEUE      4u    /* frames in flight before one is skipped */

#define ESP_OUTPUT_SERVO_HZ   50u
#define ESP_OUTPUT_SERVO_MAX  ((1u << ESP_OUTPUT_SERVO_BITS) - 1u)

/* The board names its own pins - this layer does not know a pin map, the same
 * rule the console and the UARTs follow - so they arrive here as arguments and
 * are kept for the two init functions below. */
static const int *output_motor_gpios;
static const int *output_servo_gpios;
/*
 * How many of each the *board* drives, which is not always AK_MAX_MOTORS.
 *
 * The core carries four motor slots because that is what the airframes it
 * flies need, but a chip decides how many of them can be driven: DShot here is
 * one RMT transmit channel per motor, and the ESP32-C3 has **two** of them
 * against the ESP32's eight. So the count comes down with the pins, and a
 * board that asks for more than it has gets exactly what it has - with the
 * shortfall visible in the report and caught by the preflight's mix-versus-
 * board line, rather than as a channel that silently did not come up.
 */
static unsigned output_motors;
static unsigned output_servos;

static rmt_channel_handle_t motor_channel[AK_MAX_MOTORS];
static rmt_encoder_handle_t motor_encoder;
static rmt_symbol_word_t motor_symbols[AK_MAX_MOTORS][AK_DSHOT_EDGES / 2u + 1u];
static int motors_ready;
/* One flag per channel, set when its frame has finished going out, so that the
 * driver tells a busy channel from a free one itself rather than by calling
 * transmit and reading the error. */
static volatile int motor_idle[AK_MAX_MOTORS];

static bool IRAM_ATTR on_motor_done(rmt_channel_handle_t channel,
                                    const rmt_tx_done_event_data_t *event,
                                    void *user_data)
{
    (void)channel;
    (void)event;
    *(volatile int *)user_data = 1;
    return false; /* nothing here needs the scheduler to yield */
}

static uint32_t output_khz = ESP_OUTPUT_DSHOT_KHZ;
static uint16_t output_ticks;
static uint32_t frames_sent;
static uint32_t frames_skipped;
static int servos_ready;

/* A duration list as RMT symbols: two pulses to a symbol, and the last symbol
 * half empty when there is an odd number of them - which is what a zero
 * duration in its second half says. */
static unsigned edges_to_symbols(const ak_dshot_edge_t *edges,
                                 rmt_symbol_word_t *symbols)
{
    unsigned at = 0u;
    unsigned symbol = 0u;

    while (at + 1u < AK_DSHOT_EDGES) {
        symbols[symbol].level0 = edges[at].level;
        symbols[symbol].duration0 = edges[at].ticks;
        symbols[symbol].level1 = edges[at + 1u].level;
        symbols[symbol].duration1 = edges[at + 1u].ticks;
        symbol++;
        at += 2u;
    }
    if (at < AK_DSHOT_EDGES) {
        symbols[symbol].level0 = edges[at].level;
        symbols[symbol].duration0 = edges[at].ticks;
        symbols[symbol].level1 = 0u;
        symbols[symbol].duration1 = 0u;
        symbol++;
    }
    return symbol;
}

static void motors_init(void)
{
    rmt_tx_channel_config_t cfg = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = 1000000000u / ESP_OUTPUT_TICK_NS,
        .mem_block_symbols = ESP_OUTPUT_RMT_MEM,
        .trans_queue_depth = ESP_OUTPUT_QUEUE,
    };
    rmt_copy_encoder_config_t encoder_cfg = {};

    if (rmt_new_copy_encoder(&encoder_cfg, &motor_encoder) != ESP_OK) {
        ESP_LOGW("ak-output", "no RMT copy encoder");
        return;
    }

    for (unsigned i = 0u; i < output_motors; i++) {
        cfg.gpio_num = output_motor_gpios[i];
        if (rmt_new_tx_channel(&cfg, &motor_channel[i]) != ESP_OK) {
            ESP_LOGW("ak-output", "no RMT channel for motor %u", i + 1u);
            return;
        }
        rmt_tx_event_callbacks_t callbacks = {
            .on_trans_done = on_motor_done,
        };
        /* The callback writes through a volatile pointer of its own; the
         * registration takes a plain void*, so the qualifier is cast away
         * here and re-applied where the flag is actually written. */
        if (rmt_tx_register_event_callbacks(motor_channel[i], &callbacks,
                                            (void *)&motor_idle[i]) != ESP_OK) {
            ESP_LOGW("ak-output", "no done callback for motor %u", i + 1u);
            return;
        }
        if (rmt_enable(motor_channel[i]) != ESP_OK) {
            ESP_LOGW("ak-output", "RMT channel %u did not start", i + 1u);
            return;
        }
        motor_idle[i] = 1;
    }

    output_ticks = ak_dshot_ticks_per_bit(output_khz, ESP_OUTPUT_TICK_NS);
    /* And a board that declares no motors has no DShot timing to report: the
     * arithmetic below produces a number whatever the count is, and a number
     * for channels that do not exist is exactly the kind of answer this file
     * spent 2026-09-18 learning not to give. */
    motors_ready = output_ticks != 0u && output_motors > 0u;
}

static void servos_init(void)
{
    ledc_timer_config_t timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = ESP_OUTPUT_SERVO_RES,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = ESP_OUTPUT_SERVO_HZ,
        .clk_cfg = LEDC_AUTO_CLK,
    };

    if (ledc_timer_config(&timer) != ESP_OK) {
        ESP_LOGW("ak-output", "no LEDC timer for the servos");
        return;
    }

    for (unsigned i = 0u; i < output_servos; i++) {
        ledc_channel_config_t channel = {
            .gpio_num = output_servo_gpios[i],
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .channel = (ledc_channel_t)(LEDC_CHANNEL_0 + i),
            .timer_sel = LEDC_TIMER_0,
            .duty = 0,
            .hpoint = 0,
        };
        if (ledc_channel_config(&channel) != ESP_OK) {
            ESP_LOGW("ak-output", "no LEDC channel for servo %u", i + 1u);
            return;
        }
    }
    servos_ready = 1;
}

int ak_esp_output_ready(void)
{
    /*
     * What the *board* declares is what has to have come up.
     *
     * This used to be `(motors_ready && frames_sent > 0) || servos_ready`,
     * which is a different question - "is an ESC being driven yet" - and which
     * answers *yes* for a board whose RMT channels never came up but whose
     * LEDC timer did: a quadrotor declares no servos, the servo timer is
     * configured anyway, `servos_ready` goes to 1, and the aircraft reports its
     * outputs present, arms, and writes DShot frames into channels that do not
     * exist. That is an armed aircraft with motors that never turn, which is
     * worse than a refusal: the pilot has no reason to look for the fault.
     *
     * The frame-sent half is not lost - it is what `output test` and the frame
     * counters are for, and the preflight reads `frames_sent` separately.
     */
    if (output_motors > 0u && !motors_ready) {
        return 0;
    }
    if (output_servos > 0u && !servos_ready) {
        return 0;
    }
    return output_motors > 0u || output_servos > 0u;
}

void ak_esp_output_init(const int *motor_gpios, unsigned motors,
                        const int *servo_gpios, unsigned servos)
{
    output_motor_gpios = motor_gpios;
    output_servo_gpios = servo_gpios;
    /* A second init is a fresh question, and this one is asked once per boot:
     * whatever the last attempt brought up says nothing about this one, and a
     * flag left set from it is a board that reports outputs it did not get. */
    motors_ready = 0;
    servos_ready = 0;
    /* A board cannot drive more than the core carries; one that asks for more
     * is a board file that will be told so by the preflight, not by an
     * out-of-bounds write here. */
    output_motors = motors > AK_MAX_MOTORS ? AK_MAX_MOTORS : motors;
    output_servos = servos > AK_MAX_SERVOS ? AK_MAX_SERVOS : servos;
    motors_init();
    servos_init();
}

void ak_esp_output_set_rate(uint32_t khz)
{
    uint16_t ticks = ak_dshot_ticks_per_bit(khz, ESP_OUTPUT_TICK_NS);

    if (ticks == 0u) {
        return; /* a rate DShot does not have: keep the one that works */
    }
    output_khz = khz;
    output_ticks = ticks;
}

void ak_esp_output_write(const ak_output_frame_t *frame)
{
    if (motors_ready) {
        for (unsigned motor = 0u; motor < output_motors; motor++) {
            ak_dshot_edge_t edges[AK_DSHOT_EDGES];
            rmt_transmit_config_t transmit = {
                .loop_count = 0,
                .flags.queue_nonblocking = 1,
            };
            unsigned symbols;

            /* Never call transmit on a channel that is still sending: IDF
             * logs an error every time that call fails, and a flight loop
             * filling the console with "no free transaction descriptor" once a
             * millisecond is a console nobody can read. */
            if (!motor_idle[motor]) {
                frames_skipped++;
                continue;
            }

            ak_dshot_edges(frame->dshot[motor], output_ticks, edges);
            symbols = edges_to_symbols(edges, motor_symbols[motor]);
            /* IDF takes the payload's *size in bytes*, and its copy encoder
             * walks that many bytes as `rmt_symbol_word_t` - four to a symbol.
             * Handing it the symbol count sent a quarter of the frame, and
             * neither the emulator nor the report could see it: QEMU has no
             * RMT to transmit with, so "no frame has completed" is what it
             * said either way. The host model reads the bytes the port passes
             * and decodes the symbols back into a frame, which is where this
             * was found (tests/test_arch_esp32_output.c). */
            if (rmt_transmit(motor_channel[motor], motor_encoder,
                             motor_symbols[motor],
                             symbols * sizeof(rmt_symbol_word_t), &transmit) !=
                ESP_OK) {
                frames_skipped++;
                continue;
            }
            motor_idle[motor] = 0;
            frames_sent++;
        }
    }

    if (servos_ready) {
        for (unsigned servo = 0u; servo < output_servos; servo++) {
            /* The pulse is in microseconds against a 20 ms period, so the duty
             * is the pulse over the period, scaled to whatever full scale this
             * chip's counter actually has - see ESP_OUTPUT_SERVO_MAX. The
             * division is after the multiply on purpose: the intermediate is
             * at most 20000 * 65535, which fits a uint32_t with room to
             * spare, and dividing first would round every pulse to the same
             * handful of steps. */
            uint32_t duty = ((uint32_t)frame->servo_us[servo] *
                             ESP_OUTPUT_SERVO_MAX) / 20000u;
            ledc_channel_t channel = (ledc_channel_t)(LEDC_CHANNEL_0 + servo);

            (void)ledc_set_duty(LEDC_LOW_SPEED_MODE, channel, duty);
            (void)ledc_update_duty(LEDC_LOW_SPEED_MODE, channel);
        }
    }
}

uint32_t ak_esp_output_dshot_hz(void)
{
    return motors_ready ? output_khz * 1000u : 0u;
}

uint32_t ak_esp_output_dshot_period(void)
{
    return motors_ready ? output_ticks : 0u;
}

uint32_t ak_esp_output_ccr_zero(void)
{
    return motors_ready ? ak_dshot_ccr_zero(output_ticks) : 0u;
}

uint32_t ak_esp_output_ccr_one(void)
{
    return motors_ready ? ak_dshot_ccr_one(output_ticks) : 0u;
}

uint32_t ak_esp_output_frames_sent(void)
{
    return frames_sent;
}

void ak_esp_output_report(ak_printf_fn out)
{
    if (!motors_ready) {
        out("outputs:   no dshot: the RMT channels did not come up\n");
    } else if (frames_sent == 0u) {
        out("dshot:     %u kHz configured, but no frame has completed - the "
            "RMT is not transmitting\n", output_khz);
        out("           frames sent 0, skipped %u\n", frames_skipped);
    } else {
        out("dshot:     %u kHz, %u ticks a bit of %u ns\n", output_khz,
            output_ticks, ESP_OUTPUT_TICK_NS);
        out("           RMT, one channel for each of %u motors, copy encoder, "
            "%u items\n", output_motors, AK_DSHOT_EDGES);
        out("           frames sent %u, skipped %u\n", frames_sent,
            frames_skipped);
    }
    out("servos:    %s\n", servos_ready ? "LEDC, 50 Hz, 16-bit"
                                        : "none: the LEDC channels did not come up");
}
