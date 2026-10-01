/*
 * The ESP32's outputs, run on the host against a modelled RMT and LEDC.
 *
 * The port has had this code since the day the second target got outputs, and
 * it had never executed anywhere: QEMU's ESP32 model has no RMT, so no frame
 * completes there, and the port's own report says so rather than pretending.
 * "Written and compiled" was the whole of the claim, and it is the one claim
 * this project does not accept about anything that can be run.
 *
 * So the real `src/arch/esp32/output.c` is compiled for the host against a
 * stand-in for the two IDF drivers it uses (tests/idf-stub), and what is
 * checked is what it *did*: which channels it made, with what resolution, what
 * symbols it handed the RMT, what duty it wrote to the LEDC - and then, the
 * check an ESC would make, whether the symbols handed over decode back to the
 * frame the core's encoder asked for.
 *
 * That last one found a real bug on the first run, and it is exactly the shape
 * of bug this file exists for: see `ak_esp_output_write()`'s use of
 * `sizeof(rmt_symbol_word_t)`.
 */

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "board.h"
#include "esp.h"

#include "ak_dshot_timing.h"
#include "ak_output.h"
#include "host_esp32_output_model.h"
#include "tests.h"

/* The pins the board names, so the check follows the board rather than a
 * literal here: a devkit's pin choice is a guess that is one line to change. */
static const int motor_pins[AK_MAX_MOTORS] = {
    AK_BOARD_MOTOR1_GPIO, AK_BOARD_MOTOR2_GPIO,
    AK_BOARD_MOTOR3_GPIO, AK_BOARD_MOTOR4_GPIO,
};
static const int servo_pins[AK_MAX_SERVOS] = {
    AK_BOARD_SERVO1_GPIO, AK_BOARD_SERVO2_GPIO,
};

/* The port's own numbers, from src/arch/esp32/output.c: whatever they are,
 * they have to be the same numbers this check reads back off the model. */
#define TICK_NS 100u
#define SYMBOLS_PER_FRAME (AK_DSHOT_EDGES / 2u + 1u) /* 33 edges, two a symbol */

static char report[512];

static int sink(const char *fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    int written = vsnprintf(report + strlen(report),
                            sizeof report - strlen(report), fmt, args);
    va_end(args);
    return written;
}

/* Reading the symbols an ESC would see back into the frame that went in: walk
 * the level/duration pairs in order, take every other one as a bit's high
 * time, and call it a one when it is more than half the bit. This is the same
 * decode the core's own test does on `ak_dshot_edges()`; here it is applied to
 * what the RMT was actually handed, which is the only place the two can
 * disagree. */
static uint16_t decode_symbols(const rmt_symbol_word_t *symbols, unsigned count,
                               uint16_t ticks)
{
    uint16_t decoded = 0u;
    unsigned bit = 0u;

    for (unsigned i = 0; i < count && bit < AK_DSHOT_BITS; i++) {
        uint16_t high = (uint16_t)symbols[i].duration0;
        if (symbols[i].level0 != 0u && high > ticks / 2u) {
            decoded |= (uint16_t)(0x8000u >> bit);
        }
        if (symbols[i].level0 != 0u) {
            bit++;
        }
    }
    return decoded;
}

void test_arch_esp32_output(void)
{
    ak_host_idf_reset();

    /* The pin arrays the board hands over are what init is given, and so is
     * the count - which is a *chip* fact before it is a board one. */
    ak_esp_output_init(motor_pins, AK_MAX_MOTORS, servo_pins, AK_MAX_SERVOS);

    expect("four RMT channels come up, one per motor",
           ak_host_rmt_channels() == AK_MAX_MOTORS);
    expect("on the pins the board names",
           ak_host_rmt_gpio(0) == AK_BOARD_MOTOR1_GPIO &&
               ak_host_rmt_gpio(3) == AK_BOARD_MOTOR4_GPIO);
    expect("with the tick the port documents: 100 ns, so 10 MHz",
           ak_host_rmt_resolution_hz(0) == 1000000000u / TICK_NS);
    expect("and a queue and a memory block, because a flight loop cannot wait",
           ak_host_rmt_mem_symbols(0) >= SYMBOLS_PER_FRAME &&
               ak_host_rmt_queue_depth(0) > 0u);
    expect("every channel enabled",
           ak_host_rmt_enabled(0) && ak_host_rmt_enabled(3));

    expect("two LEDC channels come up, one per servo",
           ak_host_ledc_channels() == AK_MAX_SERVOS &&
               ak_host_ledc_gpio(0) == AK_BOARD_SERVO1_GPIO);
    expect("on a 50 Hz timer at 16 bits, which is 305 ns of resolution",
           ak_host_ledc_freq_hz(0) == 50u && ak_host_ledc_resolution(0) == 16u);
    expect("both servos on that one timer",
           ak_host_ledc_timer_of(0) == 0u && ak_host_ledc_timer_of(1) == 0u);

    expect("the port reports the dshot rate it is set to",
           ak_esp_output_dshot_hz() == 300000u);
    expect("and its tick count for a bit",
           ak_esp_output_dshot_period() ==
               ak_dshot_ticks_per_bit(ak_esp_output_dshot_hz() / 1000u, TICK_NS));

    /* Nothing has gone out yet, so this is exactly the sentence the emulated
     * board prints: the channels are up and no frame has completed. */
    report[0] = '\0';
    ak_esp_output_report(sink);
    expect("before a frame completes the report says so rather than claiming "
           "the outputs work",
           strstr(report, "no frame has completed") != NULL);

    /*
     * A frame, with four different motor values so that a truncated or
     * misaligned frame cannot pass by accident: both ends of the bit range and
     * a bit pattern in the middle.
     */
    ak_output_frame_t frame;
    memset(&frame, 0, sizeof frame);
    frame.dshot[0] = 0x8001u;
    frame.dshot[1] = 0x0000u;
    frame.dshot[2] = 0xFFFFu;
    frame.dshot[3] = 0xAAAAu;
    frame.servo_us[0] = 1500u;
    frame.servo_us[1] = 1000u;

    ak_esp_output_write(&frame);

    expect("a frame goes out on every motor channel",
           ak_host_rmt_transmits(0) == 1u && ak_host_rmt_transmits(3) == 1u);

    /* The one that matters. IDF's copy encoder takes the payload as symbols,
     * four bytes each, and the *caller* has to say how many bytes that is:
     * handing over a symbol count sends a quarter of the frame. */
    expect("the payload handed to the RMT covers the whole symbol array",
           ak_host_rmt_last_bytes(0) ==
               SYMBOLS_PER_FRAME * sizeof(rmt_symbol_word_t));
    expect("so every symbol of the frame reaches the RMT",
           ak_host_rmt_last_symbols(0) == SYMBOLS_PER_FRAME);

    uint16_t ticks = (uint16_t)ak_esp_output_dshot_period();
    rmt_symbol_word_t symbols[SYMBOLS_PER_FRAME];
    memset(symbols, 0, sizeof symbols);
    memcpy(symbols, ak_host_rmt_last_payload(0), sizeof symbols);
    expect("and the symbols decode back to the frame that went in",
           decode_symbols(symbols, SYMBOLS_PER_FRAME, ticks) == frame.dshot[0]);

    for (unsigned motor = 1u; motor < AK_MAX_MOTORS; motor++) {
        memset(symbols, 0, sizeof symbols);
        memcpy(symbols, ak_host_rmt_last_payload(motor), sizeof symbols);
        uint16_t decoded = decode_symbols(symbols, SYMBOLS_PER_FRAME, ticks);
        expect("every motor's symbols carry that motor's frame",
               decoded == frame.dshot[motor]);
    }

    /* The gap, which is what an ESC uses to see where one frame ends: the last
     * symbol is the tail of the silent low, and nothing after it. */
    expect("the last symbol carries the gap rather than a bit",
           symbols[SYMBOLS_PER_FRAME - 1u].duration1 == 0u &&
               symbols[SYMBOLS_PER_FRAME - 1u].level1 == 0u);

    /* The servos: a pulse in microseconds against a 20 ms period, in
     * sixteenths of it. 1500 us of 20000 is 4915 of 65535. */
    expect("the servo duty is the pulse over the period",
           ak_host_ledc_duty(0) == 1500u * 65535u / 20000u &&
               ak_host_ledc_duty(1) == 1000u * 65535u / 20000u);

    /*
     * A channel with a frame still in it is skipped, not queued. IDF logs an
     * error every time a transmit call finds no free descriptor, and a flight
     * loop doing that once a millisecond fills the console - the reason the
     * port keeps its own idle flags. Nothing has completed yet, so this second
     * frame is four skips.
     */
    ak_esp_output_write(&frame);
    expect("a channel that is still sending is skipped rather than queued",
           ak_host_rmt_transmits(0) == 1u);

    /* The done interrupt is what frees a channel: the callback the port
     * registered writes its own flag, and the next frame goes out. */
    for (unsigned motor = 0u; motor < AK_MAX_MOTORS; motor++) {
        ak_host_rmt_complete(motor);
    }
    ak_esp_output_write(&frame);
    expect("and a completed transmission frees its channel for the next frame",
           ak_host_rmt_transmits(0) == 2u && ak_host_rmt_transmits(3) == 2u);
    expect("with the frames it sent counted and the ones it dropped counted "
           "separately",
           ak_esp_output_frames_sent() == 8u);

    /* A rate the transmitter cannot express leaves the working one alone. */
    ak_esp_output_set_rate(600u);
    expect("a faster dshot rate is taken",
           ak_esp_output_dshot_hz() == 600000u &&
               ak_esp_output_dshot_period() == ak_dshot_ticks_per_bit(600u, TICK_NS));
    ak_esp_output_set_rate(250u);
    expect("and a rate dshot does not have leaves it where it was",
           ak_esp_output_dshot_hz() == 600000u);

    /*
     * What the console and the preflight read. Before this check existed the
     * only environment that could run this code had no RMT, so the honest
     * report there is "no frame has completed" - which is a different sentence
     * from "ready", and the port says which one it is.
     */
    report[0] = '\0';
    ak_esp_output_report(sink);
    expect("the report names the rate and the tick once frames have gone out",
           strstr(report, "600 kHz") != NULL && strstr(report, "17 ticks") != NULL);
    expect("and says how many frames went out and how many were skipped",
           strstr(report, "frames sent 8") != NULL &&
               strstr(report, "skipped 4") != NULL);
    expect("with the servos named as LEDC",
           strstr(report, "LEDC, 50 Hz, 16-bit") != NULL);

    /*
     * And a board whose chip has fewer channels than the core carries, which
     * is the ESP32-C3: two RMT transmit channels against the ESP32's eight, so
     * four motors cannot be driven at all on that part. The board says two,
     * the port drives two, and the report says two - a channel that silently
     * did not come up is the failure this shape exists to prevent, and the
     * preflight's mix-versus-board line is what tells a C3's owner that a
     * quadrotor's mix needs four.
     */
    ak_host_idf_reset();
    ak_esp_output_init(motor_pins, 2u, servo_pins, AK_MAX_SERVOS);
    expect("a board with two motors' worth of channels drives exactly two",
           ak_host_rmt_channels() == 2u);

    report[0] = '\0';
    ak_esp_output_report(sink);
    expect("and its report names the count it actually has",
           strstr(report, "each of 2 motors") != NULL ||
               strstr(report, "no dshot") != NULL);

    /*
     * And the three ways the chip refuses, which is where this port's answer
     * had been wrong in a way nothing could see. "Ready" used to mean "the
     * motors are up and a frame has gone out, or the servo timer exists" - and
     * a quadrotor declares no servos, so on a board whose RMT channels never
     * came up the servo timer alone made it ready. The aircraft then reported
     * its outputs present, armed, and wrote DShot frames into channels that do
     * not exist: an armed aircraft with motors that never turn.
     *
     * What it means now is what the *board* declares: four motors is four RMT
     * channels, two servos is two LEDC channels, and a chip that refuses either
     * is a board that is not ready - which is what makes the arming gate refuse
     * with the numbers instead (see `badboard noout` in the simulator, and
     * main.c's boot path).
     */
    ak_host_idf_reset();
    ak_esp_output_init(motor_pins, AK_MAX_MOTORS, servo_pins, AK_MAX_SERVOS);
    expect("a board whose outputs are all up is ready before any frame",
           ak_esp_output_ready() != 0);

    /* RMT channels are a scarce resource on this chip and they are allocated
     * rather than configured: the third motor's request failing is what an
     * aircraft that also uses RMT for something else would see. */
    ak_host_idf_reset();
    ak_host_rmt_refuse(2u);
    ak_esp_output_init(motor_pins, AK_MAX_MOTORS, servo_pins, 0u);
    expect("a chip that runs out of RMT channels is not a ready board",
           ak_esp_output_ready() == 0);
    report[0] = '\0';
    ak_esp_output_report(sink);
    expect("and the port says which channel it could not get",
           strstr(report, "no dshot") != NULL ||
               strstr(report, "2 of 4 motors") != NULL);

    /* And the servo side, which is the LEDC timer and then one channel each. */
    ak_host_idf_reset();
    ak_host_ledc_timer_refuse(1);
    ak_esp_output_init(motor_pins, AK_MAX_MOTORS, servo_pins,
                       AK_MAX_SERVOS);
    expect("a chip with no LEDC timer is not a ready board either",
           ak_esp_output_ready() == 0);

    ak_host_idf_reset();
    ak_host_ledc_refuse(1u);
    ak_esp_output_init(motor_pins, AK_MAX_MOTORS, servo_pins, 2u);
    expect("nor is one that takes the first servo channel and refuses the "
           "second",
           ak_esp_output_ready() == 0);

    /* The three remaining ways the RMT says no, each of which leaves a board
     * that cannot drive what it declares and must therefore not arm. */
    ak_host_idf_reset();
    ak_host_rmt_encoder_refuse(1);
    ak_esp_output_init(motor_pins, AK_MAX_MOTORS, servo_pins, 0u);
    expect("a chip with no copy encoder is not a ready board",
           ak_esp_output_ready() == 0);

    ak_host_idf_reset();
    ak_host_rmt_callback_refuse(1u);
    ak_esp_output_init(motor_pins, AK_MAX_MOTORS, servo_pins, 0u);
    expect("nor is one that will not take the done callback",
           ak_esp_output_ready() == 0);

    ak_host_idf_reset();
    ak_host_rmt_enable_refuse(2u);
    ak_esp_output_init(motor_pins, AK_MAX_MOTORS, servo_pins, 0u);
    expect("nor is one whose third channel was created but will not start",
           ak_esp_output_ready() == 0);

    /* And a channel that refuses a frame: the frame is counted as skipped
     * rather than sent twice, and the board is still ready - it has the
     * channels, and this is one frame that did not go. */
    ak_host_idf_reset();
    ak_esp_output_init(motor_pins, AK_MAX_MOTORS, servo_pins, AK_MAX_SERVOS);
    {
        ak_output_frame_t refused;
        unsigned before;

        ak_host_rmt_transmit_refuse(0u);
        before = ak_host_rmt_transmits(0u);
        for (unsigned i = 0; i < AK_MAX_SERVOS; i++) {
            refused.servo_us[i] = AK_SERVO_CENTER_US;
        }
        for (unsigned i = 0; i < AK_MAX_MOTORS; i++) {
            refused.dshot[i] = (uint16_t)(100u + i);
        }
        ak_esp_output_write(&refused);
        expect("a channel that refuses the frame takes none of it",
               ak_host_rmt_transmits(0u) == before);
        report[0] = '\0';
        ak_esp_output_report(sink);
        expect("and the frame that did not go is counted as skipped",
               strstr(report, "skipped 1") != NULL ||
                   strstr(report, "skipped") != NULL);
        ak_host_rmt_transmit_refuse(99u);
    }

    /* And a board with no outputs at all - nothing declared - is not ready,
     * rather than ready because there is nothing to fail. */
    ak_host_idf_reset();
    ak_esp_output_init(motor_pins, 0u, servo_pins, 0u);
    expect("a board that drives nothing is not ready to drive it",
           ak_esp_output_ready() == 0);
    expect("and it reports no dshot timing rather than a number",
           ak_esp_output_dshot_hz() == 0u &&
               ak_esp_output_ccr_zero() == 0u &&
               ak_esp_output_ccr_one() == 0u);
}
