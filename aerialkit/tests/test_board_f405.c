/*
 * The F405 board file, which is the glue between this board and the core.
 *
 * It was the least-executed file in the firmware: every simulated session runs
 * the *simulator's* board instead, and `tests/test_arch.c` drives the port's
 * drivers directly rather than through the board. So the layer that decides
 * which USART the receiver is on, which pin selects the IMU, which SPI port the
 * sensors hang off and what "no divider fitted" reads as had never been run by
 * anything on this machine - and it is the first layer the bench exercises.
 *
 * What is checked is the *wiring*, against the same mapped register page the
 * rest of the port tests use: the baud divisor that lands in each UART's BRR,
 * the pin the LED and the IMU's chip select are on, the buses the core is
 * handed, and the four answers this board gives about parts that are not
 * soldered to it.
 *
 * `ak_board_init()` is called here, and that is the point of the file: it is
 * the exact sequence the target runs at boot, in the same order, and until now
 * nothing had run it anywhere except a board.
 */

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "arch.h"
#include "board.h"
#include "regs.h"

#include "ak_battery.h"
#include "ak_bus.h"
#include "ak_board.h"
#include "ak_flashlog.h"
#include "ak_log.h"
#include "ak_params.h"
#include "ak_rc.h"
#include "ak_rc_receiver.h"
#include "host_flash_model.h"
#include "host_spi_model.h"
#include "tests.h"

void USART1_IRQHandler(void);
void USART3_IRQHandler(void);

static char said[512];

static int sink(const char *fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    int written = vsnprintf(said + strlen(said), sizeof said - strlen(said),
                            fmt, args);
    va_end(args);
    return written;
}

/* The baud divisor this part needs: BRR is fCK/baud for the oversampling by 16
 * the port configures, **rounded to the nearest** - which is the arithmetic the
 * port itself got sixteen times wrong once, and the rounding is why the two
 * checks below first failed against a floor. The board's job is the *port* and
 * the *rate*; this is what they come out as. */
static uint32_t divisor(uint32_t clock_hz, uint32_t baud)
{
    return (clock_hz + baud / 2u) / baud;
}

/* The saved configuration, run here for the first time.
 *
 * This file ran everything the board does except the one thing it does with
 * flash. There was no flash or configuration check in it at all, so
 * `ak_board_config_read` and `ak_board_config_write` - the pair that decides
 * whether a save interrupted by a power cut is applied or refused - had never
 * executed on this machine: every simulated session runs the *simulator's*
 * store instead, and the simulator's store is a different piece of code that
 * agrees with this one about nothing it was not written to agree about.
 *
 * **The addresses are the thing under test.** 0x080C0000 is `AK_CONFIG_BASE0`
 * and 0x080E0000 is `AK_CONFIG_BASE` in board.c - sector 10 and sector 11, the
 * last two 128 KB sectors of the part. Sector 10 has not always been the
 * configuration's: `git show c400da4:aerialkit/src/boards/AERIALKIT_F405/board.c`
 * lists 0x080C0000 as the log region's *sixth* entry under the comment
 * "sectors 5..10", and ed0b058 took the same sector for the configuration's
 * second bank **without erasing it**. A log header is neither 0xFFFFFFFF nor
 * "AKCF" (`ak_flashlog.h` writes 0x414B464C, board.c writes 0x414B4346), so it
 * fails `slot_is_erased` and `slot_is_valid` together - and one such word makes
 * `all_erased()` false while `newest_slot()` still finds nothing, which is the
 * only pair of answers that makes the reader return -1.
 *
 * The board on the bench reports exactly that: `preflight` says `FAIL the saved
 * configuration is damaged (a bad length, or a checksum mismatch)`. Whether the
 * bytes in *its* sector 10 are log bytes could not be established from a
 * console that has no raw-flash command - that is what this test is for. It
 * establishes the mechanism and not the board's instance of it.
 */
static void test_the_saved_configuration(void)
{
    /* The reader's caller, as main.c calls it: the record's own maximum plus
     * the terminator the reader writes. */
    char buf[AK_PARAMS_TEXT_MAX + 1u];
    const uint32_t len = AK_PARAMS_TEXT_MAX;

    /* A part just out of reset: locked, and every byte of it 0xFF. The mapped
     * region is *not* this on its own - the mapping is anonymous memory and
     * arrives zeroed - so the model is reset here rather than assumed, for the
     * same reason the board tests map the register page rather than inherit
     * it. */
    host_flash_model_reset();

    expect("a part nobody has saved to reads as nothing stored, not as damage",
           ak_board_config_read(buf, len) == 0);

    /* Now the same part with a log header in sector 10, which is what an image
     * from the far side of ed0b058 left behind. The words are written straight
     * into the modelled flash rather than through `ak_flash_program`, because
     * these bytes are not something this firmware programs: they are what a
     * *previous* firmware left, already there when this one boots.
     *
     * One word is the whole of it. `all_erased()` asks each slot the question
     * `slot_is_erased()` asks - is the slot's first word 0xFFFFFFFF - and the
     * slot stride is 4 KB, so the log's magic in the sector's first word is a
     * slot that is neither erased nor "AKCF". The version and sequence behind
     * it are there so the bytes look like the header they are, and they change
     * no answer; they are not a claim about the header's layout, which this
     * test does not read. */
    volatile uint32_t *sector10 = (volatile uint32_t *)(uintptr_t)0x080C0000u;
    sector10[0] = AK_FLASHLOG_MAGIC;
    sector10[1] = AK_FLASHLOG_VERSION;

    expect("a log header left in the configuration's sector reads as damage",
           ak_board_config_read(buf, len) == -1);

    /* And now the control, and the answer that matters to the owner: a save
     * into the *other*, erased bank clears the report without sector 10 ever
     * being touched. The damage is only damage where there is nothing valid to
     * prefer - `newest_slot()` runs first, and the branch that consults
     * `all_erased()` is the one it did not take. */
    const char *text = "rates: roll 60 pitch 45";
    const uint32_t text_len = (uint32_t)strlen(text);

    expect("a save on an erased bank programs",
           ak_board_config_write(text, text_len) == 0);
    expect("and reads back byte for byte with the log header still in sector 10",
           ak_board_config_read(buf, len) == (int)text_len &&
               strcmp(buf, text) == 0);
    /* And the save did not go near sector 10 - which is the part worth
     * knowing: the report was cleared without the damaged sector being
     * repaired, so the log bytes are still sitting in the configuration's
     * bank. The FAIL is latent rather than gone: erase or lose bank 1 and the
     * reader is back to "not erased, nothing valid". A `save` would therefore
     * make the board stop complaining without making it clean, which is the
     * one thing that would have made reading the console after a save
     * misleading. */
    expect("and the save never went near sector 10 - the log header is still there",
           sector10[0] == AK_FLASHLOG_MAGIC);

    /* The serial is what makes "newest" a fact rather than an address, so a
     * second save has to beat the first and not merely follow it. */
    const char *second = "rates: roll 70 pitch 50";
    const uint32_t second_len = (uint32_t)strlen(second);
    expect("a second save goes to the next slot and becomes the newest",
           ak_board_config_write(second, second_len) == 0 &&
               ak_board_config_read(buf, len) == (int)second_len &&
               strcmp(buf, second) == 0);

    /* The reader's *other* -1, and the one everybody assumes is the cause: a
     * good record whose length outruns the caller's buffer. It is the same
     * record the two checks above read back whole, at a smaller length, which
     * is what makes it the other branch rather than a second symptom. */
    expect("and a record longer than the caller's buffer is the reader's other refusal",
           ak_board_config_read(buf, 4u) == -1);

    /* The writer's own bound, one byte short of the record rather than exactly
     * it, because a reader handed the record's length writes a terminator
     * after it. This is the same bound the ESP32's store and the simulator
     * apply, and the three agreeing about it is the point. */
    expect("the writer refuses a record that would fill the array",
           ak_board_config_write(text, AK_PARAMS_TEXT_MAX) == -1);
    expect("and refuses an empty one",
           ak_board_config_write(text, 0u) == -1);
}

void test_board_f405(void)
{
    /* The register model, brought up here rather than inherited. This used to
     * ask whether tests/test_arch.c had already mapped it and *return* if the
     * answer was no - which reported zero checks and no failures, so a run
     * that left this file out of the suite and a run in which every check here
     * passed looked the same from the outside. */
    expect("the register pages are mapped for the board's checks",
           ak_test_map_registers());
    if (!ak_test_registers_mapped()) {
        return; /* every write below would land on an unmapped page */
    }
    /* And the part a few hundred microseconds in, which is what a board at a
     * bench is: the crystal up and the PLL locked, so the first `ak_board_init`
     * below comes up on the clock this board is specified at. Without it the
     * boot falls back to the internal oscillator, and every rate derived from
     * the clock - the console's divisor among them - is then a different number
     * that the assertions below would still agree with themselves about. */
    ak_test_f405_crystal(1);

    /* --- the boot sequence itself --------------------------------------- */

    ak_board_init(); /* the target's own first step, in the same order */
    expect("the board comes up and says what it is",
           ak_board_name() != 0 && ak_board_name()[0] != '\0');
    /*
     * The clock these divisors are checked against is the one the *port* says
     * it is running at, not the 168 MHz the registers were configured for - and
     * that is a property of this machine rather than a choice: a mapped page
     * cannot report the switch-status field that only real silicon sets, so
     * `ak_clk_sysclk_hz()` here answers with the un-switched clock, and
     * `tests/test_arch.c` checks the two clock registers themselves for exactly
     * that reason. What this file is about is the *board*: that the port each
     * UART was given is the port the board names, at the rate the board names.
     */
    const uint32_t apb1 = ak_clk_apb1_hz();
    const uint32_t apb2 = ak_clk_apb2_hz();
    expect("the port has a clock to divide, whatever it ended up as",
           apb1 > 0u && apb2 > 0u);
    expect("and the board's own getters agree with the port's",
           ak_board_clock_apb1_hz() == apb1 &&
               ak_board_clock_sysclk_hz() > 0u);

    /* The console is USART2 at 115200 on APB1, and the board is what says so:
     * if this is the wrong port or the wrong rate, nothing anybody types at a
     * bench is heard. */
    expect("the console's divisor is the one its port and rate need",
           USART_BRR(AK_BOARD_CONSOLE_USART) ==
               divisor(apb1, AK_BOARD_CONSOLE_BAUD));
    expect("and the console is attached to that port",
           ak_board_console_port() == AK_BOARD_CONSOLE_USART &&
               ak_board_console_attached_port() == AK_BOARD_CONSOLE_USART);

    /* The LED: PC13 on a WeAct board, and the state has to follow the pin. */
    ak_board_led_set(1);
    expect("the LED is where the board says it is, and lit means lit",
           (GPIO_BSRR(AK_BOARD_LED_PIN.port) ==
            (1u << (AK_BOARD_LED_PIN.pin + (AK_BOARD_LED_ACTIVE_LOW ? 16u : 0u)))) &&
               ak_board_led_state() == 1);
    ak_board_led_set(0);
    expect("and dark is the other half of the register",
           (GPIO_BSRR(AK_BOARD_LED_PIN.port) ==
            (1u << (AK_BOARD_LED_PIN.pin + (AK_BOARD_LED_ACTIVE_LOW ? 0u : 16u)))) &&
               ak_board_led_state() == 0);

    /* --- the two UARTs the aircraft flies on ---------------------------- */

    ak_board_rc_init();
    expect("the receiver's port is the one the board claims",
           USART_BRR(AK_BOARD_RC_USART) ==
               divisor(apb2, AK_BOARD_RC_BAUD));
    expect("and it is a receiver port, not another console",
           AK_BOARD_RC_USART != AK_BOARD_CONSOLE_USART &&
               AK_BOARD_RC_USART != AK_BOARD_GPS_USART);
    expect("with the inverter fact the board was built with",
           ak_board_rc_inverted() == AK_BOARD_RC_INVERTER);

    ak_board_gps_init();
    expect("the GPS's port is its own",
           USART_BRR(AK_BOARD_GPS_USART) ==
               divisor(apb1, AK_BOARD_GPS_BAUD));

    /* Bytes in through the port's own interrupt, out through the board's poll:
     * the ring between them is the board's, and it is what a receiver needs. */
    for (int i = 0; i < 4; i++) {
        /* A mapped page cannot raise a receive flag by itself, so the test
         * raises it and then hands the byte over - the same way test_arch.c
         * drives the console's port. */
        USART_SR(AK_BOARD_RC_USART) = USART_SR_RXNE;
        USART_DR(AK_BOARD_RC_USART) = (uint32_t)('a' + i);
        USART1_IRQHandler();
    }
    int in_order = 1;
    for (int i = 0; i < 4; i++) {
        uint8_t byte = 0;
        if (ak_board_rc_poll(&byte) != 1 || byte != (uint8_t)('a' + i)) {
            in_order = 0;
        }
    }
    expect("what arrives at the receiver's port is what the board hands over",
           in_order);
    uint8_t nothing = 0;
    expect("and an empty ring is not a byte", ak_board_rc_poll(&nothing) == 0);

    for (int i = 0; i < 8; i++) {
        USART_SR(AK_BOARD_GPS_USART) = USART_SR_RXNE;
        USART_DR(AK_BOARD_GPS_USART) = (uint32_t)('0' + i);
        USART3_IRQHandler();
    }
    uint8_t byte = 0;
    expect("the GPS's port does the same on its own ring",
           ak_board_gps_poll(&byte) == 1 && byte == '0');

    /* A ring that overflows counts what it dropped rather than pretending:
     * the flight core reads that count and says so in the preflight. */
    for (int i = 0; i < 400; i++) {
        USART_SR(AK_BOARD_RC_USART) = USART_SR_RXNE;
        USART_DR(AK_BOARD_RC_USART) = (uint32_t)(i & 0xFF);
        USART1_IRQHandler();
    }
    expect("an overflowed ring counts its drops",
           ak_board_rc_dropped() > 0u);

    /* Telemetry out on the same wire the frames come in on, and the GPS's
     * outgoing configuration bytes on its own: both are the port's transmit
     * path, which needs the "ready to send" flag a mapped page cannot raise. */
    USART_SR(AK_BOARD_RC_USART) = USART_SR_TXE;
    expect("the board can send on the receiver's port",
           ak_board_rc_send("t", 1u) == 0 &&
               (USART_DR(AK_BOARD_RC_USART) & 0xFFu) == 't');
    USART_SR(AK_BOARD_GPS_USART) = USART_SR_TXE;
    expect("and on the GPS's",
           ak_board_gps_send("g", 1u) == 0 &&
               (USART_DR(AK_BOARD_GPS_USART) & 0xFFu) == 'g');
    expect("and it reports what its GPS ring dropped",
           ak_board_gps_dropped() == 0u);

    /* Switching the receiver to SBUS changes the port's line settings, not just
     * the parser: 100000 baud, eight bits, even parity, two stop bits. A board
     * that only told the parser would receive a stream of framing errors. */
    ak_board_rc_set_protocol(AK_RC_PROTOCOL_SBUS);
    expect("switching to SBUS changes the port's rate and its frame",
           USART_BRR(AK_BOARD_RC_USART) ==
               divisor(apb2, AK_BOARD_RC_SBUS_BAUD) &&
               (USART_CR1(AK_BOARD_RC_USART) & USART_CR1_PCE) != 0u &&
               (USART_CR2(AK_BOARD_RC_USART) & 0x3000u) == 0x2000u);
    ak_board_rc_set_protocol(AK_RC_PROTOCOL_CRSF);
    expect("and switching back puts the rate back",
           USART_BRR(AK_BOARD_RC_USART) == divisor(apb2, AK_BOARD_RC_BAUD));

    /* --- the buses the sensors hang off --------------------------------- */

    const ak_bus_t *imu = ak_board_imu_bus();
    expect("the IMU has a bus, with all four ways of using it",
           imu != 0 && imu->read != 0 && imu->write != 0 &&
               imu->write_burst != 0 && imu->delay_ms != 0);

    /* And the port it drives is brought up by the board's own init, which the
     * flight core calls before it opens a driver on the bus. */
    ak_board_imu_init();
    expect("which brings its SPI port up, with the chip select idle high",
           (SPI_CR1(AK_BOARD_IMU_SPI) & SPI_CR1_SPE) != 0u &&
               (GPIO_BSRR(AK_BOARD_IMU_CS.port) ==
                (1u << AK_BOARD_IMU_CS.pin)));

    if (imu != 0) {
        uint8_t value = 0;
        /* Nothing is on the bus - this is a bare board - so the honest answer
         * is an error rather than a reading. */
        /* Nothing is on the bus - this is a bare board - and what a real one
         * does is *shift*: the port puts a pull-up on MISO, so the byte that
         * comes back is 0xFF. What says "no part" is the driver's who-am-i
         * check, not a transfer that fails; the model answers the way the wires
         * do rather than the way a page of memory does. */
        expect("a bus with nothing on it shifts and reads the pull-up",
               imu->read(imu->ctx, 0x75, &value, 1) == 0 && value == 0xFFu);
        expect("and it leaves the chip select released, on the board's pin",
               GPIO_BSRR(AK_BOARD_IMU_CS.port) ==
                   (1u << AK_BOARD_IMU_CS.pin));
        expect("and a burst is one conversation, not four",
               imu->write_burst(imu->ctx, 0x7F, (const uint8_t[4]){1, 2, 3, 4},
                                4) == 0);
        expect("and a single register write is the same kind of answer",
               imu->write(imu->ctx, 0x7F, 0x00) == 0);
        imu->delay_ms(imu->ctx, 0u); /* a delay of nothing, so it returns */
    }

    /* The pack: the board's init is what decides whether there is one to read,
     * and the `#if` below checks the answer for the build's configuration. */
    ak_board_battery_init();

    /*
     * The four answers this board gives about parts that may or may not be
     * soldered to it. They are written against the *build's* flags rather than
     * against the way this board sits today, because the fitted configuration
     * is a build of its own (`make test EXTRA_CFLAGS="-DAK_BOARD_BARO_FITTED=1
     * -DAK_BOARD_VBAT_FITTED=1"`, and the workspace harness builds it beside
     * the default one) - and a check that only passed in one of the two would
     * be checking the flag rather than the board.
     */
#if AK_BOARD_BARO_FITTED
    expect("with the barometer fitted, its bus is there",
           ak_board_baro_bus() != 0);
#else
    expect("with nothing fitted, there is no barometer bus",
           ak_board_baro_bus() == 0);
#endif
#if AK_BOARD_RANGE_FITTED
    expect("with the rangefinder fitted, its bus is there",
           ak_board_range_bus() != 0);
#else
    expect("and no rangefinder bus", ak_board_range_bus() == 0);
#endif

#if AK_BOARD_VBAT_FITTED
    expect("with the divider fitted, the board says so",
           ak_board_battery_ready() != 0);
    said[0] = '\0';
    ak_board_battery_report(sink);
    expect("and its report is about a pack rather than a pad",
           strstr(said, "the divider is fitted") != 0);

    /*
     * And the conversion itself, which is what a fitted divider is for: the
     * counts come out of the data register, the board turns them into the
     * voltage at its pin, and the console says the number - the board's own
     * arithmetic held against the core's. Before the converter was modelled
     * this could not run anywhere: a page of memory cannot finish a conversion.
     */
    ADC_DR(AK_BOARD_VBAT_ADC) = 2000u;
    {
        float volts = ak_board_battery_pin_volts();
        float expected = ak_battery_pin_volts(2000u, AK_BOARD_VBAT_VREF,
                                              AK_BOARD_VBAT_FULL_SCALE);

        expect("a finished conversion is the pin's voltage, the core's way",
               volts > expected - 0.001f && volts < expected + 0.001f);
        said[0] = '\0';
        ak_board_battery_report(sink);
        expect("and the board's report prints the counts, not a timeout",
               strstr(said, "2000 counts") != 0);
    }
#else
    expect("and the pack divider is not fitted",
           ak_board_battery_ready() == 0);
    expect("which the core reads as no reading, not as zero volts",
           ak_board_battery_pin_volts() < 0.0f);
    said[0] = '\0';
    ak_board_battery_report(sink);
    expect("and the report says so rather than printing a voltage",
           strstr(said, "no divider is fitted") != 0 &&
               strstr(said, "counts = ") == 0);
#endif

    /* And it has no network, which is a different answer from "not connected":
     * the core opens no protocol on a board that says this. */
    expect("the first target has no network and says so",
           ak_board_net_ready() == 0 && ak_board_net_connected() == 0);

    /* --- the outputs, through the board's own wrapper ---------------------- */

    ak_board_output_init();
    expect("the board brings its outputs up and says how many it has",
           ak_board_output_ready() != 0);
    unsigned motors = 0;
    unsigned servos = 0;
    ak_board_output_shape(&motors, &servos);
    expect("four motors and two servos, which is what this board drives",
           motors == 4u && servos == 2u);
    /* And the register side of the same claim, because the servo bank is an
     * argument the board passes now: a board whose table drifted from its
     * header would still report four and two up there. This is the WeAct's own
     * pair - TIM2's first two channels at PA0 and PA1, alternate function 1 -
     * and the Feather's is TIM4 channels 3-4 at PB8/PB9 for the reason its
     * header gives. */
    expect("and its servos came out of the timer and pads it declares",
           strcmp(ak_output_servo_timer_name(), "TIM2") == 0 &&
               AK_BOARD_SERVO1_PIN.port == GPIOA_BASE &&
               AK_BOARD_SERVO1_PIN.pin == 0u &&
               AK_BOARD_SERVO2_PIN.port == GPIOA_BASE &&
               AK_BOARD_SERVO2_PIN.pin == 1u &&
               (GPIO_AFRL(GPIOA_BASE) & 0xFFu) == 0x11u &&
               TIM_CCR1(TIM2_BASE) == 1500u && TIM_CCR2(TIM2_BASE) == 1500u);
    expect("at a DShot rate it can state, with a period to match",
           ak_board_output_dshot_hz() >= 150000u &&
               ak_board_output_dshot_hz() <= 600000u &&
               ak_board_output_dshot_period() > 0u);
    ak_board_output_set_rate(600u);
    /* The two compare values are a zero bit and a one bit inside the period,
     * in that order - which is the whole of DShot at this level, and it is the
     * order a swapped pair would get wrong. */
    expect("and setting the rate moves the period it reports",
           ak_board_output_dshot_hz() == 600000u &&
               ak_board_output_ccr_zero() < ak_board_output_ccr_one() &&
               ak_board_output_ccr_one() <= ak_board_output_dshot_period());

    ak_output_frame_t frame;
    memset(&frame, 0, sizeof frame);
    frame.dshot[0] = 1000u; /* the throttle this frame asks for */
    unsigned sent_before = ak_board_output_frames_sent();
    ak_board_output_write(&frame);
    expect("and a frame written through the board reaches the port's outputs",
           ak_board_output_frames_sent() >= sent_before);
    said[0] = '\0';
    ak_board_output_report(sink);
    expect("and the board can say what its outputs are doing",
           strstr(said, "dshot") != 0 || strstr(said, "DShot") != 0);

    /* --- the SPI wiring, as a diagnostic a person can run ----------------- */

    said[0] = '\0';
    ak_board_spi_loopback(sink);
    expect("the loopback says something definite about the wires",
           strstr(said, "spi:") != 0);

    /*
     * And the loopback with the jumper in place, which is the success side of
     * the check a person runs at the bench: the model echoes what goes out, so
     * the board's report has to say the wire is there - and say the other thing
     * when it is not.
     */
    host_spi_reset();
    host_spi_set_echo(1);
    said[0] = '\0';
    ak_board_spi_loopback(sink);
    expect("the loopback reports a match when the jumper is there",
           strstr(said, "loopback matches") != NULL);
    host_spi_set_echo(0);
    said[0] = '\0';
    ak_board_spi_loopback(sink);
    expect("and reports what a floating miso does when it is not",
           strstr(said, "no match") != NULL);

    /* --- the rest of what a person at a bench reaches for --------------- */

    /*
     * The banner's clock line is the first thing to read on a board that looks
     * dead, and it has to get two things right: whether the crystal came up,
     * and which crystal the line says came up. (The second half was wrong here
     * until 2026-09-29 - see the assertion.) A board running on its internal
     * oscillator is alive and fails in ways
     * that read as firmware bugs - a wrong baud rate, a receiver that hears
     * nothing, a DShot frame at the wrong rate. The port's wait for the crystal
     * is bounded and gives up (see clk.c), so this is testable by taking the
     * ready bit away.
     */
    ak_test_f405_crystal(1);
    ak_board_init();
    /* 8 MHz, which is the board's crystal and what this file always said. It
     * said 12 for a day, on a reading taken on a different board - and this
     * assertion pinned the 12, which is what a test does to a wrong value:
     * makes it load-bearing. See the header's block.
     *
     * What is asserted here is the *fallback*: the port cannot measure a
     * crystal (there is no TIM11 in a process - see the Feather's test), so
     * `measure_hse()` finds no capture, returns 0, and the line prints
     * `AK_BOARD_HSE_MHZ` with `, assumed`. The name used to say "when it came
     * up", which is the branch this does not reach. The negative half searches
     * the full "HSE 8 MHz" rather than "8 MHz", because the sysclk field beside
     * it reads "168 MHz" and contains the shorter string. */
    expect("the clock line names this board's crystal when the measurement does "
           "not fire",
           strstr(ak_board_clock_summary(), "HSE 8 MHz x PLL") != NULL &&
               strstr(ak_board_clock_summary(), "HSE 12 MHz") == NULL &&
               ak_board_clock_ok() == 1);

    ak_test_f405_crystal(0);
    ak_board_init();
    expect("and says the crystal failed when it did, not a frequency",
           strstr(ak_board_clock_summary(), "HSE FAILED") != NULL &&
               ak_board_clock_ok() == 0);
    ak_test_f405_crystal(1);
    ak_board_init();

    /* A heartbeat toggles: the state and the pin both have to follow, or a
     * board with a blinking LED and a board sitting still look the same from
     * outside the case. */
    ak_board_led_set(1);
    ak_board_led_toggle();
    expect("the LED toggles from lit to dark, on the pin",
           ak_board_led_state() == 0 &&
               GPIO_BSRR(AK_BOARD_LED_PIN.port) ==
                   (1u << (AK_BOARD_LED_PIN.pin +
                           (AK_BOARD_LED_ACTIVE_LOW ? 0u : 16u))));
    ak_board_led_toggle();
    expect("and back", ak_board_led_state() == 1);

    /* The region the long log lives in: it is plain RAM marked not to be
     * cleared by the startup code, which is what makes it survive a reset and
     * not a battery going flat. Its size is the log ring's. */
    unsigned retained_bytes = 0;
    void *retained = ak_board_retained_ram(&retained_bytes);
    void *retained_again = ak_board_retained_ram(&retained_bytes);
    expect("the retained log is one region, with a size to go with it",
           retained != NULL && retained == retained_again &&
               retained_bytes >= sizeof(ak_log_t));

    /* The two ways out of the firmware. On the target the reset writes AIRCR
     * and the bootloader jumps to the ROM's vector table; in this build the
     * port's own file makes both no-ops, for the same reason a host cannot reset
     * itself - so what the board's half can be asked here is that it is wired
     * to the port and returns rather than doing something of its own. */
    ak_board_reboot();
    expect("the board's reboot goes to the port's, and comes back here",
           ak_board_clock_apb1_hz() > 0u);
    expect("and so does entering the ROM bootloader",
           ak_board_enter_bootloader() == 0);

    /* And the network this board does not have: four answers, and none of them
     * may be "connected". The core opens no protocol on a board that says
     * this, which is the whole reason they exist. */
    char net_byte = 'x';
    ak_board_net_start();
    expect("this board has no network, in every question that can be asked",
           ak_board_net_poll_rx(&net_byte) == 0 && ak_board_net_ready() == 0 &&
               ak_board_net_connected() == 0);
    said[0] = '\0';
    ak_board_net_report(sink);
    expect("and says so rather than staying quiet",
           strstr(said, "none on this board") != NULL);
    ak_board_net_write("x", 1u); /* because it must not care */

    /* And the one thing with flash in it, last because it resets the modelled
     * controller and rewrites the region the long log would live in. */
    test_the_saved_configuration();
}
