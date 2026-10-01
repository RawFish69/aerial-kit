/*
 * The wing's board file, the same way the F405's is checked in
 * tests/test_board_f405.c - and the same reason: every simulated session runs
 * the *simulator's* board, so the layer that decides which USART the receiver
 * is on, which pin selects the IMU and which bus a sensor hangs off had never
 * been executed on this machine.
 *
 * This board is a different animal from the F405's dev board: it has the IMU,
 * the barometer and the divider *fitted*, two motors and two servos on timers,
 * and its console is USART1 rather than USART2 (F405) - the kind of difference
 * a board file exists to state, and the kind a copy-and-paste from the other
 * one gets wrong.
 *
 * The entry points are renamed for this build (`ak_ghf435_*`, which is what
 * `board.h`'s `#ifdef AK_HOST_AT32` is for), so this test drives *this* board
 * with the plain names the core uses.
 */

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "at32f435/arch.h"
#include "board.h"

#include "ak_battery.h"
#include "ak_baro.h"
#include "ak_bus.h"
#include "ak_params.h"
#include "ak_board.h"
#include "ak_log.h"
#include "ak_imu.h"
#include "ak_rc.h"
#include "ak_rc_receiver.h"
#include "host_adc_model.h"
#include "host_i2c_model_at32.h"
#include "host_spi_model.h"
#include "tests.h"

void USART1_IRQHandler(void);
void USART2_IRQHandler(void);
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

/* BRR is fCK/baud, rounded to the nearest, for the oversampling by 16 this port
 * configures - the same arithmetic the F405's board test checks, on a part
 * whose clock tree is not the same one. */
static uint32_t divisor(uint32_t clock_hz, uint32_t baud)
{
    return (clock_hz + baud / 2u) / baud;
}

/* The AT32 sets and clears a pin through two different registers, so "lit"
 * means one of them was written - which is what these two helpers make
 * readable at the call sites below. */
static uint32_t scr(ak_pin_t pin)
{
    return AK_GPIO_SCR(pin.port);
}

static uint32_t clr(ak_pin_t pin)
{
    return AK_GPIO_CLR(pin.port);
}

void test_board_ghf435(void)
{
    /* The register model, brought up here rather than inherited - the two
     * arches put their clock unit and their flash controller at the same
     * addresses, so this is one mapping and not two. See the same guard in
     * tests/test_board_f405.c for what asking instead of establishing cost. */
    expect("the register pages are mapped for this board's checks",
           ak_test_map_registers());
    if (!ak_test_registers_mapped()) {
        return; /* every write below would land on an unmapped page */
    }
    /* And the part a few hundred microseconds in, so the boot below comes up on
     * the clock this board is specified at rather than on the internal
     * oscillator it falls back to when nothing answers the crystal wait.
     *
     * This one is load-bearing and was measured, not reasoned: with the clock
     * line left at the reset value the barometer section below fails two
     * checks - "and the part opens through it", "with a sample that says it is
     * valid" - because `ak_i2c_init` derives the bus timing from the clock and
     * a bus timed for 16 MHz does not complete the DPS310's transactions. It
     * passed before only because `test_arch_at32()` had been left to run first
     * and its last test happened to leave the clock up. */
    ak_test_at32_clock_up();

    /* --- the boot sequence, which on a board with parts fitted is real work */

    ak_board_init();
    expect("the board comes up and names itself",
           ak_board_name() != 0 && strstr(ak_board_name(), "GHF") != 0);
    expect("with a clock to divide", ak_clk_apb1_hz() > 0u &&
                                         ak_clk_apb2_hz() > 0u);

    const uint32_t apb1 = ak_clk_apb1_hz();
    const uint32_t apb2 = ak_clk_apb2_hz();
    expect("the console is on the port and rate the board states",
           AK_USART_BAUDR(AK_BOARD_CONSOLE_USART) ==
               divisor(apb2, AK_BOARD_CONSOLE_BAUD));
    expect("and that port is this board's, not the F405's USART2",
           AK_BOARD_CONSOLE_USART != USART2_BASE);

    /* The LED, which is active low here too: the tell-tale's patterns are read
     * off it. */
    ak_board_led_set(1);
    expect("lit means the pin is cleared, and the state says lit",
           clr(AK_BOARD_LED_PIN) == (1u << AK_BOARD_LED_PIN.pin) &&
               ak_board_led_state() == 1);
    ak_board_led_set(0);
    expect("and dark sets it", scr(AK_BOARD_LED_PIN) ==
                                   (1u << AK_BOARD_LED_PIN.pin) &&
                                   ak_board_led_state() == 0);

    /* --- the receiver's and the GPS's ports, which are not the console's --- */

    ak_board_rc_init();
    expect("the receiver's port is the one the board names",
           AK_USART_BAUDR(AK_BOARD_RC_USART) ==
               divisor(apb1, AK_BOARD_RC_BAUD));
    expect("and none of the three ports is another one",
           AK_BOARD_RC_USART != AK_BOARD_CONSOLE_USART &&
               AK_BOARD_RC_USART != AK_BOARD_GPS_USART &&
               AK_BOARD_GPS_USART != AK_BOARD_CONSOLE_USART);

    /* The port the board names is not the whole of the wiring: the pins are,
     * and the multiplexer register is where a wrong pin shows up. PB0's mux is
     * in the low register, four bits per pin. */
    expect("the receiver's receive pin is on the board's alternate function",
           ((AK_GPIO_MUXL(AK_BOARD_RC_RX.port) >> (4u * AK_BOARD_RC_RX.pin)) &
            0xFu) == AK_BOARD_RC_RX_AF);

    for (int i = 0; i < 4; i++) {
        AK_USART_STS(AK_BOARD_RC_USART) = AK_USART_STS_RDBF;
        AK_USART_DT(AK_BOARD_RC_USART) = (uint32_t)('a' + i);
        USART2_IRQHandler();
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

    ak_board_gps_init();
    expect("the GPS's port is at its own rate",
           AK_USART_BAUDR(AK_BOARD_GPS_USART) ==
               divisor(apb1, AK_BOARD_GPS_BAUD));
    for (int i = 0; i < 8; i++) {
        AK_USART_STS(AK_BOARD_GPS_USART) = AK_USART_STS_RDBF;
        AK_USART_DT(AK_BOARD_GPS_USART) = (uint32_t)('0' + i);
        USART3_IRQHandler();
    }
    uint8_t byte = 0;
    expect("and its bytes come out of its own ring",
           ak_board_gps_poll(&byte) == 1 && byte == '0');

    for (int i = 0; i < 400; i++) {
        AK_USART_STS(AK_BOARD_RC_USART) = AK_USART_STS_RDBF;
        AK_USART_DT(AK_BOARD_RC_USART) = (uint32_t)(i & 0xFF);
        USART2_IRQHandler();
    }
    expect("an overflowed ring counts its drops",
           ak_board_rc_dropped() > 0u);

    /* Telemetry out on the receiver's wire and configuration out on the GPS's:
     * the port's transmit path, which needs a flag a mapped page cannot
     * raise. */
    AK_USART_STS(AK_BOARD_RC_USART) = AK_USART_STS_TDBE;
    expect("the board can send on the receiver's port",
           ak_board_rc_send("t", 1u) == 0 &&
               (AK_USART_DT(AK_BOARD_RC_USART) & 0xFFu) == 't');
    AK_USART_STS(AK_BOARD_GPS_USART) = AK_USART_STS_TDBE;
    expect("and on the GPS's",
           ak_board_gps_send("g", 1u) == 0 &&
               (AK_USART_DT(AK_BOARD_GPS_USART) & 0xFFu) == 'g');
    expect("and it reports what its GPS ring dropped",
           ak_board_gps_dropped() == 0u);

    /* The protocol switch is a line setting, not only a parser choice: SBUS is
     * 100000 baud, eight bits, even parity and two stop bits. */
    ak_board_rc_set_protocol(AK_RC_PROTOCOL_SBUS);
    expect("switching to SBUS changes the port's rate and its frame",
           AK_USART_BAUDR(AK_BOARD_RC_USART) ==
               divisor(apb1, AK_BOARD_RC_SBUS_BAUD) &&
               (AK_USART_CTRL1(AK_BOARD_RC_USART) & AK_USART_CTRL1_PEN) != 0u &&
               (AK_USART_CTRL2(AK_BOARD_RC_USART) &
                (AK_USART_CTRL2_STOP_2 << AK_USART_CTRL2_STOP_SHIFT)) ==
                   (AK_USART_CTRL2_STOP_2 << AK_USART_CTRL2_STOP_SHIFT));
    ak_board_rc_set_protocol(AK_RC_PROTOCOL_CRSF);
    expect("and switching back puts the rate back",
           AK_USART_BAUDR(AK_BOARD_RC_USART) ==
               divisor(apb1, AK_BOARD_RC_BAUD));

    /* The console's port is checked by the console's own interrupt: USART1. */
    AK_USART_STS(AK_BOARD_CONSOLE_USART) = AK_USART_STS_RDBF;
    AK_USART_DT(AK_BOARD_CONSOLE_USART) = 'k';
    USART1_IRQHandler();
    char typed = 0;
    expect("and the console's bytes arrive on the console's port",
           ak_board_console_poll_rx(&typed) == 1 && typed == 'k');

    /* --- the buses, which on this board have parts behind them ------------ */

    const ak_bus_t *imu = ak_board_imu_bus();
    expect("the IMU has a bus, with all four ways of using it",
           imu != 0 && imu->read != 0 && imu->write != 0 &&
               imu->write_burst != 0 && imu->delay_ms != 0);
    ak_board_imu_init();
    expect("brought up by the board's own init, on the pin it names",
           scr(AK_BOARD_IMU_CS) == (1u << AK_BOARD_IMU_CS.pin));

    if (imu != 0) {
        uint8_t value = 0;
        /* Nothing is on the bus - this is a bare board - and what a real one
         * does is *shift*: the port puts a pull-up on MISO, so the byte that
         * comes back is 0xFF. What says "no part" is the driver's who-am-i
         * check, not a transfer that fails; the model answers the way the wires
         * do rather than the way a page of memory does. */
        expect("a bus with nothing on it shifts and reads the pull-up",
               imu->read(imu->ctx, 0x75, &value, 1) == 0 && value == 0xFFu);
        expect("and the burst is the same conversation",
               imu->write_burst(imu->ctx, 0x7F, (const uint8_t[4]){1, 2, 3, 4},
                                4) == 0);
        expect("and a single register write is the same kind of answer",
               imu->write(imu->ctx, 0x7F, 0x00) == 0);
        imu->delay_ms(imu->ctx, 0u);
    }

    /*
     * And the part this board really has, through the board's own bus: an
     * ICM-42688-P, with the model answering as one. This is the check the
     * board's read, its write and its burst exist for - the flight core opens
     * *this* bus at boot and flies on what comes back - and until the SPI
     * model existed there was nothing on the other end of it but a page of
     * memory that answered nothing at all.
     */
    host_spi_reset();
    host_spi_set_present(1);
    host_spi_set_register(0x75u, 0x47u); /* who-am-i: an ICM-42688-P */
    /* Level, one g up, and a quarter turn a second about z. The driver reads
     * all twelve bytes as one burst and takes each axis big-endian. */
    host_spi_set_register(0x1Fu, 0x00u); host_spi_set_register(0x20u, 0x00u);
    host_spi_set_register(0x21u, 0x00u); host_spi_set_register(0x22u, 0x00u);
    host_spi_set_register(0x23u, 0x08u); host_spi_set_register(0x24u, 0x00u);
    host_spi_set_register(0x25u, 0x00u); host_spi_set_register(0x26u, 0x00u);
    host_spi_set_register(0x27u, 0x00u); host_spi_set_register(0x28u, 0x00u);
    /* gyro z: 25 counts is 1.5 deg/s at 16.4 LSB/dps, which is 0.02618
     * radians a second - the expected number in the check below. */
    host_spi_set_register(0x29u, 0x00u); host_spi_set_register(0x2Au, 0x19u);
    {
        ak_imu_t part;
        ak_imu_sample_t sample;
        const ak_bus_t *bus = ak_board_imu_bus();

        memset(&sample, 0, sizeof sample);
        expect("the board's own imu bus opens the part the board has",
               bus != 0 && ak_imu_open(&part, bus, sink) == 0 &&
                   part.present);
        expect("which is the part the board file names",
               host_spi_register(0x75u) == 0x47u);
        expect("and a sample comes back in g and radians a second",
               ak_imu_read(&part, &sample) == 0 && sample.valid &&
                   sample.accel[2] > 0.9999f && sample.accel[2] < 1.0001f &&
                   /* 25 counts at 16.4 LSB/dps is 1.5244 deg/s, which is
                    * 0.026606 radians a second. */
                   sample.gyro[2] > 0.02655f && sample.gyro[2] < 0.02665f);
        expect("with the burst taken as one read of twelve bytes",
               host_spi_transfers() > 0u);
    }

    /* And the two parts this board *has*: the bus is there, which is the
     * different answer from the bare F405 board's. */
    expect("the barometer's bus exists, because this board has one",
           ak_board_baro_bus() != 0 &&
               ak_board_baro_bus()->read != 0 &&
               ak_board_baro_bus()->write != 0);
    /*
     * And the part on that bus, opened through the board's own bus - the same
     * call the flight core makes at boot, and the wing's only altitude: a
     * DPS310 answering as the datasheet says, with the driver's own waits
     * landing on the port's `baro_bus_delay`. Nothing had ever run that
     * function: the tests probed the part through a bus they built themselves,
     * so the board's own vtable - and the delay in it - was compiled and never
     * called.
     */
    host_i2c_at32_reset();
    ak_i2c_init(AK_BOARD_BARO_I2C, AK_BOARD_BARO_SCL, AK_BOARD_BARO_SDA,
                AK_BOARD_BARO_AF, AK_BOARD_BARO_SPEED);
    {
        /* A DPS310's register file: the product id at 0x0D, and the two ready
         * bits at 0x08 that say the coefficients are loaded and a measurement
         * is waiting. */
        uint8_t dps310[16] = { 0 };

        dps310[0x08] = 0xF0u;
        dps310[0x0D] = 0x10u;

        const ak_bus_t *bus = ak_board_baro_bus();
        ak_baro_t part;
        ak_baro_sample_t sample;

        expect("a dps310 can be attached to the board's own i2c bus",
               host_i2c_at32_attach(AK_BOARD_BARO_ADDRESS, dps310,
                                    sizeof dps310) >= 0);
        expect("and the part opens through it",
               bus != 0 && ak_baro_open(&part, bus, sink) == 0 &&
                   part.present);
        expect("with a sample that says it is valid",
               ak_baro_read(&part, &sample) == 1 && sample.valid);
    }

    /* What the *bus* is really about is the pins: a barometer on the wrong
     * multiplexer setting is a barometer that never answers, and this is the
     * register that says so. (Whether a transaction succeeds is the modelled
     * part's business, and the AT32's I2C model answers.) */
    expect("and its pins are on the alternate function the board names",
           ((AK_GPIO_MUXL(AK_BOARD_BARO_SCL.port) >>
             (4u * AK_BOARD_BARO_SCL.pin)) &
            0xFu) == AK_BOARD_BARO_AF &&
               ((AK_GPIO_MUXL(AK_BOARD_BARO_SDA.port) >>
                 (4u * AK_BOARD_BARO_SDA.pin)) &
                0xFu) == AK_BOARD_BARO_AF);

    /* The pack divider is fitted on this board - the opposite answer from the
     * F405's, and the reason the core's battery line reads differently on the
     * two targets. */
    expect("the pack divider is fitted here",
           ak_board_battery_ready() != 0 ||
               ak_board_battery_pin_volts() >= 0.0f ||
               AK_BOARD_VBAT_FITTED == 0);

    ak_board_battery_init();
    said[0] = '\0';
    ak_board_battery_report(sink);
    expect("and its report is about a pack, on the board that has one",
           strstr(said, "PC0") != 0 || strstr(said, "divider") != 0);


    /*
     * And the conversion itself, which is what a *fitted* divider is for: the
     * counts come out of the data register, the board turns them into the
     * voltage at its pin, and the console says the number. The board's own
     * arithmetic is held against the core's, so a board file cannot scale the
     * pack its own way - and before the converter was modelled this path could
     * not run at all, because a page of memory cannot finish a conversion.
     */
    AK_ADC_ODT(AK_BOARD_VBAT_ADC) = 2000u;
    {
        float volts = ak_board_battery_pin_volts();
        float expected = ak_battery_pin_volts(2000u, AK_BOARD_VBAT_VREF,
                                              AK_BOARD_VBAT_FULL_SCALE);

        expect("a finished conversion is the pin's voltage, the core's way",
               volts > expected - 0.001f && volts < expected + 0.001f);
        said[0] = '\0';
        ak_board_battery_report(sink);
        expect("and the board's report prints the counts, not a timeout",
               strstr(said, "2000 counts") != NULL);

        /* And the other half of that report: a converter that never finishes is
         * a timeout rather than an empty battery, which is a different sentence
         * on the console and a different thing to go and look at. */
        host_f4adc_set_never_finishes(1);
        said[0] = '\0';
        ak_board_battery_report(sink);
        expect("a converter that never finishes is a timeout, not 0 V",
               strstr(said, "no conversion") != NULL &&
                   strstr(said, "empty battery") != NULL);
        expect("and the reading itself says nothing rather than a number",
               ak_board_battery_pin_volts() < 0.0f);
        host_f4adc_set_never_finishes(0);
    }

    expect("this board has no network either, and says so",
           ak_board_net_ready() == 0 && ak_board_net_connected() == 0);
    ak_board_net_start(); /* a board with no radio: it must do nothing at all */

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

    /* --- and the outputs, which make this board an aircraft ---------------- */

    ak_board_output_init();
    expect("the board brings its outputs up and says how many it has",
           ak_board_output_ready() != 0);
    unsigned motors = 0;
    unsigned servos = 0;
    ak_board_output_shape(&motors, &servos);
    expect("two motors and two servos, which is what this board is",
           motors == 2u && servos == 2u);

    /*
     * And the four pads are the *aircraft's* own.
     *
     * This is the one part of the port a host test cannot derive from
     * anything: which pad a servo is on is the airframe's business, and a
     * wrong one is a servo that does not move. The reference is not a
     * datasheet here - it is the board that flies, and its own target in this
     * repository says it in its own words
     * (`upstream/inav-9.1.0/src/main/target/TWINWINGS_GHF435V2/target.c`,
     * the twin-wings target, repaired by hand to fly this wing):
     *
     *   DEF_TIM(TMR4, CH1, PB6, TIM_USE_OUTPUT_AUTO, 0,  0),  // ESC1 -> motor 1
     *   DEF_TIM(TMR4, CH2, PB7, TIM_USE_OUTPUT_AUTO, 0, 10),  // ESC2 -> motor 2
     *   DEF_TIM(TMR2, CH1, PB8, TIM_USE_OUTPUT_AUTO, 0,  1),  // RX5 pad -> servo 1
     *   DEF_TIM(TMR2, CH2, PB9, TIM_USE_OUTPUT_AUTO, 0,  2),  // TX5 pad -> servo 2
     *
     * which is exactly what the board file names - and the mux numbers that
     * follow are this part's own way of saying which timer each pad belongs to
     * (mux 2 is TMR4, mux 1 is TMR2, from the pin-to-timer table the port's
     * `regs.h` cites). The board file's comment says the pins are written in
     * two places "so that the two cannot drift apart silently"; these two
     * checks are what makes that true, because the second one reads the driver
     * rather than the header.
     */
    expect("the motor pads and muxes are the ones the aircraft's target uses",
           AK_BOARD_MOTOR1_PIN.port == GPIOB_BASE &&
               AK_BOARD_MOTOR1_PIN.pin == 6u &&
               AK_BOARD_MOTOR2_PIN.port == GPIOB_BASE &&
               AK_BOARD_MOTOR2_PIN.pin == 7u &&
               ((AK_GPIO_MUXL(GPIOB_BASE) >> 24) & 0xFu) == AK_GPIO_MUX_2 &&
               ((AK_GPIO_MUXL(GPIOB_BASE) >> 28) & 0xFu) == AK_GPIO_MUX_2);
    expect("and so are the servo pads",
           AK_BOARD_SERVO1_PIN.port == GPIOB_BASE &&
               AK_BOARD_SERVO1_PIN.pin == 8u &&
               AK_BOARD_SERVO2_PIN.port == GPIOB_BASE &&
               AK_BOARD_SERVO2_PIN.pin == 9u &&
               ((AK_GPIO_MUXH(GPIOB_BASE) >> 0) & 0xFu) == AK_GPIO_MUX_1 &&
               ((AK_GPIO_MUXH(GPIOB_BASE) >> 4) & 0xFu) == AK_GPIO_MUX_1);
    expect("at a DShot rate the board can state",
           ak_board_output_dshot_hz() >= 150000u &&
               ak_board_output_dshot_hz() <= 600000u);

    ak_board_output_set_rate(600u);
    expect("and setting the rate moves the period it reports",
           ak_board_output_dshot_hz() == 600000u &&
               ak_board_output_ccr_zero() < ak_board_output_ccr_one() &&
               ak_board_output_ccr_one() <= ak_board_output_dshot_period());

    ak_output_frame_t frame;
    memset(&frame, 0, sizeof frame);
    frame.dshot[0] = 1000u;
    unsigned sent_before = ak_board_output_frames_sent();
    ak_board_output_write(&frame);
    expect("and a frame written through the board reaches its outputs",
           ak_board_output_frames_sent() >= sent_before);
    said[0] = '\0';
    ak_board_output_report(sink);
    expect("and the board can say what its outputs are doing",
           strstr(said, "dshot") != 0 || strstr(said, "DShot") != 0);

    /* --- the parts of the board contract that are not about flying -------- */

    unsigned bytes = 0;
    expect("the board offers RAM that survives a reset, for the long log",
           ak_board_retained_ram(&bytes) != 0 && bytes > 0u);

    ak_param_t items[4];
    expect("and it adds no parameters of its own, which is an answer too",
           ak_board_param_table(items, 4u) == 4u);

    /* Both of these end in a reset on the part, and the host's port layer
     * stubs that out - which is exactly why they can be called here: what is
     * checked is that the board reaches the port, not what the part then
     * does. */
    ak_board_reboot();
    expect("the board's reboot hook reaches the port's reset",
           ak_board_enter_bootloader() == 0);

    /*
     * The banner's clock line, which is the first thing to read on a board that
     * looks dead, and the LED's toggle.
     *
     * The crystal here is the AT32's HEXT - the part's own name for the same
     * 8 MHz can - and its ready bit is what the summary is built from. The
     * port's wait for it is bounded and gives up, so taking the bit away is how
     * the failure branch is reached.
     */
    ak_test_at32_clock_up();
    ak_board_init();
    expect("the clock line names the crystal when it came up",
           strstr(ak_board_clock_summary(), "HEXT 8 MHz x PLL") != NULL &&
               ak_board_clock_ok() == 1);

    ak_test_at32_no_crystal();
    ak_board_init();
    expect("and says the crystal failed when it did, not a frequency",
           strstr(ak_board_clock_summary(), "HEXT FAILED") != NULL &&
               ak_board_clock_ok() == 0);
    ak_test_at32_clock_up();
    ak_board_init();

    ak_board_led_set(1);
    ak_board_led_toggle();
    expect("the LED toggles from lit to dark, on the pin",
           ak_board_led_state() == 0 &&
               scr(AK_BOARD_LED_PIN) == (1u << AK_BOARD_LED_PIN.pin));
    ak_board_led_toggle();
    expect("and back", ak_board_led_state() == 1);

    /* The retained region the long log survives a reset in, and the four
     * answers this board gives about a network it does not have. */
    unsigned retained_bytes = 0u;
    expect("the retained log is one region, with a size to go with it",
           ak_board_retained_ram(&retained_bytes) != NULL &&
               retained_bytes >= sizeof(ak_log_t));

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
}
