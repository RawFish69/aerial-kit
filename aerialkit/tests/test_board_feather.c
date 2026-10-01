/*
 * The Adafruit Feather F405's board file, the way the WeAct board's and the
 * wing's are checked in tests/test_board_f405.c and tests/test_board_ghf435.c -
 * and for the same reason: every simulated session runs the *simulator's* board,
 * so the layer that decides which USART the console is on, which pads the servo
 * pulses come out of and what "no divider fitted" reads as had never been
 * executed by anything on this machine. It was compile-checked, boards-checked,
 * and it had never run.
 *
 * **This is the fourth board file and the second on this part**, which makes it
 * a test of a different kind rather than a copy. The arch layer is the same one
 * the WeAct board runs - both are STM32F405s - so every difference asserted
 * below is a difference the *header* makes and not the silicon:
 *
 *   the console     USART6 on APB2      against USART2 on APB1
 *   the status pin  PC1, active high    against PC13, active low
 *   the IMU bus     I2C1 at 0x6B        against SPI1 with a chip select
 *   the servos      TIM4 CH3/CH4 PB8/9  against TIM2 CH1/CH2 PA0/PA1
 *   the shape       two motors          against four
 *
 * A board file copied from the other one gets all five wrong, and the two
 * silicon-identical boards share one set of arch objects, so the only thing that
 * can catch a copy is a test that reads this board's own facts back.
 *
 * The sixth thing this test reads is not a difference between the boards, and it
 * is the one the file had wrong: its banner's fallback printed the WeAct
 * header's *assumed* 8 MHz, inherited with the rest of the file, while this
 * board's own header declared 12. At 8 MHz the PLL asks its VCO for 504 MHz,
 * past the part's 432 MHz limit, which is why the WeAct board never enumerated
 * in the first place. Neither file said 12 in the sense of having measured one,
 * each believed the other was the 8 MHz board, and each board's test pinned its
 * own file's number, which is a ring only an outside fact can break.
 *
 * **This paragraph then said "both crystals are 12 MHz", and that sentence is
 * retracted (traps 212, 213).** It was written on the strength of the two files
 * agreeing, and the board that was actually measured - the WeAct - turned out to
 * be 8 MHz, with the 11.95 MHz reading that had produced the 12 taken on a
 * different F405 entirely. So the ring was broken from outside, and the answer
 * was the opposite of this paragraph's: the WeAct's header, banner fallback and
 * test went back to 8, and this file's 12 is left standing on its own basis -
 * Adafruit's variant file, quoted in `board.h` - rather than on the agreement
 * this paragraph used to rest on. The 12 has still never been measured on this
 * board, and the test below pins the *fallback* and says so.
 *
 * Three assertions were wrong in the first draft of this file, and all three
 * were about the *host* rather than the board - traps 204 names them. That is
 * the other half of the hazard: a test can be right about the board and wrong
 * about the machine it runs on, and a check written for the wrong board passes
 * quietly.
 *
 * The entry points are renamed for this build (`ak_feather_*`, which is what
 * `board.h`'s `#ifdef AK_HOST_FEATHER` is for), so the source below says the
 * plain names the core uses and the preprocessor resolves them to this board.
 */

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "arch.h"
#include "board.h"
#include "regs.h"

#include "ak_battery.h"
#include "ak_board.h"
#include "ak_bus.h"
#include "ak_log.h"
#include "ak_mixer.h"
#include "ak_rc.h"
#include "ak_rc_receiver.h"
#include "host_i2c_model.h"
#include "tests.h"

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
 * the port configures, rounded to the nearest, and which APB bus the port is on
 * is the whole of the arithmetic. */
static uint32_t divisor(uint32_t clock_hz, uint32_t baud)
{
    return (clock_hz + baud / 2u) / baud;
}

void test_board_feather(void)
{
    expect("the register pages are mapped for the board's checks",
           ak_test_map_registers());
    if (!ak_test_registers_mapped()) {
        return; /* every write below would land on an unmapped page */
    }
    /* The part a few hundred microseconds in: crystal up, PLL locked. Without
     * it the boot falls back to the internal oscillator and every rate derived
     * from the clock is a different number that these checks would still agree
     * with themselves about. */
    ak_test_f405_crystal(1);

    /* --- the boot sequence itself --------------------------------------- */

    ak_board_init(); /* the target's own first step, in the same order */
    /* The name's first half is `AK_BOARD`, which the host build sets to `host`
     * rather than to the board - so what this can check is the half the board
     * file writes itself, and that it is not the other board's. */
    expect("the board comes up and names this breakout, not the WeAct one",
           ak_board_name() != 0 &&
               strstr(ak_board_name(), "Adafruit Feather STM32F405") != 0 &&
               strstr(ak_board_name(), "WeAct") == 0);

    /* --- and the aircraft it is built into -------------------------------- */

    /*
     * The seventh difference between this board file and the WeAct one, and the
     * one that is not a pin: what the `airframe` parameter defaults to.
     *
     * The core's own default is a quad-X, which needs four motors, and this
     * breakout brings out two motor pads and two servo pads - so a board that
     * took the core's default could never pass the arming fit check, and its
     * `preflight` said so with both numbers in it. That refusal is in
     * src/core/main.c and is not this file's to weaken; what this file supplies
     * is the *number*, so the check has something to pass against.
     *
     * Asserted here rather than in test_mix.c because 7 is a fact about the
     * header: the same number on the WeAct board - which brings out four motor
     * pads - would be wrong, and that board answers 0. A board file copied from
     * the other one gets this wrong in the direction that refuses to arm, which
     * is a board that looks alive and never spins a motor.
     */
    const ak_mixer_t *board_mix =
        ak_mixer_for_airframe(ak_board_default_airframe());
    expect("the board names the single-motor elevon wing as its own airframe",
           ak_board_default_airframe() == 7u && board_mix != 0 &&
               strcmp(board_mix->name, "elevon-wing-single") == 0);

    /* Counted out of the mix rather than written down twice: this is the same
     * arithmetic ak_flight_apply_airframe() does, and the same comparison the
     * fit check makes. A test that hardcoded "1 and 2" would still pass if the
     * table changed under it. */
    unsigned mix_motors = 0u;
    unsigned mix_servos = 0u;
    unsigned shape_motors = 0u;
    unsigned shape_servos = 0u;

    for (unsigned i = 0; i < board_mix->count; i++) {
        if (board_mix->kind[i] == AK_OUT_SERVO) {
            mix_servos++;
        } else {
            mix_motors++;
        }
    }
    ak_board_output_shape(&shape_motors, &shape_servos);
    expect("and the mix it names is one these pads can drive",
           mix_motors == 1u && mix_servos == 2u &&
               shape_motors >= mix_motors && shape_servos >= mix_servos);

    /* And the core's default, which is what a board that said nothing would
     * get: this is the failure the entry point exists to prevent, so the test
     * states it rather than leaving the number 7 looking arbitrary. */
    expect("while the core's default is the mix this board cannot drive",
           ak_mixer_for_airframe(0u)->count == 4u && shape_motors < 4u);

    const uint32_t apb1 = ak_clk_apb1_hz();
    const uint32_t apb2 = ak_clk_apb2_hz();
    expect("the port has a clock to divide, whatever it ended up as",
           apb1 > 0u && apb2 > 0u);
    expect("and the board's own getters agree with the port's",
           ak_board_clock_apb1_hz() == apb1 &&
               ak_board_clock_sysclk_hz() > 0u);

    /* The console is USART6, which is on **APB2** - the WeAct board's USART2 is
     * on APB1 - so this one check is what a copied board file fails: the right
     * port with the wrong divisor is a console nobody can read. */
    expect("the console is on the port and the rate the board names",
           AK_BOARD_CONSOLE_USART == USART6_BASE &&
               ak_board_console_port() == AK_BOARD_CONSOLE_USART &&
               ak_board_console_attached_port() == AK_BOARD_CONSOLE_USART);
    expect("and its divisor is the one the second APB bus needs",
           USART_BRR(AK_BOARD_CONSOLE_USART) ==
               divisor(apb2, AK_BOARD_CONSOLE_BAUD));

    /* The LED: PC1 and active *high*, the opposite of the WeAct board's PC13.
     * The polarity is asserted rather than parameterised, because it is one of
     * the facts that differ. */
    expect("the status pin is PC1 and active high",
           AK_BOARD_LED_ACTIVE_LOW == 0 &&
               AK_BOARD_LED_PIN.port == GPIOC_BASE &&
               AK_BOARD_LED_PIN.pin == 1u);
    ak_board_led_set(1);
    expect("so lit means the pin is driven, not released",
           GPIO_BSRR(AK_BOARD_LED_PIN.port) == (1u << AK_BOARD_LED_PIN.pin) &&
               ak_board_led_state() == 1);
    ak_board_led_set(0);
    expect("and dark is the other half of the register",
           GPIO_BSRR(AK_BOARD_LED_PIN.port) ==
               (1u << (AK_BOARD_LED_PIN.pin + 16u)) &&
               ak_board_led_state() == 0);
    ak_board_led_set(1);
    ak_board_led_toggle();
    expect("and a heartbeat toggles the state and the pin together",
           ak_board_led_state() == 0 &&
               GPIO_BSRR(AK_BOARD_LED_PIN.port) ==
                   (1u << (AK_BOARD_LED_PIN.pin + 16u)));

    /* --- the receiver, and the UART this board does not have ------------- */

    ak_board_rc_init();
    expect("the receiver's port is the one the board claims",
           AK_BOARD_RC_USART == USART3_BASE &&
               USART_BRR(AK_BOARD_RC_USART) == divisor(apb1, AK_BOARD_RC_BAUD));
    /* APB1 for the receiver and APB2 for the console, on one board: getting
     * these the other way round is two ports that both look configured. */
    expect("and it is a receiver port on the other APB bus from the console",
           AK_BOARD_RC_USART != AK_BOARD_CONSOLE_USART);
    expect("with the inverter fact the board was built with",
           ak_board_rc_inverted() == AK_BOARD_RC_INVERTER);

    /* Bytes in through the port's own interrupt: USART3 is the one this board
     * gives the receiver, where the WeAct board gives it USART1. */
    for (int i = 0; i < 4; i++) {
        USART_SR(AK_BOARD_RC_USART) = USART_SR_RXNE;
        USART_DR(AK_BOARD_RC_USART) = (uint32_t)('a' + i);
        USART3_IRQHandler();
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

    /* A ring that overflows counts what it dropped rather than pretending. */
    for (int i = 0; i < 400; i++) {
        USART_SR(AK_BOARD_RC_USART) = USART_SR_RXNE;
        USART_DR(AK_BOARD_RC_USART) = (uint32_t)(i & 0xFF);
        USART3_IRQHandler();
    }
    expect("an overflowed ring counts its drops", ak_board_rc_dropped() > 0u);

    USART_SR(AK_BOARD_RC_USART) = USART_SR_TXE;
    expect("the board can send telemetry back up the receiver's wire",
           ak_board_rc_send("t", 1u) == 0 &&
               (USART_DR(AK_BOARD_RC_USART) & 0xFFu) == 't');

    /* Switching to SBUS changes the port's line settings, not just the parser:
     * 100000 baud, eight bits, even parity, two stop bits. */
    ak_board_rc_set_protocol(AK_RC_PROTOCOL_SBUS);
    expect("switching to SBUS changes the port's rate and its frame",
           USART_BRR(AK_BOARD_RC_USART) ==
               divisor(apb1, AK_BOARD_RC_SBUS_BAUD) &&
               (USART_CR1(AK_BOARD_RC_USART) & USART_CR1_PCE) != 0u &&
               (USART_CR2(AK_BOARD_RC_USART) & 0x3000u) == 0x2000u);
    ak_board_rc_set_protocol(AK_RC_PROTOCOL_CRSF);
    expect("and switching back puts the rate back",
           USART_BRR(AK_BOARD_RC_USART) == divisor(apb1, AK_BOARD_RC_BAUD));

    /*
     * The GPS is answered with no-ops rather than by a port, because this
     * breakout has two usable UARTs and both are spoken for. What matters is
     * that the contract is *complete*: a function that fell off the end of a
     * non-void body answers with garbage, and this file's own header used to
     * record exactly that about itself.
     */
    ak_board_gps_init();
    expect("this board has no GPS, and answers every way it is asked",
           ak_board_gps_poll(&nothing) == 0 && ak_board_gps_dropped() == 0u &&
               ak_board_gps_send("g", 1u) == 0);

    /* --- the IMU: the bus this port exists for --------------------------- */

    const ak_bus_t *imu = ak_board_imu_bus();
    expect("the IMU has a bus, with the ways of using it that it has",
           imu != 0 && imu->read != 0 && imu->write != 0 &&
               imu->delay_ms != 0);

    /* The modelled bus is process-global and the tests before this one have
     * been on it, so it is cleared *before* the board brings its own port up:
     * the reset clears the peripheral's registers with the slaves, and doing it
     * afterwards would take this port's own configuration with it. This is the
     * order the modelled bus has to be used in and the first version of this
     * file had it the other way round. */
    host_i2c_model_reset();
    ak_board_imu_init();
    /* CR1 is read through the model, not from the page: with `-DAK_HOST_I2C`
     * the driver's register writes go into the model and the mapped page stays
     * zero, so reading the page would say the port is down when it is up. */
    expect("which brought up I2C1 on the Qwiic pads, at AF4",
           AK_BOARD_IMU_I2C == I2C1_BASE &&
               (host_i2c_read(AK_BOARD_IMU_I2C, I2C_OFF_CR1) & I2C_CR1_PE) !=
                   0u &&
               AK_BOARD_IMU_SCL.port == GPIOB_BASE &&
               AK_BOARD_IMU_SCL.pin == 6u &&
               AK_BOARD_IMU_SDA.port == GPIOB_BASE &&
               AK_BOARD_IMU_SDA.pin == 7u &&
               ((GPIO_MODER(GPIOB_BASE) >> (6u * 2u)) & 0x3u) == GPIO_MODE_AF &&
               ((GPIO_MODER(GPIOB_BASE) >> (7u * 2u)) & 0x3u) == GPIO_MODE_AF &&
               ((GPIO_AFRL(GPIOB_BASE) >> 24) & 0xFFu) == 0x44u);
    /* And no burst, which is not a gap: an LSM6DSO has no configuration upload
     * to shift at speed, and a null burst means "one byte at a time" to
     * ak_bus.h. It is also the one slot the WeAct board's SPI bus fills and
     * this one does not, so a copy of that file is caught here. */
    expect("an I2C part with nothing to upload says so rather than pretending",
           imu != 0 && imu->write_burst == 0);

    /*
     * And a transaction, against a part attached to the modelled bus - the
     * end-to-end half, and the only check here that could catch a board whose
     * address and whose bus were not the same peripheral. The model answers the
     * transaction sequences a page of memory cannot: the start, the address
     * byte, the register number, the repeated start, the byte.
     */
    {
        /* An LSM6DSO's who-am-i is 0x6C at register 0x0F, which is the value
         * the real driver's probe reads. */
        uint8_t regs[HOST_I2C_REGISTERS];
        memset(regs, 0, sizeof regs);
        regs[0x0F] = 0x6Cu;

        /* And the address itself, as a literal, because it is a fact about the
         * breakout in hand rather than a round trip through the header.
         *
         * It was 0x6A here until 2026-09-30, and 0x6A in the header, and the
         * two agreed and were both wrong: a scan of the real I2C1 found one
         * answering address, 0x6B - the same LSM6DSO, strapped the other way.
         * The header agreed with this test and neither agreed with the part,
         * which is the failure mode a test that only round-trips a constant
         * cannot see. So the number is written out, and the sentence a person
         * gets when it changes says which of the two straps is fitted. */
        int slave = host_i2c_attach((uint8_t)AK_BOARD_IMU_ADDRESS, regs,
                                    (unsigned)sizeof regs);
        expect("this board's address is the one its LSM6DSO breakout answers "
               "at (0x6B, SA0 high; measured 2026-09-30)",
               slave >= 0 && AK_BOARD_IMU_ADDRESS == 0x6Bu);
        if (slave >= 0 && imu != 0) {
            uint8_t who = 0u;
            /* `host_i2c_last_register()` is the slave's pointer *after* the
             * transfer, not the number the master sent: a part's pointer
             * auto-increments, so a one-byte read of 0x0F leaves it at 0x10.
             * That is worth asserting rather than working around - it says the
             * read started at 0x0F and moved by exactly one, which is the whole
             * of what a register read of a sensor is. (The arch test reads
             * three bytes from 0x04 and gets 0x07 for the same reason.) */
            expect("and the bus reaches it at the address the board states",
                   imu->read(imu->ctx, 0x0F, &who, 1) == 0 && who == 0x6Cu &&
                       host_i2c_last_address() ==
                           (uint8_t)AK_BOARD_IMU_ADDRESS &&
                       host_i2c_last_register() == 0x10u &&
                       host_i2c_addressed((unsigned)slave) != 0);
            expect("and a write lands in the part, not beside it",
                   imu->write(imu->ctx, 0x10, 0x60u) == 0 &&
                       host_i2c_register((unsigned)slave, 0x10) == 0x60u);
            imu->delay_ms(imu->ctx, 0u); /* a delay of nothing, so it returns */
        }
    }

    /* --- the two parts this board does not carry ------------------------- */

    expect("with nothing fitted, there is no barometer bus",
           ak_board_baro_bus() == 0);
    expect("and no rangefinder bus", ak_board_range_bus() == 0);

    ak_board_battery_init();
    expect("and the pack divider is not a number this board can state",
           AK_BOARD_VBAT_FITTED == 0 && ak_board_battery_ready() == 0);
    expect("which the core reads as no reading, not as zero volts",
           ak_board_battery_pin_volts() < 0.0f);

    /*
     * The report, which is the piece of this file a person reads at a bench -
     * and the piece that was lying. It printed the WeAct board's pad, divider
     * and *filename*: "PC0, ADC1 input 3" is a pad and a channel that cannot
     * both be true on a board whose divider is on PA3, and the sentence below it
     * said "10k over 1k" about a divider nobody has measured on this breakout.
     * Both are checked by name now, and the pad in the report is built from the
     * header rather than typed.
     */
    said[0] = '\0';
    ak_board_battery_report(sink);
    expect("the pack's report names this board's pad, and not the other board's",
           strstr(said, "A3, ADC1 input 3") != NULL &&
               strstr(said, "PC0") == NULL);
    expect("and does not claim a divider ratio it has not measured",
           strstr(said, "10k") == NULL && strstr(said, "times 11") == NULL &&
               strstr(said, "unmeasured") != NULL);
    expect("and points at this board's own header when it says what to change",
           strstr(said, "FEATHER_F405") != NULL &&
               strstr(said, "AERIALKIT_F405") == NULL);

    /* --- the outputs, which are the reason this board's shape is 2 and 2 -- */

    ak_board_output_init();
    expect("the board brings its outputs up", ak_board_output_ready() != 0);
    unsigned motors = 0;
    unsigned servos = 0;
    ak_board_output_shape(&motors, &servos);
    /* Not the timer's four: only two of TIM3's channels reach this header, so
     * only two motors can be plugged in. This is also what makes a wing's mixer
     * fit this board and a quadrotor's not. */
    expect("two motors and two servos, which is what can be plugged in",
           motors == 2u && servos == 2u);

    /* And the register side of the servo claim, which is the part a header
     * cannot state: the pads and the channels the *board* hands over. TIM4's
     * free pair is channels 3 and 4, so the compare registers are CCR3 and
     * CCR4; the arch's own PA0/PA1 pair must still be untouched, because a board
     * that fell back to the arch layer's choice would drive two pulses into
     * pads this breakout does not bring out - which is what it did until
     * 2026-09-29. */
    expect("and its servos came out of the timer and pads it declares",
           strcmp(ak_output_servo_timer_name(), "TIM4") == 0 &&
               AK_BOARD_SERVO_TIMER == TIM4_BASE &&
               AK_BOARD_SERVO1_PIN.port == GPIOB_BASE &&
               AK_BOARD_SERVO1_PIN.pin == 8u &&
               AK_BOARD_SERVO2_PIN.port == GPIOB_BASE &&
               AK_BOARD_SERVO2_PIN.pin == 9u);
    expect("with the second channel pair's compare registers centred",
           TIM_CCR3(TIM4_BASE) == 1500u && TIM_CCR4(TIM4_BASE) == 1500u &&
               TIM_CCR1(TIM4_BASE) == 0u && TIM_CCR2(TIM4_BASE) == 0u);
    /* Channels 3 and 4 live in CCMR2, whose halves are at bit 4 and bit 12. */
    expect("both of them in pwm mode 1, in the second compare register",
           ((TIM_CCMR2(TIM4_BASE) >> 4) & 0x7u) == TIM_CCMR_OCM_PWM1 &&
               ((TIM_CCMR2(TIM4_BASE) >> 12) & 0x7u) == TIM_CCMR_OCM_PWM1);
    expect("and enabled, with the counter running and its gate on",
           (TIM_CCER(TIM4_BASE) & (TIM_CCER_CCE(3) | TIM_CCER_CCE(4))) ==
               (TIM_CCER_CCE(3) | TIM_CCER_CCE(4)) &&
               (TIM_CR1(TIM4_BASE) & TIM_CR1_CEN) != 0u &&
               (RCC_APB1ENR & RCC_APB1ENR_TIM4EN) != 0u);
    expect("and the pads are on the second timer's alternate function",
           ((GPIO_MODER(GPIOB_BASE) >> (8u * 2u)) & 0x3u) == GPIO_MODE_AF &&
               ((GPIO_MODER(GPIOB_BASE) >> (9u * 2u)) & 0x3u) == GPIO_MODE_AF &&
               ((GPIO_AFRH(GPIOB_BASE) >> 0) & 0xFu) == 2u &&
               ((GPIO_AFRH(GPIOB_BASE) >> 4) & 0xFu) == 2u);
    expect("and the other timer's pads were not configured by this bank",
           ((GPIO_MODER(GPIOA_BASE) >> (0u * 2u)) & 0x3u) ==
                   GPIO_MODE_INPUT &&
               ((GPIO_MODER(GPIOA_BASE) >> (1u * 2u)) & 0x3u) ==
                   GPIO_MODE_INPUT);

    expect("at a DShot rate it can state, with a period to match",
           ak_board_output_dshot_hz() >= 150000u &&
               ak_board_output_dshot_hz() <= 600000u &&
               ak_board_output_dshot_period() > 0u);
    ak_board_output_set_rate(600u);
    /* The two compare values are a zero bit and a one bit inside the period, in
     * that order, which is the whole of DShot at this level. */
    expect("and setting the rate moves the period it reports",
           ak_board_output_dshot_hz() == 600000u &&
               ak_board_output_ccr_zero() < ak_board_output_ccr_one() &&
               ak_board_output_ccr_one() <= ak_board_output_dshot_period());

    /*
     * A frame through the board's own wrapper, and it has to land in the compare
     * registers the board named rather than the arch's.
     *
     * The frame starts a motor burst as well - one port, one write path - and
     * the burst has to be ended through the controller's own interrupt, the way
     * every frame on hardware is. The test before this one in the group leaves a
     * burst running, so it is ended *here* first as well: otherwise this board's
     * frame would be dropped as one that arrived while the dma was busy, the
     * compare registers would still read right, and the checks below would pass
     * for a reason that is not this board's.
     */
    if (ak_output_busy() != 0) {
        DMA1_Stream4_IRQHandler();
    }

    ak_output_frame_t frame;
    memset(&frame, 0, sizeof frame);
    frame.servo_us[0] = 1100u;
    frame.servo_us[1] = 1900u;

    uint32_t sent_before = ak_board_output_frames_sent();
    uint32_t skipped_before = ak_output_frames_skipped();
    ak_board_output_write(&frame);
    expect("and a frame's pulses land in the channels this board named",
           TIM_CCR3(TIM4_BASE) == 1100u && TIM_CCR4(TIM4_BASE) == 1900u &&
               TIM_CCR1(TIM4_BASE) == 0u && TIM_CCR2(TIM4_BASE) == 0u &&
               ak_output_frames_skipped() == skipped_before &&
               ak_board_output_frames_sent() == sent_before);

    DMA1_Stream4_IRQHandler();
    expect("which completes it and counts it once",
           ak_output_busy() == 0 &&
               ak_board_output_frames_sent() == sent_before + 1u);

    said[0] = '\0';
    ak_board_output_report(sink);
    expect("and the report names the timer the pulses come out of",
           strstr(said, "TIM4") != NULL && strstr(said, "servos:") != NULL);

    /* --- the rest of what a person at a bench reaches for ---------------- */

    /*
     * The banner's clock line, which is the first thing to read on a board that
     * looks dead: a board on its internal oscillator is alive and fails in ways
     * that read as firmware bugs. It printed "HSE 8 MHz x PLL" until this test
     * existed - the WeAct header's assumption, inherited with the rest of the
     * file's banner fallback - while this board's own header declared 12.
     *
     * This comment used to say "both crystals are 12 MHz", and that sentence is
     * retracted: it was written on the strength of two files agreeing with each
     * other, and the board that was actually measured turned out to be 8 MHz
     * (traps 212, 213). What this test can reach is narrower than either
     * version claimed. The port cannot measure a crystal in a process - there
     * is no TIM11 capture to fire - so `measure_hse()` finds nothing, returns
     * 0, and the line prints the header's macro with `, assumed` after it. That
     * fallback is what is pinned below, suffix and all, and it is not a
     * measurement of this board: this board's crystal has never been measured.
     * The 12 rests on Adafruit's variant file, which board.h says and this test
     * cannot check.
     */
    ak_test_f405_crystal(1);
    ak_board_init();
    /* The crystal is named rather than the whole line searched for "8 MHz":
     * the sysclk field reads "168 MHz", which contains it. The suffix is part
     * of the assertion, so a change that made the fallback print
     * indistinguishably from a measurement would fail here. */
    expect("the clock line names the header's crystal, and says it is assumed",
           strstr(ak_board_clock_summary(), "HSE 12 MHz x PLL, assumed") != NULL &&
               strstr(ak_board_clock_summary(), "HSE 8 MHz") == NULL &&
               ak_board_clock_ok() == 1);

    ak_test_f405_crystal(0);
    ak_board_init();
    expect("and says the crystal failed when it did, not a frequency",
           strstr(ak_board_clock_summary(), "HSE FAILED") != NULL &&
               ak_board_clock_ok() == 0);
    ak_test_f405_crystal(1);
    ak_board_init();

    /* This board has no SPI bus at all - its IMU is on I2C1 - so the loopback
     * diagnostic is answered rather than stubbed, and the answer must not be a
     * loopback result: a person reading "loopback matches" on a board with no
     * SPI would go and look for a jumper that is not there. */
    said[0] = '\0';
    ak_board_spi_loopback(sink);
    expect("the SPI diagnostic says there is no SPI, rather than testing one",
           strstr(said, "no SPI bus") != NULL && strstr(said, "loopback") == 0);

    /* The region the long log lives in: plain RAM marked not to be cleared by
     * the startup code, which is what makes it survive a reset. */
    unsigned retained_bytes = 0;
    void *retained = ak_board_retained_ram(&retained_bytes);
    void *retained_again = ak_board_retained_ram(&retained_bytes);
    expect("the retained log is one region, with a size to go with it",
           retained != NULL && retained == retained_again &&
               retained_bytes >= sizeof(ak_log_t));

    /* The two ways out of the firmware, which in this build are the port's
     * no-ops: what the board's half can be asked is that it is wired to the port
     * and returns rather than doing something of its own. */
    ak_board_reboot();
    expect("the board's reboot goes to the port's, and comes back here",
           ak_board_clock_apb1_hz() > 0u);
    expect("and so does entering the ROM bootloader",
           ak_board_enter_bootloader() == 0);

    /* And the network this board does not have: four answers, and none of them
     * may be "connected". The core opens no protocol on a board that says this,
     * which is the whole reason they exist. */
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
