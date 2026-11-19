// Host test for the dual-role WS2812 output path: the exact 24-bit words the
// RMT driver sends, the bit timing, and a full time sweep proving that no
// pattern can emit any hue other than the role color, red fault or amber
// mismatch. Built with the system C++ compiler by tests/test_led_output.py.

#include "aerialkit_led_state_machine.h"

#include <cstdio>
#include <cstdlib>
#include <initializer_list>
#include <utility>

namespace
{
int failures;

void check(bool condition, const char *what, unsigned a = 0, unsigned b = 0)
{
    if (!condition)
    {
        std::printf("FAIL %s (%u, %u)\n", what, a, b);
        ++failures;
    }
}

constexpr uint8_t BRIGHTNESS = 26;

constexpr AerialKitLedPattern ROLE_PATTERNS[] = {
    AerialKitLedPattern::BootIdentity,
    AerialKitLedPattern::SlowBreath,
    AerialKitLedPattern::FastBreath,
    AerialKitLedPattern::DoublePulse,
    AerialKitLedPattern::Steady,
    AerialKitLedPattern::SinglePulse,
};

void test_role_words()
{
    const AerialKitLedRgb blue = aerialkit_role_color(false);
    const AerialKitLedRgb green = aerialkit_role_color(true);
    check(aerialkit_ws2812_grb_word(blue, BRIGHTNESS) == 0x00001A, "TX blue word",
          aerialkit_ws2812_grb_word(blue, BRIGHTNESS));
    check(aerialkit_ws2812_grb_word(green, BRIGHTNESS) == 0x1A0000, "RX green word",
          aerialkit_ws2812_grb_word(green, BRIGHTNESS));
    check(aerialkit_ws2812_grb_word({255, 0, 0}, BRIGHTNESS) == 0x001A00, "red word");
    check(aerialkit_ws2812_grb_word({0, 0, 0}, BRIGHTNESS) == 0, "off word");
}

void test_bit_timing()
{
    // WS2812B-2020 V6 limits, in nanoseconds.
    constexpr unsigned NS = AERIALKIT_WS2812_TICK_NS;
    check(NS * AERIALKIT_WS2812_RMT_CLK_DIV == 50 && 80000000u / AERIALKIT_WS2812_RMT_CLK_DIV == 40000000u,
          "25 ns tick from 80 MHz APB / 2");
    auto within = [](unsigned value, unsigned low, unsigned high) { return value >= low && value <= high; };
    check(within(AERIALKIT_WS2812_T0H_TICKS * NS, 220, 380), "T0H", AERIALKIT_WS2812_T0H_TICKS * NS);
    check(within(AERIALKIT_WS2812_T0L_TICKS * NS, 580, 1000), "T0L", AERIALKIT_WS2812_T0L_TICKS * NS);
    check(within(AERIALKIT_WS2812_T1H_TICKS * NS, 580, 1000), "T1H", AERIALKIT_WS2812_T1H_TICKS * NS);
    check(within(AERIALKIT_WS2812_T1L_TICKS * NS, 580, 1000), "T1L", AERIALKIT_WS2812_T1L_TICKS * NS);
    check((AERIALKIT_WS2812_T0H_TICKS + AERIALKIT_WS2812_T0L_TICKS) * NS >= 1250, "0-bit period");
    check((AERIALKIT_WS2812_T1H_TICKS + AERIALKIT_WS2812_T1L_TICKS) * NS >= 1250, "1-bit period");
    check(AERIALKIT_WS2812_RESET_HALF_TICKS < 32768, "RMT 15-bit duration");

    // 0x800001: first and last data bits are 1, the rest 0; item 24 latches.
    const uint32_t word = 0x800001;
    check(AERIALKIT_WS2812_FRAME_ITEMS == 25, "24 data items + latch");
    for (uint8_t index = 0; index < 24; ++index)
    {
        const AerialKitRmtItem item = aerialkit_ws2812_item(word, index);
        const bool one = index == 0 || index == 23;
        check(item.level0 == 1 && item.level1 == 0, "high then low", index);
        check(item.duration0 == (one ? AERIALKIT_WS2812_T1H_TICKS : AERIALKIT_WS2812_T0H_TICKS),
              "high ticks", index, item.duration0);
        check(item.duration1 == (one ? AERIALKIT_WS2812_T1L_TICKS : AERIALKIT_WS2812_T0L_TICKS),
              "low ticks", index, item.duration1);
        check(item.duration0 != 0 && item.duration1 != 0, "no premature RMT end marker", index);
    }
    const AerialKitRmtItem latch = aerialkit_ws2812_item(word, 24);
    check(latch.level0 == 0 && latch.level1 == 0, "latch is low");
    check((latch.duration0 + latch.duration1) * NS >= 280000u, "latch > 280 us",
          (latch.duration0 + latch.duration1) * NS);

    // Exact waveform of the RX green word 0x1A0000: G=00011010, then 16 zeros.
    const uint32_t green = aerialkit_ws2812_grb_word(aerialkit_role_color(true), BRIGHTNESS);
    const char *expected = "000110100000000000000000";
    for (uint8_t index = 0; index < 24; ++index)
    {
        const bool one = aerialkit_ws2812_item(green, index).duration0 == AERIALKIT_WS2812_T1H_TICKS;
        check(one == (expected[index] == '1'), "green waveform bit", index);
    }
}

void test_selftest()
{
    AerialKitLedRgb color {};
    check(aerialkit_led_selftest_color(0, color) && color.red == 255 && color.green == 0 && color.blue == 0, "selftest red");
    check(aerialkit_led_selftest_color(1500, color) && color.red == 0 && color.green == 255 && color.blue == 0, "selftest green");
    check(aerialkit_led_selftest_color(2999, color) && color.red == 0 && color.green == 0 && color.blue == 255, "selftest blue");
    check(!aerialkit_led_selftest_color(3000, color), "selftest ends after 3 s");
    check(aerialkit_ws2812_grb_word({255, 0, 0}, BRIGHTNESS) == 0x001A00 &&
          aerialkit_ws2812_grb_word({0, 255, 0}, BRIGHTNESS) == 0x1A0000 &&
          aerialkit_ws2812_grb_word({0, 0, 255}, BRIGHTNESS) == 0x00001A,
          "selftest words are single-channel");

    const uint8_t tx[] = {0xC8, 8, 0x7F, 0xEE, 0xEA, 'A', 'K', 'L', 'T', 0};
    check(aerialkit_led_selftest_frame(tx, 0xEE), "TX selftest frame accepted");
    check(!aerialkit_led_selftest_frame(tx, 0xEC), "TX frame ignored by RX");
    const uint8_t wrong[] = {0xC8, 8, 0x7F, 0xEE, 0xEA, 'A', 'K', 'L', 'X', 0};
    check(!aerialkit_led_selftest_frame(wrong, 0xEE), "wrong magic ignored");
}

void test_no_unrelated_hue()
{
    for (const bool rx : {false, true})
    {
        for (const AerialKitLedPattern pattern : ROLE_PATTERNS)
        {
            bool sawOn = false;
            for (uint32_t ms = 0; ms < 12000; ++ms)
            {
                const AerialKitLedRgb color = aerialkit_led_frame(pattern, ms, rx);
                const uint32_t word = aerialkit_ws2812_grb_word(color, BRIGHTNESS);
                const uint32_t allowed = rx ? 0xFF0000 : 0x0000FF;
                check((word & ~allowed) == 0, "non-role channel lit",
                      static_cast<unsigned>(pattern), ms);
                check(word <= allowed && (word & allowed) <= (rx ? 0x1A0000u : 0x1Au),
                      "brightness cap", static_cast<unsigned>(pattern), ms);
                sawOn = sawOn || word != 0;
            }
            check(sawOn, "pattern never lights", static_cast<unsigned>(pattern), rx);
        }

        for (uint32_t ms = 0; ms < 3000; ++ms)
        {
            const uint32_t red = aerialkit_ws2812_grb_word(
                aerialkit_led_frame(AerialKitLedPattern::RedBlink, ms, rx), BRIGHTNESS);
            check(red == 0 || red == 0x001A00, "fault is pure red", ms, red);
            const uint32_t amber = aerialkit_ws2812_grb_word(
                aerialkit_led_frame(AerialKitLedPattern::AmberTriple, ms, rx), BRIGHTNESS);
            check(amber == 0 || amber == 0x0C1A00, "mismatch is fixed amber", ms, amber);
        }
    }
}

void test_boot_pulse_counts()
{
    for (const bool rx : {false, true})
    {
        unsigned rising = 0;
        bool previous = false;
        const uint32_t cycle = AERIALKIT_LED_BOOT_PULSE_MS + AERIALKIT_LED_BOOT_GAP_MS;
        const uint32_t window = rx ? 2 * cycle : cycle;
        for (uint32_t ms = 0; ms < window; ++ms)
        {
            const bool on = aerialkit_led_on(AerialKitLedPattern::BootIdentity, ms, rx);
            rising += on && !previous;
            previous = on;
        }
        check(rising == (rx ? 2u : 1u), "boot pulse count", rx, rising);
    }
}

void test_breathing_periods()
{
    // Slow (bind) breathing repeats every 3 s, fast (searching) every 1 s,
    // and both reach the full raw level once per period.
    for (const auto &spec : {std::pair<AerialKitLedPattern, uint32_t>{AerialKitLedPattern::SlowBreath, 3000},
                             std::pair<AerialKitLedPattern, uint32_t>{AerialKitLedPattern::FastBreath, 1000}})
    {
        unsigned peaks = 0;
        for (uint32_t ms = 0; ms < 6000; ++ms)
        {
            const uint8_t level = aerialkit_led_master_level(spec.first, ms);
            check(aerialkit_led_master_level(spec.first, ms + spec.second) == level,
                  "breathing period", spec.second, ms);
            peaks += level == AERIALKIT_LED_RAW_MAX &&
                     aerialkit_led_master_level(spec.first, ms + 1) < AERIALKIT_LED_RAW_MAX;
        }
        check(peaks == 6000 / spec.second, "breathing peaks", spec.second, peaks);
    }
    for (uint32_t ms = 0; ms < 5000; ms += 7)
    {
        check(aerialkit_led_master_level(AerialKitLedPattern::Steady, ms) == AERIALKIT_LED_RAW_MAX,
              "connected is steady", ms);
    }
}
} // namespace

int main()
{
    test_role_words();
    test_bit_timing();
    test_selftest();
    test_no_unrelated_hue();
    test_boot_pulse_counts();
    test_breathing_periods();
    if (failures == 0)
    {
        std::printf("OK\n");
    }
    return failures == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}
