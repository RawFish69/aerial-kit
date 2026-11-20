#pragma once

#include <stdint.h>

enum class AerialKitLedPattern : uint8_t
{
    BootIdentity,
    SlowBreath,
    FastBreath,
    DoublePulse,
    Steady,
    SinglePulse,
    RedBlink,
    AmberTriple,
};

enum class AerialKitLedState : uint8_t
{
    RadioFault,
    RoleOrModelMismatch,
    BootIdentity,
    Bind,
    HostPresentDisconnected,
    Searching,
    NoSerialInput,
    Connected,
};

struct AerialKitLedInputs
{
    bool radio_fault;
    bool role_or_model_mismatch;
    bool boot_active;
    bool bind_active;
    bool rf_connected;
    bool host_present;
    bool host_seen;
};

struct AerialKitLedDecision
{
    AerialKitLedState state;
    AerialKitLedPattern pattern;
};

struct AerialKitLedRgb
{
    uint8_t red;
    uint8_t green;
    uint8_t blue;
};

constexpr uint8_t AERIALKIT_LED_RAW_MIN = 51;
constexpr uint8_t AERIALKIT_LED_RAW_MAX = 255;
constexpr uint16_t AERIALKIT_LED_SLOW_PERIOD_MS = 3000;
constexpr uint16_t AERIALKIT_LED_FAST_PERIOD_MS = 1000;
constexpr uint16_t AERIALKIT_LED_PULSE_MS = 100;
constexpr uint16_t AERIALKIT_LED_BOOT_PULSE_MS = 350;
constexpr uint16_t AERIALKIT_LED_BOOT_GAP_MS = 250;
constexpr uint16_t AERIALKIT_LED_MISMATCH_PERIOD_MS = 1500;

inline AerialKitLedDecision aerialkit_led_decide(
    const AerialKitLedInputs &inputs)
{
    if (inputs.role_or_model_mismatch)
    {
        return {AerialKitLedState::RoleOrModelMismatch,
                AerialKitLedPattern::AmberTriple};
    }
    if (inputs.radio_fault)
    {
        return {AerialKitLedState::RadioFault,
                AerialKitLedPattern::RedBlink};
    }
    if (inputs.boot_active)
    {
        return {AerialKitLedState::BootIdentity,
                AerialKitLedPattern::BootIdentity};
    }
    if (inputs.bind_active)
    {
        return {AerialKitLedState::Bind, AerialKitLedPattern::SlowBreath};
    }
    if (inputs.rf_connected)
    {
        return {AerialKitLedState::Connected, AerialKitLedPattern::Steady};
    }
    if (inputs.host_present)
    {
        return {AerialKitLedState::HostPresentDisconnected,
                AerialKitLedPattern::DoublePulse};
    }
    if (inputs.host_seen)
    {
        return {AerialKitLedState::NoSerialInput,
                AerialKitLedPattern::SinglePulse};
    }
    return {AerialKitLedState::Searching, AerialKitLedPattern::FastBreath};
}

constexpr AerialKitLedRgb aerialkit_role_color(bool rx_role)
{
    return rx_role ? AerialKitLedRgb{0, AERIALKIT_LED_RAW_MAX, 0}
                   : AerialKitLedRgb{0, 0, AERIALKIT_LED_RAW_MAX};
}

inline bool aerialkit_led_on(
    AerialKitLedPattern pattern,
    uint32_t elapsed_ms,
    bool rx_role)
{
    switch (pattern)
    {
    case AerialKitLedPattern::Steady:
    case AerialKitLedPattern::SlowBreath:
    case AerialKitLedPattern::FastBreath:
        return true;
    case AerialKitLedPattern::DoublePulse:
        elapsed_ms %= AERIALKIT_LED_FAST_PERIOD_MS;
        return elapsed_ms < AERIALKIT_LED_PULSE_MS ||
               (elapsed_ms >= 250 &&
                elapsed_ms < 250 + AERIALKIT_LED_PULSE_MS);
    case AerialKitLedPattern::SinglePulse:
        return elapsed_ms % 2000 < AERIALKIT_LED_PULSE_MS;
    case AerialKitLedPattern::RedBlink:
        return elapsed_ms % 200 < AERIALKIT_LED_PULSE_MS;
    case AerialKitLedPattern::AmberTriple:
        elapsed_ms %= AERIALKIT_LED_MISMATCH_PERIOD_MS;
        return elapsed_ms < 100 || (elapsed_ms >= 200 && elapsed_ms < 300) ||
               (elapsed_ms >= 400 && elapsed_ms < 500);
    case AerialKitLedPattern::BootIdentity:
        elapsed_ms %=
            AERIALKIT_LED_BOOT_PULSE_MS + AERIALKIT_LED_BOOT_GAP_MS;
        if (!rx_role)
        {
            return elapsed_ms < AERIALKIT_LED_BOOT_PULSE_MS;
        }
        return elapsed_ms < AERIALKIT_LED_BOOT_PULSE_MS ||
               (elapsed_ms >= AERIALKIT_LED_BOOT_PULSE_MS +
                                      AERIALKIT_LED_BOOT_GAP_MS &&
                elapsed_ms < 2 * AERIALKIT_LED_BOOT_PULSE_MS +
                                  AERIALKIT_LED_BOOT_GAP_MS);
    }
    return false;
}

inline uint8_t aerialkit_breath_level(
    uint32_t elapsed_ms,
    uint16_t period_ms)
{
    const uint32_t half = period_ms / 2;
    uint32_t phase = elapsed_ms % period_ms;
    const uint32_t triangle = phase < half ? phase : period_ms - phase;
    const uint32_t smooth = triangle * triangle * (3 * half - 2 * triangle) /
                            (half * half);
    return static_cast<uint8_t>(
        AERIALKIT_LED_RAW_MIN +
        (AERIALKIT_LED_RAW_MAX - AERIALKIT_LED_RAW_MIN) * smooth / half);
}

inline uint8_t aerialkit_led_master_level(
    AerialKitLedPattern pattern,
    uint32_t elapsed_ms)
{
    switch (pattern)
    {
    case AerialKitLedPattern::SlowBreath:
        return aerialkit_breath_level(
            elapsed_ms, AERIALKIT_LED_SLOW_PERIOD_MS);
    case AerialKitLedPattern::FastBreath:
        return aerialkit_breath_level(
            elapsed_ms, AERIALKIT_LED_FAST_PERIOD_MS);
    default:
        return AERIALKIT_LED_RAW_MAX;
    }
}

inline AerialKitLedRgb aerialkit_led_frame(
    AerialKitLedPattern pattern,
    uint32_t elapsed_ms,
    bool rx_role)
{
    if (!aerialkit_led_on(pattern, elapsed_ms, rx_role))
    {
        return {0, 0, 0};
    }
    if (pattern == AerialKitLedPattern::RedBlink)
    {
        return {AERIALKIT_LED_RAW_MAX, 0, 0};
    }
    if (pattern == AerialKitLedPattern::AmberTriple)
    {
        return {AERIALKIT_LED_RAW_MAX,
                static_cast<uint8_t>(AERIALKIT_LED_RAW_MAX / 2),
                0};
    }

    AerialKitLedRgb color = aerialkit_role_color(rx_role);
    const uint8_t level = aerialkit_led_master_level(pattern, elapsed_ms);
    color.red = static_cast<uint8_t>(color.red * level / 255);
    color.green = static_cast<uint8_t>(color.green * level / 255);
    color.blue = static_cast<uint8_t>(color.blue * level / 255);
    return color;
}

constexpr uint8_t aerialkit_led_scale(uint8_t value, uint8_t brightness)
{
    return static_cast<uint8_t>(
        (static_cast<uint16_t>(value) * brightness) / 255U);
}

struct AerialKitWs2812ChannelOrder
{
    uint8_t first;
    uint8_t second;
    uint8_t third;
};

// Hardware testing of all six wire orders established G-R-B as the physical
// channel order for LED1 on the assembled Rev A3 boards.
constexpr AerialKitWs2812ChannelOrder AERIALKIT_WS2812_OPERATIONAL_ORDER = {
    1, // green
    0, // red
    2, // blue
};

// Every possible assignment of logical R/G/B to the three bytes on the wire.
// Names identify the first, second and third logical channel sent.
constexpr AerialKitWs2812ChannelOrder AERIALKIT_WS2812_PERMUTATIONS[] = {
    {0, 1, 2}, // RGB
    {0, 2, 1}, // RBG
    {1, 0, 2}, // GRB
    {1, 2, 0}, // GBR
    {2, 0, 1}, // BRG
    {2, 1, 0}, // BGR
};
constexpr uint8_t AERIALKIT_WS2812_PERMUTATION_COUNT =
    sizeof(AERIALKIT_WS2812_PERMUTATIONS) /
    sizeof(AERIALKIT_WS2812_PERMUTATIONS[0]);

constexpr uint8_t aerialkit_led_channel(const AerialKitLedRgb &color, uint8_t channel)
{
    return channel == 0 ? color.red : channel == 1 ? color.green : color.blue;
}

constexpr uint32_t aerialkit_ws2812_wire_word_with_order(
    const AerialKitLedRgb &color,
    uint8_t brightness,
    const AerialKitWs2812ChannelOrder &order)
{
    return (static_cast<uint32_t>(aerialkit_led_scale(
                aerialkit_led_channel(color, order.first), brightness)) << 16) |
           (static_cast<uint32_t>(aerialkit_led_scale(
                aerialkit_led_channel(color, order.second), brightness)) << 8) |
           aerialkit_led_scale(aerialkit_led_channel(color, order.third), brightness);
}

// The 24-bit word sent to the LED, MSB first, after the global brightness cap.
constexpr uint32_t aerialkit_ws2812_wire_word(
    const AerialKitLedRgb &color,
    uint8_t brightness)
{
    return aerialkit_ws2812_wire_word_with_order(
        color, brightness, AERIALKIT_WS2812_OPERATIONAL_ORDER);
}

// WS2812B-2020 (V6) waveform. Its limits are tighter than the classic
// WS2812 timing used by Arduino neopixelWrite() (0.4/0.8 us), which violates
// T0H <= 380 ns, T1L >= 580 ns and the 1.25 us minimum bit period:
//   T0H 220-380 ns, T0L/T1H/T1L 580-1000 ns, bit >= 1.25 us, reset > 280 us.
// RMT runs at 80 MHz APB / 2 = 25 ns per tick.
constexpr uint8_t AERIALKIT_WS2812_RMT_CLK_DIV = 2;
constexpr uint16_t AERIALKIT_WS2812_TICK_NS = 25;
constexpr uint16_t AERIALKIT_WS2812_T0H_TICKS = 12;  // 300 ns
constexpr uint16_t AERIALKIT_WS2812_T0L_TICKS = 38;  // 950 ns
constexpr uint16_t AERIALKIT_WS2812_T1H_TICKS = 30;  // 750 ns
constexpr uint16_t AERIALKIT_WS2812_T1L_TICKS = 24;  // 600 ns
constexpr uint16_t AERIALKIT_WS2812_RESET_HALF_TICKS = 6000;  // 2 x 150 us low
constexpr uint8_t AERIALKIT_WS2812_DATA_ITEMS = 24;
constexpr uint8_t AERIALKIT_WS2812_FRAME_ITEMS = AERIALKIT_WS2812_DATA_ITEMS + 1;

struct AerialKitRmtItem
{
    uint16_t duration0;
    uint8_t level0;
    uint16_t duration1;
    uint8_t level1;
};

// Item 0..23: one data bit, MSB first. Item 24: the low latch/reset period,
// always sent so a frame never depends on the idle gap before the next one.
constexpr AerialKitRmtItem aerialkit_ws2812_item(uint32_t wire_word, uint8_t index)
{
    return index >= AERIALKIT_WS2812_DATA_ITEMS
               ? AerialKitRmtItem{AERIALKIT_WS2812_RESET_HALF_TICKS, 0,
                                  AERIALKIT_WS2812_RESET_HALF_TICKS, 0}
           : ((wire_word >> (23 - index)) & 1U) != 0
               ? AerialKitRmtItem{AERIALKIT_WS2812_T1H_TICKS, 1,
                                  AERIALKIT_WS2812_T1L_TICKS, 0}
               : AerialKitRmtItem{AERIALKIT_WS2812_T0H_TICKS, 1,
                                  AERIALKIT_WS2812_T0L_TICKS, 0};
}

// Hardware LED self-test: pure red, green, blue for one second each, at the
// same brightness cap, through the operational driver.
constexpr uint32_t AERIALKIT_LED_SELFTEST_STEP_MS = 1000;
constexpr uint8_t AERIALKIT_LED_SELFTEST_TYPE = 0x7F;
constexpr uint32_t AERIALKIT_LED_PERMUTATION_GAP_MS = 500;
constexpr uint32_t AERIALKIT_LED_PERMUTATION_PERIOD_MS =
    3 * AERIALKIT_LED_SELFTEST_STEP_MS + AERIALKIT_LED_PERMUTATION_GAP_MS;
constexpr uint8_t AERIALKIT_LED_PERMUTATION_MARKER = 'P';

inline bool aerialkit_led_selftest_frame(
    const uint8_t *frame, uint8_t own_address)
{
    // C8 08 7F <dest=own address> <origin> 'A' 'K' 'L' 'T' <crc>, CRC already
    // checked by the CRSF parser.
    return frame[1] == 8 && frame[2] == AERIALKIT_LED_SELFTEST_TYPE &&
           frame[3] == own_address && frame[5] == 'A' && frame[6] == 'K' &&
           frame[7] == 'L' && frame[8] == 'T';
}

inline bool aerialkit_led_permutation_frame(
    const uint8_t *frame, uint8_t own_address)
{
    return frame[1] == 8 && frame[2] == AERIALKIT_LED_SELFTEST_TYPE &&
           frame[3] == own_address && frame[5] == 'A' && frame[6] == 'K' &&
           frame[7] == 'L' && frame[8] == AERIALKIT_LED_PERMUTATION_MARKER;
}

constexpr AerialKitLedRgb aerialkit_led_selftest_primary(uint8_t channel)
{
    return channel == 0 ? AerialKitLedRgb{AERIALKIT_LED_RAW_MAX, 0, 0}
           : channel == 1 ? AerialKitLedRgb{0, AERIALKIT_LED_RAW_MAX, 0}
                          : AerialKitLedRgb{0, 0, AERIALKIT_LED_RAW_MAX};
}

inline bool aerialkit_led_selftest_color(uint32_t elapsed_ms, AerialKitLedRgb &color)
{
    switch (elapsed_ms / AERIALKIT_LED_SELFTEST_STEP_MS)
    {
    case 0:
    case 1:
    case 2:
        color = aerialkit_led_selftest_primary(
            elapsed_ms / AERIALKIT_LED_SELFTEST_STEP_MS);
        return true;
    default: return false;
    }
}

struct AerialKitLedPermutationSample
{
    uint8_t permutation;
    uint8_t channel;
    bool gap;
    AerialKitLedRgb color;
};

// One second of logical red, green and blue under each of the six wire
// permutations, followed by a half-second off period.
inline bool aerialkit_led_permutation_sample(
    uint32_t elapsed_ms,
    AerialKitLedPermutationSample &sample)
{
    const uint32_t total =
        AERIALKIT_WS2812_PERMUTATION_COUNT * AERIALKIT_LED_PERMUTATION_PERIOD_MS;
    if (elapsed_ms >= total)
    {
        return false;
    }
    sample.permutation =
        static_cast<uint8_t>(elapsed_ms / AERIALKIT_LED_PERMUTATION_PERIOD_MS);
    const uint32_t phase = elapsed_ms % AERIALKIT_LED_PERMUTATION_PERIOD_MS;
    if (phase >= 3 * AERIALKIT_LED_SELFTEST_STEP_MS)
    {
        sample.channel = 3;
        sample.gap = true;
        sample.color = {0, 0, 0};
        return true;
    }
    sample.channel =
        static_cast<uint8_t>(phase / AERIALKIT_LED_SELFTEST_STEP_MS);
    sample.gap = false;
    sample.color = aerialkit_led_selftest_primary(sample.channel);
    return true;
}

void aerialkitLedMarkSerialInput();
void aerialkitLedObserveFrame(const void *frame);
int aerialkitLedTimeout();

// Shared WS2812 driver (aerialkit_ws2812.cpp), IDF RMT with checked returns.
bool aerialkitWs2812Begin();
// Show() applies the global WS2812_BRIGHTNESS cap; ShowCapped() sends values
// that are already final (boot identity and fault loops).
bool aerialkitWs2812Show(const AerialKitLedRgb &color);
bool aerialkitWs2812ShowCapped(const AerialKitLedRgb &color);
bool aerialkitWs2812ShowOrdered(
    const AerialKitLedRgb &color,
    const AerialKitWs2812ChannelOrder &order);
bool aerialkitWs2812Healthy();
uint32_t aerialkitWs2812Failures();
int aerialkitWs2812LastError();
