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

void aerialkitLedMarkSerialInput();
int aerialkitLedTimeout();
