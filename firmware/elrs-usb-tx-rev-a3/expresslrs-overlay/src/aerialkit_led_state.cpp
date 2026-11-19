#include "targets.h"

#if defined(AERIALKIT_DUAL_ROLE) && defined(PLATFORM_ESP32_S3)

#include "aerialkit_led_state_machine.h"

#include "common.h"

#include <Arduino.h>
#include <HWCDC.h>

void WS281BsetLED(uint32_t color);

namespace
{
constexpr uint32_t SERIAL_INPUT_FRESH_MS = 750;
constexpr uint32_t BIND_HOLD_MS = AERIALKIT_LED_SLOW_PERIOD_MS;
constexpr int LED_UPDATE_MS = 20;

#if defined(TARGET_RX)
constexpr bool RX_ROLE = true;
#else
constexpr bool RX_ROLE = false;
#endif

uint32_t lastSerialInputMs;
bool serialInputSeen;
bool previousBindActive;
uint32_t bindHoldUntilMs;

void setLed(const AerialKitLedRgb &color)
{
    const uint32_t packed =
        (static_cast<uint32_t>(color.red) << 16) |
        (static_cast<uint32_t>(color.green) << 8) |
        color.blue;
    WS281BsetLED(packed);
}

bool serialInputFresh(uint32_t now)
{
    return serialInputSeen && now - lastSerialInputMs <= SERIAL_INPUT_FRESH_MS;
}

bool modelMismatch()
{
    return connectionState == connected &&
           (!connectionHasModelMatch || !teamraceHasModelMatch);
}

bool radioFault()
{
    return connectionState == radioFailed ||
           connectionState == hardwareUndefined;
}
} // namespace

void aerialkitLedMarkSerialInput()
{
    lastSerialInputMs = millis();
    serialInputSeen = true;
}

int aerialkitLedTimeout()
{
    const uint32_t now = millis();
    const bool bindActive = InBindingMode;
    if (bindActive && !previousBindActive)
    {
        bindHoldUntilMs = now + BIND_HOLD_MS;
    }
    previousBindActive = bindActive;
    const bool bindIndication =
        bindActive || static_cast<int32_t>(bindHoldUntilMs - now) > 0;

    const bool usbHostPresent = HWCDC::isConnected();
    const bool hostPresent = usbHostPresent || serialInputFresh(now);
    if (hostPresent)
    {
        serialInputSeen = true;
    }

    const AerialKitLedInputs inputs = {
        .radio_fault = radioFault(),
        .role_or_model_mismatch = modelMismatch(),
        .boot_active = false,
        .bind_active = bindIndication,
        .rf_connected = connectionState == connected,
        .host_present = hostPresent,
        .host_seen = serialInputSeen,
    };
    const AerialKitLedDecision decision = aerialkit_led_decide(inputs);
    setLed(aerialkit_led_frame(decision.pattern, now, RX_ROLE));
    return LED_UPDATE_MS;
}

#endif
