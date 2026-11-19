#include "targets.h"

#if defined(AERIALKIT_DUAL_ROLE) && defined(PLATFORM_ESP32_S3)

// Dual-role LED device. It replaces the upstream devRGB.cpp/esp32rgb.cpp
// (I2S) implementation, which is compiled out of dual-role builds. The only
// colors this file can emit are the role color, red (fault) and amber
// (mismatch), plus the pure red/green/blue hardware self-test, all through
// aerialkit_ws2812.cpp.
//
// initialize(), event() and timeout() all run in the device framework's
// core-0 task, so frames are never sent concurrently from two contexts.

#include "aerialkit_led_state_machine.h"

#include "common.h"
#include "devLED.h"

#include <Arduino.h>
#include <HWCDC.h>

namespace
{
constexpr uint32_t SERIAL_INPUT_FRESH_MS = 750;
constexpr uint32_t BIND_HOLD_MS = AERIALKIT_LED_SLOW_PERIOD_MS;
constexpr int LED_UPDATE_MS = 20;

#if defined(TARGET_RX)
constexpr bool RX_ROLE = true;
constexpr uint8_t OWN_CRSF_ADDRESS = 0xEC; // CRSF_ADDRESS_CRSF_RECEIVER
#else
constexpr bool RX_ROLE = false;
constexpr uint8_t OWN_CRSF_ADDRESS = 0xEE; // CRSF_ADDRESS_CRSF_TRANSMITTER
#endif

bool lastWordValid;
uint32_t lastWord;

volatile uint32_t lastSerialInputMs;
volatile bool serialInputSeen;
volatile bool selfTestRequested;
bool selfTestActive;
uint32_t selfTestStartMs;
bool previousBindActive;
uint32_t bindHoldUntilMs;

void ledShow(const AerialKitLedRgb &color)
{
    const uint32_t word = aerialkit_ws2812_wire_word(color, 255);
    if (lastWordValid && word == lastWord)
    {
        return;
    }
    lastWordValid = aerialkitWs2812Show(color);
    lastWord = word;
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

bool initialize()
{
    return aerialkitWs2812Begin();
}

int start()
{
    return DURATION_IMMEDIATELY;
}

bool runSelfTest(uint32_t now)
{
    if (selfTestRequested)
    {
        selfTestRequested = false;
        selfTestActive = true;
        selfTestStartMs = now;
        USBSerial.print("AK-LEDTEST red green blue 1000ms each\r\n");
    }
    if (!selfTestActive)
    {
        return false;
    }
    AerialKitLedRgb color {};
    if (aerialkit_led_selftest_color(now - selfTestStartMs, color))
    {
        ledShow(color);
        return true;
    }
    selfTestActive = false;
    USBSerial.print(aerialkitWs2812Healthy() ? "AK-LEDTEST done led=ok\r\n"
                                             : "AK-LEDTEST done led=error\r\n");
    return false;
}
} // namespace

void aerialkitLedMarkSerialInput()
{
    lastSerialInputMs = millis();
    serialInputSeen = true;
}

void aerialkitLedObserveFrame(const void *frame)
{
    if (aerialkit_led_selftest_frame(static_cast<const uint8_t *>(frame), OWN_CRSF_ADDRESS))
    {
        selfTestRequested = true;
    }
}

int aerialkitLedTimeout()
{
    const uint32_t now = millis();
    if (runSelfTest(now))
    {
        return LED_UPDATE_MS;
    }

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
    ledShow(aerialkit_led_frame(decision.pattern, now, RX_ROLE));
    return LED_UPDATE_MS;
}

// devWIFI.cpp calls this upstream hook for handset button colors. The
// dual-role board has no button LEDs, and no other color may be shown.
void setButtonColors(uint8_t, uint8_t)
{
}

device_t RGB_device = {
    .initialize = initialize,
    .start = start,
    .event = aerialkitLedTimeout,
    .timeout = aerialkitLedTimeout,
    .subscribe = EVENT_CONNECTION_CHANGED | EVENT_ENTER_BIND_MODE | EVENT_EXIT_BIND_MODE,
};

#endif
