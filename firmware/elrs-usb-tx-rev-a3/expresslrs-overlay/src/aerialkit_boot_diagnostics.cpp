#include "targets.h"

#if defined(AERIALKIT_DUAL_ROLE) && defined(PLATFORM_ESP32_S3)

#include "aerialkit_boot_diagnostics.h"
#include "aerialkit_boot_logic.h"

#include <Arduino.h>
#include <esp32-hal-rmt.h>

#include <stddef.h>

namespace
{
constexpr int ROLE_PIN = 4;
constexpr int TX_ENABLE_PIN = 6;
constexpr int RX_ENABLE_PIN = 7;
constexpr int LED_PIN = 21;
constexpr int RMT_TICK_NS = 100;
constexpr int WS2812_ITEMS = 24;
constexpr uint32_t FAULT_ON_MS = 100;
constexpr uint32_t FAULT_OFF_MS = 100;

#if defined(TARGET_RX)
constexpr bool RX_ROLE = true;
#else
constexpr bool RX_ROLE = false;
#endif

void setFemEnablesLow()
{
    pinMode(TX_ENABLE_PIN, OUTPUT);
    digitalWrite(TX_ENABLE_PIN, LOW);
    pinMode(RX_ENABLE_PIN, OUTPUT);
    digitalWrite(RX_ENABLE_PIN, LOW);
}

bool writeColor(uint8_t green, uint8_t red, uint8_t blue)
{
    rmt_obj_t *rmt = rmtInit(LED_PIN, RMT_TX_MODE, RMT_MEM_64);
    if (rmt == nullptr)
    {
        return false;
    }

    bool ok = rmtSetTick(rmt, RMT_TICK_NS) > 0.0f;
    rmt_data_t data[WS2812_ITEMS] {};
    const uint8_t channels[] = {green, red, blue};
    size_t index = 0;
    for (uint8_t channel : channels)
    {
        for (uint8_t mask = 0x80; mask != 0; mask >>= 1)
        {
            const bool high = (channel & mask) != 0;
            data[index].level0 = 1;
            data[index].duration0 = high ? 8 : 4;
            data[index].level1 = 0;
            data[index].duration1 = high ? 4 : 8;
            ++index;
        }
    }
    ok = ok && rmtWriteBlocking(rmt, data, WS2812_ITEMS);
    ok = rmtDeinit(rmt) && ok;
    return ok;
}

void serialLine(const char *message)
{
    USBSerial.println(message);
}

[[noreturn]] void faultLoop(
    const char *message,
    uint8_t green,
    uint8_t red,
    uint8_t blue,
    uint8_t pulses,
    uint32_t pause_ms)
{
    setFemEnablesLow();
    USBSerial.begin(RX_ROLE ? 420000 : 400000);
    while (true)
    {
        for (uint8_t pulse = 0; pulse < pulses; ++pulse)
        {
            const bool ledOk = writeColor(green, red, blue);
            if (!ledOk)
            {
                serialLine("AK-FAULT LED output unavailable");
            }
            delay(FAULT_ON_MS);
            if (ledOk)
            {
                (void)writeColor(0, 0, 0);
            }
            delay(FAULT_OFF_MS);
        }
        serialLine(message);
        delay(pause_ms);
    }
}
} // namespace

void aerialkitBootPreflight()
{
    setFemEnablesLow();
    pinMode(ROLE_PIN, INPUT);
    const bool roleHigh = digitalRead(ROLE_PIN) == HIGH;
    if (!aerialkit_role_matches(roleHigh, RX_ROLE))
    {
        faultLoop(
            "AK-FAULT GPIO4 role mismatch; FEM enables held low",
            AERIALKIT_WS2812_LEVEL / 2,
            AERIALKIT_WS2812_LEVEL,
            0,
            3,
            1000);
    }

    const aerialkit_heartbeat_spec_t heartbeat =
        aerialkit_heartbeat_spec(RX_ROLE);
    for (uint8_t pulse = 0; pulse < heartbeat.pulses; ++pulse)
    {
        const bool ledOk = writeColor(
            heartbeat.green, heartbeat.red, heartbeat.blue);
        if (!ledOk)
        {
            serialLine("AK-FAULT initial LED output unavailable");
            return;
        }
        delay(heartbeat.on_ms);
        if (!writeColor(0, 0, 0))
        {
            serialLine("AK-FAULT LED clear failed");
            return;
        }
        delay(heartbeat.off_ms);
    }
}

[[noreturn]] void aerialkitHardwareUndefinedFault()
{
    faultLoop(
        "AK-FAULT hardware layout not loaded",
        0,
        AERIALKIT_WS2812_LEVEL,
        0,
        1,
        0);
}

#endif
