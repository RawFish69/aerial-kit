#include "targets.h"

#if defined(AERIALKIT_DUAL_ROLE) && defined(PLATFORM_ESP32_S3)

#include "aerialkit_boot_diagnostics.h"
#include "aerialkit_boot_logic.h"
#include "common.h"
#include "config.h"
#include "FHSS.h"
#include "OTA.h"

#include <Arduino.h>
#include <esp_ota_ops.h>
#include <esp_system.h>

#include "aerialkit_led_state_machine.h"

#include <stdio.h>

#include <stddef.h>

namespace
{
bool roleHighAtBoot;
uint32_t bootStartedAt;
bool haveBootStartedAt;
uint32_t lastRfSampleAt;
uint8_t previousTxEnable;
uint8_t previousRxEnable;
uint32_t txEnableHighSamples;
uint32_t rxEnableHighSamples;
uint32_t txEnableRises;
uint32_t rxEnableRises;
uint32_t usbFrames;
uint32_t usbRcFrames;
uint8_t lastUsbFrameType;

const char *resetReasonName(esp_reset_reason_t reason)
{
    switch (reason)
    {
    case ESP_RST_POWERON: return "poweron";
    case ESP_RST_EXT: return "external";
    case ESP_RST_SW: return "software";
    case ESP_RST_PANIC: return "panic";
    case ESP_RST_INT_WDT: return "int_wdt";
    case ESP_RST_TASK_WDT: return "task_wdt";
    case ESP_RST_WDT: return "wdt";
    case ESP_RST_DEEPSLEEP: return "deepsleep";
    case ESP_RST_BROWNOUT: return "brownout";
    case ESP_RST_SDIO: return "sdio";
    default: return "unknown";
    }
}

constexpr int ROLE_PIN = 4;
constexpr int TX_ENABLE_PIN = 6;
constexpr int RX_ENABLE_PIN = 7;
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
    return aerialkitWs2812ShowCapped({red, green, blue});
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
    roleHighAtBoot = roleHigh;
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

void aerialkitBootReport()
{
    const esp_partition_t *running = esp_ota_get_running_partition();
    char elf[9] = "unknown";
    (void)esp_ota_get_app_elf_sha256(elf, sizeof(elf));
    const auto *rate = ExpressLRS_currAirRate_Modparams;
    const uint32_t seed = OtaGetUidSeed();
    char line[384];
    const int length = snprintf(
        line,
        sizeof(line),
        "AK-BOOT role=%s slot=%s gpio4=%s expect_gpio4=%s reset=%s(%d) elf=%s "
        "uid=%02x%02x%02x%02x%02x%02x seed=0x%08lx crc=0x%04x "
        "rate_idx=%u rate_enum=%u interval_us=%u fhss0=%u freq=%lu "
        "domain=%s conn=%u txen=%u rxen=%u "
        "led=%s led_failures=%u led_error=0x%x "
        "ws2812_t0h=300ns t1h=750ns latch=300us\r\n",
        RX_ROLE ? "rx" : "tx",
        running != nullptr ? running->label : "unknown",
        roleHighAtBoot ? "high" : "low",
        RX_ROLE ? "low" : "high",
        resetReasonName(esp_reset_reason()),
        static_cast<int>(esp_reset_reason()),
        elf,
        UID[0], UID[1], UID[2], UID[3], UID[4], UID[5],
        static_cast<unsigned long>(seed),
        static_cast<unsigned>(OtaCrcInitializer),
        rate != nullptr ? static_cast<unsigned>(rate->index) : 0xffU,
        rate != nullptr ? static_cast<unsigned>(rate->enum_rate) : 0xffU,
        rate != nullptr ? static_cast<unsigned>(rate->interval) : 0U,
        static_cast<unsigned>(FHSSsequence[0]),
        static_cast<unsigned long>(FHSSgetInitialFreq()),
        FHSSconfig != nullptr ? FHSSconfig->domain : "unknown",
        static_cast<unsigned>(connectionState),
        static_cast<unsigned>(digitalRead(TX_ENABLE_PIN)),
        static_cast<unsigned>(digitalRead(RX_ENABLE_PIN)),
        aerialkitWs2812Healthy() ? "rmt-ok" : "rmt-fail",
        static_cast<unsigned>(aerialkitWs2812Failures()),
        static_cast<unsigned>(aerialkitWs2812LastError()));
    if (length > 0)
    {
        USBSerial.write(reinterpret_cast<const uint8_t *>(line),
                        static_cast<size_t>(length) < sizeof(line)
                            ? static_cast<size_t>(length)
                            : sizeof(line) - 1);
    }
}

void aerialkitRfSample(uint32_t now)
{
    if (!haveBootStartedAt)
    {
        bootStartedAt = now;
        haveBootStartedAt = true;
        previousTxEnable = digitalRead(TX_ENABLE_PIN);
        previousRxEnable = digitalRead(RX_ENABLE_PIN);
    }

    const uint8_t txEnable = digitalRead(TX_ENABLE_PIN);
    const uint8_t rxEnable = digitalRead(RX_ENABLE_PIN);
    txEnableHighSamples += txEnable != 0;
    rxEnableHighSamples += rxEnable != 0;
    txEnableRises += previousTxEnable == 0 && txEnable != 0;
    rxEnableRises += previousRxEnable == 0 && rxEnable != 0;
    previousTxEnable = txEnable;
    previousRxEnable = rxEnable;

    if ((now - bootStartedAt) > 15000U || (now - lastRfSampleAt) < 1000U)
    {
        return;
    }
    lastRfSampleAt = now;

    const auto *rate = ExpressLRS_currAirRate_Modparams;
    const int8_t rssiInst = Radio.GetRssiInst(SX12XX_Radio_All);
    const uint16_t irq = Radio.GetIrqStatus(SX12XX_Radio_All);
#if defined(TARGET_TX)
    const unsigned linkMode = config.GetLinkMode();
#else
    const unsigned linkMode = 0xffU;
#endif
    char line[320];
    const int length = snprintf(
        line,
        sizeof(line),
        "AK-RF role=%s conn=%u rate_idx=%u rate_enum=%u "
        "linkmode=%u usb_frames=%lu usb_rc=%lu last_usb_type=0x%02x "
        "txen=%u rxen=%u tx_high=%lu rx_high=%lu tx_rise=%lu rx_rise=%lu "
        "rssi_inst=%d last_rssi=%d last_snr=%d irq=0x%04x\r\n",
        RX_ROLE ? "rx" : "tx",
        static_cast<unsigned>(connectionState),
        rate != nullptr ? static_cast<unsigned>(rate->index) : 0xffU,
        rate != nullptr ? static_cast<unsigned>(rate->enum_rate) : 0xffU,
        linkMode,
        static_cast<unsigned long>(usbFrames),
        static_cast<unsigned long>(usbRcFrames),
        static_cast<unsigned>(lastUsbFrameType),
        static_cast<unsigned>(txEnable),
        static_cast<unsigned>(rxEnable),
        static_cast<unsigned long>(txEnableHighSamples),
        static_cast<unsigned long>(rxEnableHighSamples),
        static_cast<unsigned long>(txEnableRises),
        static_cast<unsigned long>(rxEnableRises),
        static_cast<int>(rssiInst),
        static_cast<int>(Radio.LastPacketRSSI),
        static_cast<int>(Radio.LastPacketSNRRaw),
        static_cast<unsigned>(irq));
    if (length > 0)
    {
        USBSerial.write(
            reinterpret_cast<const uint8_t *>(line),
            static_cast<size_t>(length) < sizeof(line)
                ? static_cast<size_t>(length)
                : sizeof(line) - 1);
    }
}

void aerialkitRfNoteUsbFrame(uint8_t frame_type)
{
    usbFrames++;
    usbRcFrames += frame_type == CRSF_FRAMETYPE_RC_CHANNELS_PACKED;
    lastUsbFrameType = frame_type;
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
