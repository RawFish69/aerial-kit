#include "targets.h"

#if defined(AERIALKIT_DUAL_ROLE) && defined(PLATFORM_ESP32_S3)

// The one WS2812 output path for dual-role builds: boot identity, fault loops,
// the state-machine LED device and the self-test all send frames here.
//
// It uses the IDF RMT driver directly rather than the Arduino rmt* wrapper,
// because its blocking write discards the rmt_write_items() result and always
// returns true. Every call here is checked, the channel clock is read back,
// and each frame ends with an explicit >280 us low reset period.

#include "aerialkit_led_state_machine.h"

#include <Arduino.h>
#include <driver/rmt.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

#ifndef WS2812_BRIGHTNESS
#define WS2812_BRIGHTNESS 255
#endif

namespace
{
constexpr gpio_num_t LED_PIN = GPIO_NUM_21;
// The highest TX channel; nothing else in these builds uses RMT.
constexpr rmt_channel_t LED_CHANNEL = RMT_CHANNEL_3;
constexpr TickType_t TX_TIMEOUT = pdMS_TO_TICKS(10);

SemaphoreHandle_t ledLock;
bool ready;
uint32_t failures;
esp_err_t lastError = ESP_OK;

bool fail(esp_err_t error)
{
    lastError = error;
    ++failures;
    return false;
}

bool beginLocked()
{
    if (ready)
    {
        return true;
    }
    rmt_config_t config = RMT_DEFAULT_CONFIG_TX(LED_PIN, LED_CHANNEL);
    config.clk_div = AERIALKIT_WS2812_RMT_CLK_DIV;
    config.mem_block_num = 1;
    config.tx_config.carrier_en = false;
    config.tx_config.loop_en = false;
    config.tx_config.idle_level = RMT_IDLE_LEVEL_LOW;
    config.tx_config.idle_output_en = true;

    esp_err_t error = rmt_config(&config);
    if (error != ESP_OK)
    {
        return fail(error);
    }
    error = rmt_driver_install(LED_CHANNEL, 0, 0);
    if (error != ESP_OK)
    {
        return fail(error);
    }

    // Read back the tick: APB source and divider 2 give exactly 25 ns.
    rmt_source_clk_t source = RMT_BASECLK_MAX;
    uint8_t divider = 0;
    if (rmt_get_source_clk(LED_CHANNEL, &source) != ESP_OK ||
        rmt_get_clk_div(LED_CHANNEL, &divider) != ESP_OK ||
        source != RMT_BASECLK_APB ||
        divider != AERIALKIT_WS2812_RMT_CLK_DIV ||
        APB_CLK_FREQ != 80000000)
    {
        (void)rmt_driver_uninstall(LED_CHANNEL);
        return fail(ESP_ERR_INVALID_STATE);
    }
    ready = true;
    return true;
}
} // namespace

bool aerialkitWs2812Begin()
{
    if (ledLock == nullptr)
    {
        ledLock = xSemaphoreCreateMutex();
        if (ledLock == nullptr)
        {
            return fail(ESP_ERR_NO_MEM);
        }
    }
    if (xSemaphoreTake(ledLock, portMAX_DELAY) != pdTRUE)
    {
        return fail(ESP_ERR_TIMEOUT);
    }
    const bool ok = beginLocked();
    xSemaphoreGive(ledLock);
    return ok;
}

namespace
{
bool sendWord(uint32_t word)
{
    if (!aerialkitWs2812Begin())
    {
        return false;
    }
    rmt_item32_t items[AERIALKIT_WS2812_FRAME_ITEMS];
    for (uint8_t index = 0; index < AERIALKIT_WS2812_FRAME_ITEMS; ++index)
    {
        const AerialKitRmtItem item = aerialkit_ws2812_item(word, index);
        items[index].duration0 = item.duration0;
        items[index].level0 = item.level0;
        items[index].duration1 = item.duration1;
        items[index].level1 = item.level1;
    }

    if (xSemaphoreTake(ledLock, portMAX_DELAY) != pdTRUE)
    {
        return fail(ESP_ERR_TIMEOUT);
    }
    esp_err_t error = rmt_write_items(LED_CHANNEL, items, AERIALKIT_WS2812_FRAME_ITEMS, false);
    if (error == ESP_OK)
    {
        error = rmt_wait_tx_done(LED_CHANNEL, TX_TIMEOUT);
    }
    xSemaphoreGive(ledLock);
    return error == ESP_OK ? true : fail(error);
}
} // namespace

bool aerialkitWs2812Show(const AerialKitLedRgb &color)
{
    return sendWord(aerialkit_ws2812_grb_word(color, WS2812_BRIGHTNESS));
}

bool aerialkitWs2812ShowCapped(const AerialKitLedRgb &color)
{
    return sendWord(aerialkit_ws2812_grb_word(color, 255));
}

bool aerialkitWs2812Healthy()
{
    return ready && lastError == ESP_OK;
}

uint32_t aerialkitWs2812Failures()
{
    return failures;
}

int aerialkitWs2812LastError()
{
    return static_cast<int>(lastError);
}

#endif
