// ELRS config for ESP32-C3 (TX and RX)
#pragma once

#include <Arduino.h>

// ------------------------- Bind phrase / UID -------------------------
// Same phrase on TX and RX for binding. Max 6 bytes used for UID.
#define ELRS_BIND_PHRASE "elrsuav"
#define ELRS_UID_LEN 6

// ------------------------- Radio (SX1280) pins -------------------------
// ESP32-C3 available GPIOs only: 0, 1, 4, 5, 6, 7, 10, 18, 19
#define RADIO_PIN_NSS   7
#define RADIO_PIN_RST   18
#define RADIO_PIN_BUSY  19
#define RADIO_PIN_DIO1  4
#define RADIO_PIN_SCK   6
#define RADIO_PIN_MISO  5
#define RADIO_PIN_MOSI  10

// ------------------------- UART (CRSF) -------------------------
// TX: computer sends CRSF here. RX: output CRSF to flight controller.
// Uses GPIO 0, 1 (Serial1). Avoid holding GPIO 0 low at boot (bootloader).
#define CRSF_UART_BAUD 420000
#define CRSF_PIN_TX    0
#define CRSF_PIN_RX    1

// ------------------------- Rate -------------------------
// Packet interval in microseconds (500 Hz = 2000 us)
#define ELRS_PACKET_INTERVAL_US  2000
#define ELRS_FHSS_HOP_INTERVAL   2     // hop every N packets
#define ELRS_PACKET_SIZE         8     // OTA4 = 8 bytes

// ------------------------- 2.4 GHz FHSS -------------------------
// ISM 2.4 GHz band (same as ExpressLRS)
#define ELRS_FREQ_START_HZ  2400400000UL
#define ELRS_FREQ_END_HZ    2479400000UL
#define ELRS_FREQ_COUNT     80
#define ELRS_FHSS_SEQ_LEN   256

// ------------------------- CRSF channel range -------------------------
#define CRSF_CHANNEL_VALUE_MIN 172
#define CRSF_CHANNEL_VALUE_MID 992
#define CRSF_CHANNEL_VALUE_MAX 1811
#define CRSF_NUM_CHANNELS      16
