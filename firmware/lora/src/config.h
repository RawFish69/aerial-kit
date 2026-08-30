// firmware/lora config template.
//
// This is a starting point for a long-range, low-bandwidth LoRa link
// (telemetry/backup command channel), separate from the high-rate 2.4 GHz
// RC link in ../ELRS. Point-to-point: flash the same firmware onto two
// boards and they'll heartbeat to each other.
//
// To adapt this to your own band/frequency, don't edit the numbers below
// directly -- add a new [env:my_band] section in platformio.ini that sets
// -DLORA_FREQ_MHZ (see the lora_433/lora_868/lora_915 examples already
// there). The #ifndef guards here mean a build_flags define always wins;
// editing this file only changes the fallback defaults.
#pragma once

#include <Arduino.h>

// ------------------------- Frequency -------------------------
// Check your local ISM band regulations before choosing/transmitting on a
// frequency: 433 MHz and 868 MHz are common in the EU/Asia, 915 MHz in the
// US/Australia. Duty-cycle and max EIRP limits vary by region and band.
#ifndef LORA_FREQ_MHZ
#define LORA_FREQ_MHZ 433.0
#endif

// LoRa PHY parameters: bandwidth (kHz), spreading factor (6-12), coding
// rate denominator (5-8, as in 4/5..4/8). Lower bandwidth / higher SF /
// higher CR = longer range but lower data rate and more airtime per packet.
#ifndef LORA_BANDWIDTH_KHZ
#define LORA_BANDWIDTH_KHZ 125.0
#endif
#ifndef LORA_SPREADING_FACTOR
#define LORA_SPREADING_FACTOR 9
#endif
#ifndef LORA_CODING_RATE
#define LORA_CODING_RATE 7
#endif
#ifndef LORA_SYNC_WORD
#define LORA_SYNC_WORD 0x12  // private-network sync word; RadioLib default
#endif
#ifndef LORA_TX_POWER_DBM
#define LORA_TX_POWER_DBM 10  // keep well under your region's EIRP limit
#endif

// ------------------------- Radio (SX127x) pins -------------------------
// ESP32-C3-DevKitM-1 usable GPIOs: 0, 1, 4, 5, 6, 7, 10, 18, 19.
// Same SPI/NSS/RST pins as ../ELRS for a consistent wiring layout; SX127x
// only needs one IRQ line (DIO0) instead of SX1280's DIO1+BUSY.
#define LORA_PIN_NSS  7
#define LORA_PIN_RST  18
#define LORA_PIN_DIO0 4
#define LORA_PIN_SCK  6
#define LORA_PIN_MISO 5
#define LORA_PIN_MOSI 10

// ------------------------- Heartbeat demo -------------------------
#ifndef LORA_HEARTBEAT_INTERVAL_MS
#define LORA_HEARTBEAT_INTERVAL_MS 1000
#endif

// Give each board a different ID (e.g. -DLORA_NODE_ID=2 for the second
// board) so you can tell them apart in the serial log.
#ifndef LORA_NODE_ID
#define LORA_NODE_ID 1
#endif
