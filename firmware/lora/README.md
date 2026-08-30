# lora — LoRa point-to-point template

A minimal, working example of a LoRa link on ESP32-C3, meant as a starting
point rather than a finished protocol. It's separate from the high-rate
2.4 GHz RC link in `../elrs`: LoRa (433/868/915 MHz depending on region)
trades bandwidth for range, so it's better suited to a long-range,
low-rate telemetry or backup command channel than primary RC control.

Flash the same firmware onto two boards (give them different
`LORA_NODE_ID` values) and they'll heartbeat to each other over the air,
printing RSSI/SNR for each received packet — a quick way to confirm your
wiring and band choice work before building your own protocol on top.

## Hardware

- **MCU**: ESP32-C3 (e.g. esp32-c3-devkitm-1).
- **Radio**: SX1276/SX1278/RFM9x-family LoRa module (SPI: NSS, SCK, MISO,
  MOSI; DIO0; RST).
- Only these ESP32-C3 GPIOs are used, same layout as `../elrs`: **0, 1, 4,
  5, 6, 7, 10, 18, 19**.

| Signal | GPIO |
|--------|------|
| NSS    | 7    |
| RST    | 18   |
| DIO0   | 4    |
| SCK    | 6    |
| MISO   | 5    |
| MOSI   | 10   |

Adjust in `src/config.h` if your wiring differs.

## Customizing the band/frequency

`platformio.ini` has three ready-made presets:

```bash
pio run -e lora_433   # 433 MHz (common EU/Asia ISM band)
pio run -e lora_868   # 868 MHz (EU)
pio run -e lora_915   # 915 MHz (US/AU)
```

To add your own, copy one of the `[env:lora_*]` sections and change
`LORA_FREQ_MHZ`. **Check your local ISM band regulations** (allowed
frequencies, duty cycle, max EIRP) before transmitting — they vary by
country and this template doesn't enforce any of them.

Other LoRa PHY parameters (bandwidth, spreading factor, coding rate, sync
word, TX power) live in `src/config.h` with the same `-D` override
pattern, so you can also set them per-environment in `platformio.ini` if
you want e.g. a long-range/low-rate preset vs. a short-range/high-rate one.

## Extending this into a real link

This template only sends a tiny heartbeat struct (node ID, counter,
uptime). To build your own protocol on top:

1. Replace `HeartbeatPacket` in `src/main.cpp` with your own packed
   struct (telemetry fields, command fields, etc.) — keep it under the
   LoRa payload size limit (~255 bytes, much less in practice at higher
   spreading factors).
2. If you need request/response instead of open-loop heartbeats, add a
   packet-type byte and branch on it in `handleReceivedPacket()`.
3. Consider adding a simple CRC/sequence-number check if packet loss on
   your link needs to be detected by the application layer (LoRa's own
   CRC only catches corruption, not implicit loss of retries you may add).
