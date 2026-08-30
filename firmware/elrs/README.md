# ELRS — ExpressLRS-style link for aerial-kit

ESP32-C3 **Transmitter (TX)** and **Receiver (RX)** using an ExpressLRS-compatible over-the-air protocol over 2.4 GHz SX1280 FLRC. Intended for autonomous flight: computer sends CRSF over UART to TX; RX outputs CRSF to the flight controller.

## Hardware

- **MCU**: ESP32-C3 (e.g. esp32-c3-devkitm-1).
- **Radio**: SX1280 2.4 GHz module (SPI: NSS, SCK, MISO, MOSI; DIO1, RST, BUSY).
- **UART**: CRSF @ 420000 baud (TX: from computer; RX: to flight controller).

Pin definitions are in `src/config.h`. Only these ESP32-C3 GPIOs are used: **0, 1, 4, 5, 6, 7, 10, 18, 19**.

| Signal   | GPIO |
|----------|------|
| NSS      | 7    |
| RST      | 18   |
| BUSY     | 19   |
| DIO1     | 4    |
| SCK      | 6    |
| MISO     | 5    |
| MOSI     | 10   |
| CRSF TX  | 0    |
| CRSF RX  | 1    |

Adjust in `config.h` if your wiring differs. Avoid holding GPIO 0 low at boot (enters bootloader).

## Bind phrase

TX and RX must use the **same bind phrase** (and thus same UID) so FHSS and CRC match. Set in `src/config.h`:

```c
#define ELRS_BIND_PHRASE "elrsuav"
```

## Build and upload

```bash
cd firmware/elrs

# Transmitter (computer → RF)
pio run -e elrs_tx
pio run -e elrs_tx -t upload

# Receiver (RF → flight controller)
pio run -e elrs_rx
pio run -e elrs_rx -t upload
```

## Usage

1. **TX**: Connect computer UART (CRSF) to CRSF TX/RX pins. Send CRSF RC channel frames; TX sends them over the air at 500 Hz (configurable in `config.h`).
2. **RX**: Power on after TX. After sync, RX outputs CRSF RC to the FC UART. Connect FC serial to CRSF TX/RX at 420000 baud.

## Protocol summary

- **OTA**: 8-byte packets (OTA4-style): RCDATA (4 channels + switches) or SYNC (timing/FHSS). CRC and nonce for validation.
- **FHSS**: 2.4 GHz ISM, 80 channels, 256-step sequence derived from bind phrase.
- **CRSF**: Same frame format as `../espnow` / aerial-kit CRSF (16×11-bit channels, DVB-S2 CRC).

## Rate and interval

Default: 500 Hz packet rate (`ELRS_PACKET_INTERVAL_US = 2000`), hop every 2 packets (`ELRS_FHSS_HOP_INTERVAL`). Edit `src/config.h` to change.

## Notes

- This is a minimal ELRS-compatible stack for aerial-kit. It is not a full ExpressLRS fork; sync word uses RadioLib defaults so it will not bind with standard ExpressLRS hardware.
- For production you may want to add telemetry (link stats, battery) and/or vendored SX1280 driver with custom sync word from UID for full ExpressLRS compatibility.
