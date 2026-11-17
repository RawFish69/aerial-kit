# Hardware reference · Rev A3

## Board and power

- Layout outline is 24 × 30 mm with 1 mm corner radii and four copper layers (EAGLE layers 1, 2, 15 and 16). Design-rule minima are 5 mil trace and spacing and 0.2 mm drill. The approved fabrication stackup and impedance measurements are unknown.
- J1 is a USB4105-GF-A USB-C receptacle. R1/R2 are 5.1 kΩ CC pull-downs for a 5 V USB sink. F1 is a 0.5 A hold PTC from VBUS_IN to VBUS. D1 is USBLC6-2SC6. R4/R5 are 22 Ω series on USB D+/D−. R3 is a 0 Ω shield connection. C2 (4.7 nF) and C3/C4 (12 pF) are marked DNP in CAD.
- U2 TLV62569DBVR and L1 2.2 µH generate nominal 3.32 V from fused VBUS. FB1 feeds 3V3_RADIO (U4) and FB2 feeds 3V3_PA (U5). C9/C11/C12 positive rail nodes are useful respectively for checking main, radio and PA rails. Schematic estimated input budget is about 428 mA at 4.75 V and 85% efficiency with its assumed combined load. Actual current and USB behavior need measurement.
- U3 is ESP32-S3-MINI-1U-N8 with nominal 8 MB flash. R11/C19 pull EN high with a 10 kΩ / 1 µF delay. GPIO0 is pulled high and SW1 grounds it. GPIO45, GPIO46 and GPIO3 are pulled low. SW2 grounds GPIO4 in TX and leaves the 10 kΩ pull-up to select RX. Verify the switch orientation with continuity. Firmware must implement role selection.

## Firmware-facing signals

| Function | ESP32-S3 GPIO | Endpoint |
|---|---:|---|
| BOOT/BIND | 0 | SW1 and TP2 |
| Role select | 4 | SW2, low TX / high RX in design |
| FEM TX enable | 6 | TP5, then R17 to U5 TXEN |
| FEM RX enable | 7 | TP6, then R18 to U5 RXEN |
| SX1280 DIO1 / IRQ | 8 | U4 DIO1 and TP8 |
| SX1280 BUSY | 9 | U4 BUSY and TP7 |
| SX1280 NSS | 10 | U4 NSS, pulled high |
| SX1280 MOSI | 11 | U4 MOSI |
| SX1280 SCK | 12 | U4 SCK |
| SX1280 MISO | 13 | U4 MISO |
| SX1280 reset | 14 | U4 NRESET |
| USB D− / D+ | 19 / 20 | R5 / R4 to J1 |
| Addressable RGB LED | 21 | R15 to LED1 WS2812B-2020 data-in |
| I²C SDA / SCL | 34 / 48 | J3.5 / J3.6 with 4.7 kΩ pull-ups |
| UART0 TX / RX | 43 / 44 | J3.3 / J3.4, board perspective |
| Breakout GPIO | 35 / 36 / 37 / 38 | J5.3 through J5.6 |
| JTAG TCK / TDO / TDI / TMS | 39 / 40 / 41 / 42 | TP9 / TP4 / TP3 / TP10 |

## Pads and connectors

- **J3** pins 1–7 are 3V3, GND, UART0 TX, UART0 RX, SDA, SCL, and fused USB VBUS (5 V output when USB powered). Keep external serial at 3.3 V logic and leave J3 power pins disconnected for UART recovery.
- **J5** pins 1–6 are 3V3, GND, GPIO35, GPIO36, GPIO37 and GPIO38. Schematic notes describe the breakout group as bottom-side. The PCB places J3 mirrored (bottom) and J5 unmirrored (top). Verify physical pad side and pin 1 from the layout/silkscreen before soldering.
- **Reset** is TP1 (ESP_EN, silk RST). **Boot** is SW1 or TP2 (GPIO0, silk IO0). TP5/TP6 are on the top in the PCB. TP1–TP4 and TP7–TP10 are on the bottom. See the component CSV for coordinates in the PCB coordinate frame. For bottom views, follow the board orientation indicated in your CAD viewer.
- **J2** is the bottom-side U.FL for the ELRS RF chain. The antenna connector on the ESP32-S3 module serves its separate Wi-Fi/BLE radio.

## RF design and bring-up

U4 SX1280IMLTRT uses a 52 MHz crystal Y1 and an external 15 µH L2 for its DC/DC configuration. Its RFIO runs through matching/filtering and C30 to the U5 RFX2401C front end. U5 then runs through the antenna LPF and R24 to J2. The design calls for 50 Ω RF geometry, with VNA/spectrum validation pending. The schematic specifies U5 TXEN high for TX, TXEN low/RXEN high for RX, and both low for shutdown. R19/R20 provide shutdown defaults at reset. The schematic records +5 dBm as the maximum permitted SX1280 drive into the FEM. Verify the manufacturer limits and allow margin when deriving the firmware power table. Keep RF disabled during USB/bootstrap checks, and use the intended antenna or a suitable 2.4 GHz 50 Ω terminated bench setup before any TX test.
