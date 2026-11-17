# Notes transcribed from schematic

## Sheet 1

- USB-C SINK / ESD / VBUS PROTECTION
- NATIVE USB TO ESP32-S3 GPIO19 / GPIO20
- CC1 and CC2 each have 5.1k Rd. USB-C sink/UFP, 5V, no Power Delivery.
- R3 joins shell to GND; C2 is optional, not fitted. F1 protects against sustained overcurrent.
- VBUS input capacitance: C1 0.1u + C6 4.7u + C7 0.1u = 4.9u nominal.
- D1 at connector. R4/R5 at module. Route both USB segments as 90-ohm differential pairs.
- ESP GPIO19=D-, GPIO20=D+. Native USB supports flashing and host data; UART pads provide recovery.
- Firmware must advertise its USB power requirement and hold RF off until USB configuration.

## Sheet 2

- 5V -> 3V3 BUCK (TLV62569, 1.5 MHz, 2 A)
- FILTERED RF RAILS
- VOUT = 0.6 V x (1 + 453k/100k) = 3.32 V. L = 2.2 uH, COUT = 22 uF (TI DS Table 4 'checked' combination), CFF = 6.8 pF (TI 8.2.2.2).
- Budget: S3 350mA + RF/LED 170mA = 520mA at 3.32V; at 4.75V / 85% efficiency, USB input ~428mA. RF duty limits apply.
- Place U2/L1 at the USB end; keep BUCK_SW copper minimal and away from RF section.
- 3V3_RADIO: SX1280 only.  3V3_PA: RFX2401C only (TX ~90-120 mA pulses).
- Separate beads keep PA current steps off the transceiver and buck noise off both.

## Sheet 3

- ESP32-S3-MINI-1U-N8 / 8MB FLASH / NATIVE USB
- Straps: GPIO0 pull-up / BOOT button; GPIO45 pull-down (3.3V flash); GPIO46 pull-down (download enabled); GPIO3 pull-down.
- EN: 10k / 1uF. Module pin 3 decoupled locally by 10u + 100n. All module GND and centre lands bonded to ground plane.
- Module RF connector is Wi-Fi/BLE only, not ELRS. Disable ESP Wi-Fi/BLE during normal ELRS operation.
- ELRS bases: Unified_ESP32S3_2400_TX / RX. Separate firmware images; custom boot role selection and USB RX output required.

## Sheet 4

- RADIO: SX1280 (Semtech ref.) -> RFX2401C PA/LNA (Skyworks app.) -> LPF -> U.FL
- RF chain: RFIO -> C26 shunt / L3 series (match) -> C27-L4||C28-C29 pi harmonic filter -> C30 DC block (RFIO is DC-biased,
- RFX2401C TXRX is DC-shorted to GND). Values = Semtech reference BOM. SX1280 DC-DC used (L2): ELRS radio_dcdc = true.
- SX1280 output must stay <= +5 dBm (RFX2401C TX input abs max). ELRS power_values limited accordingly.
- FEM control (Skyworks table 6): TXEN=1 TX, TXEN=0/RXEN=1 RX, both 0 = shutdown.
- R19/R20 100k pull-downs hold the FEM in shutdown while the ESP32 is in reset/boot (GPIO hi-Z).
- C33 0.3p at ANT pin (5th harmonic) + 5th-order LC LPF (Skyworks fig 3 option 1). All 50 ohm, 0402.
- R24 = fitted U.FL series link; RF path and LPF must be checked with VNA / spectrum analyzer.
- HCI 52MHz crystal ESR max 60 ohm; Semtech allows up to 90 ohm only if C0 <3pF. HCI family specification C0 <3pF; oscillator startup must be verified in bring-up.

## Sheet 5

- BOOT / STATUS / ROLE AT RESET
- BOTTOM-SIDE GPIO / UART / JTAG / TEST PADS
- SW2 low=TX, high=RX. Sample only at reset; firmware changes role after reboot. RX is the open-switch default.
- J5 GPIO35/36/37/38 are 3.3V bidirectional GPIO on this S3 N8 module. Headers are not 5V tolerant.
- BOOT/BIND on GPIO0. Reset via TP1. USB flashing is normal; UART0 pads provide recovery.
- J3: 3.3V logic. 5V is USB output after F1; do not backfeed USB.
