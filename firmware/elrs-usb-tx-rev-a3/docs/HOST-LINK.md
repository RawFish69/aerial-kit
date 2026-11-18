# Rev A3 host link

This describes how a host talks to the Aerial Kit ELRS USB TX Rev A3 using
the pinned ExpressLRS 4.1.0 target.

## Selected transport

Use a **3.3 V USB-UART adapter** connected to UART0 on J3:

| Host signal | J3 pin | ESP32-S3 signal |
| --- | ---: | --- |
| Host TX to board RX | J3.4 | GPIO44 / U0RXD |
| Host RX from board TX | J3.3 | GPIO43 / U0TXD |
| Ground | J3.2 | GND |

Use full-duplex mode: `serial_rx` and `serial_tx` are different GPIOs. Do not
tie them together. Leave J3.1 (3V3) and J3.7 (USB VBUS) disconnected when the
board is powered by USB.

The normal handset CRSF rate is **400000 baud, 8N1, non-inverted**. The pinned
ExpressLRS `CRSFHandset.cpp` starts autobaud detection at 400000 and only cycles
through its handset baud table. The 420000 rate is the receiver-to-flight-
controller link, not the host-to-TX link.

Native USB-CDC on GPIO19/20 is not a drop-in RC transport in stock
ExpressLRS 4.1.0. On the ESP32-S3 TX build, `USBSerial` is assigned to the
TxUSB/configuration side, while RC frame input is read by `CRSFHandset` from
UART0. A host therefore needs the USB-UART adapter for autonomous CRSF input.

Native USB on GPIO19/20 supports:

- ROM flashing.
- ExpressLRS configuration and firmware-update workflows.
- Other ExpressLRS TxUSB traffic exposed by the firmware.

It does **not** carry `RC_CHANNELS_PACKED` from the host to the CRSF handset
parser without a custom firmware bridge.

## TX and RX images

ExpressLRS builds separate TX and RX applications:

- TX target: `AERIALKIT_ELRS_USB_TX_REV_A3_via_UART`.
- RX target: `AERIALKIT_ELRS_USB_TX_REV_A3_RX_via_UART`.

The physical SW2 level on GPIO4 does not choose between images at runtime.
Set the switch to match the image being run and verify its continuity because
the first prototype read low while its physical label indicated RX.

The RX image outputs receiver frames on UART0 at the receiver-to-FC rate,
normally 420000 baud, unless configuration changes it.

## Firmware overlay

The board overlay is stored under `patches/` and pins ExpressLRS 4.1.0 at
commit `a9d4a9cb5b5687c4c9d7e9e7fbdf44ad93651da6`. It:

- supplies the Rev A3 target JSON;
- selects the ESP32-S3 2.4 GHz TX or RX environment;
- enables the SX1280 DC/DC path;
- maps TXEN/RXEN, SPI, USB and LED pins;
- limits the WS2812 to 26/255.

Both firmware environments build in this repository. No hardware was flashed
or opened as part of this host-link milestone.

## Host integration decision

The host-side implementation will provide:

- a pure-Python CRSF encoder/decoder independent of ROS;
- a selectable serial transport for a real UART and an in-memory transport
  for tests/SIL;
- a ROS radio backend that keeps the existing `/uav/backend/*` contract;
- a separate simulated ELRS link node for loss, latency and packet-rate tests.
