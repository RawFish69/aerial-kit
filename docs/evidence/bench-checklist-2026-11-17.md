# Direct ELRS / CRSF bench checklist — 2026-11-17

This is the ordered bench handoff for the Aerial Kit ELRS USB TX Rev A3 and
the direct CRSF ROS backend. Nothing in this checklist was performed during
the overnight run: no serial port, radio or flight controller was opened, and
no RF was transmitted. Completing it would establish a bench-tested link, not
flight readiness.

## Replacement artifact gate — 2026-11-19

Do not use the earlier two-slot artifact
`5e4c3fd99679541b6c4fa10899b53335dbde64a5af40c0ae8bae156fd2e19b5d`.
It caused an RX LittleFS format failure and invalid-image panic.

Use the four-slot factory artifact whose manifest records SHA-256:

```text
2edc686e95af5ec933be8786d2605b8232d00697fd2ec52212131ae012a41cd5
```

Do not use `7210b9ff…`: it paired a DIO bootloader with QIO-built
applications and caused invalid flash reads, LittleFS failures and an RX panic.

The `2edc686e…` image must list `tx_1`, `rx_0`, `rx_1` and `role_state`, and
its manifest must record `bootloader_qio_80m.elf`, SDK memory type
`qio_qspi`, application header mode `dio` and bootloader mode `dio`.

This artifact uses the bench-confirmed G-R-B physical order. The final visual
gate is blue-only TX, green-only RX, one TX boot pulse and two RX boot pulses.

## Stop conditions

- Remove all propellers. Keep the aircraft restrained until electrical and
  failsafe checks are complete.
- Fit a suitable 2.4 GHz antenna or a 50 ohm terminated RF load before powering
  any TX or RX image. The diagnostic image may run on an open connector because
  it keeps both FEM enables low and never commands the SX1280 to transmit.
- Start at the lowest configured RF power. Increase it only after the conducted
  or terminated path has been checked.
- Stop if the role switch, selected application and RF front end do not agree.

## 1. Confirm the role switch by continuity

1. Power the board down and disconnect USB and the host adapter.
2. Check continuity from GPIO4 to ground and to 3.3 V in both switch positions.
   Record which position is electrically low and high.
3. Match the printed **TX** position to GPIO4 high and the printed **RX**
   position to GPIO4 low. The printed labels are authoritative for this board.
4. Label the verified positions before applying power.

Expected result: exactly one application role agrees in each switch position.

## 2. Verify the supplied images

From `firmware/elrs-usb-tx-rev-a3/`:

```bash
sha256sum -c dist/SHA256SUMS
```

Expected result: every recorded artifact verifies. Rebuild and repeat rather
than flashing a checksum failure.

## 3. Run the non-RF diagnostic

With the RF connector open and no TX/RX image active:

```bash
./scripts/flash.sh diagnostic /dev/ttyACM0
```

Open native USB serial and copy the diagnostic report. Expected minimum:

```text
DIAG usb=ok uart0_tx=43 uart0_rx=44
DIAG sx1280 spi=ok version=0xA9B7 busy=0 dio1=0
DIAG fem txen=0 rxen=0 final=shutdown
```

Record the GPIO4 role reading, FEM readback, strap values and any remote I2C
devices. Do not proceed if the SX1280 SPI check fails or either FEM enable
cannot be returned low.

## 4. Flash the single dual-role image

1. Attach an antenna or 50 ohm load.
2. Power off and set SW2 to either verified position.
3. Build and flash the single artifact:

   ```bash
   ./scripts/build.sh dual
   ./scripts/flash.sh /dev/ttyACM0
   ```

4. The board boots TX when GPIO4 is high and RX when GPIO4 is low.
5. To test the other role, power off, move SW2, and power on again. Do not
   reflash.

Expected result: one full-flash image installs both applications, native USB
remains available for configuration, and repeated reset/power-cycle tests
boot the role selected by GPIO4 every time. A role mismatch must leave both
FEM enables low; it must not transmit or receive.

After flashing, use `--after watchdog_reset` if the board remains in download
mode. A plain RTS reset can leave the boot strap latched. Verify the write
immediately after flashing; runtime LittleFS writes intentionally change one
flash region after boot, so a later whole-image verification is not expected
to remain byte-identical.

The factory image must report these offsets:

```text
tx_0      0x10000
tx_1      0x2b0000
rx_0      0x450000
rx_1      0x590000
role_state 0x6d0000
spiffs    0x6d2000
```

LittleFS must mount without a format error on first boot. The factory image
includes an empty filesystem at `0x6d2000`.

Wi-Fi OTA updates the inactive slot for the actual running role. Use a full
USB/ROM factory flash for the initial install or recovery.

## 5. Configure and bind over native USB

Connect each board's native USB CDC port and use the CRSF CLI/configurator
without a Wi-Fi dependency. Configure both boards with:

- the same binding phrase;
- the same regulatory domain, packet rate (initially 150 Hz) and telemetry
  ratio;
- the lowest available transmitter power for the first powered RF test.

Power the receiver first, then the transmitter, with a load on both RF ports.
Expected result: the receiver acquires the transmitter and reports link
statistics. A failure to bind is not diagnosed by raising power; check role,
domain, binding phrase and image first.

## 6. USB-only TX/RX link test

No J3 adapter and no Wi-Fi are required. With both boards on their native USB
ports, identify the two CDC devices:

```bash
ls -l /dev/serial/by-id/
```

Use the TX logical rate of 400000 and the RX logical rate of 420000 in the
tool or CLI. CDC ignores the baud setting, but the nominal value documents the
CRSF endpoint being emulated.

Expected USB behavior:

- TX receives valid `RC_CHANNELS_PACKED` frames from its host and starts RF
  scheduling on the first valid RC frame.
- TX returns `LINK_STATISTICS`, telemetry and device information on the same
  native USB stream.
- RX emits valid `RC_CHANNELS_PACKED` plus CRC to its USB host.
- Telemetry frames written by the RX host are forwarded over RF.
- The two devices remain distinct `/dev/ttyACM*` endpoints.

At boot, expect one blue pulse for TX or two green pulses for RX at the
configured low brightness, followed directly by the fixed role/state pattern.
The TX heartbeat is one pulse, not two. Stop if both LEDs are dark, colors or
pulse counts disagree with the selected role, or a slow red fault pattern
appears.

Use the LED state table in
[`elrs-led-state-machine-2026-11-19.md`](elrs-led-state-machine-2026-11-19.md).
There must be no rainbow, packet-rate hue, random gradient or unrelated-color
flicker after boot. Bind is slow role-color breathing, searching is faster
role-color breathing, and connected is steady in the role color. A model/role
mismatch is three amber pulses; a radio fault is fast red.

Open the host port with DTR and RTS deasserted. `aerial_kit.comms`
`SerialTransport` does this by default. If a tool nevertheless resets the
board while opening the port, resynchronize on CRSF sync byte `0xC8`; do not
treat ROM or bootloader text as a protocol frame.

## 7. Optional J3 UART test

Use a 3.3 V USB-UART adapter:

| Host | J3 |
| --- | --- |
| Adapter TX | J3.4 / GPIO44 |
| Adapter RX | J3.3 / GPIO43 |
| Ground | J3.2 / GND |

Do not connect J3.1 or J3.7 while the board is USB-powered. Run the backend only
after confirming the adapter is 3.3 V:

```bash
source ros2_ws/install/setup.bash
ros2 run hw_bridge crsf_radio_backend_node --ros-args \
  -p transport:=serial \
  -p serial_port:=/dev/ttyUSB0 \
  -p serial_baud:=400000 \
  -p airframe_name:=fpv_quad_5in
```

Keep `/uav/backend/enable` false and inspect the bytes on the UART or a protocol
analyzer. Expected result: valid 26-byte `RC_CHANNELS_PACKED` frames at the
configured rate, disarmed channel 6, minimum channel 3 throttle, and no serial
open/write errors. `/uav/radio/status` should report a stale telemetry link when
no receiver telemetry is connected.

Then drive an enable rising edge through the command stack. Expected result:
channel 6 (arm) goes high while channel 5 (mode) is high and channel 3 remains
at 172 for `arm_hold_sec` (1.0 s by default), followed by normal AETR sticks.
Force throttle above `throttle_low` before the hold completes and confirm the
FC/receiver does not arm. Disable must drive channel 6 and throttle low
immediately.

Unplug the USB-UART adapter while frames are running. Expected result:
`/uav/radio/status` reports the disconnect, the node retries without exiting, and
a reconnect begins another full arm hold at low throttle. It must not resume an
armed handover from the previous connection.

## 8. Restore the firmware HIL harness before FC-in-the-loop

This stage is currently blocked. The public firmware tree builds only the ARM
Feather F405 image; the Makefile has no host/SIL target. The historical notes
refer to these files and targets, but they are not present in this checkout:

- the host peripheral tests and models under `firmware/tests/` (`tests/` exists
  in this public tree, but only fixture data is retained);
- `firmware/tools/ak_control.h` and the shared control library it describes
  (`tools/akcontrol.py` is present, but cannot load the missing library);
- `make control-check`, `make control-experiment` and `make sil`;
- the `aerialkit-fw-sim` executable;
- the MuJoCo `ActuatorPlant`/IMU adapter used to exchange motor and sensor data
  with the firmware control core.

Do not invent a replacement in this checklist. Restore the retained development
tree or implement and test those host targets first. Then the intended HIL test
is: firmware mixer outputs drive the simulated motors, simulated IMU feeds the
firmware, start with hover, then small stick steps, and compare the plant's
attitude/position with a serial transcript. This overnight run did not perform
that test.

## 9. FC bench test with props off

1. Configure the FC receiver UART for CRSF at 420000 baud, 8N1, non-inverted.
2. Confirm AETR on channels 1-4, mode on channel 5 and arm on channel 6.
   AerialKit uses a 1300-count threshold. A Betaflight setup needs its AUX
   mapping changed to this order.
3. Connect the receiver's CRSF output to the FC and keep the FC outputs
   disconnected from ESCs if the board permits it; otherwise keep props removed
   and the aircraft restrained.
4. Arm and disarm repeatedly. Expected result: the FC changes armed state only
   while channel 6 requests arm and its own arming gates pass.
5. Move roll, pitch, yaw and throttle one at a time. Expected result: only the
   corresponding output or commanded attitude changes, with the documented
   sign. Confirm mode changes on channel 5. At yaw 90 degrees (nose north),
   command a world-north velocity demand and confirm it appears as
   body-forward motion, not body-right.
6. Compare handset/FC telemetry with independent instruments. Pitch must not be
   swapped with roll; battery volts/current and GPS latitude/longitude/speed
   must match their real magnitudes rather than byte-swapped nonsense.
7. Configure the FC failsafe before testing it. It must not hold the last command.
   For the AerialKit firmware, a quadrotor stops its motors and latches failsafe;
   a fixed wing circles down as documented in `docs/04-flight-core.md`. For
   another FC, use its controlled-land or motor-stop policy and record the exact
   settings.
8. While armed with outputs at a safe low level, remove transmitter RF or
   disconnect the CRSF signal. Expected result at the FC: a stale receiver/link
   indication followed by the configured failsafe action. The aircraft must not
   re-arm when the link returns; cycle the arm switch through off and on.
9. Repeat the link-loss test from a fresh arm cycle. Copy the FC console, UART
   counters, failsafe state and output/ESC signal evidence into a dated record.

## 10. Configurator binding and Wi-Fi workflow

Use the configurator's **Device → Radio module** page and the board's native
USB CDC path. The custom Rev A3 image carries CRSF over native USB in both
roles; a J3 adapter is optional:

- TX role: **USB serial TX (400000 baud)**;
- RX role: **USB serial RX (420000 baud)**.

Power off before changing SW2. GPIO4 is sampled at reset: high boots TX and low
boots RX. The combined image does not need to be reflashed to change roles.
The printed `TX`/`RX` labels are authoritative; verify the electrical position
by continuity as a wiring check. CDC ignores the selected baud. The role shown after connection
comes from the module's CRSF device information and should agree with GPIO4.

After **Connect radio** and **Allow changes**, open **Binding and Wi-Fi**. These
are three separate operations:

1. **Binding phrase / UID.** The phrase field shows the MD5-derived six-byte UID
   the module will actually store. Confirm a phrase or direct UID write, then
   expect the module to reboot. Reconnect and use **Read UID** to verify the
   readback; the phrase itself cannot be recovered from the module.
   The RX role's advertised **Bind Storage** setting remains separate under
   **Settings**; changing that storage policy is not a phrase/UID write.
2. **Explicit bind.** **Start binding mode** on a TX or **Enter Bind Mode** on an
   RX asks for confirmation before sending CRSF `COMMAND` subcommand
   `0x10/0x01`, the same request as
   `aerial-kit-link ... --write radio bind`. That command has no completion
   response, so sending it proves only that the request left the host. A
   receiver double-blink (100 ms on/off, then a 1 s pause, orange/red at the
   overlay's 10% brightness) is the RX-side binding evidence. Do not claim a
   successful bind until both endpoints connect and report link statistics.
3. **Wi-Fi.** **Enable module Wi-Fi** asks the TX to enter Wi-Fi update mode;
   **Enable receiver Wi-Fi** asks a connected TX to relay that request to the
   RX. Confirm the link-loss warning first. The fixed LED state machine does
   not use the upstream yellow-to-green cross-fade; keep role color as the
   identity and use the web page for mode confirmation. Join `ExpressLRS TX`
   or `ExpressLRS RX` with password `expresslrs`, then open
   `http://10.0.0.1/`. The configurator cannot switch the host's Wi-Fi network
   or embed that native page.

If a module's parameter menu does not advertise a bind entry or its role does
not advertise Wi-Fi, the page says so. The stock direct bind command remains
available, but do not infer Wi-Fi support from the role image alone.

Stop after the props-off FC and failsafe checks. This checklist does not
authorize a flight or claim that the link, tuning or airframe is flight-ready.
