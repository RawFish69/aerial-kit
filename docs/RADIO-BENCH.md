# Native-USB CRSF radio bench

`aerial-kit-link` provides bounded, reusable CRSF bench operations over the
shared `aerial_kit.comms.crsf` codec. The USB-only Rev A3 bench uses native
USB-CDC on both endpoints:

- TX logical host rate: 400000 baud.
- RX logical FC-facing rate: 420000 baud.
- CDC may ignore baud; the logical value names the endpoint behaviour.
- Serial ports are opened with DTR and RTS deasserted.
- Decoders resynchronize on CRSF address byte `0xC8`.

ROM or bootloader text is not CRSF. A port that resets while opening may emit
that text first; the decoder discards it until a valid frame begins.

## Commands

```bash
# A real DEVICE_INFO response is required.
ak radio status --port /dev/ttyACM0 --baud 400000

# Read the MSP RXTX UID with a four-second window and one post-reconnect retry.
ak radio uid --port /dev/ttyACM0 --baud 400000 --role tx
ak radio uid --port /dev/ttyACM1 --baud 420000 --role rx

# Force a reset/open and capture the role's RF boot identity: UID seed,
# packet-rate enum, FHSS first channel, initial frequency and FEM pin state.
ak radio diag --port /dev/ttyACM0 --baud 400000 --role tx --json tx-boot.json
ak radio diag --port /dev/ttyACM1 --baud 420000 --role rx --json rx-boot.json

# During a drive/receive run, sample GPIO6/7 transitions, SX1280 IRQ state,
# instantaneous RSSI and the last packet statistics once per second.
ak radio diag --port /dev/ttyACM1 --role rx --duration 12 \
  --watch-rf --json rx-rf.json

# Set one shared phrase on each endpoint. Each command reads the old UID,
# writes, reconnects, and refuses success without an exact six-byte readback.
ak radio bind-phrase replace-with-your-phrase \
  --port /dev/ttyACM1 --baud 420000 --role rx --write
ak radio bind-phrase replace-with-your-phrase \
  --port /dev/ttyACM0 --baud 400000 --role tx --write

# A direct six-byte UID is also supported and has the same gates/readback.
ak radio set-uid AA:BB:CC:DD:EE:FF \
  --port /dev/ttyACM0 --baud 400000 --role tx --write

# Decode all valid CRSF traffic and preserve counters as JSON evidence.
ak radio monitor \
  --port /dev/ttyACM0 --baud 400000 --duration 5 --json monitor.json

# Send 26-byte RC_CHANNELS_PACKED frames at 150 Hz.
ak radio drive \
  --port /dev/ttyACM0 --baud 400000 \
  --hz 150 --duration 10 --pattern arm-test --write --json drive.json

# A Betaflight vehicle with arm on AUX1/CH5 and its mode switch on CH7.
ak radio drive \
  --port /dev/ttyACM0 --baud 400000 \
  --profile betaflight --pattern arm-test --write

# Hold an explicit roll,pitch,yaw,throttle setpoint after the arm sequence.
# This is bounded bench control and always sends a safe throttle-low/arm-low
# sequence on exit.
ak radio drive \
  --port /dev/ttyACM0 --profile betaflight --duration 2 \
  --sticks 0.2,0,0,0.35 --confirm-arm --write --json stick-test.json

# Bounded live keyboard control through the transmitter. The stream starts
# disarmed; press Enter to arm after the low-throttle hold.
ak radio teleop \
  --port /dev/ttyACM0 --profile betaflight --duration 30 \
  --write --confirm-arm --json teleop.json

# Capture FC-facing receiver frames and channels.
ak radio capture \
  --port /dev/ttyACM1 --baud 420000 --duration 10 --json capture.json

# Inject validated attitude, battery, GPS and barometer telemetry.
ak radio inject \
  --port /dev/ttyACM1 --baud 420000 \
  --file protocols/vectors/crsf.json --write --json inject.json

# The bench first proves that TX and RX report the same non-zero UID.
# RF PASS still requires observed RX RC and link statistics.
ak radio bench \
  --tx /dev/ttyACM0 --rx /dev/ttyACM1 \
  --duration 15 --json bench.json --write
```

The guided pair command performs both phrase writes/readbacks and then the
two-board bench as one bounded run:

```bash
ak radio pair --tx /dev/ttyACM0 --rx /dev/ttyACM1 \
  --phrase "replace-with-your-phrase" --duration 15 \
  --json pair.json --write
```

Use `--json -` for JSON on standard output. Human output remains a compact
summary; JSON is stable and sorted.

The complete two-endpoint acceptance helper is:

```bash
python3 firmware/tools/verify_uid_binding.py \
  --tx /dev/ttyACM0 --rx /dev/ttyACM1 \
  --phrase "replace-with-your-phrase" --write --confirm --rf-duration 15 \
  --json uid-binding.json
```

Omit `--write --confirm` for a read-only UID comparison. With both flags, the
helper writes the same phrase to RX and TX sequentially, waits for each reboot,
requires exact readback from both, and only then starts the two-board drive.

## Evidence rules

`radio drive` establishes only that valid RC frames were written and that the
configured arm sequence completed. It reports received telemetry and link
statistics if observed, but its TX-only result leaves RX capture and RF link
as `NOT TESTED`.

`radio capture` fails when no valid RC frame arrives. It distinguishes:

- no bytes;
- resynchronization/framing traffic without valid RC;
- CRC errors;
- valid RC frames and their cadence.

When RF does not acquire, compare `radio diag` from both roles before changing
power or antennas. `uid`, `seed`, `crc`, `rate_enum`, `fhss0` and `freq` must
describe the same 2.4 GHz configuration. RX is expected to cycle
`rate_enum` while searching; TX reports the configured rate it is actually
transmitting. `txen`/`rxen` describe the FEM pins at boot, and `conn` is the
ExpressLRS connection-state enum.

`radio inject` rejects malformed, partial, non-telemetry and mutation frames.
Injection proves the sequence was sent, not that RF forwarded it.

`radio bench` marks `rf_link` PASS only when receiver RC frames and link
statistics are observed. Command transmission alone cannot produce RF PASS.
A single transceiver may still pass USB identity and device-information checks,
but cannot establish a two-endpoint RF link.

## Safety

All commands that transmit RC, arm, inject or run a two-board bench require
`--write`. `radio bench`
refuses mismatched or all-zero endpoint UIDs before transmitting RC. Arm
patterns wait at low throttle for longer than the firmware's
500 ms arm hold. Every drive exit path, including timeout and Ctrl-C, sends a
throttle-low/armed frame followed by an arm-low frame before returning.

`--profile aerial-kit` keeps the Aerial Kit default of arm on CH6 and mode on
CH5. `--profile betaflight` uses the bench-verified arm default of CH5 and the
workflow's mode default of CH7, which must still be checked against the
vehicle's Betaflight modes tab. `--arm-channel`, `--mode-channel` and
`--velocity-channel` override those defaults for a particular vehicle.

`--sticks` is a second explicit control gate on top of `--write`: it also
requires `--confirm-arm`, waits for a throttle-low arm hold, then holds the
requested normalised values for the bounded duration. Do not use the bounded
bench command as an in-flight stick controller; its exit path disarms.

`radio teleop` is the live keyboard path. A/D, W/S and Q/E step roll, pitch and
yaw, R/F change throttle, C centres all sticks, and X panic-disarms and exits.
Ctrl-C, timeout, panic and transport failure all attempt a final disarmed
frame. It requires an interactive terminal plus `--write --confirm-arm`; the
command starts disarmed and Enter is still required before the arm hold. A
sent arm request is not evidence that the flight controller armed.

Use antennas or 50-ohm loads and minimum RF power for physical testing. Do not
change the Rev A3 role switch while powered.

## Current Rev A3 final color artifact

The bench-confirmed G-R-B operational order is frozen in this final color
artifact:

```text
size: 8331264 bytes
sha256: 7c6714425e4a2981ef787267df89627838e61b155e288536e66cf465faeada63
flash offset: 0x0
bootloader: bootloader_qio_80m.elf / qio_qspi
```

The final identity is recorded in
[`evidence/rev-a3-rx-role-hang-fix-2026-11-19.md`](evidence/rev-a3-rx-role-hang-fix-2026-11-19.md).
The remaining hardware gate is visual confirmation of one blue TX boot pulse at
GPIO4 high, two green RX boot pulses at GPIO4 low, blue-only TX states and
green-only RX states. The superseded role record is
[`evidence/rev-a3-role-inverted-final-2026-11-19.md`](evidence/rev-a3-role-inverted-final-2026-11-19.md).
