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

Use antennas or 50-ohm loads and minimum RF power for physical testing. Do not
change the Rev A3 role switch while powered.

## Current Rev A3 candidate

The current locally generated full factory image observed on 2026-11-19 is:

```text
size: 8331264 bytes
sha256: 5e9d6c6165bd2698239931da49f2bb1a7043b5847eb1ca764d8e72f6212ebb5e
flash offset: 0x0
bootloader: bootloader_qio_80m.elf / qio_qspi
```

The manifest and `SHA256SUMS` agree. See
[`evidence/rev-a3-usb-bench-artifact-2026-11-19.md`](evidence/rev-a3-usb-bench-artifact-2026-11-19.md).
This is a candidate identity, not hardware acceptance. The firmware owner must
freeze the artifact before the bench operator flashes it.
