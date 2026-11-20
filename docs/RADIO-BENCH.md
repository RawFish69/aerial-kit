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
aerial-kit-link radio status --port /dev/ttyACM0 --baud 400000

# Decode all valid CRSF traffic and preserve counters as JSON evidence.
aerial-kit-link radio monitor \
  --port /dev/ttyACM0 --baud 400000 --duration 5 --json monitor.json

# Send 26-byte RC_CHANNELS_PACKED frames at 150 Hz.
aerial-kit-link radio drive \
  --port /dev/ttyACM0 --baud 400000 \
  --hz 150 --duration 10 --pattern arm-test --write --json drive.json

# Capture FC-facing receiver frames and channels.
aerial-kit-link radio capture \
  --port /dev/ttyACM1 --baud 420000 --duration 10 --json capture.json

# Inject validated attitude, battery, GPS and barometer telemetry.
aerial-kit-link radio inject \
  --port /dev/ttyACM1 --baud 420000 \
  --file protocols/vectors/crsf.json --write --json inject.json

# A two-board run; RF PASS still requires observed link statistics.
aerial-kit-link radio bench \
  --tx /dev/ttyACM0 --rx /dev/ttyACM1 \
  --duration 15 --json bench.json --write
```

Use `--json -` for JSON on standard output. Human output remains a compact
summary; JSON is stable and sorted.

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

`radio inject` rejects malformed, partial, non-telemetry and mutation frames.
Injection proves the sequence was sent, not that RF forwarded it.

`radio bench` marks `rf_link` PASS only when receiver RC frames and link
statistics are observed. Command transmission alone cannot produce RF PASS.
A single transceiver may still pass USB identity and device-information checks,
but cannot establish a two-endpoint RF link.

## Safety

All commands that transmit RC, arm, inject or run a two-board bench require
`--write`. Arm patterns wait at low throttle for longer than the firmware's
500 ms arm hold. Every drive exit path, including timeout and Ctrl-C, sends a
throttle-low/armed frame followed by an arm-low frame before returning.

Use antennas or 50-ohm loads and minimum RF power for physical testing. Do not
change the Rev A3 role switch while powered.

## Current Rev A3 final color artifact

The bench-confirmed G-R-B operational order is frozen in this final color
artifact:

```text
size: 8331264 bytes
sha256: b3f6d1b0948be120aa920da34957ab0b01cf397e238bb459d427583ccbf7af43
flash offset: 0x0
bootloader: bootloader_qio_80m.elf / qio_qspi
```

The final identity is recorded in
[`evidence/rev-a3-led-grb-final-2026-11-19.md`](evidence/rev-a3-led-grb-final-2026-11-19.md).
The remaining hardware gate is visual confirmation of one blue TX boot pulse at
GPIO4 high, two green RX boot pulses at GPIO4 low, blue-only TX states and
green-only RX states. The authoritative role-inversion record is
[`evidence/rev-a3-role-inverted-final-2026-11-19.md`](evidence/rev-a3-role-inverted-final-2026-11-19.md).
