# Protocol layer

`aerial_kit.comms` is the Python boundary between a program and the radios,
flight controllers and vehicles it talks to. It owns framing and protocol
semantics; command policies, estimators and airframe dynamics stay above this
layer.

## Layers

```text
application / CLI / mission script
              |
           clients                 typed operations and callbacks
              |
            codecs                 CRSF, AKP, MAVLink record framing
              |
           framing                 stream resynchronisation and reassembly
              |
          transports               USB serial, TCP, UDP, memory
```

Each layer has one job:

| layer | responsibility |
| --- | --- |
| transport | move byte arrays, report timeouts, reconnect after a serial failure |
| framing | find frame boundaries in a byte stream and reject bad lengths/checksums |
| codec | encode and decode protocol records without performing side effects |
| client | correlate requests and responses, expose telemetry callbacks, enforce write policy |

The dependency direction is downward only. A codec does not open a port, a
transport does not know a command number, and a high-level client does not
carry a second copy of a wire layout.

## Protocols

### CRSF / ELRS

CRSF is the control and telemetry protocol between an ELRS transmitter module, a
receiver and the flight controller. RC channel frames are fixed at 150 Hz by
default in `CrsfRadio`; telemetry is decoded incrementally. The 16 packed 11-bit
channels, telemetry frames, device information and parameter tree are validated
against ExpressLRS 4.1.0.

The host must be able to send sticks while receiving telemetry without blocking
either direction. A stalled host stops producing RC frames; the receiver or
flight controller then applies its own configured failsafe.

### Aerial Kit protocol (AKP)

AKP is the configuration and diagnostics protocol implemented by the flight
controller. Its frame is:

```text
AA 55  version  command  length  payload  crc16le
```

CRCs cover the version, command, length and payload. Replies set the top bit of
the command. Command `0x01` through `0x16` covers identity, parameters,
telemetry, logs, RC channels, sensors, outputs, preflight, calibration,
missions, performance and motor telemetry.

Typed clients surface refusals from the firmware instead of retrying them.
Writes require an explicit `allow_write=True`; construction of a client alone
never changes device state.

### MAVLink

MAVLink v2 is the wire protocol for PX4 and ArduPilot. The core codec needs no
external package. `pymavlink` is an optional dependency used for broader dialect
support and cross-checking vectors.

Vehicle commands, parameter writes, mission writes and setpoints are blocked by
default. A read-only client can heartbeat, identify the vehicle, request
telemetry and download parameters/missions. Mutating operations require both
`allow_write=True` and a success acknowledgement before they are reported as
applied.

## What runs over what

| route | transport | carried protocols |
| --- | --- | --- |
| host to ELRS transmitter module | USB CDC or USB-UART | CRSF |
| ELRS transmitter to receiver | RF, opaque to this layer | CRSF-compatible control/telemetry |
| receiver to flight controller | UART | CRSF |
| host to Aerial Kit flight controller | USB serial | AKP |
| host to simulator or bridge | TCP or UDP | AKP, MAVLink |
| host to PX4/ArduPilot companion link | serial, TCP or UDP | MAVLink |
| host to Aerial Kit FC through ELRS | RF via CRSF envelopes | AKP tunnel plus RC |
| tests | memory/loopback | any supported protocol |

## Aerial Kit over the radio

ELRS remains the RF waveform. AKP is carried inside CRSF's MSP envelope:

| direction | CRSF type | nominal payload budget |
| --- | --- | --- |
| uplink request | `0x7A` | 8 bytes |
| uplink write | `0x7C` | 8 bytes |
| downlink response | `0x7B` | 58 bytes |

AKP frames are segmented because an AKP frame can be much larger than one CRSF
frame. The provisional tunnel header is:

```text
A1  session  seq_flags  total_length  chunk...
```

`seq_flags` uses the low six bits for sequence, bit 6 for first segment and bit
7 for last. A receiver accepts one active session, deduplicates sequence
numbers, reassembles by sequence, and rejects a transfer with a missing first
or final segment. The radio scheduler gives RC and attitude telemetry priority;
AKP traffic is rate-limited and must never starve either.

The pinned ExpressLRS source defines the MSP frame types and payload bounds. The
TX/RX forwarding direction must be verified in source and in the simulated
radio before this transport is marked supported. If forwarding does not hold,
the fallback is ELRS serial passthrough ("Airport"), which carries AKP directly
but gives up simultaneous RC on that serial link. That fallback must be
explicitly selected and documented at the client.

## Safety rules

- No hardware access happens during import or client construction.
- Opening a transport does not transmit.
- Writes, calibration, output tests, flashing and vehicle commands are opt-in
  methods and covered by refusal tests.
- An armed-state refusal from AKP is returned as a typed error and is never
  retried.
- MAVLink write methods require `allow_write=True`; acknowledgements are
  correlated by command and sequence.
- A timeout is not permission to repeat a mutation. Only read operations retry.
- CRSF RC generation must fail safe by stopping frames, not by manufacturing
  neutral sticks indefinitely.

## Verification boundary

Protocol vectors are generated from source definitions and checked without
hardware. Serial reconnect, TCP/UDP timeouts, packet loss, CRSF segmentation,
MAVLink request/response handling and AKP refusal paths are exercised with
memory transports or fake devices.

Nothing in this layer claims a real radio link, flight controller or vehicle
has been tested. Hardware bench procedures remain a separate checklist and
must include a dummy load or antenna, lowest transmit power, props off, and
explicit operator approval before any RF or motor action.
