# ESP-NOW custom protocol

This documents the wire format implemented by `src/protocol.h` / `src/protocol.cpp`.
It is the custom over-the-air protocol used by the ESP-NOW TX and RX.

## Frame layout

All multi-byte fields are little-endian on the wire.

```text
offset  size  field
0       1     magic (0xA5)
1       1     version (0x01)
2       1     packet type
3       1     sequence
4       2     payload length
6       N     payload (N = payload length, max 128)
6+N     2     crc16
```

`crc16` is the CCITT-FALSE CRC (poly `0x1021`, init `0xFFFF`) over all bytes from
offset 0 through the end of the payload (i.e. excluding the two CRC bytes themselves).

## Packet types

| Value | Name | Payload |
|-------|------|---------|
| `0x10` | `PKT_RC_COMMAND` | `RcCommandPayload`: 16 x 11-bit CRSF channels, timestamp, flags |
| `0x11` | `PKT_DIRECT_CMD` | `DirectCommandPayload`: roll/pitch/yaw/throttle, timestamp |
| `0x20` | `PKT_TELEMETRY_REQ` | none |
| `0x21` | `PKT_TELEMETRY_DATA` | `TelemetryPayload`: voltage, current, RSSI, link quality, timestamp |
| `0x30` | `PKT_CONFIG` | reserved |
| `0xF0` | `PKT_ACK` | none |
| `0xF1` | `PKT_NACK` | none |
| `0xFF` | `PKT_HEARTBEAT` | none |

## Sequence and failsafe

The sender increments `sequence` for every frame. The receiver drops duplicate deliveries
that repeat the most recently seen sequence number and counts them in `ProtocolStats.
duplicates`. Link timeout is 1000 ms: after that the RX failsafes to center sticks,
minimum throttle, and disarm AUX channels.

## Future improvements

- Reject stale/duplicate sequence numbers per sender MAC (single-sender dedup is done).
- Add ACK/NACK retry handling for telemetry requests and configuration.
- Move the shared CRC/frame code into a `firmware/lib/crsf`-style common library once the
  firmware merge decision is made.
