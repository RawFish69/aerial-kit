# Rev A3 UID/binding fix — 2026-11-19

## Findings

The hardware RX UID response was valid. The tools rejected it because
ExpressLRS uses two different inner checksums:

- CRSF `0x7A` MSP-v2 UID requests use DVB-S2;
- MSP-v1 `0x7B` responses and `0x7C` writes use XOR.

Python and the web configurator applied DVB-S2 to responses and writes. This
made valid RX/TX UID responses look like timeouts and made direct UID/phrase
writes invalid to the firmware. It also explains identical RX zero-UID reads
after a write and TX appearing silent while it continued accepting RC frames.

Python parameter reads had a second defect: the client concatenated repeated
`fieldId,chunksRemaining` network headers into the parameter body. Multi-frame
parameter tables could therefore fail as "not fully reassembled". The client
now keys chunks by `chunksRemaining`, retries missing requests, reports the
missing index, and synthesizes exactly one record header.

The firmware setter also marked RX configuration dirty and scheduled a reboot
without committing it first. The new patch commits the direct UID or phrase
before reboot.

## Frozen artifact

```text
path: firmware/elrs-usb-tx-rev-a3/dist/aerialkit-rev-a3-elrs-dual-role.bin
size: 8331264 bytes
sha256: 96553fa544ed40ac3ce791157429a1072adab6ec632f5a9c2277636358e7837b
flash offset: 0x0
bootloader: bootloader_qio_80m.elf / qio_qspi
TX application: f6d7ee1e28a05ff4666aae26c9fcb8c3339cb7c499f1aa1b81247318d38880e6
RX application: 5656c97506d1a67a56bcdd2053ed849ad25f2e244c523342f7cbe3eb712204a6
```

The identity and application hashes are recorded in
`firmware/elrs-usb-tx-rev-a3/releases/96553fa5/`.

## Verification

- Firmware Python host suite: 25 passed, 1 skipped before the final rebuild.
- ExpressLRS native suite: 104 passed.
- The native routing test drives two RC frames around an addressed TX UID
  request and requires the `0x7B` response to reach the USB connector.
- Python protocol tests use the exact hardware RX UID response and independent
  frame bytes.

## Hardware status

No hardware was flashed or opened during this work. Hardware PASS remains
blocked until the bench reports:

1. both boards flash `96553fa5…` and verify the data hash;
2. `ak radio uid` reads both endpoints without timeout;
3. `ak radio pair` writes one phrase and reads the same non-zero UID from both;
4. RX RC capture succeeds;
5. link statistics are observed from both directions.

PR #59 remains draft until those results exist.
