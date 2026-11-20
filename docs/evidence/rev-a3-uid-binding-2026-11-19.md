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

### RF diagnostic candidate

The `96553fa5…` image passed UID binding on hardware but failed RF
acquisition. Both endpoints read `e9:77:41:50:7c:5f`; TX drove 4500 RC frames
in 30 seconds, but RX captured no RC, every LQ/RSSI/SNR field was zero, TX
reported RF mode 27, and RX cycled modes 21 through 30.

The next diagnostic candidate adds UID seed, CRC initializer, rate index/enum,
FHSS first channel, initial frequency, connection state and FEM pin state to
the `AK-BOOT` line:

```text
path: firmware/elrs-usb-tx-rev-a3/dist/aerialkit-rev-a3-elrs-dual-role.bin
size: 8331264 bytes
sha256: bfff8af53735af248cd82240b8a3ff6fa838f8f1a2fcf7f97fdf0cdfba6665d7
TX application: ec2c2803f95474bb56a88b45b8379eff0906d2e59712d53eac5a968bef81e7b9
RX application: 76f6dcdbd5e05b5b356b8b11bc5900c4a46d998a2959aacdec06ef5cc6c9e80b
```

The companion 0.4.3 client rejects `chunksRemaining=0xff` as an out-of-range
stale chunk, validates response addressing, preserves adjacent CRSF frames
from the same USB read, addresses RX parameters to `0xEC`, and paces every
successive chunk before retrying missing chunks. Its `ak radio diag` command
captures `AK-BOOT` as JSON.

The next candidate aligns RX's initial search mode with TX's default LoRa
250 Hz mode. `bfff8af5…` exposed TX `rate_idx=6/enum=27/4000us` and RX
`rate_idx=0/enum=33/1000us`; the RX sweep did not acquire before the 30-second
run ended. The replacement artifact is:

```text
path: firmware/elrs-usb-tx-rev-a3/dist/aerialkit-rev-a3-elrs-dual-role.bin
size: 8331264 bytes
sha256: 70de34ee438385f40c8b53a51e507ea48012c478e07499b0065e1a5a78fefd9d
TX application: fa0b1a0dfd8c5f13164fb1de4d830c121c4d50e4f340cc09fa7018238c421763
RX application: 8fdf39b9ce67628adba804e0b92c88b6c4f6bbc27b7681f5656ce0ea243da5c0
```

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

1. both boards flash `70de34ee…` and verify the data hash;
2. `ak radio diag` shows matching `uid`, `seed`, `crc`, `rate_enum`, `fhss0`
   and `freq`, with the expected FEM pin for each role;
3. `ak radio uid` reads both endpoints without timeout;
4. `ak radio pair` writes one phrase and reads the same non-zero UID from both;
5. `ak radio params` completes against both roles;
6. RX RC capture succeeds;
7. link statistics are observed from both directions.

PR #59 remains draft until those results exist.
