# Rev A3 RX startup hang fix — 2026-11-19

## Failure

Candidate `b3f6d1b0…` booted the RX application with GPIO4 low and produced two
green boot pulses, followed by a dark LED. Native USB enumerated, but no
`AK-BOOT` line or CRSF `DEVICE_INFO` response appeared. No new coredump was
created.

## Root cause

There were two application-side FIFO/role guards:

1. `aerialkitBootPreflight()` used the shared inverted
   `aerialkit_role_matches()` and passed correctly.
2. `setupTargetCommon()` in upstream `rxtx_common.cpp` had a separate
   `enforceAerialKitRoleSelect()` with the old direct comparisons:

```text
TX expected GPIO4 low
RX expected GPIO4 high
```

The RX reached the second guard after its boot pulses, failed the stale check,
held both FEM enables low, and entered its permanent `delay(1000)` loop. That
is before `setupSerial()` and `aerialkitBootReport()`, which explains the
enumerating USB controller with no application output or coredump.

## Fix

The upstream guard now calls the same shared role predicate as the preflight:

- TX calls `aerialkit_role_matches(roleHigh, false)`;
- RX calls `aerialkit_role_matches(roleHigh, true)`.

The duplicate truth table is gone. A regression test checks that both call
sites use the shared predicate and that the old direct expressions are absent.

The labeled role contract remains:

```text
GPIO4 high -> TX
GPIO4 low  -> RX
```

The confirmed G-R-B LED mapping and all state patterns are unchanged.

## Frozen artifact

```text
path: firmware/elrs-usb-tx-rev-a3/dist/aerialkit-rev-a3-elrs-dual-role.bin
size: 8331264 bytes
sha256: 7c6714425e4a2981ef787267df89627838e61b155e288536e66cf465faeada63
timestamp: 2026-11-19 18:30:42 -0800
flash offset: 0x0
bootloader: bootloader_qio_80m.elf / qio_qspi
application header mode: dio
build inputs: fb39e24624de7f2a813136782108917bb7b58ab599f04ce5aa9eff82f773ceca
```

Application hashes:

```text
TX bin: 9c9118fb3ed41d66aabf4b652fdd584a3b9635bf0441214368062f09a529bc08
RX bin: 34836893ede92096476f8945e78a0193150bb4fb20a7998da021691c0eb0b4ec
TX ELF: 1b232841e53b4545181381774104f79661d968c4f704d532baa40249f39f8691
RX ELF: 4307fc245b540bf5e322d1ab6d31c37f3f7272b76b444d0a397165fbca65536b
```

Release identity: `releases/7c671442/`.

## Validation

- TX and RX PlatformIO builds passed.
- Native USB/LED tests: 9 passed.
- Firmware Python tests after the final build: 25 passed.
- The regression checks both role guards and the generated manifest.
- Manifest, application/ELF hashes and `SHA256SUMS` agree.

## Hardware gate

Test both labeled positions with this exact artifact:

- printed TX, GPIO4 high: one blue pulse, blue-only states, `AK-BOOT`
  reports `role=tx gpio4=high expect_gpio4=high`;
- printed RX, GPIO4 low: two green pulses, green-only states, `AK-BOOT`
  reports `role=rx gpio4=low expect_gpio4=low`, and CRSF `DEVICE_INFO`
  returns at 420000;
- no role mismatch loop, panic or coredump;
- RADIO/SX1280 reports healthy and both FEM enables remain low on any actual
  mismatch.

This artifact is frozen pending that both-role hardware result.
