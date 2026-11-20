# Rev A3 final GRB LED artifact — 2026-11-19

Superseded: this artifact preserves the confirmed G-R-B mapping but retains
the old GPIO4 low=TX/high=RX role contract. It must not be used for labeled
hardware. The role-inverted replacement is recorded in
`rev-a3-role-inverted-final-2026-11-19.md`.

## Result

The bench completed the six-order physical WS2812 diagnostic and confirmed
that the assembled Rev A3 LED channel order is G-R-B. The operational mapping
is now:

```cpp
AERIALKIT_WS2812_OPERATIONAL_ORDER = {
    1, // green
    0, // red
    2, // blue
};
```

This changes the physical wire order only. GPIO4 low=TX/high=RX, the LED state
decision table, boot pulse counts, bind/searching timing, connected behavior,
red fault and amber mismatch colors are unchanged.

The explicit all-six-order diagnostic remains available for board bring-up.
It does not affect the operational GRB path.

## Frozen final artifact

```text
path: firmware/elrs-usb-tx-rev-a3/dist/aerialkit-rev-a3-elrs-dual-role.bin
size: 8331264 bytes
sha256: d71f5fc2aca3f0bfe4aa594c9321d7faaf01f23be722add044df656f1bf43a24
timestamp: 2026-11-19 17:45:48 -0800
flash offset: 0x0
bootloader: bootloader_qio_80m.elf / qio_qspi
application header mode: dio
build inputs: e4bcb0519289ac754b069a747bb73f521525f65db0f02093d2580e7597e3323d
```

Application hashes:

```text
TX bin: 8b9b481a02f5c1d85492c48201f1155f810d0f8715c14d6fe751ee05f33d0c86
RX bin: a7d5e19bbc2a6fbb2becf3b97edd1a7a8adb0ce31d07ae3c5b176a125c07fb39
TX ELF: b1603b847c590f0b04fe607d7a3c3e822e2a4470564082bc29a33139845a0679
RX ELF: e0e2f689c9c58d7c6a82df71d4ccfe0d004d4e751c1d9a00fa76d19fdee69f6d
```

The release identity is recorded in `releases/d71f5fc2/`.

## Validation

- TX and RX PlatformIO builds passed.
- Firmware Python tests after the final build: 24 passed.
- Native USB/LED state tests: 9 passed.
- The linked applications contain the checked RMT path and no upstream HSV,
  rainbow, brightness-fade or I2S LED symbols.
- Manifest, application/ELF hashes and `SHA256SUMS` agree.

## Final hardware gate

Flash the exact `d71f5fc2…` image at `0x0` and verify:

- TX on GPIO4 low: one blue boot pulse, then blue-only state patterns.
- RX on GPIO4 high: two green boot pulses, then green-only state patterns.
- Red fault and amber mismatch patterns remain distinct and unchanged.
- No rainbow, mixed or unrelated colors appear.
- `AK-BOOT` reports `led=rmt-ok led_failures=0 led_error=0x0`.

This artifact is frozen for that validation. It is not a full hardware PASS
until those visual checks are recorded by the operator.
