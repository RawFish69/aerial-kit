# Rev A3 labeled-role final artifact — 2026-11-19

## Result

The Rev A3 printed switch labels are authoritative:

```text
GPIO4 high -> TX
GPIO4 low  -> RX
```

The inversion is applied at every role decision point:

- the bootloader selects `tx_0`/`tx_1` when GPIO4 is high and
  `rx_0`/`rx_1` when GPIO4 is low;
- the compiled TX application expects GPIO4 high;
- the compiled RX application expects GPIO4 low;
- either application holds both FEM enables low when the sampled GPIO4 level
  disagrees with its compiled role;
- the artifact manifest records `gpio4_low=rx`, `gpio4_high=tx`;
- `AK-BOOT` prints both the observed GPIO4 level and the expected level.

The bench-confirmed G-R-B WS2812 mapping is unchanged. Role colors, boot pulse
counts, bind/searching timing, connected behavior, red fault and amber
mismatch patterns are also unchanged.

## Frozen final artifact

```text
path: firmware/elrs-usb-tx-rev-a3/dist/aerialkit-rev-a3-elrs-dual-role.bin
size: 8331264 bytes
sha256: b3f6d1b0948be120aa920da34957ab0b01cf397e238bb459d427583ccbf7af43
timestamp: 2026-11-19 18:05:27 -0800
flash offset: 0x0
bootloader: bootloader_qio_80m.elf / qio_qspi
application header mode: dio
build inputs: 1e194a1e8fc1ef5369ae422b72b7b9d6c5268b57b5744359f00c74f1b67b5371
```

Application hashes:

```text
TX bin: 68e32f2a764be212f56f01e478dfc5e90f875e9e06a7d0550947ff5f48be7cc7
RX bin: 83fbe9c78235a9dffcb2637f4a11fcc9b956f7f538ceca57f903820001ebe4da
TX ELF: 4fee7cf2168f4369c582ea9fa1c1678417d51487659c4ea9d345cb284a142a44
RX ELF: 4e6bfb02842195362ad3e8c85b970b66d1ed85c9920c7dbe6fb89499067c5ef2
```

The release identity is recorded in `releases/b3f6d1b0/`.

## Validation

- TX and RX PlatformIO builds passed.
- Firmware Python tests after the final build: 25 passed.
- Native USB/LED state tests: 9 passed, including the inverted application
  mismatch truth table.
- Manifest role mapping and factory artifact agreement are tested.
- The linked applications retain the checked RMT/G-R-B path and contain no
  upstream HSV, rainbow, fade or I2S LED symbols.

## Hardware gate

Flash the exact `b3f6d1b0…` image at `0x0` and verify:

- printed TX position, GPIO4 high: one blue boot pulse, then blue-only states;
- printed RX position, GPIO4 low: two green boot pulses, then green-only
  states;
- removing or contradicting the selected role does not boot the opposite RF
  role;
- a role mismatch holds both FEM enables low;
- `AK-BOOT` agrees between role, slot, `gpio4`, and `expect_gpio4`;
- no rainbow, mixed or unrelated colors appear;
- `led=rmt-ok led_failures=0 led_error=0x0`.

This artifact is frozen for that validation. It is not a full hardware PASS
until the operator records the labeled-position checks.
