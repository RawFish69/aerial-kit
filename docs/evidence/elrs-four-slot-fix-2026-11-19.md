# Rev A3 ELRS four-slot factory-image fix — 2026-11-19

## Trigger

The first dual-role 0.3.0-era artifact used the stock two-application
partition table. On hardware, the RX application entered a LittleFS format
failure and then the ESP image loader reported an invalid image at
`0x1f0000`. Both LEDs remained off because the failed RX application never
reached LED initialization.

RF testing was stopped. No hardware action was taken while producing this fix.

## Corrected layout

The factory image now uses the four-slot layout:

```text
tx_0        0x10000   1664K
tx_1        0x2b0000  1664K
rx_0        0x450000  1280K
rx_1        0x590000  1280K
role_state  0x6d0000  8K
spiffs      0x6d2000  1152K
```

The bootloader samples GPIO4 at reset, reads the active TX/RX slot from two
independent role-state records, and falls back only to the other slot for the
same role. The factory image writes `tx_0`, `rx_0`, both role-state records and
an empty LittleFS filesystem. The `tx_1` and `rx_1` OTA slots are erased.

## Validation

- ExpressLRS 4.1.0 TX build: 1,484,325 bytes, limit 1,703,936 bytes.
- ExpressLRS 4.1.0 RX build: 1,121,357 bytes, limit 1,310,720 bytes.
- Factory artifact: 8,331,264 bytes.
- SHA-256:
  `033d66391ecd01d91964af802688e92102035d9624ca04bc8b81db86f593ee64`.
- Partition-table offsets agree with the source CSV and the manifest.
- Both role-state records have valid magic, sequence, CRC, TX slot 0 and RX
  slot 2.
- `mklittlefs -l` reads the embedded filesystem.
- Firmware tests: 3 passed.

## Bench gate

The corrected image must be flashed to both boards from offset `0x0`. Do not
reuse the previous artifact. Expected boot evidence:

```text
no esp_littlefs format error
TX_0 or RX_0 selected according to the verified GPIO4 level
valid application magic at the selected slot
LED initialization follows application startup
```

Any invalid image magic or LittleFS error remains a stop condition.
