# Rev A3 native-USB candidate history — 2026-11-19

This candidate was superseded by the final RMT-LED artifact. It is retained as
history because the candidate changed during concurrent bench and firmware
work; it is not approved for flashing.

```text
path: /home/cy/Projects/aerial-kit-dual-role-v2/firmware/elrs-usb-tx-rev-a3/dist/aerialkit-rev-a3-elrs-dual-role.bin
size: 8331264 bytes
sha256: 5e9d6c6165bd2698239931da49f2bb1a7043b5847eb1ca764d8e72f6212ebb5e
flash offset: 0x0
bootloader: bootloader_qio_80m.elf
bootloader SDK type: qio_qspi
application header flash mode: dio
```

The artifact manifest has the same SHA-256 and records the unchanged role
layout:

```text
tx_0       0x10000
tx_1       0x2b0000
rx_0       0x450000
rx_1       0x590000
role_state 0x6d0000
spiffs     0x6d2000
```

`sha256sum -c dist/SHA256SUMS` passed from the firmware artifact directory.

The current approved build identity is:

```text
sha256: 5fbc25c9d23b32b33859a7ae02fce5f69aee718cb60c217850d241fc45317838
```

See `rev-a3-rmt-final-2026-11-19.md` for the final evidence. Hardware acceptance
must still confirm:

- clean boot with no invalid-image, LittleFS or panic messages;
- GPIO4 low: one blue TX pulse and TX operational patterns;
- GPIO4 high: two green RX pulses and RX operational patterns;
- two distinct native-USB endpoints;
- TX RC input and DEVICE_INFO/LINK_STATISTICS/telemetry output;
- RX RC output and telemetry input/downlink;
- LINK_STATISTICS from both endpoints before an RF PASS claim.

Do not flash any older hash. Do not rebuild after the bench handoff freezes
the artifact, because the exact binary identity is part of the evidence.
