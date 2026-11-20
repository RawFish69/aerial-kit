# AK USB ELRS Rev A3

This directory freezes the hardware-tested `AK USB ELRS Rev A3` artifact built
with upstream ExpressLRS 4.1.0.

## Artifact

```text
path: firmware/elrs-usb-tx-rev-a3/dist/aerialkit-rev-a3-elrs-dual-role.bin
size: 8331264 bytes
sha256: 9b48f01f522783d872b72f36c69c7171bcc851e2fcbfe5f60cb717efcb0331b4
flash offset: 0x0
bootloader: bootloader_qio_80m.elf / qio_qspi
```

Application hashes:

```text
TX bin:  7af9f7ded31fe33f326fed37d4606a9cc5e26ff07c358f59b499143f4a8b5148
RX bin:  bc9b30c8a426037c03c546660aca72f0d362ab881bab5d61bcccc601cd68f090
TX ELF:  f53d1168ef5df0c2f225f98ab9c4bdb49a4956efe20f3f5bb300afdc83fed8f1
RX ELF:  1ce4bf143b2fafcda5808301347f036f1b6c2b3b5511aa347a9640dbbdafff9c
```

The unchanged partition, role-state, LittleFS and bootloader metadata are in
`manifest.json`. Do not rebuild after release publication.

## Accepted hardware pair

```text
A TX: 90:70:69:28:56:E4
B RX: 90:70:69:27:20:38
UID:  10:73:a1:1f:a8:0f
```

Board C, `90:70:69:27:20:64`, is not part of the accepted pair. Its
`RADIO_BUSY` and `RADIO_DIO1` pins are bridged.

## Tested scope

- 250 Hz RF link at 25 mW;
- 3,000-frame TX drive with zero stalls and zero bad CRCs;
- 1,767 RC frames and 137 link-statistics frames received at RX;
- approximately 100% link quality in both directions;
- uplink approximately -29 dBm and +13 dB SNR;
- downlink approximately -33 dBm and +12 dB SNR;
- RX telemetry injection reached TX;
- solid blue TX and solid green RX connected LEDs;
- power cycle 1 of 3 reacquired the link and passed RC, telemetry and link
  statistics.
- TX board A linked to an external Betaflight drone receiver configured for
  2G4 RX at 100 mW and ExpressLRS 4.1.0;
- the receiver acquired the link in approximately 10-15 seconds, showed a
  solid link LED and produced live changing `MSP_RC`;
- minimum-throttle arm and automatic disarm passed on AUX1/CRSF channel 5;
- the AUX5/CRSF channel 9 beeper test passed with `play_sound 4: ARMING`.

All real-drone checks were performed with propellers removed. NOT TESTED:
power cycles 2-3, flight, range or attenuation.
