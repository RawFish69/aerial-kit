# Rev A3 physical WS2812 order diagnostic — 2026-11-19

## Why another candidate is required

The physical color evidence does not identify one channel order:

- `5576f902…` sent G-R-B. RX displayed blue for a logical green role color,
  TX displayed green for a logical blue role color, and red was correct.
- `19c7b3ce…` changed the table to B-R-G. The connected RX still reports role
  `rx`, slot `rx_0`, GPIO4 high and RX ELF prefix `d28b7656`, but its role LED
  is still blue.

The second observation is inconsistent with the first if the LED has a fixed
three-channel permutation: the two candidates drive green from different wire
positions, yet both appeared blue. Another guess would not be evidence.

The operational state machine, GPIO4 mapping, boot pulse count, bind timing,
fault red and mismatch amber are unchanged in this diagnostic candidate.

## Diagnostic behavior

The new CRSF frame

```text
C8 08 7F <own address> EA 41 4B 4C 50 <crc>
                            A  K  L  P
```

starts all six wire permutations in this order:

| Label | First byte | Second byte | Third byte |
| --- | --- | --- | --- |
| RGB | R | G | B |
| RBG | R | B | G |
| GRB | G | R | B |
| GBR | G | B | R |
| BRG | B | R | G |
| BGR | B | G | R |

For each permutation the firmware sends the known logical colors for one
second each:

```text
red   R=255 G=0   B=0
green R=0   G=255 B=0
blue  R=0   G=0   B=255
```

Each group is followed by 500 ms off. The six groups therefore take 21
seconds. The USB stream labels every transition:

```text
AK-LEDPERM begin sequence=RGB,RBG,GRB,GBR,BRG,BGR step=1000ms gap=500ms
AK-LEDPERM order=RGB logical=red
AK-LEDPERM order=RGB logical=green
AK-LEDPERM order=RGB logical=blue
AK-LEDPERM order=RGB logical=gap
...
AK-LEDPERM done led=ok
```

The operational mapping remains B-R-G while this diagnostic runs only through
an explicit temporary order supplied by the test command.

## Timing

The production RMT path is unchanged:

```text
T0H    300 ns
T0L    950 ns
T1H    750 ns
T1L    600 ns
latch  300 us
```

The firmware reports the RMT health and non-zero latch in `AK-BOOT`.

## Hardware procedure

Use the board's measured role, not the printed switch label. For the currently
connected RX:

```bash
python3 firmware/elrs-usb-tx-rev-a3/tools/usb_crsf_bench.py \
  /dev/serial/by-id/<rx-device> \
  --baud 420000 --rate 0 --ping-interval 0 \
  --duration 24 --led-permutation 0xEC --write
```

For a GPIO4-low TX:

```bash
python3 firmware/elrs-usb-tx-rev-a3/tools/usb_crsf_bench.py \
  /dev/serial/by-id/<tx-device> \
  --baud 400000 --rate 0 --ping-interval 0 \
  --duration 24 --led-permutation 0xEE --write
```

Record the physical color observed during each labelled logical phase:

| Encoded order | Logical red | Logical green | Logical blue |
| --- | --- | --- | --- |
| RGB | | | |
| RBG | | | |
| GRB | | | |
| GBR | | | |
| BRG | | | |
| BGR | | | |

The correct physical order is the row that produces:

```text
logical red   -> physical red
logical green -> physical green
logical blue  -> physical blue
```

If no row does this, stop and record the complete matrix. That result would
show that the problem is not a simple R/G/B wire permutation.

After the operator reports the successful row, change only
`AERIALKIT_WS2812_OPERATIONAL_ORDER`, retain the existing state logic, and
produce the final acceptance artifact.

## Frozen diagnostic artifact

```text
path: firmware/elrs-usb-tx-rev-a3/dist/aerialkit-rev-a3-elrs-dual-role.bin
size: 8331264 bytes
sha256: 00a675f7cef01b010688f24e1e8e6fe44deef5d08660a1264a5edeadfc0225b6
timestamp: 2026-11-19 17:19:25 -0800
flash offset: 0x0
bootloader: bootloader_qio_80m.elf / qio_qspi
application header mode: dio
build inputs: 5f969a023d06ba0842eacc4fb49de83a54c8a753958aab64e012c598a20f2281
```

Application hashes:

```text
TX bin: d02079ba2ecacf8b601ad03ee598638488fe21c5bedefd58401a6f823626827b
RX bin: 6ea205681a6ab349a74967a39a4ffbdc45dd493162f9378790e29847a0e4c711
TX ELF: b4d5bda1d673c583416aa8c24ea7a0d11efdc008d54985edb79f6a9bbef01768
RX ELF: f66754f54ebb5522aa58de0c854f06b8f76c123e517de67620bf6a5c54d7694b
```

The identity is recorded in `releases/00a675f7/`.

## Validation

- TX and RX PlatformIO builds passed.
- Firmware Python tests: 24 passed.
- Native USB/LED state tests: 9 passed.
- The linked applications contain the diagnostic and RMT paths and no
  upstream HSV, rainbow, brightness-fade or I2S LED symbols.
- No hardware was flashed or tested while producing this diagnostic.

This candidate is not hardware-accepted. Its purpose is to obtain the missing
physical color matrix before selecting and freezing the final operational
order.
