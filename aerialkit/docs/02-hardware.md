# AerialKit - hardware notes

## Bench inventory

| Board | MCU | State | Use |
| --- | --- | --- | --- |
| WeAct STM32F405RGT6 dev board | STM32F405RGT6, 1 MB flash, 128 KB SRAM + 64 KB CCM | On the bench; ROM DFU proven (all four reference families were written to it on 2026-09-14) | AerialKit's first target (M0-M4) |
| JHEMCU GHF435 AIO V2 | **AT32F435** (Artery), not STM32 | Flying the twin-wings wing on INAV 9.1.0 | AerialKit's third target: `src/boards/AERIALKIT_GHF435/`, `src/arch/at32f435/` and `targets/aerialkit-ghf435/` in the workspace. It builds and passes the structural checks; **nothing has been flashed**, so every pin below it is read rather than measured - [25-at32-survey.md](25-at32-survey.md) is the survey and the board file is where each fact is cited |
| ESP32 devkits | the **family**: WROOM, C3, S2/S3 - the owner has them with their pins soldered (2026-09-17), and the port has a board file per chip (`ports/esp32` + `src/arch/esp32`, [17-esp32-port.md](17-esp32-port.md)) | **Two** images per build now, and the name says which hardware: `aerialkit-esp32.bin` is the emulated-Ethernet build QEMU runs, `aerialkit-esp32-wifi.bin` is the one a devkit takes (radio, IMU and divider fitted) | M6 - the target builds, runs under QEMU, and every port file has been driven against a model; what is left is a part on the bench and a receiver (the table above) |

The wing itself (twin pusher, elevon servos, CRSF on UART2) is documented in
`fc-firmware-workspace/projects/twin-wings/`.

**The pin tables below are claims, and four of them are wrong.**
`make boards` reads every board header and compares its resources against each
other, and the first run found four pads two functions want — including the
status LED and the first servo output on the same pin of a *bare* ESP32S2
devkit. [30-boards.md](30-boards.md) is the list, the rule, and what the check
does not prove. Until they are fixed, the numbers in this page and in the board
files are what the firmware believes rather than what it can drive.

## What each board still needs before it can fly

The goal is a **quadrotor and a fixed wing flying on this firmware**, and the
four boards in the inventory are not in the same state towards it. Writing the
gap down in one place, because "which board is closest" and "what do I have to
solder" are the two questions a bench session starts with, and the answer is in
three different board files otherwise:

| Board | Airframe it can fly | What is missing | Where it goes, and what the firmware says until it is there |
| --- | --- | --- | --- |
| **JHEMCU GHF435 AIO V2** (the wing's board) | **the fixed wing** - two motors and two elevon servos, the mixer its INAV build already flies | *nothing but the flash*: the IMU (ICM-42688P), the barometer (DPS310), the pack divider and the onboard receiver are all fitted and all declared in `src/boards/AERIALKIT_GHF435/board.h` | [26-ghf435-bringup.md](26-ghf435-bringup.md), in order. Its tell-tale image is published beside the flight one (§2a) |
| **WeAct STM32F405RGT6** (the bench board) | **a quadrotor** - four motor outputs on TIM3 (PA6/PA7/PB0/PB1) and two servos on TIM2 | **an IMU** and **a receiver**. The barometer and the pack divider the owner soldered are the `-fitted` image's; the IMU bus is wired and idle (SPI2, CS PB12, SCK PB13, MISO PB14, MOSI PB15), and the receiver's UART is USART1 (PA10 RX / PA9 TX - CRSF at 420000, SBUS needs an inverter) | Solder an InvenSense/Bosch part to that bus and the drivers probe it by name; until then the boot says `imu: none - the flight core stays in failsafe` and the arm gate refuses with `the attitude estimate has not seen the accelerometer`, because a quadrotor with no attitude has no aircraft. [09-sensors.md](09-sensors.md) has the probes and [05-bringup.md](05-bringup.md) the session |
| **Adafruit Feather STM32F405 Express** | **a wing** - two motors on TIM3 (PA6/PA7, Feather pins A2/A3) and two elevon servos on TIM4 (PB8/PB9, pins 9/10). The motor timer's other two channels and the arch's PA0/PA1 servo pads are on the package and **not on this header**, which is why the board's shape says 2 and 2 rather than 4 and 2 - see [07-outputs.md](07-outputs.md) and traps 204 | **An IMU that the firmware can read**, and a receiver. The owner confirms the LSM6DSO breakout is plugged into the Qwiic connector (I2C1, PB6/PB7); the firmware probes address 0x6A, but the 2026-09-30 `imu` reply was `nothing answered on the bus`. That reply does not distinguish an address NACK from a later register-read failure. The pack divider on PA3 is labelled `VDIV` but its ratio is unmeasured, so `AK_BOARD_VBAT_FITTED` stays 0; SBUS needs an inverter on PB11 as it does on the WeAct board | **The Feather image has run on the board.** After the 2026-09-30 DFU flash, `preflight` read `airframe 7`, `outputs: 2 motors, 2 servos`, and `saved configuration: none stored`; the IMU remained undetected and arming was refused for the missing receiver. See [f405-preflight-2026-09-30.txt](evidence/f405-preflight-2026-09-30.txt) for the earlier baseline and dated superseding reading. The first flash is complete; the console's `dfu` is available for later flashes. |
| **ESP32 devkit** (WROOM, C3, S2/S3) | **a quadrotor**, on the chips with enough RMT channels - the ESP32 has eight, the S3 four, and the **C3 only two**, which is why a C3's board file says two motors and a quadrotor's mix does not fit it | **an IMU** and **a receiver**, soldered: the port has the buses (SPI for the IMU, I2C for the baro) and the radio for the configurator, but no parts | [17-esp32-port.md](17-esp32-port.md)'s "When there is a board" list. The image a devkit takes is `aerialkit-esp32-wifi.bin` (the radio build), not the emulator's |

**Two things follow from that table, and both are worth saying out loud.**

**The wing is one flash away from flying.** The GHF435 is the aircraft's own
controller with every part AerialKit needs already on it, which is why it was
surveyed before the F405 was finished - and it is also the board whose existing
INAV build is the only working flight controller on the wing, so the first
session has a way back and a way to compare.

**The quadrotor needs a part, not code.** Its airframe, mixer, rate loop,
arming gates, failsafe and navigation all run today
([18-software-in-the-loop.md](18-software-in-the-loop.md) flies them), and the
F405's outputs, control loop and console have host checks behind them. What the
bench board does not have is the sensor the loop is built around: **nothing
arms without an IMU, and an IMU is one breakout board on four wires**
(3V3, GND, and the SPI pair above). That - and a receiver - is the whole
difference between the quadrotor flying and not, on either the F405 or an
ESP32 with the same part soldered to it.

## Assumptions in the M0 board file

These come from the board being an STM32F405RGT6 part and from the ROM DFU path
working, and they are marked in `src/boards/AERIALKIT_F405/board.h` so they can
be corrected in one place:

| Assumption | Why we believe it | How to check |
| --- | --- | --- |
| 8 MHz HSE crystal — **read off the board's own output** on 2026-09-29: the banner's `clocks:` line reads `HSE 8 MHz x PLL` on the *measured* branch at `168 MHz sysclk`, and the image that printed it runs `PLLM = 8` with USB enumerating, so HSE/PLLM is 1 MHz | This row said 12 MHz for a day, on an **11.95 MHz** reading taken on 2026-09-28 that was not this board: it is a 12 MHz crystal read about 0.4% low, and 0.4% is HSI's own tolerance — HSI is the measurement's reference, so a part whose HSI sits low reads every crystal low by that fraction. The 252 MHz / USB 72 MHz failure this row used to attribute here belongs to the two boards swapped: a **12 MHz part running an image built for an 8 MHz one**. Traps 212 | The banner's `clocks:` line, which prints the measured crystal to whole megahertz — so it corroborates nothing by itself — or a waveform on the crystal. Note that no instrument has measured it yet: the 8 MHz is arithmetic from `PLLM` and a host-timed 168 MHz sysclk, plus a USB that enumerates |
| Status LED on PC13, active low | Common on this class of dev board | Look at it. If it never lights, it is this line |
| Console on USART2 PA2 (TX) / PA3 (RX), 115200 8N1 | The pins are on the header and USART2 is on APB1 | A USB-TTL adapter, or move it to USART1 PA9/PA10 |
| USB device on PA11 (DM) / PA12 (DP), alternate function 10 | The native OTG FS pins, on the board's own USB connector; the ROM bootloader uses the same pair to enumerate, so the wiring is proven even though AerialKit's own device is not | `/dev/ttyACM0` and the kernel log, [05-bringup.md](05-bringup.md) step 3a |
| Outputs on PA6/PA7/PB0/PB1 (motors, TIM3) and PA0/PA1 (servos, TIM2) | These are the timer channels that are free. The motors are the timer's - one DMA burst writes all four - but the servos are the *board's* and are passed to `ak_output_init()`: the Feather's breakout brings out neither PA0 nor PA1, so its servos are TIM4 CH3/CH4 on PB8/PB9 | A scope on each pin; see [07-outputs.md](07-outputs.md) |
| IMU bus on SPI2 PB13/PB14/PB15, chip select PB12 | The spare SPI port, mode 3 as the InvenSense parts want | `spi` with a jumper from PB15 to PB14; see [09-sensors.md](09-sensors.md) |
| Barometer bus on I2C1 PB6 (SCL) / PB7 (SDA), 400 kHz, address 0x77 | Where a barometer lives on the boards that have one, including the wing's own AIO; PB6/PB7 are the free pair | **The barometer is fitted** - the owner soldered it, which is why the `-fitted` image is the one this board takes ([05-bringup.md](05-bringup.md)). [20-i2c.md](20-i2c.md) has the three things to settle, starting with the pull-ups |
| Rangefinder on the *same* I2C1 pair, address 0x52 | An I2C bus is two wires and a set of addresses: a second part is an address, not a port | A TOF10120 underneath the aircraft reads the height of its landing gear; `range` prints what it sees, and [09-sensors.md](09-sensors.md) has the filter and what the landing rule does with it. **Not fitted yet** |
| Receiver on USART1 PA10 (RX) / PA9 (TX) | A UART of its own, so the console keeps USART2: CRSF at 420000 8N1, SBUS at 100000 8E2 | `rc` and its counters; see [08-receiver.md](08-receiver.md). **SBUS needs an inverter on the pin** - the F4 cannot invert its own |
| GPS on USART3 PB11 (RX) / PB10 (TX), 9600 baud | The u-blox factory rate; a module configured otherwise needs that changed | `gps` and its counters; see [13-gps.md](13-gps.md) |
| Battery on PC0 (ADC1 input 10) behind a 10k/1k divider | The first ADC pin that is not already a timer, UART, SPI or the LED | **Fitted** (the pair the owner soldered with the barometer). Compare `battery` with a multimeter and hand the number to `calibrate vbat`; see [19-battery.md](19-battery.md) |

**A trap worth knowing about, because it cost the console once:** configuring a
UART and choosing the console used to be the same operation, so initialising the
receiver and then the GPS left console output pointing at the GPS's transmit
pin - the firmware booted silently and the console was dead. No host test can
see that, because the port code is not in the host build. `ak_uart_init()` now
only configures a port; `ak_console_attach()` is separate and is called once, by
the board, for the port the console is actually wired to.

**And the assumptions a board can be *told* about are now built twice.** Three
of the rows above are not guesses about wiring but flags in the board header -
`AK_BOARD_BARO_FITTED`, `AK_BOARD_VBAT_FITTED` (and on the ESP32,
`AK_BOARD_IMU_FITTED`) - and on this bench board they are 0, so the code under
`#if` for a fitted part is code no build here compiles. That is the same hole
the ESP32's Wi-Fi half fell into and stayed in: the configuration a real board
needs is the one nothing builds. So each target's build compiles **both**
configurations now - the bare board and the fitted one - and the flags are
guarded (`#ifndef`) so a build can set them:

```sh
make BOARD=AERIALKIT_F405 EXTRA_CFLAGS="-DAK_BOARD_BARO_FITTED=1 -DAK_BOARD_VBAT_FITTED=1" check
```

That is what `scripts/fw build aerialkit-f405` runs after its own build, and
what the ESP32's script does for the radio and both parts at once. Nothing runs
the fitted *image* here - there is no soldered board on this desk - but the
configuration is not compile-only either: the F405 pass runs `test` as well as
`check`, so the board's own tests are compiled *and run* in the fitted
configuration. **That half was false until 2026-09-29**: `EXTRA_CFLAGS` reaches
the image's `CFLAGS` and never the host build's, so the pass reported the bare
suite's count against a fitted image and the fitted arms of
`tests/test_board_f405.c` had never been taken. See `21-port-on-the-host.md` and
traps 205. The ESP32's two configurations are still compiled rather than run,
for the reason the rest of that port is.

**The WeAct board has no on-board USB-serial adapter, so the console has two
doors and neither is a chip on the board.** One is a 3.3 V USB-TTL adapter on
PA2/PA3. The other is AerialKit's own OTG FS device on PA11/PA12 - the cable the
board is plugged into for flashing carries it, and the host sees a CDC serial
port (`0483:5740`, so Linux binds `cdc_acm` with no rule of its own). The
adapter is the fallback, the USB device is the one that needs no extra part, and
[05-bringup.md](05-bringup.md) step 3a is where the kernel log is read. Neither
has been observed on this board yet: the flashed image predates the USB device.

## Flash layout

AerialKit currently has no bootloader. It links at `0x08000000`, the whole
image including the vector table, and is written through the ROM bootloader —
exactly the layout and the method the bench work documented for INAV-style
images:

```sh
dfu-util -a 0 -s 0x08000000:leave -D dist/aerialkit-f405/aerialkit-f405-fitted.bin
```

The `-fitted` name is the board with the barometer and the pack divider soldered
on, which is the one on this bench; `aerialkit-f405.bin` is the bare board, and
[05-bringup.md](05-bringup.md) §1 says which is which.

The 1 MB part is divided three ways, and each boundary is enforced by something
that fails loudly rather than by a comment:

| Range | What it is | What enforces it |
| --- | --- | --- |
| `0x08000000` – `0x0801FFFF` (128 KB) | the image; about 74 KB of it is used | `ASSERT` in `linker/stm32f405rg.ld` — an image that grew past `0x08020000` would be erased out from under itself by the first log wrap, so the link fails instead |
| `0x08020000` – `0x080BFFFF` (640 KB) | the blackbox in flash: five 128 KB sectors, records of 52 bytes, a ring that erases the oldest sector when it needs room — on the ground only ([11-blackbox.md](11-blackbox.md)) | `_Static_assert` in `src/boards/AERIALKIT_F405/board.c` — the region may not reach the configuration's first bank |
| `0x080C0000` – `0x080FFFFF` (256 KB) | the saved configuration: **two banks of thirty-two records**, four kilobytes a slot, each carrying a serial — a save writes the next erased slot of the bank holding the newest record, and a full bank hands over to the other, whose sector is erased first. The erase therefore never lands on the bank holding the newest record, so a power cut inside it cannot leave the aircraft with no configuration at all | `_Static_assert` in the board file that a record fits one slot; `ak_board_config_write`, which chooses the slot, the bank and the erase; `make config-recycle` drives the power cut through the real board |

Both of those live in code a board change has to touch: moving the log means
editing the region table and the two assertions together, which is the point.

## The wing's board, and its own layout

The third target is not an STM32F405 and its flash is not the same shape, so the
numbers above are the F405's and these are the GHF435's. What is the same is the
*approach*: the image at the base of flash, the blackbox above it, the saved
configuration at the top, and an assertion at each boundary.

| | AT32F435RGT7 (the wing's AIO) |
| --- | --- |
| Flash | 1 MB in **two banks of 512 KB**, erased in **2 KB pages** - there is no "sector" here, and the controller refuses a program that crosses a bank |
| RAM | the two regions the reference's own linker script maps: 128 KB at `0x20010000` and 64 KB at `0x10000000`. The 64 KB below `0x20010000` is *not* mapped here, because INAV - the only implementation of this chip there is to read - does not map it either; if the part hands it to the application, the linker script and the stack top move down together ([25-at32-survey.md](25-at32-survey.md)) |
| Image | `0x08000000` – `0x0801FFFF` (128 KB), asserted by `linker/at32f435rg.ld` |
| Blackbox | `0x08020000` – `0x080DFFFF` (768 KB, six regions of 128 KB = 64 pages each), asserted by `src/boards/AERIALKIT_GHF435/board.c` |
| Configuration | `0x080E0000` – the last 128 KB of bank 1, of which **two** 2 KB pages hold the record (2060 bytes: a magic word, a length, a sum and 2048 bytes of text - one page is not enough, which is a thing this board found out when its own file was finally run against a host) |

The log region is 128 KB for the same reason the F405's sectors are: 128 KB
minus the 32-byte header divides exactly into 60-byte record slots, so a region
has no tail. Every log, on every part, is 2184 records per region.

**Flashing this one is unproven.** Artery's ROM DFU bootloader lives at
`0x1FFF0000` and takes the same kind of flat image at `0x08000000`, and the
image AerialKit produces is linked for exactly that - but no image has been
written to this board yet, so the command below is the F405's method read
across, not a path anyone has walked:

```sh
dfu-util -a 0 -s 0x08000000:leave -D dist/aerialkit-ghf435/aerialkit-ghf435.bin
```

What to look for first is a banner at 115200 8N1 on USART1 (PA9/PA10) - this
board has no USB console in AerialKit yet, so a UART adapter on those pads is
the only door. If it prints, the clock, the flash wait states and the console
are all right; if it does not, those are the three things to suspect, in that
order.
