> Historical notes from the full development tree. Current public targets and commands are in the [firmware guide](../../docs/flight-controller-firmware.md). Host C tests and simulator harnesses referenced below are retained outside this public source tree.

# AerialKit - the wing's board bring-up (AT32F435)

The first time this firmware meets the flight controller the wing actually
flies, which is not the board [05-bringup.md](05-bringup.md) is about. That page
is the WeAct STM32F405 on the bench: no gyro, no baro, no receiver, five outputs
and a USB-TTL adapter. This one is a complete aircraft controller - the JHEMCU
GHF435 AIO V2, an Artery AT32F435 - with an ICM-42688P, a DPS310, 16 MB of
blackbox flash, an onboard ELRS receiver, GPS pads, two motors and two elevon
servos, and a USB port that is already the cable it is flashed through.

**Nothing on this page has been done yet.** The port is written and every part
of it is run against fakes ([25-at32-survey.md](25-at32-survey.md) has what was
measured and what was read): this page is what to do now, in the order that
stops at the first surprise.

## What trying this costs

The board is not blank. It runs **INAV 9.1.0** (`TWINWINGS_GHF435V2`, the
twin-motor mixer repaired by hand, differential thrust on, the onboard receiver
verified on USART2) and it is the only flight controller that currently works on
this aircraft. Flashing AerialKit replaces that, and there are two ways back,
both written down with hashes:

| Way back | Where |
| --- | --- |
| Re-flash the INAV build that is on it now | `dist/ghf435-v2/inav_9.1.0_TWINWINGS_GHF435V2.bin` (built in this workspace; the same target the board was flashed from on 2026-09-13) |
| Restore the firmware it shipped with | `projects/twin-wings/ghf435-inav/backups/ghf435v2-betaflight-2025.12.2-fullflash.bin` (the whole 1 MB read out of the board, with its sha256 in that directory's README) |

**And the way into DFU by hand is not a button you can press.** The board's
drawing says the BOOT button needs a solder joint shorted first, and that has
never been tried on this board: it has only ever been put in DFU from running
firmware (Betaflight's `bl`, then INAV's `bl rom`). AerialKit has the same
command - `dfu` on the console - and it is in the image for exactly this reason.
What it cannot cover is a firmware that does not run at all, which is why the
solder joint is worth knowing about *before* the flash and not after.

## Before starting

| Need | Where |
| --- | --- |
| Props off, and no ESC power | the first passes are USB only, which is how the board was flashed and how INAV was configured. Nothing on the board needs the pack to talk |
| The image, built on the machine that will flash it | `cd aerial-kit && fw build aerialkit-ghf435`, then `dist/aerialkit-ghf435/aerialkit-ghf435.bin` and its sha256. Record which machine built it: two builds of one revision are two images (record the compiler version and build host alongside the image) |
| `dfu-util` | the system package, or the copy the wing project unpacked at `projects/twin-wings/third_party/tools/dfu-util/usr/bin/dfu-util` |
| USB access without sudo | `projects/twin-wings/udev/99-at32-dfu.rules`, installed once; without it, run under sudo |

## 1. Put it in DFU and flash it

If INAV is still running, its CLI can do the first half: `bl rom`. Otherwise the
BOOT button and the solder joint. Then:

```bash
dfu-util -l
#   Found DFU: [2e3c:df11] alt=0 name="@Internal Flash   /0x08000000/512*002Kg"
```

`2e3c:df11` is Artery's ROM, and the alternate setting names the geometry this
port's linker script is written against: the flash starts at `0x08000000` and it
erases in 2 KB pages. Then:

```bash
dfu-util -d 2e3c:df11 -a 0 -s 0x08000000:leave \
    -D dist/aerialkit-ghf435/aerialkit-ghf435.bin
```

Two things about that command are worth knowing before it runs. **AerialKit's
image is flat**: vector table at `0x08000000`, code behind it, and no config
regions - INAV's image leaves the first 32 KB for its own defaults, and this one
does not, because its saved configuration is in the last 128 KB of the part
([02-hardware.md](02-hardware.md)). And **`:leave` may not be honoured**: the
AT32 ROM ignored a standalone `DFU_DETACH` in the wing project's tests, so
expect to unplug and replug the cable to boot what was just written.

## 2. The banner

The board comes back as `2e3c:5740` - Artery's vendor id, AerialKit's product id
([06-console.md](06-console.md) and `src/arch/at32f435/usb.c` say why) - so on
the Pi:

```bash
journalctl -k -f          # then replug, or press reset
# expect: idVendor=2e3c, idProduct=5740
#         cdc_acm 1-1.x:1.0: ttyACM0: USB ACM device
cat /dev/ttyACM0
```

The fallback is USART1: **PA9 is transmit, PA10 is receive, 115200 8N1**, which
needs a USB-TTL adapter. Either door carries the same bytes.

What to look for in what comes out:

```text
  product:  aerialkit-ghf435
  board:    AERIALKIT_GHF435 / JHEMCU GHF435 AIO V2 (AT32F435)
  clocks:   288 MHz sysclk (HEXT 8 MHz x PLL), apb1 144 MHz, apb2 144 MHz
```

**`288 MHz` and `HEXT` are the first two facts this board has ever produced**,
or not. The clock is a PLL of this part's own kind (ms/ns/fr rather than the
F4's M/N/P) and a firmware that fell back to the internal clock prints `HICK,
HEXT FAILED` instead - which on this board means the crystal, and on this board
nothing works well without it: the USB clock is derived from that PLL.

If the banner is silent and the host never saw a USB device, the order of
suspicion is: the flash did not take (re-enter DFU and check `dfu-util -l`), the
clock, the wrong pad. If the *host* sees `2e3c:df11` instead of `2e3c:5740`, the
ROM is still running and the image did not start - which is a bad image rather
than a bad board.

**And one item is already off that list, because of what the F405 did on
2026-09-17.** Its USB device put *nothing* on the bus - no `new USB device`
line at all - because its driver never wrote `GCCFG`, and this core holds D+
down while it is powered down or waiting for VBUS sensing. This port writes
**both** bits now (`PWRDOWN` and `NOVBUSSENS`, `src/arch/at32f435/usb.c`), and
`tests/test_arch_at32.c` asserts both against the mapped register block - so
"the device never appeared at all" is not one of the things to expect here. If
that happens anyway, the USB core is not the first suspect; the clock and the
image are.

## 2a. If the LED never blinks: the tell-tale image

The F405's page has this section (`05-bringup.md` §6a) because its console went
silent for two days and the LED was the only instrument left. **This board
needs it more, not less**, for two reasons that are both about this board:
its USB console has *also* never enumerated - the two ARM ports share that
driver, and the bug that made the F405's console silent (a setup byte read out
of the wrong place in the request) was in both copies, fixed in both before
either was flashed - and **this board is the one that is hard to get back into
DFU**, since the BOOT joint has never been shorted and every DFU entry so far
has been from a firmware that was already running. A board that runs nothing
and says nothing is the one case where the LED has to answer.

The instrument is the same sources compiled with one more flag, and the
target's own build publishes it beside the flight image:

```bash
cd fc-firmware-workspace
scripts/fw build aerialkit-ghf435        # writes both images
```

`dist/aerialkit-ghf435/aerialkit-ghf435-telltale.bin` is this board's flight
configuration plus `-DAK_BOOT_STAGE=1` - **105,752 bytes against the flight
image's 105,224**, both from the same build (2026-09-18). By hand it is
`make BOARD=AERIALKIT_GHF435 ARCH=at32f435 PART=at32f435rg
PRODUCT=aerialkit-ghf435 EXTRA_CFLAGS="-DAK_BOOT_STAGE=1" all`.

**The LED is PC13, lit by pulling the pin low** (`AK_BOARD_LED_PIN`,
`AK_BOARD_LED_ACTIVE_LOW` in `src/boards/AERIALKIT_GHF435/board.h`) - the same
pin and the same polarity as the F405's board, which is a coincidence and worth
knowing before reading a pattern off it. The flight image uses it as a 2 Hz
heartbeat once the loop runs; the tell-tale blinks the boot's stage instead.

The grammar is the core's, so it is the same as the F405's: thirteen stages,
the stage number blinked three times over at each one, and a crash as ten quick
blinks, a pause, and then the stage the boot had reached (that number survives
in `.noinit`, and the next boot prints the pc and the fault status over the
console - §2).

```text
1 clock          2 barometer bus   3 console UART   4 USB core
5 tick           6 banner          7 selftest       8 parameter apply
9 outputs       10 battery        11 IMU probe     12 altitude
13 network and preflight
```

| What the LED does | What it means |
| --- | --- |
| a pattern, and then nothing | the boot stopped right after the work that stage does |
| ten quick blinks, a pause, then a stage number | it **crashed**, and that number is how far it got |
| patterns through 13, then 2 Hz | the boot completed and the loop is running: the trouble is the console (or the cable), not the boot |
| dark, and the board is powered | the image is not running at all - back to DFU, and §2's order of suspicion |

**And the image is checked for being the instrument.** A tell-tale that was
built without the flag blinks nothing, which on a board with no console reads
exactly like a boot that stopped before stage one - so `make check` now asserts
it in both directions (`scripts/check-image.sh --telltale`): the tell-tale's
`ak_board_boot_mark` has to be the board's real one with its blink plan linked,
and the flight image's has to be the two-instruction stub with no plan. That
check runs on this target's build and on the F405's, and it is the difference
between an image whose *name* says tell-tale and an image that blinks.

## 3. What the board says about itself, in order

One command at a time, and stop at the first thing that is not what is written
here. Everything on this list is checked against a fake on a host
([21-port-on-the-host.md](21-port-on-the-host.md)); none of it has been checked
against this board.

| Type | Expect | If it is silent or wrong |
| --- | --- | --- |
| `preflight` | the board's own summary: clock ok, console attached where the board says, outputs present, what is fitted and what is not | it names the first thing that disagrees with the board file. Read it before anything else |
| `status` | state `disarmed`, an attitude in mrad, the parameter count, the build revision | a state that is not disarmed at boot is a firmware fault, not a board one |
| `imu` | `icm42688p`, samples arriving at about a kilohertz, no errors | `absent` means the SPI1 bus or the chip select (PA4): the part is on the board, so this is a bus problem - `spi` with a jumper from PA7 to PA6 tests the wires |
| `baro` | `dps310` and a pressure that moves when you breathe on it | `no bus` means I2C2 (PH2/PH3). This part's I2C is the newer peripheral and its transfers are checked on a host, not on a wire |
| `rc` | `crsf`, frames climbing, channels moving with a **bound** transmitter | zero frames means the port: USART2 was verified live on this board, USART1 was not ([25-at32-survey.md](25-at32-survey.md)) |
| `battery` | counts and a voltage at the pin, and a pack voltage that is the right *shape* | the divider is on the board but its ratio is not known: `vbat_ratio` is the F405 bench board's until somebody puts a multimeter on a pack. Do not believe a cell count before that - and then fix it with the meter in hand rather than a calculator: `calibrate vbat <what the meter says>` reads the pin and writes the ratio |
| `output` | TMR4 for the motors, TMR2 for the servos, 300 kHz DShot, and the counts of frames sent | motors and servos move only under `output test`, with the aircraft disarmed, and nothing is wired yet |
| `gps` | a fix, if a module is on the pads | the port has the peripheral's transmit/receive swap bit set, because that is what the live board does |
| `save` then `load` | "saved" and then the same parameters back | a save that reports it did nothing means the flash pages under the record (two of them, this part's 2 KB pages) |
| `log flash` | the records in the internal blackbox | an empty log on a board that has just booted is normal; a log that stops growing while armed is the internal flash having no erased region left |

## 4. What is read, not measured

Four facts about this board came from the reference target rather than from the
board, and each one is a measurement away - do them before anything flies
([25-at32-survey.md](25-at32-survey.md)):

1. **The gyro's orientation.** The reference target says CW90 for this part and
   the Betaflight target for the same board says CW180 plus -45 degrees of board
   yaw. Those cannot both be right, and AerialKit has the same three parameters
   (`align_board_*`). Rotate the board by hand and watch `status`: the sign that
   moves the other way is the sign that is wrong.
2. **The servo pads.** PB8 and PB9 are UART5's pads on the manufacturer's
   drawing, released by the wing's target so the elevons can have TMR2. Nothing
   has checked continuity to the MCU.
  3. **The GPS pads.** PB10/PB11, with the swap bit set because that is what the
     live Betaflight resource map says. One `gps` command with a module attached
     settles it.
  4. **Which motor is left and which is right.** This decides the sign of the yaw
     axis in the mixer, and the wing flies either way - badly - until it is right.
  5. **Which way each elevon's linkage goes**, which is the same question for the
     servos and is a parameter rather than a wire: `output test` moves one servo
     at a time and a hand on the surface says whether it goes the way the mixer
     thinks. If it does not, `servo1_reverse` (or 2) is the fix, and `output`
     prints what each servo is set to. Two hundred microseconds of `trim` is
     there for a centre that is not mechanically at 1500 - the linkage is the
     better fix, and the parameter is for when there is no more room on it.

     **And expect to need both.** The aircraft's own INAV mixer - the one it
     flies on today, in `projects/twin-wings/ghf435-inav/` - has **both**
     elevons reversed (`smix 0 1 0 50 0 -1`, `smix 3 2 1 50 0 -1`: INAV's `-1`
     is its reverse flag), and this firmware's table has the opposite sign on
     both (the parity check in `tests/test_mixer_parity.c` is what keeps that
     relationship exactly as it is). So with the horns where INAV had them, a
     roll command moves both elevons the same way and the wing does not turn at
     all - [07-outputs.md](07-outputs.md) calls that the one that stops a wing
     flying. Set `servo1_reverse 1` and `servo2_reverse 1`, then `output test`
     again and watch both surfaces do what the pages say before anything is
     armed.

## 5. And then it is a board that can be calibrated

The order the calibrations go in, once the sensors answer: `calibrate` with the
aircraft still and disarmed (the gyro's bias), `calibrate vbat <pack volts>` with
a multimeter across the pack, `calibrate rc` with the sticks centred, and `calibrate accel <face>`
for each of the six faces - which is what the attitude estimate's level is
built from, and what the arming check's "within 25 degrees of upright" compares
against ([10-calibration.md](10-calibration.md)).

Only then does the aircraft have the things a first flight needs, and the flight
itself is [23-first-flight.md](23-first-flight.md)'s page.

## Reflashing, and getting back out

`dfu` on the console hands the part back to the ROM, which is the whole of the
way back in while AerialKit is running - and the only way that does not need the
solder joint. After it, the host sees `2e3c:df11` again and either the INAV
image or the vendor image above can be written the same way AerialKit was.
