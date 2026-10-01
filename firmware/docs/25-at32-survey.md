> Historical notes from the full development tree. Current public targets and commands are in the [firmware guide](../../docs/flight-controller-firmware.md). Host C tests and simulator harnesses referenced below are retained outside this public source tree.

# AerialKit - the wing's own board, and what a port to it would take

The plan says the GHF435 AIO V2 is the wing's board, that it is an **AT32F435**
rather than an STM32F405, and that AerialKit support for it is "a later target"
([01-plan.md](01-plan.md), the bench inventory). This page is the survey that
comes before writing any of it, in the shape [22-esp32-survey.md](22-esp32-survey.md)
used for the ESP32: what is already known about the chip and this board, at which
revision it was read, what the port boundary looks like, and what has to be true
before a single line of it can be trusted.

Why it is worth doing at all, plainly: the goal is met when a quadrotor and a
fixed wing both fly on AerialKit on **hardware we can actually build for**. The
WeAct F405 on the bench has no inertial sensor, so it can verify a port and
cannot fly anything; the wing's board is in the drawer, is a complete aircraft
controller (gyro, baro, blackbox flash, OSD, receiver, five outputs, two of them
servos), and AerialKit cannot run on it. Nothing else in this project is between
the fixed wing and the air.

**Nothing here is copied.** The AT32 support in the pinned INAV checkout is
Artery's own firmware library, and its header says so: "download from Artery
official website is the copyrighted work of Artery". So it is read for the
arithmetic and the register names, and reimplemented - the same rule this
repository has followed for every driver it has.

## What was looked at

| Source | Where | Revision | What it is for |
| --- | --- | --- | --- |
| The wing's own INAV target | `upstream/inav-9.1.0/src/main/target/TWINWINGS_GHF435V2/` | `e519b69` (pinned) | the pin map, the sensor set, and what was live-verified on this physical board |
| That target's notes in this workspace | `projects/twin-wings/ghf435-inav/` | - | the board's history: what was flashed, what was measured, what is still open |
| Artery's device header and clock driver | `upstream/inav-9.1.0/lib/main/AT32F43x/Drivers/CMSIS/Device/ST/AT32F43x/at32f435_437.h`, `at32f435_437_clock.c` | `e519b69` | the register map and the clock recipe a port has to reproduce |
| INAV's AT32 drivers | `upstream/inav-9.1.0/src/main/drivers/*at32*` (fourteen files: `adc`, `bus_i2c`, `bus_spi`, `dma`, `serial_uart`, `serial_usb_vcp`, `system`, `timer`, `timer_impl_stdperiph`, `usb_msc`, `rcc_at32f43x_periph.h`, `timer_def_at32f43x.h` and the two headers) | `e519b69` | how each peripheral is actually driven on this part, and where it differs from the F4 |
| Betaflight 2026.6.1 | `upstream/betaflight-2026.6.1/src/main/drivers/serial_uart_impl.h` | `6dbc4218` | checked, and worth recording: Betaflight's AT32 support is one mention in a comment, so **INAV is the only reference implementation for this chip** |

## The chip and the board

Facts, with where each came from:

| | | Source |
| --- | --- | --- |
| Part | AT32F435, Artery, Cortex-M4F. INAV builds this target for the **AT32F435RGT7** (its flash page-size switch names that part), and the board is sold as a JHEMCU GHF435 AIO V2; the exact suffix printed on the chip is worth reading off it before a port is flashed | the wing target plus `config_streamer_at32f43x.c` |
| Clock | 8 MHz HEXT, PLL 72/1/FR2 -> **288 MHz** system, AHB /1, APB1 and APB2 /2 (144 MHz) | Artery's `at32f435_437_clock.c`, the header comment and `system_clock_config()` |
| LDO | 1.3 V before the PLL is enabled, flash divider 3 | same file |
| Flash | 1 MB internal, **2 KB sectors** | the firmware's memory map in INAV's `at32_flash_f43xG.ld` (10 K + 6 K + 16 K + 992 K) and `config_streamer_at32f43x.c`'s `FLASH_PAGE_SIZE 0x800` for the RG part |
| RAM | the linker hands this build **192 KB in two regions** (64 K at `0x10000000`, 128 K at `0x20010000`), and the comment at the top of the same script claims 384 KB for the series. Which this exact part has is a one-minute question with the board in hand, and it matters: AerialKit's F405 image spends 78 KB of its 128 KB | same linker script |
| Bootloader | Artery's ROM DFU at `0x1FFF0000` | `system_at32f43x.c`, with the comment "AT32 DIAGRAM2-1 AT32F435/437 DFU BOOTLOADER ADDR" |
| Gyro | ICM42688P on SPI1 (PA5/6/7, CS PA4) | the wing target; detected live on the board |
| Barometer | DPS310 (and BMP280 driver family) on I2C2 (SCL PH2, SDA PH3), the same bus as an external compass | the wing target |
| Blackbox | an M25P16-class SPI flash on SPI3 (PB3/4/5, CS PA15), 16 MB | the wing target |
| OSD | MAX7456 on SPI2 (PB13/14/15, CS PB12) | the wing target |
| Receiver | **the onboard ELRS/CRSF on USART2 (PA8/PB0), live-verified on this board** - UART1 produced no receiver data, whatever the manufacturer's drawing says | the wing target's own comment, and the session that found it |
| GPS | USART3 (PB10/PB11) with the AT32's **TX/RX swap bit** set, because that is what the live board does | the wing target |
| Outputs | motors on TMR4 CH1/CH2 (PB6/PB7), the two elevon servos on TMR2 CH1/CH2 (PB8/PB9) | the wing target's output plan |
| Pack | PA0 (volts) and PA1 (current - a shunt this board has and AerialKit's F405 board does not) | the wing target |

Open on the board itself, and therefore open for the port - none of these is a
firmware question and none can be settled by writing code:

- continuity of PB8/PB9 (the servo pads) and PB10/PB11 (the GPS pads);
- the **gyro orientation**: the wing target inherits CW90 from the stock AIO
  target, while the Betaflight target for the same board says CW180 plus a
  board yaw of -45 degrees. AerialKit has the same three parameters
  (`align_board_*`), and which numbers are right has to be measured;
- which ESC is left and which is right, which is what decides the sign of the
  yaw axis in the mixer.

## What a port would have to reproduce, and where this part differs

The differences that matter are not the pin map - that is a table - but the
peripherals where the AT32 is not an F4 with different addresses:

1. **The clock.** There is no HSE/PLLCFGR arrangement to copy from the F405
   port: this part has a clock unit with its own names (a source-enable and
   stable-wait per source, PLL `ns`/`ms`/`fr` multipliers instead of M/N/P), an
   LDO voltage that has to be raised *before* the PLL comes up, and a flash
   wait-state divider that has to be set with it. Get it wrong and the chip runs
   at the wrong speed or not at all; there is no "it mostly works" here.
2. **The GPIO.** Configuration is not MODER/AFR: this part has a per-pin source
   and multiplexed-function scheme, so every pin in the board file is a
   different kind of number from the F405's.
3. **The flash controller.** 2 KB sectors rather than 16 KB, so the
   configuration record and any log layout have to be recomputed rather than
   rescaled, and the arithmetic is checkable on a host the same way the F405's
   was (`tests/host_flash_model.c`).
4. **The timers and DMA for DShot.** INAV drives these through Artery's
   standard-peripheral layer with a per-channel TMR DMA request and a circular
   mode burst, which is the same *shape* AerialKit's F405 DShot path already
   has ([07-outputs.md](07-outputs.md)) - so the encoder is reusable and only
   the register layer is new - but "the same shape" is a claim about a datasheet
   until it is on a scope.
5. **The USART TX/RX swap bit**, which this board's GPS port needs and which the
   STM32F4's USART simply does not have - the same family of difference as the
   one that makes SBUS need a transistor in front of an F405
   ([08-receiver.md](08-receiver.md)).
6. **Two things this board has that AerialKit does not yet drive at all**: a
   current shunt (PA1) and an external SPI flash for the blackbox. Neither is
   required for a first flight, and both are gaps rather than ports.

## What the port would look like, and in what order

AerialKit's own boundary already says where this goes: `src/core/` touches no
hardware, `src/boards/<BOARD>/` answers the board contract in `ak_board.h`, and
`src/arch/<MCU>/` is the register layer. So a third MCU is a directory, a board,
and a target - nothing in the flight core changes:

```
src/arch/at32f435/     regs.h, clk.c, gpio.c, systick.c, system.c, usart.c,
                       spi.c, i2c.c, adc.c, output.c (TMR + DMA), flash.c, usb.c
src/boards/AERIALKIT_GHF435/   board.c, board.h - the pin table above
targets/aerialkit-ghf435/      target.conf
```

The order to write it in is the order the F405 port was written in, because that
order was decided by what can be checked without a board
([21-port-on-the-host.md](21-port-on-the-host.md)):

1. **the clock and the tick** - and a host test that checks the PLL arithmetic
   against the numbers above, since a clock that is wrong is a clock that is
   wrong everywhere;
2. **the console** (USART or USB), because a board that cannot talk cannot be
   brought up;
3. **the configuration record** in the internal flash, with the 2 KB sector
   arithmetic checked against the modelled controller;
4. **the sensor buses** (SPI1 for the gyro, I2C2 for the baro), whose drivers
   already exist and are bus-agnostic by design;
5. **the outputs** (TMR4/TMR2, DShot and servo PWM);
6. **the receiver and the GPS** (USART2, USART3 with the swap bit);
7. **the pack** (ADC, and the current shunt if the pilot wants it).

What can be claimed at each step is bounded by the same rule as everything else
here: a host test that runs the real driver against a modelled register file
proves the arithmetic and the sequence, a build proves the linker placed it, and
**nothing about this part can be verified without the board, a cable, and
permission to flash it.**

### The first step, as it now exists

`src/arch/at32f435/` holds the chip's memory map, its clock unit, the flash
divider, the regulator, the GPIO ports and the core's own tick - and three of
the seven steps of the order above:

- **the clock**: 8 MHz crystal to 288 MHz through ms/ns/fr, with the regulator
  raised to 1.3 V first, the flash controller's divider set and waited for, the
  crystal's stability and the PLL's lock each waited on, and a fallback to the
  internal 8 MHz clock - named, and reported - if the crystal never starts or is
  one this part cannot multiply to exactly 288 MHz (the board *declares* its can
  and the PLL's three numbers are derived from it, because this part has no way
  to measure one - `clk.c`'s header and traps 203);
- **the pins**: an output, an alternate function with a pull, an open-drain bus
  pin and an analog channel, each written through this part's own scheme. There
  is no MODER and no AFR here: a pin's mode is two bits in one register, and its
  function number is a nibble in `muxl` or `muxh` depending on which half of the
  port it is - which is the mistake a port written by pattern-matching the F405
  would make, and one of the checks exists because of it;
- **the configuration record's flash**: this part has **2 KB pages** where the
  F405 has 16 to 128 KB sectors, and it is **two banks of 512 KB** on a 1 MB
  part, each bank with its *own* unlock, status, control and address registers -
  so which controller a write goes to is decided by the address, and a driver
  that always used the first bank's registers would erase the wrong one. Page
  erase and word program run against a modelled controller
  (`tests/host_flash_model_at32.c`), and the checks read the flash back rather
  than the model, because a model with the wrong geometry erases the wrong
  region and reports success;
- **the outputs**: motors and servos, which is the step that makes this port an
  aircraft. This part's timers run at the **system clock** - 288 MHz - so the
  servo tick and every DShot bit time are counted in different numbers from the
  F405's 84 MHz: 960 ticks per bit at DShot300, where the F405 needs 280. The
  servos are ordinary PWM on TMR2 (PB8/PB9, mux 1) and the motors are DShot on
  TMR4 (PB6/PB7, mux 2). The bit values reach the compare registers **one DMA
  channel per motor** through this part's request multiplexer, rather than the
  single burst the F405 uses: the burst registers exist here (`dmactrl` and
  `dmadt` are the F4's DCR and DMAR), but the only DShot implementation for this
  chip that exists to read - INAV's - uses per-channel requests, and the burst's
  length encoding has no reference to check it against. The first bit is loaded
  by hand and the DMA carries the rest, which is the F405's own off-by-one; and
  the frame the DMA walks is the *transposed* encoder output, which the check for
  "each channel carries its own motor's bits" is there to hold;

  **And the four pads are checked against the aircraft's own target now.**
  Which pad a servo is on is the one part of a port no host can derive - a wrong
  one is a surface that does not move - so `tests/test_board_ghf435.c` asserts
  both the pads and their muxes against the board that flies:
  `upstream/inav-9.1.0/src/main/target/TWINWINGS_GHF435V2/target.c`'s
  `DEF_TIM(TMR4, CH1/CH2, PB6/PB7, ...)` for the motors and
  `DEF_TIM(TMR2, CH1/CH2, PB8/PB9, ...)` for the servos, with mux 2 and mux 1
  read out of the mapped mux registers - which is also what says *which timer*
  each pad belongs to, in this part's own encoding. Moving one pad to a
  neighbour's costs that check and nothing else, which is the point: the board
  file writes the pins in two places and the second check reads the *driver*
  rather than the header, so the "they cannot drift apart silently" comment in
  `board.h` is now something the suite enforces rather than something it hopes;
- **the pack's ADC**: one channel, one conversion at a time, polled, with this
  part's two differences - the prescaler lives in a *common* block and divides
  the **AHB** clock, so from 288 MHz the value to ask for is a divide by eight
  (not the F4's divide by four of a 42 MHz bus), and the status flags are
  cleared by writing a **zero** to the bit, the opposite of the flash
  controller's flags two files over. The second one was a real bug in the first
  version of this file: it cleared the flag by writing a one, which would have
  left a stale end-of-conversion set and answered every battery reading with the
  *previous* conversion's value. The check that caught it is in the test file,
  and the fix cites Artery's own `adc_flag_clear()` writing the complement of the
  flag for exactly that reason;
- **the interrupt-driven receive** the receiver and the GPS need: one ring per
  port, filled by the interrupt and emptied by the main loop, so a receiver
  talking at 420000 baud and a GPS at 9600 cannot lose each other's bytes - a
  single ring shared between them interleaves two protocols into nonsense, and
  there is a check that says so. Only the byte-arrived flag is enabled; the
  error flags are handled by the same two accesses that clear them, and what the
  ring cannot keep is counted. The handler checks that a byte is actually there
  before taking one, because an interrupt that fires for another reason must not
  put a byte of nothing into a protocol;
- **the barometer's bus, as far as its timing**: this part's I2C is the *newer*
  peripheral - one `clkctrl` register describing the whole bus, rather than the
  F405's clock-control and rise-time pair - so the arithmetic is not the F405's
  renamed, it is the other one, with the I2C specification's rise/fall/setup
  times and a prescaler that has to be *searched* for until every field fits its
  width. At 144 MHz and 400 kHz that lands on `0x30d80f3b`, which decodes back
  to 401 kHz. A wrong one does not fail: it runs the bus at the wrong speed, and
  a bus too fast reads exactly like a barometer that is not answering - which is
  why the rate is decoded back out of the register rather than compared with a
  constant. The pins are open drain with pull-ups, which I2C requires and no
  other bus in this firmware does. What is *not* here yet is the transfers - a
  start, an address, a register number, the bytes - which are the next step and
  want the modelled slave the F405 port already has;
- **the sensor bus**: SPI master, mode 3, eight bits, software chip select -
  what the inertial part on this board wants and what the F405's SPI was written
  for too. The one number that is not the same is the divider, because this
  part's SPI1 sits on a 144 MHz bus where the F405's sits on 84: the setting is
  chosen to land near the same 10 MHz. This part also splits its divider between
  two registers (`mdiv_l` and `mdiv_h`, plus a divide-by-three bit), so the init
  clears what it does not use rather than assuming a reset state. What a host
  test can show is the configuration and the shape of a transfer; what a device
  answers is a bench question, and the bring-up checklist's loopback jumper is
  where it is answered;
- **the console's port**: configure at a baud rate, attach the sink, write, and
  poll for a typed byte. `baudr` holds the peripheral clock over the baud rate -
  the same value the F405's BRR holds and for the same reason, and the same
  arithmetic this project has already had sixteen times wrong once. This part
  adds one thing the F405 cannot do at all: a **swap** bit that exchanges the
  transmit and receive pins inside the peripheral, which is how the wing's GPS
  port is wired, so `ak_uart_init_swap()` exists here and has no counterpart
  there. The interrupt-driven receive rings the receiver and the GPS need came
  with them - one per port, so a receiver at 420000 baud and a GPS at 9600
  cannot eat each other's bytes - and that is where this list and the port
  diverged: the list was written before the port was, and the paragraph after it
  is where the port actually got to;
- **the tick**: SysTick, which is the core's rather than this part's, with a
  reload of a millisecond of whatever the clock code ended up on - so a board
  that fell back to the internal clock still counts in milliseconds, which is
  the only reason the fallback is worth having.

Two hundred and one checks in `tests/test_arch_at32.c` run the real drivers
against a mapped register block: the register arithmetic decoded rather than compared, the
bus dividers by what they mean, the in-force flash divider, the fallback, the
PLL formula on its own, where each pin's function number lands, the set and
clear strobes, and a tick that follows the clock. They are compiled into the
same test binary as the F405 port - one arch per firmware, two of them on a host
- which is why this port's entry points are renamed under `AK_HOST_AT32`.

Six of them are there because of what the board file turned out to need: this
part gives each *pin* its own function number, and the wing's receiver is the
proof - USART2 listens on PB0 with function 6 and talks on PA8 with function 8,
so an entry point that takes one number for both pins is a receiver that never
hears anything. The GPS's port needs a third thing instead: its two pins share
function 7 and the *peripheral's* swap bit is what puts the firmware's transmit
on the pad the module is listening to. Those facts lived in an API that could
not express them, which is what writing the board file found.

Two of the checks were made by breaking the code on purpose, and both found
something:

- with the PLL multiplier changed from 72 to 36, the *reported* frequency used
  to stay at 288 MHz - the firmware was repeating the number it meant to ask for
  instead of decoding the register it wrote. It reports what it configured now,
  read back, and the same sabotage fails both checks. (The 72 is no longer a
  literal to change: since 2026-09-29 the three numbers come from
  `ak_at32_pll_cfg_for()`, so the sabotage is done there and the same two checks
  still catch it.)
- with every mux function number written to the low register (the shape a
  pattern-matched port has), a pin above the seventh writes its function to the
  wrong place and the check says so - which is the failure that would otherwise
  look like a servo that simply does not move;
- with the divisor multiplied by sixteen as well as divided - the exact bug the
  F405 port had for its whole life before a host test found it - four checks
  fail at once, including the console's, which is the difference between a
  console that prints nothing legible on the bench and one that prints;
- with the frame's transpose removed, so every DMA channel would send the first
  motor's bits, the check for the hand-loaded first bit fails - which is the
  difference between an aircraft with two motors and one with two copies of the
  same motor;
- with the ADC's status flag cleared by writing a one instead of a zero - the
  mistake this part invites by having *both* conventions in it, one file apart -
  the check that a conversion cannot be faked says so. That one was found while
  writing it rather than by breaking it afterwards, which is the better way
  round;
- with the receive handler no longer checking that a byte is there, a spurious
  interrupt puts a byte of nothing into the stream, and the check says so;
- with the I2C prescaler search stopped at one - so the setup field overflows and
  the bus cannot be configured at all - four checks fail, which is the difference
  between a barometer that answers and one that is simply not there;
- with the second SPI control register left uncleared - the assumption that a
  register resets to zero stays zero - the bus checks its own divider and fails;
- with every bank using the first bank's registers, the second bank's checks
  fail, which is the difference between a configuration that saves and an erase
  aimed at the wrong controller. That one was written *after* the first version
  of the driver did not refuse a write that crossed the bank boundary - the
  check that found it is still there, and it is the one that would otherwise
  have turned a saved record into half a firmware.

## What this survey decides

1. **The wing's board gets a port of its own, as a third arch directory**, not a
   fork of the F405 one and not a shared "STM32-like" layer. The two parts agree
   on less than they disagree about, and a shared layer would be a layer that is
   wrong for one of them.
2. **INAV is the only reference implementation for this chip**, so its AT32
   drivers are the oracle - read, cited by file and revision, and reimplemented,
   with Artery's library read for the arithmetic and never copied.
3. **The first three steps are the ones worth writing before a board is
   involved** (clock, console, configuration record): they are the ones whose
   errors are silent, whose arithmetic is checkable on a host, and which nothing
   else can be built on top of.
4. **The blackbox stays in the internal flash for the first version.** The board
   has 16 MB of external SPI flash and AerialKit has no driver for it; the
   internal 1 MB with 2 KB sectors still holds a log worth having, and adding a
   driver for a part nobody has read yet - on the one bus a mistake costs the
   flight - is not a first step.
5. **It is a multi-session job, and the first step of it is now on the bench
   instead of on this page**: `src/arch/at32f435/` has the chip's register map
   and its clock, checked against a mapped register block by
   `tests/test_arch_at32.c` the same way the F405 port was checked before its
   board existed ([21-port-on-the-host.md](21-port-on-the-host.md)). What that
   leaves is everything else in the list above, and the milestone order in
   [01-plan.md](01-plan.md) still has this target after M7 for the reason it
   always did: none of it can be verified without that board, a cable and
   permission to flash it. What would move it up is a decision to fly the wing
   before buying another flight controller - which is the owner's.

## Where the port got to

The list above is a plan; what is in the repository now is the whole of it, and
the three things that turn a port into a target:

- **`src/arch/at32f435/`** - the register map, the clock, the pins, the tick,
  the console, the flash, the sensor bus, the I2C timing, the receive rings, the
  ADC and the outputs, with the checks described above;
- **`src/boards/AERIALKIT_GHF435/`** - the board file: the console on USART1,
  the onboard receiver on USART2 (the live-verified fact), the GPS on USART3
  with the swap bit, the ICM-42688P on SPI1, the barometer's bus on I2C2, the
  two motors and two servos, the pack on ADC1, the blackbox in the internal
  flash, and the LED. Each line says where it comes from and which of them are
  read rather than measured;
- **`linker/at32f435rg.ld`** and **`scripts/image-facts/at32f435rg.txt`** - the
  memory map it is linked against, and the numbers its built image is measured
  against: the stack top at the top of the 128 KB region, and a **74-word**
  vector table, which is eighteen more words than the F405's because this part's
  motor DMA channels are interrupts 56 and 57 rather than 15;
- **`targets/aerialkit-ghf435/`** in the workspace - the target, which builds the
  image, runs the host tests, the portability check and the protocol check, and
  then measures the image it built. `scripts/fw build aerialkit-ghf435`
  publishes it to `dist/aerialkit-ghf435/`.
- **the board file is run by a host, not only compiled.** `aerialkit-tests` used
  to link one board file - the F405's - so this board's console, barometer bus,
  configuration record and log layout were checked by the compiler and by
  nothing else. The Makefile links both boards now, with this board's entry
  points renamed for the host build alone, and 25 checks drive it: the console
  it attaches, the bus it hands over (a register read through it comes back from
  a modelled DPS310), the record it saves and loads through this part's flash
  controller, the six regions of its blackbox, the two function numbers on the
  receiver's pins, the GPS's swap bit, the pack, the outputs and the absence of a
  network.

  **And that is where the port's one remaining silent failure was.** The
  configuration record is 2060 bytes and this part erases 2 KB pages, so the
  board's own guard - "the record fits in one page" - was always true, and
  `ak_board_config_write` was dead code: every `save` would have done nothing at
  all, on a board where the console says it saved. The record spans two pages
  now, both erased first, with a `_Static_assert` that fails the build if it ever
  grows past them.

  **And this part's erase unit is what made the save safe** (2026-09-18, after
  the F405's own config had a failed-write hole): a slot is those two pages and
  a save erases *only its own slot*, so the record that is already there is
  never inside an erase - which is a property the F405 cannot have, with its
  128 KB erase. The ring holds thirty-two of them and the serial decides which
  is newest; the F405's board file points here for the difference.

- **And the board says what it drives, so the firmware can refuse to fly a mix it
  cannot.** `ak_board_output_shape()` answers two motors and two servos here, and
  that is the wing's mix exactly. It matters on *this* board more than on the
  other two, because the AIO it is built from is a quadrotor's: four motor pads,
  and the wing gives up the two on TMR2 so its elevons can have them. So
  `airframe=quad` on this board is a build mistake rather than a preference, and
  arming refuses it with both numbers printed - see the first gate in
  [04-flight-core.md](04-flight-core.md).
- **the barometer's transfers**, which is the last piece of the port proper -
  and the last thing the first version of this page listed as missing. This
  part's I2C does not move bytes the way the F405's does: the address, the
  direction and the byte count are one register write, the peripheral performs
  the address phase and the acknowledge itself, and the count is what ends the
  transfer. So the sequences are Artery's own master routines read as the
  oracle, and they run against a modelled device
  (`tests/host_i2c_model_at32.c`) the way the F405's run against theirs. **That
  model found a real bug the first time it ran**: the mask that clears the
  fields a transfer owns left out the byte count, so each transfer OR'd its
  count into the previous one's - a barometer that answers once after a boot and
  times out after that. [20-i2c.md](20-i2c.md) has the whole story.
- **the USB console**, which is the F405's driver re-derived rather than a
  second design: this part carries the same Synopsys full-speed OTG core at the
  same address with the same register layout, and the three things that are not
  the same are all in one function - the core comes out of reset **in
  power-down** (`gccfg.pwrdown`, which Artery's own library sets right after the
  reset), the phy clock is **gated** (`pcgcctl.stoppclk`), and the field at
  GUSBCFG bit 6 is the **turn-around time** rather than the F405's transceiver
  select - so the F405's line there would write zero into this part's timing.
  The clock is this part's too: the USB peripheral is fed from the PLL and
  divided by six, 288 to 48 MHz. `tests/test_arch_at32.c` runs the whole device
  path - enumeration, the descriptors, the class requests a serial driver makes,
  typing, a bus reset - and checks those three registers and the clock divider
  on the way, which the F405's own test cannot. One more thing is this part's
  and was *missing* from the port until the USB work compared it with the
  reference: its two data pins want the **stronger driver**, and this part's mux
  mode does not set the drive field at all - the F405's `ak_pin_af` writes a
  slew rate for every mux pin, this one's has a separate field, and the reset
  value is not what a 12 Mb/s pair wants. `ak_pin_drive()` is that field on its
  own, and the USB pins are its one caller.

The shared reset path had to change shape for this to work at all. The vector
table lived in `src/arch/arm/cortex-m4/startup.c`, which both parts compile -
and it was the F405's table, with `DMA1_Stream4_IRQHandler` at index 31. A table
is a part's fact and not the core's, so each arch provides a `vectors.h` now and
the shared file includes it: one reset path, one copy of the FPU enable and the
fault capture, and two tables. That is the same split as the pin-function tables
one layer up, for the same reason - and `scripts/check-image.sh` reads the
length and each named entry out of the part's facts file, so a table that drifts
from its part fails the build instead of producing a board that does nothing.

What is still open, and each of these is a step rather than a detail:

- **nothing has been flashed.** No image has been on this board, so the clock,
  the console and every pin below it are read from the reference target rather
  than measured. The first thing to look for is a banner - at 115200 on USART1,
  or on the USB port the board is already plugged into for flashing, where it
  comes up as `/dev/ttyACM0` (2E3C:5740, the vendor id this part's own code
  uses, so `lsusb` says which of the two boards is on the cable).
