> Historical notes from the full development tree. Current public targets and commands are in the [firmware guide](../../docs/flight-controller-firmware.md). Host C tests and simulator harnesses referenced below are retained outside this public source tree.

# AerialKit - decisions

Written before the first line of code, as the goal asked. Each entry says what
we chose, why, and what would make us revisit it.

## 1. Where the firmware lives - decided, then reversed

**Reversed 2026-09-15, on the owner's call.** AerialKit is the firmware's
*name*, not yet a repository of its own. The firmware lives **inside the
private `fc-firmware-workspace` repository**, at `firmware/`, while it is
being built and tested, and moves to a public repository of its own once it
has been tested. The tree moved in as `firmware/` **with its history** - a
subtree merge, so the eventual public repository can be produced by splitting
that directory out and keeping the log rather than starting again at an
initial import - and both `targets/aerialkit-*/target.conf` files now say
`UPSTREAM_ROOT=aerialkit`, one line each, exactly as the note below predicted.
Both targets were rebuilt from the new location before the sibling copy was
left alone: F405 host checks (739 then, 853 with the USB console, the modelled
flash controller and the blackbox in flash), the image, and the client against
the firmware's own protocol; ESP32 thirty-two checks under QEMU.

Everything below is the original decision, kept because the reasoning is still
the reasoning - it is a decision that was made deliberately and reversed
deliberately, not a mistake to be tidied away. Paragraphs that name
`<historical-firmware-root>` or `../aerialkit` describe the layout as it was before the move.

### As originally decided

**Decision.** AerialKit is its own repo, checked out as a sibling of
`fc-firmware-workspace` (`<historical-firmware-root>` on the NAS, `firmware/`
on the laptop). The harness reaches it through
`targets/aerialkit-f405/target.conf`, which sets `UPSTREAM_ROOT=../aerialkit`.

**Why.** It is ours, not an upstream, so it gets its own history and version
tags. It builds and flashes without the harness, which matters the day we want
to build it on a machine that does not have the four reference families
checked out. And the workspace keeps its model intact: `upstream/` stays
pristine clones pinned in `revisions.json`, and nothing about the four existing
families changes.

**The alternative considered.** Tracking the sources inside the workspace (say
`firmware/` next to `upstream/`) would have removed the sibling-path
assumption. Rejected: the firmware would then carry the build harness's release
rhythm, and a firmware repo that can be cloned on its own is worth more when
the ESP32 tooling arrives.

**Revisit if.** The sibling layout turns out to be a burden on either host —
the fix is one `UPSTREAM_ROOT` line, not a restructure.

## 2. Bare metal, our own register layer, no libc

**Decision.** No RTOS for now. One `main()` driving a cooperative loop, with
interrupts only where they earn their place (SysTick first). The MCU layer is
our own register definitions, taken from RM0090; we do not use ST's HAL, CMSIS
device headers, libopencm3 or ChibiOS. The image links `-nostdlib` against
libgcc only, with our own `memcpy`/`memset`/`memmove`/`strlen` and our own
startup code.

**Why.** A flight controller's failure modes are timing and invisibility, so
the fewer layers and the fewer license obligations between us and the silicon
the better. `-nostdlib` also means there is no hidden heap, no `malloc`, and no
libc lock in an interrupt path — the things that make "why did it glitch once"
so expensive.

**What we give up.** We write our own `printf`-lite, startup and delay
primitives, and we carry the register definitions ourselves. That is a few
hundred lines once, in exchange for being able to read all of them.

**Revisit if.** The estimator and the scheduler outgrow a superloop, or an
ESP32 port proves that ESP-IDF's FreeRTOS is the only sane way to use its
radios. The port boundary in `src/core` vs `src/arch` is what makes that
reversible.

## 3. STM32F405 first, and the port boundary from the first commit

**Decision.** The first target is STM32F405 on the WeAct F405RGT6 dev board.
`src/core/` is portable and may not include an MCU header; `src/arch/<mcu>/`
and `src/boards/<board>/` hold everything else.

**Why the F405 and that board.** The CPU is the one both airframes in this
project can plausibly use, the workspace already carries an ARM GCC for it, and
the WeAct board is the one board here with a proven write path: the
`projects/f405-bench` work wrote all four reference families to it over the ROM
DFU bootloader, so flashing AerialKit is the same operation rather than a new
one.

**Why the split now.** The ESP32 is a different instruction set, a different
interrupt model and a different radio story. Writing the F405 code with that
port in mind costs nothing today and saves the rewrite later.

**Note on the wing's board.** The GHF435 AIO V2 that flies the twin-wings wing
is an **AT32F435** (Artery), not an STM32F405. AerialKit support for it is a
separate, later target; nothing in this repository claims it works there.

## 4. License: GPL-3.0-or-later

**Decision.** AerialKit is GPL-3.0-or-later.

**Why.** The three families whose algorithms we will read most closely —
Betaflight, INAV and ArduPilot — are GPLv3. Any real porting of their code or
their derived algorithms into AerialKit keeps the whole work under GPLv3, and
deciding that now avoids a licensing archaeology exercise later. PX4 is BSD-3
and ESP-IDF is Apache-2.0, so code from those is compatible either way;
permissive-only dependencies inside GPLv3 code are fine, the reverse is not.

**Rule that follows.** Anything copied or closely derived gets a source
comment naming the project, the file and the revision, and a line in
[docs/03-attribution.md](03-attribution.md). Reading a driver to understand a
register sequence and then writing our own is the default and needs no entry;
copying text does.

**Revisit if.** We decide AerialKit should be linkable into proprietary work,
which the current scope does not need. That decision has to be made before
GPL-derived code lands, not after.

## 5. The flight core does no I/O, and airframes are mixer tables

**Decision.** `src/core/flight/` includes no MCU header, does no I/O, allocates
nothing and calls no libc - our own `sqrtf` and `atan2f` included. A mixer is a
table of coefficients, one row per output, and adding an airframe means adding
a table, not a branch in the control code.

**Why.** Two consequences we wanted. First, the control laws can be tested on
the development machine in a second, so a gain change costs `make test` instead
of a flash and a walk to the bench. Second, the only place a control sign lives
is one number in one row, which is the difference between "the rudder is
reversed" being a five-second fix and being an afternoon.

**What it costs.** The board layer has to gather samples and emit outputs,
which is real work that a HAL would have hidden. It also means the selftest has
to be written against an injected print function rather than calling `printf`,
which is a small awkwardness in exchange for the property above.

**Revisit if.** A port turns out to need a facility the core cannot express
without MCU headers. The fix is a new function in `ak_board.h`, not an include
in `src/core/flight/`.

## 6. The ESP32 target uses ESP-IDF, and AerialKit for the aircraft

**Decision.** The ESP32 port is an ESP-IDF application with AerialKit's portable
core inside it, rather than a bare-metal port like the STM32's.

**Why.** The two chips are different problems. The STM32F405 starts the way
this firmware expects - a vector table at the base of flash, our startup code,
our registers - and a vendor HAL there would have cost more than it saved. The
ESP32 does not: the ROM bootloader wants an app image with a header, a segment
table and a checksum, the flash cache has to come up before anything executes
from mapped memory, and the reason this target exists at all is the radio,
which is a vendor blob and a co-processor rather than a register map.

**What stays ours.** The flight core, mixer, estimator, own math, parameter
table, console and blackbox - all of it already compiles for xtensa and riscv32
unchanged (`make port`). IDF supplies the boot chain, the image format, Wi-Fi
and FreeRTOS underneath; it does not get to own a control law.

**Revisit if.** The IDF build turns out to drag the core's timing into a shape
it cannot hold - a 1 kHz loop with jitter we cannot bound would do it. The port
boundary makes that reversible: the core does not include an arch header.

## 7. The console can hand the part to its ROM bootloader

**Decided on 2026-09-17**, when the wing's board became a target. That board is
put into DFU by hand with a button **and a solder joint** - its own drawing says
so - and neither has been tried on this physical board, because the firmware it
arrived with (Betaflight, then INAV) could reach the ROM by command. AerialKit
would have been the first firmware on it that could not be replaced from a
terminal, which is a one-way flash with extra steps.

So the console has a `dfu` command, the board contract has
`ak_board_enter_bootloader()`, and the arch layer has the jump: the ROM's first
two words are a stack pointer and a reset handler, and setting one and branching
to the other is the bootloader starting itself up. INAV does the same thing on
this part and on the STM32F405, which is where the sequence and the address
came from ([03-attribution.md](03-attribution.md)).

**What it costs.** Two functions per board, one per arch, and a command - and on
the host build an empty pair of them, because a process on a Pi has no AIRCR to
write and nothing at 0x1FFF0000 to jump to.

**What it does not fix.** A firmware that does not run at all cannot hand
anything over, and that is the case the solder joint exists for. This is the
case where the firmware is healthy and somebody wants to flash something else,
which is every reflash after the first one.

**Revisit if.** The wing's board turns out to have its BOOT joint already
shorted - then this is convenience rather than the difference between a
flashable board and an unflashable one, and it stays anyway: the ESP32's board
answers that it has no such path rather than pretending, and a board that says
which of the two it is is worth the four lines.

## 8. The boot says where it stopped, and the tell-tale is a build flag

**Decided on 2026-09-18**, after the F405's first two bench sessions produced
the same sentence - "the LED is steady and the console is silent" - and four
hand-built diagnostic images were made to answer it. Those images worked and
were the wrong shape: each was a whole-tree patch against the revision of the
moment, kept in a directory outside the repository, so every one of them was a
firmware that stopped existing the moment the tree moved. The fourth predates
the USB decode fix, which means the most recent thing the file was built to
diagnose is no longer the thing in the file.

So the question moves into the firmware, in two halves:

* **Where the boot is** is a record every image keeps. `src/core/ak_boot.h`
  numbers thirteen stages, `ak_boot_mark()` is called by the code that does
  each one - four of them in the board file, nine in `main()` - and the
  simulator asserts at the end of a session that the boot walked all of them.
  It costs a store and a call per stage.
* **Saying it out loud on a board with no console** is a build:
  `-DAK_BOOT_STAGE=1` makes the status pin blink the stage number three times
  over, and a crash ten quick blinks and then the stage it reached.
  `targets/aerialkit-f405/target.conf` publishes that build as
  `dist/aerialkit-f405/aerialkit-f405-telltale.bin` on every build.

  **Both ARM boards publish it** (2026-09-18, later the same day): the wing's
  board added `dist/aerialkit-ghf435/aerialkit-ghf435-telltale.bin` to its own
  target, because it is in the position the F405 was in when the instrument
  was invented - a console nobody has ever read (the two ports share the USB
  driver, and the setup-byte bug was in both copies), and the awkward DFU
  entry, since the BOOT joint has never been shorted on that board. And
  `make check` asserts that each image is what its name says: the tell-tale's
  `ak_board_boot_mark` is the real one with its blink plan linked, the flight
  image's is the two-instruction stub
  (`scripts/check-image.sh --telltale`). A tell-tale that lost the flag blinks
  nothing, which reads exactly like a boot that stopped at stage zero.

**What it costs.** In the diagnostic image, everything a blink needs: the two
plan builders, the runner, the board's loop and 324 bytes of RAM for the
pattern. In the *flight* image almost nothing - `nm` on the shipped
`aerialkit-f405.bin` shows two symbols for the whole thing, `ak_boot_mark` at
twenty bytes and the board's empty `ak_board_boot_mark` at two, with the plans,
the runner and the stage names all dropped by the linker because no flight path
calls them. Seventeen marks at four bytes each is the rest: **about ninety
bytes of flash, and a store and a call that do nothing**.

**What it does not fix.** A board whose LED is not where the board file says it
is, or whose LED is a different one - the power LED on this bench board is
steady whenever the board has power, which is the same thing to look at. The
tell-tale is a *second* instrument beside the console, not a replacement for
one, and the first question at the bench is still which of the two lights is
being watched.

**Revisit if.** A board needs the blink and its console works from its first
instruction (either ESP32): then the flag is off there - the tell-tale is for
the board whose only door is a USB device that may not enumerate. If a part
appears whose status pin is not one GPIO, the pattern arithmetic moves and the
pin handling in the board file is what changes.
