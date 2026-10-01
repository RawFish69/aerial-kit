# AerialKit Firmware
#
#   make [BOARD=AERIALKIT_F405] [ARCH=stm32f405] [PART=stm32f405rg]
#        [OUT=build] [CROSS=arm-none-eabi-]
#
# The cross-compiler has to be on PATH, or named with CROSS. The workspace
# harness puts its shared toolchain there; see target.conf in
# fc-firmware-workspace/targets/aerialkit-f405/.
#
# REV and STAMP are embedded in the image. That is deliberate: it is what makes
# fc-firmware-workspace/scripts/compare-firmware.sh able to tell "same code,
# later build" apart from "different code", the way it does for the other
# families.

BOARD   ?= AERIALKIT_F405
ARCH    ?= stm32f405
PART    ?= stm32f405rg
PRODUCT ?= aerialkit-f405
OUT     ?= build
CROSS   ?= arm-none-eabi-

# --- three variables, two of them wrong together -----------------------------
#
# BOARD, ARCH and PART each have their own default and nothing relates them, and
# two of the combinations that produces are wrong in ways whose first symptom
# names something else. Both measured on 2026-09-28:
#
#   make BOARD=AERIALKIT_GHF435
#     -> `'GPIOH_BASE' undeclared ... did you mean 'GPIOA_BASE'?` in the middle
#        of that board's pin table, because ARCH was left at stm32f405. It reads
#        as "this board does not build". The board builds.
#   make BOARD=AERIALKIT_F405 ARCH=stm32f405 PART=at32f435rg
#     -> links cleanly and writes a .bin. `make check` then fails three vector
#        assertions - "vector 54 is not USART2_IRQHandler ... so the receiver
#        hears nothing" - which read as a receiver and a DShot bug. The image is
#        the F405's, linked against the AT32 part's memory map: RAM at
#        0x20010000 instead of 0x20000000, and a different flash layout. It is
#        the one of the two that gets as far as an artifact.
#
# Neither is a missing feature. Both are a board, an arch and a part that do not
# go together, and refusing them by name here costs one line and saves the
# reading. The arch a board belongs to is declared by the board itself
# (`AK_BOARD_ARCH`, in its board.h); a part belongs to the arch its name starts
# with, which is the convention `linker/` already uses for the script names. The
# three ARM boards declare it; the four ESP32 ones do not and are not meant to,
# because they are not built here at all - `scripts/esp32-families.sh` builds
# them with the RISC-V toolchain, and their board.h includes `esp.h` rather than
# `arch.h`. A board that declares nothing takes part in neither check.
BOARD_ARCH := $(shell sed -n \
    's/^#define[ \t]*AK_BOARD_ARCH[ \t]*\([A-Za-z0-9_]*\).*/\1/p' \
    src/boards/$(BOARD)/board.h 2>/dev/null | head -n 1)
ifneq ($(BOARD_ARCH),)
ifneq ($(ARCH),$(BOARD_ARCH))
$(error BOARD=$(BOARD) is an $(BOARD_ARCH) board, but ARCH=$(ARCH). Build it with ARCH=$(BOARD_ARCH) - docs/28-build.md has the whole invocation, PART and PRODUCT included)
endif
endif
ifeq ($(filter $(ARCH)%,$(PART)),)
$(error PART=$(PART) is not an $(ARCH) part. A part is named <arch><package>, and linker/$(PART).ld is the script it is linked against - docs/28-build.md lists the pairs)
endif
# And BOARD has to name a directory that exists. A misspelling does not stop the
# build - there are no board sources to fail on - so it compiles the whole core
# against an include path that is not there, writes those objects into whatever
# OUT was (`build`, the F405's, by default), and fails much later with the
# objects left behind. Twelve characters was the whole of it: `AERIALKIT_GHF43`.
ifeq ($(wildcard src/boards/$(BOARD)),)
$(error no board directory src/boards/$(BOARD). This tree's boards are: $(notdir $(wildcard src/boards/*)))
endif

CC      := $(CROSS)gcc
OBJCOPY := $(CROSS)objcopy
SIZE    := $(CROSS)size

REV     ?= $(shell git -C $(CURDIR) describe --always --dirty 2>/dev/null || echo unknown)
STAMP   ?= $(shell date +'%b %d %Y / %H:%M:%S')

LDSCRIPT := linker/$(PART).ld
INCLUDES := src/core src/core/flight src/core/sensors src/arch/$(ARCH) \
            src/arch/arm/cortex-m4 src/boards/$(BOARD)

SRCS := $(wildcard src/core/*.c) \
        $(wildcard src/core/flight/*.c) \
        $(wildcard src/core/sensors/*.c) \
        $(wildcard src/arch/arm/cortex-m4/*.c) \
        $(wildcard src/arch/$(ARCH)/*.c) \
        $(wildcard src/boards/$(BOARD)/*.c)
OBJS := $(SRCS:%.c=$(OUT)/%.o)
DEPS := $(OBJS:.o=.d)

# --- the host build: the same flight core, compiled for this machine ---------
#
# src/core/flight/ is portable by construction - it includes no MCU header and
# does no I/O - so a control-law change can be tested in a second instead of a
# flash cycle. `make test` also cross-checks our own sqrt/atan2 against libm,
# which the target image cannot do because it does not link libm at all.
HOST_CC     ?= cc
HOST_OUT    ?= build-host
HOST_CFLAGS ?= -std=c11 -O2 -g -Wall -Wextra -Wshadow -Wconversion \
               -Wdouble-promotion -Isrc/core -Isrc/core/flight \
               -Isrc/core/sensors -Itests
HOST_CFLAGS += -DAK_PRODUCT=$(PRODUCT) -DAK_BOARD=host -DAK_REV=$(REV)
# The flags the objects' staleness is keyed on: everything above, and
# deliberately not the build stamp below it. `$(STAMP)` is
# `date +'%b %d %Y / %H:%M:%S'` - a reading of the clock - so a stamp file
# written from a variable that contains it changes on every invocation, and
# every object that depends on that file is then permanently out of date. That
# is not "up to date"; it is "always dirty", and it is the more expensive
# failure of the two because it is silent: two consecutive `make host` runs
# with nothing changed between them recompiled 139 objects each, which was every
# object that build made at the time - it is 140 now, because trap 190's rule
# has joined it. `make host` is not "the whole host tree": the tree holds 179
# objects across all its targets. The stamp value still reaches every object - it is
# in `$(HOST_CFLAGS)`, which every compile and link line uses - it is only kept
# out of the *key*, because a dependency cannot be keyed on the time of day.
HOST_CFLAGS_KEYED := $(HOST_CFLAGS)
HOST_CFLAGS += -DAK_STAMP_VALUE="$(STAMP)"

# `$(HOST_CFLAGS_TARGET)` is empty here and is filled in per object, by the
# forty-nine `$(HOST_OUT)/....o: HOST_CFLAGS_TARGET += ...` lines further down.
# It exists as a variable of its own for one reason: so that the per-object
# flags can be *in the key* without the clock being in it as well. `$(HOST_CFLAGS)`
# carries `-DAK_STAMP_VALUE="$(STAMP)"` by the time any recipe runs, so a stamp
# written from `$(HOST_CFLAGS)` is written from the time of day - trap 188, and
# the reason `$(HOST_CFLAGS_KEYED)` exists. But target-specific additions are
# made to `$(HOST_CFLAGS)`, which means they were in *no* key at all: editing
# `-DAK_HOST_AT32` on one line rebuilt nothing, because the one shared stamp
# file's contents are the global flags and the object's own timestamp is older
# than nothing. Measured on 2026-09-24 with `REV` pinned so the dirty bit could
# not be the cause: changing a global keyed flag recompiled 140 objects,
# changing a per-target flag recompiled **0**, and `make -n` shows the changed
# define on that object's command line - so the object left behind is genuinely
# stale and not merely un-rebuilt (trap 189).
#
# Two variables rather than one because the two have different lifetimes, and
# the split is what lets the stamp below be written from `$(HOST_CFLAGS_KEYED)
# $(HOST_CFLAGS_TARGET)` - every input a bench varies, and no clock reading.
HOST_CFLAGS_TARGET :=

# And `$(HOST_FITTED_CFLAGS)`, which is `$(EXTRA_CFLAGS)`'s counterpart for the
# *host* binary's copy of one board - and a variable of its own because the
# obvious cure is the wrong one.
#
# `$(EXTRA_CFLAGS)` reaches `$(CFLAGS)` (below) and therefore the image. It does
# not reach `$(HOST_CFLAGS)`, which is built from scratch above. So the fitted
# configuration - this board's file compiled with the barometer and the divider
# soldered on - was built and never tested: `make … EXTRA_CFLAGS="-DAK_BOARD_BARO_FITTED=1
# -DAK_BOARD_VBAT_FITTED=1" test` reported the bare build's check count to the
# check, and the two defines appear on no host object's command line at all.
# Nothing was red, which is the whole of why it lasted (trap 205).
#
# Adding `$(EXTRA_CFLAGS)` to `$(HOST_CFLAGS)` fits the test as well as the image
# and is still wrong, twice over. A flag that says "the barometer is soldered on"
# is true of one board and this host binary holds four: given to all of them it
# takes five of `tests/test_board_feather.c`'s assertions - the board that
# genuinely carries neither part, and says so - and turns them red (measured,
# `2270 checks, 5 failed`). And `$(EXTRA_CFLAGS)` is not only board facts: the
# tell-tale image puts `-DAK_BOOT_STAGE=1` in it, whose board half is a
# millisecond spin on the status pin - code that has no meaning in a host test
# and every reason not to run in one.
#
# So the fitted facts travel in a variable of their own, and they are scoped to
# the board they are about: `$(BOARD)`, the board the image is being built for.
# The three lines below are that scope - one per board this binary carries - and
# the board and board-test object rules further down use them. The GHF435's
# macros are unconditional `1`s in its header, so the flag is a no-op there; the
# Feather's are guarded the same way the F405's are, so a fitted Feather pass
# would reach its test through the same line rather than through a new one.
HOST_FITTED_CFLAGS ?=
HOST_FITTED_AERIALKIT_F405   = $(if $(filter AERIALKIT_F405,$(BOARD)),$(HOST_FITTED_CFLAGS))
HOST_FITTED_AERIALKIT_GHF435 = $(if $(filter AERIALKIT_GHF435,$(BOARD)),$(HOST_FITTED_CFLAGS))
HOST_FITTED_FEATHER_F405     = $(if $(filter FEATHER_F405,$(BOARD)),$(HOST_FITTED_CFLAGS))

# The parameter table is a core service the flight core itself describes, so
# anything that links the flight objects needs ak_params and ak_text too. The
# console is only needed by the target and the tests, not by the tools.
SUPPORT_SRCS := src/core/ak_text.c src/core/ak_params.c src/core/ak_ring.c \
                src/core/ak_boot.c
CORE_SRCS    := $(SUPPORT_SRCS) src/core/ak_cli.c src/core/ak_proto.c \
                src/core/ak_console_link.c src/core/ak_flashlog.c
FLIGHT_OBJS := $(patsubst %.c,$(HOST_OUT)/%.o,$(wildcard src/core/flight/*.c) \
               $(wildcard src/core/sensors/*.c))
CORE_OBJS   := $(patsubst %.c,$(HOST_OUT)/%.o,$(CORE_SRCS))
SUPPORT_OBJS := $(patsubst %.c,$(HOST_OUT)/%.o,$(SUPPORT_SRCS))
# tests/host_stubs.c is not a test: it is the *simulator's* stand-in for what
# the board supplies for real (the retained-RAM fault record), and the simulator
# gets it through FW_SIM_EXTRA below. The tests binary links the port's own
# fault record instead - the real `ak_fault_record()`, with the frame layout
# that has to be right - so the stand-in is filtered out of this list.
TEST_OBJS   := $(filter-out $(HOST_OUT)/tests/host_stubs.o, \
                 $(patsubst %.c,$(HOST_OUT)/%.o,$(wildcard tests/*.c)))
TOOL_OBJS   := $(patsubst %.c,$(HOST_OUT)/%.o,$(wildcard tools/*.c))
# The F405 port, compiled for this machine and run against a mapped register
# block: the clock, the pins, the UARTs, the tick, the outputs, the flash
# controller, both sensor buses, the ADC and the USB device. Compiling the real
# sources rather than a stand-in is the whole point - the stand-ins are what
# the simulator and the other tests already have.
ARCH_HOST_SRCS := src/arch/stm32f405/clk.c src/arch/stm32f405/gpio.c \
                  src/arch/stm32f405/usart.c src/arch/stm32f405/systick.c \
                  src/arch/stm32f405/output.c src/arch/stm32f405/flash.c \
                  src/arch/stm32f405/spi.c src/arch/stm32f405/adc.c \
                  src/arch/stm32f405/i2c.c src/arch/stm32f405/usb.c \
                  src/arch/stm32f405/system.c \
                  src/core/time.c
# And the AT32's clock, for the same reason: the third target's port is written
# before its board can be flashed, and the clock is the first thing in it that
# can be checked without one. It is a second arch in one test binary, so its
# entry points are renamed for that object alone (see arch.h) - one arch per
# firmware, two of them here.
ARCH_HOST_SRCS += src/arch/at32f435/clk.c src/arch/at32f435/gpio.c \
                  src/arch/at32f435/systick.c src/arch/at32f435/usart.c \
                  src/arch/at32f435/flash.c src/arch/at32f435/spi.c \
                  src/arch/at32f435/i2c.c src/arch/at32f435/adc.c \
                  src/arch/at32f435/output.c src/arch/at32f435/usb.c \
                  src/arch/at32f435/system.c
# And the ESP32's outputs, which are the second target's version of the same
# problem and had never run anywhere: QEMU's ESP32 model has no RMT, so no
# frame completes there and the port's report says so. Here the real file is
# compiled against a stand-in for the two IDF drivers it calls and driven by
# tests/test_arch_esp32_output.c - the same method as the F405's flash and I2C,
# and the one that found that the frames it sent were a quarter of a frame.
ARCH_HOST_SRCS += src/arch/esp32/output.c
# And the second target's two sensor buses, for the same reason and with the
# same stand-in: `spi.c` and `i2c.c` translate one bus protocol into another
# chip's driver calls, and nothing on this machine could run them - a devkit
# carries no sensor and QEMU models neither peripheral. See
# tests/test_arch_esp32_bus.c.
ARCH_HOST_SRCS += src/arch/esp32/spi.c src/arch/esp32/i2c.c \
                  src/arch/esp32/uart.c
# And its pack's converter, which is the last file on that target nothing had
# ever run: QEMU models no ADC, so `adc.c` had only ever been compiled. The
# calibration itself is IDF's and cannot come here - what can is the port's
# side of it, and that is where the unit was wrong (IDF's ADC_UNIT_1 is 0, and
# the board header says 1 for the same converter). See tests/test_arch_esp32_adc.c.
ARCH_HOST_SRCS += src/arch/esp32/adc.c
# And its network, in the configuration a board is in. net.c builds the medium
# at compile time and QEMU's ESP32 has no radio, so every automated run of that
# file has been the *Ethernet* build: the Wi-Fi half - which role the radio is
# put in, the credentials, the WPA2 floor, the reconnect after a dropout - had
# been compiled and never run, here or anywhere else. It is compiled with the
# Kconfig choice spelled the way the generated sdkconfig.h spells it, against
# the same stand-in headers, and driven by tests/test_arch_esp32_net.c. See
# docs/17-esp32-port.md.
ARCH_HOST_SRCS += src/arch/esp32/net.c
ARCH_HOST_OBJS := $(patsubst %.c,$(HOST_OUT)/%.o,$(ARCH_HOST_SRCS))
HOST_OBJS   := $(FLIGHT_OBJS) $(CORE_OBJS) $(TEST_OBJS) $(TOOL_OBJS) \
               $(ARCH_HOST_OBJS)
HOST_DEPS   := $(HOST_OBJS:.o=.d)

# BOARD, PRODUCT and REV are passed as bare tokens and stringified in
# src/core/version.h; only the stamp needs quoting, because it has spaces.
#
# `-ffunction-sections -fdata-sections` are what make the linker's own
# `--gc-sections` (below) mean anything: without them every object is one
# section, so a function nothing calls is kept because the function beside it
# is called. The image was carrying **two and a half kilobytes** of code no
# path reaches - the unused half of several modules, and the tell-tale's own
# pattern arithmetic in the flight build that does not blink - and the ESP32
# port has always had these flags, because ESP-IDF sets them. Measured
# 2026-09-18: 106,392 -> 103,892 bytes of flash on the bare F405 image.
CFLAGS := -std=c11 -O2 -g3 \
          -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 \
          -ffreestanding -fno-common -ffunction-sections -fdata-sections \
          -fstack-usage \
          -Wall -Wextra -Wshadow -Wundef -Wdouble-promotion -Wconversion \
          $(addprefix -I,$(INCLUDES)) \
          -DAK_BOARD=$(BOARD) -DAK_PRODUCT=$(PRODUCT) -DAK_REV=$(REV)

# And the flags a *build variant* needs, which is a define rather than a flag:
#   make EXTRA_CFLAGS=-DAK_BOARD_VBAT_FITTED=1 check
# compiles this board with the part fitted, which is the half of a board file
# that a bare bench never builds - the same shape of hole the ESP32's Wi-Fi
# half fell into. The target builds pass it for the fitted configuration they
# check; nothing here assumes it is empty.
#
# It reaches the image and nothing else - `$(CFLAGS)` here, and `$(HOST_CFLAGS)`
# is not built from it. So this variable is *not* the whole of a build variant:
# the board facts in it have a second home, `$(HOST_FITTED_CFLAGS)` above, which
# is what carries them to the host binary's copy of the board being built. A
# variant that is about the image alone - `-DAK_BOOT_STAGE=1`, whose board half
# spins on the status pin - belongs here and nowhere else.
#
# `$(EXTRA_CFLAGS)` is added *before* the key is taken and not after it, which
# matters more than it looks: it is what distinguishes one board variant from
# another, so leaving it out of the key would reintroduce the exact defect this
# is fixing, for the one input a bench actually varies. The stamp is the only
# thing kept out, and the only reason it is kept out is that it is a clock.
CFLAGS += $(EXTRA_CFLAGS)

# As for `$(HOST_CFLAGS_KEYED)` above, and for the same reason: this is what
# `$(OUT)/.cflags` is written from, and it stops at the revision and the build
# variant. A clock reading in the key makes every ARM image rebuild from
# nothing on every run: measured on the default image, three consecutive `make
# all` runs with nothing changed recompiled 59 objects each with the key as it
# was (trap 188) and 59, 0, 0 with the key as it is now. That measurement is of
# `$(OUT)`; the `profiles` stage is not evidence either way, because it deletes
# each profile's output directory before building it and so has never had an
# incremental build to converge.
CFLAGS_KEYED := $(CFLAGS)
CFLAGS += -DAK_STAMP_VALUE="$(STAMP)"

# -nostdlib on purpose: no libc, no heap, no hidden locks. libgcc is still
# needed for the integer helpers the compiler calls on a Cortex-M4.
LDFLAGS := -T $(LDSCRIPT) -nostdlib -Wl,--gc-sections \
           -Wl,-Map=$(OUT)/$(PRODUCT).map -Wl,--print-memory-usage
LIBS    := -lgcc

all: $(OUT)/$(PRODUCT).bin $(OUT)/$(PRODUCT).hex size

host: $(HOST_OUT)/aerialkit-tests $(HOST_OUT)/aerialkit-replay \
      $(HOST_OUT)/aerialkit-gen-trace $(HOST_OUT)/aerialkit-sim \
      $(HOST_OUT)/aerialkit-fw-sim $(HOST_OUT)/aerialkit-fuzz \
      $(HOST_OUT)/aerialkit-contract-vectors

# The unit tests, then the whole firmware in the loop twenty-four times: as a
# quadrotor, as a fixed wing, on the bench being calibrated, flying missions,
# brought back by a fence and by a flat pack, and through the failures an
# airframe has to survive - a receiver that unplugs or fails on its own, a gyro
# or a barometer that dies, a fix that degrades, lies, or goes away entirely.
# The simulator asserts what it saw - the rate the loop turned at, the radio
# bytes reaching the parser, the aircraft arming and taking the attitude the
# stick asked for, the motors stopping when the link went on the quad, and the
# wing turning round and coming back inside its arrival radius and then handing
# the sticks back, and the mission reaching both of its waypoints and counting
# them, and the fence taking an aircraft off a pilot whose link is up and
# giving it back, and a pack that goes critical under a mission taking the
# aircraft off the mission rather than only off a pilot - and exits non-zero
# when any of that did not happen, which makes it a test rather than a
# transcript. All twenty-four cost well under a second together.
#
# The seconds are the length of each simulated flight. The quad only unplugs
# its receiver at seven seconds and the wing only gets home in the twenties, so
# shortening these finds less rather than running faster.
SIL_SECONDS       ?= 20
SIL_WING_SECONDS  ?= 40
SIL_BENCH_SECONDS ?= 16
SIL_MISSION_SECONDS ?= 75
# The same waypoint list on the quadrotor: a quad flies it at six metres a
# second instead of a wing's ten, over five hundred metres, so the mission is
# longer rather than the list shorter.
SIL_QUADMISSION_SECONDS ?= 130
# And the same list with the pack going critical under it. The aircraft is down
# and disarmed at about 103 s, and the run carries on to the end of the
# mission's own timeline on purpose: the pilot's switch stays on until 114 s -
# sixty-eight seconds after the pack took the mission away - and the last
# `status` is typed at 119 s, which is where the checks read that the aircraft
# finished the flight disarmed. Shortening this does not save time, it removes
# the evidence that the mission did not restart.
SIL_FLATPACK_SECONDS ?= 130
# The fence, and now its lid: out of the ring at nineteen seconds, handed back
# inside it, and then a climb through a ceiling set above the aircraft - which
# is another fourteen seconds of flying, because the pilot has to climb to it
# and the navigator has to bring the aircraft down under it afterwards.
SIL_FENCE_SECONDS ?= 70
SIL_QUADRTH_SECONDS ?= 100
SIL_GPSLOST_SECONDS ?= 70
# The pack goes flat with the link up and the pilot flying: out, home, down,
# and a pack that recovers partway - which must not cancel the return.
SIL_BATTERY_SECONDS ?= 110
# The sticks in rate mode: the other flight law the mode channel selects, with
# a stick on each axis and the mode switch thrown.
SIL_RATE_SECONDS ?= 18
# An aircraft held at 60 degrees of roll with the arm switch thrown, and then
# put down level and armed: the slowness is on purpose, because the estimate
# has to have settled on the angle before the refusal means what it says.
SIL_TILT_SECONDS ?= 20
# A wing thrown by hand, which in this simulator is a switch and a climb rather
# than a throw: the launch holds an attitude and a throttle for four seconds,
# the pilot takes it back with a stick, and it flies on for the rest of the run.
SIL_LAUNCH_SECONDS ?= 14
# And the same launch with the pack going critical *under* it, which is the
# failsafe the launch has to give way to; the return it asks for takes longer
# than the launch itself, so this run is a wing's flight rather than a throw.
SIL_LAUNCH_PACK_SECONDS ?= 45
# The return with the fix degrading to four satellites partway home: the
# navigation pauses, the aircraft drifts with the wind while it holds, and the
# return picks up again when the fix is usable - so it lands, later.
SIL_BADFIX_SECONDS ?= 170
# And three of the sessions again with the sensors as noisy as a real part
# leaves them - the IMU at its datasheet noise, the barometer at half a pascal,
# the fix a metre and a half wide. The wing's return takes longer with a noisy
# fix (33 s against 22), which is why this one has a length of its own.
SIL_WING_NOISY_SECONDS ?= 55
SIL_WING_LOST_SECONDS  ?= 40
# The quadrotor's mission with no wind at all, which is the case that needs the
# navigator to earn a heading before it can fly anywhere: longer, because that
# takes time the wind used to save.
SIL_MISSION_CALM_SECONDS ?= 150
# The same GPS loss with a pilot who has said what to do about it: the wait
# before the descent starts, and twenty-something metres of it at six tenths of
# a metre a second.
SIL_GPSLOSTLAND_SECONDS ?= 130
# And the long ones: a mission that is left on, so the aircraft is still holding
# the station (the quadrotor) or circling it (the wing) five minutes later. The
# drift that matters takes minutes to show and every other session is under
# three.
SIL_HOLD_SECONDS ?= 300
# The board that lies: eight half-second stages, the boot before them and the
# last answer after. The two boot-time variants type one command each.
SIL_BADBOARD_SECONDS ?= 6
SIL_BADBOOT_SECONDS ?= 2
# A GPS module that is on the bus and never speaks: the four configuration
# frames the firmware sends it, and then the rest of a flight without a fix.
SIL_GPSQUIET_SECONDS ?= 10
# The bench sweep and the arm switch: the test starts at 700 ms, the switch is
# thrown at 2 s, and the aircraft arms half a second later.
SIL_TESTARM_SECONDS ?= 6
# An aircraft held in a hand, wobbling, and armed: the gyro bias measurement
# never gets its five hundred still samples, so the firmware abandons it and
# says so. Then it is put down and flown.
SIL_ARM_NOW_SECONDS ?= 10
# The bench's mistakes: four calibrations tried on an armed aircraft, a pack
# that is not a pack, a divider with nothing on it, a face that is not a face,
# a receiver that has never spoken and a gyro calibration somebody interrupts.
# Long enough for the whole bench tape: the four armed refusals, the pack and
# the face, the gyro calibration somebody interrupts, and the failed save at
# 12 s with the honest one after it at 14.2 s.
SIL_REFUSE_SECONDS ?= 17

# --- the control core as a shared library ------------------------------------
#
# Everything above builds *executables*, and the control core only ever gets
# into one by being linked into a program that also has a `main` and, usually,
# a board. That is the wrong shape for a simulation in another language, which
# wants to load the core and call it.
#
# It is a separate set of objects rather than `HOST_OBJS` because a shared
# library needs position-independent code and the executables above are built
# without it: linking the non-PIC objects into a `.so` fails on x86-64 with
# "relocation R_X86_64_32S ... can not be used when making a shared object",
# and adding -fPIC to every host object to fix that would change the code
# generation of the whole test suite to buy one artifact. So the files that go
# into the library are compiled twice, once each way.
#
# Which files: the flight core, its two services, and the glue. `src/core/
# flight/` and `src/core/sensors/` are named for the reason the host build
# exists at all - they include no MCU header and do no I/O - and `ak_control.c`
# is the only file here that is not firmware.
CONTROL_SRCS := $(wildcard src/core/flight/*.c) $(wildcard src/core/sensors/*.c) \
                $(SUPPORT_SRCS) tools/ak_control.c
CONTROL_OBJS := $(patsubst %.c,$(HOST_OUT)/pic/%.o,$(CONTROL_SRCS))
CONTROL_DEPS := $(CONTROL_OBJS:.o=.d)
CONTROL_LIB  := $(HOST_OUT)/libaerialkit-control.so

# `-MMD -MP`, which every other compile rule in this file already had. This one
# went without it for as long as it has existed, and that is trap 183.
#
# These are the objects that go into the shared library a Python process loads
# through ctypes, and most of them include `ak_flight.h`. `tools/ak_control.c`
# is the one that matters: it declares `static ak_flight_t flight;`, so its idea
# of the struct's size is fixed when it is compiled. With no `.d` file, make
# knew only that `pic/tools/ak_control.o` came from `tools/ak_control.c` - which
# is to say, it knew nothing about the header.
#
# So when a header grew, the only object that rebuilt was the one whose own `.c`
# the patch had also edited. On 2026-09-21 the arming gate and its filter added
# `arm_innov_ms` and `arm_accel[3]` to `ak_flight_t` and touched only
# `ak_flight.c` and `ak_flight.h`: `pic/src/core/flight/ak_flight.o` rebuilt
# against the 520-byte struct, `pic/tools/ak_control.o` stayed at 504, and one
# library then held two revisions of one header. `ak_flight_init` wrote past the
# smaller global, and the sanitizer reported it as an overflow in the sources.
#
# Measured, before this line existed: `build-asan` held 179 `.o` and 140 `.d`,
# and every one of the 39 objects with no `.d` beside it was under `pic/`.
# And `$(HOST_OUT)/.cflags`, for the second half of the same story. That stamp is
# written from `$(HOST_CFLAGS)`, which carries `-DAK_REV=$(REV)` - so its contents
# change whenever the revision does, which is what makes a commit visible to make.
# The rule above for the ordinary host objects has depended on it since it was
# added; this one did not, and the reason it matters is that the revision is the
# one flag that changes *without any file changing*: committing the working tree
# moves `REV` from `b29fda7-dirty` to `2f4c636` while every source keeps its
# timestamp, so make relinked nothing and the library went on reporting the
# revision before the commit. `tools/akcontrol_check.py` compares that string
# against the tree's, so the gate turned red on a tree that was correct - trap
# 187, and the second defect traced to this one rule: it is a hand-copy of the
# host rule differing only by `-fPIC`, and a copy diverges from its original
# silently. Trap 183 was this rule missing `-MMD -MP`; this is the same rule
# missing the flag stamp.
#
# Measured before this line existed, with the pic objects up to date:
#
#     $ make REV=deadbee1 build-host/libaerialkit-control.so
#     $ strings build-host/libaerialkit-control.so | grep -E '^[0-9a-f]{7}'
#     d81d211
#
# - zero compile lines, the old revision, and exit 0. After it, the same command
# recompiles the pic objects and the library reads `deadbee1`.
# `-fPIC` is a per-object flag like any other, and it is here rather than in the
# recipe for the same reason the forty-nine lines above are in
# `$(HOST_CFLAGS_TARGET)` and not in `$(HOST_CFLAGS)`: a flag written into a
# recipe is in no key, so editing it would rebuild nothing. A target-specific
# assignment on a pattern rule applies to every target matching the pattern.
$(HOST_OUT)/pic/%.o: HOST_CFLAGS_TARGET += -fPIC
$(HOST_OUT)/pic/%.o: %.c $(HOST_OUT)/.cflags.d/pic/%.o.flags
	@mkdir -p $(@D)
	$(HOST_CC) $(HOST_CFLAGS) $(HOST_CFLAGS_TARGET) -MMD -MP -c $< -o $@

$(CONTROL_LIB): $(CONTROL_OBJS)
	@mkdir -p $(@D)
	$(HOST_CC) -shared -o $@ $(CONTROL_OBJS) -lm

# What the Python process needs in front of it when the library it loads is the
# sanitizing one, and nothing at all when it is not.
#
# `tools/akcontrol_check.py` loads `$(CONTROL_LIB)` with ctypes. An interpreter
# that has not itself been built with AddressSanitizer cannot load a shared
# object that has: the load fails with "ASan runtime does not come first in
# initial library list; you should either link runtime to your application or
# manually preload it with LD_PRELOAD", and the check exits 1 without running.
# That is what the sanitizer's own message asks for, so that is what this does.
#
# Preloading the runtime has a second consequence, and it is the reason for the
# suppression file rather than a bare LD_PRELOAD: LeakSanitizer then watches
# CPython too, and CPython does not free everything it allocates before it
# exits. On this machine that is 150 allocations in 149,106 bytes, every one of
# them in interpreter frames and none with a `libaerialkit-control` frame. They
# are suppressed by pattern in `tools/lsan.supp` rather than by setting
# `detect_leaks=0`, because a leak that originates in the control library must
# have a library frame at the top of its stack, so a pattern naming a CPython
# symbol cannot suppress one - while `detect_leaks=0` would suppress all of it.
#
# The guard is the host compiler, and it has to be: `make test` on an ordinary
# build has no instrumented library and must not preload a runtime into a Python
# that has no use for it. `-fsanitize=address` in `$(HOST_CC)` is exactly the
# fact that decides it, and the sanitize target passes it there.
#
# Nothing in this repository needed any of it before 2026-09-20, because the
# sanitize stage aborted at check 230 of 2118 and never reached this line. See
# finding 1 in docs/28-build.md.
# `ASAN_OPTIONS=protect_shadow_gap=0` used to be set here as well, by the same
# reflex that put it on the whole of `sanitize`. It was measured out on
# 2026-09-24 and the check does not need it: this runs the control library
# through ctypes, and the library does not map the ARM System Control Space -
# `0xE000E000` belongs to `tests/test_arch.c`'s model, which is a different
# program. Run both ways, the two logs are byte-identical, 98 lines each and
# `akcontrol_check: 71 checks, 0 failed` in both, so the option was buying
# nothing and costing the gap protection on 71 checks. The two settings that
# *are* needed stay: preloading the runtime, without which an uninstrumented
# interpreter cannot load an instrumented shared object at all, and the
# suppression file, without which LeakSanitizer reports CPython's own exit
# allocations as if they were the library's.
CONTROL_LIB_ENV :=
ifneq (,$(findstring -fsanitize=address,$(HOST_CC)))
CONTROL_LIB_ENV := LD_PRELOAD=$(shell $(firstword $(HOST_CC)) -print-file-name=libasan.so) \
                   LSAN_OPTIONS=suppressions=$(CURDIR)/tools/lsan.supp
endif

# The arguments the tests binary is run with. Empty by default, so `make test`
# runs all 2269 checks and nothing above changes. It exists for one caller:
# `sanitize`, which has to run the six register-model tests apart from the rest
# because they map the ARM System Control Space at `0xE000E000` and that address
# is inside AddressSanitizer's shadow gap - so they are the only checks that
# need `protect_shadow_gap=0`, and everything else should be run with the gap
# protected rather than inheriting a concession made for six tests out of 2269.
# The two halves are the whole of the binary's checks: 885 with `--only-registers`, 1384
# with `--no-registers`, 2269 with neither, zero failed in each.
TEST_ARGS ?=

test: host $(CONTROL_LIB) oracle-link
	@$(HOST_OUT)/aerialkit-tests $(TEST_ARGS)
	@$(HOST_OUT)/aerialkit-fw-sim $(SIL_SECONDS) quad \
	    > $(HOST_OUT)/sil-quad.log 2>&1 && \
	    grep '^sim: \(ok\|FAIL\|PASS\)' $(HOST_OUT)/sil-quad.log || \
	    { printf '\nthe quadrotor, in the loop, FAILED:\n\n'; \
	      cat $(HOST_OUT)/sil-quad.log; exit 1; }
	@$(HOST_OUT)/aerialkit-fw-sim $(SIL_WING_SECONDS) wing \
	    > $(HOST_OUT)/sil-wing.log 2>&1 && \
	    grep '^sim: \(ok\|FAIL\|PASS\)' $(HOST_OUT)/sil-wing.log || \
	    { printf '\nthe fixed wing, in the loop, FAILED:\n\n'; \
	      cat $(HOST_OUT)/sil-wing.log; exit 1; }
	@# And the owner's rule for the one case the wing's return cannot cover:
	@# nobody is flying it and there is no navigator, because RTH is off - the
	@# configuration a board ships with. It circles down instead of stopping.
	@$(HOST_OUT)/aerialkit-fw-sim $(SIL_WING_LOST_SECONDS) winglost \
	    > $(HOST_OUT)/sil-wing-lost.log 2>&1 && \
	    grep '^sim: \(ok\|FAIL\|PASS\)' $(HOST_OUT)/sil-wing-lost.log || \
	    { printf '\nthe lost wing, in the loop, FAILED:\n\n'; \
	      cat $(HOST_OUT)/sil-wing-lost.log; exit 1; }
	@$(HOST_OUT)/aerialkit-fw-sim $(SIL_BENCH_SECONDS) bench \
	    > $(HOST_OUT)/sil-bench.log 2>&1 && \
	    grep '^sim: \(ok\|FAIL\|PASS\)' $(HOST_OUT)/sil-bench.log || \
	    { printf '\nthe bench session, in the loop, FAILED:\n\n'; \
	      cat $(HOST_OUT)/sil-bench.log; exit 1; }
	@$(HOST_OUT)/aerialkit-fw-sim $(SIL_MISSION_SECONDS) mission \
	    > $(HOST_OUT)/sil-mission.log 2>&1 && \
	    grep '^sim: \(ok\|FAIL\|PASS\)' $(HOST_OUT)/sil-mission.log || \
	    { printf '\nthe mission, in the loop, FAILED:\n\n'; \
	      cat $(HOST_OUT)/sil-mission.log; exit 1; }
	@$(HOST_OUT)/aerialkit-fw-sim $(SIL_FENCE_SECONDS) fence \
	    > $(HOST_OUT)/sil-fence.log 2>&1 && \
	    grep '^sim: \(ok\|FAIL\|PASS\)' $(HOST_OUT)/sil-fence.log || \
	    { printf '\nthe fence, in the loop, FAILED:\n\n'; \
	      cat $(HOST_OUT)/sil-fence.log; exit 1; }
	@$(HOST_OUT)/aerialkit-fw-sim $(SIL_QUADRTH_SECONDS) quadrth \
	    > $(HOST_OUT)/sil-quadrth.log 2>&1 && \
	    grep '^sim: \(ok\|FAIL\|PASS\)' $(HOST_OUT)/sil-quadrth.log || \
	    { printf '\nthe quadrotor return, in the loop, FAILED:\n\n'; \
	      cat $(HOST_OUT)/sil-quadrth.log; exit 1; }
	@$(HOST_OUT)/aerialkit-fw-sim $(SIL_GPSLOST_SECONDS) gpslost \
	    > $(HOST_OUT)/sil-gpslost.log 2>&1 && \
	    grep '^sim: \(ok\|FAIL\|PASS\)' $(HOST_OUT)/sil-gpslost.log || \
	    { printf '\nthe gps-loss session, in the loop, FAILED:\n\n'; \
	      cat $(HOST_OUT)/sil-gpslost.log; exit 1; }
	@$(HOST_OUT)/aerialkit-fw-sim $(SIL_GPSLOSTLAND_SECONDS) gpslostland \
	    > $(HOST_OUT)/sil-gpslost-land.log 2>&1 && \
	    grep '^sim: \(ok\|FAIL\|PASS\)' $(HOST_OUT)/sil-gpslost-land.log || \
	    { printf '\nthe gps-loss landing, in the loop, FAILED:\n\n'; \
	      cat $(HOST_OUT)/sil-gpslost-land.log; exit 1; }
	@$(HOST_OUT)/aerialkit-fw-sim $(SIL_GPSLOSTLAND_SECONDS) gpslostland norange \
	    > $(HOST_OUT)/sil-gpslost-land-norange.log 2>&1 && \
	    grep '^sim: \(ok\|FAIL\|PASS\)' $(HOST_OUT)/sil-gpslost-land-norange.log || \
	    { printf '\nthe same pilot setting with no rangefinder, in the loop, FAILED:\n\n'; \
	      cat $(HOST_OUT)/sil-gpslost-land-norange.log; exit 1; }
	@$(HOST_OUT)/aerialkit-fw-sim $(SIL_QUADRTH_SECONDS) quadrth norange \
	    > $(HOST_OUT)/sil-quadrth-norange.log 2>&1 && \
	    grep '^sim: \(ok\|FAIL\|PASS\)' $(HOST_OUT)/sil-quadrth-norange.log || \
	    { printf '\nthe return with no rangefinder, in the loop, FAILED:\n\n'; \
	      cat $(HOST_OUT)/sil-quadrth-norange.log; exit 1; }
	@# The part the landing rule prefers, failing *while* it is being used and
	@# coming back before the aircraft is down. This session existed as a
	@# scenario word for months and had never been run by the suite or checked;
	@# the first run found a real bug (see tools/fw_sim.c and the header's
	@# ak_range_result_t).
	@$(HOST_OUT)/aerialkit-fw-sim $(SIL_QUADRTH_SECONDS) quadrth rangefail \
	    > $(HOST_OUT)/sil-quadrth-rangefail.log 2>&1 && \
	    grep '^sim: \(ok\|FAIL\|PASS\)' $(HOST_OUT)/sil-quadrth-rangefail.log || \
	    { printf '\nthe return with a rangefinder that went quiet, FAILED:\n\n'; \
	      cat $(HOST_OUT)/sil-quadrth-rangefail.log; exit 1; }
	@# The board a devkit out of the bag is: no barometer, no rangefinder, no
	@# pack divider - every ESP32 this project has - flown, to check the claim
	@# that such a board still flies and says what it is missing.
	@$(HOST_OUT)/aerialkit-fw-sim $(SIL_QUADRTH_SECONDS) quadrth bare \
	    > $(HOST_OUT)/sil-quadrth-bare.log 2>&1 && \
	    grep '^sim: \(ok\|FAIL\|PASS\)' $(HOST_OUT)/sil-quadrth-bare.log || \
	    { printf '\nthe return on a bare board, FAILED:\n\n'; \
	      cat $(HOST_OUT)/sil-quadrth-bare.log; exit 1; }
	@$(HOST_OUT)/aerialkit-fw-sim $(SIL_QUADMISSION_SECONDS) mission quad \
	    > $(HOST_OUT)/sil-quadmission.log 2>&1 && \
	    grep '^sim: \(ok\|FAIL\|PASS\)' $(HOST_OUT)/sil-quadmission.log || \
	    { printf '\nthe quadrotor mission, in the loop, FAILED:\n\n'; \
	      cat $(HOST_OUT)/sil-quadmission.log; exit 1; }
	@$(HOST_OUT)/aerialkit-fw-sim $(SIL_MISSION_CALM_SECONDS) mission quad calm \
	    > $(HOST_OUT)/sil-quadmission-calm.log 2>&1 && \
	    grep '^sim: \(ok\|FAIL\|PASS\)' $(HOST_OUT)/sil-quadmission-calm.log || \
	    { printf '\nthe quadrotor mission in still air, in the loop, FAILED:\n\n'; \
	      cat $(HOST_OUT)/sil-quadmission-calm.log; exit 1; }
	@$(HOST_OUT)/aerialkit-fw-sim $(SIL_FLATPACK_SECONDS) mission quad flatpack \
	    > $(HOST_OUT)/sil-flatpack.log 2>&1 && \
	    grep '^sim: \(ok\|FAIL\|PASS\)' $(HOST_OUT)/sil-flatpack.log || \
	    { printf '\nthe mission with a pack that went critical, in the loop, FAILED:\n\n'; \
	      cat $(HOST_OUT)/sil-flatpack.log; exit 1; }
	@# The mission the *pilot* ends, with a stick, while the switch stays on:
	@# the one state main.c has a branch of its own for.
	@$(HOST_OUT)/aerialkit-fw-sim $(SIL_QUADMISSION_SECONDS) mission quad takeback \
	    > $(HOST_OUT)/sil-takeback.log 2>&1 && \
	    grep '^sim: \(ok\|FAIL\|PASS\)' $(HOST_OUT)/sil-takeback.log || \
	    { printf '\nthe mission taken back by the pilot, in the loop, FAILED:\n\n'; \
	      cat $(HOST_OUT)/sil-takeback.log; exit 1; }
	@$(HOST_OUT)/aerialkit-fw-sim $(SIL_BATTERY_SECONDS) battery \
	    > $(HOST_OUT)/sil-battery.log 2>&1 && \
	    grep '^sim: \(ok\|FAIL\|PASS\)' $(HOST_OUT)/sil-battery.log || \
	    { printf '\nthe battery session, in the loop, FAILED:\n\n'; \
	      cat $(HOST_OUT)/sil-battery.log; exit 1; }
	@$(HOST_OUT)/aerialkit-fw-sim $(SIL_RATE_SECONDS) rate \
	    > $(HOST_OUT)/sil-rate.log 2>&1 && \
	    grep '^sim: \(ok\|FAIL\|PASS\)' $(HOST_OUT)/sil-rate.log || \
	    { printf '\nthe rate-mode session, in the loop, FAILED:\n\n'; \
	      cat $(HOST_OUT)/sil-rate.log; exit 1; }
	@$(HOST_OUT)/aerialkit-fw-sim $(SIL_TILT_SECONDS) tilt \
	    > $(HOST_OUT)/sil-tilt.log 2>&1 && \
	    grep '^sim: \(ok\|FAIL\|PASS\)' $(HOST_OUT)/sil-tilt.log || \
	    { printf '\nthe tilted-arm session, in the loop, FAILED:\n\n'; \
	      cat $(HOST_OUT)/sil-tilt.log; exit 1; }
	@$(HOST_OUT)/aerialkit-fw-sim $(SIL_LAUNCH_SECONDS) launch \
	    > $(HOST_OUT)/sil-launch.log 2>&1 && \
	    grep '^sim: \(ok\|FAIL\|PASS\)' $(HOST_OUT)/sil-launch.log || \
	    { printf '\nthe hand launch, in the loop, FAILED:\n\n'; \
	      cat $(HOST_OUT)/sil-launch.log; exit 1; }
	@# And the launch that meets a failsafe: the pack goes critical while the
	@# launch is flying the aircraft, so the *reason* has to win over the mode.
	@$(HOST_OUT)/aerialkit-fw-sim $(SIL_LAUNCH_PACK_SECONDS) launch pack \
	    > $(HOST_OUT)/sil-launch-pack.log 2>&1 && \
	    grep '^sim: \(ok\|FAIL\|PASS\)' $(HOST_OUT)/sil-launch-pack.log || \
	    { printf '\nthe launch with a pack that went critical, FAILED:\n\n'; \
	      cat $(HOST_OUT)/sil-launch-pack.log; exit 1; }
	@$(HOST_OUT)/aerialkit-fw-sim $(SIL_QUADRTH_SECONDS) quadrth nobaro \
	    > $(HOST_OUT)/sil-quadrth-nobaro.log 2>&1 && \
	    grep '^sim: \(ok\|FAIL\|PASS\)' $(HOST_OUT)/sil-quadrth-nobaro.log || \
	    { printf '\nthe return with a dead barometer, in the loop, FAILED:\n\n'; \
	      cat $(HOST_OUT)/sil-quadrth-nobaro.log; exit 1; }
	@$(HOST_OUT)/aerialkit-fw-sim $(SIL_SECONDS) noimu \
	    > $(HOST_OUT)/sil-no-imu.log 2>&1 && \
	    grep '^sim: \(ok\|FAIL\|PASS\)' $(HOST_OUT)/sil-no-imu.log || \
	    { printf '\nthe gyro failing in flight, in the loop, FAILED:\n\n'; \
	      cat $(HOST_OUT)/sil-no-imu.log; exit 1; }
	@$(HOST_OUT)/aerialkit-fw-sim $(SIL_BADFIX_SECONDS) quadrth badfix \
	    > $(HOST_OUT)/sil-quadrth-badfix.log 2>&1 && \
	    grep '^sim: \(ok\|FAIL\|PASS\)' $(HOST_OUT)/sil-quadrth-badfix.log || \
	    { printf '\nthe return with a degraded fix, in the loop, FAILED:\n\n'; \
	      cat $(HOST_OUT)/sil-quadrth-badfix.log; exit 1; }
	@$(HOST_OUT)/aerialkit-fw-sim $(SIL_QUADRTH_SECONDS) quadrth gpsjump \
	    > $(HOST_OUT)/sil-quadrth-jump.log 2>&1 && \
	    grep '^sim: \(ok\|FAIL\|PASS\)' $(HOST_OUT)/sil-quadrth-jump.log || \
	    { printf '\nthe return with a lying fix, in the loop, FAILED:\n\n'; \
	      cat $(HOST_OUT)/sil-quadrth-jump.log; exit 1; }
	@$(HOST_OUT)/aerialkit-fw-sim $(SIL_QUADRTH_SECONDS) quadrth gyrobias \
	    > $(HOST_OUT)/sil-quadrth-gyrobias.log 2>&1 && \
	    grep '^sim: \(ok\|FAIL\|PASS\)' $(HOST_OUT)/sil-quadrth-gyrobias.log || \
	    { printf '\nthe return with a warmed-up gyro, in the loop, FAILED:\n\n'; \
	      cat $(HOST_OUT)/sil-quadrth-gyrobias.log; exit 1; }
	@$(HOST_OUT)/aerialkit-fw-sim $(SIL_QUADRTH_SECONDS) quadrth noisy \
	    > $(HOST_OUT)/sil-quadrth-noisy.log 2>&1 && \
	    grep '^sim: \(ok\|FAIL\|PASS\)' $(HOST_OUT)/sil-quadrth-noisy.log || \
	    { printf '\nthe return with noisy sensors, in the loop, FAILED:\n\n'; \
	      cat $(HOST_OUT)/sil-quadrth-noisy.log; exit 1; }
	@$(HOST_OUT)/aerialkit-fw-sim $(SIL_WING_NOISY_SECONDS) wing noisy \
	    > $(HOST_OUT)/sil-wing-noisy.log 2>&1 && \
	    grep '^sim: \(ok\|FAIL\|PASS\)' $(HOST_OUT)/sil-wing-noisy.log || \
	    { printf '\nthe fixed wing with noisy sensors, in the loop, FAILED:\n\n'; \
	      cat $(HOST_OUT)/sil-wing-noisy.log; exit 1; }
	@$(HOST_OUT)/aerialkit-fw-sim $(SIL_SECONDS) quad noisy \
	    > $(HOST_OUT)/sil-quad-noisy.log 2>&1 && \
	    grep '^sim: \(ok\|FAIL\|PASS\)' $(HOST_OUT)/sil-quad-noisy.log || \
	    { printf '\nthe sticks with noisy sensors, in the loop, FAILED:\n\n'; \
	      cat $(HOST_OUT)/sil-quad-noisy.log; exit 1; }
	@$(HOST_OUT)/aerialkit-fw-sim $(SIL_SECONDS) quad rxfailsafe \
	    > $(HOST_OUT)/sil-rx-failsafe.log 2>&1 && \
	    grep '^sim: \(ok\|FAIL\|PASS\)' $(HOST_OUT)/sil-rx-failsafe.log || \
	    { printf '\nthe receiver in its own failsafe, in the loop, FAILED:\n\n'; \
	      cat $(HOST_OUT)/sil-rx-failsafe.log; exit 1; }
	@# Not a flight: a board that stopped, and the boot that reads the record
	@# it left behind. The three lines it prints are the ones somebody at a
	@# bench copies into addr2line, so the format is a check here.
	@$(HOST_OUT)/aerialkit-fw-sim 3 fault \
	    > $(HOST_OUT)/sil-fault.log 2>&1 && \
	    grep '^sim: \(ok\|FAIL\|PASS\)' $(HOST_OUT)/sil-fault.log || \
	    { printf '\nthe boot after a crash, FAILED:\n\n'; \
	      cat $(HOST_OUT)/sil-fault.log; exit 1; }
	@# And every parser that eats somebody else's bytes, under a fixed seed's
	@# worth of noise: see tools/fuzz_parsers.c. This is the run whose value
	@# goes up under `make sanitize`, where an out-of-bounds index in a parser
	@# is visible instead of merely lucky.
	@$(MAKE) --no-print-directory fuzz
	@# The two long sessions: five minutes of holding, which is where a slow
	@# drift in a position or altitude loop shows and the short sessions cannot
	@# see it.
	@$(HOST_OUT)/aerialkit-fw-sim $(SIL_HOLD_SECONDS) mission quad hold \
	    > $(HOST_OUT)/sil-quad-hold.log 2>&1 && \
	    grep '^sim: \(ok\|FAIL\|PASS\)' $(HOST_OUT)/sil-quad-hold.log || \
	    { printf '\nthe quadrotor holding station, in the loop, FAILED:\n\n'; \
	      cat $(HOST_OUT)/sil-quad-hold.log; exit 1; }
	@$(HOST_OUT)/aerialkit-fw-sim $(SIL_HOLD_SECONDS) mission hold \
	    > $(HOST_OUT)/sil-wing-hold.log 2>&1 && \
	    grep '^sim: \(ok\|FAIL\|PASS\)' $(HOST_OUT)/sil-wing-hold.log || \
	    { printf '\nthe wing circling, in the loop, FAILED:\n\n'; \
	      cat $(HOST_OUT)/sil-wing-hold.log; exit 1; }
	@# The preflight's failure lines, and the board is the one that is wrong:
	@# six wrong facts one at a time, with a `preflight` after each - and the
	@# two that are decided at boot rather than by a running command.
	@$(HOST_OUT)/aerialkit-fw-sim $(SIL_BADBOARD_SECONDS) badboard \
	    > $(HOST_OUT)/sil-badboard.log 2>&1 && \
	    grep '^sim: \(ok\|FAIL\|PASS\)' $(HOST_OUT)/sil-badboard.log || \
	    { printf '\nthe board that lies, in the loop, FAILED:\n\n'; \
	      cat $(HOST_OUT)/sil-badboard.log; exit 1; }
	@$(HOST_OUT)/aerialkit-fw-sim $(SIL_BADBOOT_SECONDS) badboard noout \
	    > $(HOST_OUT)/sil-badboard-noout.log 2>&1 && \
	    grep '^sim: \(ok\|FAIL\|PASS\)' $(HOST_OUT)/sil-badboard-noout.log || \
	    { printf '\nthe board with no outputs, in the loop, FAILED:\n\n'; \
	      cat $(HOST_OUT)/sil-badboard-noout.log; exit 1; }
	@$(HOST_OUT)/aerialkit-fw-sim $(SIL_BADBOOT_SECONDS) badboard twomotor \
	    > $(HOST_OUT)/sil-badboard-twomotor.log 2>&1 && \
	    grep '^sim: \(ok\|FAIL\|PASS\)' $(HOST_OUT)/sil-badboard-twomotor.log || \
	    { printf '\nthe two-motor board, in the loop, FAILED:\n\n'; \
	      cat $(HOST_OUT)/sil-badboard-twomotor.log; exit 1; }
	@# A module that is there and not talking this parser's language - the state
	@# a person is in when the GPS is wired, powered, and still speaking NMEA.
	@$(HOST_OUT)/aerialkit-fw-sim $(SIL_GPSQUIET_SECONDS) gpsquiet \
	    > $(HOST_OUT)/sil-gps-quiet.log 2>&1 && \
	    grep '^sim: \(ok\|FAIL\|PASS\)' $(HOST_OUT)/sil-gps-quiet.log || \
	    { printf '\nthe quiet gps, in the loop, FAILED:\n\n'; \
	      cat $(HOST_OUT)/sil-gps-quiet.log; exit 1; }
	@# The output test meeting the arm switch: the safety interlock that keeps
	@# a bench sweep from sharing the outputs with the pilot who just took them.
	@$(HOST_OUT)/aerialkit-fw-sim $(SIL_TESTARM_SECONDS) testarm \
	    > $(HOST_OUT)/sil-testarm.log 2>&1 && \
	    grep '^sim: \(ok\|FAIL\|PASS\)' $(HOST_OUT)/sil-testarm.log || \
	    { printf '\nthe output test and the arm switch, FAILED:\n\n'; \
	      cat $(HOST_OUT)/sil-testarm.log; exit 1; }
	@# An aircraft somebody was holding when it was armed: the bias the
	@# firmware measures for itself never gets its still samples, so it says
	@# so - once - and flies on the stored one.
	@$(HOST_OUT)/aerialkit-fw-sim $(SIL_ARM_NOW_SECONDS) armnow \
	    > $(HOST_OUT)/sil-armnow.log 2>&1 && \
	    grep '^sim: \(ok\|FAIL\|PASS\)' $(HOST_OUT)/sil-armnow.log || \
	    { printf '\narmed while it was being held, FAILED:\n\n'; \
	      cat $(HOST_OUT)/sil-armnow.log; exit 1; }
	@# The bench's mistakes, which is where the calibrations' sentences live -
	@# including the guard `calibrate accel` was missing.
	@$(HOST_OUT)/aerialkit-fw-sim $(SIL_REFUSE_SECONDS) refuse \
	    > $(HOST_OUT)/sil-refuse.log 2>&1 && \
	    grep '^sim: \(ok\|FAIL\|PASS\)' $(HOST_OUT)/sil-refuse.log || \
	    { printf '\nthe calibrations refused, FAILED:\n\n'; \
	      cat $(HOST_OUT)/sil-refuse.log; exit 1; }
	@$(HOST_OUT)/aerialkit-fw-sim $(SIL_REFUSE_SECONDS) refuse rcquiet \
	    > $(HOST_OUT)/sil-refuse-rcquiet.log 2>&1 && \
	    grep '^sim: \(ok\|FAIL\|PASS\)' $(HOST_OUT)/sil-refuse-rcquiet.log || \
	    { printf '\nthe calibrations with no receiver, FAILED:\n\n'; \
	      cat $(HOST_OUT)/sil-refuse-rcquiet.log; exit 1; }
	@python3 tools/bench_check.py --sim \
	    --sim-binary $(HOST_OUT)/aerialkit-fw-sim \
	    --out $(HOST_OUT)/bench-check.log > $(HOST_OUT)/bench-check.out 2>&1 || \
	    { printf '\nthe bring-up checklist, typed at the console, FAILED:\n\n'; \
	      cat $(HOST_OUT)/bench-check.out; exit 1; }
	@grep -E '^  (ok|\.\.)' $(HOST_OUT)/bench-check.out \
	    | sed 's/^/console: /'
	@# And the same tool through the *serial* path it is used on at a bench.
	@# `--sim` above reaches the console through a pipe; `--port` is the one
	@# a person uses, and on 2026-09-18 it had never been executed anywhere.
	@# The board for this is a pty pair with the simulator on the far end -
	@# see tools/bench_port_check.py, which also carries the case that
	@# mattered: a board that had already booted when the port was opened.
	@python3 tools/bench_port_check.py --out $(HOST_OUT) \
	    > $(HOST_OUT)/bench-port.out 2>&1 || \
	    { printf '\nthe bring-up checklist, through a serial port, FAILED:\n\n'; \
	      cat $(HOST_OUT)/bench-port.out; exit 1; }
	@sed 's/^/port: /' $(HOST_OUT)/bench-port.out
	@# And the read that frames every command above, against a console that
	@# prints a prompt nobody asked for. On 2026-09-28 the table check was
	@# reported as flaking - "no table came back" while all 92 rows arrived
	@# when read directly - and the cause was this file's own framing: the
	@# read ended on the first prompt it saw, so a prompt left on the wire by
	@# the command before ended the next read on its first byte. Both halves
	@# of the cure are falsified by the two cases here; see
	@# tools/bench_read_check.py.
	@python3 tools/bench_read_check.py \
	    > $(HOST_OUT)/bench-read.out 2>&1 || \
	    { printf '\nthe bench tool\x27s read, FAILED:\n\n'; \
	      cat $(HOST_OUT)/bench-read.out; exit 1; }
	@sed 's/^/read: /' $(HOST_OUT)/bench-read.out
	@# And the protocol client's own command lines, which the pages print and
	@# nothing ran: `akproto.py --port ... hello`, `list`, `get`, `log out.csv`,
	@# `--source flash log crash.csv` and `--host HOST:PORT status` - the last
	@# one being the transport a board on a network answers on, and the one the
	@# first-flight page ends with (pulling the log that survives the battery).
	@python3 tools/akproto_cli_check.py --out $(HOST_OUT) \
	    > $(HOST_OUT)/akproto-cli.out 2>&1 || \
	    { printf '\nthe protocol client’s command line, FAILED:\n\n'; \
	      cat $(HOST_OUT)/akproto-cli.out; exit 1; }
	@sed 's/^/client: /' $(HOST_OUT)/akproto-cli.out
	@# And the numbers the pages say came from the references: INAV's launch
	@# throttle, climb angle and timeout, and the arming gate's 25 degrees -
	@# read out of INAV's own settings table, this board's own dump in
	@# projects/twin-wings/, and Betaflight's header, then compared with the
	@# firmware's parameter table. A claim about somebody else's firmware is
	@# the easiest kind to get wrong quietly.
	@AK_SIM=$(HOST_OUT)/aerialkit-fw-sim python3 tools/reference_defaults_check.py \
	    > $(HOST_OUT)/reference-defaults.out 2>&1 || \
	    { printf '\nthe defaults we borrow, FAILED:\n\n'; \
	      cat $(HOST_OUT)/reference-defaults.out; exit 1; }
	@sed 's/^/borrowed: /' $(HOST_OUT)/reference-defaults.out
	@# And the other direction, which is the one that goes stale: every command
	@# the firmware actually has, against the pages that describe it.
	@python3 tools/docs_check.py $(HOST_OUT)/aerialkit-fw-sim \
	    > $(HOST_OUT)/docs-check.out 2>&1 || \
	    { printf '\nthe commands against the documentation, FAILED:\n\n'; \
	      cat $(HOST_OUT)/docs-check.out; exit 1; }
	@sed 's/^/docs: /' $(HOST_OUT)/docs-check.out
	@python3 tools/docs_check.py --self-test > $(HOST_OUT)/docs-check-self.out 2>&1 || \
	    { printf '\nthe documentation check, against a console it should fail:\n\n'; \
	      cat $(HOST_OUT)/docs-check-self.out; exit 1; }
	@sed 's/^/docs: /' $(HOST_OUT)/docs-check-self.out | tail -2
	@# And every name a registration spells, against the buffer a load reads it
	@# back through. It is a source check rather than a test because the test
	@# walks the table the *host* binary builds - thirty-three flight parameters
	@# and the Wi-Fi ones - while the board's table is built in ak_firmware_main,
	@# and the names that were silently unrecoverable (`quad_return_speed`,
	@# `servo*_travel_us`, `quad_hold_land_s`, `launch_climb_deg`,
	@# `launch_timeout_s`) are exactly the ones no host test can reach.
	@# `--self-test` first, because a checker that cannot be shown to fail is
	@# indistinguishable from one that returns zero.
	@python3 tools/param_name_check.py --self-test
	@# `--board` is the half that makes the source read a measurement rather
	@# than a reading: it asks the built firmware which names it has and
	@# requires every one of them to have been in the source set. Without it
	@# the check passes just as well on a registration it silently failed to
	@# parse - and the first version of this file did exactly that, finding
	@# one name out of three in `bias_names` and reporting a clean tree.
	@python3 tools/param_name_check.py --board $(HOST_OUT)/aerialkit-fw-sim \
	    > $(HOST_OUT)/param-name.out 2>&1 || \
	    { printf '\nthe parameter names a load has to match, FAILED:\n\n'; \
	      cat $(HOST_OUT)/param-name.out; exit 1; }
	@sed 's/^/params: /' $(HOST_OUT)/param-name.out
	@# And the suite's own scope, which is the other half of the question above.
	@# The suite is declared once, in `scripts/ci.sh`; `make coverage` and
	@# `make sanitize` each called themselves the suite while reaching five of
	@# its ten stages, and neither said so, because in both the claim was prose
	@# in a recipe and prose does not fail. This one is a check, so a sixth
	@# oracle cannot be added to the gate and left out of the map.
	@python3 tools/suite_scope_check.py --self-test
	@python3 tools/suite_scope_check.py
	@# And every command the firmware has, typed at it once: the pages say each
	@# one exists (`docs_check.py`, above), and this says each one answers. It is
	@# the bench's question - a console that stops talking - and no session
	@# happens to type all of them.
	@python3 tools/command_smoke.py $(HOST_OUT)/aerialkit-fw-sim
	@# And the image report's arithmetic, which needs no cross toolchain: the
	@# fixtures are a real link's map and a real link's program headers, and the
	@# numbers they are held to are the ones that link's own
	@# --print-memory-usage printed. It belongs here rather than in a target
	@# somebody has to remember, because the day it is wrong is the day every
	@# per-profile figure the project quotes is wrong.
	@python3 tools/image_report_check.py
	@# And the board manifests against each other, which is the one claim no
	@# build in this repository makes: every board header is a set of
	@# assertions about hardware, and until this ran nothing read them
	@# together - `src/boards/ESP32S2DEV/board.h` gives the status LED and the
	@# first servo one pad, in the configuration a bare devkit builds, and
	@# every test, image and profile check in the tree passed anyway.
	@python3 tools/board_resources_check.py
	@# And the cross-repository contract, which is the one set of numbers two
	@# trees have to agree about: the attitude units, the telemetry frame and
	@# the blackbox record this firmware writes, and the NED->ENU rotation the
	@# other repository writes. It reads a committed file of vectors produced
	@# by both sides and recomputes every byte, so it needs neither a
	@# cross toolchain nor the sibling checkout - and a change to an encoder
	@# that the vectors no longer describe fails here rather than at the bench.
	@python3 tools/contract_check.py
	@# And the control core through the ABI a Python simulation drives it by,
	@# which is the one seam that crosses a language boundary and therefore the
	@# one that fails without a compiler error: the struct layouts, the age and
	@# TTL rules C5 and C6 put at the boundary, the receiver round trip through
	@# the firmware's own decoder, and two runs of one experiment compared to
	@# the last bit. It needs the shared library and python3's ctypes and
	@# nothing else - not numpy, not the sibling checkout, not a board.
	@$(CONTROL_LIB_ENV) AK_CONTROL_LIB=$(CONTROL_LIB) AK_REV='$(REV)' python3 tools/akcontrol_check.py
	@# And the records, which are the only files here that are *ported* between
	@# trees rather than edited in one: the ledger and hosts/nas/agents/ get
	@# copied, merged and carried by batches, and each time that has happened the
	@# failure was the same shape - the copy was shorter and nothing noticed.
	@# The gate written for the last port asserted five things and all five
	@# passed on a copy 2,353 lines short, because every one of them asked
	@# whether a specific thing was *absent*. This asks the other question:
	@# every section HEAD has must still be here. A loss that has already been
	@# committed is HEAD itself and this cannot see it; that is what --base-dir
	@# is for, and tools/records_check.py says so in its own docstring rather
	@# than leaving it to be discovered. See traps 152 and 153.
	@python3 tools/records_check.py > $(HOST_OUT)/records-check.out 2>&1 || \
	    { printf '\nthe records, a section short of what they were:\n\n'; \
	      cat $(HOST_OUT)/records-check.out; exit 1; }
	@sed 's/^/records: /' $(HOST_OUT)/records-check.out
	@python3 tools/records_check.py --self-test > $(HOST_OUT)/records-self.out 2>&1 || \
	    { printf '\nthe records check, against a file it should fail:\n\n'; \
	      cat $(HOST_OUT)/records-self.out; exit 1; }
	@sed 's/^/records: /' $(HOST_OUT)/records-self.out

# The same, on its own, for when that is the thing being changed.
command-check: host
	@python3 tools/command_smoke.py $(HOST_OUT)/aerialkit-fw-sim

# The bring-up checklist, typed at a console. Against the simulator it checks
# the runner and the console; against a board it is the bench evidence
# (docs/05-bringup.md), and PORT is the only thing that changes:
#
#   make bench-check                       # the simulator
#   make bench-check PORT=/dev/ttyACM0     # the F405, once there is one
BENCH_ARGS ?=
PORT ?=

bench-check: host
	@if [ -n "$(PORT)" ]; then \
	    python3 tools/bench_check.py --port "$(PORT)" $(BENCH_ARGS); \
	else \
	    python3 tools/bench_check.py --sim \
	        --sim-binary $(HOST_OUT)/aerialkit-fw-sim $(BENCH_ARGS); \
	fi

# console.c is here for the same reason the arch files are: test_arch.c drives
# the real console formatter into the real UART driver. Nothing else in the
# tests links it, so a two-hundred-byte formatter stays out of their way.
#
# And *three* board files, which is the third target's doing and then the
# fourth board's: test_arch_at32.c drives the wing's board the way test_arch.c
# drives the F405's - its console, its configuration record in this part's
# flash, its log regions, its barometer and its receiver pins - and
# tests/test_board_feather.c drives the Adafruit breakout's, which is the same
# part as the F405's and a different header. Three boards cannot all define
# ak_board_init(). The AT32's entry points are renamed for the host build alone
# (src/boards/AERIALKIT_GHF435/board.h) and the Feather's likewise
# (src/boards/FEATHER_F405/board.h, AK_HOST_FEATHER), so this binary can link
# all three and each test runs the real board it is about. It is cheaper than
# compiling the whole test set a second time, which is the alternative and is
# what a fourth *part* would push for - a second board on a part that is
# already here costs one board file and one test file, because the arch layer
# is the same for both and is compiled once.
TEST_EXTRA_OBJS := $(HOST_OUT)/src/core/console.o \
                   $(HOST_OUT)/src/boards/AERIALKIT_F405/board.o \
                   $(HOST_OUT)/src/boards/AERIALKIT_GHF435/board.o \
                   $(HOST_OUT)/src/boards/FEATHER_F405/board.o \
                   $(HOST_OUT)/src/arch/arm/cortex-m4/fault.o \
                   $(HOST_OUT)/src/arch/esp32/fault.o \
                   $(HOST_OUT)/src/arch/esp32/led.o \
                   $(HOST_OUT)/src/arch/esp32/system.o \
                   $(HOST_OUT)/src/arch/esp32/adc_curve.o

$(HOST_OUT)/aerialkit-tests: $(FLIGHT_OBJS) $(CORE_OBJS) $(TEST_OBJS) \
                             $(ARCH_HOST_OBJS) $(TEST_EXTRA_OBJS)
	@mkdir -p $(@D)
	$(HOST_CC) $(HOST_CFLAGS) $(FLIGHT_OBJS) $(CORE_OBJS) $(TEST_OBJS) \
	    $(ARCH_HOST_OBJS) $(TEST_EXTRA_OBJS) -lm -o $@

$(HOST_OUT)/aerialkit-replay: $(FLIGHT_OBJS) $(SUPPORT_OBJS) $(HOST_OUT)/tools/replay.o
	@mkdir -p $(@D)
	$(HOST_CC) $(HOST_CFLAGS) $(FLIGHT_OBJS) $(SUPPORT_OBJS) \
	    $(HOST_OUT)/tools/replay.o -lm -o $@

# The parsers under bytes nobody sent them. It is built with everything else so
# that `make sanitize` gets a *sanitized* one - which is where it earns its
# keep; see tools/fuzz_parsers.c for what it checks and what it does not.
$(HOST_OUT)/aerialkit-fuzz: $(FLIGHT_OBJS) $(SUPPORT_OBJS) \
                            $(HOST_OUT)/src/core/ak_proto.o \
                            $(HOST_OUT)/tools/fuzz_parsers.o
	@mkdir -p $(@D)
	$(HOST_CC) $(HOST_CFLAGS) $(FLIGHT_OBJS) $(SUPPORT_OBJS) \
	    $(HOST_OUT)/src/core/ak_proto.o $(HOST_OUT)/tools/fuzz_parsers.o \
	    -lm -o $@

$(HOST_OUT)/aerialkit-gen-trace: $(HOST_OUT)/tools/gen-trace.o
	@mkdir -p $(@D)
	$(HOST_CC) $(HOST_CFLAGS) $(HOST_OUT)/tools/gen-trace.o -lm -o $@

# The simulator is the firmware's half of the config protocol on a pipe, with
# the real parameter table behind it - so the client can be tested against the
# firmware rather than alongside it.
$(HOST_OUT)/aerialkit-sim: $(FLIGHT_OBJS) $(SUPPORT_OBJS) \
                           $(HOST_OUT)/src/core/ak_proto.o \
                           $(HOST_OUT)/src/core/ak_flashlog.o \
                           $(HOST_OUT)/tools/akproto_sim.o
	@mkdir -p $(@D)
	$(HOST_CC) $(HOST_CFLAGS) $(FLIGHT_OBJS) $(SUPPORT_OBJS) \
	    $(HOST_OUT)/src/core/ak_proto.o $(HOST_OUT)/src/core/ak_flashlog.o \
	    $(HOST_OUT)/tools/akproto_sim.o -lm -o $@

# The whole firmware against a simulated aircraft. core/time.c is left out: the
# simulator supplies the clock and the delay, so that time passes only when the
# firmware waits - and the airframe moves with it.
FW_SIM_OBJS := $(filter-out $(HOST_OUT)/src/core/time.o,$(CORE_OBJS))
FW_SIM_EXTRA := $(HOST_OUT)/src/core/console.o $(HOST_OUT)/tests/host_stubs.o
$(HOST_OUT)/aerialkit-fw-sim: $(FLIGHT_OBJS) $(FW_SIM_OBJS) \
                              $(HOST_OUT)/src/core/main.o \
                              $(HOST_OUT)/tools/fw_sim.o \
                              $(HOST_OUT)/tools/attitude_step.o $(FW_SIM_EXTRA)
	@mkdir -p $(@D)
	$(HOST_CC) $(HOST_CFLAGS) $(FLIGHT_OBJS) $(FW_SIM_OBJS) \
	    $(HOST_OUT)/src/core/main.o $(HOST_OUT)/tools/fw_sim.o \
	    $(HOST_OUT)/tools/attitude_step.o $(FW_SIM_EXTRA) -lm -o $@

# The estimator against an independent quaternion oracle: truth is integrated
# from body rates by q' = 0.5 q (x) (0, omega) and the accelerometer the
# estimator is fed comes from that quaternion, so agreement means both are
# right rather than both share an approximation (F2 in the 2026-09-18
# assessment). A separate binary with its own main(), which is why it lives in
# tests/oracle/ rather than tests/: `TEST_OBJS` globs tests/*.c into the main
# test binary and a second main() would collide with it.
#
# This is B2's acceptance test. It failed until C7 fixed the propagation, which
# integrated body rates as if they were Euler derivatives (F1); it now measures
# 0.0003 deg against a 2 deg tolerance and runs as a hard `ci.sh` stage, so this
# is a regression guard rather than a to-do.
estimator-oracle: $(HOST_OUT)/estimator-oracle
	@$(HOST_OUT)/estimator-oracle

# Two objects, not FLIGHT_OBJS: the estimator and the maths it uses, and
# nothing else. A test that links the whole flight directory can pass or fail
# for reasons that have nothing to do with the file under test.
ESTIMATOR_OBJS := $(HOST_OUT)/src/core/flight/ak_estimator.o \
                  $(HOST_OUT)/src/core/flight/ak_math.o

$(HOST_OUT)/estimator-oracle: tests/oracle/test_estimator_oracle.c \
                              $(ESTIMATOR_OBJS)
	@mkdir -p $(@D)
	$(HOST_CC) $(HOST_CFLAGS) tests/oracle/test_estimator_oracle.c \
	    $(ESTIMATOR_OBJS) -lm -o $@

# The plant's attitude step against the same kind of oracle, and for the same
# reason: `sim_attitude_step()` is what every other measurement in this suite is
# taken against, so the one thing that must not be taken on trust is the step
# itself. Until now the only thing asserting it was the kinematics was a scratch
# harness in a job's temporary directory behind an AK_ATTITUDE_TRACE hook - see
# the ledger's item 6. The arithmetic now lives in tools/attitude_step.c so that
# a second translation unit can reach it at all.
#
# Two objects, not FLIGHT_OBJS, on the estimator oracle's rule: the step under
# test and the trigonometry it uses, and nothing else. `-Itools` is on this rule
# alone rather than in HOST_CFLAGS, because this is the only host source outside
# tools/ that includes a tools/ header.
ATTITUDE_OBJS := $(HOST_OUT)/tools/attitude_step.o \
                 $(HOST_OUT)/src/core/flight/ak_math.o

attitude-kinematics: $(HOST_OUT)/attitude-kinematics
	@$(HOST_OUT)/attitude-kinematics

$(HOST_OUT)/attitude-kinematics: tests/oracle/test_attitude_kinematics.c \
                                 $(ATTITUDE_OBJS)
	@mkdir -p $(@D)
	$(HOST_CC) $(HOST_CFLAGS) -Itools tests/oracle/test_attitude_kinematics.c \
	    $(ATTITUDE_OBJS) -lm -o $@

# B3's baseline: every route into the configuration, measured against one
# parameter table, and the armed-state policy none of the routes has yet. It
# asserts the behaviour B3 is supposed to produce, so it FAILS until B3.1 and
# B3.2 land - the same shape, and the same reason for living in tests/oracle/,
# as the estimator oracle above.
#
# FLIGHT_OBJS and CORE_OBJS, unlike the oracle's two objects: the question here
# is a property of the *routes*, so the console, the protocol, the parameter
# table and the flight core all have to be the real ones. A stand-in for any of
# them would be measuring the stand-in.
config-policy: $(HOST_OUT)/config-policy
	@$(HOST_OUT)/config-policy

# The port's own fault record, which is what the tests binary links too: the
# console's status report calls ak_fault_present(), and the stand-in in
# tests/host_stubs.c is the *simulator's* - linking that here would be the one
# place in this binary where a stand-in replaced the real thing.
#
# And the same objects the tests binary links, which this rule had been a board
# file short of: the console prints ak_board_name() and ak_board_clock_summary(),
# which live in a board file and nowhere else, so this binary stopped linking the
# day the console started naming its board - `undefined reference to
# ak_board_name`. Nothing noticed, because nothing ran it. Measured 2026-09-30
# against 2997dfe: the same two calls are in ak_cli.c and the same short rule is
# in the Makefile, so the breakage predates this line and the oracle has been
# unrunnable rather than failing.
#
# The board brings the arch layer with it, and the arch layer brings the ESP32
# files and the two sensor buses, whose stand-ins live in the other files under
# tests/. So the honest object set is the tests binary's own, less the one file
# that has `main` in it: `$(TEST_OBJS)` minus `tests/test_aerialkit.o`. Anything
# smaller and this rule is choosing which stand-ins count, which is the one
# judgement the comment above says not to make - and the two attempts at a
# smaller set are recorded here because both failed in exactly that way, first
# on ak_board_name and then on host_i2c_read.
#
# Linked as an archive rather than as objects, so that set costs nothing: the
# linker pulls the stand-ins (`tests/host_i2c_model.o` and its siblings, reached
# from the sensor buses) and leaves the test cases, which nothing refers to.
# Listing the same objects on the command line instead pulls all of them, and
# then the twenty test files' calls to the harness's `expect` are undefined
# here - which is the error that says an archive is the mechanism and not a
# way of getting a different one.
#
# `--start-group` is for the members that refer to each other. `test_aerialkit.o`
# is left out because it is the one file with `main` and with the harness's
# `expect`, and this binary has its own of both.
CONFIG_POLICY_LIB  := $(HOST_OUT)/libaerialkit-config-policy.a
CONFIG_POLICY_OBJS := $(filter-out $(HOST_OUT)/tests/test_aerialkit.o, \
                       $(FLIGHT_OBJS) $(CORE_OBJS) $(TEST_OBJS) \
                       $(ARCH_HOST_OBJS) $(TEST_EXTRA_OBJS))

$(CONFIG_POLICY_LIB): $(CONFIG_POLICY_OBJS)
	@mkdir -p $(@D)
	@rm -f $@
	$(AR) rcs $@ $(CONFIG_POLICY_OBJS)

$(HOST_OUT)/config-policy: tests/oracle/test_config_policy.c $(CONFIG_POLICY_LIB)
	@mkdir -p $(@D)
	$(HOST_CC) $(HOST_CFLAGS) tests/oracle/test_config_policy.c \
	    -Wl,--start-group $(CONFIG_POLICY_LIB) -Wl,--end-group -lm -o $@

# B3.3's baseline: the configuration ring recycled, with the power cut inside
# the erase. The board's own comment names the hazard and records that the
# second sector which would remove it was not taken; this is that sentence
# turned into a number. It asserts the behaviour B3.3 is supposed to produce, so
# it FAILS until the repair lands - the same shape, and the same reason for
# living in tests/oracle/, as the two oracles above.
#
# The F405's board object and its flash driver, which is the pair under test: a
# stand-in for either would be measuring the stand-in. The model behind the
# driver is tests/host_flash_model.c, whose `program_refused` is exactly the
# state this is about - the erase completed and the program did not.
config-recycle: $(HOST_OUT)/config-recycle
	@$(HOST_OUT)/config-recycle

# B4's baseline: what the control loop does with a timestamp it did not expect.
# The measurement is a ratio - the attitude advance across a gap of N ms over
# the advance across one millisecond, which is N if the elapsed time was
# believed and 1 if step_dt() clamped it to a single millisecond - so the
# estimator's own arithmetic cancels and this file needs no quaternion oracle of
# its own. It asserts the behaviour B4 is supposed to produce, so it FAILS until
# the repair lands: the same shape, and the same reason for living in
# tests/oracle/, as the three probes above.
#
# FLIGHT_OBJS and SUPPORT_OBJS: step_dt() is static inside ak_flight.c, so the
# only way to measure it is through ak_flight_step, which means the real flight
# core and the real receiver decode. There are no board seams here at all - a
# timestamp is arithmetic, and nothing in this probe touches a board.
#
# **SUPPORT_OBJS and not CORE_OBJS, which is what this rule used to link.** The
# difference is the console, the protocol, the console link and the flash log -
# and linking them was not neutral, because `ak_cli.o`'s banner prints
# `ak_board_name()`. So a probe that touches no board needed one to answer for
# the console's banner, and when the banner gained that call this rule stopped
# linking. Nothing said so: the oracle is built by CI and by nothing else, so it
# was red for as long as it took a stage to be read. The cure is to link what the
# probe measures rather than everything the core has, which is also what makes
# the sentence above true rather than aspirational - a rule that links the
# console is a rule that needs a board, however little it means to.
#
# The same lesson, one file over: config-policy could not link either, for the
# same call, and was repaired the same way. Two probes, one cause, and the cause
# was a link line wider than the thing it links.
timing: $(HOST_OUT)/timing
	@$(HOST_OUT)/timing

$(HOST_OUT)/timing: tests/oracle/test_timing.c \
                    $(FLIGHT_OBJS) $(SUPPORT_OBJS) \
                    $(HOST_OUT)/src/arch/arm/cortex-m4/fault.o
	@mkdir -p $(@D)
	$(HOST_CC) $(HOST_CFLAGS) tests/oracle/test_timing.c \
	    $(FLIGHT_OBJS) $(SUPPORT_OBJS) \
	    $(HOST_OUT)/src/arch/arm/cortex-m4/fault.o -lm -o $@

# Every oracle, linked but not run - and it is a prerequisite of `test:`.
#
# The five probes above live in tests/oracle/, are run by their own CI stages,
# and are built by nothing else. That is a reasonable place for them to be and
# it had one consequence nobody had looked at: a link line is only tested when
# its target is rebuilt and run, so a probe that stopped linking kept its last
# good binary and said nothing. `timing` did exactly that - see its rule above
# for the cause - and the first report of it was a CI stage that had been red
# for as long as it took somebody to read the stage.
#
# This target is the cheap half of the cure: link them, do not run them, and let
# `make test` - which every machine runs - be where a broken link line stops
# being silent. Running them still belongs to CI, where a probe that fails a
# measurement is a result rather than an interruption. It is on `test:`'s
# prerequisite list rather than inside its recipe so that a link error is a
# make error, in the same shape as every other build failure here.
oracle-link: $(HOST_OUT)/estimator-oracle $(HOST_OUT)/attitude-kinematics \
             $(HOST_OUT)/config-policy $(HOST_OUT)/config-recycle \
             $(HOST_OUT)/timing

RECYCLE_MODELS := $(HOST_OUT)/tests/host_flash_model.o \
                  $(HOST_OUT)/tests/host_adc_model.o \
                  $(HOST_OUT)/tests/host_i2c_model.o \
                  $(HOST_OUT)/tests/host_spi_model.o \
                  $(HOST_OUT)/tests/host_usb_model.o

# The F405's arch objects alone, not ARCH_HOST_OBJS: that list also carries the
# AT32's and the ESP32's, whose own models and IDF symbols exist for the main
# test binary and have nothing to do with this question. Linking them here would
# be a probe that needs a wifi stack to measure a flash erase.
RECYCLE_ARCH := $(patsubst %.c,$(HOST_OUT)/%.o, \
                  $(filter src/arch/stm32f405/%,$(ARCH_HOST_SRCS))) \
                $(HOST_OUT)/src/core/time.o

$(HOST_OUT)/config-recycle: tests/oracle/test_config_recycle.c \
                            $(RECYCLE_MODELS) $(RECYCLE_ARCH) \
                            $(HOST_OUT)/src/boards/AERIALKIT_F405/board.o \
                            $(HOST_OUT)/src/arch/arm/cortex-m4/fault.o \
                            $(FLIGHT_OBJS) $(CORE_OBJS)
	@mkdir -p $(@D)
	$(HOST_CC) $(HOST_CFLAGS) tests/oracle/test_config_recycle.c \
	    $(RECYCLE_MODELS) $(RECYCLE_ARCH) \
	    $(HOST_OUT)/src/boards/AERIALKIT_F405/board.o \
	    $(HOST_OUT)/src/arch/arm/cortex-m4/fault.o \
	    $(FLIGHT_OBJS) $(CORE_OBJS) -lm -o $@

# Software in the loop, out loud: the transcript of both whole flights, which is
# what `make sil` is for. `make test` runs the same binary and reads its exit
# code instead. See docs/18-software-in-the-loop.md.
sil: $(HOST_OUT)/aerialkit-fw-sim
	@$(HOST_OUT)/aerialkit-fw-sim $(SIL_SECONDS) quad
	@$(HOST_OUT)/aerialkit-fw-sim $(SIL_WING_SECONDS) wing
	@$(HOST_OUT)/aerialkit-fw-sim $(SIL_BENCH_SECONDS) bench
	@$(HOST_OUT)/aerialkit-fw-sim $(SIL_MISSION_SECONDS) mission
	@$(HOST_OUT)/aerialkit-fw-sim $(SIL_FENCE_SECONDS) fence
	@$(HOST_OUT)/aerialkit-fw-sim $(SIL_QUADRTH_SECONDS) quadrth
	@$(HOST_OUT)/aerialkit-fw-sim $(SIL_GPSLOST_SECONDS) gpslost
	@$(HOST_OUT)/aerialkit-fw-sim $(SIL_QUADMISSION_SECONDS) mission quad
	@$(HOST_OUT)/aerialkit-fw-sim $(SIL_BATTERY_SECONDS) battery
	@$(HOST_OUT)/aerialkit-fw-sim $(SIL_RATE_SECONDS) rate
	@$(HOST_OUT)/aerialkit-fw-sim $(SIL_QUADRTH_SECONDS) quadrth nobaro
	@$(HOST_OUT)/aerialkit-fw-sim $(SIL_SECONDS) noimu
	@$(HOST_OUT)/aerialkit-fw-sim $(SIL_BADFIX_SECONDS) quadrth badfix
	@$(HOST_OUT)/aerialkit-fw-sim $(SIL_QUADRTH_SECONDS) quadrth gpsjump
	@$(HOST_OUT)/aerialkit-fw-sim $(SIL_QUADRTH_SECONDS) quadrth noisy
	@$(HOST_OUT)/aerialkit-fw-sim $(SIL_WING_NOISY_SECONDS) wing noisy
	@$(HOST_OUT)/aerialkit-fw-sim $(SIL_SECONDS) quad noisy
	@$(HOST_OUT)/aerialkit-fw-sim $(SIL_SECONDS) quad rxfailsafe
	@$(HOST_OUT)/aerialkit-fw-sim $(SIL_FLATPACK_SECONDS) mission quad flatpack
	@$(HOST_OUT)/aerialkit-fw-sim $(SIL_TILT_SECONDS) tilt
	@$(HOST_OUT)/aerialkit-fw-sim $(SIL_LAUNCH_SECONDS) launch
	@$(HOST_OUT)/aerialkit-fw-sim 3 fault
	@$(HOST_OUT)/aerialkit-fw-sim $(SIL_BADBOARD_SECONDS) badboard
	@$(HOST_OUT)/aerialkit-fw-sim $(SIL_BADBOOT_SECONDS) badboard noout
	@$(HOST_OUT)/aerialkit-fw-sim $(SIL_BADBOOT_SECONDS) badboard twomotor
	@$(HOST_OUT)/aerialkit-fw-sim $(SIL_HOLD_SECONDS) mission quad hold
	@$(HOST_OUT)/aerialkit-fw-sim $(SIL_HOLD_SECONDS) mission hold

# Re-record the transcripts in docs/evidence. They are recordings of a run
# rather than build output - the revision and the build stamp inside each one
# are the ones it was made from - so this is how they stay honest after the
# console or the loop changes. `make sil` plays the same sessions out loud to a
# terminal; this writes them down.
SIL_TRACE ?= 1

sil-trace: $(HOST_OUT)/aerialkit-fw-sim
	@mkdir -p docs/evidence
	@$(HOST_OUT)/aerialkit-fw-sim $(SIL_SECONDS) quad > docs/evidence/sil-flight.txt
	@$(HOST_OUT)/aerialkit-fw-sim $(SIL_WING_SECONDS) wing > docs/evidence/sil-wing-flight.txt
	@$(HOST_OUT)/aerialkit-fw-sim $(SIL_WING_LOST_SECONDS) winglost \
	    > docs/evidence/sil-wing-lost.txt
	@$(HOST_OUT)/aerialkit-fw-sim $(SIL_BENCH_SECONDS) bench > docs/evidence/sil-bench-calibration.txt
	@$(HOST_OUT)/aerialkit-fw-sim $(SIL_MISSION_SECONDS) mission > docs/evidence/sil-mission.txt
	@$(HOST_OUT)/aerialkit-fw-sim $(SIL_FENCE_SECONDS) fence > docs/evidence/sil-fence.txt
	@$(HOST_OUT)/aerialkit-fw-sim $(SIL_QUADRTH_SECONDS) quadrth > docs/evidence/sil-quad-return.txt
	@$(HOST_OUT)/aerialkit-fw-sim $(SIL_GPSLOST_SECONDS) gpslost > docs/evidence/sil-gps-lost.txt
	@$(HOST_OUT)/aerialkit-fw-sim $(SIL_GPSLOSTLAND_SECONDS) gpslostland > docs/evidence/sil-gps-lost-land.txt
	@$(HOST_OUT)/aerialkit-fw-sim $(SIL_QUADRTH_SECONDS) quadrth norange > docs/evidence/sil-quad-return-norange.txt
	@$(HOST_OUT)/aerialkit-fw-sim $(SIL_QUADRTH_SECONDS) quadrth rangefail > docs/evidence/sil-quad-return-rangefail.txt
	@$(HOST_OUT)/aerialkit-fw-sim $(SIL_QUADRTH_SECONDS) quadrth bare > docs/evidence/sil-bare-board.txt
	@$(HOST_OUT)/aerialkit-fw-sim $(SIL_QUADMISSION_SECONDS) mission quad > docs/evidence/sil-quad-mission.txt
	@$(HOST_OUT)/aerialkit-fw-sim $(SIL_MISSION_CALM_SECONDS) mission quad calm > docs/evidence/sil-quad-mission-calm.txt
	@$(HOST_OUT)/aerialkit-fw-sim $(SIL_FLATPACK_SECONDS) mission quad flatpack > docs/evidence/sil-quad-mission-flatpack.txt
	@$(HOST_OUT)/aerialkit-fw-sim $(SIL_QUADMISSION_SECONDS) mission quad takeback > docs/evidence/sil-mission-takeback.txt
	@$(HOST_OUT)/aerialkit-fw-sim $(SIL_TILT_SECONDS) tilt > docs/evidence/sil-tilt.txt
	@$(HOST_OUT)/aerialkit-fw-sim $(SIL_LAUNCH_SECONDS) launch > docs/evidence/sil-launch.txt
	@$(HOST_OUT)/aerialkit-fw-sim $(SIL_LAUNCH_PACK_SECONDS) launch pack > docs/evidence/sil-launch-pack.txt
	@$(HOST_OUT)/aerialkit-fw-sim 3 fault > docs/evidence/sil-fault.txt
	@$(HOST_OUT)/aerialkit-fw-sim $(SIL_BADBOARD_SECONDS) badboard \
	    > docs/evidence/sil-preflight-lies.txt
	@$(HOST_OUT)/aerialkit-fw-sim $(SIL_BADBOOT_SECONDS) badboard noout \
	    > docs/evidence/sil-preflight-no-outputs.txt
	@$(HOST_OUT)/aerialkit-fw-sim $(SIL_BADBOOT_SECONDS) badboard twomotor \
	    > docs/evidence/sil-preflight-two-motors.txt
	@$(HOST_OUT)/aerialkit-fw-sim $(SIL_GPSQUIET_SECONDS) gpsquiet \
	    > docs/evidence/sil-gps-quiet.txt
	@$(HOST_OUT)/aerialkit-fw-sim $(SIL_TESTARM_SECONDS) testarm \
	    > docs/evidence/sil-output-test-arm.txt
	@$(HOST_OUT)/aerialkit-fw-sim $(SIL_ARM_NOW_SECONDS) armnow \
	    > docs/evidence/sil-armed-while-held.txt
	@$(HOST_OUT)/aerialkit-fw-sim $(SIL_REFUSE_SECONDS) refuse \
	    > docs/evidence/sil-calibrations-refused.txt
	@$(HOST_OUT)/aerialkit-fw-sim $(SIL_REFUSE_SECONDS) refuse rcquiet \
	    > docs/evidence/sil-calibrations-no-receiver.txt
	@$(HOST_OUT)/aerialkit-fw-sim $(SIL_HOLD_SECONDS) mission quad hold \
	    > docs/evidence/sil-quad-hold.txt
	@$(HOST_OUT)/aerialkit-fw-sim $(SIL_HOLD_SECONDS) mission hold \
	    > docs/evidence/sil-wing-hold.txt
	@$(HOST_OUT)/aerialkit-fw-sim $(SIL_BATTERY_SECONDS) battery > docs/evidence/sil-battery.txt
	@$(HOST_OUT)/aerialkit-fw-sim $(SIL_RATE_SECONDS) rate > docs/evidence/sil-rate-mode.txt
	@$(HOST_OUT)/aerialkit-fw-sim $(SIL_QUADRTH_SECONDS) quadrth nobaro > docs/evidence/sil-quad-return-nobaro.txt
	@$(HOST_OUT)/aerialkit-fw-sim $(SIL_SECONDS) noimu > docs/evidence/sil-no-imu.txt
	@$(HOST_OUT)/aerialkit-fw-sim $(SIL_BADFIX_SECONDS) quadrth badfix > docs/evidence/sil-quad-return-badfix.txt
	@$(HOST_OUT)/aerialkit-fw-sim $(SIL_QUADRTH_SECONDS) quadrth gpsjump > docs/evidence/sil-quad-return-jump.txt
	@$(HOST_OUT)/aerialkit-fw-sim $(SIL_QUADRTH_SECONDS) quadrth gyrobias > docs/evidence/sil-quad-return-gyrobias.txt
	@$(HOST_OUT)/aerialkit-fw-sim $(SIL_QUADRTH_SECONDS) quadrth noisy > docs/evidence/sil-quad-return-noisy.txt
	@$(HOST_OUT)/aerialkit-fw-sim $(SIL_WING_NOISY_SECONDS) wing noisy > docs/evidence/sil-wing-noisy.txt
	@$(HOST_OUT)/aerialkit-fw-sim $(SIL_SECONDS) quad noisy > docs/evidence/sil-quad-noisy.txt
	@$(HOST_OUT)/aerialkit-fw-sim $(SIL_SECONDS) quad rxfailsafe > docs/evidence/sil-rx-failsafe.txt
	@echo "wrote docs/evidence/sil-*.txt from this build"

# The client against the firmware's own code, over a pipe.
proto-test: host
	@# AK_PRODUCT is what the simulator was built as: the check compares the
	@# product name the firmware reports against the one it was compiled with,
	@# rather than against a literal that only one of the targets would pass.
	@AK_SIM=$(HOST_OUT)/aerialkit-sim AK_PRODUCT=$(PRODUCT) \
	    python3 tools/akproto_check.py
	@# And the same client against the *whole firmware*, on the console path a
	@# board actually uses: the protocol server in main.c, the real parameter
	@# table, the real logs. The check above drives a stand-in whose answers are
	@# its own; this one drives the aircraft's.
	@AK_SIM=$(HOST_OUT)/aerialkit-fw-sim AK_PRODUCT=$(PRODUCT) \
	    python3 tools/akproto_firmware_check.py
	@# And the *other* client: the Pi companion, which lives beside this
	@# checkout in the workspace rather than in it. Its check drives the same
	@# simulated board through the same protocol code, so a change to the
	@# wire or to a record's layout that breaks the companion fails here
	@# rather than on the bench. Skipped, with a line, when the companion is
	@# not beside the firmware - which is what the split into its own repo
	@# will look like.
	@# AK_SIM, AK_PRODUCT and AKPROTO_PATH are passed for the same reason they
	@# are above: all three are the *build's* rather than the script's.
	@# Without AK_SIM the companion looks for a simulator in build-host/, which
	@# is whatever was built there by hand last - on a clean tree, nothing.
	@#
	@# AKPROTO_PATH is newer than the other two and was missing, which meant
	@# this line had never once passed on this machine. The companion finds
	@# the firmware's protocol client by searching three places
	@# (companion/fc_link.py, `_PROTOCOL_CANDIDATES`): $AKPROTO_PATH, then
	@# `projects/aerialkit/tools` relative to itself, then
	@# `aerial-kit/aerialkit/tools`. The middle one assumes the
	@# companion and the firmware are siblings *under projects/*, and the last
	@# is a home directory that does not exist here - and this repository's
	@# firmware is at the root, one level up from projects/, so none of the
	@# three resolves. The import then raises before the script has read its
	@# own argv, which is why the path it is handed on the line below could
	@# never have helped. The firmware knows where its own tools are; it
	@# should say so. See 04-traps.md §40.
	@if [ -f ../projects/flight-computer/tools/fc_link_check.py ]; then \
	    AK_SIM=$(HOST_OUT)/aerialkit-sim AK_PRODUCT=$(PRODUCT) \
	    AKPROTO_PATH=$(CURDIR)/tools \
	    python3 ../projects/flight-computer/tools/fc_link_check.py \
	        "$(CURDIR)" || exit 1; \
	else \
	    echo "companion:  not in this checkout - the link check is skipped"; \
	fi
	@# And the third client, which is the one with a window on it: the
	@# configurator's half that talks to the aircraft, against the same
	@# simulator. A window cannot be tested; the controller under it can, and
	@# that is where everything that can be wrong lives (tools/akconfig.py).
	@AK_SIM=$(HOST_OUT)/aerialkit-sim python3 tools/akconfig_check.py
	@# And then the window itself, which the lines above used to say could not
	@# be tested at all. It can: Tk builds its widgets, fills them from the
	@# aircraft and answers when they are clicked, without a screen or a person
	@# (under Xvfb when this machine has no display, and it says so and skips
	@# where there is no Tk at all). It is the check that found the window's
	@# first real bugs - see tools/akconfig_window_check.py.
	@AK_SIM=$(HOST_OUT)/aerialkit-sim python3 tools/akconfig_window_check.py
	@# And the same window against a board it does not own: a Betaflight
	@# stand-in that answers MSP with Betaflight's own layouts and constants.
	@# It is the first half of what the owner asked for - a configurator that
	@# is compatible with the other firmwares - and the half that is provable
	@# on this machine: the client, the detection, and what the window shows
	@# and refuses to do. See tools/msp_check.py and docs/27-configurator.md.
	@#
	@# AK_SIM is passed for the same reason the two lines above pass it, and
	@# it is the reason this line took a target build down: the MSP check
	@# drives *our own* firmware as well (the window has to recognise our
	@# board before it ever asks about MSP), so it spawns a simulator - and
	@# with no AK_SIM it spawns `build-host/aerialkit-sim`, the default that
	@# exists in a working tree somebody has run `make test` in. In a target
	@# build, which keeps its host tools in `$(HOST_OUT)`, that either finds a
	@# *stale* simulator built by hand - a different product, so the check
	@# proves nothing about this target - or finds nothing at all and dies
	@# with a traceback. Both were true on 2026-09-18: the three harness
	@# builds that day passed against a leftover F405 simulator, and the same
	@# builds failed in a clean worktree. See 04-traps.md §27.
	@AK_SIM=$(HOST_OUT)/aerialkit-sim python3 tools/msp_check.py
	@python3 tools/akconfig_msp_check.py
	@# And the *third* protocol: MAVLink, which is ArduPilot and PX4. Its
	@# checks are against pymavlink - the reference implementation, kept in
	@# upstream/ for exactly this - so the frames this client builds are
	@# compared with the frames the reference builds rather than with the
	@# client's own idea of them. See tools/mavlink_check.py.
	@python3 tools/mavlink_check.py
	@python3 tools/akconfig_mavlink_check.py

# Just the window, for when that is the thing being changed.
window-test: host
	@AK_SIM=$(HOST_OUT)/aerialkit-sim python3 tools/akconfig_window_check.py

# And the window against the one firmware that streams: the ESP32 under QEMU,
# which is the only network link this machine has. It needs ESP-IDF on the path
# (esptool merges the image) and qemu-system-xtensa, so it is not part of
# `make test` - `scripts/esp32-proto.sh` runs it beside the port's own check,
# with IDF_PATH set the way that script sets it.
net-window-test:
	@python3 tools/akconfig_net_check.py

# The client against the firmware's own protocol server, which is the console
# path a board's configurator uses - and which is otherwise only exercised
# through the ESP32 under QEMU.
firmware-proto-check: host
	@AK_SIM=$(HOST_OUT)/aerialkit-fw-sim AK_PRODUCT=$(PRODUCT) \
	    python3 tools/akproto_firmware_check.py

# How much stack the image needs, from the compiler's own call graph. This is
# the other question a host cannot answer by running anything: the test build
# gets eight megabytes of stack and the part has tens of kilobytes, so an
# overflow is invisible here and looks like "the board stopped saying anything"
# on the bench - which is the failure this exists to find first.
# scripts/check-stack.sh builds the image once with `-fcallgraph-info=su` and
# tools/stack_report.py adds the frame sizes up along the graph. It is not part
# of `make test`: it needs the ARM toolchain, and it is a check on the *image*
# rather than on the host build. docs/21-port-on-the-host.md has the numbers.
STACK_LIMIT ?= 12288

stack-check:
	@./scripts/check-stack.sh $(BOARD) $(ARCH) $(PART) $(PRODUCT) \
	    --limit $(STACK_LIMIT)

# --- the web configurator's own suite ----------------------------------------
#
# `apps/configurator` is self-contained by design - its own package.json, its own
# node_modules, its own vitest - and until this target existed, nothing in this
# repository ran it. The two targets whose names sound like it are not it:
# `window-test` drives the Python window against a fake aircraft and `proto-test`
# drives the protocol client against the firmware over a pipe, and both are the
# firmware's side of the pair. Measured when this was written, 2026-09-20: 186
# tests passed, 10 skipped (sitl.test.ts, which needs a live SITL), 12 files,
# 5.6 s, plus `tsc --noEmit` clean. The app's README has the commands a person
# runs by hand; this target runs the same two, in the app's own directory.
#
# `CONFIGURATOR` is a variable rather than a path baked into the script so that a
# checkout that keeps the app somewhere else can point at it, and so the CI stage
# can hand down the same value it checked for.
CONFIGURATOR ?= $(abspath $(CURDIR)/../apps/configurator)

configurator-check:
	@CONFIGURATOR="$(CONFIGURATOR)" ./scripts/check-configurator.sh

# --- the host suite under the sanitizers -------------------------------------
#
# The tests check what the firmware computes; this checks that *how* it
# computes it is defined - no out-of-bounds index, no signed overflow, no left
# shift of a negative, no use of uninitialised memory - which is the one class
# of bug the checks cannot see, because on the compiler in front of you the
# undefined thing usually does what you meant. It is the same suite, built with
# the sanitizers and pointed at build-asan/ (which is ignored, and so is a
# build like any other).
#
# It runs all of the host work: the suite with its twenty-seven simulated
# sessions, the protocol checker with the configurator (which builds a simulator
# of its own - the one path `make test` never builds), and the fuzzer, which is
# worth more here than anywhere else because an out-of-bounds index in a parser
# is invisible in an ordinary build. About eight minutes.
#
# On its first run it found the firmware's own arithmetic using undefined
# behaviour in two places (the BMP280's datasheet shifts, which are signed
# shifts of negative values, and a BMP388 intermediate that overflows a signed
# 64-bit at a full-scale raw reading) plus the same flat-index fault in four
# test doubles. See docs/21-port-on-the-host.md and the CHANGELOG entry for
# 2026-09-17.
#
# `-fno-sanitize-recover=all` is what makes this a check rather than a report:
# a finding stops the run with a non-zero status. To collect the whole list
# instead, override it:
#
#   make sanitize SAN_FLAGS="-fsanitize=address,undefined -fno-sanitize-recover=address"
#
SAN_OUT   ?= build-asan
# The two extras are not part of `undefined`: a float division by zero is not
# undefined behaviour in C - it is `inf`, which is *worse* in a flight core,
# because it propagates quietly into an altitude or a motor output - and a
# float-to-integer conversion that is out of range is the same shape. Both are
# checked here (2026-09-17: clean across the suite, the sessions and the fuzzer).
SAN_FLAGS ?= -fsanitize=address,undefined,float-divide-by-zero,float-cast-overflow \
             -fno-omit-frame-pointer -fno-sanitize-recover=all

# What the sanitized runs are given in their environment, and why one option is
# not optional here.
#
# `tests/test_arch.c` runs the port's register code against a block that behaves
# by `mmap`ing the ARM System Control Space at its real address, `0xE000E000`
# (PRIVATE_BASE). On x86-64 that address is inside AddressSanitizer's shadow
# gap: shadow for a byte is `(address >> 3) + 0x7fff8000`, so `0xE000E000` asks
# for shadow at `0x9bff9c00`, which is past the end of the low shadow and inside
# the gap - a region ASan never maps. The mapping succeeds and the *write*
# would succeed; what fails is the interceptor, which cannot look up the shadow
# for the address and reports `unknown-crash ... in memset` against a page that
# is mapped and owned by the test.
#
# The other three windows the same file maps are unaffected, and that is the
# whole illustration: `0x40000000`, `0x50000000` and `0x08000000` all put their
# shadow in the low shadow (`0x87ff8000`, `0x89ff8000`, `0x80ff8000`), so they
# are checked normally. Only `0xE000E000` is past `0x8fff7000`.
#
# `protect_shadow_gap=0` is ASan's own answer to this, and its documented
# purpose is exactly this case - an application that maps memory in the shadow
# gap. It does not stop the arch tests being instrumented; they are compiled and
# checked with everything else, which is the property that matters, because a
# carve-out there would silence the instrument in the code most likely to write
# out of bounds.
#
# What it gives up is wider than it sounds, and this is measured rather than
# reasoned about. With the option on, ASan maps the gap as ordinary zeroed
# memory; its shadow therefore reads back as "nothing poisoned" and an access
# anywhere in the gap is permitted. Two probes: a write to `0xA0000000`, which
# nothing maps, succeeds; so does a write to `0x9bff9c00`, the shadow of the
# modelled page itself. Without the option both are reported as `unknown-crash`
# against the `memset` interceptor, which is the failure this variable exists to
# clear.
#
# What is *not* given up is the check that matters for the rest of the suite,
# and that is the reason this is acceptable: the same two probes run with and
# without the option report identically for everything outside the gap - a
# heap-buffer-overflow at `p[16]` on a 16-byte allocation, and a write to
# `0x1000` in unmapped low memory, are both caught in both runs. The gap is 24
# terabytes at addresses between `0x8fff7000` and `0x02008fff6fff`, above
# anything this program allocates; what is lost is a wild pointer whose value
# happens to land there. The remedy that would keep even that is to run the two
# arch files in a process of their own with the option and everything else
# without - and that remedy is now applied, so this variable no longer wraps the
# suite. Read on.
#
# Measured 2026-09-20: without it the suite aborts at tests/test_arch.c:155
# (`reset_registers`, writing `0xE000E000`) after 230 of its 2118 checks, and
# with it the run is `2118 checks, 0 failed`. That count is the reason this is
# worth more than a green tick: the abort is at check 230, so the 1888 checks
# after it - the mixer, the flight core, the estimators, the whole of the rest
# of the suite - had never once been through the sanitizers. The same abort
# happens on the committed tree, so it is inherited and not the property of any
# one change. docs/28-build.md carries the finding and the minimisation.
#
# **2026-09-24: the concession now covers the six tests that need it and
# nothing else.** The paragraph above used to end "there is a change to the test
# runner - `main` in tests/test_aerialkit.c takes no arguments, so there is
# nothing to select on today". That is no longer true: the runner takes
# `--only-registers` and `--no-registers`, and the two halves were measured to
# be exactly the whole run (885 + 1384 = 2269 checks, zero failed in each, as of
# 2026-09-29). So `sanitize` runs `--no-registers` with the gap **protected** -
# the stronger setting, and 1384 checks that had never had it - and applies this
# variable around the one invocation that maps `0xE000E000`. Measured before the
# change: the register half exits 1 with the gap protected (`unknown-crash ...
# located in the shadow gap area`) and 0 with it; the other checks, every
# `aerialkit-fw-sim` scenario, `proto-test`, `fuzz` and the ABI check all pass
# protected, so none of them needs it. What the concession is still worth
# keeping for is the 885.
SAN_ENV ?= ASAN_OPTIONS=protect_shadow_gap=0

# How hard the fuzzer looks, and with which seed. Fixed by default: a failure
# that cannot be re-run is a rumour, and the point of this one is that
# `make fuzz` says the same thing twice.
# 200,000 iterations is about five seconds of a developer machine's time and
# thirty seconds of a sanitized one, and it puts roughly a hundred megabytes of
# random bytes through the five parsers.
FUZZ_ITERATIONS ?= 200000
FUZZ_SEED ?= 0x5EED2026

fuzz: $(HOST_OUT)/aerialkit-fuzz
	@$(HOST_OUT)/aerialkit-fuzz --iterations $(FUZZ_ITERATIONS) \
	    --seed $(FUZZ_SEED)

# --- what the firmware runs, and what it never runs --------------------------
#
# Everything above says whether the firmware is *right*; this says which parts
# of it were asked. It builds the host tools with --coverage, runs the suite,
# the fuzzer and a session of every kind, and reports the lines GCC never saw
# executed - by directory, and worst file first. The answer is the map for the
# next test: the uncovered parts are the port registers (a mapped page is not a
# chip), the radios, and anything that needs the part to act on a write.
#
# `-O2` and coverage can disagree about inlining, so the numbers are about the
# code that *ran* rather than about the source anybody wrote - the same caveat
# tools/stack_report.py carries, for the same reason.
COV_OUT ?= build-cov

coverage:
	@# The stamp is part of `CFLAGS` and it is the current time, so a run made
	@# of several `make` invocations would rebuild everything on each of them
	@# and leave gcov reporting "stamp mismatch with notes file" - the data from
	@# the first run belongs to objects the second run replaced. Holding it
	@# still is what makes the map add up.
	@$(MAKE) --no-print-directory host HOST_OUT=$(COV_OUT) STAMP=coverage \
	    HOST_CC="$(HOST_CC) --coverage"
	@find $(COV_OUT) -name '*.gcda' -delete 2>/dev/null || true
	@# `make test` in full - every check, every session, the bench checklist,
	@# the docs check and the fuzzer - against the instrumented build, because
	@# the point of a coverage map is that it says what the suite reaches rather
	@# than what a list chosen here happens to reach. The fuzzer's iterations
	@# are cut for the same reason: under --coverage the same bytes cost several
	@# times as much and the map is what this is after.
	@$(MAKE) --no-print-directory test HOST_OUT=$(COV_OUT) STAMP=coverage \
	    HOST_CC="$(HOST_CC) --coverage" FUZZ_ITERATIONS=20000 \
	    > $(COV_OUT)/coverage-test.log 2>&1
	@# Then the two stages `make test` does not drive, each invoked the way its
	@# own stage invokes it: the protocol client against the simulator, and the
	@# firmware-side half against the streaming one.
	@AK_SIM=$(COV_OUT)/aerialkit-sim AK_PRODUCT=$(PRODUCT) \
	    python3 tools/akproto_check.py > /dev/null
	@AK_SIM=$(COV_OUT)/aerialkit-fw-sim AK_PRODUCT=$(PRODUCT) \
	    python3 tools/akproto_firmware_check.py > /dev/null
	@# `window-test` is deliberately not here, and that is a measurement rather
	@# than a judgement about it: it drives `aerialkit-sim` too, and running
	@# `akconfig_window_check.py` against this instrumented build on 2026-09-24
	@# moved the firmware count by 0 (8578 before, 8578 after, exit 0) and the
	@# harness count by 0 (14469 of 14936 both times). It reaches no line the
	@# suite above does not already reach, so adding it would buy the map
	@# nothing. `net-window-test` cannot run here at all (it wants
	@# qemu-system-xtensa), and `sanitize`, `profiles` and `stack` are different
	@# builds of the same sources rather than other roads through them.
	@# And the five oracles, which `make test` cannot drive: they are stages of
	@# their own, because when one fails the report has to name which of the
	@# five. The suite is declared once, in `scripts/ci.sh`, by the assignment
	@# `--only suite` reads -
	@#   GROUP_suite="host test estimator-oracle attitude-kinematics
	@#                config-policy config-recycle timing boards contract control"
	@# - and five of those ten `make test` already reaches: itself and `host`
	@# directly, `boards`, `contract` and `control` through the Python checks
	@# inside it. These five it does not. A map built from `make test` alone
	@# therefore reports the lines they reach as lines nothing reaches, which is
	@# the one thing a coverage map must not do: its whole job is to send the
	@# next test somewhere useful, and a line wrongly marked unreached sends it
	@# to write a test that already exists. Measured on 2026-09-24, wiring them
	@# in moves the firmware line count in one direction only - 8578 of 8862 to
	@# 8584 of 8862, six lines - and the six are the clock-reset branch at
	@# `src/core/flight/ak_flight.c:361`, which the timing oracle's "four faults,
	@# four counters" scenario drives and which no SIL session reaches. A fault
	@# path was sitting in an unreached list.
	@$(MAKE) --no-print-directory estimator-oracle attitude-kinematics \
	    config-policy config-recycle timing HOST_OUT=$(COV_OUT) \
	    STAMP=coverage HOST_CC="$(HOST_CC) --coverage" \
	    > $(COV_OUT)/coverage-oracles.log 2>&1
	@python3 tools/coverage_report.py $(COV_OUT)

# --- and the same suite on the other architecture ----------------------------
#
# This machine is aarch64 and the laptop is x86_64, and "it builds and runs on
# both" is a claim the plan makes (M1). It was a *procedure* - install the cross
# compiler and qemu, run three commands - which is the kind of claim that goes
# stale quietly, so it is a target now. What it catches that a same-architecture
# run cannot: a signedness assumption (`char` is signed there and unsigned here,
# which has already cost this repository one warning), and anything that quietly
# relies on a pointer width or an alignment.
#
# It skips with a line, like the ESP32's QEMU checks do, so a machine without the
# cross toolchain is not a failing machine - it just does not make this claim.
CROSS_CC   ?= x86_64-linux-gnu-gcc
CROSS_QEMU ?= qemu-x86_64
CROSS_ROOT ?= /usr/x86_64-linux-gnu
CROSS_OUT  ?= build-x86

cross-check:
	@if ! command -v $(CROSS_CC) >/dev/null || ! command -v $(CROSS_QEMU) >/dev/null; then \
	    echo "cross-check: no $(CROSS_CC)/$(CROSS_QEMU) on this machine - skipped"; \
	    exit 0; \
	fi
	@$(MAKE) --no-print-directory host HOST_OUT=$(CROSS_OUT) HOST_CC=$(CROSS_CC)
	@printf 'cross-check: the suite as x86_64 sees it\n'
	@$(CROSS_QEMU) -L $(CROSS_ROOT) $(CROSS_OUT)/aerialkit-tests
	@$(CROSS_QEMU) -L $(CROSS_ROOT) $(CROSS_OUT)/aerialkit-fw-sim 20 quad \
	    | grep '^sim: \(ok\|FAIL\|PASS\)'
	@$(CROSS_QEMU) -L $(CROSS_ROOT) $(CROSS_OUT)/aerialkit-fw-sim 60 wing \
	    | grep '^sim: \(ok\|FAIL\|PASS\)'
	@$(CROSS_QEMU) -L $(CROSS_ROOT) $(CROSS_OUT)/aerialkit-fuzz \
	    --iterations 20000 --seed 0x5EED2026

sanitize:
	@$(MAKE) --no-print-directory host HOST_OUT=$(SAN_OUT) \
	    HOST_CC="$(HOST_CC) $(SAN_FLAGS)"
	@# The suite in two halves, and the order matters: the 1384 checks that do
	@# not map `0xE000E000` run first, with the shadow gap **protected**, which
	@# is the setting they have never been run under. `$(SAN_ENV)` is not on
	@# this line and must not be.
	@$(MAKE) --no-print-directory test HOST_OUT=$(SAN_OUT) \
	    HOST_CC="$(HOST_CC) $(SAN_FLAGS)" TEST_ARGS=--no-registers
	@# And the six that do, in an invocation of their own, as the only place
	@# the concession applies. It is the binary the line above has just built
	@# and relinked, run directly rather than through `make test`, so that the
	@# environment difference between the two halves is visible in one file.
	@$(SAN_ENV) $(SAN_OUT)/aerialkit-tests --only-registers
	@# And the protocol checker, which builds its own simulator
	@# (tools/akproto_sim.c) and drives the configurator against it - the one
	@# path that `make test` does not build, and the one whose flash double had
	@# the same flat-index fault as the simulator's.
	@$(MAKE) --no-print-directory proto-test HOST_OUT=$(SAN_OUT) \
	    HOST_CC="$(HOST_CC) $(SAN_FLAGS)"
	@# And the parsers, under the sanitizers, which is where fuzzing belongs:
	@# an out-of-bounds index in a parser is invisible in an ordinary build.
	@$(MAKE) --no-print-directory fuzz HOST_OUT=$(SAN_OUT) \
	    HOST_CC="$(HOST_CC) $(SAN_FLAGS)"
	@# And the five oracles, for the reason the coverage target runs them and
	@# for one more. The same reason: `make test` cannot drive them - they are
	@# stages of their own, so that a failure names which of the five - which
	@# meant that until this line existed the fault paths they cover had never
	@# been under the sanitizers at all. The one more: they are the only thing
	@# in this repository that drives the flight core's fault branches - a clock
	@# that went backwards, a gap past the catch-up budget, a duplicate sample,
	@# an unusable one - and a fault branch is where an index or a length goes
	@# wrong on the path nobody takes. This is the second of the two targets
	@# that had a narrower idea of "the whole suite" than `scripts/ci.sh`'s
	@# `GROUP_suite` declares; `tools/suite_scope_check.py` now holds the two
	@# together, so a sixth oracle cannot be added to the gate and forgotten
	@# here.
	@$(MAKE) --no-print-directory estimator-oracle attitude-kinematics \
	    config-policy config-recycle timing HOST_OUT=$(SAN_OUT) \
	    HOST_CC="$(HOST_CC) $(SAN_FLAGS)"
	@printf 'aerialkit: the suite under the sanitizers, green\n'

# Regenerate the recorded trace used as M1 evidence. It is a recording, not a
# build output: it lives in docs/evidence/ and is committed.
trace: host
	@mkdir -p docs/evidence
	@$(HOST_OUT)/aerialkit-gen-trace > docs/evidence/trace-hover.csv
	@echo "wrote docs/evidence/trace-hover.csv"
	@$(HOST_OUT)/aerialkit-replay quad-x < docs/evidence/trace-hover.csv \
	    | tail -5

$(HOST_OUT)/%.o: %.c $(HOST_OUT)/.cflags.d/%.o.flags
	@mkdir -p $(@D)
	$(HOST_CC) $(HOST_CFLAGS) $(HOST_CFLAGS_TARGET) -MMD -MP -c $< -o $@

# One test gets the port's own directories on its include path, and only that
# one: tests/test_arch.c includes the real board file and the real register
# map so it can run the real port code. Everything else stays as it was, so a
# core file cannot reach an MCU header by accident on this build - which is the
# rule that keeps the ESP32 port a port, and it is checked where it matters by
# `make port` and by the ESP32 build, neither of which has these paths.
$(HOST_OUT)/tests/test_arch.o: HOST_CFLAGS_TARGET += -Isrc/arch/stm32f405 \
                                              -Isrc/boards/AERIALKIT_F405

# The board file is compiled for the same reason: the configuration record -
# magic, length, checksum - is board code, and it is the piece that decides
# whether a save that was interrupted is applied or refused.
#
# It takes `$(HOST_FITTED_AERIALKIT_F405)` for the reason the variable above
# gives: whether the barometer and the divider are soldered on is a `#ifndef`
# in this board's header, so an image built with them fitted and a test object
# built without them are two different boards that agree about nothing - which
# is exactly what the fitted pass used to be.
$(HOST_OUT)/src/boards/AERIALKIT_F405/board.o: HOST_CFLAGS_TARGET += \
    -Isrc/arch/stm32f405 -Isrc/boards/AERIALKIT_F405 \
    $(HOST_FITTED_AERIALKIT_F405)

# The third board, for the same reason, on this part's include path and with
# its entry points renamed - see the note on TEST_EXTRA_OBJS. Its macros are
# unconditional in its header, so the fitted flags have nothing to switch here;
# the line is the same shape as the other two so that the rule is the rule.
$(HOST_OUT)/src/boards/AERIALKIT_GHF435/board.o: HOST_CFLAGS_TARGET += \
    -DAK_HOST_AT32 -Isrc/arch/at32f435 -Isrc/boards/AERIALKIT_GHF435 \
    $(HOST_FITTED_AERIALKIT_GHF435)

# And the fault record, which is board code in the sense that matters here: it
# reads the Cortex-M's fault status registers through the same mapped page the
# rest of the port runs against, and it is the file that has to be right when
# a board stops saying anything. tests/test_fault.c drives it with a frame of
# its own making - that is what the record half of it exists as its own
# function for.
$(HOST_OUT)/src/arch/arm/cortex-m4/fault.o: HOST_CFLAGS_TARGET += \
    -Isrc/arch/stm32f405 -Isrc/boards/AERIALKIT_F405

# And the board's own file, which needs the port's register map and its board
# header for the same reason: it is where the two meet.
#
# This is the object the fitted pass exists for: the `#if AK_BOARD_BARO_FITTED`
# arms in it are the checks that were never taken, and a define that reaches
# board.c and not this file is a test that asks the bare question of a fitted
# board - so it takes the same flags the board object does, from the same
# variable, in the same pass.
$(HOST_OUT)/tests/test_board_f405.o: HOST_CFLAGS_TARGET += \
    -Isrc/arch/stm32f405 -Isrc/boards/AERIALKIT_F405 \
    $(HOST_FITTED_AERIALKIT_F405)

# The third target's board file, whose entry points are renamed for this build
# (the same trick as its arch layer): two boards cannot both define
# ak_board_init() in one binary.
$(HOST_OUT)/tests/test_board_ghf435.o: HOST_CFLAGS_TARGET += -Isrc/arch \
    -Isrc/arch/at32f435 -Isrc/boards/AERIALKIT_GHF435 -DAK_HOST_AT32 \
    $(HOST_FITTED_AERIALKIT_GHF435)

# And the fourth board file, which is the *second* on this part - and that is
# the difference from the AT32 above. There the rename buys a second set of
# arch objects; here the arch objects are the same ones (this part's) and the
# only file that differs is board.c, so the Feather brings the board file and
# its test and nothing else. The rename is still needed for the one reason: two
# definitions of ak_board_init() in one binary.
# `-DAK_HOST_FEATHER` is on the test object as well as the board's, and that is
# not symmetry: it is what makes the test's own calls to `ak_board_init()` and
# the rest resolve to *this* board. Without it on this object the test compiles
# against the unrenamed names, links against the WeAct board's file, and every
# assertion in it becomes a check of the wrong board that passes - which is how
# this line was found, with 30 failures in tests/test_board_feather.c before it
# was here.
$(HOST_OUT)/src/boards/FEATHER_F405/board.o: HOST_CFLAGS_TARGET += \
    -DAK_HOST_FEATHER -Isrc/arch/stm32f405 -Isrc/boards/FEATHER_F405 \
    $(HOST_FITTED_FEATHER_F405)
$(HOST_OUT)/tests/test_board_feather.o: HOST_CFLAGS_TARGET += \
    -DAK_HOST_FEATHER -Isrc/arch/stm32f405 -Isrc/boards/FEATHER_F405 \
    $(HOST_FITTED_FEATHER_F405)

# And the flash driver is compiled against the modelled controller in
# tests/host_flash_model.c, because its success path - a sector that erases, a
# word that lands where it was addressed - is the one thing a plain mapped
# region cannot reach: it needs the part to act on what is written. The flag is
# on this object alone; the target build never sees it.
$(HOST_OUT)/src/arch/stm32f405/flash.o: HOST_CFLAGS_TARGET += -DAK_HOST_FLASH
# And both ARM SPI ports against a modelled *device* (tests/host_spi_model.c),
# for the half a mapped register block cannot be: the wing's board has an
# ICM-42688-P on that bus, and the board's own read, write and burst - and the
# SPI loopback that needs one jumper - had never run anywhere.
$(HOST_OUT)/src/arch/stm32f405/spi.o: HOST_CFLAGS_TARGET += -DAK_HOST_SPI
# And both converters against a model of the one thing a mapped block cannot do:
# finish a conversion (tests/host_adc_model.c). The flight pack's voltage is the
# caller, and its *success* path had never run on this machine.
$(HOST_OUT)/src/arch/stm32f405/adc.o: HOST_CFLAGS_TARGET += -DAK_HOST_ADC_F4
$(HOST_OUT)/src/arch/at32f435/adc.o: HOST_CFLAGS_TARGET += -DAK_HOST_AT32 -DAK_HOST_ADC_AT32
$(HOST_OUT)/src/arch/at32f435/spi.o: HOST_CFLAGS_TARGET += -DAK_HOST_AT32 -DAK_HOST_SPI_AT32

# And the I2C driver against the modelled bus in tests/host_i2c_model.c, for the
# same reason: its transaction sequences need a device that answers, and there
# is no sensor on the bench board. The flag is on this object alone.
$(HOST_OUT)/src/arch/stm32f405/i2c.o: HOST_CFLAGS_TARGET += -DAK_HOST_I2C
$(HOST_OUT)/src/arch/at32f435/clk.o: HOST_CFLAGS_TARGET += -DAK_HOST_AT32
$(HOST_OUT)/src/arch/at32f435/gpio.o: HOST_CFLAGS_TARGET += -DAK_HOST_AT32
$(HOST_OUT)/src/arch/at32f435/systick.o: HOST_CFLAGS_TARGET += -DAK_HOST_AT32
$(HOST_OUT)/src/arch/at32f435/usart.o: HOST_CFLAGS_TARGET += -DAK_HOST_AT32
$(HOST_OUT)/src/arch/at32f435/flash.o: HOST_CFLAGS_TARGET += -DAK_HOST_AT32 -DAK_HOST_FLASH_AT32
# And the third port's I2C gets the same treatment as the F405's: the driver is
# compiled against a modelled bus (tests/host_i2c_model_at32.c), because the
# sequences on this part are not the F405's - the address, the direction and the
# byte count are one register write, and the acknowledge is the peripheral's.
$(HOST_OUT)/src/arch/at32f435/i2c.o: HOST_CFLAGS_TARGET += -DAK_HOST_AT32 -DAK_HOST_I2C_AT32
$(HOST_OUT)/src/arch/at32f435/output.o: HOST_CFLAGS_TARGET += -DAK_HOST_AT32
$(HOST_OUT)/src/arch/at32f435/usb.o: HOST_CFLAGS_TARGET += -DAK_HOST_AT32
# And both USB drivers against a modelled FIFO (tests/host_usb_model.c), because
# the one thing a mapped register block cannot give them is a *queue*: reading
# FIFO 0 is what drops the core's entry, and a setup packet is two words while a
# data packet is as many as its length needs. The old test gave both reads the
# same word, which is not a packet any host sends.
$(HOST_OUT)/src/arch/stm32f405/usb.o: HOST_CFLAGS_TARGET += -DAK_HOST_USB
$(HOST_OUT)/src/arch/at32f435/usb.o: HOST_CFLAGS_TARGET += -DAK_HOST_USB
# And the system file, which is where the console's `dfu` command's jump lives.
# It is in the host build because the *board* calls it, and a board that is
# linked into the test binary has to be able to resolve what it calls.
$(HOST_OUT)/src/arch/at32f435/system.o: HOST_CFLAGS_TARGET += -DAK_HOST_AT32
# Both system files are compiled for the host with this, because their two
# functions are assembly that has nothing to operate on in a process: no AIRCR
# register, and no ROM at 0x1FFF0000. The flag is the seam; on the target the
# guard is not defined and the real code is what is built.
$(HOST_OUT)/src/arch/stm32f405/system.o: HOST_CFLAGS_TARGET += -DAK_HOST_SYSTEM
$(HOST_OUT)/src/arch/at32f435/system.o: HOST_CFLAGS_TARGET += -DAK_HOST_SYSTEM
$(HOST_OUT)/tests/test_arch_at32.o: HOST_CFLAGS_TARGET += -Isrc/arch \
    -Isrc/arch/at32f435 -Isrc/boards/AERIALKIT_GHF435 -DAK_HOST_AT32

# The ESP32's outputs, for the host, against the stand-in IDF headers in
# tests/idf-stub. Two objects get that include path and nothing else does, so a
# core file cannot reach an `esp_*` declaration by accident - the rule `make
# port` exists to keep. The test gets the board's own header too, because the
# pins it drives are the board's to name.
$(HOST_OUT)/src/arch/esp32/output.o: HOST_CFLAGS_TARGET += \
    -Isrc/arch/esp32 -Itests/idf-stub -Wno-conversion
# -Wno-conversion is for IDF's own bitfields: `rmt_symbol_word_t` packs two
# 15-bit durations and two 1-bit levels, so every symbol the port builds is a
# narrowing assignment and GCC says so six times. IDF does not build with
# -Wconversion and this object is compiled here to be *run*, not linted - the
# real IDF build is what catches the port drifting from these types.
$(HOST_OUT)/tests/test_arch_esp32_output.o: HOST_CFLAGS_TARGET += \
    -Isrc/arch/esp32 -Isrc/boards/ESP32DEV -Itests/idf-stub
# The model is the other half of that: it *defines* the calls, so it needs the
# same declarations the port compiled against, or the two could disagree about
# a signature and the linker would be the one to notice.
$(HOST_OUT)/tests/host_esp32_output_model.o: HOST_CFLAGS_TARGET += -Itests/idf-stub

# The ESP32's sensor buses, on the same footing: the two port files and the
# test compile against the stand-in IDF headers, and the model is the other
# half of the same declarations.
# And both of the ESP32's sensor buses get a seam of their own, for the one
# thing their models cannot do: make the *peripheral* refuse rather than the
# part stay quiet. `bus_state` is one attempt per boot and never retried, so
# the host build needs a way to ask again.
$(HOST_OUT)/src/arch/esp32/spi.o: HOST_CFLAGS_TARGET += -Isrc/arch/esp32 -Itests/idf-stub -DAK_HOST_SPI_ESP
$(HOST_OUT)/src/arch/esp32/i2c.o: HOST_CFLAGS_TARGET += -Isrc/arch/esp32 -Itests/idf-stub -DAK_HOST_I2C_ESP
$(HOST_OUT)/tests/test_arch_esp32_bus.o: HOST_CFLAGS_TARGET += \
    -Isrc/arch/esp32 -Itests/idf-stub
$(HOST_OUT)/tests/host_esp32_bus_model.o: HOST_CFLAGS_TARGET += -Itests/idf-stub
$(HOST_OUT)/src/arch/esp32/uart.o: HOST_CFLAGS_TARGET += -Isrc/arch/esp32 -Itests/idf-stub
$(HOST_OUT)/tests/test_arch_esp32_uart.o: HOST_CFLAGS_TARGET += -Isrc/arch/esp32 -Itests/idf-stub
$(HOST_OUT)/tests/host_esp32_uart_model.o: HOST_CFLAGS_TARGET += -Itests/idf-stub
$(HOST_OUT)/src/arch/esp32/adc.o: HOST_CFLAGS_TARGET += \
    -Isrc/arch/esp32 -Itests/idf-stub -DAK_HOST_ADC -DCONFIG_IDF_TARGET_ESP32=1
# The target define is what the port's per-chip branches key on - the ADC's
# calibration scheme is the first of them - and the host build has to say which
# chip it is standing in for, because nothing includes the stub's sdkconfig.h.
$(HOST_OUT)/tests/test_arch_esp32_adc.o: HOST_CFLAGS_TARGET += \
    -Isrc/arch/esp32 -Isrc/boards/ESP32DEV -Itests/idf-stub -DAK_HOST_ADC
$(HOST_OUT)/tests/host_esp32_adc_model.o: HOST_CFLAGS_TARGET += -Itests/idf-stub

# And the same port file again, as a chip whose converter calibrates with a
# *curve* - which is every ESP32 after the original one, and the branch of
# adc.c that the ESP32 build never compiles and the S2/S3/C3 builds compile
# without being able to run. The entry points are renamed so both schemes can
# be in one test binary, the way the two fault records are; the test drives
# this one directly.
ADC_CURVE_RENAMES := -Dak_esp_adc_init=ak_esp_adc_curve_init \
    -Dak_esp_adc_mv=ak_esp_adc_curve_mv \
    -Dak_esp_adc_report=ak_esp_adc_curve_report \
    -Dak_esp_adc_forget_for_host=ak_esp_adc_curve_forget_for_host

# This is the fourth rule in the file that produces an object, and it was the
# only one of the four without a flags stamp - found by counting the objects a
# build makes rather than trusting that they had all been rebuilt. `make -n host`
# names 140 and recompiles 139, and the difference was this file, last compiled
# on 2026-09-21 and still in the tests binary three days and several revisions
# later (trap 190).
#
# The stamp alone would not have finished the job: five of this rule's flags
# were written into its own recipe, where no stamp can see them, so editing
# `-DCONFIG_IDF_TARGET_ESP32` or the renames would still have rebuilt nothing.
# They are a target-specific assignment now, which puts them in the key *and*
# makes this rule an ordinary object rule with no special case left in it - the
# defines and the include paths are the same mechanism the other forty-nine
# objects use, rather than a fifth place for flags to live.
$(HOST_OUT)/src/arch/esp32/adc_curve.o: HOST_CFLAGS_TARGET += \
    -Isrc/arch/esp32 -Itests/idf-stub -DAK_HOST_ADC \
    -DCONFIG_IDF_TARGET_ESP32=0 $(ADC_CURVE_RENAMES)
$(HOST_OUT)/src/arch/esp32/adc_curve.o: src/arch/esp32/adc.c \
    $(HOST_OUT)/.cflags.d/src/arch/esp32/adc_curve.o.flags
	@mkdir -p $(@D)
	$(HOST_CC) $(HOST_CFLAGS) $(HOST_CFLAGS_TARGET) -MMD -MP -c $< -o $@


# And the ESP32's fault record, LED and reset, which are the rest of what a
# person at a bench reads first: why the chip restarted, and whether it is
# blinking. The record's entry points are renamed for this build - one firmware
# has one `ak_fault`, and the tests binary already holds the ARM port's - and
# `ak_arch_reset` is renamed with them so the two ports' system files can both
# be in it.
$(HOST_OUT)/src/arch/esp32/fault.o: HOST_CFLAGS_TARGET += \
    -Isrc/arch/esp32 -Isrc/boards/ESP32DEV -Itests/idf-stub \
    -Dak_fault=ak_esp_fault -Dak_fault_present=ak_esp_fault_present \
    -Dak_fault_clear=ak_esp_fault_clear
$(HOST_OUT)/src/arch/esp32/led.o: HOST_CFLAGS_TARGET += \
    -Isrc/arch/esp32 -Isrc/boards/ESP32DEV -Itests/idf-stub
$(HOST_OUT)/src/arch/esp32/system.o: HOST_CFLAGS_TARGET += \
    -Isrc/arch/esp32 -Isrc/boards/ESP32DEV -Itests/idf-stub \
    -Dak_arch_reset=ak_esp_arch_reset
$(HOST_OUT)/tests/test_arch_esp32_fault.o: HOST_CFLAGS_TARGET += \
    -Isrc/arch/esp32 -Isrc/boards/ESP32DEV -Itests/idf-stub
$(HOST_OUT)/tests/host_esp32_system_model.o: HOST_CFLAGS_TARGET += -Itests/idf-stub

# The ESP32's network, as the Wi-Fi build - which is the one nothing here can
# run end to end, because QEMU's ESP32 has no radio. `-DCONFIG_AK_NET_WIFI` is
# the Kconfig choice, which is what the generated sdkconfig.h would define; the
# model behind the stand-in headers is tests/host_esp32_net_model.c, and it
# needs the same declarations the port compiled against.
$(HOST_OUT)/src/arch/esp32/net.o: HOST_CFLAGS_TARGET += \
    -Isrc/arch/esp32 -Itests/idf-stub -DCONFIG_AK_NET_WIFI=1 -Wno-conversion \
    -DAK_HOST_NET
# -Wno-conversion for the same reason the outputs' rule has it: lwip's recv and
# send return ssize_t and take size_t, and this port assigns the count to an int
# the way IDF's own examples do. The object is compiled here to be *run*; the
# ESP32 build is what checks it against IDF's real headers.
$(HOST_OUT)/tests/test_arch_esp32_net.o: HOST_CFLAGS_TARGET += \
    -Isrc/arch/esp32 -Itests/idf-stub -DCONFIG_AK_NET_WIFI=1
$(HOST_OUT)/tests/host_esp32_net_model.o: HOST_CFLAGS_TARGET += -Itests/idf-stub

# The host's clock, and the reason a delay has to mean something else there:
# the tick is a variable no timer moves, so `ak_delay_ms()` - which a port's own
# bus driver calls for the datasheet's waits - would wait for ever. The flag is
# on these two objects and nothing else; the simulator links its own stubs and
# no target build sees it.
$(HOST_OUT)/src/core/time.o: HOST_CFLAGS_TARGET += -DAK_HOST_TICK
$(HOST_OUT)/src/arch/stm32f405/systick.o: HOST_CFLAGS_TARGET += -DAK_HOST_TICK

# make tracks files, not flags. An object compiled before -DPRODUCT changed is
# still "up to date" and quietly wrong, which costs an hour the first time it
# happens. Writing the flags to a file the objects depend on turns that into a
# rebuild.
#
# `$(HOST_CFLAGS_KEYED)` and not `$(HOST_CFLAGS)`, and the difference is the
# whole point: `$(HOST_CFLAGS)` carries `-DAK_STAMP_VALUE="$(STAMP)"`, which is a
# clock reading, so writing *that* into a file the objects depend on makes every
# object permanently out of date rather than permanently correct - the same
# mistake as before, with the sign flipped, and it was every object the build
# makes, on every run (trap 188). What belongs in this file is the flags that are a function of the
# inputs, and the time of day is not one of them.
# One stamp per object, and it is per object because it has to be. Forty-nine
# lines add to `$(HOST_CFLAGS_TARGET)` for one object each, so a single shared
# file would hold whichever object's flags make considered first; every other
# object would compare against another object's flags, rewrite the file, and the
# build would never converge - trap 188 arriving by a different route, and the
# reason trap 189 sat open rather than being a one-line change.
#
# What makes the per-object part cheap is make doing it: a target-specific
# assignment applies to the target *and to its prerequisites*, so the stamp
# written while building `clk.o`'s prerequisite carries `clk.o`'s flags and no
# other object's. Nothing has to be listed twice, which matters more than the
# arithmetic - the failure mode in trap 190 was a rule that looked like its
# three siblings and was not, and a fix needing a second list of the same
# targets would have the same shape.
#
# `.o.flags` and not `.o`, so that no stamp file can be matched by the object
# pattern rule above it. The name is the only thing keeping those two rules
# apart, and relying on make preferring the shorter stem would be relying on a
# tie-break to carry meaning.
#
# `.cflags.d/` and not `.cflags/`, because `.cflags` is a *file* in every build
# tree that predates this change - the single shared stamp this replaces - and a
# path cannot be both. It failed loudly rather than quietly, which is worth
# saying: `mkdir: cannot create directory 'build-host/.cflags': Not a
# directory`, exit 2, on the first object. A name that collided with an artifact
# of the previous scheme would otherwise have turned into a clean-build-only
# Makefile, working for whoever ran `make clean` and failing for everyone else.
#
# The cost is one `cmp` per object per build, and it is not nothing: 140 of them
# take a no-op `make host` from 0.20s to 0.46s, measured here on 2026-09-24 by
# running the same tree with the recipe's body reduced to its `mkdir` - three
# runs each, 0.20/0.21/0.21 against 0.45/0.46/0.46. That is 0.25s on the inner
# loop, paid to make a stale object impossible rather than unlikely. The `cmp`
# and not a plain redirect, because touching a file whose contents have not
# changed is exactly the defect this family is about, and a version of this that
# avoided the fork by reading the file with `$(file <...)` was considered and
# left alone: 0.25s is cheaper than a second mechanism to be wrong about.
$(HOST_OUT)/.cflags.d/%.o.flags: FORCE
	@mkdir -p $(@D)
	@printf '%s\n' '$(HOST_CFLAGS_KEYED) $(HOST_CFLAGS_TARGET)' | cmp -s - $@ || \
	    printf '%s\n' '$(HOST_CFLAGS_KEYED) $(HOST_CFLAGS_TARGET)' > $@

# Without this line the stamps are deleted as fast as they are made, and the
# build then compiles 139 objects on every run for ever - which is the same
# observable as trap 188, from an unrelated cause, and it took a second
# measurement to tell them apart. Make deletes a file it considers
# *intermediate*: made by a pattern rule, and not named in the makefile as a
# target or a prerequisite. These are named as prerequisites, but only as
# `$(HOST_OUT)/.cflags.d/%.o.flags` - a pattern - so no *particular* stamp is
# named anywhere, and every one of them is intermediate by that definition. The
# single stamp that did survive was `adc_curve.o`'s, the only one written out
# literally, in the explicit rule below; that it was the exception is what
# identified the cause.
#
# The pattern form was tried first and does not do it - `.SECONDARY:
# $(HOST_OUT)/.cflags.d/%.o.flags` left the behaviour exactly as it was, 139
# objects per run and one stamp on disk after three runs. The bare form fixes
# it. That is written down as an observation and not as an explanation: what the
# manual promises for a *pattern* in `.SECONDARY` was not established here, and
# the difference between the two forms is not something this file should be
# guessing about.
#
# `.SECONDARY` and not `.PRECIOUS`: both stop the deletion, but they say
# different things about why. These files are wanted afterwards - the next
# build compares against them - which is exactly what secondary means.
.SECONDARY:

FORCE:

$(OUT)/$(PRODUCT).elf: $(OBJS) $(LDSCRIPT)
	@mkdir -p $(@D)
	$(CC) $(CFLAGS) $(OBJS) $(LDFLAGS) $(LIBS) -o $@
	@printf 'aerialkit: %s board=%s rev=%s stamp=%s\n' "$(PRODUCT)" "$(BOARD)" "$(REV)" "$(STAMP)"

$(OUT)/%.bin: $(OUT)/%.elf
	$(OBJCOPY) -O binary $< $@

$(OUT)/%.hex: $(OUT)/%.elf
	$(OBJCOPY) -O ihex $< $@

$(OUT)/%.o: %.c $(OUT)/.cflags
	@mkdir -p $(@D)
	$(CC) $(CFLAGS) -MMD -MP -c $< -o $@

$(OUT)/.cflags: FORCE
	@mkdir -p $(@D)
	@printf '%s\n' '$(CFLAGS_KEYED)' | cmp -s - $@ || printf '%s\n' '$(CFLAGS_KEYED)' > $@

size: $(OUT)/$(PRODUCT).elf
	@$(SIZE) $<

# Structural checks that need no board. See scripts/check-image.sh.
# --- portability: does the portable code compile anywhere else? --------------
#
# src/core, src/core/flight and src/core/sensors are supposed to be free of the
# MCU: no arch header, no libc, no assumption about the instruction set. The
# cheapest way to keep that true is to compile them for another architecture
# now and then - and the goal's second target is an ESP32, so the architectures
# to try are the two Espressif ones, xtensa and riscv32.
#
# No link and no startup: this is a source-level check. It says nothing about
# whether anything runs on an ESP32, and it is exactly the check that catches a
# stray arch include in the flight core.
PORT_SRCS  := $(wildcard src/core/*.c) $(wildcard src/core/flight/*.c) \
              $(wildcard src/core/sensors/*.c)
PORT_OUT   ?= build-port
PORT_CFLAGS ?= -std=c11 -O2 -ffreestanding -Wall -Wextra -Wshadow -Wconversion \
               -Wdouble-promotion -c -Isrc/core -Isrc/core/flight \
               -Isrc/core/sensors -DAK_PRODUCT=$(PRODUCT) -DAK_BOARD=portable \
               -DAK_REV=$(REV) -DAK_STAMP_VALUE="$(STAMP)"

port:
	@./scripts/check-portable.sh "$(TOOLCHAIN_ROOT)" "$(PORT_OUT)" $(PORT_SRCS)

# The shared toolchains live one directory above this one, inside the
# workspace. The default was `../fc-firmware-workspace/toolchain` while
# aerialkit was a *sibling* of the workspace, and it kept working as a string
# after the tree moved in - it just stopped finding anything, and the target
# printed "not found; skipping" and passed. A path that is wrong in a way that
# passes is the one worth naming.
TOOLCHAIN_ROOT ?= ../toolchain

# The structural checks are the *part's*, not the image's: the stack top, the
# length of the vector table and which handler each named entry points at all
# differ between the F405 and the AT32. They live in scripts/image-facts/
# $(PART).txt, one file per part, and the Makefile points at the one for the
# part being built so that `make check` on either target means the same thing.
#
# `check` depends on the image, so it is built with whatever BOARD/ARCH/PART
# this invocation was given - which is why every target.conf runs its whole
# build as *one* make command. Called by hand with the variables left off, this
# target rebuilds the image for the default part and then measures it against
# $(FACTS) for that part; that is the same mistake in a different place.
FACTS ?= scripts/image-facts/$(PART).txt

check: $(OUT)/$(PRODUCT).bin
	@# The one thing this check knows about *which* image was asked for: a
	@# build with a diagnostic LED flag on is checked for carrying the
	@# instrument, and every other build for not carrying it. The flags are in
	@# EXTRA_CFLAGS, which is what both target.confs use to switch them on -
	@# see the `AK_BOOT_STAGE` block in targets/aerialkit-f405/target.conf and
	@# the wing's own target.conf, which publishes the same instrument, and
	@# the `AK_USB_TRACE` block beside it.
	@#
	@# Two flags, two names, because the check's answer differs: a tell-tale
	@# has to have the boot path behind it and the trace does not. Both are
	@# read off the same blink plan, so a build with either is a diagnostic
	@# image as far as "does this carry the plan" is concerned.
	@./scripts/check-image.sh $(OUT)/$(PRODUCT).bin $(FACTS) $(OUT)/$(PRODUCT).elf \
	    $(if $(findstring AK_BOOT_STAGE=1,$(EXTRA_CFLAGS)),--telltale,) \
	    $(if $(findstring AK_USB_TRACE=1,$(EXTRA_CFLAGS)),--usb-trace,)

# --- what the image costs, and the whole of CI in one command ---------------
#
# `make size` is the toolchain's `size`, which answers a different question than
# the one that decides whether a feature fits: it counts `.data` twice - once as
# flash, once as RAM - and leaves out the alignment padding the linker reserved
# between sections. tools/image_report.py reads the ELF's program headers
# instead, and its numbers are the linker's own `--print-memory-usage` byte for
# byte; `make image-report-check` is what holds it to that.
#
# `make image-report` measures the one profile this invocation was given, the
# same way `make check` checks the one part this invocation was given. The whole
# matrix is `make ci`, because measuring five images means five builds and that
# is not something a target named after one image should do without saying so.
READELF := $(CROSS)readelf
NM      := $(CROSS)nm

image-report: $(OUT)/$(PRODUCT).elf
	@python3 tools/image_report.py --readelf $(READELF) --nm $(NM) \
	    $(PRODUCT)=$(OUT)/$(PRODUCT).elf

image-report-check:
	@python3 tools/image_report_check.py

# B5's check, on its own: the board manifests against each other, and the
# validator against boards whose answer is known. It needs no compiler and no
# cross toolchain - it is a reader over `src/boards/*/board.h`, which is what it
# has to be, because neither the ESP-IDF nor `arm-none-eabi-gcc` is installed on
# this host and a compiled per-board check would report "not run" for all six
# boards.
#
#   make boards           # the six real boards; exit 1 if a pad is newly shared
#   make boards PINS=1    # and every pin every board claims
#   make boards-check     # the validator itself, against fixtures
boards:
	@python3 tools/board_resources.py $(if $(PINS),--show-pins,)

boards-check:
	@python3 tools/board_resources.py
	@python3 tools/board_resources_check.py

# The cross-repository contract (docs/31-contract.md). `contract-check` is the
# one that runs everywhere: it reads the committed vectors and recomputes every
# byte with an implementation that shares no code with the firmware that
# produced them. `contract-vectors` is the deliberate act of regenerating the
# file, and it needs the other repository checked out beside this one - the
# NED->ENU group is that repository's statement to make, not this one's:
#
#   make contract-check
#   make contract-vectors REPO=~/Projects/aerial-kit
# The records (AERIAL-KIT-GOAL-PROGRESS.md and hosts/nas/agents/), on their
# own. No compiler, no cross toolchain and no board - it reads the repository's
# own history, so a records edit can be checked before it is committed. The
# port-time form is `--base-dir`, for the two trees that share no ancestry:
#
#   make records-check
#   make records-check ARGS="--base-dir /path/to/the/other/tree"
records-check:
	@python3 tools/records_check.py $(ARGS)

contract-check:
	@python3 tools/contract_check.py

contract-vectors: $(HOST_OUT)/aerialkit-contract-vectors
	@python3 tools/contract_vectors.py --binary $(HOST_OUT)/aerialkit-contract-vectors $(if $(REPO),--repo $(REPO),)

# The contract's vector producer. It links the flight objects for the blackbox
# encoder and ak_proto.o for the telemetry frame, which is the same pair
# tools/akproto_sim.c links - it has to, or the vectors would be about a
# different build of the protocol than the one the simulator speaks.
$(HOST_OUT)/aerialkit-contract-vectors: $(FLIGHT_OBJS) $(SUPPORT_OBJS) \
                                        $(HOST_OUT)/src/core/ak_proto.o \
                                        $(HOST_OUT)/tools/contract_vectors.o
	@mkdir -p $(@D)
	$(HOST_CC) $(HOST_CFLAGS) $(FLIGHT_OBJS) $(SUPPORT_OBJS) \
	    $(HOST_OUT)/src/core/ak_proto.o $(HOST_OUT)/tools/contract_vectors.o \
	    -lm -o $@

# The control core as a shared library, for a program that is not this firmware
# (tools/ak_control.h, docs/32-control-in-the-loop.md).
#
# `control-check` is the one that runs everywhere: it needs no compiler of its
# own beyond the library and no numpy - it loads the library, compares the C
# side's own description of every struct against the binding's, and checks the
# configuration identity the contract asks for. `control-experiment` is the
# closed loop, and it needs the Python simulation, which lives in the other
# repository:
#
#   make control-check
#   make control-experiment REPO=~/Projects/aerial-kit
control-check: $(CONTROL_LIB)
	@$(CONTROL_LIB_ENV) AK_CONTROL_LIB=$(CONTROL_LIB) AK_REV='$(REV)' python3 tools/akcontrol_check.py $(if $(REPO),--repo $(REPO),)

control-experiment: $(CONTROL_LIB)
	@$(CONTROL_LIB_ENV) AK_CONTROL_LIB=$(CONTROL_LIB) python3 tools/akcontrol_experiment.py $(if $(REPO),--repo $(REPO),) $(ARGS)

# `control-experiment` cannot run where it is checked - it needs numpy and the
# sibling checkout - so without this its syntax would go untested until someone
# who can run it tried to. This compiles every tool without importing any of
# them, which is all that can be asked of a file whose dependencies are
# elsewhere. `python3 -m py_compile` writes __pycache__; the rm keeps a check
# from leaving anything behind in a tree that is later diffed.
python-syntax:
	@python3 -m py_compile tools/*.py && rc=0 || rc=1; \
	rm -rf tools/__pycache__; \
	if [ $$rc -eq 0 ]; then echo "python-syntax: all tools compile"; \
	else echo "python-syntax: FAILED" >&2; fi; \
	exit $$rc

# The entry point. Everything this host can run, in the order it has to run,
# with a manifest of the revision and toolchain written before anything is
# built. It is what a pull request runs (see .github/workflows/aerialkit.yml)
# and what a person runs before saying a change is finished.
#
# Nothing fast about it: the profile stage builds five images. That is the point
# of it - the per-profile numbers are the ones that were never measured, because
# measuring them was five commands nobody ran together.
#
#   make ci                        # everything, into ci-report/
#   make ci CI_ARGS=--only test    # one stage
#   make ci CI_ARGS=--list         # what the stages are
CI_OUT  ?= ci-report
CI_ARGS ?=

ci:
	@./scripts/ci.sh --out $(CI_OUT) $(CI_ARGS)

# `clean` used to be `rm -rf $(OUT) $(HOST_OUT) $(CI_OUT)`, and the three
# directories it left behind were the three that build the same objects with
# different flags: `build-asan` for `sanitize`, `build-cov` for `coverage`,
# `build-stack` for `stack-check`. Each is a full copy of the tree's objects, so
# each is a copy that can go stale on its own - and unlike `build-host`, nothing
# notices, because a stale object in an out-of-the-way directory is only read by
# the one stage that directory serves.
#
# Measured on 2026-09-21, which is why this line changed: `sanitize` reported
# `AddressSanitizer: unknown-crash src/core/flight/ak_flight.c:229 in
# ak_flight_init`, "0 bytes after global variable 'flight' ... of size 504",
# which reads exactly like a struct that outgrew a buffer. It was not.
# `build-asan/pic/tools/ak_control.o` was dated 2026-09-20 08:37:40 - compiled
# when `ak_flight_t` was 504 bytes - while `build-asan/pic/src/core/flight/
# ak_flight.o` was dated 2026-09-21 13:05:32, compiled after the struct grew.
# One binary, two revisions of one header, and the runtime diagnosed the
# disagreement as a defect in the sources. `make clean` is the command that was
# supposed to prevent it and did not reach the directory.
clean:
	rm -rf $(OUT) $(HOST_OUT) $(SAN_OUT) $(COV_OUT) $(CI_OUT) build-stack build-port

-include $(DEPS) $(HOST_DEPS) $(CONTROL_DEPS)

.PHONY: all host test proto-test window-test net-window-test fuzz cross-check coverage command-check firmware-proto-check stack-check configurator-check sanitize sil trace port size check image-report image-report-check ci clean oracle-link config-policy config-recycle timing boards boards-check \
     contract-check contract-vectors FORCE
