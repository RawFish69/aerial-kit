> Historical notes from the full development tree. Current public targets and commands are in the [firmware guide](../../docs/flight-controller-firmware.md). Host C tests and simulator harnesses referenced below are retained outside this public source tree.

# AerialKit - GPS

```text
USART3 PB11 (RX) -> interrupt -> ring buffer -> UBX parser -> fix
       PB10 (TX)   <- UBX-CFG-VALSET: message rates, NMEA off, rate keys
```

## Why UBX, and what is parsed

u-blox modules speak UBX, and NAV-PVT is the message that carries everything a
navigation loop wants in one frame: fix type, satellites, position, altitude,
ground speed, course, and the accuracy figures that say how much to trust them.
One frame every 200 ms at 5 Hz, rather than three messages to correlate.

Everything is kept in the integer unit the protocol uses - 1e-7 degrees,
millimetres, millimetres per second, 1e-5 degrees of course - so nothing is lost
to a float round trip on the way in, and the console formatter (which has no
floating point on purpose) prints all of it:

```text
ak> gps
gps:       41230 bytes, 401 messages, 401 NAV-PVT, 0 bad checksums, 0 bad lengths, 0 other
fix:       type 3 (ok), 11 satellites, pdop 1.23, hacc 1500 mm
position:  52.1234567, 4.9876543
altitude:  1234000 mm msl
motion:    3000 mm/s, course 270.12345 deg
fixes:     401, valid now: yes
```

## What is verified, and how

Two kinds of check, and the difference between them is the honest part:

- **The framing is confirmed against a captured frame.** INAV's unit test
  carries a real UBX configuration message with its bytes spelled out
  (`B5 62 06 8A 09 00 01 01 00 00 25 00 31 10 01 02 A7`). Feeding those bytes
  through this parser is a check on the sync pair, the little-endian length and
  the Fletcher-style checksum that came from somewhere other than this code, and
  it is in the test as a literal.
- **The NAV-PVT decode is pinned to the upstream layout.** The test builds its
  payload from `tests/ubx_reference.h` - INAV's `ubx_nav_pvt` copied field for
  field, in the order the struct declares - and `_Static_assert`s that every
  offset the parser reads equals `offsetof()` of that struct. The parsed values
  are then checked: fix type, satellites, position at 1e-7 degrees including
  negative (southern and western) values, altitude, accuracy, DOP, speed,
  course, and the arrival timestamp. `make test` also flies the position through
  the simulator and reads it back out of the firmware's own boot line, so the
  frame-to-home path is checked end to end (see
  [18-software-in-the-loop.md](18-software-in-the-loop.md)).

  What that still does not prove is that INAV's struct matches what a module
  puts on the wire. It is a second reading of the same published interface
  description, not a capture from a receiver, and a receiver is the only thing
  that can settle it. What it does prove is that this repository has one answer
  to the question instead of three.

Also covered: a corrupted frame is rejected by the checksum, an over-long frame
is counted rather than stored, a NAV-PVT of the wrong length is counted
separately (a receiver speaking a version we do not know is not noise), and a
frame that stops half way is abandoned after 10 ms of silence - without that,
the parser would swallow the beginning of the next frame and one dropped byte
would cost two frames.

## Every offset was two bytes too high

The constants the parser read from were `fix type 22, flags 23, satellites 25,
longitude 26, latitude 30, altitude 38, accuracy 42, speed 62, course 66, DOP
78`. Every one of them is two bytes high: a u-blox NAV-PVT payload has the fix
type at 20, and the rest follow from the interface description's field order.
With these numbers a real module produces frames that pass the length check,
pass the checksum, and decode into nonsense - position, altitude and course all
wrong, and a fix type that is usually zero, so `ak_gps_fix_valid` says no and
nothing downstream ever sees a fix at all.

Nothing caught it for a whole milestone, and the reason is worth keeping:
the unit test built its payload from the same constants the parser read it with,
and the simulator built its frames the same way. Three pieces of code agreed
with each other and disagreed with every receiver in the world. The document
that should have caught it *claimed the cross-check had been done*.

The fix is three things, and only the first is a number:

1. The offsets are right, and they moved into `ak_gps.h` where they are visible
   rather than buried in the parser.
2. The test builds its frames from `tests/ubx_reference.h`, which is INAV's own
   struct - a piece of upstream the parser cannot drift away from - and checks
   the constants against `offsetof()` of it at compile time.
3. The simulator builds its frames from the same reference, and the flight
   check reads the position back out of the firmware's console. Both of those
   fail if the parser is wrong by one byte on either side of the layout.

Restoring the old constants now fails the build with ten static assertions, and
moving a single offset inside the parser fails the flight check in
`make test`. That was the point of doing it this way rather than correcting the
numbers.

## What is not

- **No module has spoken to it.** The baud rate is 9600, the u-blox factory
  default; a module already configured to 115200 by another flight controller
  would need that changed here, and the counters would show it as a stream of
  bad checksums.
- **The module is configured but not acknowledged.** The firmware asks for
  UBX NAV-PVT and switches the NMEA sentences off (see below), and it does not
  read the module's ACK - the evidence that the ask worked is that fixes start
  arriving, which is the evidence that matters anyway.
- **The constellation and the dynamic model are not set.** A module left to
  choose its own satellites is fine; a module told it is in a car is not, and
  `CFG_NAVSPG_DYNMODEL` is a key this firmware does not send yet.
- **Return-to-home is written and never flown.** The navigator consumes the fix
  ([14-navigation.md](14-navigation.md)), but `rth_enable` defaults to off and no
  aircraft has done it. (Since that was written the return has been flown in the
  loop many times - see [18-software-in-the-loop.md](18-software-in-the-loop.md)
  - and it is still a simulator until the board is on a bench.)
- ~~The accuracy figures - `hacc`, `pdop`, satellite count - are parsed, shown,
  and not yet used to decide anything.~~ **The satellite count and the fix type
  decide now**, and the paragraph below is what that cost.

## What the navigator will believe

A fix used to be worth navigating on if it was fresh, the receiver called it ok,
and it was 2D or better. Nothing else about it mattered: `hacc`, `pdop` and the
satellite count were printed on the console and ignored. That is the wrong way
round for the one thing those numbers are for, because a navigator that flies
to a position is only as right as the position - and a receiver with four
satellites in view can be tens of metres out.

INA V's rule is `fixType == 3D && numSat >= gpsMinSats`, with `gps_min_sats`
defaulting to **6** and ranging 5..10, and its reason is a property of the
receivers rather than of any firmware: *"some GPS receivers appeared to be very
inaccurate with low satellite count"*. AerialKit has that rule now, with the
same default and the same range, as `gps_min_sats`. The 3D half is for a
different reason: a 2D fix has no height, and the fused altitude takes its
absolute reference from the first fix the aircraft sees on the ground, so a 2D
fix could anchor the ground level to nothing.

What that changes is not the console, it is the *navigator*: `gps_valid` is
false for a fix it will not use, and a navigator that is already flying **holds**
rather than navigating on it - level, at the altitude it was told to hold, which
is the same answer it gives when the frames stop altogether
([14-navigation.md](14-navigation.md)). Measured in the loop
(`aerialkit-fw-sim 170 quadrth badfix`): twenty seconds into the return the
module's count drops to four satellites with a 3D fix it still calls ok; the
console says `usable:    no - 4 satellites, and the navigator wants 6`; the
aircraft holds 23.0 m for the whole twenty seconds, its motors never below 0.53
of a 0.55 hover - and it **drifts from 96 m to 176 m from home**, because with
no position worth using there is nothing to hold it in place but the wind.
Twenty seconds later the count is back and the return picks up where it left
off: it lands 2 m from home. The transcript is
[evidence/sil-quad-return-badfix.txt](evidence/sil-quad-return-badfix.txt).

What is *not* claimed: the sim's module reports its own count honestly, and a
receiver that lies about either number is not something any of this can see.

### And the fix that lies about where it is

The gate above is about the fix's *own* numbers. A fix can also be wrong while
every number on it looks perfect - the type is 3D, the count is eleven, the
accuracy figure is the same as it was a second ago, and the position is fifty
metres from the truth. Nothing in this firmware can refuse that one, and the
honest question is what it costs.

Both reference implementations are in the same position, which is worth
checking rather than assuming:

- **INAV** still carries `INAV_GPS_GLITCH_RADIUS` (2.5 m) and
  `INAV_GPS_GLITCH_ACCEL` (10 m/s²) in
  `src/main/navigation/navigation_pos_estimator_private.h` - and in the pinned
  9.1.0 checkout they are **used nowhere**: the position estimator that replaced
  the old navigation code handles a bad fix as an estimate-innovation question
  rather than with a radius test.
- **Betaflight's** GPS rescue (`src/main/flight/gps_rescue_multirotor.c`)
  sanity-checks the *sensor*: a fix that goes away, or fifteen seconds of low
  satellite count, ends the rescue. A fix that is present and wrong is not
  something it checks either.

So this firmware does what they do - it trusts a fix that says it is good - and
the defence is the one thing that is always there: the loop is closed. Measured
in the loop, with the position jumping **50 m north for two seconds** while the
aircraft is 20 m from home on its way back
([evidence/sil-quad-return-jump.txt](evidence/sil-quad-return-jump.txt)): the
return stops closing for those two seconds - the distance sits at 17-18 m
instead of falling - and then picks up where it left off and lands **2 m from
home**. A glitch along the path (the direction the aircraft is already going)
costs even less: the velocity command is saturated and does not change at all.

What that means for a real aircraft: a bad fix buys a pause and a few metres of
corrected course, not a lost aircraft. What would change it is a glitch that
*sustains* - which is the low-satellite and stale-fix case the gate above does
cover.

## Bench check

Wire a u-blox module's TX to PB11, power it where it can see sky, and `gps`
should show messages climbing at about five a second and a fix type of 3 with a
plausible position. Indoors it will show a fix type of 0 or 2 and no satellites,
which is the correct answer to "can it see the sky" - the counters tell "wired
wrong" (no bytes) from "no sky" (bytes, no fix) from "wrong baud" (bytes, bad
checksums).

## Configuration: asking a stock module to speak UBX

A u-blox out of the box speaks NMEA at 9600 baud, which this parser cannot read,
so a perfectly good module looks like a stream of noise until it is asked. At
boot - and again after two seconds, up to four times, because a module still
booting will not hear the first ask - the firmware sends one UBX-CFG-VALSET
frame that:

- asks for **NAV-PVT once per navigation solution** (the message this parser
  reads),
- switches **NMEA off** (GGA, GLL, GSA, RMC, VTG) and the UBX messages nobody
  reads (SAT, SIG, POSLLH, STATUS, VELNED, TIMEUTC), which at 9600 baud is the
  difference between a fix and a queue,
- sets the measurement rate to 200 ms.

The settings go into the module's RAM, not its battery-backed store, on purpose:
the firmware sends them at every boot, and a module that remembers a setting it
was given once is a module nobody can reason about later.

The frame layout is not guessed. INAV's unit test carries a real captured
VALSET frame from u-center with one key in it, and the builder's test compares
its bytes to that frame - header, version, layer byte, little-endian key,
trailing checksum and all - before checking anything else. The keys and values
come from INAV's `gps_ublox.h` and the sequence it sends at the same point in
its startup (see [03-attribution.md](03-attribution.md)).

**And what the module does with it is not observed at all** - there is no
acknowledgement in this design, and the fourth attempt is the last: a module that
was not listening is a module the aircraft flies without. What *is* now run on
this machine, since 2026-09-18, is the sending of it: `aerialkit-fw-sim 10
gpsquiet` is an aircraft whose module never answers, so the four frames are the
only thing on that wire, and the checks read them back - the four attempts and
no more, the UBX header, the 200 ms measurement rate and NAV-PVT set to once per
solution. Before that session, every other one had a module that was talking
from the first second, so the firmware's first question - "has it said anything
yet?" - always said yes and this path had never run. See
[18-software-in-the-loop.md](18-software-in-the-loop.md).
