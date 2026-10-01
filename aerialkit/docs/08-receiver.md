# AerialKit - the receiver

CRSF or SBUS in on their own UART, decoded into sticks, switches and a freshness
stamp:

```text
USART1 PA10 (RX) -> interrupt -> ring buffer -> CRSF or SBUS parser -> channels
        PA9  (TX)   <- CRSF telemetry out: attitude, pack, gps, mode
                                                             -> flight core
```

## Wiring and rate

| | |
| --- | --- |
| Port | USART1, alternate function 7 |
| Pins | PA10 receive, PA9 transmit |
| CRSF | 420000 baud, 8N1 |
| SBUS | 100000 baud, 8E2 - even parity, two stop bits |
| Console | still USART2 on PA2/PA3, still polled |

`rc_protocol` picks, and it applies immediately: both parsers exist from boot
and the board reconfigures its port. It is one setting to a pilot and two to
the port, because the two protocols do not even agree on the frame format.

**SBUS is inverted**, and that is a hardware fact rather than a firmware one. It
idles low and starts with a high bit - the opposite of a UART - and the STM32F4
cannot invert its own pins, because the part has no such bit (the F0, F3, F7,
G0, G4, H7 and L4 do). A board with an SBUS pad has a transistor or a gate on it
for exactly this reason; the WeAct dev board does not, so
`AK_BOARD_RC_INVERTER` is 0 and `rc` says so rather than letting a stream of
framing errors read as a wrong baud rate. Some receivers - most ELRS boards and
a few FrSky ones - also expose an un-inverted SBUS pad, which needs none of it.

## How bytes get in

The UART receives one byte at a time in an interrupt and pushes it into a ring
buffer. The main loop empties the ring and feeds the parser. Neither side waits
for the other, and a full ring drops the newest byte and counts it rather than
overwriting something the reader has not seen - a dropped byte costs one frame,
which the CRC throws away, and that is the cheapest possible failure.

The parser itself (`ak_crsf.c`) was written before this and is unchanged: it
checks the envelope, the CRC-8/DVB-S2, unpacks sixteen 11-bit channels, and
throws away a frame that stops half way after 5 ms of silence. All of that is
host-tested, and `tests/crsf_fixture.c` builds frames the way a receiver does
so the decoder is not checked against its own encoder.

The SBUS parser (`ak_sbus.c`) is the same shape with less to check. Twenty-five
bytes: a 0x0F header, twenty-two bytes of channels at eleven bits each, the
flags, and a 0x00 footer. There is no CRC, so a frame that is corrupt in the
middle is credible and the two fixed bytes are all there is - which is the
protocol's weakness, and the reason the flags byte is read rather than ignored:

| Flag | Meaning | What AerialKit does |
| --- | --- | --- |
| bit 2 | the receiver has lost frames | counts it; the channels are still the pilot's |
| bit 3 | the receiver's own failsafe is active | counts it, **does not** return the channels as a command, and does not refresh the link's timestamp |

That last row is the one worth having. A receiver that has lost its transmitter
keeps sending frames - with throttle down, or with the last position, or with
whatever it was configured to send, and there is no telling which from the
outside. The flight core detects a dead link by time, and a timeout cannot see
this case at all; it is the parser's job not to keep the link looking alive.

**And it is flown, in the version of that case that is worst.** The host tests
call the parser; the loop now calls it the way a receiver would, with the
failsafe configured as *hold the last position* - so the frames carry a
hovering throttle and centred sticks, which is indistinguishable from a good
command except for the flag. `aerialkit-fw-sim 20 quad rxfailsafe`
([evidence/sil-rx-failsafe.txt](evidence/sil-rx-failsafe.txt)): the transmitter
goes off, the receiver keeps sending, and the aircraft **stops its motors 236 ms
later** - with 450 frames carrying 0.55 of throttle having arrived in between.
The check is the difference between those two numbers. Disabling the parser's
failsafe test turns the same run into an aircraft that flies those frames for
nine seconds, which is how the check is known to measure the flag rather than
the timeout: the two cases are the same frames and the same clock, and only one
of them is a link.

And the whole of it - the parser, the two receivers on one port, and the line
settings that tell them apart - is 25 host checks: 23 on the frame and its
flags, 2 on the USART format bits, and one in the loop where the quadrotor
switches protocol from the console and then flies on it.

## What the console shows

```text
ak> rc
receiver:  crsf, 8412 bytes, 401 frames
crsf:      401 crc errors, 0 rejected
link:      framing
channels:  992 992 172 992 992 1811 172 172
sticks:    roll 0, pitch 0, yaw 0, throttle 0 per-mille
switches:  arm off, mode rate
uart:      0 bytes dropped by the receive buffer
```

And the same command on a board reading an SBUS receiver - from the quadrotor's
session in the loop, where the parameter is typed at the console at 200 ms:

```text
ak> rc
receiver:  sbus, 7875 bytes, 315 frames
sbus:      315 good, 0 failsafe, 0 with lost frames, 0 rejected
link:      framing
channels:  1401 992 1073 992 1811 1811 172 992
sticks:    roll 499, pitch 0, yaw 0, throttle 549 per-mille
switches:  arm on, mode angle
uart:      0 bytes dropped by the receive buffer
```

The counters are the useful part, because they distinguish the ways a receiver
can be wrong:

| What it looks like | What it means |
| --- | --- |
| `no frames yet`, 0 bytes | nothing is arriving: wiring, or the receiver is not powered |
| bytes climbing, `crc errors` climbing | something is arriving that is not CRSF at this rate - usually the baud rate is wrong |
| `rejected` climbing | the byte stream has no frame envelope in it at all |
| frames climbing, channels in range (172..1811) | the link is good |
| frames climbing, channels out of range | the framing is right and the scaling is not, which is a protocol mismatch rather than a wire |
| `sbus:` `failsafe` climbing | the receiver is up but has lost its transmitter: the aircraft will go to failsafe on the timeout even though bytes are arriving |
| `sbus:` `rejected` at every byte | SBUS selected and no inverter on the pin, or the wrong protocol |

## Bench checks

With a receiver wired and powered, `rc` should show frames climbing and the
channels moving with the sticks. Without one, a logic analyser or a second
UART can play a CRSF frame into PA10 - the fixture in the tests is a working
encoder to copy - and the same counters should move.

A receiver that is not there at all is not a hazard: no frames means no fresh
`last_update_ms`, which is a failsafe, which means zero throttle, and the
arming path additionally refuses to arm without a converged attitude estimate.

## The other direction: what the handset is told

CRSF is a two-way bus, and PA9 has been labelled "wired for telemetry later"
in the board file since it was written. The later is now: the firmware sends
the frames a handset and an ELRS/Crossfire receiver already understand, built
to the byte layout Betaflight writes (which is what the CRSF document says),
from the same state everything else uses.

| Frame | Rate | What it carries |
| --- | --- | --- |
| `0x1E` attitude | 10 Hz | roll, pitch and yaw from the estimator, radians x 10000 |
| `0x08` battery | 5 Hz | the pack voltage in tenths of a volt, and the percentage the configured thresholds imply |
| `0x02` gps | 2 Hz | position in degrees x 1e7, ground speed, course, altitude above msl, satellites |
| `0x21` flight mode | 1 Hz | `DISARM`, `ANGLE`, `ACRO`, `RTH`, `!FS!` |
| `0x29` device info | on a device ping | the flight controller's name, so a handset lists it |

Two of those are worth reading twice.

The **percentage** is an estimate: a straight line from the ceiling a cell can
be detected at (`cell_detect_v`) down to the critical threshold the pilot
configured. It is the same estimate Betaflight sends, and the voltage travels
in the same frame so a handset shows both - a percentage on its own would be a
guess wearing a number's clothes.

The **flight mode** is a wire format rather than this firmware's vocabulary: the
flight core's state is "on autopilot", and a transmitter's screen has room for
`RTH`. The mapping is in `ak_crsf_flight_mode()`, and the one thing it must not
do is disagree with the flight core about who is flying - a mode that says ACRO
while the navigator has the aircraft is worse than no mode at all.

**The device info frame is the handset's first question, and it is answered
once per ping.** A transmitter sends `0x28` and will not list the flight
controller until a `0x29` comes back; the pings are counted by the receiver and
each one is answered exactly once from the flight loop (`telemetry_service()`
in `src/core/main.c`), because an answer sent on every telemetry tick would
spend the link it is flying on saying the same thing again. The layout -
destination, origin, the name, the twelve bytes of serial number that are zero
here, the parameter count and the version - is Betaflight's
`crsfFrameDeviceInfo` (`upstream/betaflight-2026.6.1/src/main/telemetry/crsf.c`),
which is what a handset has actually been built against. The simulator now
sends two real pings up the wire while the quadrotor is climbing and decodes
the answers back ([evidence/sil-quad-return.txt](evidence/sil-quad-return.txt));
neither the "one answer per ping" nor the "same frame a handset expects" check
survives removing the answer from `main.c`, which is how they were checked.

What is *not* sent: current and mAh (no current sensor on any board here, and
the frame's zeros say so), link statistics (that is the receiver's own frame -
the one thing on this wire the flight controller does not own), and
parameter-over-CRSF (`0x2B`-`0x2D`), which is the handset's own configuration
menu. AerialKit's parameter table has its own protocol, and two ways to write
it is one way too many.

The frame *encoder* is checked on the host against the CRC-8/DVB-S2 catalogue
value, field by field, and by feeding every frame back into the firmware's own
receive parser - one protocol, two halves, checked against each other. The
content is checked in the simulator, where the handset's view is compared with
the aircraft that was flying: the pack it was holding, the attitude it had, the
fix the module sent, and `ANGLE` on the way out followed by `RTH` when the link
went. What has not happened is a real handset: no CRSF transmitter has ever
been wired to PA9, so the first bench session is where that turns into a fact -
`rc` prints the counters (`telemetry: N frames out`), and the handset either
lists the aircraft or does not.
Until the IMU driver exists, that estimate never converges, so this firmware
cannot arm even with a receiver attached and the switch on. That is worth
knowing before plugging anything in: it is safe by construction right now, and
it will not stay that way by accident - arming is the thing to test on purpose,
with the props off, when the IMU lands.
