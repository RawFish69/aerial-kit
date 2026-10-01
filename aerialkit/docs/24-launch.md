# AerialKit - the hand launch

Every other mode in this firmware can wait for a pilot to be holding the sticks.
A launch cannot. One hand is on the wing and the other is on the transmitter, and
the two seconds after the throw are the two seconds an aircraft with no altitude
needs the climb nobody is holding a stick for. `launch_channel` names a receiver
channel, and throwing that switch - with the aircraft armed, level and still -
hands the aircraft to a launch: **a climb attitude and a throttle, held until
somebody takes it back.**

```
ak> set launch_channel 8
launch_channel = 8
... arm the aircraft, throttle down ...
sim:   3001 ms  launch switch on - the aircraft is in a hand
launch: 18 degrees of climb at 70 per cent - a stick gives it back
state:     on autopilot
motors:    699 700 0 0 per-mille
... the throw ...
sim:   7001 ms  the pilot takes it: 20 m up, nose 18 deg
launch: over - a stick
state:     armed
motors:    599 599 0 0 per-mille
```

## What it does, and what it deliberately does not

The manoeuvre is a `ak_rc_command_t` handed to the flight core with `managed`
set - the same machinery a mission and a fence return use - with three numbers in
it, and all three are INAV's defaults, because INAV is the implementation that
flies this airframe today
([03-attribution.md](03-attribution.md)):

| Parameter | Default | Where it comes from |
| --- | --- | --- |
| `launch_throttle` | 0.70 | `nav_fw_launch_thr`, 1700 of 1000..2000 |
| `launch_climb_deg` | 18 | `nav_fw_launch_climb_angle`, "attitude of model, not climb slope" |
| `launch_timeout_s` | 5 | `nav_fw_launch_timeout`, milliseconds there and seconds here |

**And those three are read back out of INAV on every build** -
`tools/reference_defaults_check.py`, in `make test`, takes the default and the
range from INAV's own `settings.yaml`, checks that this aircraft's own CLI dump
(`projects/twin-wings/ghf435-inav/inav-postflash-default-config.txt`) holds the
same default, and compares the arithmetic with the firmware's parameter table:
`(1700 - 1000) / (2000 - 1000) = 0.70`, 18 degrees, and 5000 ms as 5 seconds.
It is the same reason the arming gate's 25 degrees is checked against both
references' `small_angle` (`DEFAULT_SMALL_ANGLE` in Betaflight's `imu.c`, whose
`#ifdef USE_RACE_PRO` branch says 180 - which is why the check reads the
ordinary branch rather than the first `#define` it finds). A number borrowed
from somebody else's firmware is the easiest kind to get wrong quietly, and the
launch is the manoeuvre where a wrong one is least recoverable.

What is **not** borrowed is the cruise throttle: INAV flies a wing at
`nav_fw_cruise_thr` (1400, i.e. 0.40 of its range) and holds altitude by moving
the throttle between `nav_fw_min_thr` and `nav_fw_max_thr` (1200..1700); this
firmware holds `rth_cruise` (0.55) and learns a standing pitch instead
([14-navigation.md](14-navigation.md)). The check prints both numbers rather
than failing on them, because the difference is a design choice with a page
behind it, not a transcription.

The command is **an attitude, not a rate**, whatever the mode channel says: a
rate command in the same two seconds is an aircraft rotating until somebody
stops it. It ends in one of four ways, and each one says so on the console:

- **a stick** (`launch: over - a stick`). A roll, pitch or yaw stick past a
  fifteenth of travel hands the aircraft back, which is the same fraction and
  the same reason a mission is taken back with - INAV calls its equivalent a
  launch abort deadband. The throttle stick is deliberately *not* one of them:
  the pilot's throttle is at idle during a launch, and touching it is part of
  taking the aircraft, not a reason to drop it.
- **the time** (`launch: over - the time`), `launch_timeout_s` after the switch
  went up.
- **the switch** (`launch: over - the switch`).
- **the aircraft** (`launch: over - returning home`, `- disarmed`, `- failsafe`):
  a launch is a mode the pilot asked for and a failsafe is a *reason*, so the
  fence and a pack that has gone critical stop it exactly as they stop a mission.
  This is checked in the code and in the order of it: the launch is stepped
  *after* the failsafe block, and the day the history of this file is written
  down is the day a mode that returned before the failsafes cost an aircraft.

**And that last one is on the tape now**: `aerialkit-fw-sim 45 launch pack`
throws a wing with `battery_rth` on and sags the pack to 3.25 V a cell four and
a half seconds in - seven metres up, in the middle of the launch:

```
sim:   4501 ms  the pack goes critical, 7 m up, mid-launch
battery: 3.29 V a cell - bringing it home
launch: over - the pack
```

The two checks are that both of those sentences are there and that the
*return* is the thing flying the aircraft afterwards (the status says the
navigator has it, and it is closing on home). The wing's own launch checks -
the climb attitude at handover, the stick ending it - are not asked here,
because this run's ending is not the stick's:
[evidence/sil-launch-pack.txt](evidence/sil-launch-pack.txt).

**Three of those four are on the tape now.** The simulator's `launch` session
throws a wing, hands it to the pilot with a stick, cycles the switch and throws
a *second* launch - then ends that one with the switch going down, which is the
way out a pilot reaches for when the throw has gone wrong and the hand is
already on the transmitter. The checks read both endings off the console
(`launch: over - a stick` and `launch: over - the switch`) and require *two*
launches, because "the switch ended it" is also true of a launch that never
started. The failsafe ending is the run above; the *timeout* is still only the
unit tests' and the code's, because a launch that ends on its own clock is the
one ending nobody performs.

**The switch is an edge, not a level.** The first build of this asked for a
launch whenever the switch was up and the aircraft was armed, and the simulator
said so on its first run: a launch that ended on its timeout left the switch up,
the next pass started another one, and the aircraft was launched again every five
seconds for as long as somebody held the switch. One request, one launch;
cycling the switch asks for another. The edge only counts while the pilot is
flying the aircraft, so a switch flipped on the bench does not fire the moment
somebody arms a wing in their hands - arm first, then the switch, which is
INAV's order too.

## What is not here yet: the throw itself

INAV's launch is a twelve-state machine that watches the **accelerometer and the
GPS for the throw** (`nav_fw_launch_accel`, 1.9 g; `nav_fw_launch_velocity`,
3 m/s), so the motors stay at idle until the aircraft is moving and the pilot
never touches the transmitter at all. AerialKit does not have that half, and the
reason is the instrument rather than the ambition: the simulator's wing has no
throw in it - its speed follows its throttle, with no acceleration transient and
no hand - so a detection threshold here would be a number nobody could measure,
which is the one kind of number this project does not keep. The switch is the
detection until there is a plant, an airframe, or a log of a real throw to
measure one against.

That also means the honest limits: this holds a climb for a pilot who has
already thrown, and it is worth nothing if the wing leaves the hand with the
motors at idle because nobody flipped the switch. The procedure for the first
launch is in [23-first-flight.md](23-first-flight.md).

What is measured, in the loop, is `aerialkit-fw-sim 14 launch`
([evidence/sil-launch.txt](evidence/sil-launch.txt)): the motors reached the
launch throttle (0.71 of full, against 0.70 asked for), the aircraft flew the
attitude it was given (18 degrees of nose-up when the pilot took it, against the
18 it was asked for), it left the ground while the launch was flying it (20 m),
the mid-launch `status` says `on autopilot`, and the stick gave it back with the
later `status` saying `armed`. Not verified: a throw, a wing, or a wind.
