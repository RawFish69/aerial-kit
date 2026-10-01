> Historical notes from the full development tree. Current public targets and commands are in the [firmware guide](../../docs/flight-controller-firmware.md). Host C tests and simulator harnesses referenced below are retained outside this public source tree.

# AerialKit - alignment, gyro bias and accelerometer scale

Three things stand between "the sensor works" and "the aircraft knows which way
it is pointing", and all of them are wrong on a real board by default.

## Board alignment

No flight controller has its inertial sensor bolted on square. The part is
rotated somewhere between "close enough" and "90 degrees in two axes", and the
firmware has to know: a roll axis that is really the pitch axis is a crash that
looks like a tuning problem.

Three parameters say how the sensor sits in the airframe:

| Parameter | Meaning |
| --- | --- |
| `align_roll_deg` | rotation about the airframe's x (forward) axis |
| `align_pitch_deg` | then about y (right) |
| `align_yaw_deg` | then about z (down) |

They are the rotation that takes a vector in the sensor's frame and expresses it
in the airframe's, applied roll, then pitch, then yaw - the order is part of the
answer, not a detail. Zero is the identity, so a board mounted square is not
quietly rotated by the code that is supposed to leave it alone.

The matrix is written out in closed form rather than built from a library,
because the order is the thing that matters and a library call would hide it.
The test checks the property that catches a mistake in that closed form: an
orthonormal matrix at arbitrary angles, plus the 90 and 180 degree cases
somebody will actually hit.

## Gyro bias

Every MEMS gyro reports a small rate when it is not moving. The control loop
cannot tell that offset from a slow rotation, so it integrates into a growing
attitude error until the aircraft is holding an angle nobody asked for.

**All four calibrations refuse while the aircraft is armed** - the gyro, the
receiver's centres, the pack's divider and the accelerometer's six faces - and
that last one is a *fix* rather than a feature: it was missing its guard until a
bench session asked for it. The session is the four of them tried in the wrong
order, and its first check is the count:

```
sim: ok   every calibration refuses to run on an armed aircraft, the
          accelerometer's included    4 of the four calibrations refused while armed
```

**And so does `save`, which is not a calibration but has the same rule for the
same kind of reason.** It writes flash, and on the F405 that is an erase of the
whole 128 KB configuration sector with the instructions coming out of the same
bank: about a second during which the loop does not run. In the air the DShot
frames stop and an ESC's own failsafe times out in a fraction of that, so the
command refuses with `save: refusing - this writes flash, and an armed aircraft
is not a bench` - and the same session types it in that state to prove it:

```
sim: ok   and a save on an armed aircraft is refused, because the write stalls
          the loop    the console says why, and writes nothing
```

Two reasons, and the second is why the missing guard was a defect rather than
untidiness: holding an armed aircraft in six attitudes is a hand near a live
throttle, and the last face writes `accel_bias_*` and `accel_scale_*`, which the
flight loop applies to the next sample - so an armed aircraft would have its
attitude estimate changed under it. Removing the guard again fails that check
with "3 of the four". The rest of the same session is the other sentences a
bench produces, each of which is a diagnosis:

| The command | What comes back |
| --- | --- |
| `calibrate vbat 99` | `99.00 V is not a pack this aircraft flies` |
| `calibrate vbat 12.6` with no pack on the divider | `the pin reads 0.0 mV - connect the pack` |
| `calibrate accel 9` | `'9' is not a face` |
| `calibrate rc` with a receiver that has never spoken | `no frames from the receiver` |
| `calibrate` with somebody moving the aircraft | `failed with 7 good samples and 1993 rejected` |
| `set mission_channel 7` then `save` on a board whose flash refuses the record | `save: the board refused the write`, and `params` still says `1 changed since the last save` |

Transcripts: [evidence/sil-calibrations-refused.txt](evidence/sil-calibrations-refused.txt)
and [evidence/sil-calibrations-no-receiver.txt](evidence/sil-calibrations-no-receiver.txt).

The last row is the one that is not a calibration and is in this session because
it is the same kind of bench mistake. **A save that failed is not a save**, and
two things have to agree about it: the console says the board refused the write,
and the parameter that was set is still counted as unsaved. A firmware that
printed `saved` anyway - or marked the table saved - would leave an aircraft
flying the settings it had before with nobody told, and the reason that is
worse than a refusal is that nothing on the aircraft would look wrong. The
session also saves the same parameter again on an honest board, which is what
stops the check from passing on a firmware that refused every save; the
*controller* side of the same failure - the part that never finishes an
operation - is in [21-port-on-the-host.md](21-port-on-the-host.md).

`calibrate` measures it:

```text
ak> calibrate
gyro bias: roll 0.412, pitch -0.088, yaw 0.155 dps (500 samples, 0 rejected)
calibrate: done - 'save' keeps it across a reboot
```

Three things about it are deliberate:

- **It refuses while armed.** A calibration that runs when the aircraft is not
  still is worse than none, because it is wrong in a way that looks deliberate.
- **It refuses samples that show movement, and the test for that is
  steadiness rather than size.** A sample counts as still when it sits within
  about three degrees a second of the mean of the samples already accepted,
  and when the accelerometer is still pointing where it was pointing. The
  obvious test - "the rate must be small" - cannot work, because the offset
  being measured *is* a rate: a part with five degrees a second in it fails a
  three-degree test on every sample and is never calibrated at all. That is
  measured, not reasoned about: 2198 samples, 2198 rejected, and a heading that
  walked 109 degrees while the aircraft was parked. The accelerometer is asked
  as well because a steady rotation reads exactly like an offset on a gyro,
  and a rotating aircraft's gravity vector moves. What neither catches is a
  steady rotation about the *vertical*; nothing on this aircraft separates that
  from a yaw offset.
- **A reading beyond what a part can be off by is refused as a plot.** Twenty
  degrees a second, the spec limit of the parts this flies: more than that is
  not a calibration, it is a different sensor or a different aircraft.
- **It refines rather than doubles.** The accumulator holds what the gyro
  reports *after* the current compensation, so calibrating twice converges on
  the same value instead of applying the offset twice.

The result lands in `gyro_bias_roll`, `gyro_bias_pitch` and `gyro_bias_yaw`, in
degrees per second - which means `params` shows it, `save` keeps it across a
reboot, and `set gyro_bias_yaw 0.2` is a legal way to correct it by hand.

### And the measurement the aircraft takes for itself

The number saved from the bench is the number the part had at whatever
temperature the bench was. The flight happens at another one, and a gyro bias
that moves with temperature is a heading that walks away with nothing to stop
it - in yaw there is no accelerometer to correct it, and the GPS-track aid only
works while the aircraft is moving. So the firmware measures the bias itself:
once, at power-up, while the aircraft is disarmed and still.

```text
gyro: bias measured while disarmed: 12 -4 8 mdps (500 samples, 0 rejected)
gyro: no bias measured before arming (0 samples, 500 rejected as moving); the stored bias stands
```

The first line is the normal one. The second is what happens when the aircraft
is picked up and carried before it is armed, and it is the honest answer: the
stored bias stands. Nothing that is moving is ever written into a bias.

**And the second line is on a tape now.** The `armnow` session in the simulator
holds the aircraft with the small rotation a hand cannot help
(`sim_handled`), throws the arm switch while it is still being held, and reads
back what the firmware measured - zero samples, two thousand two hundred
rejected as moving - and then puts it down and flies it, because an aircraft
told "the stored bias stands" is an aircraft expected to fly. Transcript:
[evidence/sil-armed-while-held.txt](evidence/sil-armed-while-held.txt).

The first version of that session tried the other way in, arming the board
before the measurement could finish, and the simulator refused it: arming needs
the attitude estimate to have converged (fifty accelerometer updates) and then
holds the switch for `arm_hold_ms` - five hundred by default - so a *still*
aircraft has always finished its five hundred samples first. The margin is
fifty milliseconds, which is worth knowing before shortening that hold or
slowing the inertial sensor down; the way in that is wide open is a hand.

Three things were learned getting this right, and all three were measured:

- **The measurement has to run while *disarmed*.** The first version ran it at
  arm, when the switch has just come up and the aircraft may already be
  rolling: the fence session, which arms and then launches, ended up fighting
  its own yaw axis with 13.6 degrees a second of elevator, because a wing's
  take-off roll had been written into the bias.
- **The test for "still" cannot be the size of the rate**, for the reason
  above.
- **And it is one measurement per power-up, not one per flight.** A second
  measurement taken after a landing was tried and removed: the aircraft that
  has just landed is still settling, and half a second of *steady* reading from
  a rolling airframe is the shape of a bias. Measured with five degrees a
  second of real bias in the part, the post-landing measurement came back
  4575, 5770 and 5032 mdps on the three axes, and the heading then walked 15
  degrees while parked. The moment to measure is when somebody has put the
  aircraft down and switched it on.

What it buys, measured on the host (the estimator and the calibration, with a
part carrying five degrees a second that nobody has calibrated):

```text
uncalibrated: 100 deg of heading drift in 20 s
after the arm-time measurement: -0.0 deg in 20 s
```

and in the loop: `aerialkit-fw-sim 100 quadrth gyrobias` gives the aircraft a
gyro that has warmed up since the bench, and the heading walks **zero degrees**
while it is parked at the end of the flight - against 109 degrees with the
measurement disabled, which is the check failing and the reason it exists.

## What is verified

On the host, arithmetic only: the identity, the 90 and 180 degree cases, the
matrix staying orthonormal at odd angles, that a constant offset is measured
exactly, that applying it leaves a stationary gyro at zero, that moving samples
are rejected, that a second pass converges on the same value, and that the
number survives the trip through degrees per second.

## Accelerometer, six positions

A gyro bias is a constant that grows an attitude error over time. An
accelerometer error is an attitude error that is there immediately, and the
angle loop will fly the aircraft to it: a part reading 1.02 g with an axis up
and -0.97 g with it down is 20 mg off centre and half a percent off scale, which
is about 1.2 degrees of tilt nobody asked for, on every axis at once, in a way
that on the bench looks like a badly levelled desk.

Six positions measure both at once, because each axis gets read pointing up and
pointing down:

```text
bias  = (up + down) / 2
scale = 2 / (up - down)
```

It is six commands, one per face, because a calibration that has to be done in
one go is a calibration nobody finishes:

```text
ak> calibrate accel
calibrate accel: which face? [0] level, [1] inverted, [2] nose down,
                 [3] nose up, [4] right side down, [5] left side down
ak> calibrate accel 2
calibrate accel: hold the aircraft nose down, not moving...
  [0] level            measured
  [1] inverted         measured
  [2] nose down        measured
  [3] nose up          -
  [4] right side down  -
  [5] left side down   -
calibrate accel: nose down done - 3 faces to go
```

After the sixth:

```text
accel bias:  0.0300 -0.0200 0.0400 g
accel scale: 0.9801 1.0204 1.0054
calibrate accel: done - 'save' keeps it across a reboot
```

The faces are the *airframe's*, not the sensor's, and the correction is applied
after alignment - so the instruction is "nose down", not "sensor x negative",
and a board mounted at an angle does not need a different set of six.

Four things are deliberate, and they are all the same idea:

- **A face can be redone.** Starting one again throws its old samples away
  rather than averaging the two attempts, which would mix a wrong face into a
  right one.
- **Samples taken while it is moving are refused**, on the gyro and on the
  magnitude of gravity, and counted.
- **A face that was not flat fails the whole thing.** The two readings for an
  axis have to be roughly 2 g apart - the check that stands between this and a
  correction nobody should believe - and if they are not, nothing is written
  and the old correction stays.
- **The result goes into the parameter table** as `accel_bias_x/y/z` and
  `accel_scale_x/y/z`, so `params` shows it, `save` keeps it, and it is
  range-checked like anything else.

### What is verified

The arithmetic is checked the way that catches a sign, a reciprocal, or a scale
applied on the wrong side of the bias: a part with a *known* error is placed on
all six faces, and the recovered correction has to be that error - and then a
reading from that part, corrected, has to come out as exactly one g up and
nothing sideways.

And the whole thing is driven end to end in
[18-software-in-the-loop.md](18-software-in-the-loop.md), which is the check
worth reading: the simulator holds the aircraft on the six faces and types the
six commands at the console, the firmware measures a simulated part with a known
bias and scale, the parameters land in the table, and the sample the *flight
loop* uses comes out level. That found a bug the unit tests could not - the
first version printed the result with `%g`, and the console formatter has no
floating point, so it printed `%g %g %g` at itself.

## What is not

**No real sensor has been rotated or calibrated.** What is left is the part a
bench answers, not a test:

- whether `align_yaw_deg 90` is the right correction or its inverse for a
  particular mounting. The sign convention is consistent inside the firmware;
  which way round it goes on hardware is measured, not reasoned.
- whether a measured bias is stable with temperature. It will not be entirely,
  and what to do about that - calibrate warm, or estimate it in flight - is a
  decision for after there is something to fly.
- whether six positions is enough on hardware: the arithmetic assumes the
  board's axes are the airframe's axes after alignment, so a mounting that is
  not close to square gets a correction that is right on the faces measured and
  approximate between them. A board rotated 30 degrees is the case to think
  about, and the answer is `align_*` first, exactly.
- whether the accelerometer's scale is stable with temperature, or whether it
  drifts enough to matter. Both of these want a bench and a gyro table, and
  neither has had one.
