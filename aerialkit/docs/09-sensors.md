# AerialKit - inertial sensors and the barometer

```text
driver            ak_bus_t                    the board
ak_imu_icm42688   read(reg, buf, len)  <--   SPI2, mode 3, chip select PB12
                  write(reg, value)          PB13 SCK, PB14 MISO, PB15 MOSI
                  delay_ms(ms)
```

## The bus is a seam, not a wrapper

A driver asks for a register by the number in its datasheet. It never learns
whether it is on SPI or I2C, which pins, or how a chip select works - the SPI
layer sets the read flag (bit 7 of the address byte, an InvenSense convention)
and the board drives the chip select. That is what makes the whole driver
testable against a register file in a test, and it is why adding a second IMU
is a table entry plus a register map rather than a new abstraction.

## The parts

Six names, three drivers, one table: a board asks for "an IMU on this bus" and
gets whichever answers.

| Driver | Who-am-i | Notes |
| --- | --- | --- |
| `ak_imu_icm42688.c` | 0x47 `icm42688p`, 0x42 `icm42605` | 1 kHz, gyro ±2000 dps, accel ±16 g, low noise mode, the AFSR workaround, the low-latency UI filters |
| `ak_imu_mpu6000.c` | 0x68 `mpu6000`, 0x70 `mpu6500`, 0x71 `mpu9250` | 1 kHz through its divider, DLPF 42 Hz, gyro ±2000 dps, accel ±16 g, auxiliary I2C off |
| `ak_imu_bmi270.c` | 0x24 `bmi270` | 1600 Hz, gyro ±2000 dps, accel ±16 g, performance mode - and eight kilobytes of Bosch's own firmware uploaded at every boot, without which its data is invalid |

The 42605 is the same part as the 42688-P under another who-am-i, so it is a
second table entry rather than a second driver - and adding it immediately
found a bug: the init re-read the who-am-i and insisted on 0x47, so the entry
failed its own driver's configuration. The check now accepts either part the
driver serves, which is what it was always for.

The MPU-6500 and MPU-9250 are in the table for the same reason and on the same
terms: the same who-am-i *register* with a different value in it, the same
configuration registers with the same bit encodings, and the same twelve bytes
out. The reference implementation writes the same CONFIG, GYRO_CONFIG,
ACCEL_CONFIG, INT_PIN_CFG and INT_ENABLE bytes for a 6500 as for a 6000 and uses
one DLPF table for the family, so `0x03` is "the 42 Hz setting" on all of them
whether or not each datasheet's own table calls that bandwidth 42. They are
worth having because the wing's flight controller lists all four IMU options
and one of them may well be the part on it - a firmware that answers "0x70,
which is not a known part" is a firmware that cannot fly that board.

One difference, written down rather than smoothed over: the reference's
MPU-6500 sequence resets the part (PWR_MGMT_1 bit 7, a SIGNAL_PATH_RESET, a
hundred milliseconds of waiting) before configuring it, and AerialKit does not,
because the sequence here is the 6000's and the host tests pin it write by
write. A 6500 that misbehaves on a bench is worth trying that reset against
first.

## The BMI270, which is a part with a program in it

Every other sensor here answers a who-am-i and takes configuration writes.
The BMI270 takes a *program*: eight kilobytes of firmware for its own
microcontroller, which has to be uploaded after every reset before its gyro and
accelerometer data mean anything. It is the first part in this tree whose data
is invalid rather than merely unconfigured, and the first thing in this
repository that is copied rather than written - see
[03-attribution.md](03-attribution.md), where that is recorded in the terms the
plan asked for.

Three things came out of that difference:

- **`ak_bus_t` grew an optional burst write.** Eight kilobytes one byte at a
  time is eight thousand transactions where the transport can do one that
  never lifts the chip select, and the BMI270's upload is a single register
  write as far as the part is concerned. A bus without a burst falls back to
  the byte at a time, and the host test runs *both* paths and holds them
  against the same bytes - the point of an optional call is that the driver
  never finds out which it got.
- **The part is asked whether it took the file.** The internal status
  register's low bit says the feature engine started, and a part that says no
  is refused rather than read from. Zeros from a sensor whose data is invalid
  are a perfectly plausible accelerometer reading at free fall, which is
  exactly the kind of plausible wrong answer this repository is built to
  refuse.
- **The file itself is checked byte for byte.** Its length, its first and last
  bytes and the sum of all eight kilobytes are held against Bosch's published
  array, because a copy that lost its tail compiles perfectly and would leave
  a part that reports nothing.

**And the half the flight loop actually calls had never run.** Everything above
is about getting the part configured; `bmi_read()` - one twelve-byte burst, the
little-endian assembly, the sign extension and the two scales - was compiled and
never executed by anything. A scale or a byte order wrong there is not a broken
sensor, it is an aircraft whose attitude estimate is plausible and wrong, which
is the kind of fault that survives a bench and ends a flight. The host test now
reads a sample with numbers chosen so each one pins something: a value of exactly
one LSB of the range in use, a *negative* one (where a missing sign extension
shows), zeros where a wrong register offset would put the gyro, and a gyro at
full scale. It also runs the sentences a person reads when the part will not
open - the sink `ak_imu_open()` takes is the console at boot, and the lines
naming the part, its status byte and the configuration it was given are now
printed by a test rather than merely compiled, which is the difference between
a diagnosis and a part that reads zeros. `ak_imu_bmi270.c` has no line the host
suite does not reach.

The MPU-6000 is *not* the same part: its data registers are at 0x3B rather than
0x1F, it has no bank select, its clock is a three-bit field in a
power-management register, and its sample rate is a divider rather than an
output-data-rate encoding. It is also the sensor every flight controller of the
last decade has had on it, which is the reason to drive it.

Every address and constant was read out of the reference implementations in the
workspace rather than recalled, and [03-attribution.md](03-attribution.md) names
the revisions:

| Fact | Where it came from |
| --- | --- |
| Register map, FSR/ODR encoding, the AFSR workaround | Betaflight 2026.6.1 `accgyro_spi_icm426xx.c` |
| Who-am-i 0x47 for the 42688-P, 0x42 for the 42605 | Betaflight `accgyro_mpu.h` |
| 16.4 counts per dps at ±2000 dps | INAV 9.1.0 `accgyro_icm42605.c` |
| 2048 counts per g at ±16 g | Betaflight's `acc_1G` for that range |

## What is verified, and what is not

Verified on the host, against a fake bus that answers like the part:

| Check | Why that is the check |
| --- | --- |
| A part with who-am-i 0x47 is detected, 0x42 is refused, an empty bus is not a sensor | a driver wrong about its part must fail where you can see it, not read plausible numbers from the wrong registers |
| The write sequence and every value in it, in order | pinning the sequence makes a change to the driver a change to the test, visible in a diff |
| The delays happen | the part needs time after a mode change before its output means anything |
| Scaling: 2048 counts per g, 16.4 per dps, converted to rad/s, sign included | where a units bug lives, and it is checkable by hand |
| A failed read gives `valid = 0`, not a stale sample | the flight core reads that as "no attitude", which is the safe direction |
| The part is re-checked after configuration | a sensor that stops answering must not stay "present" |

Not verified, and this is the longer list: **no real inertial sensor has been
connected.** No SPI clock has been measured, no chip select timing checked, no
who-am-i read from a physical part. The register map is right by citation, not
by contact.

Three things stood between this driver and a usable attitude, and all three
exist now: **board alignment** (three rotation parameters), **gyro bias**
(measured by `calibrate` and by the aircraft itself at power-up, and stored in
the parameter table so it survives a reboot), and the **accelerometer's
six-position calibration** (`calibrate accel 0` to `5`, one face per command,
which writes a bias and a scale per axis) - see
[10-calibration.md](10-calibration.md). What none of them can fix is a part
that has never been connected: the arithmetic is checked against the datasheet
and against itself, and the checks that would matter are a real part held on a
real face.

## Bench checks

**With no sensor, which is what this board has:** jumper MOSI (PB15) to MISO
(PB14) and run `spi`. That writes a pattern and reads it back, and it is the
only way to check the port without a device:

```text
ak> spi
spi:       sent    55 aa 00 ff 5a a5
spi:       read    55 aa 00 ff 5a a5
spi:       loopback matches - MOSI reaches MISO
```

A match proves the clock, the pins and the transfer path. No match with nothing
wired is what a floating MISO does, and the line says so.

**With an IMU wired:** the boot banner should name the part, `imu` should show
samples climbing with roughly (0, 0, 1) g of accel when it is flat and still,
and `status` should show the attitude estimate converging. When both are true
the aircraft can arm - which is the milestone this document is the precondition
for, and the point at which "props off" stops being advice and becomes a rule.

## The barometer

A barometer is the one sensor that measures *height* directly, and height is the
axis this aircraft flies worst: a wing holding an altitude on GPS alone is
holding it on a measurement that lags seconds behind and wanders a few metres
while it catches up.

It is on **I2C**, because that is where the barometer is on the boards that have
one - including the wing's own AIO. The driver did not change when the bus did;
[20-i2c.md](20-i2c.md) has the bus, its timing arithmetic and the three read
lengths the acknowledge has to be put in different places for.

It sits on the same bus seam as the IMU and on the board's own choice of bus -
I2C on this one, for the reason above. It is the same shape as an IMU driver -
a name, a who-am-i, two functions - and the probe decides which part answered
and so which arithmetic to use.

There are three drivers, and between them they cover every barometer the plan
names - which is the same reason the IMU table has three parts in it:

| Driver | Who-am-i | What it is |
| --- | --- | --- |
| `ak_baro_dps310.c` | 0x10 `dps310`, 0x11 `spl06-003` | the Infineon part, and the same part under another name with three more coefficient bytes and one more term in its pressure polynomial - which is exactly why the probe decides the arithmetic |
| `ak_baro_bmp280.c` | 0x58 `bmp280`, 0x60 `bme280` | the Bosch part, and the BME280 that is the same part with a humidity sensor nothing here asks about. **Not the same arithmetic at all**: 64-bit intermediates, shifts of 33 and 47 bits, a Q24.8 result |
| `ak_baro_bmp388.c` | 0x50 `bmp388`, 0x60 `bmp390` | the newer Bosch part, and the BMP390 that is the same part under another id. Its arithmetic comes in two variants and this driver is the fixed-point one - no double precision unit on the target and no libm in the core |

The BMP388 is worth a paragraph of its own, because the datasheet gives its
compensation twice - a float variant with scaled coefficients and a 64-bit
integer one - and **the two do not come out in the same units**: the integer
result is hundredths of a pascal and the float one is pascals. The Bosch API
says so in its own comment ("if pressure is 9528709 which is 9528709/100 =
95287.09 Pascal") and in its limits, 3000000 and 12500000 against the float
variant's 30000.0 and 125000.0. A driver that mixed them up would report a
pressure a hundred times too high - still a pressure, still in the right
ballpark for no atmosphere, and an altitude out by about eleven kilometres.
The host test pins the integer answer and the float answer side by side, which
is what caught the factor of a hundred on the way in.

That test also caught a real one in the last line of the integer arithmetic:
the sum before the final multiplication is about 4.4e17 at sea level, and
multiplying *that* by 25 overflows a signed 64-bit - it came out negative, which
reads as a pressure below absolute zero. The Bosch API casts to `uint64_t`
there for exactly this reason, and so does this driver now.

The BMP280's arithmetic is the one worth being careful about, because a
narrowing mistake in it does not fail - it produces a number that looks like a
pressure. The test uses the datasheet's own worked example and two more points
computed separately from the same equations, one cold and one hot: one point
cannot tell a wrong shift from a wrong constant that happens to work at room
temperature. It found two things on the way in. Python's `//` floors where C's
`/` truncates, and on these numbers the two disagree - the datasheet's
published 25.08 degrees and 100653 pascals is what the implementation here
produces. And the filter field's encoding is not the oversampling's: off, 2, 4,
8 and 16 are 0..4, so an 8x filter is 0x03 while 0x04 is a 16x filter, one bit
away and entirely plausible.

**And the SPL06 half of the first driver had never been run.** The probe has
distinguished the two ids since the driver was written, and every sample the
tests read was a DPS310's: the three extra coefficient bytes, the twelve-bit
split of c31 and c40 across two registers, the sign of a negative one, and the
two extra terms of the eleven-term polynomial had been compiled and never
executed. That is what a wing's altitude would be flown on if the part fitted to
the board were an SPL06 - and the two are pin-compatible, which is exactly why a
board can have either. The test now reads a sample from each id, with
coefficients chosen so that a nibble the wrong way round, or a missing sign
extension, changes the answer, and holds both against the datasheet's equations
computed separately. `ak_baro_dps310.c` has no line the host suite does not
reach: the rest of what was left was the two ways a part does not come up - a
register the bus will not answer, and a bus that fails *part-way* through the
initialisation, which is worse than one that never answered - and the line the
boot prints naming the part it found.

| Fact | Where it came from |
| --- | --- |
| Register map, reset sequence, oversampling, the coefficient decode | INAV 9.1.0 `barometer_dps310.c` @ `e519b69` |
| The compensation arithmetic (datasheet sections 4.9.1 and 4.9.2) | the same file, which writes those sections out in code |
| The scale factors for sixteen-times oversampling | the same file's table, 253952 |
| Who-am-i 0x10 for the DPS310, 0x11 for the SPL06-003 | the same file |
| The BMP280 register map, chip ids, coefficient layout, the data frame, the oversampling and filter encodings, and the integer compensation arithmetic | INAV 9.1.0 `barometer_bmp280.c` and `.h` @ `e519b69`, which follow the datasheet's sections 3.11 and 4.3. The expected numbers in `tests/test_bmp280.c` were computed separately, from those equations |
| The BMP388 register map, chip ids, trimming data layout, forced-mode sequence, and the fixed-point compensation arithmetic | INAV 9.1.0 `barometer_bmp388.c` @ `e519b69` (datasheet sections 3.11 and 9.1), checked against Bosch's BMP3-Sensor-API `bmp3.c` @ `db0f6562` - which is where the units of the integer variant and the `uint64_t` cast in its last line are written down. The expected numbers in `tests/test_bmp388.c` were computed twice, once from each variant, and agree to a hundredth of a pascal |
| Trimming words P5 and P6 are unsigned | INAV and the Bosch API both read them that way; ArduPilot's float driver reads them signed, which is harmless for the values real parts have |

What the firmware adds is the part that turns pressure into a height, and it is
in the core where a host test can hold it against the standard atmosphere:

```text
h = 44330 * (1 - (p / p0) ^ 0.190295)
```

There is no `pow()` in an image that links no libm, so the fifth term of the
binomial series is used instead, and the test checks the whole curve against
libm's exact one from -100 m to 2 km: worst error under ten centimetres. It is
clamped past `p/p0 = 0.65` (about 4.5 km) rather than left to diverge, because a
wrong number is better than an infinity in a control loop.

Two things about it are worth stating rather than implying:

- **It measures pressure, not height.** The reference is captured while the
  aircraft is disarmed and standing on the ground, and everything the firmware
  uses it for is a *change*: the absolute pressure is today's weather and the
  height above the bench is something a person can check by lifting the
  aircraft a metre.
- **The compensation is pinned, not proven.** The expected pressures in the
  test were worked out separately from the datasheet's equations, which is the
  same evidence as a second implementation - and a real part is still the only
  thing that can say the coefficients were decoded the way that part packs
  them. None has been on this bench.

Where it is wired, and what it is for: **nothing, yet.** The bench board has no
barometer and the pin map has no chip select for one, so `baro` says "none
fitted" and the driver waits. The simulator is where it is exercised end to end
- it answers with the pressure the standard atmosphere gives at the airframe's
height, and the flight check is that the firmware reads that height back:

```text
sim: ok   the barometer reads the height the aircraft is flying at
          the barometer says 11.4 m, the airframe is at 11.4 m
```

## The height the aircraft flies on

The barometer is no longer a number on a console page: it is the altitude the
navigator holds, through `ak_altitude.*` - a complementary filter with the same
argument as the attitude estimate, and for the same reason. The barometer is
fast and drifts; the GPS is slow, noisy and absolute; so the barometer's
movement goes into the answer immediately, and the GPS moves the *offset*
between the two slowly enough not to chase its own noise.

What that buys is measurable, and the test measures it rather than asserting it:
the same synthetic five-minute flight is given to all three - a barometer
drifting three centimetres a second, a GPS with four metres of noise at five
hertz, and the filter - and their errors against a height both are trying to
measure come out as

```text
rms error: fused 0.43 m, gps 2.06 m, baro 5.47 m
```

Five times better than the GPS alone and thirteen times better than the
barometer alone, which is the property a complementary filter is supposed to
have and the reason to carry two sensors for one axis. The leak rate is not a
guess either: the drift error grows with it and the noise error shrinks, the
minimum of the sum is around fifteen seconds for those two numbers, and the
comment next to it in `ak_altitude.c` says so.

Two things it does *not* need: the other sensor. A board with no barometer gets
the GPS altitude, exactly as it always did - the filter degenerates to the one
sensor it has - and a board with no GPS still knows how far it has climbed since
take-off, which is what a barometer is for. Both of those are cases in the test
rather than sentences in a document.

And the whole chain is checked in the simulator, which is the only place the
pieces meet: a standard atmosphere computes the pressure at the airframe's
height, the DPS310's registers report it, the driver compensates it, the series
turns it into a height, the filter fuses it with the GPS, and the check is that
the height the *navigator* is flying on matches the airframe:

```text
sim: ok   the barometer reads the height the aircraft is flying at
          the barometer says 11.4 m, the airframe is at 11.4 m
sim: ok   and the height the navigator flies on is that height
          the fused height is 11.4 m, the airframe is at 11.4 m
```

What is still not verified is anything about a *real* barometer: the part has
never been on this bench, so its coefficients have never been decoded from a
part that produced them, and the drift being corrected here is a model.

### And a barometer that stops answering

A part that dies in flight does not announce itself: its reads fail, the sample
the estimate is built from stops arriving, and the height the navigator flies
on becomes *still*. Still is what a landing looks like, so this was worth
measuring rather than assuming - and it was wrong in two ways.

Measured in the loop, with the barometer failing as a return's descent began
(`aerialkit-fw-sim 100 quadrth nobaro`): the navigator's height froze at
twenty metres, so the descent it was commanding never appeared to happen, the
landing rule never fired - it needs the hover height first - and the aircraft
ended the session **sitting on the ground with its motors still running**. On
an aircraft that is props grinding the ground; over a slope or an obstacle it is
worse.

Two fixes, both narrow:

- **The estimate falls back to the GPS.** The filter already has that answer
  for a board with no barometer fitted: the GPS, relative to the same ground
  reference. `ak_altitude_baro_lost()` says "the part has stopped" - only the
  caller knows, because only it sees the reads failing - and two failed reads in
  a row (60 ms at 32 Hz) is what says it. A good sample puts it back, and the
  offset is *re-seated* on recovery rather than added to: the leak had been
  dragging that offset the whole time the part was gone, so the first sample
  added to it gives a height nobody measured (the host test caught this: a
  return to 20 m of pressure came back as **-60 m**).
- **The landing rule refuses to decide on a height that is not being
  measured.** The rule reads the *stall* of a descent, and a dead sensor stalls
  perfectly, so it now requires the height to be live - the barometer answering
  or the fix valid, the two sources the estimate is built from. Without either,
  it says nothing at all rather than stopping the motors in the air.

After both: the same run lands normally - motors stopped at 0.6 m, 0 m from
home, disarmed, at the same second as the healthy flight - with the console
having said `baro: 2 reads failed in a row - the height is coming from the
gps`. The unit half is in `tests/test_altitude.c`
(`test_the_barometer_stops_answering`), and the loop half is the recorded
transcript [evidence/sil-quad-return-nobaro.txt](evidence/sil-quad-return-nobaro.txt).

## How far the ground is

Every measurement above is a *change* or an absolute number that wanders. The
barometer is anchored once, on the ground, and the anchor leaks; the GPS is
absolute to a few metres. That is fine for flying at altitude and worst exactly
where a flight ends, because the last metre of a descent is the same size as
the error in the instrument flying it - and an aircraft that believes it has
arrived while it is still a metre up either hovers until the pack is flat or
stops its motors in the air.

So the aircraft can carry a rangefinder pointed down: a **TOF10120** on the
I2C bus the barometer is on, at its own address (`0x52`, which is what the part
answers to out of the box). It is a light, cheap, two-metre part, and the two
metres are the point: it does not replace the barometer, it answers the
question the barometer cannot, and it says nothing at all above its range
([`src/core/sensors/ak_rangefinder_tof10120.c`](../src/core/sensors/ak_rangefinder_tof10120.c)).

The service around it - `ak_rangefinder.c` - is deliberately not a decision
maker. It owns the part's polling period, the counters, and one filter:

> **A reading that says the ground moved faster than the aircraft can move is
> not the aircraft.** Eight metres a second is what that means here, and the
> limit is scaled by the time since the last reading that was believed, because
> a step measured over a longer gap is a slower rate. A refused reading is
> *dropped*, not clamped, and the baseline is the last reading that was
> believed - so one refusal is not a permanent refusal, and a reading the
> aircraft could have made is never thrown away for being large.

What the filter cannot see is a part that is *steadily* wrong: a lidar looking
down at a hedge from twenty metres reports a metre and a half for as long as
the aircraft is over the hedge. That is the job of the rule that uses the
number, and it is done there - a landing is refused unless the rangefinder and
the height estimate agree about where the aircraft is ([15-preflight.md](
15-preflight.md), and the landing rule in [`src/core/main.c`](../src/core/main.c)).

What it is used for, then, is two things, both of them in the navigator:

**The last of a descent.** Once the return has begun its landing, the height
the vertical loop flies is the part's when it has a reading: the target is the
ground and the measurement is the distance to it, so the loop keeps asking the
aircraft down until the part says it has arrived, whatever the barometric
estimate believes. The rate the loop damps with is still the barometer's,
because differentiating a noisy two-metre part is a worse rate than the window
the profile already keeps.

**A landing with no position at all.** A quadrotor that has lost its fix
currently holds - level, at its altitude, drifting with the wind - until the
pack is flat. With a rangefinder fitted and `quad_hold_land_s` set, it does
what a pilot would instead: after that many seconds of holding it comes down
where it is, on the part, and stops. The parameter is off by default, for the
same reason the fence and the battery return are: it takes an aircraft that is
still flying and commits it to landing. And it *requires* the part - without
something measuring the ground, the descent would be flown on the instrument
that has already failed, so without one the hold goes on being a hold. That
pair is the measured evidence below.

### What is verified, and what is not

Verified on a host, against a modelled TOF10120 on a fake bus: the register the
part has to be written before it answers, the address register read back, the
two bytes of distance and which end of them is which, the rule that at or past
2000 mm there is nothing to report, the 30 mm blind zone, the polling period,
the staleness window, and the filter above - a step the aircraft could have
made is kept, one it could not is refused, and the same reading later is
believed again because the aircraft had longer to move. 32 checks, in
`tests/test_rangefinder.c`.

Verified in the loop, with a rangefinder on the simulated aircraft:

| Session | What it shows |
| --- | --- |
| `quadrth` (part fitted) | the return lands as before and stops its motors at the ground, now on the part's verdict rather than a stalled estimate |
| `quadrth norange` | the same flight with no part: the barometric rule lands it, which is the path every earlier flight in this repository took |
| `quadrth rangefail` | the part **stops answering as the descent begins and comes back four seconds later**: the console says so once each way, the `range` report carries the count while it is away, and the return still lands on the rule that does not need the ground |
| `gpslostland` | the fix is gone, the link is down, and after eight seconds of holding the aircraft comes down where it is and stops on the ground |
| `gpslostland norange` | the same pilot setting with no part: it holds, and the motors keep turning |

**The `rangefail` session is the one that ran last, and it found a bug.** The
scenario word had existed since the part was written and no session had ever
run it or checked anything it did - and its first run printed, on a console
somebody would be reading in the air:

```
rangefinder: no answer - the ground is not being measured
rangefinder: answering again after 1 failed read
rangefinder: no answer - the ground is not being measured
rangefinder: answering again after 1 failed read
...
```

once per poll, for as long as the part was dead. `ak_rangefinder_read()` was
returning **0 for two different facts** - "the part was not asked this pass"
and "the part answered, and the reading was not usable" - and the caller
treated everything that was not a fault as an answer, so a dead part looked
like a part answering every other pass and `range_fails` never rose above one.
The count is the whole point of that line: it is how long the ground has not
been measured. The service now returns four values instead of three
(`ak_range_result_t`: a reading, nothing usable, idle, fault), the caller only
clears its count on an actual answer, and the same session reads:

```
rangefinder: no answer - the ground is not being measured
          10 reads in a row with no answer          <- the `range` report, 1 s later
rangefinder: answering again after 40 failed reads
```

The check is the pair: the two sentences once each, the count in the report,
and the aircraft still on the ground 1 m from home. The unit test pins the
distinction that was missing (`AK_RANGE_IDLE` is not an answer).

Not verified: **the part**. No TOF10120 has been on this bench, so its
datasheet's numbers (the blind zone, the 2000 mm range) are the ones the driver
carries rather than ones measured here, and the mount - how far the sensor sits
above the ground when the aircraft is resting on its gear - is exactly the
number the landing rule's `range_land_mm` exists to absorb rather than guess.
