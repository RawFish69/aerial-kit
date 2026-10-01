> Historical notes from the full development tree. Current public targets and commands are in the [firmware guide](../../docs/flight-controller-firmware.md). Host C tests and simulator harnesses referenced below are retained outside this public source tree.

# 29 - The control loop's timing, and what the counters mean

The flight core runs its control law once per call to `ak_flight_step()`, on a
one-millisecond period. Everything in this document is about what it does when
that period is not what it expected - a bus that stalls, a sensor that repeats a
timestamp, a receiver that has stopped answering - and about the numbers `status`
prints so that a pilot on the ground can tell those apart: eight of them from
`flight.timing`, and a ninth that says what each link cost the loop.

The measurement behind it is `make timing`, whose probe is
`tests/oracle/test_timing.c`; the repair it drove is recorded in
`hosts/nas/agents/CHANGELOG.md`. Read that probe before changing anything here.

## Two clocks, not one

This is the thing to understand first, because getting it wrong is what the old
code did:

- **The sensor's interval.** `ak_imu_sample_t.time_ms` is the timestamp that came
  with the reading. The estimator integrates the gyro against *this*, because it
  is how much rotation the gyro is reporting.
- **The loop's period.** How long since the last `ak_flight_step()`. The rate
  loop's integral and derivative terms need *this*, because it is how often the
  control law actually ran.

**On the ports that exist today these happen to be the same number, and that is
a property of the ports rather than of the core.** `main.c` stamps the sample
with the loop's own clock - `imu_sample.time_ms = now;`, `main.c:3390` - once per
iteration, whether or not the bus returned new data. So `AK_FLIGHT_LOOP_MS` and
the sample interval coincide, and the distinction is invisible in the numbers.

It is not invisible in what the numbers *mean*, and that is why the core keeps
them apart rather than deriving one from the other. The moment a driver stamps
samples where they are produced - a FIFO read with the sensor's own timestamp,
which is what a gyro running faster than the loop actually wants, and what the
BMI270 at 1600 Hz (`BMI_GYRO_CONF_1600HZ`, written at `ak_imu_bmi270.c:151`)
produces while
the loop reads at 1000 - the two diverge by the ratio of the rates, and every
term that was given the wrong one is wrong by that factor. The core used to hand
the sensor's interval to both; `AK_FLIGHT_LOOP_MS` now lives in `ak_flight.h`
and `main.c` uses the core's rather than keeping its own copy of it.

A consequence worth knowing: because the timestamp is the loop clock, two
iterations that land in the same millisecond - which is what a loop catching up
after a stall does, `next_loop += AK_FLIGHT_LOOP_MS` with `now` unmoved - are a
genuine `duplicates` count, and the core integrates nothing for the second one
because nothing happened in between.

## `max_dt_ms` bounds a step, not an interval

`max_dt_ms` defaults to 50 and is settable from 5 to 200. It is **the longest
one integration step may span**.

It used to mean something else. A sensor interval longer than it was replaced by
a single millisecond, on the reasoning that a huge step would trip the
derivative term. The reasoning was right and the repair was wrong: the aircraft
had flown through the whole gap and the gyro had been working the entire time,
so the rotation was knowable and the estimator was being told a fifty-first of
it. A bus that was simply too slow put the estimate permanently at a fraction of
the true rate, and a duplicate sample - clamped *up* to half a millisecond -
invented rotation that had not happened. The same expression was wrong in both
directions at once.

A gap is now integrated in pieces of at most `max_dt_ms` each, so no single step
is large enough to trip the derivative term or to alias a fast rotation into a
slow one - which is what the clamp was for, and it is still achieved - while the
whole of the elapsed time reaches the estimate. The number of pieces is capped at
`AK_FLIGHT_MAX_CATCHUP` (16). Past that the surplus is **counted** rather than
hidden, which is the difference between this and what it replaced.

On a normal one-millisecond loop this is one piece and the arithmetic is exactly
what it always was. That invariance is asserted: the probe's one-millisecond and
fifty-millisecond rows are identical to the last representable digit before and
after the repair.

## The counters

Printed by `status` on two lines and reachable as `flight.timing`:

| field | counts | what it means |
| --- | --- | --- |
| `gap_steps` | iterations | the sensor interval was longer than `max_dt_ms` |
| `catchup_steps` | extra pieces | what those gaps cost, beyond the one piece a normal step takes |
| `duplicates` | samples | the timestamp did not advance since the last one |
| `unusable` | samples | the driver marked the reading invalid |
| `dropped_ms` | milliseconds | time the loop could not integrate at all |
| `clock_resets` | timestamps | the timestamp went *backwards* |
| `long_loops` | iterations | the loop's own period was longer than `max_dt_ms` |
| `max_loop_ms` | milliseconds | the longest loop period seen |

`loops:` above them is `flight.steps`, the iteration count the log cadence
already reads; it is what makes the rest percentages.

One more line is printed under them, and it is not part of `flight.timing`
because it is not the flight core's: `links:` is the most bytes each of the four
links has taken **in a single pass**, kept per link by `drain_count()` in
`main.c` and read back through `ak_main_drain_max()`. On a healthy aircraft it sits
at 26 for the receiver and a handful for the console, and a number at or near
`AK_DRAIN_QUOTA` means that link is being drained to its quota every pass - which
is what a flood looks like from the inside. It is a maximum since boot and never
decreases, so it can be read after the console has gone quiet.

## Reading them

**A slow bus.** `gaps` and `catchup pieces` climb together, and the attitude is
still right - the rotation was knowable and was integrated. This is the loop
coping. If the numbers climb and the aircraft flies badly anyway, the problem is
the *rate*, not the arithmetic: a sensor that cannot deliver at the rate the loop
wants is a sensor problem, and the estimate is being told the truth about a gyro
that is not being sampled fast enough.

**A frozen sensor.** `duplicate samples` climbs. Nothing is integrated, because
nothing happened. On a real gyro this is usually a driver that has stopped
draining its FIFO rather than a part that has stopped measuring.

**A rebooting or second source.** `clock resets` climbs. A timestamp that goes
backwards is told apart from a long gap *before* the subtraction, because
unsigned arithmetic reports a hundred milliseconds backwards as nearly 2^32
milliseconds forwards and the two want opposite answers.

**A loop that is being called too slowly.** `long loops` and `longest` climb,
and `gaps` does not. This is the pair that says the aircraft is being *flown*
too slowly rather than *fed* badly, and it is the pair to look at after a
telemetry flood, a console being pasted into, or anything else that made the
main loop late. The `links:` line under them is what tells you which link it
was; the `flood` session in `tools/fw_sim.c` is the one that drives this on
purpose.

**Unusable readings.** `unusable` and `dropped ms` climb together: the elapsed
time is counted, but there was no rate to integrate it with and the attitude
does not advance. An unusable reading is not filtered into `flight->gyro`
either - feeding it through would put a garbage rate straight into the term the
rate loop's proportional gain multiplies.

`dropped ms` also counts the surplus of any gap past the catch-up budget. That
number being large is not a fault in itself; it is the loop saying it could not
use that time, which is true and worth knowing.

## What the loop does about communications

The console, the network, the receiver and the GPS are each drained at most
`AK_DRAIN_QUOTA` (32) bytes per pass, so a link that streams without pausing
cannot decide how often the control gate is reached. A byte left in the FIFO is
picked up microseconds later; a command answered a microsecond late is a command
answered, and an iteration missed is an aircraft that did not fly.

The barometer and the rangefinder - the two reads that block on I2C - are read
*after* the control step rather than before it, so the aircraft is flown with the
newest attitude there is before anything slow is asked for.

**This does not make the loop faster.** A single-threaded loop cannot read a
slow part without waiting for it, so `max_loop_ms` will still show the pass where
a barometer took two milliseconds. What it changes is *which* work is late: a
height estimate a millisecond stale rather than a control output. Making the
reads themselves non-blocking - the part signalling a data-ready pin, or a
conversion started and collected a pass later - is a change to the sensor
services, not to this loop.

## What is not measured

- **Execution duration and deadline overruns in microseconds.** `long_loops` and
  `max_loop_ms` count the loop's *period*, which is what the core can see without
  a cycle counter. A DWT cycle counter would give the duration of the work
  itself, and reading one is a port change rather than a core one; it is not
  done.
- **Hardware.** Nothing in this document has been run on a real board. The
  numbers above come from `make timing` on the host, which drives the real flight
  core with synthetic timestamps.
- **What a flood costs.** `tools/fw_sim.c`'s `flood` session feeds the console
  200,000 bytes of paste while armed and checks that no pass took more than the
  quota. It can do that, and it cannot do more: **a scripted byte in this model
  takes no simulated time**, so the flood cannot be shown to slow the loop down.
  The session's second check is therefore about the loop's rate *after* the
  console goes quiet, not during. Making a byte cost what a byte costs is the
  change that would close this, and it is not made - 42 sessions schedule
  against `virtual_ms` and `fw_sim.c:60` records that an earlier attempt at this
  accounting broke the receiver and the GPS senders once already.
- **The other three links, flooded.** The console is the one with a session. The
  network link has the same quota and no flood test, because the simulator's
  network is a loopback pair rather than a client that can be made to shout.
- **Execution duration and deadline overruns in microseconds.** `long_loops` and
  `max_loop_ms` count the loop's *period*, which is what the core can see without
  a cycle counter. A DWT cycle counter would give the duration of the work
  itself, and reading one is a port change rather than a core one; it is not
  done.
