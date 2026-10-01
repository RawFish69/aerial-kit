# 32 — The control core in the loop

The C control core, built as a shared library from the same sources the MCU
image is built from, driving a plant that lives in the other repository.

    make control-check                                    # no numpy, no sibling
    make control-experiment REPO=~/Projects/aerial-kit    # the closed loop

This is task 7 of `AERIAL-KIT-ASSESSMENT-2026-09-18.md`, whose acceptance is one
sentence: *"a reproducible rate/attitude experiment with the same C sources as
MCU."* Both halves of that sentence are load-bearing and they are checked by two
different programs, because they fail in two different ways. Task 8 added the
second plant, so the experiment now flies two aircraft rather than one.

`make control-check` answers *is the seam honest*. It loads the library,
compares the C side's own description of every struct against the binding's, and
drives the core through the ABI. It needs no numpy and no sibling, so it runs
inside `make test` and inside CI.

`make control-experiment` answers *does it fly*. It closes the loop against two
plants in the other repository — `aerial_kit.dynamics.fixed_wing
.FixedWingDynamics`, a 6-DOF rigid body, and
`aerial_kit.dynamics.multirotor_actuator.ActuatorPlant`, an actuator-level
quadrotor — so a mistake in the control core shows up as an aircraft that does
not fly, rather than as a number disagreeing with a number this repository also
wrote. That is the whole value of reaching across: a plant in this tree would
share this tree's mistakes. The wing is the harder aircraft and the quadrotor is
the cleaner seam; the next section says why both are worth the run.

## The seam

`ak_flight_step()` is the entire estimator-and-control step, and
`src/core/flight/` includes no MCU header and does no I/O. That is why the
library is possible at all: those objects link into a host process unchanged,
with a second `-fPIC` object set built under `$(HOST_OUT)/pic/`.

The ABI is `tools/ak_control.h`, version `AKC_ABI_VERSION 2`, thirteen exported
`akc_*` symbols:

    akc_abi_version  akc_product  akc_board  akc_revision  akc_built
    akc_config       akc_reset    akc_config_hash  akc_config_text
    akc_set_board_outputs         akc_step  akc_state  akc_layout

`akc_layout` is the one that is not about flying. It hands back the C compiler's
own field names, offsets and sizes for any of the seven crossing structs, and
`tools/akcontrol.py` compares them against ctypes' before it will drive
anything. A struct whose layout the two sides disagree about has no compiler
error and no crash — only numbers that look like numbers — which is exactly the
failure this mechanism exists to catch. `akcontrol_check.py` mutates a copy of
the table three ways to confirm the comparison bites: two fields swapped, one
widened to `c_double`, one struct that does not exist. Each is reported, in
words, as the thing it is.

**One aircraft per process.** `flight`, `params`, `items`, `trims`,
`configured`, `output_sequence` and `last_outputs` are file-scope statics in
`ak_control.c`, because a board has one aircraft and the firmware says so. Two
`Core` objects in one process therefore alias the same C state, which is not a
bug but is a trap: an early version of the experiment constructed a second
`Core` and reported "two different gains give identical torque" and
"determinism: false", both of which were the same aliasing. `Core` is now an
explicit singleton — a second construction with a different path raises, a
second with the same path returns the first — and `reset()` or a fresh
`config()` is how a second experiment starts. A `config()` is a complete reset:
it re-inits the flight state, re-reads the parameter table, re-applies the
airframe and the gains, and clears the output sequence.

## What crosses

Per the contract's C5, C6 and C7. Nothing here changes the firmware's own
behaviour; the boundary is where these rules live.

* **In** — `akc_samples_t` carries `t_ms` and a `sequence`, and the three
  sensors separately, because they do not run at one rate and a consumer that
  pretended they did would have to lie about two of them. Values are SI: gyro
  rad/s, accel in g, pressure Pa, temperature °C, latitude and longitude in
  1e-7 degrees. `akc_command_t` is the stick shape — `roll/pitch/yaw` −1..1,
  `throttle` 0..1, plus the two switches.
* **Out** — `akc_outputs_t` is `motor[]` 0..1 and `servo[]` −1..1, with the
  encoded forms alongside (`dshot[]`, `servo_us[]`) so the normalised value can
  be checked against what a board would actually emit.
* **Age bounds are the consumer's.** `AKC_SAMPLE_MAX_AGE_MS` is 250 ms, the
  same number as `rc_timeout_ms` and for the same reason; a sample older than
  that is refused with `AKC_SAMPLE_STALE` rather than flown.
  `AKC_COMMAND_MAX_AGE_MS` is 500 ms. The firmware has no TTL on a command
  value — a consumed command is consumed — and the contract puts that
  responsibility on the consumer, so it is here.
* **A refused step still returns a frame.** Every early return writes the
  outputs the core is holding, so "what did it do" is never a guess about
  whether the caller's buffer is fresh.
* **No truth field.** The state carries estimates and states. Where the aircraft
  actually is has nowhere to be in `akc_state_t`, which is what keeps the
  experiment an experiment.

Two conventions that are **not** in the ABI because they belong to the airframe:
a motor fraction of 1.0 is so many newtons, and a servo of 1.0 is so many
degrees. The experiment owns them, prints them in its header, and states the
sign: `ak_mixer_elevon_wing` gives both elevons a positive pitch coefficient and
a positive pitch command is nose-up, so a positive servo is trailing-edge-up —
which is the convention the sibling plant's `cm_delta_e` is defined against.

## The experiment

An elevon wing at 500 Hz — 2 ms per step, the rate the F405 runs its control
loop at. It is trimmed at 12 m/s by the plant's own `compute_trim`, not by
flying to it. Arming takes 712 ms: `arm_hold_ms` is 500, and `AK_ARM_THROTTLE`
refuses an arm request that arrives with the throttle open, so the run opens
with the throttle shut and the wing gliding on its trim speed. The first version
of this file held throttle at trim throughout, never armed, and flew nothing.

Two experiments, both measured against the **plant's** attitude rather than the
estimate's — asking the loop whether it succeeded would be asking the suspect
for an alibi.

### Rate: a roll step each way, in rate mode

    + step  setpoint +105 deg/s, mean  +75 deg/s (72%), settled in 3.22 s
    - step  setpoint -105 deg/s, mean  -68 deg/s (65%), short of 90% after 4.5 s
    torque limit 0.60, reached on 0 and 1 of 2250 steps
    tilt error    mean 25.24 deg, max 43.08 deg
    departure     airspeed 12.00 -> 47.05 m/s, pitch truth +0.0 -> -64.0 deg

The sign is asserted, not eyeballed: each step must drive the airframe in the
direction the stick asked for, and the setpoint's sign is checked too, because a
run where the core commanded the wrong sign and the airframe faithfully obeyed
would otherwise pass.

Three things in this table are worth reading carefully.

**The settle time is seconds, and that is the shipped tuning.** The loop is
converging — the error falls from 77 deg/s to under 6 over four seconds — but
the integrator winds slowly. A torque-limit explanation was checked and rejected:
the limit is reached on 1 of 2250 steps, so the shortfall is not the airframe
being asked for more than it has. The first version of this experiment used a
half-second doublet and reported 61 deg/s of error, which was a transient read
as a steady-state offset; a number like that is worse than no number, which is
why the step is now held for 4.5 s and why the second half reports honestly that
it does not get there.

**The wing departs.** Nothing in this core holds altitude or airspeed. The rate
loop holds a *rate*, so a wing whose trim elevator is not zero is free to pitch,
and pitching trades height for speed: 12 m/s to 47 m/s and 64 degrees nose-down
over nine seconds. A quadrotor does not care. A wing does, and this is the
measurement of how much. It is the clearest single argument for the assessment's
next task — an independent actuator-to-sensor plant, and then airframe-specific
control laws — because no amount of ABI correctness fixes it.

**The tilt error is large because the aircraft is manoeuvring.** 25 degrees mean
is not the estimator failing; it is a complementary filter trusting an
accelerometer on a body that is accelerating, and an accelerometer on an
accelerating body does not measure gravity. The firmware has a trust band
(`AK_ACCEL_TRUST_LOW`/`HIGH`) for exactly this and normal flight stays inside it.

### Attitude: a pitch step, in angle mode

    truth pitch  -7.2 deg before the step, +7.5 deg held after
    tilt error   mean 4.18 deg, max 7.97 deg
    departure    airspeed 12.00 -> 10.47 m/s, pitch truth +0.0 -> +1.8 deg

Same airframe, same gains, same 500 Hz — the only change is that the stick is
interpreted as an angle rather than a rate. The wing holds. The commanded tilt
is reached and kept, and the departure line goes almost nowhere, because holding
attitude is very nearly holding the trim condition. This is the contrast that
makes the rate-mode result legible rather than alarming.

### The quadrotor, on the actuator-level plant

The same core, the same 500 Hz, and a different aircraft and plant — task 8's
addition. The plant takes four motor fractions, so nothing between the mixer and
the rotors is being simulated away.

    level accelerometer  (-0.0000, -0.0000, +1.0000) g
    armed from           600 ms
    roll rate, +0.15 stick  +1.837 rad/s, settled
    roll rate, -0.15 stick  -1.838 rad/s, settled
    nose-up: front pair     +0.7164 motor fractions above the rear pair, at most
    pitch rate              +1.837 rad/s reached, +1.833 rad/s asked for (100%)
    steps with a motor against a stop   0 of 5500 flown
    hover throttle          0.4088, from the plant's own m*g/(4*max_thrust)

Three of those are worth separating from the wing's numbers, because they are
*exact* where the wing's are not, and that is the whole reason for having both.

**The rate is the rate law's own answer.** 0.15 of a stick against
`max_rate_dps` 700 is 105 deg/s, and 1.837 rad/s is 105.25 deg/s — the settled
rate matches the firmware's chain end to end (stick, decoder, rate law, mixer,
four motor fractions, rotor geometry) to within 0.2%, on a plant that was
written by reading `ak_mixer.c` rather than by importing it. Where the wing's
rate mode falls 28% short and says so, this one arrives, because nothing here is
fighting a trim condition.

**The nose-up direction is B7's correction, measured on the aircraft.** The
front pair rises. The mixer's header claimed the opposite until 2026-09-19, and
the table agreed with it, and both were wrong about the aircraft — so this is
the direction confirmed against a plant with independent geometry rather than
against the comment that was already there.

**Nothing is against a stop.** 0 of 5500. The wing's table reports its torque
limit being reached on 1 of 2250 steps and has to argue that it does not explain
the shortfall; here the question does not arise, so the rate result above is not
an authority limit being read as a control result.

And the seam is the cleanest thing in this document: motor fraction in, motor
fraction out, no scale factor and no frame conversion. The wing half needs three
invented constants to cross the same gap. That difference is what the next
section is about.

### Reproducibility

    the same experiment, a second time           ok  5500 steps compared, bit for bit
    the same quadrotor experiment, a second time ok  5500 steps compared, bit for bit

Each aircraft is run a second time from a fresh plant and compared step by step:
time, result, motors, servos, the plant's quaternion and the estimate's. Not
"close" — equal. This is what "reproducible" has to mean if a result is going to
be filed under a configuration identity, and it is also what pins the ABI down:
anything that reads uninitialised memory or depends on process state will fail
here.

Each run files itself under its own configuration, and the two are different
configurations rather than one experiment reported twice:

    wing   config hash   0x6bdef714   airframe=1
           plant         aerial_kit.dynamics.fixed_wing.FixedWingDynamics
    quad   config hash   0xb61e3f01   airframe=0
           plant         aerial_kit.dynamics.multirotor_actuator.ActuatorPlant

— C8's identity, the library's own revision, and the plant. Without those the
numbers above are an anecdote, and the two hashes are the cheapest proof that
the aircraft really is being reconfigured rather than merely re-planted.

Each half also reads back from the core the constants *it* reasons with and
checks them, because a default that moved would change what every number above
means without changing how any of them reads. The wing half reads two and prints
`ok` on both; the quadrotor half reads one, `max_rate_dps`, because its other
number is a measurement (`0 of 5500` against a stop) rather than a limit
compared against a constant:

    wing   ok  the two defaults this file explains itself with are the ones the core is using
    quad   ok  the rate this file explains itself with is the one the core is using (max_rate_dps=700)

That check is written `not (abs(actual - expected) <= 1e-3)` rather than
`abs(actual - expected) > 1e-3`, and the difference is not style: a key the core
never printed parses to `nan`, and `nan > 1e-3` is **false**, so the obvious
spelling would pass a missing value. Measured: `nan` gives `False` for the
obvious form and `True` for the one used.

## What was wrong, and is now not

Five defects were found by building this, and four of them were invisible from
inside the firmware.

**`rate_kp_roll` never reached the control law.** The most serious one.
`ak_flight_init` copies the gains into each axis's `ak_pid_t` once, and
`ak_pid_update` reads the copy; `parameters_changed()` re-applied the airframe
but never re-seeded the PIDs. So a gain written over the console changed the
parameter table, the saved text and the configuration hash, and changed nothing
about how the aircraft flew. Measured before the fix: `rate_kp_roll` 0.25 and
0.90 both produced a torque of 0.60000 and the identical hash `0xde3d0201`.
After: 0.1529 and 0.5504. `docs/06-console.md` had been claiming the opposite in
so many words — *"the same memory the control loop reads every iteration"* — and
now carries the correction and the caveat that a gain change resets the
integrators.

**The layout table was silently short.** `gps` declared 11 rows and had 10;
`state` declared 22 and stopped at 22 of 38 actual fields. A partial table is
precisely the silent-prefix failure the mechanism exists to prevent, and it
would have passed every check that did not already know the answer. The counts
are now computed at runtime by walking to the null terminator, and
`akcontrol_check.py` asserts the expected count per struct.

**The accelerometer's sign.** `ak_types.h` calls the field "g" and says no more.
`docs/31-contract.md` C1 is normative — *"z reads +1 with the board level and the
right way up"* — and `ak_estimator.c`'s `est_body_up` agrees, but the first
version of the experiment fed it a body-FRD specific force, which is the
negation. The estimator concluded the aircraft was upside down and the run said
so: a steady 120 degrees of attitude error, roll off by 180 while pitch agreed.
Nothing in the firmware is wrong here; the boundary had a convention that was
written down in two places and read in neither.

**Two `Core` objects aliased one aircraft.** Described above. Fixed by making the
singleton explicit rather than by documenting it.

**`reset()` silently discarded the configuration.** `ak_flight_init` calls
`ak_flight_default_config`, so a reset wiped `cfg` while `configured` stayed 1
and `config_hash` would have changed with nothing reporting it. The ABI now
keeps the saved text and re-applies it, which is what `reset()`'s own
documentation promised.

## What this does not prove

**It is not the MCU.** The sources are the same; the build is not. This is a
host `-fPIC` object set compiled by the host toolchain, not the F405 image, and
nothing here exercises interrupts, DMA, timer jitter or the 1 kHz loop's actual
timing. The assessment's B4 work made timing observable and bounded; measuring
it needs the board.

**It is not an independent plant for the estimator.** The plant is a real rigid
body in another repository, which is a genuine independence for the *control*
path. But the estimator and the plant share a frame convention, and the
experiment is what found the sign error — so the loop is "the firmware's
estimate against another repository's truth", not "against the world".

**The quadrotor had to be given a plant before it could be flown here.** The
sibling's *existing* multirotor plant takes `set_command(body_rates, thrust)` — a
hover-normalised thrust — so its input is already the flight controller's own
output shape, and closing the loop around it would have tested nothing about the
actuator path. That was true, and it was the reason the wing was the first
aircraft used. It is no longer the whole picture: an actuator-level multirotor
plant now exists beside it, `aerial_kit.dynamics.multirotor_actuator`, whose
input is **four motor fractions** — the firmware's own units, the same four
numbers `ak_mixer.c` produces. The quadrotor is flown on it in the same
experiment, and the seam is worth stating because it is unusually clean:

    plant   aerial_kit.dynamics.multirotor_actuator.ActuatorPlant
    seam    motor fraction in, motor fraction out: no scale factor, no frame
            conversion

No scale factor and no frame conversion, where the wing half of the experiment
needs three invented constants (`MAX_THRUST_N`, `MAX_ELEVON_RAD`, and a Shepperd
quaternion conversion). That is not a coincidence and it is the point of the
plant: it is NED/FRD-native, like the firmware, rather than z-up like every other
model in the sibling. A frame convention both sides documented and neither
checked is what produced the 120-degree flip in B7's first run.

**What the quadrotor half measured.** A +0.15 roll stick settles at
**+1.837 rad/s** against the +1.833 rad/s the rate law asked for, and the same
the other way; a nose-up command raises the **front** pair by up to **+0.7164**
motor fractions, which is the corrected direction B7 had to fix in the mixer's
own header comment; and **0 of 5500** steps put a motor against a stop, so the
authority limits are not being touched and are not what produces any of it. Run
twice, 5500 steps compare bit for bit.

**And what it found is that the two repositories' allocators do not meet.** The
assessment asked to "ensure allocator outputs actually drive the multirotor".
Measured, the answer is that they cannot, yet, and the reason is three
conventions that differ across that seam and that neither repository had written
down, because until there was an actuator-level plant the return of
`MultirotorAirframe.allocate` was **discarded** and the mismatches were
unreachable:

* `Wrench.force_body[2]` is upward thrust in the sibling (z-up) and a negative z
  force in the firmware (FRD), so the same hover is `+9.81` there and `-9.81`
  here;
* `allocate()` returns **newtons** (they sum to `force_body[2]`; `trim()` is
  `m*g/4`) while `UAVDynamics` takes a **hover fraction**, a factor of 2.4525 on
  a 1 kg quadrotor;
* ~~the two quad-X yaw columns disagree about which pair is
  counter-clockwise~~ - **this third item was wrong**, corrected 2026-09-19.
  The two columns were being compared without aligning the rotors, and a
  quad-X is numbered differently in each tree (by polygon angle here, by corner
  there). Read in the same rotor order they are elementwise negatives - the
  same pairing and a single global sign - which is exactly what a moment picks
  up from the z-negation between the sibling's z-up frame and FRD. It is
  therefore accounted for by the frame conversion and must not be corrected
  again. "Fixing" it inverts the yaw axis at twice the demand, measured. See
  trap 76 and trap 80.

Wiring one to the other is now done, and it needs **no** spin-table decision:
`aerial_kit/dynamics/quad_x_seam.py` in the sibling carries the frame rotation,
a rotor permutation **matched by position** rather than tabulated, and the
newtons-to-fraction division. A yaw demand of 0.02 N*m arrives at the plant as
0.02 N*m to 1e-12 on every axis, and the plant flown open-loop turns the way the
sibling's own convention says. `sim_py/tests/test_quad_x_seam.py` is the
evidence, including the measurement that negating the sibling's spin column
breaks yaw - the change the old reading of trap 76 implied.

That seam is for the **sibling allocator** as a producer. The C-core path above
is a different one and genuinely has no conversion, because `ak_flight_step()`
speaks the plant's frames and units end to end - which is why the experiment's
quadrotor half prints `seam  motor fraction in, motor fraction out`.

**The tuning numbers are not a recommendation.** 8.0 N and 25 degrees are round
numbers chosen so the loop closes, not fitted values. Tuning them is a bench's
job, and one measured F405 qualification is task 9.

## Contracts

* **C5** — implemented at the host boundary: `AKC_SAMPLE_MAX_AGE_MS`,
  `AKC_COMMAND_MAX_AGE_MS`, per-sensor ages and stale flags in the state. The
  firmware is unchanged and this does not ask it to change.
* **C7** — implemented: `motor[]`/`servo[]` cross normalised, and
  `tools/akcontrol_experiment.py` is a worked conversion. The wing half prints
  both of its scale constants; the quadrotor half prints neither, because on a
  plant that shares the firmware's frames and units there are none to print.
* **C8** — implemented as `ak_params_hash`, reachable as `identity:` on the
  console and as `akc_config_hash()` across the ABI. **The wire protocol still
  carries `params.changed` and not the hash**, so a configurator over the network
  cannot yet name the configuration it is looking at; that is a protocol version
  bump and a client change on both sides, and it is the owner's call at a bench
  session.

## Where it lives

    tools/ak_control.h            the ABI, version 2
    tools/ak_control.c            the seam: config, step, state, layout
    tools/akcontrol.py            the ctypes binding, with the singleton
    tools/akcontrol_check.py      70 checks; runs in make test and CI
    tools/akcontrol_experiment.py the closed loop; numpy and the sibling,
                                  both aircraft in one file on purpose
    src/core/flight/ak_flight.c   ak_flight_apply_config()
    src/core/ak_params.h          ak_params_hash()

The two plants are in the sibling repository and are **not** in this one:

    aerial_kit/dynamics/fixed_wing.py            the wing, 6-DOF, z-up (ENU)
    aerial_kit/dynamics/multirotor_actuator.py   the quadrotor, NED/FRD-native

That second file is task 8's deliverable, and it is uncommitted in a public
repository the owner has not authorized a push to — so the numbers it produces
are reproducible from a working tree and not yet from a clone. The 41 tests that
carry it are in `sim_py/tests/test_multirotor_actuator.py` beside the rest of
that package's suite (189 before it, 230 after).

`make control-check` is a CI stage of its own rather than a line in `make test`
alone, because it is the only check here that crosses a language boundary.
`make python-syntax` compiles every tool without importing any of them, so a
syntax error in the experiment is caught where numpy is not installed — the
experiment itself cannot run in CI, and a file whose only reader is the one
person who can run it is a file that rots.
