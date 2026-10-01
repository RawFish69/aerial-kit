# AerialKit - the cross-repository contract

    make contract-check

Two repositories have to agree about numbers: this firmware, and the Python
package (`aerial-kit`, reviewed here at `57e1811`) that simulates airframes and
talks to firmware. They already agree in most places by accident of both being
written by people who know the conventions. This page is the part that has to be
true on purpose, because a disagreement here is not a compile error — it is an
aircraft that banks the wrong way, or an experiment that reports a number in
units nobody wrote down.

It is the 2026-09-18 assessment's task 6: *"Define the minimal cross-repository
contract. Document NED/FRD and ROS transforms, SI units, quaternion convention,
samples/outputs, command kinds, authority/expiry, config generation and
capability states. Implement serialization/golden-vector tests, not a full
networking framework."*

**The contract is a boundary, not a merger.** Neither tree is asked to change
its internal conventions. The Python simulator is world-ENU because ROS and
every visualiser it feeds are; this firmware is earth-NED because aviation is
and because the accelerometer's sign falls out of it. What the contract fixes
is the *crossing*: which transform, which units, which order, and where the
conversion is allowed to happen.

## The normative summary

| # | Item | The contract |
| --- | --- | --- |
| C1 | Body frame | **FRD** — x forward, y right, z down. Both sides, no exception. |
| C2 | World frame | This firmware is **NED**; the Python simulator is **ENU**. The crossing is the fixed rotation in §C2, implemented on the Python side at `ros2_ws/src/mavlink_bridge/mavlink_bridge/frame_transforms.py`. |
| C3 | Attitude | Unit **quaternion, w-first** `(w, x, y, z)`, body→world, no Euler in transit. `(x, y, z, w)` appears only at the ROS message boundary. |
| C4 | Units | **SI at every crossing**: metres, m/s, radians, rad/s, seconds, newtons. Each wire surface's integer scaling is in §C4 and is part of the contract, not a detail of the encoder. |
| C5 | Samples | Every sample carries a **timestamp and a sequence**; a consumer rejects a sample older than the stated age. **Implemented at the host boundary** — see §C5; the firmware still has no TTL on a sample, and the consumer is where it lives. |
| C6 | Commands | The stick shape (`roll/pitch/yaw` -1..1, `throttle` 0..1) is the command contract. Authority and expiry in §C6. |
| C7 | Actuators | **Normalised at the boundary**: motors 0..1, servos -1..1. Newtons and radians stay inside the airframe that knows its own geometry. **Implemented at the host boundary** — see §C7, and `tools/akcontrol_experiment.py` for a worked conversion. |
| C8 | Config identity | A configuration must be nameable. **Implemented** as `ak_params_hash` — see §C8. |
| C9 | Capability | The state and capability vocabulary in §C9, so a consumer can say what the aircraft is doing without parsing prose. |

## C1 — Frames

The firmware's statement, which is the oldest one in the tree and is normative:

    body frame    x forward, y right, z down
    gyro          rad/s about x, y, z
    accel         g about x, y, z (z reads +1 with the board level and the
                  right way up, because z points down)

— `src/core/flight/ak_types.h:13-20`. The Python fixed-wing dynamics state the
same body frame: *"Body axes are FRD (x-forward, y-right, z-down) — the standard
convention for the alpha/beta/lift/drag formulas below"*
(`aerial_kit/dynamics/fixed_wing.py:3`).

**Sign conventions that are part of the contract**, because they are the ones a
reader guesses wrong:

* **pitch positive nose-up** (`src/core/flight/ak_estimator.c:129-133`; the
  firmware notes that Betaflight's is nose-down)
* **roll positive = right wing down** (`ak_estimator.c:150`, `atan2(uy, uz)`)
* **yaw positive = nose right**, heading 0 = north, bearings in hundredths of a
  degree (`src/core/flight/ak_nav.h:286-288`)

## C2 — NED to ENU, and the one rotation that matters

The firmware's attitude is body-FRD → earth-NED. The Python simulator's is
body-FRD → world-ENU. Those are different quaternions for the same physical
attitude, and the difference is a fixed rotation that the Python repository has
already written down and tested:

    _NED_ENU_Q = (0, √2/2, √2/2, 0)
    _AIRCRAFT_BASELINK_Q = (0, 1, 0, 0)          # FRD body -> FLU baselink

    q_enu = _NED_ENU_Q ⊗ q_ned ⊗ _AIRCRAFT_BASELINK_Q

— `ros2_ws/src/mavlink_bridge/mavlink_bridge/frame_transforms.py:16-57`. This
is **the** transform; the contract does not introduce a second one. A consumer
that receives this firmware's quaternion and wants ROS's converts exactly here,
and the identity is checkable: at level attitude, heading north, the firmware's
quaternion is `(√2/2, 0, √2/2, 0)`-class and the Python fixed wing's is not the
identity either — *"level, heading +world-x, is a 180-degree rotation about the
shared x-axis"* (`aerial_kit/dynamics/fixed_wing.py:11-13`).

Position and velocity cross the same way:

    (x_e, y_n, z_u) = (y_ned, x_ned, -z_ned)         # frame_transforms.py:23-25

### The yaw origin differs by a quarter turn

This is the one place the two repositories disagree about something a reader
would assume they agreed on, and it was found by running the vectors rather
than by reading either side.

**This firmware's yaw is from north, clockwise.** *"yaw positive = nose right,
heading 0 = north"* (`ak_nav.h:286-288`) — the aviation convention, and the one
`ak_gps_course()` returns.

**The simulator's `heading_rad` is from +world-x, counter-clockwise.** In ENU,
+world-x is *east* (`aerial_kit/dynamics/fixed_wing.py:79-88`).

They differ by exactly 90°, and the relation is a contract term:

    heading_rad = pi/2 - yaw

Checked over six headings including a negative one
(`contract/aerialkit-contract-v1.json`, `ned_enu.level_attitudes`): the
firmware's level-north attitude taken through §C2 equals the simulator's
`level_attitude_quat(pi/2)`, not `level_attitude_quat(0)`. A consumer that
passes one into the other without the conversion flies ninety degrees off, and
nothing in either tree would have said so — the two are both unit quaternions,
both plausible, and both about a level aircraft.

**Where the conversion may happen.** At a boundary, once, named. Never inside a
control loop and never twice. This firmware does not carry ENU anywhere and does
not need to; a Python consumer does not carry NED anywhere except at the
MAVLink/ROS edge it already has.

## C3 — Attitude representation

The firmware carries the attitude as a **quaternion `(w, x, y, z)`**, w first:
*"The attitude, carried as a quaternion (w, x, y, z), because body rates are not
Euler derivatives"* (`src/core/flight/ak_estimator.h:26-43`; identity at
`ak_estimator.c:256-259` is `q[0] = 1.0f`).

Euler angles exist on both sides and are **derived, never integrated**:

* firmware: `roll`, `pitch`, `yaw` fields filled from `q`, ZYX order
  (`ak_estimator.c:129-131`, `:162-175`). Yaw is carried **unwrapped** so a
  multi-turn heading is not lost, while the quaternion's own yaw is wrapped to
  (-π, π] (`:154-160`, `:243-248`).
* Python: the fixed-wing controller *"avoids extracting classical Euler angles …
  it uses body axes projected onto the world frame directly"*
  (`aerial_kit/controllers/fixed_wing.py:16-23`).

**The contract is the quaternion.** Any crossing uses it; an Euler angle is a
display format and its order is stated wherever it appears. The Python side
stores `[w, x, y, z]` (`aerial_kit/types.py:43-47`) and the one place it must
swap is the RotorPy boundary, which says so: *"RotorPy stores quaternions as
[x, y, z, w]"* (`sim_py/backends/rotorpy_backend.py:80-81`).

## C4 — Units, and the scaling at each wire surface

**SI at every crossing.** No crossing carries degrees, per-mille or scaled
integers as its *meaning* — where an integer encoding exists it is a transport
detail and is written down here.

| Surface | Attitude | Position | Note |
| --- | --- | --- | --- |
| Console `status` | **milliradians**, `roll/pitch/yaw` | - | `ak_cli.c:137-140` |
| Console `status` (outputs) | motors and servos in **per-mille** | - | `ak_cli.c:141-145` |
| Protocol STATUS / telemetry | **tenths of a degree**, `int16`, wrapped to ±1800 | `lat_e7`/`lon_e7`, 1e-7 degrees | `ak_proto.h:82-86`; the conversion is one function, `ak_attitude_ddeg()`, so the two callers cannot disagree (`ak_math.h:114-117`) |
| Console `status` (yaw) | **milliradians, not wrapped** | - | the console prints the estimator's continuous yaw, so it exceeds ±3141 past half a turn — the one surface where a heading keeps its turns |
| Blackbox record | **0.1 deg** roll/pitch, 0.1 deg yaw | `alt_mm`, `lat_e7`/`lon_e7` | `ak_log.h:37-75` |
| Blackbox record (rest) | gyro **0.1 dps**, accel **0.001 g**, sticks **-1000..1000 per-mille**, torque **-100..100 percent**, motors **0..254** with 255 = no output | | the CSV header states all of it (`ak_log.c:82-107`) |
| Parameter table | **degrees** for waypoints, **deg/s** for rate gains, **g** and **dps** for calibration results | | units live in each parameter's help text; the choice of degrees over the protocol's 1e-7 is deliberate — *"because these are typed and read by people"* (`main.c:3002-3005`) |

The integer encodings are **saturating and total**, not best-effort: the ddeg
conversion wraps so it always lands inside ±1800, and motors are clamped to 254.
A consumer that decodes one of these must apply the same rule, which is what the
golden vectors in §C10 check.

### Two edges of that conversion, measured

Both of these came out of the vectors rather than out of reading the code, and
both are the kind of thing a decoder written from the prose gets wrong:

* **±1800 is reachable, and the wrap boundary is one ULP wide.** An angle of
  exactly `AK_PI` (the float32 `0x40490fdb`) encodes as **1800**; the next
  float32 up, `0x40490fdc`, encodes as **−1799**. So the field's range is
  `[−1800, 1800]` and *not* the `[−1799, 1799]` a reader would guess from
  "wrapped to ±180". A decoder that clamps to ±1799 loses a value the firmware
  really sends.
* **The conversion truncates toward zero, so small angles lose their sign to
  the unit.** 359.9° encodes as 3599 tenths only after wrapping, and wrapping
  puts it at −0.00138 rad, which is −0.79 tenths, which truncates to **0**. A
  heading a hair short of a full turn reads as due north, and 0.05° of
  resolution is gone at every multiple of a turn. This is inherent to
  truncation and is why the field is a wrapped angle rather than a bearing.

A consumer must reproduce the firmware's *arithmetic*, not just its formula:
this conversion is C `float` throughout, and a float64 reimplementation gets
the `pi` case wrong by one unit — which is a real wrap to the wrong side, not a
rounding difference. `tools/contract_check.py` models float32 deliberately and
says so; falsifying that model (using float64) fails 16 of its 159 checks.

## C5 — Samples, timestamps and staleness

The firmware's rule, and the reason it exists: a number that is merely *present*
is not a number that is *current*.

* **RC link freshness**: `link_live = decoded && (now_ms - last_update_ms) <= rc_timeout_ms`
  (default 250 ms, `ak_flight.c:490-494`). The arm gate reads freshness, not
  presence.
* **GPS fix age**: `ak_gps_fix_valid(gps, now_ms, max_age_ms)` — a stale fix
  stops a return from starting (`src/core/sensors/ak_gps.h:119`).
* **Partial protocol frame**: abandoned after `AK_PROTO_GAP_MS` (50 ms) of
  silence (`ak_proto.c:380-386`).
* **Time base**: `uint32_t` milliseconds, monotonic, `ak_time_ms()`.

The Python simulator's standalone core has **no timestamps and no staleness** —
`SimState.t` is the sample time and nothing else (`aerial_kit/types.py:51`). It
does not need them to integrate; it will need them the moment it is driven by
the C core or by hardware, which is task 7.

**So the contract states the direction of travel:** a sample crossing between
the two carries `(t_ms, sequence)` and the consumer applies an age bound. The
firmware already does this internally on every link; the contract asks for the
same two fields on the crossing rather than inventing a third convention.

**Implemented at the host boundary.** `tools/ak_control.h` carries
`AKC_SAMPLE_MAX_AGE_MS` (250 ms, the same number as `rc_timeout_ms`, and for the
same reason) and refuses a step whose inertial sample is older than that with
`AKC_SAMPLE_STALE`. The refusal is per-sensor and named: a missing barometer and
a two-second-old barometer are different answers, and the state reports
`imu_age_ms`, `baro_age_ms`, `gps_age_ms` with an `*_stale` flag each, so a
consumer can see which one it was.

`AKC_COMMAND_MAX_AGE_MS` (500 ms) is the TTL on a command value, which the
paragraph above says the firmware has and does not. It lives in the consumer
because that is where the paragraph puts it: this ABI consumes the command it is
handed and stamps it, and a command past its TTL is refused rather than replayed
on the firmware's behalf. A refused step still returns the outputs the core is
holding, so "what did it do" is never a guess.

The firmware side is unchanged and this contract does not ask it to change. A
sample arriving at `ak_flight_step` still has no age bound of its own; that is
the MCU loop's job and it does it with `link_live`.

## C6 — Command kinds, authority and expiry

**The command contract is the stick shape**, and it is the same on both sides:

    roll -1..1, pitch -1..1, yaw -1..1, throttle 0..1

— `ak_rc_command_t`, `src/core/flight/ak_rc.h:27-34`; the guidance output is
deliberately the same shape, *"sticks and a throttle"* (`ak_nav.h:11-16`). This
is also what the Python repository's own firmware-side documents use:
*"Throttle is 0..1"* (`docs/FIRMWARE_WING_CONTROL.md:9-16`).

**Authority, in the firmware's terms — a consumer must not assume it can do what
a console can do:**

* **No console command can arm.** Arming is the arm switch on the receiver and
  nothing else (`docs/06-console.md:194-210`); `output test` and the calibration
  commands refuse while armed.
* **The configuration may be written only while disarmed.** One policy, one
  function: `flight->state == AK_FLIGHT_DISARMED` (`ak_flight.c:736-750`), and
  every route into a save asks it (`ak_params_store`'s `writable` argument — see
  `04-traps.md` §59 for what happened when a route did not).
* **Guidance never overrides a live pilot** unless the pilot has handed over
  (`managed`, `ak_flight.c:709-716`).
* Arm gates, in order, are a public vocabulary: outputs, no link, failsafe, not
  requested, throttle, not converged, not level (`ak_flight.h:295-309`).

**Expiry.** The firmware has *link* expiration (above) and **no TTL on a command
value itself** — a consumed command is consumed. A consumer that buffers a
command is responsible for its own clock; the contract does not let one be
replayed late on the firmware's behalf. The Python repository's ROS bridge
already does this with `command_timeout_sec` (0.5 s,
`mavlink_bridge_node.py:52`), which is the same rule on the other side.

## C7 — Actuators, and the units that do not cross

The firmware's actuator domains are normalised: **motors 0..1** (0 = stopped),
**servos -1..1** (0 = centre), stated in `ak_types.h:13-20`.

The Python airframes are *not*: `MultirotorAirframe.allocate()` returns per-rotor
thrust in **newtons**, `TwinWingAirframe.allocate()` returns
`[throttle_L, throttle_R, elevon_L, elevon_R]` in **newtons and radians**
(`aerial_kit/airframes/multirotor.py:56-77`, `aerial_kit/airframes/fixed_wing.py:73-98`).

**Both are correct and neither should change.** Newtons are the right output for
an allocator that knows the vehicle's geometry; a normalised command is the
right input for a board whose PWM range is a build-time fact and whose ESC
calibration is not the flight controller's business. So the contract puts the
normalisation **at the boundary**: the crossing speaks `motors 0..1` and
`servos -1..1`, and whichever side owns the airframe applies its own scale and
states it. Task 7 is where this becomes an interface; the contract fixes which
side of it the units live on.

**Implemented, and worked once end to end.** `akc_outputs_t` crosses as
`motor[]` 0..1 and `servo[]` -1..1, and alongside them the encoded forms a board
would actually emit — `dshot[]` and `servo_us[]` — so a consumer can check the
normalised value against the thing it becomes without re-deriving the scaling.
`tools/akcontrol_experiment.py` is the worked conversion in the other direction:
it takes the sibling repository's `FixedWingDynamics`, which wants
`[throttle_L_N, throttle_R_N, elevon_L_rad, elevon_R_rad]`, and maps
`motor 1.0 = 8.0 N` and `servo 1.0 = 25 deg` with both constants printed in the
run header. Those two numbers are the airframe's and belong to the airframe;
what the contract fixes is that they are stated at the conversion and not
assumed anywhere on the other side of it.

The **sign** is the part worth writing down, because it is the part a scale
factor does not settle. `ak_mixer_elevon_wing` gives both elevons a positive
pitch coefficient and a positive pitch command is nose-up, so a positive servo
is trailing-edge-up — which is the convention the sibling plant's `cm_delta_e`
is defined against. The experiment relies on that agreement and says so; when
the two disagree the aircraft flies the wrong way and the sign is where to
look.

## C8 — Config generation: the one requirement

**The firmware reports no identity for its configuration.** The survey found
exactly three signals, and only the first is visible outside the device:

* `params.changed` — how many parameters differ from the last save; printed by
  console `status` (`ak_cli.c:152`) and carried in the protocol's HELLO reply as
  text (`ak_proto.c:201-202`).
* the load report — applied/unknown/unmentioned across an upgrade
  (`ak_params.c:478-487`), printed in `status` and `load`.
* the flash record's `serial` and `sum` — a per-save counter and an FNV-1a over
  the saved text, both **internal**: they pick the newest valid slot and nothing
  reports them (`src/boards/AERIALKIT_F405/board.c:252-269`, `:396`).

That is not enough for the thing both repositories want next. An experiment
result has to name the configuration it ran (`AERIAL-KIT-ASSESSMENT-2026-09-18.md:640`:
*"firmware identity/config hash in result"*), and `changed = 0` is not an
identity — it is the absence of a difference from something unnamed.

**The requirement, stated so it can be implemented and tested:** a stable
32-bit hash over the *effective* configuration, reported on the console and
available to a consumer, with two properties: it changes when any non-secret
parameter changes, and **a secret parameter's value cannot be recovered from
it** — so the hash covers a secret's *name* and a fixed marker, not its contents,
and therefore does not change when only a password changes. That last is a real
trade-off and belongs in the contract rather than in a comment: the alternative
is publishing an offline-guessable digest of a Wi-Fi password.

**Status: implemented, as `ak_params_hash` (`src/core/ak_params.h:159-179`).**
It is FNV-1a — the same algorithm the F405's flash record already uses, so there
is one answer here rather than two — over the effective table, with a secret
contributing its name and the `***` marker and never its contents.

The two properties are checkable, and `tools/akcontrol_check.py` checks them
rather than describing them:

* the hash moves when a non-secret parameter moves, and does not move when the
  *order* of the lines changes or when an unknown name appears, because it
  covers the effective table rather than the text that produced it;
* over a configuration with no secrets set it equals FNV-1a over
  `ak_params_serialize`'s bytes exactly — the check recomputes it in Python from
  the text and compares. The two differ only at a secret, where `serialize`
  writes the value and `ak_params_hash` writes the marker.

It is reachable the two ways a consumer needs. The console's `status` prints it
as `identity:  0x%08x` (`src/core/ak_cli.c:163`), so a person at a bench can
read it off; and `akc_config_hash()` carries it across the host ABI, and also
into `akc_state_t.config_hash` so a logged run carries its own identity rather
than needing it fetched separately (`tools/ak_control.h:203`).

The one thing this does **not** yet cover is the wire protocol: the HELLO reply
still carries `params.changed` and not the hash, so a configurator over the
network cannot name the configuration it is looking at. That is a protocol
version bump and a client change on both sides, and it is the owner's call at
the first bench session rather than something to slip in behind one.

## C9 — Capability and state vocabulary

A consumer must be able to say what the aircraft is doing without parsing prose.
The firmware's vocabulary, all of it already on the wire:

* **Flight state** — six values, and the names are part of the contract:
  `disarmed`, `armed`, `failsafe`, `returning home`, `on autopilot`,
  `circling down` (`ak_flight.h:23-39`, `ak_flight.c:754-776`).
* **Link state** — `link_live` (freshness, per C5).
* **Position quality** — `gps_fix_type` (u-blox 0-5) and `gps_satellites`.
* **Sensor presence and health** — estimator `converged`, the blackbox's
  `RC_LIVE | IMU_VALID | GPS_VALID` flags, and the arm-block reason list, which
  is the firmware's one public explanation of *why not*.
* **Identity** — `product`, `board`, `rev` (git describe), `built`
  (`src/core/ak_version.h:36-39`), in the banner, the `version` command and the
  protocol's HELLO.
* **Transport capability** — `can_stream` per link: the network streams, the
  console does not (`ak_proto.h:141-148`).

## C10 — The vectors

    make contract-check

`contract/aerialkit-contract-v1.json` holds the golden vectors and
`tools/contract_check.py` is the independent implementation that checks them -
159 checks, and the run is `159 checks, 0 failed`.

**Three implementations, not two.** The file is produced by *this firmware's
own encoders* (`tools/contract_vectors.c` calls `ak_attitude_ddeg()`,
`ak_proto_telemetry_frame()` and `ak_log_encode_record()`) and by *the other
repository's own module* (imported, not reimplemented). Neither of those is a
test: they are the things being tested. The checker is the third, and it shares
no code with either — it re-derives the CRC, the packing, the field order, the
float32 rounding and the quaternion algebra from the layouts in this page and
`docs/16-protocol.md`.

That split is what makes the file evidence. A vector someone typed is a second
opinion about the format; a vector the firmware printed is the format.

The groups, in the order of the sections above:

1. **The attitudes** (23 cases) — the protocol's tenths of a degree and the
   console's milliradians over the cardinal angles, both edges of the wrap, a
   multi-turn angle, the wrap limit and past it, a subnormal, and a NaN. The
   inputs are stored as **float32 bit patterns**, because "the input was pi/2"
   is a claim about how a decimal literal rounded and a bit pattern is not.
2. **The telemetry frame** (33 bytes) — built from a status whose signed
   fields are exercised in both directions and whose motors cover `0`, `254`
   and `255`. Checked byte for byte against the firmware's, and then field by
   field so a failure names the field rather than the hex string.
3. **The blackbox record** — 51 bytes on the wire, byte for byte, **and the
   fact that the C struct is 56**. Five bytes of padding sit between `yaw` and
   `alt_mm` and after `flags`, and the tree encodes field by field rather than
   copying the struct. A client that assumes the two are the same reads every
   field after the first gap as noise, and this is the check that keeps them
   different.
4. **The NED→ENU rotation** — nine attitudes through §C2's composition, six
   level headings that pin the quarter-turn yaw origin, and four positions.
   Plus the negative control: dropping the baselink term must change the
   answer, or that term is doing nothing and the two frames have been
   silently conflated.

**The check's own check.** Eight mutations are applied to a copy of the document
on every run — a tenth of a degree in the angle table, a nibble in the frame, a
byte in the wire record, the struct and the wire conflated, a field offset
moved, a rotated attitude, the yaw origin off by a quarter turn, a position
mapped the wrong way — and each must produce a failure. Without that, a checker
whose comparisons never ran would report zero failures and look exactly like a
contract that holds. (It caught one of those on the first run: the frame
mutation replaced the first byte of a frame that already began with `aa`, so it
changed nothing and the self-test reported a mutation it had not made.)

And it was falsified in both directions, as `docs/30-boards.md`'s validator was:

| Falsification | What was removed | Result |
| --- | --- | --- |
| A | the float32 modelling — every value kept in float64 | **16 checks failed**, starting with `pi`: float64 wraps it to −1799 where the firmware answers 1800 |
| B | the baselink term from §C2's composition | **18 checks failed**: every rotated attitude, and the check that dropping the term changes the answer |

What the vectors deliberately do **not** cover: anything the firmware cannot be
asked for. They are produced by running this tree's encoders, so a vector no
interface can produce would be a claim rather than a test.

**Regenerating is a deliberate act**, and `make contract-vectors` needs the
other repository checked out beside this one, because the NED→ENU group is that
repository's statement to make. The check does not: it reads the committed file
and runs anywhere.

## What this contract does not do

* It does not make the Python simulator NED, or this firmware ENU.
* It does not fix the Python repository's own internal inconsistencies — and
  there are two the survey found, both of which this contract *pins at the
  boundary* rather than repairs: the multirotor dynamics thrust along body +Z
  while labelling its rates FRD (`aerial_kit/dynamics/multirotor.py:36`, `:125`,
  which is FLU geometry), and `sim_py/teleop/model.py:17` says the body axes are
  *"ENU-like: +X forward, +Y left, +Z up"*. The fixed-wing side is unambiguous.
  A firmware counterpart must therefore state which body frame it is talking to,
  which is why C1 is first.
* It does not add a networking framework. The vectors are a file and a checker.
* It does not settle the wire change that C8 might need. Adding a field to the
  protocol's HELLO is a `AK_PROTO_VERSION` bump and a client change
  (`tools/akproto.py`, `apps/configurator`), and that is a decision for the
  owner with the first bench session rather than a side effect of writing a
  contract.
