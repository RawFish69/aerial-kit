#!/usr/bin/env python3
"""A rate and attitude experiment: the C control core flying a Python plant.

    make control-experiment REPO=~/Projects/aerial-kit

This is the acceptance experiment for B7, and it is the half that
`tools/akcontrol_check.py` cannot state. That file drives the core and reads it
back, which shows the seam is honest; it does not show that the core *flies*
anything. Here the loop is closed: the plant's body rates and specific force go
in as an inertial sample, the core's motors and servos go out as the plant's
actuator command, and one 500 Hz time step is one of each.

**The plant is the other repository's, and it is not a mock.** It is
`aerial_kit.dynamics.fixed_wing.FixedWingDynamics` - a 6-DOF rigid body with a
lift/drag/moment model - so a bug in the control core shows up as an aircraft
that does not fly rather than as a number that disagrees with a number this
repository also wrote. That is the whole value of reaching across: a plant in
this tree would share this tree's mistakes.

**Two aircraft, and why both.** The wing is driven through
`FixedWingDynamics`, which takes `[throttle_L_N, throttle_R_N, elevon_L_rad,
elevon_R_rad]`: newtons and radians, the airframe's units, which is the far side
of C7's line and costs this file two invented constants to cross.

The quadrotor was *not* flyable here when this experiment was written, and the
reason was in the sibling repository rather than in this one: its multirotor
plant takes `set_command(body_rates, thrust)` - a hover-normalised thrust - so
its input is already the flight controller's own output shape, and closing the
loop around it would have tested the controller against itself (F2). B8's
deliverable is the plant that fixes that: `ActuatorPlant` takes four motor
fractions and integrates a rigid body, a rotor geometry and a motor lag. It is
NED and FRD because its consumer is this firmware, so the seam needs **no**
scale factor and **no** frame conversion - the firmware's `out.motor[i]` and the
plant's `set_motors()` are the same quantity in the same units.

So both are flown, and the second one is the harder test. A mixer sign error
that the wing would hide - because its elevons and its motors are separate
outputs - lands in the quadrotor's four motors and shows up as an aircraft
rolling the wrong way. `ak_mixer.h`'s pitch comment was wrong about the aircraft
for months and the table agreed with it (trap 73); this is the run that would
have caught that.

**What is measured, and what is only reported.** The run asserts three things:
that the loop closes at all, that a stick command moves the aircraft the way
the command means, and that two runs of it are identical. Everything else - how
well the estimate tracks the truth, how much the aircraft oscillates - is
printed as a number with no threshold against it, because a threshold would be
this file inventing a tuning target that no bench has set.

**The two conventions this file owns.** They are the airframe's, which is
exactly why they are here and not in the ABI: a motor fraction of 1.0 is
`MAX_THRUST_N` newtons and a servo of 1.0 is `MAX_ELEVON_RAD` radians. The
servo's *sign* follows from the firmware's own docs rather than from a guess -
`ak_mixer_elevon_wing` gives both elevons a positive pitch coefficient, and a
positive pitch command is nose-up, so a positive servo is trailing-edge-up,
which is what the plant's `cm_delta_e` is defined against. The magnitudes are
chosen round numbers, not fitted ones, and tuning them is the bench's job.
"""

from __future__ import annotations

import argparse
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, HERE)

import akcontrol  # noqa: E402

DEFAULT_REPO = os.path.expanduser("~/Projects/aerial-kit")

# The aircraft, and the loop it is flown at. 2 ms is 500 Hz, the rate the F405
# runs its control loop at, and the rate the core's own timing counters are
# measured against.
DT_MS = 2
DT_S = DT_MS / 1000.0
AIRSPEED_MPS = 12.0        # the cruise the trim, and so the run, starts from
MAX_THRUST_N = 8.0         # per motor, at a motor fraction of 1.0
MAX_ELEVON_RAD = math.radians(25.0)  # per elevon, at a servo of 1.0
GRAVITY = 9.81

# The sticks. Throttle is not here: it comes from the plant's own closed-form
# trim, which is the aircraft's answer to "what holds this speed" rather than a
# number this file picked. The two sticks are round numbers well inside
# `max_rate_dps`, chosen so a full-scale servo command is not needed to see the
# sign.
ROLL_STICK = 0.15
PITCH_STICK = 0.15

# The roll step. `HOLD_S` is long because the shipped roll-rate loop needs
# seconds, not milliseconds, to reach a commanded rate on this airframe: a
# half-second doublet - the first shape this file used - measured 61 deg/s of
# error that was the integrator still winding, not a steady-state offset, and
# a transient reported as an error is worse than no number at all. `STEP_AT`
# is where the rate-mode half of the run begins.
STEP_AT = 0.4
HOLD_S = 4.5

# The full-stick rate the core maps a command to, from its own `max_rate_dps`.
MAX_RATE_DPS = 700.0

# The core's own ceiling on the normalised torque it asks the mixer for, as
# shipped. Read back from the core below and checked against this, so a default
# that moves is noticed here rather than in the numbers it explains.
TORQUE_LIMIT = 0.60

# `airframe=1` is the elevon wing: two motors, two elevons, and the mixer table
# whose signs the mapping below follows.
CONFIG = "airframe=1\n"


# ---------------------------------------------------------------------------
# Frames. The estimator's quaternion is body-FRD -> NED; the plant's is
# body-FRD -> world-ENU. Both are body-FRD, which is the convention this
# firmware uses everywhere, so the conversion is a change of world basis and
# nothing else - no re-derivation of what a positive roll means.
# ---------------------------------------------------------------------------

def enu_to_ned_matrix():
    """ENU coordinates from NED ones: N is E's axis, E is N's, and D is -U."""
    return [[0.0, 1.0, 0.0],
            [1.0, 0.0, 0.0],
            [0.0, 0.0, -1.0]]


def rotmat_from_quat(q):
    w, x, y, z = q
    return [[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]]


def quat_from_rotmat(m):
    """Shepperd's method, taking the branch whose diagonal entry is largest.

    The naive `w = sqrt(1 + trace)/2` loses all its precision near a 180-degree
    rotation, which is exactly where this plant starts: a level wing's
    body-FRD -> ENU rotation *is* a 180-degree rotation about x. The branch
    that divides by the largest component keeps the significant figures.
    """
    trace = m[0][0] + m[1][1] + m[2][2]
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        q = [0.25 * s,
             (m[2][1] - m[1][2]) / s,
             (m[0][2] - m[2][0]) / s,
             (m[1][0] - m[0][1]) / s]
    elif m[0][0] > m[1][1] and m[0][0] > m[2][2]:
        s = math.sqrt(1.0 + m[0][0] - m[1][1] - m[2][2]) * 2.0
        q = [(m[2][1] - m[1][2]) / s, 0.25 * s,
             (m[0][1] + m[1][0]) / s, (m[0][2] + m[2][0]) / s]
    elif m[1][1] > m[2][2]:
        s = math.sqrt(1.0 + m[1][1] - m[0][0] - m[2][2]) * 2.0
        q = [(m[0][2] - m[2][0]) / s, (m[0][1] + m[1][0]) / s,
             0.25 * s, (m[1][2] + m[2][1]) / s]
    else:
        s = math.sqrt(1.0 + m[2][2] - m[0][0] - m[1][1]) * 2.0
        q = [(m[1][0] - m[0][1]) / s, (m[0][2] + m[2][0]) / s,
             (m[1][2] + m[2][1]) / s, 0.25 * s]
    norm = math.sqrt(sum(c * c for c in q))
    return [c / norm for c in q]


def matmul(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)]
            for i in range(3)]


def body_ned_quat(body_enu_quat):
    """The plant's attitude in the estimator's frame."""
    return quat_from_rotmat(matmul(enu_to_ned_matrix(),
                                   rotmat_from_quat(body_enu_quat)))


def quat_angle_between(a, b):
    """How far apart two rotations are, in degrees. Convention-free: q and -q
    are the same rotation, so the dot product is taken unsigned."""
    dot = abs(sum(x * y for x, y in zip(a, b)))
    dot = max(-1.0, min(1.0, dot))
    return math.degrees(2.0 * math.acos(dot))


def gravity_row(q):
    """The third row of the body-to-NED rotation, from a quaternion.

    This is the quantity `est_body_up()` in `ak_estimator.c` computes, to the
    letter, and it is the whole of what the accelerometer observes about
    attitude: the direction gravity points in the body frame. Comparing it
    across the seam compares exactly the attitude that is knowable.

    Yaw is not in it. Nothing in this core measures heading - there is no
    magnetometer, and GPS heading is only available while moving fast enough -
    so the estimator's yaw is unreferenced and starts wherever the quaternion
    was initialised, which is level pointing north. An earlier version of this
    file compared the full rotation and reported a steady 90 degrees of error
    for an aircraft flying east that was, tilt for tilt, exactly right: the
    error was this file measuring a heading the firmware never claimed.
    """
    return [2 * (q[1] * q[3] - q[0] * q[2]),
            2 * (q[2] * q[3] + q[0] * q[1]),
            1 - 2 * (q[1] * q[1] + q[2] * q[2])]


def tilt_error_deg(estimate_q, truth_q):
    """How far apart the two gravity directions are, in degrees. The vector is
    unit by construction, so this is a plain acos of the dot product."""
    a = gravity_row(estimate_q)
    b = gravity_row(truth_q)
    dot = max(-1.0, min(1.0, sum(x * y for x, y in zip(a, b))))
    return math.degrees(math.acos(dot))


# ---------------------------------------------------------------------------


class Plant:
    """The other repository's wing, and the two conversions across C7.

    `sense()` reads what an inertial unit bolted to this body would report.
    The accelerometer measures *specific force* - the non-gravitational
    acceleration - so it is the velocity change the plant just integrated,
    minus gravity, rotated into the body. Using the plant's own integrated
    velocity rather than its force model keeps this honest: an accelerometer
    reads what happened, and if the plant's integration is wrong the sensor is
    wrong in the same way a real one would be.
    """

    def __init__(self, dynamics):
        self.d = dynamics
        self.d.velocity = _np().array([AIRSPEED_MPS, 0.0, 0.0])
        self.last_accel_world = _np().zeros(3)
        self.sequence = 0

    def sense(self):
        """What an inertial unit bolted to this body would report.

        The gyro is the plant's own body rate - no conversion, because both
        sides already mean body FRD rad/s.

        The accelerometer is where this file has to know something that is not
        physics. `ak_types.h` calls the field "g" and nothing more, but
        `docs/31-contract.md` C1 is normative about it - "z reads +1 with the
        board level and the right way up, because z points down" - and
        `ak_estimator.c` agrees: `est_body_up` returns "the direction up points
        in the body frame, which is what a level accelerometer measures", and
        the correction is driven by that against the sample. So a level
        aircraft is `(0, 0, +1)`, and the field is a specific force with the
        *body z axis reversed* - not the body-FRD quantity a real inertial unit
        reports, and not what the first version of this file produced. Feeding
        it the body-FRD sign gave the estimator an aircraft that was upside
        down, and the run showed it: a steady 120 degrees of attitude error with
        the roll off by 180 and the pitch agreeing, which is the flip and
        nothing else.

        The transpose is written out rather than taken from the plant so this
        file's sensor path does not inherit a convention it is meant to be
        checking.
        """
        gyro = [float(v) for v in self.d.body_rates]
        rot = rotmat_from_quat([float(v) for v in self.d.attitude_quat])
        accel_g = [-sum(rot[j][i] * self.last_accel_world[j] for j in range(3))
                   / GRAVITY for i in range(3)]
        self.sequence += 1
        return gyro, accel_g

    def advance(self, motors, servos, dt):
        np = _np()
        thrust_l = motors[0] * MAX_THRUST_N
        thrust_r = motors[1] * MAX_THRUST_N
        elevon_l = servos[0] * MAX_ELEVON_RAD
        elevon_r = servos[1] * MAX_ELEVON_RAD
        before = self.d.velocity.copy()
        self.d.step(np.array([thrust_l, thrust_r, elevon_l, elevon_r]), dt)
        # Specific force in world axes: what the accelerometer's proof mass
        # would have felt, gravity removed.
        self.last_accel_world = (self.d.velocity - before) / dt - \
            np.array([0.0, 0.0, -GRAVITY])

    def trim_sticks(self):
        """The stick positions that hold this aircraft at its cruise, computed
        by the plant's own closed-form trim rather than by flying to it."""
        t_l, t_r, e_l, e_r = self.d.compute_trim(AIRSPEED_MPS)
        return (t_l / MAX_THRUST_N, t_r / MAX_THRUST_N,
                e_l / MAX_ELEVON_RAD, e_r / MAX_ELEVON_RAD)


_NUMPY = None


def _np():
    global _NUMPY
    if _NUMPY is None:
        import numpy
        _NUMPY = numpy
    return _NUMPY


def load_repo(path):
    """Import the sibling repository's plant, or say why not."""
    if not os.path.isdir(path):
        return None, "%s is not a directory" % path
    sys.path.insert(0, path)
    try:
        from aerial_kit.dynamics.fixed_wing import (  # noqa: E402
            FixedWingDynamics, FixedWingParams)
    except Exception as exc:  # noqa: BLE001 - the reason is the point
        return None, "%s: %s" % (path, exc)
    # The quadrotor's plant is the newer of the two and is not in every checkout
    # of the sibling repository; a missing one is reported rather than being
    # allowed to look like a failure of the aircraft that is present.
    try:
        from aerial_kit.dynamics.multirotor_actuator import (  # noqa: E402
            ActuatorPlant, ActuatorPlantParams)
    except Exception:  # noqa: BLE001
        ActuatorPlant = ActuatorPlantParams = None
    try:
        import numpy  # noqa: F401
    except ImportError:
        return None, "numpy is not installed, and the plant needs it"
    return (FixedWingDynamics, FixedWingParams, ActuatorPlant,
            ActuatorPlantParams), None


def fly(core, dynamics, command_of, seconds, config=CONFIG):
    """Close the loop for `seconds` and return the trace.

    The run opens with an arming phase: `arm_hold_ms` of the arm switch held
    with the throttle below `throttle_low`, which is the one thing this
    firmware will not skip - `AK_ARM_THROTTLE` refuses an arm request that
    arrives with the throttle open, and a first version of this file that held
    throttle at trim throughout never armed and so never flew. During those
    milliseconds the wing is gliding on its trim speed, which is what a launch
    looks like.

    `command_of(t, trace, trim)` is then called with `t` in seconds *from the
    moment the aircraft armed*, so a command's timing does not depend on how
    long the arming took, and `trim` in stick units - the plant's own
    closed-form trim, so the throttle column is the aircraft's answer to "what
    holds this speed" and not a number this file guessed.
    """
    core.config(config)
    core.set_board_outputs(2, 2)

    plant = Plant(dynamics)
    trim = plant.trim_sticks()
    trace = []
    now = 0
    steps = int(seconds * 1000) // DT_MS
    armed_at = None

    for _ in range(steps):
        gyro, accel = plant.sense()
        if armed_at is None:
            roll = pitch = yaw = 0.0
            throttle = 0.0
            angle_mode = False
        else:
            roll, pitch, yaw, throttle, angle_mode = command_of(
                (now - armed_at) / 1000.0, trace, trim)
        samples = core.samples(
            now, core.imu(t_ms=now, sequence=plant.sequence, gyro=gyro,
                          accel=accel), sequence=plant.sequence)
        command = core.command(t_ms=now, roll=roll, pitch=pitch, yaw=yaw,
                               throttle=throttle, angle_mode=angle_mode,
                               arm_request=True)
        result, out = core.step(samples, command, now)
        state = core.state()
        if armed_at is None and state.state == 1:
            armed_at = now

        truth = body_ned_quat([float(v) for v in plant.d.attitude_quat])
        estimate = [state.q_wxyz[i] for i in range(4)]

        trace.append({
            "t_ms": now,
            "armed_at": armed_at,
            "result": result,
            "state": state.state,
            "motor": [out.motor[0], out.motor[1]],
            "servo": [out.servo[0], out.servo[1]],
            "roll": state.roll, "pitch": state.pitch, "yaw": state.yaw,
            "gyro": [state.gyro[0], state.gyro[1], state.gyro[2]],
            "rate_setpoint": [state.rate_setpoint[i] for i in range(3)],
            "torque": [state.torque[i] for i in range(3)],
            "truth_roll": _roll_of(truth), "truth_pitch": _pitch_of(truth),
            "truth_quat": truth, "estimate_quat": estimate,
            "tilt_error_deg": tilt_error_deg(estimate, truth),
            "yaw_offset_deg": quat_angle_between(estimate, truth),
            "converged": state.converged,
            "airspeed": plant.d.airspeed_alpha_beta()[0],
        })

        plant.advance([out.motor[0], out.motor[1]],
                      [out.servo[0], out.servo[1]], DT_S)
        now += DT_MS

    # The trim rides along on every entry rather than in a side channel: it is
    # part of what this run was, and a trace read back later should not need
    # the plant to say what the throttle meant.
    for entry in trace:
        entry["trim"] = trim
    return trace


def _roll_of(q):
    """Roll from a body-FRD -> NED quaternion, the aerospace convention."""
    w, x, y, z = q
    return math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))


def _pitch_of(q):
    w, x, y, z = q
    return math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))


def summarise(trace, label):
    moves = [s for s in trace if s["result"] == 0]
    armed = [s for s in trace if s["state"] == 1]
    errors = [s["tilt_error_deg"] for s in trace if s["converged"]]
    yaws = [s["yaw_offset_deg"] for s in trace if s["converged"]]
    print("  %s" % label)
    print("    steps                %d, of which flown %d" % (len(trace), len(moves)))
    print("    armed from           %s"
          % ("%d ms" % armed[0]["t_ms"] if armed else "never"))
    if errors:
        print("    tilt error           mean %.2f deg, max %.2f deg "
              "(estimate against truth, after convergence)"
              % (sum(errors) / len(errors), max(errors)))
    else:
        print("    tilt error           the estimator never converged")
    if yaws:
        # Not an error and not reported as one: the heading offset is whatever
        # it is, because nothing here observes heading.
        print("    heading offset       %.1f deg, unobservable: no magnetometer"
              % (sum(yaws) / len(yaws)))
    if moves:
        final = moves[-1]
        first = moves[0]
        print("    plant trim           throttle %.3f, elevons %+.3f %+.3f"
              % (final["trim"][0], final["trim"][2], final["trim"][3]))
        # The departure, as a number rather than something to be inferred from
        # two endpoints. Nothing in this core holds altitude or airspeed: the
        # rate loop holds a *rate*, so a wing whose trim elevator is not zero
        # is free to pitch, and pitching trades height for speed. A quadrotor
        # does not care; a wing does. This is the line that says how much.
        print("    departure            airspeed %.2f -> %.2f m/s, pitch truth "
              "%+.1f -> %+.1f deg"
              % (first["airspeed"], final["airspeed"],
                 math.degrees(first["truth_pitch"]),
                 math.degrees(final["truth_pitch"])))
        print("    final roll           estimate %+.1f deg, truth %+.1f deg"
              % (math.degrees(final["roll"]), math.degrees(final["truth_roll"])))
        print("    final pitch          estimate %+.1f deg, truth %+.1f deg"
              % (math.degrees(final["pitch"]), math.degrees(final["truth_pitch"])))
        print("    servo range used     %+.3f .. %+.3f"
              % (min(min(s["servo"]) for s in moves),
                 max(max(s["servo"]) for s in moves)))
    return moves


# ===========================================================================
# The quadrotor, on the actuator-level plant
# ===========================================================================
#
# The wing above needs three invented constants to cross the seam: a motor
# fraction means `MAX_THRUST_N` newtons and a servo means `MAX_ELEVON_RAD`
# radians, and neither number is in either repository. The quadrotor needs
# **none**. `ak_mixer_apply_limited` writes a motor fraction, and
# `ActuatorPlant.set_motors()` takes a motor fraction, and they are the same
# quantity in the same units with no scale factor between them - which is what
# an actuator-level plant buys, and the reason the assessment asked for one.
#
# It is also the loop B7 could not close. That run used the wing because the
# quadrotor's plant in the sibling repository takes `set_command(body_rates,
# thrust)`, an input already shaped like the flight controller's own output;
# closing a loop around it would have tested the controller against itself
# (see F2). `ActuatorPlant` takes four motor commands and integrates a rigid
# body, a rotor geometry and a motor lag, so a sign error in `ak_mixer.c` now
# shows up as an aircraft rolling the wrong way.

# The airframe number the `airframe` parameter uses: 0 is the quad-X, and the
# plant's default motor order is that table's row order.
CONFIG_QUAD = "airframe=0\n"

# The plant's aircraft. A 1 kg quadrotor on 0.2 m arms: the mass and the inertia
# are the plant's, and the firmware is not told them, so a mistuning here is the
# firmware's problem to have rather than this file's to hide.
QUAD_MASS_KG = 1.0
QUAD_ARM_M = 0.2
QUAD_MAX_THRUST_N = 6.0  # per motor at a motor fraction of 1.0

# The sticks, as fractions of full scale, matching the wing's.
QUAD_ROLL_STICK = 0.15
QUAD_PITCH_STICK = 0.15


class QuadPlant:
    """The sibling repository's actuator-level quadrotor, across the seam.

    Two things are deliberately *not* done here.

    There is no frame conversion. That is the plant's whole design decision:
    it is NED and FRD because its consumer is this firmware, so
    `attitude_quat` is already body-FRD -> NED and goes to the estimator as it
    stands. The wing plant is ENU, which is why `body_ned_quat()` exists above;
    for this one the honest conversion is none at all.

    And there is no accelerometer sign flip. The plant's `sense()` already
    reports the field the way `docs/31-contract.md` C1 defines it and
    `est_body_up()` reads it - `(0, 0, +1)` g level - rather than the way a real
    inertial unit reports it. That is a convention this file would otherwise
    have to apply a second time, and applying it twice is exactly how B7's first
    version flew an aircraft upside down (trap 67).
    """

    def __init__(self, plant):
        self.p = plant
        self.sequence = 0
        # Start at equilibrium rather than at rest: motors at hover, so the
        # aircraft is holding altitude on the first step. The analogue of the
        # wing's closed-form trim, using the plant's own arithmetic.
        self.p.set_hover(1.0)
        self.p.motor_actual[...] = self.p.motor_command.copy()

    def hover_throttle(self):
        """The throttle stick that hovers, from the plant's own numbers.

        `ak_mixer_quad_x` gives every motor row a throttle coefficient of 1.0,
        so the throttle stick *is* the motor fraction, and the fraction that
        holds this aircraft up is `m*g / (4 * max_thrust)`. Nothing fitted, and
        nothing the firmware had to be told.
        """
        p = self.p.params
        return p.mass_kg * GRAVITY / (4.0 * p.max_thrust_per_motor_n)

    def sense(self):
        s = self.p.sense()
        self.sequence = s.sequence
        return list(s.gyro_rps), list(s.accel_g)

    def advance(self, motors, dt):
        self.p.set_motors(_np().array(motors, dtype=float))
        self.p.step(dt)

    @property
    def attitude_quat(self):
        return [float(v) for v in self.p.attitude_quat]

    @property
    def body_rates(self):
        return [float(v) for v in self.p.body_rates]


def fly_quad(core, plant, command_of, seconds, config=CONFIG_QUAD):
    """Close the loop on the quadrotor and return the trace.

    `command_of(t, trace, hover)` is called with `t` in seconds from the arm and
    `hover` the plant's own hover throttle, so the throttle column is the
    aircraft's answer to "what holds it up" rather than a number this file
    picked.
    """
    core.config(config)
    core.set_board_outputs(4, 0)

    trace = []
    now = 0
    steps = int(seconds * 1000) // DT_MS
    armed_at = None

    for _ in range(steps):
        gyro, accel = plant.sense()
        if armed_at is None:
            roll = pitch = yaw = 0.0
            throttle = 0.0
            angle_mode = False
        else:
            roll, pitch, yaw, throttle, angle_mode = command_of(
                (now - armed_at) / 1000.0, trace, plant.hover_throttle())
        samples = core.samples(
            now, core.imu(t_ms=now, sequence=plant.sequence, gyro=gyro,
                          accel=accel), sequence=plant.sequence)
        command = core.command(t_ms=now, roll=roll, pitch=pitch, yaw=yaw,
                               throttle=throttle, angle_mode=angle_mode,
                               arm_request=True)
        result, out = core.step(samples, command, now)
        state = core.state()
        if armed_at is None and state.state == 1:
            armed_at = now
            # The arming phase holds the throttle shut, so a quadrotor spends
            # those milliseconds in free fall - the wing spends them gliding.
            # Zeroing the motion at the arm starts the flight from rest, so a
            # sinking start is not read later as a control problem.
            plant.p.position[...] = 0.0
            plant.p.velocity[...] = 0.0

        motors = [out.motor[i] for i in range(4)]
        truth = plant.attitude_quat
        estimate = [state.q_wxyz[i] for i in range(4)]

        trace.append({
            "t_ms": now,
            "armed_at": armed_at,
            "result": result,
            "state": state.state,
            "motor": motors,
            "saturated": sum(1 for m in motors if m <= 1e-6 or m >= 1.0 - 1e-6),
            "roll": state.roll, "pitch": state.pitch, "yaw": state.yaw,
            "gyro": [state.gyro[0], state.gyro[1], state.gyro[2]],
            "rate_setpoint": [state.rate_setpoint[i] for i in range(3)],
            "torque": [state.torque[i] for i in range(3)],
            "truth_roll": _roll_of(truth), "truth_pitch": _pitch_of(truth),
            "truth_quat": truth, "estimate_quat": estimate,
            "tilt_error_deg": tilt_error_deg(estimate, truth),
            "converged": state.converged,
            # The plant's own view of what the emitted motors do. This is the
            # independent half: the firmware says what torque it asked for and
            # the plant says what torque those motors make, and the two were
            # computed by different code in different languages.
            "body_rates": plant.body_rates,
            "achieved_moment": [float(v) for v in
                                plant.p.achieved_wrench().moment_body],
            "altitude": -float(plant.p.position[2]),
        })

        plant.advance(motors, DT_S)
        now += DT_MS

    for entry in trace:
        entry["hover"] = plant.hover_throttle()
    return trace


def quad_experiment(core, ActuatorPlant, ActuatorPlantParams, seconds):
    """The quadrotor's half: does the firmware's mixer fly an aircraft?"""
    np = _np()
    ok = True

    def fresh():
        return QuadPlant(ActuatorPlant(params=ActuatorPlantParams(
            mass_kg=QUAD_MASS_KG,
            max_thrust_per_motor_n=QUAD_MAX_THRUST_N,
        ), motor_positions=quad_positions()))

    # --- the convention, before anything flies -------------------------------
    #
    # A level quadrotor's accelerometer has to read (0, 0, +1) g. If it does
    # not, every number after this one is measuring the wrong aircraft, so it is
    # checked first and on its own.
    probe = fresh()
    for _ in range(5):
        probe.advance([probe.hover_throttle()] * 4, DT_S)
    accel = probe.sense()[1]
    close_enough = (abs(accel[0]) < 1e-6 and abs(accel[1]) < 1e-6
                    and abs(accel[2] - 1.0) < 1e-3)
    print("  level accelerometer  (%+.4f, %+.4f, %+.4f) g  %s"
          % (accel[0], accel[1], accel[2], "ok  " if close_enough else "FAIL"))
    ok = ok and close_enough

    # --- the rate experiment -------------------------------------------------
    def roll_step(elapsed, trace, hover):
        if elapsed < STEP_AT:
            return (0.0, 0.0, 0.0, hover, False)
        if elapsed < STEP_AT + HOLD_S:
            return (QUAD_ROLL_STICK, 0.0, 0.0, hover, False)
        if elapsed < STEP_AT + 2 * HOLD_S:
            return (-QUAD_ROLL_STICK, 0.0, 0.0, hover, False)
        return (0.0, 0.0, 0.0, hover, False)

    print("\nthe quadrotor, rate mode: a roll step each way")
    first = fly_quad(core, fresh(), roll_step, seconds)

    def since_arm(s):
        return (s["t_ms"] - s["armed_at"]) / 1000.0 if s["armed_at"] else -1.0

    def mean_of(window, key, index):
        if not window:
            return 0.0
        return sum(s[key][index] for s in window) / len(window)

    flown = [s for s in first if s["result"] == 0]
    armed = [s for s in first if s["state"] == 1]
    print("    steps                %d, of which flown %d"
          % (len(first), len(flown)))
    print("    armed from           %s"
          % ("%d ms" % armed[0]["t_ms"] if armed else "never"))
    if not armed:
        print("    FAIL  the core never armed, so nothing below is a flight")
        return False

    # The roll rate, taken from the *plant* rather than from the estimator's
    # gyro echo. The truth is the aircraft; the estimate is what the loop
    # believes, and asking it whether it succeeded is asking the suspect.
    def plant_roll_between(a, b):
        window = [s for s in first if a <= since_arm(s) < b]
        return mean_of(window, "body_rates", 0)

    before = plant_roll_between(0.1, STEP_AT)
    right = plant_roll_between(STEP_AT + HOLD_S - 0.5, STEP_AT + HOLD_S)
    left = plant_roll_between(STEP_AT + 2 * HOLD_S - 0.5, STEP_AT + 2 * HOLD_S)
    print("    roll rate at hover     %+.3f rad/s" % before)
    print("    roll rate, +%.2f stick %+.3f rad/s, settled" % (QUAD_ROLL_STICK, right))
    print("    roll rate, -%.2f stick %+.3f rad/s, settled" % (QUAD_ROLL_STICK, left))

    # The sign, asserted: the stick's direction has to reach the airframe. This
    # is the whole chain at once - stick, decoder, rate loop, `ak_mixer.c`'s
    # roll column, four motor fractions, and the plant's rotor geometry - and it
    # is the check that a table and a comment agreeing with each other cannot
    # pass (trap 73).
    # Against the number the core maps the stick to, not against zero: 0.15 of
    # full stick is `0.15 * max_rate_dps`, and a loop that reaches it has done
    # the whole job at once - decoder, rate law, `ak_mixer.c`'s roll column, four
    # motor fractions, and the plant's rotor geometry.
    wanted = math.radians(QUAD_ROLL_STICK * MAX_RATE_DPS)
    ok_roll = (abs(right - wanted) < 0.15 * wanted
               and abs(left + wanted) < 0.15 * wanted)
    print("    %s  a right roll stick rolls the aircraft right, at the rate "
          "the core asked for (%+.3f rad/s)" % ("ok  " if ok_roll else "FAIL",
                                                wanted))
    ok = ok and ok_roll

    # --- and the same for pitch, which is where the sign was wrong ----------
    #
    # `ak_mixer.h` said "nose-up raises the rear pair" until 2026-09-19. The
    # table agreed with the sentence and both were wrong about the aircraft;
    # the simulator found it as a 1,861 degree runaway the first time its quad
    # plant modelled thrust honestly. So the check is two-part: the motors the
    # firmware *emits* must raise the front pair, and the airframe must
    # consequently pitch nose-up.
    def pitch_step(elapsed, trace, hover):
        if elapsed < STEP_AT:
            return (0.0, 0.0, 0.0, hover, False)
        return (0.0, QUAD_PITCH_STICK, 0.0, hover, False)

    print("\nthe quadrotor, the nose-up direction")
    second = fly_quad(core, fresh(), pitch_step, seconds)
    # The differential has to be measured in the moment the command arrives, and
    # that is a fact about the airframe rather than about this file's patience.
    # `ak_mixer.c`'s differential is what *starts* the rotation; once the rate
    # loop has the rate it asked for it commands almost nothing, because a rigid
    # body with no rotor damping and no aerodynamics conserves the rate it was
    # given. Averaged over the whole held window - the first version of this
    # check - the differential reads as zero and the sign, which is the only
    # thing being tested, is averaged away with it.
    onset = [s for s in second if STEP_AT <= since_arm(s) < STEP_AT + 0.05
             and s["result"] == 0]
    if onset:
        diffs = [(s["motor"][1] + s["motor"][3]) / 2.0
                 - (s["motor"][0] + s["motor"][2]) / 2.0 for s in onset]
        peak = max(diffs)
        print("    motor fractions      first %.0f ms: front pair %+.4f above "
              "the rear pair at most" % (1000 * 0.05, peak))
        # And the direction, on the first step the command is in: this is the
        # `ak_mixer.h` table against the aircraft, which is the disagreement
        # that stood until 2026-09-19.
        print("    %s  a nose-up command raises the *front* pair"
              % ("ok  " if peak > 0.0 and diffs[0] > 0.0 else "FAIL"))
        ok_front = peak > 0.0 and diffs[0] > 0.0

        # The rate itself, against the number the core maps the stick to rather
        # than against zero. `max_rate_dps` was read back from the core above and
        # checked, so `stick * max_rate_dps` is the setpoint the loop was given,
        # and a loop that reaches it has done the whole job: decoder, rate law,
        # mixer, motors, and the plant's rotor geometry.
        # The rate is measured in the settled window, not the onset one: the
        # same 50 ms that carries the differential carries the transient, and
        # the loop is still climbing through it (0.63 rad/s at the first step,
        # 1.86 by the end). What the loop *reaches* is the question here, and
        # onset above is what answers the mixer's sign.
        wanted = math.radians(QUAD_PITCH_STICK * MAX_RATE_DPS)
        settled = [s for s in second
                   if STEP_AT + HOLD_S - 0.5 <= since_arm(s) < STEP_AT + HOLD_S
                   and s["result"] == 0]
        reached = mean_of(settled, "body_rates", 1)
        print("    pitch rate           %+.3f rad/s reached, %+.3f rad/s asked "
              "for (%.0f%%), nose up positive"
              % (reached, wanted, 100.0 * reached / wanted if wanted else 0.0))
        ok_pitch = abs(reached - wanted) < 0.15 * wanted
        print("    %s  and it reaches the rate the stick asked for"
              % ("ok  " if ok_pitch else "FAIL"))
        ok = ok and ok_front and ok_pitch
    else:
        print("    FAIL  no flown steps to measure")
        ok = False

    # --- what happened when the aircraft ran out of authority ---------------
    #
    # Not a pass/fail: a number to report. `ak_mixer_apply_limited` reduces the
    # *differential* rather than clamping each motor, so a demand past the
    # motors' authority should cost torque and keep thrust. The wing's
    # experiment has nothing to measure here because nothing saturates; a quad
    # at a step input does.
    # Armed steps only. Before arming every motor is commanded to zero and a
    # zero is not a saturated command - counting those read as 301 steps
    # "against a stop" with a mean throttle of 0.000, which is the arming phase
    # and not a statement about authority at all.
    # After the step begins, and armed. The first armed step of all legitimately
    # commands every motor to zero, which `saturated` counts as a motor against a
    # stop and which is a boundary of arming rather than a statement about
    # authority. Guarding both is cheaper than explaining the number later.
    sat = [s for s in first if s["saturated"] and s["state"] == 1
           and since_arm(s) >= STEP_AT]
    print("\nwhen the motors ran out of authority")
    print("    steps with a motor against a stop   %d of %d flown"
          % (len(sat), len(flown)))
    if sat:
        # Thrust is the sum; if the mixer is doing its job this stays near
        # hover while the torque is what gives way.
        total = sum(sum(s["motor"]) for s in sat) / len(sat)
        hover_total = 4.0 * sat[0]["hover"]
        print("    mean total motor fraction there     %.4f (hover is %.4f)"
              % (total, hover_total))
        print("    mean altitude at those steps        %.3f m"
              % (sum(s["altitude"] for s in sat) / len(sat)))
        near = abs(total - hover_total) < 0.15 * hover_total
        print("    %s  thrust held while authority gave way"
              % ("ok  " if near else "note"))
        ok = ok and near

    # --- reproducibility -----------------------------------------------------
    print("\nthe same quadrotor experiment, a second time")
    again = fly_quad(core, fresh(), roll_step, seconds)

    def digest(trace):
        return [(s["t_ms"], s["result"], tuple(s["motor"]),
                 tuple(s["truth_quat"]), tuple(s["estimate_quat"]),
                 tuple(s["body_rates"])) for s in trace]

    same = digest(first) == digest(again)
    print("    %s  %d steps compared, bit for bit"
          % ("ok  " if same else "FAIL", len(first)))
    if not same:
        for i, (a, b) in enumerate(zip(digest(first), digest(again))):
            if a != b:
                print("      first difference at step %d" % i)
                break
    ok = ok and same

    # --- what this run is filed under ---------------------------------------
    print("\nwhat the quadrotor result is filed under")
    print("    config hash   0x%08x" % core.config_hash)
    print("    plant         aerial_kit.dynamics.multirotor_actuator.ActuatorPlant")
    print("    seam          motor fraction in, motor fraction out: no scale "
          "factor, no frame conversion")
    print("    hover throttle %.4f, from the plant's own m*g/(4*max_thrust)"
          % first[0]["hover"])

    # The one constant this half reasons with, read back from the core rather
    # than trusted, for the same reason the wing half reads its two back: the
    # 1.833 rad/s every rate above is judged against is
    # `QUAD_ROLL_STICK * max_rate_dps`, so a `max_rate_dps` that moved would
    # change what those rates *mean* without changing what they read. This is
    # the quad's counterpart of the wing's defaults check, and it is one
    # constant rather than two because the quad's authority report is a
    # measurement (`0 of 5500`) rather than a limit compared against a number.
    quad_shown = {}
    for line in core.config_text().splitlines():
        name, _, value = line.partition("=")
        if name in ("max_rate_dps", "torque_limit"):
            quad_shown[name] = value
    wanted_dps = quad_shown.get("max_rate_dps", "nan")
    # Written as `not (<=)` rather than `>` on purpose: a missing or unparsable
    # value becomes nan, and `nan > 1e-3` is *false*, so the obvious spelling
    # would pass a key the core never printed. Trap 74 is the same shape.
    if not (abs(float(wanted_dps) - MAX_RATE_DPS) <= 1e-3):
        print("    FAIL  max_rate_dps is %s in this build, and this file "
              "explains its numbers with %.2f" % (wanted_dps, MAX_RATE_DPS))
        ok = False
    else:
        print("    ok    the rate this file explains itself with is the one "
              "the core is using (max_rate_dps=%s)" % wanted_dps)
    return ok


def quad_positions():
    from aerial_kit.dynamics.multirotor_actuator import quad_x_positions
    return quad_x_positions(QUAD_ARM_M)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--repo", default=os.environ.get("AK_REPO", DEFAULT_REPO),
                        help="the aerial-kit checkout with the Python plant")
    parser.add_argument("--lib", default=os.environ.get("AK_CONTROL_LIB"),
                        help="the shared library; defaults to build-host/")
    # Long enough that `arm_hold_ms` plus the doublet plus a settle all fit.
    # At 3 s the arming took 712 ms and the windows ran off the end of the
    # trace, which read as a failed experiment rather than as a short one.
    parser.add_argument("--seconds", type=float, default=11.0)
    args = parser.parse_args(argv)

    repo, why = load_repo(os.path.expanduser(args.repo))
    if repo is None:
        print("akcontrol_experiment: not in this checkout")
        print("  %s" % why)
        print("  point --repo or AK_REPO at the aerial-kit checkout that has "
              "aerial_kit/dynamics/fixed_wing.py")
        return 0

    try:
        core = akcontrol.Core(args.lib)
    except (akcontrol.LoadError, akcontrol.AbiError, akcontrol.LayoutError) as exc:
        print("akcontrol_experiment: %s" % exc, file=sys.stderr)
        return 1

    (FixedWingDynamics, FixedWingParams,
     ActuatorPlant, ActuatorPlantParams) = repo

    ident = core.identity()
    print("akcontrol_experiment: the C control core flying a Python plant")
    print("  library    %s" % core.path)
    print("  product    %s, revision %s" % (ident["product"], ident["revision"]))
    print("  built      %s" % ident["built"])
    print("  abi        %d" % ident["abi"])
    print("  plant      %s" % os.path.expanduser(args.repo))
    print("  airframe   elevon wing, %d Hz, from %.1f m/s"
          % (1000 // DT_MS, AIRSPEED_MPS))
    print("  mapping    motor 1.0 = %.1f N, servo 1.0 = %.0f deg"
          % (MAX_THRUST_N, math.degrees(MAX_ELEVON_RAD)))

    # --- the rate experiment -------------------------------------------------
    #
    # A roll doublet in rate mode: half a second each way about trim. The
    # interesting question is not whether it rolls - a wing with ailerons
    # rolls - but whether the sign is the one the stick asked for, which is
    # the whole chain at once: stick, decoder, rate loop, mixer, servo, and
    # this file's own unit conversion.
    def roll_step(elapsed, trace, trim):
        throttle = trim[0]
        if elapsed < STEP_AT:
            return (0.0, 0.0, 0.0, throttle, False)
        if elapsed < STEP_AT + HOLD_S:
            return (ROLL_STICK, 0.0, 0.0, throttle, False)
        if elapsed < STEP_AT + 2 * HOLD_S:
            return (-ROLL_STICK, 0.0, 0.0, throttle, False)
        return (0.0, 0.0, 0.0, throttle, False)

    print("\nthe rate experiment: a roll step each way, in rate mode")
    first = fly(core, FixedWingDynamics(FixedWingParams()), roll_step,
                args.seconds)
    summarise(first, "run 1")

    # What the doublet did: the roll rate before, during and after each half.
    # Timed from the arm, so this does not depend on how long arming took.
    def since_arm(s):
        return (s["t_ms"] - s["armed_at"]) / 1000.0 if s["armed_at"] else -1.0

    def roll_rate_between(a, b):
        window = [s for s in first if a <= since_arm(s) < b]
        if not window:
            return 0.0
        return sum(s["gyro"][0] for s in window) / len(window)

    before = roll_rate_between(0.0, STEP_AT)
    right = roll_rate_between(STEP_AT + HOLD_S - 0.5, STEP_AT + HOLD_S)
    left = roll_rate_between(STEP_AT + 2 * HOLD_S - 0.5, STEP_AT + 2 * HOLD_S)
    print("    roll rate at trim       %+.3f rad/s" % before)
    print("    roll rate, +%.2f stick  %+.3f rad/s, settled"
          % (ROLL_STICK, right))
    print("    roll rate, -%.2f stick  %+.3f rad/s, settled"
          % (ROLL_STICK, left))

    # The rate setpoint is the core's own output and the gyro is the plant's,
    # and both are body roll rate in rad/s - the one place in either experiment
    # where a commanded number and a measured one of the same kind sit side by
    # side. Each half is measured on its own: averaged together, the two
    # opposite commands cancel and the error reads as noise near zero.
    halves = []
    for label, sign, start in (("+", 1.0, STEP_AT),
                               ("-", -1.0, STEP_AT + HOLD_S)):
        half = [s for s in first if start <= since_arm(s) < start + HOLD_S
                and s["result"] == 0]
        if not half:
            continue
        commanded = math.degrees(sum(s["rate_setpoint"][0] for s in half)
                                 / len(half))
        reached = math.degrees(sum(s["gyro"][0] for s in half) / len(half))
        # Settling: the first moment the airframe is within 10% of the rate it
        # was asked for *in the direction it was asked for*. The sign matters:
        # the second half opens with the first half's rate still on the
        # airframe, which is the right magnitude and the wrong direction, and a
        # test that only looked at magnitude reported that half as settling
        # instantly. Held long enough that a half which never gets there is
        # reporting a real shortfall rather than a window that closed early.
        settle = None
        for s in half:
            if (s["gyro"][0] * sign > 0.0
                    and abs(s["gyro"][0]) >= 0.9 * abs(s["rate_setpoint"][0])):
                settle = since_arm(s) - start
                break
        print("    %s step  setpoint %+.0f deg/s, mean %+.0f deg/s (%.0f%%), %s"
              % (label, commanded, reached,
                 100.0 * reached / commanded if commanded else 0.0,
                 "settled in %.2f s" % settle if settle is not None
                 else "short of 90%% after %.1f s" % HOLD_S))
        # Whether the gap is the loop failing or the airframe being asked for
        # more than it has. `torque_limit` is the core's own ceiling on the
        # normalised torque it asks the mixer for, so a run sitting on it is
        # reporting a tuning limit rather than a control failure, and the two
        # must never be read as the same number.
        sat = sum(1 for s in half if abs(s["torque"][0]) >= TORQUE_LIMIT - 1e-3)
        print("           torque limit %.2f, reached on %d of %d steps"
              % (TORQUE_LIMIT, sat, len(half)))
        halves.append((sign, commanded, reached, settle))

    # The sign, asserted rather than eyeballed: each step must drive the
    # airframe in the direction the stick asked for. The setpoint sign is
    # checked too, because a run where the core commanded the wrong sign and
    # the airframe faithfully obeyed would otherwise pass.
    ok_sign = (len(halves) == 2
               and all(commanded * sign > 0.0
                       and reached * sign > abs(before) for
                       sign, commanded, reached, _ in halves))
    print("    %s  the stick's sign reaches the airframe"
          % ("ok  " if ok_sign else "FAIL"))

    # --- the attitude experiment --------------------------------------------
    #
    # The same loop with the angle loop in it, which is the other law in this
    # core: the stick asks for a tilt and the outer loop turns it into a rate.
    def pitch_step(elapsed, trace, trim):
        if elapsed < 0.4:
            return (0.0, 0.0, 0.0, trim[0], True)
        return (0.0, PITCH_STICK, 0.0, trim[0], True)

    print("\nthe attitude experiment: a pitch step, in angle mode")
    second = fly(core, FixedWingDynamics(FixedWingParams()), pitch_step,
                 args.seconds)
    summarise(second, "run 2")

    # The angle loop's acceptance, stated as one: before the step the aircraft
    # is trimmed level, and after it the aircraft is holding a nose-up tilt
    # that the stick asked for. Both halves come from the *truth*, not the
    # estimate - the estimate is what the loop believes, and asking it whether
    # it succeeded would be asking the suspect for an alibi.
    held = [s for s in second if since_arm(s) >= 2.5 and s["result"] == 0]
    before_step = [s for s in second if 0.1 <= since_arm(s) < 0.4
                   and s["result"] == 0]
    ok_step = False
    if held and before_step:
        was = math.degrees(sum(s["truth_pitch"] for s in before_step)
                           / len(before_step))
        now = math.degrees(sum(s["truth_pitch"] for s in held) / len(held))
        print("    truth pitch          %+.1f deg before the step, %+.1f deg "
              "held after" % (was, now))
        ok_step = now - was > math.radians(2.0) and now > 0.0
    print("    %s  the stick's sign reaches the airframe"
          % ("ok  " if ok_step else "FAIL"))
    ok_sign = ok_sign and ok_step

    # --- reproducibility -----------------------------------------------------
    #
    # The same experiment again, from a fresh plant, and compared to the last
    # bit. This is what "reproducible" has to mean if a result is going to be
    # filed under a configuration identity: not that the numbers are close,
    # that they are the same numbers.
    print("\nthe same experiment, a second time")
    again = fly(core, FixedWingDynamics(FixedWingParams()), roll_step,
                args.seconds)

    def digest(trace):
        return [(s["t_ms"], s["result"], tuple(s["motor"]), tuple(s["servo"]),
                 tuple(s["truth_quat"]), tuple(s["estimate_quat"]))
                for s in trace]

    same = digest(first) == digest(again)
    print("    %s  %d steps compared, bit for bit" % ("ok  " if same else "FAIL",
                                                       len(first)))
    if not same:
        for i, (a, b) in enumerate(zip(digest(first), digest(again))):
            if a != b:
                print("      first difference at step %d" % i)
                break

    # --- what this run is filed under ---------------------------------------
    #
    # C8's identity, the revision the library was built from, and the plant's
    # own provenance. Without these the numbers above are an anecdote.
    print("\nwhat this result is filed under")
    print("    config hash   0x%08x" % core.config_hash)
    print("    revision      %s" % ident["revision"])
    print("    built         %s" % ident["built"])
    print("    plant         aerial_kit.dynamics.fixed_wing.FixedWingDynamics")
    shown = {}
    for line in core.config_text().splitlines():
        name, _, value = line.partition("=")
        if name in ("airframe", "rate_kp_roll", "rate_kp_pitch",
                    "rate_ki_roll", "rate_ki_pitch",
                    "max_rate_dps", "torque_limit"):
            shown[name] = value
            print("    %s" % line)

    # The two constants this file reasons with are read back from the core
    # rather than trusted. `MAX_RATE_DPS` turns a stick into the setpoint the
    # settle times are measured against, and `TORQUE_LIMIT` decides whether a
    # shortfall is the loop or the airframe; a default that moved would change
    # what every number above means without changing how any of them read.
    ok_defaults = True
    for name, expected in (("max_rate_dps", MAX_RATE_DPS),
                           ("torque_limit", TORQUE_LIMIT)):
        actual = float(shown.get(name, "nan"))
        if abs(actual - expected) > 1e-3:
            print("    FAIL  %s is %s in this build, and this file explains "
                  "its numbers with %.2f" % (name, shown.get(name, "absent"),
                                             expected))
            ok_defaults = False
    if ok_defaults:
        print("    ok    the two defaults this file explains itself with are "
              "the ones the core is using")

    ok_wing = ok_sign and same and ok_defaults

    # --- the quadrotor, on the actuator-level plant --------------------------
    #
    # A new section rather than a new file: it is the same core, the same ABI
    # and the same experiment run against a different airframe, and splitting it
    # would let the two drift apart at exactly the seam they share.
    print("\n" + "=" * 75)
    print("the quadrotor: the same core, on an actuator-level plant")
    print("=" * 75)
    if ActuatorPlant is None:
        print("  the sibling checkout has no "
              "aerial_kit/dynamics/multirotor_actuator.py")
        print("  so the quadrotor half cannot run here. That file is B8's")
        print("  deliverable and this is the loop it exists to close.")
        print("  not run: quadrotor experiment")
        ok_quad = None
    else:
        ok_quad = quad_experiment(core, ActuatorPlant, ActuatorPlantParams,
                                  args.seconds)

    print("\n" + "=" * 75)
    print("result")
    print("  %s  the elevon wing, on the sibling's 6-DOF wing plant"
          % ("ok  " if ok_wing else "FAIL"))
    if ok_quad is None:
        print("  --    the quadrotor: not run (no actuator plant in the checkout)")
    else:
        print("  %s  the quadrotor, on the sibling's actuator-level plant"
              % ("ok  " if ok_quad else "FAIL"))
    return 0 if (ok_wing and ok_quad is not False) else 1


if __name__ == "__main__":
    sys.exit(main())
