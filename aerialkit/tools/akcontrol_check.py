#!/usr/bin/env python3
"""The C control core, through the ABI a Python simulation uses.

    make control-check

`build-host/libaerialkit-control.so` is the flight core - `ak_flight_step` and
everything it is linked against, built from the same sources as the MCU image -
behind `tools/ak_control.h`. `tools/akcontrol.py` is the Python side. This is
the check that the seam between them says what `docs/31-contract.md` says it
says, and it is written to run on a machine that has python3 and nothing else:
no numpy, no sibling repository, no board.

**What it is checking, and why each part is here.**

  * *The layout.* A struct crossing a language boundary fails quietly, so the
    offsets the compiler chose are compared against the ones ctypes chose. The
    mutation self-test at the end is what says the comparison is looking at
    anything: a check that cannot fail is a check that is not running.
  * *Identity (§C8).* The configuration has a name, the name is a hash of the
    effective table rather than of the text that produced it, and it is
    recomputed here from `config_text()` by an independent FNV-1a. Two ways of
    saying the same table have to give the same number, and one changed value
    has to give a different one - otherwise a result filed under an identity is
    filed under nothing.
  * *The boundary rules (C5, C6, C7).* A stale sample is refused, an expired
    command is refused and its sticks are zeroed, an absent sensor is reported
    separately from a sensor reading zero, and what crosses out is normalised.
    Each of these is a decision the firmware does *not* make for itself, which
    is why each of them needs a check here rather than a unit test there.
  * *The receiver round trip.* The command goes in as the stick shape and is
    inverted into receiver counts so that the firmware's own `ak_rc_decode`
    turns it back into sticks. The value that comes back in `state.cmd_*` is the
    firmware's answer, not this file's, and that is what makes "there is no
    second decoder" a fact rather than an intention.
  * *Flying.* It converges, it arms, the motors follow the throttle, the rate
    loop's setpoint is the configured maximum, and a changed gain changes what
    the loop asks for. That last one is the check that the configuration
    actually reaches the control law.
  * *Reproducibility.* Two runs, same inputs, same numbers, bit for bit. This is
    the half of the acceptance criterion - "a reproducible rate/attitude
    experiment" - that does not need a plant to state.

The other half does, and it is `tools/akcontrol_experiment.py`: that one closes
the loop around a rigid body, and it needs the sibling repository, so it is not
in `make test`.
"""

from __future__ import annotations

import ctypes
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, HERE)

import akcontrol  # noqa: E402  (after the path is set, on purpose)

# The lib is rebuilt by `make control-check`; this is the same default the
# binding uses, so a hand run and a `make` run look at the same file.
LIB = os.environ.get("AK_CONTROL_LIB") or os.path.join(
    ROOT, "build-host", "libaerialkit-control.so"
)

# The revision the tree is at *now*, which the Makefile passes beside the
# library's path. The library's own revision was baked in when it was compiled,
# so the two disagreeing means the library is not the one this tree builds.
# That is trap 183: a suite green with 2120 checks against a library one full
# batch behind, where the only sign was a banner nothing compared. Empty, or
# "unknown" from a tree with no git, skips the comparison rather than failing
# it, so a hand run or a shallow checkout still works.
TREE_REV = os.environ.get("AK_REV", "")


def rev_commit(rev):
    """The commit half of a `git describe --always --dirty` string.

    Only the commit is compared, never the `-dirty` suffix. Dirtiness is not
    staleness: editing any tracked file the library does not contain - a test, a
    document, a header it never includes - makes the tree `-dirty` while the
    library stays perfectly current for the commit it was built from. Failing on
    that would make this check fire on ordinary work and be turned off.
    """
    return rev.split("-dirty")[0]

checks = 0
failures = 0
quiet = False


def check(ok, label, detail=""):
    global checks, failures
    checks += 1
    if not ok:
        failures += 1
    if quiet and ok:
        return
    suffix = ("  %s" % detail) if detail else ""
    print("  %s  %s%s" % ("ok  " if ok else "FAIL", label, suffix))


def close(a, b, tol=1e-4):
    return abs(a - b) <= tol


# ---------------------------------------------------------------------------
# The aircraft, and how it is flown here.
#
# A 2 ms step is 500 Hz: the loop the F405 runs, and the rate this file's
# numbers were measured at. The stick shape is C6's - roll, pitch, yaw in
# -1..1, throttle 0..1 - and `arm_hold_ms` is 500 by default, so a run that
# wants an armed aircraft holds the switch for at least that long.
# ---------------------------------------------------------------------------

DT = 2
LEVEL = (0.0, 0.0, 1.0)  # accelerometer reading of an aircraft sitting level


class Run:
    """One flight: a clock, a sample sequence number, and the last outputs."""

    def __init__(self, core, throttle=0.0, roll=0.0, pitch=0.0, yaw=0.0,
                 angle_mode=False, arm=True, accel=LEVEL, dt=DT):
        self.core = core
        self.dt = dt
        self.throttle = throttle
        self.roll = roll
        self.pitch = pitch
        self.yaw = yaw
        self.angle_mode = angle_mode
        self.arm = arm
        self.accel = accel
        self.i = 0
        self.now = 0
        self.result = None
        self.out = None

    def step(self, now=None, command=None, samples=None):
        if now is None:
            now = self.now
        if samples is None:
            samples = self.core.samples(
                now,
                self.core.imu(t_ms=now, sequence=self.i, accel=self.accel),
            )
        if command is None:
            command = self.core.command(
                t_ms=now, roll=self.roll, pitch=self.pitch, yaw=self.yaw,
                throttle=self.throttle, angle_mode=self.angle_mode,
                arm_request=self.arm,
            )
        self.result, self.out = self.core.step(samples, command, now)
        self.i += 1
        self.now = now + self.dt
        return self.result, self.out

    def fly(self, ms):
        """Advance by simulated milliseconds at the run's rate."""
        for _ in range(int(ms) // self.dt):
            self.step()
        return self.result, self.out, self.core.state()

    def until_armed(self, limit_ms=4000):
        """Fly with the arm switch held until the core arms, and return how
        long that took - or None, which means it never did."""
        for _ in range(int(limit_ms) // self.dt):
            self.step()
            if self.core.state().state == akcontrol.FLIGHT_ARMED:
                return self.core.state().now_ms
        return None


# Added to the binding by this check rather than in it: these are the firmware's
# own enum values, and a binding has no business naming them.
akcontrol.FLIGHT_ARMED = 1
akcontrol.ARM_ARMED = None


def fresh_config(core, text):
    """A config is a complete reset - it re-initialises the core before
    applying the table - so this is how a second experiment starts."""
    core.config(text)
    core.set_board_outputs(4, 0)
    return core


# ---------------------------------------------------------------------------


def check_identity(core):
    print("\nthe library says what it is")

    check(core.abi_version == akcontrol.AKC_ABI_VERSION,
          "the ABI version is the one this binding was written against",
          "abi=%d" % core.abi_version)
    check(core.product == "aerialkit-f405",
          "the product is the one the sources build for", core.product)
    for name in ("board", "revision", "built"):
        check(bool(getattr(core, name)), "the %s is stamped" % name,
              getattr(core, name))

    # And the one stamp that is compared rather than printed. A stamp that is
    # only printed is not a check: the revision above was read by eye and agreed
    # with nothing, so a library built from another commit reported that other
    # commit through a whole green run and no line of output was false. This is
    # the line that would have been.
    if TREE_REV and TREE_REV != "unknown":
        check(rev_commit(str(core.revision)) == rev_commit(TREE_REV),
              "the library was built from the commit the tree is on",
              "library %s, tree %s" % (core.revision, TREE_REV))


def check_layout(core):
    print("\nthe layout, as the compiler set it")

    # What each struct must have. A third statement of the layout, next to the
    # header and the binding, and the one that catches a field removed from
    # *both* of those - which the offset comparison below cannot, because it
    # would agree with itself.
    expected = {
        "imu": 5, "baro": 5, "gps": 10, "samples": 5,
        "command": 7, "outputs": 6, "state": 38,
    }

    problems = akcontrol.verify(core.lib)
    check(not problems, "every field of every struct is where ctypes thinks",
          "%d problem(s)" % len(problems) if problems else "")
    for line in problems:
        print("        %s" % line)

    for which in sorted(expected):
        got = akcontrol.CLayout()
        rc = core.lib.akc_layout(which.encode(), ctypes.byref(got))
        check(rc == 0 and got.fields == expected[which],
              "%-8s has the %d fields the header declares" % (which, expected[which]),
              "C says %d" % got.fields if rc == 0 else "no layout")

    # Every struct the binding knows is reachable, and a name it does not know
    # is refused rather than answered with something.
    missing = [w for w in akcontrol.STRUCTS
               if akcontrol.CLayout and
               core.lib.akc_layout(w.encode(), ctypes.byref(akcontrol.CLayout())) != 0]
    check(not missing, "no struct the binding declares is missing from the library",
          ", ".join(missing))
    check(core.lib.akc_layout(b"no-such-struct",
                              ctypes.byref(akcontrol.CLayout())) != 0,
          "an unknown struct name is refused")


def check_layout_self_test(core):
    """The mutation test: if the comparison cannot fail, it is not a check.

    A copy of the binding's struct table is corrupted in the two ways a struct
    really goes wrong - a field moved, and a field's type changed - and
    `verify()` has to notice both. Nothing here touches the library; what is
    being shown is that the comparison reads the field list rather than, say,
    only the struct's size.
    """
    print("\nthe layout check, shown to bite")

    saved = akcontrol.STRUCTS
    try:
        # A field moved: swap two adjacent members of the command, which is
        # exactly what an insertion in the header does to everything below it.
        class Moved(ctypes.Structure):
            _fields_ = [
                ("t_ms", ctypes.c_uint32),
                ("roll", ctypes.c_float),
                ("pitch", ctypes.c_float),
                ("yaw", ctypes.c_float),
                ("throttle", ctypes.c_float),
                ("arm_request", ctypes.c_int),   # was angle_mode
                ("angle_mode", ctypes.c_int),    # was arm_request
            ]

        akcontrol.STRUCTS = dict(saved, command=Moved)
        global quiet
        quiet = True
        problems = akcontrol.verify(core.lib, only=["command"])
        quiet = False
        check(bool(problems), "two fields swapped is a disagreement",
              problems[0] if problems else "nothing reported")

        # A field's type changed: a float read as an int is the same size on
        # this machine and a different number in the program.
        class Retyped(ctypes.Structure):
            _fields_ = [
                ("t_ms", ctypes.c_uint32),
                ("roll", ctypes.c_double),       # was c_float
                ("pitch", ctypes.c_float),
                ("yaw", ctypes.c_float),
                ("throttle", ctypes.c_float),
                ("angle_mode", ctypes.c_int),
                ("arm_request", ctypes.c_int),
            ]

        akcontrol.STRUCTS = dict(saved, command=Retyped)
        problems = akcontrol.verify(core.lib, only=["command"])
        check(bool(problems), "a widened field is a disagreement",
              problems[0] if problems else "nothing reported")

        # And a whole struct the library has never heard of.
        akcontrol.STRUCTS = dict(saved, nothing=akcontrol.Imu)
        problems = akcontrol.verify(core.lib, only=["nothing"])
        check(bool(problems), "a struct the library does not have is a disagreement",
              problems[0] if problems else "nothing reported")
    finally:
        akcontrol.STRUCTS = saved


def check_config_identity(core):
    print("\nthe configuration's identity (C8)")

    fresh_config(core, "airframe=0\n")
    base = core.config_hash
    check(base != 0, "a configured core has an identity", "0x%08x" % base)
    check(core.config_hash == base, "and it does not drift when read again")

    # Recomputed here, from the text the library itself says it would save. Two
    # implementations of the same function in two languages, which is the only
    # way a hash crossing a boundary is checked rather than trusted.
    def fnv1a(data):
        h = 0x811C9DC5
        for byte in data:
            h = ((h ^ byte) * 16777619) & 0xFFFFFFFF
        return h

    text = core.config_text()
    check(fnv1a(text.encode()) == base,
          "it is FNV-1a over the saved table, recomputed here",
          "0x%08x vs 0x%08x" % (fnv1a(text.encode()), base))
    check(text.endswith("\n") and "=" in text,
          "and the saved table is the firmware's own name=value format",
          "%d bytes" % len(text))

    # The identity is of the *table*, not of the text. Same table written two
    # ways is one aircraft and must be one number.
    fresh_config(core, "rate_kp_roll=0.900\nairframe=0\n")
    reordered = core.config_hash
    fresh_config(core, "airframe=0\nrate_kp_roll=0.900\n")
    check(reordered == core.config_hash,
          "the order of the lines does not change the identity",
          "0x%08x" % core.config_hash)
    check(reordered != base, "a changed value does change it",
          "0x%08x != 0x%08x" % (reordered, base))

    # A parameter this build has never heard of is skipped by the loader, so it
    # is not part of the aircraft and must not be part of its name.
    fresh_config(core, "airframe=0\nnot_a_parameter=7\n")
    check(core.config_hash == base,
          "a name the build does not have is not part of the identity")

    # Same question, one level down: the *state* has to carry the same number,
    # or an experiment logs one identity and flies another.
    r = Run(core)
    r.step()
    check(core.state().config_hash == core.config_hash,
          "the state reports the same identity the library does",
          "0x%08x" % core.state().config_hash)

    # And a core that has not been configured has no identity at all: zero is
    # not a hash of the defaults, it is the absence of an answer.
    core.reset()
    check(core.config_hash == base, "a reset keeps the configuration")
    check(Run(core).step() and True, "and the core still flies after one")


def check_boundary(core):
    print("\nwhat crosses the boundary (C5, C6, C7)")

    fresh_config(core, "airframe=0\n")
    r = Run(core, throttle=0.0)
    result, out, state = r.fly(1000)
    check(result == akcontrol.AKC_OK, "a fresh frame in time flies",
          akcontrol.describe(result))
    check(state.link_live == 1, "the synthesised receiver is a live link")

    # A sample that is present but old is refused, and refused *before* it is
    # integrated - the whole of C5. 250 ms is the bound; 251 is over it.
    now = 5000
    stale = core.samples(now - (akcontrol.AKC_SAMPLE_MAX_AGE_MS + 1),
                         core.imu(t_ms=now - (akcontrol.AKC_SAMPLE_MAX_AGE_MS + 1)))
    result, _ = core.step(stale, core.command(t_ms=now), now)
    check(result == akcontrol.AKC_SAMPLE_STALE, "a sample past its age bound is refused",
          akcontrol.describe(result))
    edge = core.samples(now - akcontrol.AKC_SAMPLE_MAX_AGE_MS,
                        core.imu(t_ms=now - akcontrol.AKC_SAMPLE_MAX_AGE_MS))
    result, _ = core.step(edge, core.command(t_ms=now), now)
    check(result == akcontrol.AKC_OK, "a sample exactly at the bound is not",
          akcontrol.describe(result))

    # A refused step hands back the frame the core is still holding, rather
    # than leaving the caller's buffer as it found it. That is the difference
    # between a documented contract and a sentence in a comment.
    fresh_config(core, "airframe=0\n")
    r = Run(core, throttle=0.0)
    r.fly(1000)
    held = r.out
    before = held.sequence
    result, after = core.step(stale, core.command(t_ms=now), now)
    check(result == akcontrol.AKC_SAMPLE_STALE and after.sequence == before,
          "a refused step reports the outputs still held, not a new frame",
          "sequence %d -> %d" % (before, after.sequence))
    check(close(after.motor[0], held.motor[0]),
          "and the same motor values the core is holding",
          "%.4f vs %.4f" % (after.motor[0], held.motor[0]))

    # An absent sensor is not a sensor reading zero: a stationary aircraft
    # reports zero rate and is perfectly healthy. Reported, and still flown -
    # the firmware has a failsafe path and this is how a caller reaches it.
    fresh_config(core, "airframe=0\n")
    r = Run(core, throttle=0.0)
    r.fly(1000)
    n = r.now
    dead = core.samples(n, core.imu(t_ms=n, sequence=r.i, valid=False))
    result, _ = core.step(dead, core.command(t_ms=n), n)
    check(result == akcontrol.AKC_NO_IMU,
          "an inertial sample that is not valid is reported, not silently zero",
          akcontrol.describe(result))

    # C6's TTL. The firmware has none on a command value; the boundary does,
    # because here the caller is the link.
    fresh_config(core, "airframe=0\n")
    r = Run(core, throttle=0.0)
    r.fly(1000)
    n = r.now
    late = core.command(t_ms=n - (akcontrol.AKC_COMMAND_MAX_AGE_MS + 1),
                        throttle=0.9, roll=1.0)
    result, _ = core.step(r.core.samples(n, r.core.imu(t_ms=n, sequence=r.i)), late, n)
    state = core.state()
    check(result == akcontrol.AKC_COMMAND_EXPIRED,
          "a command past its TTL is refused", akcontrol.describe(result))
    check(state.command_expired == 1, "and the state says so")
    check(close(state.cmd_throttle, 0.0) and close(state.cmd_roll, 0.0),
          "and the sticks it was asked for are not what the core flew",
          "throttle %.3f roll %.3f" % (state.cmd_throttle, state.cmd_roll))

    # C7: what comes out is normalised, and the encoded frame agrees.
    fresh_config(core, "airframe=0\n")
    r = Run(core, throttle=0.0)
    _, out, _ = r.fly(1000)
    check(all(-0.0001 <= out.motor[i] <= 1.0001 for i in range(4)),
          "motors cross as 0..1",
          " ".join("%.3f" % out.motor[i] for i in range(4)))
    check(all(-1.0001 <= out.servo[i] <= 1.0001 for i in range(2)),
          "servos cross as -1..1",
          " ".join("%.3f" % out.servo[i] for i in range(2)))
    check(all(out.dshot[i] == 0 for i in range(4) if out.motor[i] == 0.0),
          "a motor at zero encodes as DShot zero, which is the off command")
    check(all(out.servo_us[i] != 0 for i in range(2)),
          "and a servo at centre is a pulse width, not a zero",
          " ".join("%d" % out.servo_us[i] for i in range(2)))

    # The sequence is the producer's counter: one per step that flew.
    fresh_config(core, "airframe=0\n")
    r = Run(core, throttle=0.0)
    for _ in range(10):
        r.step()
    first = r.out.sequence
    for _ in range(10):
        r.step()
    check(r.out.sequence == first + 10,
          "the output sequence advances once per step that flew",
          "%d -> %d" % (first, r.out.sequence))
    check(core.state().steps == 20, "and the core counts the steps it took",
          "steps=%d" % core.state().steps)


def check_receiver(core):
    print("\nthe receiver round trip: the firmware's own decoder")

    # The command goes in as the stick shape and is inverted into receiver
    # counts so that the core's own decoder turns it back. What `state.cmd_*`
    # holds is that decoder's answer. If it matches the stick that was asked
    # for, there is one decoder in the system and it is the firmware's.
    def stick(**kwargs):
        fresh_config(core, "airframe=0\n")
        r = Run(core, **kwargs)
        r.fly(600)
        return core.state()

    state = stick(roll=0.5)
    check(close(state.cmd_roll, 0.5, 0.01), "a roll stick round trips",
          "%.4f" % state.cmd_roll)
    state = stick(pitch=-0.75)
    check(close(state.cmd_pitch, -0.75, 0.01), "a pitch stick round trips",
          "%.4f" % state.cmd_pitch)
    state = stick(yaw=0.25)
    check(close(state.cmd_yaw, 0.25, 0.01), "a yaw stick round trips",
          "%.4f" % state.cmd_yaw)
    state = stick(throttle=0.0)
    check(close(state.cmd_throttle, 0.0), "throttle at the bottom is zero",
          "%.4f" % state.cmd_throttle)
    state = stick(throttle=1.0)
    check(close(state.cmd_throttle, 1.0), "throttle at the top is one",
          "%.4f" % state.cmd_throttle)

    # The deadband is the firmware's, and this check is what says the inversion
    # did not go around it: a stick inside it decodes as zero even though a
    # count was sent.
    state = stick(roll=0.01)
    check(close(state.cmd_roll, 0.0), "a stick inside the deadband decodes as zero",
          "%.4f" % state.cmd_roll)
    state = stick(roll=0.05)
    check(state.cmd_roll > 0.0, "and one outside it does not",
          "%.4f" % state.cmd_roll)

    state = stick(arm=True)
    check(state.cmd_arm_request == 1, "the arm switch round trips")
    state = stick(arm=False)
    check(state.cmd_arm_request == 0, "and so does its absence")
    state = stick(angle_mode=True)
    check(state.cmd_angle_mode == 1, "the mode switch round trips")


def check_flying(core):
    print("\nflying, on numbers this file did not choose")

    fresh_config(core, "airframe=0\n")
    r = Run(core, throttle=0.0)
    r.fly(50)
    check(core.state().converged == 0, "a core that has just started is not converged")
    r.fly(200)
    check(core.state().converged == 1,
          "the estimator levels itself on the accelerometer",
          "converged by %d ms" % core.state().now_ms)

    fresh_config(core, "airframe=0\n")
    r = Run(core, throttle=0.0)
    armed_at = r.until_armed()
    check(armed_at is not None, "it arms when the switch is held",
          "%s ms" % armed_at)
    check(armed_at is not None and armed_at >= 500,
          "and not before the hold time has passed", "arm_hold_ms=500")

    # An armed aircraft at zero throttle turns its motors at idle. Not zero:
    # a motor that is stopped is a motor that cannot be commanded.
    fresh_config(core, "airframe=0\n")
    r = Run(core, throttle=0.0)
    r.until_armed()
    _, out, _ = r.fly(20)
    check(out.motor[0] > 0.0, "an armed aircraft idles its motors",
          "%.4f" % out.motor[0])

    # Throttle above the stick's low position with the switch already on: the
    # motors have to follow it.
    for throttle in (0.2, 0.5, 0.8):
        fresh_config(core, "airframe=0\n")
        r = Run(core, throttle=0.0)
        r.until_armed()
        r.throttle = throttle
        _, out, _ = r.fly(200)
        check(out.motor[0] > throttle * 0.5,
              "at throttle %.1f the motors are turning" % throttle,
              "motor=%.4f" % out.motor[0])
        if throttle == 0.8:
            high = out.motor[0]
    check(high > 0.5, "and at 0.8 they are well up", "%.4f" % high)

    # The rate loop's setpoint in rate mode is the configured maximum, and the
    # default is 700 deg/s.
    fresh_config(core, "airframe=0\n")
    r = Run(core, throttle=0.0)
    r.until_armed()
    r.roll = 1.0
    r.fly(200)
    state = core.state()
    check(close(math.degrees(state.rate_setpoint[0]), 700.0, 1.0),
          "full roll in rate mode asks for max_rate_dps",
          "%.1f deg/s" % math.degrees(state.rate_setpoint[0]))

    # Angle mode is a different law: the same stick asks for a tilt, and the
    # setpoint is the angle loop's output rather than a rate.
    fresh_config(core, "airframe=0\n")
    r = Run(core, throttle=0.0, angle_mode=True)
    r.until_armed()
    r.roll = 1.0
    r.fly(200)
    state = core.state()
    check(state.cmd_angle_mode == 1 and
          not close(math.degrees(state.rate_setpoint[0]), 700.0, 1.0),
          "angle mode asks for a different setpoint from the same stick",
          "%.1f deg/s" % math.degrees(state.rate_setpoint[0]))

    # The configuration has to reach the control law, or an identity is a label
    # on nothing. Same stick, same gyro, two gains, an integral switched off so
    # that what is compared is the proportional term the gain is in.
    torques = {}
    for kp in (0.25, 0.90):
        fresh_config(core, "airframe=0\nrate_ki_roll=0.0\nrate_kp_roll=%.3f\n" % kp)
        r = Run(core, throttle=0.0, roll=0.05)
        r.until_armed()
        r.roll = 0.05
        r.fly(100)
        torques[kp] = core.state().torque[0]
    check(torques[0.90] > torques[0.25] * 2.0,
          "a changed rate gain changes what the loop asks for",
          "kp 0.25 -> %.4f, kp 0.90 -> %.4f" % (torques[0.25], torques[0.90]))


def check_reproducible(core):
    """Two runs of the same experiment, compared to the last bit.

    This is the acceptance criterion's own claim - "a reproducible rate and
    attitude experiment" - stated where it can be checked without a plant. What
    is compared is the whole of what crosses back: the result code, every motor
    and servo, the encoded frames, the output sequence, and the estimator's
    attitude.
    """
    print("\ntwo runs of one experiment")

    config = "airframe=0\n"

    def run_once():
        fresh_config(core, config)
        trace = []
        r = Run(core, throttle=0.0)
        r.until_armed()
        for i in range(600):
            # A rate disturbance on one axis, so the loop is doing something
            # rather than holding still and agreeing trivially.
            r.accel = LEVEL
            now = r.now
            samples = core.samples(
                now,
                core.imu(t_ms=now, sequence=r.i,
                         gyro=(0.02 * math.sin(i * 0.05), 0.01, -0.005)),
            )
            command = core.command(t_ms=now, roll=0.1, throttle=0.3, arm_request=True)
            result, out = core.step(samples, command, now)
            state = core.state()
            trace.append((
                result,
                tuple(out.motor), tuple(out.servo),
                tuple(out.dshot), tuple(out.servo_us), out.sequence,
                tuple(state.q_wxyz), state.roll, state.pitch, state.yaw,
                tuple(state.torque),
            ))
            r.i += 1
            r.now = now + r.dt
        return trace

    first = run_once()
    second = run_once()

    check(len(first) == len(second), "both runs are the same length")
    check(first == second, "and every number in them is identical",
          "%d steps compared" % len(first))

    if first != second:
        for i, (a, b) in enumerate(zip(first, second)):
            if a != b:
                print("        first difference at step %d:" % i)
                print("          %s" % (a,))
                print("          %s" % (b,))
                break

    # And the traces are not trivially equal - a run that never moved would
    # also be reproducible. This is the check that the check is looking at
    # something.
    motors = {step[1] for step in first}
    check(len(motors) > 1, "the trace is not a constant",
          "%d distinct motor frames" % len(motors))
    attitudes = {step[6] for step in first}
    check(len(attitudes) > 1, "and the attitude estimate is moving",
          "%d distinct attitudes" % len(attitudes))


def main():
    print("akcontrol_check: the C control core through the ABI a simulation uses")

    try:
        core = akcontrol.Core(LIB)
    except (akcontrol.LoadError, akcontrol.AbiError, akcontrol.LayoutError) as exc:
        print("  FAIL  %s" % exc)
        print("akcontrol_check: 1 checks, 1 failed")
        return 1

    print("  library: %s" % core.path)

    check_identity(core)
    check_layout(core)
    check_layout_self_test(core)
    check_config_identity(core)
    check_boundary(core)
    check_receiver(core)
    check_flying(core)
    check_reproducible(core)

    print("\nakcontrol_check: %d checks, %d failed" % (checks, failures))
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
