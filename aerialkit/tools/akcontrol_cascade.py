#!/usr/bin/env python3
"""The cascade on the firmware's estimator: one flight, two instruments.

`akcontrol_experiment.py` closes the firmware's *own* control loop on the
sibling repository's plant and compares the estimator's attitude with the
plant's truth alongside it. This file asks the question that leaves open: what
does the sibling's **cascade** do when the attitude and the rate it steers by are
the estimator's rather than the truth's?

What the cascade steers by, and why that is worth replacing
-----------------------------------------------------------
`actuator_loop.zup_state_of` hands the cascade the plant's *true* attitude and
*true* body rate. Every number item 7 measured - a 3 m step settling to 0.000 m
at 8.3 degrees of tilt - is therefore a statement about the control law with
perfect instrumentation, and says nothing about either on a noisy, delayed
inertial measurement. That is the gap this tool closes, and it is the one the
ledger named as what was left of item 7.

**The estimator is the firmware's, not a second one written here.** It is
`ak_estimator.c`, over the ABI, stepped by `akc_step` with nothing but the
plant's `sense()`. Two consequences, both deliberate:

* it is the code that flies the aircraft, so the measurement is about the
  estimator that will be there rather than about a stand-in; and
* `ak_control.h` says *"There is no field here for truth"* - the plant's
  attitude is in no struct in the ABI - so a loop that reads the estimate
  **cannot** accidentally read the truth. The seam is honest by construction
  rather than by care.

The truth is carried along anyway, and used for exactly one thing: scoring the
estimate. It never reaches the controller.

The boundary, stated rather than left to be found
-------------------------------------------------
**Stage 1 stays on the truth in both arms.** The cascade's first stage needs a
position and a velocity, and there is no honest way to give it either: the
plant's `sense()` produces a gyro and an accelerometer and nothing else - no
barometer, no GNSS, no range - and `akc_state_t` has no position field. A
position invented for this loop would be the answer written into the question.
So what is measured here is the **attitude and rate instrumentation**, which is
the half that exists, and the position half is left named as the gap it is.

The arming case, because a falling aircraft is unobservable
----------------------------------------------------------
The flight starts on the ground with the motors stopped, and the estimator is
fed for a settling second before the cascade takes over. That is not staging: a
body in free fall has no specific force for an accelerometer to measure, so the
attitude it implies does not exist, and the plant says so where `ground_enabled`
is defined. A real aircraft arms on a bench for the same reason. The settling
second is run in **both** arms, with the same motor commands, so the two start
from the same plant state; and it is run with the aircraft already tilted, so
the estimator has something to converge from.

What the checks in this file are, and what they are not
------------------------------------------------------
They are invariants of the **harness**: that the frame map is the sibling's own,
that the falsification arm reproduces the truth arm exactly, that the ABI
accepts every step, and that the substitution is actually wired. They are not
assertions that the flight is good. How well the sensed arm flies is the
*measurement*, and a bad number there is a result to print rather than a check
to fail - a tool that failed whenever its subject misbehaved would only be able
to report one thing.

    python3 tools/akcontrol_cascade.py                 # the measurement
    python3 tools/akcontrol_cascade.py --falsify       # + the mechanism arms
"""

from __future__ import annotations

import argparse
import math
import os
import sys
from types import SimpleNamespace

HERE = os.path.dirname(os.path.abspath(__file__))
FIRMWARE = os.path.dirname(HERE)
if HERE not in sys.path:
    sys.path.insert(0, HERE)

import akcontrol                                                    # noqa: E402
from akcontrol_experiment import CONFIG_QUAD, DT_MS, tilt_error_deg  # noqa: E402

DEFAULT_REPO = os.environ.get("AK_REPO", os.path.dirname(FIRMWARE))
DEFAULT_LIB = os.path.join(FIRMWARE, "build-host", "libaerialkit-control.so")

# The sibling's own test numbers, so a figure here and a figure there are about
# the same aircraft rather than about two.
ARM, MASS, YAW_C, TMAX = 0.2, 1.0, 0.02, 6.0
DT = DT_MS / 1000.0                     # 500 Hz, the firmware's control rate
SECONDS = 16.0
SETTLE_S = 1.0
TARGET = (3.0, 0.0, 2.0)
TILT0_DEG = (20.0, -10.0)               # the arming case: it arms while tilted
TOL_M = 0.05

# The estimator's accelerometer trust band, from `ak_estimator.c`. Copied here
# rather than read out of the C because the point of measuring the band
# occupancy is to compare it against what the firmware believes; a number
# imported from the thing under test could not disagree with it.
ACCEL_TRUST_LOW, ACCEL_TRUST_HIGH = 0.7, 1.3

# Instrumentation, and these are *chosen*, not measured - the standing the
# plant's own `motor_tau_s` claims for itself. 0.3 dps and 0.02 g are a
# consumer-grade MEMS part at rest; two steps of delay is 4 ms, which is the
# transport plus the sample-and-hold of a part reporting faster than the loop
# consumes it.
NOISE = {"gyro_noise_dps": 0.3, "accel_noise_g": 0.02, "sample_delay_steps": 2}
CLEAN = {"gyro_noise_dps": 0.0, "accel_noise_g": 0.0, "sample_delay_steps": 0}

# The scale that closes the accelerometer trust gate without touching the
# accelerometer's direction. 3 g is outside [0.7, 1.3] whatever the manoeuvre
# does, so the estimator integrates the gyro alone - which is a *deliberate lie
# about the magnitude*, and is therefore a diagnostic and never a proposal.
GATE_SHUT_SCALE = 3.0


def load_sibling(path):
    """The sibling repository's plant, cascade and seam, or the reason not."""
    if not os.path.isdir(path):
        return None, "%s is not a directory" % path
    sys.path.insert(0, path)
    try:
        import numpy as np
        from aerial_kit.airframes.multirotor import MultirotorAirframe
        from aerial_kit.controllers import (
            CascadeController, CascadeGains, PIDController)
        from aerial_kit.dynamics.actuator_loop import (
            FRD_TO_ZUP_BODY, NED_TO_ENU, fly, zup_attitude_quat)
        from aerial_kit.dynamics.multirotor_actuator import (
            QUAD_X_SPIN, ActuatorPlant, ActuatorPlantParams, quad_x_positions)
        from aerial_kit.dynamics.rotations import quat_to_rotmat, rotmat_to_quat
        from aerial_kit.types import SimState
    except Exception as exc:                            # noqa: BLE001
        return None, "%s: %s" % (path, exc)
    return SimpleNamespace(np=np, MultirotorAirframe=MultirotorAirframe,
                           CascadeController=CascadeController,
                           CascadeGains=CascadeGains, PIDController=PIDController,
                           FRD_TO_ZUP_BODY=FRD_TO_ZUP_BODY,
                           NED_TO_ENU=NED_TO_ENU, fly=fly,
                           zup_attitude_quat=zup_attitude_quat,
                           QUAD_X_SPIN=QUAD_X_SPIN,
                           ActuatorPlant=ActuatorPlant,
                           ActuatorPlantParams=ActuatorPlantParams,
                           quad_x_positions=quad_x_positions,
                           quat_to_rotmat=quat_to_rotmat,
                           rotmat_to_quat=rotmat_to_quat, SimState=SimState), None


def quat_from_roll_pitch(roll_deg, pitch_deg):
    """Body-axis roll then pitch, w-first. Written here rather than imported so
    that a wrong one is this file's wrong one and not the sibling's."""
    r, p = math.radians(roll_deg) / 2.0, math.radians(pitch_deg) / 2.0
    cr, sr, cp, sp = math.cos(r), math.sin(r), math.cos(p), math.sin(p)
    return [cr * cp, sr * cp, cr * sp, -sr * sp]


def zup_quat_from_ned(s, q_ned):
    """The firmware's body->NED quaternion as the sibling's z-up body->world one.

    The same map, applied the same way, as `actuator_loop.zup_attitude_quat`
    applies to the plant's own attitude - because the estimator's frame is the
    plant's frame. `check_the_map_matches` proves the two agree rather than
    asserting that they do, since a frame map that is right for one vector and
    hand-written for the next is the defect trap 88 is about.
    """
    return s.rotmat_to_quat(
        s.NED_TO_ENU @ s.quat_to_rotmat(q_ned) @ s.FRD_TO_ZUP_BODY)


def tilt_of(s, q):
    """The angle between a body's own up axis and the world's, in degrees.

    The same quantity `akcontrol_experiment.tilt_error_deg` scores, computed
    from one rotation rather than from a pair so that a single attitude can be
    reported against a moving one, step by step.
    """
    r = s.quat_to_rotmat(s.np.asarray(q, dtype=float))
    return math.degrees(math.acos(max(-1.0, min(1.0, float(r[2, 2])))))


class OnEstimate:
    """A sensor adapter, not a controller: two fields replaced, the rest passed.

    It steps the firmware core once per call with the plant's own inertial
    sample, replaces the state's attitude and body rate with the estimator's,
    and hands the result to the cascade unchanged. It computes no control law -
    every field of the target it returns is the inner controller's - which is
    what makes the two arms a comparison of instrumentation rather than of
    controllers.

    `bypass` is the falsification: the ABI is stepped exactly as it is
    otherwise, and its answer is discarded in favour of the plant's truth. The
    arm that sets it must then be indistinguishable from the arm that never
    wrapped anything, and if it is not, the wrapper is doing more than it says.

    `accel_scale` multiplies the accelerometer *magnitude* only, leaving its
    direction alone. At 1.0 it is the plant's measurement as it stands; at
    `GATE_SHUT_SCALE` it closes the estimator's trust gate and nothing else.
    """

    def __init__(self, s, inner, plant, core, *, bypass=False, accel_scale=1.0):
        self.s, self.inner, self.plant, self.core = s, inner, plant, core
        self.bypass = bypass
        self.accel_scale = accel_scale
        self.results = {}
        self.tilt_err = []
        self.accel_mag = []
        self.age_ms = []
        self.rate_err = []
        self.calls = 0
        self.converged_at = None
        self.converged_tilt_err = None
        self.flight = slice(0, None)

    @property
    def command_kind(self):
        return self.inner.command_kind

    @property
    def name(self):
        return type(self.inner).__name__ + "+estimator"

    def sense_and_step(self):
        """One inertial sample through the firmware, and what came back.

        Split out of `compute` because the settling second needs the estimator
        and nothing else: the inner controller would have to be handed a
        waypoint it is not being flown to, and inventing one is how a settle
        phase turns into a flight.
        """
        np = self.s.np
        sample = self.plant.sense()

        # `now` is when the sample is *consumed* and `t_ms` when it was *taken*.
        # They are not the same number and the difference is the delay: handing
        # the ABI the sample's own timestamp would report every sample as brand
        # new, and the delay this run exists to measure would vanish into the
        # one field that is there to expose it.
        now = int(round(1000.0 * float(self.plant.time_s)))
        taken = int(round(float(sample.t_ms)))
        accel = [self.accel_scale * float(v) for v in sample.accel_g]
        samples = self.core.samples(
            now,
            self.core.imu(t_ms=taken, sequence=int(sample.sequence),
                          gyro=[float(v) for v in sample.gyro_rps],
                          accel=accel, valid=1),
            sequence=int(sample.sequence),
        )
        # Disarmed and centred: the estimator is the whole of what is wanted
        # here, and a stick input the cascade did not ask for would be this file
        # flying the aircraft.
        command = self.core.command(t_ms=now, roll=0.0, pitch=0.0, yaw=0.0,
                                   throttle=0.0, angle_mode=False,
                                   arm_request=False)
        result, _out = self.core.step(samples, command, now)
        abi = self.core.state()

        self.calls += 1
        self.results[result] = self.results.get(result, 0) + 1
        self.age_ms.append(now - taken)
        # The band occupancy is measured on the plant's own accelerometer, not
        # on the scaled copy handed to the ABI: the question is what the sensor
        # said, not what this file made of it.
        self.accel_mag.append(float(np.linalg.norm(
            np.asarray(sample.accel_g, dtype=float))))
        truth_ned = np.array(self.plant.attitude_quat, dtype=float)
        estimate_ned = np.array([abi.q_wxyz[i] for i in range(4)], dtype=float)
        self.tilt_err.append(tilt_error_deg(estimate_ned, truth_ned))
        est_rate = np.array([abi.gyro[i] for i in range(3)], dtype=float)
        self.rate_err.append(float(np.linalg.norm(
            est_rate - np.array(self.plant.body_rates, dtype=float))))
        if self.converged_at is None and int(abi.converged):
            self.converged_at = now
        return estimate_ned, est_rate

    def compute(self, state, target_waypoint, cfg):
        if self.bypass:
            # Still step the ABI, so the arm is the same work with the answer
            # thrown away rather than a different amount of work.
            self.sense_and_step()
            return self.inner.compute(state, target_waypoint, cfg)

        estimate_ned, est_rate = self.sense_and_step()
        substituted = self.s.SimState(
            position=state.position,
            velocity=state.velocity,
            t=state.t,
            attitude_quat=zup_quat_from_ned(self.s, estimate_ned),
            body_rates=self.s.FRD_TO_ZUP_BODY @ est_rate,
        )
        return self.inner.compute(substituted, target_waypoint, cfg)


def build(s, noise, seed):
    """A fresh aircraft, on the ground, armed-tilted, motors stopped.

    Same construction as the sibling's own tests, plus the instrumentation. The
    plant is seeded, so the noisy arm is reproducible and the noise is a fixed
    draw rather than a different experiment every run.
    """
    plant = s.ActuatorPlant(
        params=s.ActuatorPlantParams(
            mass_kg=MASS, max_thrust_per_motor_n=TMAX, yaw_torque_coeff=YAW_C,
            seed=seed, **noise),
        motor_positions=s.quad_x_positions(ARM),
        spin=s.QUAD_X_SPIN,
    )
    plant.attitude_quat[...] = quat_from_roll_pitch(*TILT0_DEG)
    plant.motor_command[...] = 0.0
    plant.motor_actual[...] = 0.0
    airframe = s.MultirotorAirframe(arms=4, layout="x", arm_length_m=ARM,
                                    mass_kg=MASS, yaw_torque_coeff=YAW_C)
    return plant, airframe


def fly_arm(s, noise, *, on_estimate, lib, bypass=False,
            flight_accel_scale=1.0, seed=11):
    """Settle, then fly one mission with one instrument substituted.

    `flight_accel_scale` is applied only once the flight has begun, so that an
    arm which closes the trust gate still converges from the same place as the
    arm that does not. Changing it during the settling second as well would
    confound "the correction is wrong in flight" with "the correction never
    ran", and those two have different answers.
    """
    plant, airframe = build(s, noise, seed)
    controller = s.CascadeController(
        inner=s.PIDController(),
        gains=s.CascadeGains(mass_kg=MASS, max_tilt_deg=35.0))
    core = None
    adapter = None
    if on_estimate:
        core = akcontrol.Core(lib)
        core.config(CONFIG_QUAD)
        core.set_board_outputs(4, 0)
        adapter = OnEstimate(s, controller, plant, core, bypass=bypass)
        controller = adapter

    # The settling second: motors stopped, held up by the ground, which is the
    # only condition under which the accelerometer can see which way is down.
    # Run in both arms with the same commands, so the two start together.
    for _ in range(int(round(SETTLE_S / DT))):
        if adapter is not None:
            adapter.sense_and_step()
        plant.step(DT)

    settle_calls = 0
    if adapter is not None:
        settle_calls = adapter.calls
        adapter.converged_tilt_err = adapter.tilt_err[-1]
        adapter.accel_scale = flight_accel_scale

    trace = s.fly(controller, airframe, plant,
                  s.np.array(TARGET, dtype=float), dt=DT,
                  steps=int(round(SECONDS / DT)))

    if adapter is not None:
        # Only the flight, so the settling transient does not swamp the mean
        # with a number that is mostly how far the aircraft started from level.
        adapter.flight = slice(settle_calls, None)
    return trace, adapter


def summarise(trace, adapter):
    """The flight, plus what the estimator was doing while it was flown."""
    row = {
        "final_err_m": float(trace.position_error[-1]),
        "max_err_m": float(trace.position_error.max()),
        "settled_s": trace.settled_after(TOL_M),
        "max_tilt_deg": float(trace.tilt_deg.max()),
        "clamped_steps": int(trace.clamped.sum()),
        "final_speed": float(abs(trace.velocity[-1]).max()),
    }
    if adapter is not None:
        err = adapter.tilt_err[adapter.flight]
        mag = adapter.accel_mag[adapter.flight]
        row["conv_ms"] = adapter.converged_at
        row["conv_err_deg"] = adapter.converged_tilt_err
        row["tilt_err_mean_deg"] = sum(err) / len(err)
        row["tilt_err_max_deg"] = max(err)
        row["age_ms"] = max(adapter.age_ms)
        row["rate_err_max"] = max(adapter.rate_err)
        row["accel_min"] = min(mag)
        row["accel_max"] = max(mag)
        row["in_band"] = sum(1 for m in mag
                             if ACCEL_TRUST_LOW < m < ACCEL_TRUST_HIGH)
        row["band_n"] = len(mag)
        row["results"] = dict(adapter.results)
    return row


def check_the_map_matches(s):
    """The tool's frame map against the sibling's own, on the same attitude.

    `rotmat_to_quat` does not canonicalise the sign, so two spellings of one
    rotation are compared by the dot product and never elementwise - the lesson
    item 7's tests already carry.
    """
    plant, _ = build(s, CLEAN, 3)
    plant.attitude_quat[...] = quat_from_roll_pitch(*TILT0_DEG)
    mine = s.np.array(zup_quat_from_ned(s, s.np.array(plant.attitude_quat,
                                                     dtype=float)))
    theirs = s.np.array(s.zup_attitude_quat(plant))
    dot = abs(float(s.np.dot(mine, theirs)))
    return dot, dot > 1.0 - 1e-12


ROW_KEYS = ["final_err_m", "max_err_m", "settled_s", "max_tilt_deg",
            "clamped_steps", "final_speed", "conv_ms", "conv_err_deg",
            "tilt_err_mean_deg", "tilt_err_max_deg", "age_ms", "rate_err_max"]


def cell(value):
    if value is None:
        return "--"
    if isinstance(value, float):
        return "%.4f" % value
    return str(value)


def print_table(rows, columns):
    print("%-22s %s" % ("", " ".join("%16s" % c for c in columns)))
    for key in ROW_KEYS:
        print("%-22s %s"
              % (key, " ".join("%16s" % cell(rows[c].get(key)) for c in columns)))


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--repo", default=DEFAULT_REPO,
                        help="the sibling aerial-kit checkout")
    parser.add_argument("--lib", default=DEFAULT_LIB,
                        help="libaerialkit-control.so")
    parser.add_argument("--falsify", action="store_true",
                        help="also run the falsification and mechanism arms")
    args = parser.parse_args(argv)

    s, why = load_sibling(args.repo)
    if s is None:
        print("akcontrol_cascade: %s" % why)
        return 2
    if not os.path.exists(args.lib):
        print("akcontrol_cascade: %s is not built - `make control-check` builds it"
              % args.lib)
        return 2

    failures = []
    dot, ok = check_the_map_matches(s)
    print("frame map vs the sibling's own: |dot| = %.17g  %s"
          % (dot, "exact" if ok else "FAIL"))
    if not ok:
        failures.append("the frame map disagrees with zup_attitude_quat")

    truth, _ = fly_arm(s, NOISE, on_estimate=False, lib=args.lib)
    sensed, adapter = fly_arm(s, NOISE, on_estimate=True, lib=args.lib)
    rows = {"truth": summarise(truth, None), "sensed": summarise(sensed, adapter)}

    print()
    print_table(rows, ["truth", "sensed"])
    delta = float(s.np.abs(sensed.position - truth.position).max())
    print("%-22s %16s %16.4f" % ("peak |delta position| m", "--", delta))
    print("%-22s %16s %16.4f"
          % ("peak |delta tilt| deg", "--",
             float(s.np.abs(sensed.tilt_deg - truth.tilt_deg).max())))
    print("%-22s %16s %16s"
          % ("result codes", "--", str(rows["sensed"]["results"])))

    band = rows["sensed"]["in_band"] / float(rows["sensed"]["band_n"])
    print()
    print("estimator converged at %s ms (the settling second is %d ms), from "
          "%.3f deg of tilt error"
          % (rows["sensed"]["conv_ms"], int(SETTLE_S * 1000),
             rows["sensed"]["conv_err_deg"]))
    print("sample age %s ms of a %d-step delay; %d ABI steps"
          % (rows["sensed"]["age_ms"], NOISE["sample_delay_steps"],
             adapter.calls))
    print("accelerometer magnitude in flight: %.4f to %.4f g, inside the "
          "firmware's [%.1f, %.1f] trust band for %d of %d samples (%.1f%%)"
          % (rows["sensed"]["accel_min"], rows["sensed"]["accel_max"],
             ACCEL_TRUST_LOW, ACCEL_TRUST_HIGH, rows["sensed"]["in_band"],
             rows["sensed"]["band_n"], 100.0 * band))

    if rows["sensed"]["conv_ms"] is None:
        failures.append("the estimator never converged in the settling second")
    if set(rows["sensed"]["results"]) != {0}:
        failures.append("the ABI refused steps: %s" % rows["sensed"]["results"])
    if rows["sensed"]["tilt_err_max_deg"] <= 0.0:
        failures.append("the estimator's attitude is exact, which it cannot be - "
                        "the substitution is not wired")
    if delta == 0.0:
        failures.append("the two arms flew identically - the estimate is not in "
                        "the loop")

    if args.falsify:
        bypass, _ = fly_arm(s, NOISE, on_estimate=True, lib=args.lib,
                            bypass=True)
        same = float(s.np.abs(bypass.position - truth.position).max())
        print()
        print("falsification: the bypass arm puts the *truth* through the same "
              "wrapper, so it must be the truth arm exactly")
        print("  peak |bypass - truth| position: %.3g m" % same)
        if same != 0.0:
            failures.append("the bypass arm is not the truth arm: %.3g m apart, "
                            "so the wrapper does more than substitute two "
                            "fields" % same)

        # The mechanism: the same clean flight, converged the same way, with the
        # accelerometer correction switched off at the moment the flight starts
        # and at no other time.
        clean, clean_ad = fly_arm(s, CLEAN, on_estimate=True, lib=args.lib)
        gyro, gyro_ad = fly_arm(s, CLEAN, on_estimate=True, lib=args.lib,
                                flight_accel_scale=GATE_SHUT_SCALE)
        rows2 = {"clean": summarise(clean, clean_ad),
                 "gyro only": summarise(gyro, gyro_ad)}
        print()
        print_table(rows2, ["clean", "gyro only"])
        print("both converged to %.4f and %.4f deg of tilt error before the "
              "flight began, so the only difference between them is the "
              "accelerometer correction during it"
              % (rows2["clean"]["conv_err_deg"],
                 rows2["gyro only"]["conv_err_deg"]))
        ratio = (rows2["clean"]["tilt_err_mean_deg"]
                 / max(rows2["gyro only"]["tilt_err_mean_deg"], 1e-9))
        print("in-flight attitude error, correction on over correction off: "
              "%.4f / %.4f = %.2fx"
              % (rows2["clean"]["tilt_err_mean_deg"],
                 rows2["gyro only"]["tilt_err_mean_deg"], ratio))

        if rows2["clean"]["in_band"] != rows2["clean"]["band_n"]:
            failures.append(
                "the accelerometer left the trust band in flight, so the gate "
                "did fire and this run does not measure what it says: %d of %d"
                % (rows2["clean"]["in_band"], rows2["clean"]["band_n"]))

    print()
    if failures:
        for line in failures:
            print("FAIL: %s" % line)
        return 1
    print("akcontrol_cascade: ok (the harness; the flight is the measurement)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
