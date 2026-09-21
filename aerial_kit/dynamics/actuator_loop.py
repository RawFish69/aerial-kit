"""Flying the actuator plant with a controller: the closed loop, and the seam it needs.

``multirotor_actuator.py`` is a plant whose only input is four bounded motor
commands. ``quad_x_seam.py`` converts a wrench into them. ``controllers/cascade.py``
converts an acceleration demand into a wrench. This module is the fourth piece -
the loop that runs all three together - and the one piece of arithmetic none of
them owns: **the plant is NED/FRD and everything above it is z-up**.

The conversion, and the 90 degrees that looks like a bug
-------------------------------------------------------
Both world frames are right-handed and both are attached to the same ground, so
the map between them is a proper rotation and nothing physical happens when it
is applied. It is not the identity, though, and the surprise is worth writing
down before it is discovered: **a level aircraft heading north reports a heading
of 90 degrees** in this frame, because ENU's ``y`` is north where NED's ``x`` is
north, and the heading is read off the body ``x`` axis. The aircraft is not
rotated by 90 degrees; the axes it is measured against are. ``test_cascade.py``
asserts that number so that it is a recorded fact rather than something the next
reader has to derive, and the loops above never see it: the cascade's desired
heading defaults to the heading the aircraft already has.

Two frames, one map:

* world position, velocity and attitude: ``T = NED_TO_ENU``, the same matrix
  for all three.
* body vectors (an attitude on the right, and a body rate): the FRD-to-z-up
  ``flip = diag(1, -1, -1)`` - the same ``flip``, and the same reason for it,
  as ``quad_x_seam.z_up_wrench_to_frd``.

**The world map is a swap, not a ``z`` negation, and getting that wrong is
invisible in the attitude and fatal in the position.** ``NED_TO_ENU`` sends
NED ``x`` (north) to ENU ``y`` and NED ``y`` (east) to ENU ``x``; only the
third component is a plain negation. An earlier version of
:func:`zup_state_of` applied ``(x, y, -z)`` to position and velocity while
applying the full matrix to the attitude. Every attitude then read correctly,
every unit test on a stationary aircraft passed, and the closed loop flew
away - because a controller handed a north-south/east-west-swapped position
asks for a correction 90 degrees from the one it wants, and one 90-degree
error is exactly a flight along the diagonal.

What this loop closes on
------------------------
The plant's **true state**, not ``sense()``. The plant has a noisy, delayed
sample with the firmware's own sign conventions on the accelerometer precisely
so that an estimator can be driven from it, and this loop does not contain an
estimator - the firmware has one and it is exercised over the ABI, which is a
different claim about a different piece of code. A trace from here is therefore
a statement about the *airframe and the control law*, and says nothing about how
either behaves on a noisy inertial measurement. That is stated wherever one is
cited rather than left to be assumed.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Mapping

import numpy as np

from ..interfaces import Controller
from ..types import SimState, Waypoint
from .multirotor_actuator import ActuatorPlant
from .quad_x_seam import motor_commands
from .rotations import quat_to_rotmat, rotmat_to_quat, yaw_of

# NED world coordinates as ENU world coordinates.
NED_TO_ENU = np.array([[0.0, 1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, -1.0]])
# FRD body coordinates as z-up body coordinates. The same map the seam applies
# to a wrench, which is a half turn about body x and therefore proper - so a
# moment and a body rate need no extra sign of their own.
FRD_TO_ZUP_BODY = np.diag([1.0, -1.0, -1.0])


def zup_attitude_quat(plant: ActuatorPlant) -> np.ndarray:
    """The plant's attitude as a z-up body-to-world rotation, w-first.

    ``R_enu = T @ R_ned @ flip``, then back through a quaternion because that is
    what ``SimState`` carries. The round trip is checked against the matrix in
    ``test_cascade.py`` rather than assumed.
    """
    rotation = (
        NED_TO_ENU @ quat_to_rotmat(plant.attitude_quat) @ FRD_TO_ZUP_BODY
    )
    return rotmat_to_quat(rotation)


def zup_state_of(plant: ActuatorPlant) -> SimState:
    """The plant's state in the frame everything above it speaks.

    World vectors go through ``NED_TO_ENU`` - position, velocity and attitude
    alike, the same matrix, because they are all the same world frame and a
    map that is right for one of them is right for the three.

    Body vectors go through ``FRD_TO_ZUP_BODY``: the attitude on its right,
    and the body rate directly. A rate is legitimate under that flip because
    the flip is a proper rotation and an angular velocity transforms like a
    vector under one - it is only under an *improper* map that a pseudovector
    needs a sign of its own, which is the same argument `quad_x_seam` makes
    for the moment.
    """
    position = np.asarray(plant.position, dtype=float)
    velocity = np.asarray(plant.velocity, dtype=float)
    return SimState(
        position=NED_TO_ENU @ position,
        velocity=NED_TO_ENU @ velocity,
        t=float(plant.time_s),
        attitude_quat=zup_attitude_quat(plant),
        body_rates=FRD_TO_ZUP_BODY @ np.asarray(plant.body_rates, dtype=float),
    )


@dataclass
class ActuatorTrace:
    """One run of the loop: what was asked for, what the motors did, where it went.

    Positions and attitudes are in the **z-up ENU frame the cascade works in**,
    not the plant's. ``motor_cmd`` is what the allocator asked the motors for and
    ``motor_actual`` is what they gave: the two differ by the lag and by
    saturation, and ``clamped`` counts the motors that saturated on each step.
    Keeping both is the point of ``ActuatorPlant`` existing at all, so a trace
    that only kept ``motor_actual`` would throw away the half that says whether
    the aircraft was ever asked for something it could not do.
    """

    t: np.ndarray
    position: np.ndarray
    velocity: np.ndarray
    attitude_quat: np.ndarray
    tilt_deg: np.ndarray
    heading_deg: np.ndarray
    accel_cmd: np.ndarray
    thrust_n: np.ndarray
    moment_body: np.ndarray
    motor_cmd: np.ndarray
    motor_actual: np.ndarray
    clamped: np.ndarray
    target_position: np.ndarray
    steps: int = field(default=0)

    @property
    def position_error(self) -> np.ndarray:
        """Distance from the target at every step."""
        return np.linalg.norm(self.position - self.target_position, axis=1)

    def settled_after(self, tolerance_m: float) -> float | None:
        """The time after which the error never exceeds ``tolerance_m`` again.

        ``None`` if there is no such time - which is the answer that matters, so
        it is returned rather than a large number that reads like a failure to
        converge in time.
        """
        within = self.position_error <= tolerance_m
        if not within[-1] or not within.any():
            return None
        first = int(self.steps - 1 - np.argmax(within[::-1]))
        return float(self.t[first])


def fly(
    controller: Controller,
    airframe,
    plant: ActuatorPlant,
    target_position: np.ndarray,
    *,
    dt: float = 0.002,
    steps: int = 2500,
    cfg: Mapping[str, Any] | None = None,
    controller_period_steps: int = 1,
) -> ActuatorTrace:
    """Run the closed loop and return everything worth looking at afterwards.

    ``controller_period_steps`` exists because the cascade is not the same kind
    of thing at every rate. Its stage 4 has to outrun the motor's 30 ms lag and
    its stage 1 does not, so a run that evaluates the position loop at 100 Hz
    and the whole cascade at 500 Hz is the honest arrangement and
    ``test_cascade.py`` measures what happens when it is not.
    """
    if steps <= 0 or dt <= 0.0:
        raise ValueError(f"steps and dt must be > 0, got steps={steps}, dt={dt}")
    if controller_period_steps < 1:
        raise ValueError(
            f"controller_period_steps must be >= 1, got {controller_period_steps}"
        )
    target = np.asarray(target_position, dtype=float).reshape(3)
    cfg = {} if cfg is None else cfg
    waypoint = Waypoint(position=target)

    # Every array is one longer than the number of steps: the trace starts with
    # the state the loop was handed, so index 0 is the initial condition rather
    # than the first thing the controller did to it.
    count = steps + 1
    motor_count = plant.motor_command.shape[0]
    t = np.zeros(count)
    position = np.zeros((count, 3))
    velocity = np.zeros((count, 3))
    attitude_quat = np.zeros((count, 4))
    tilt_deg = np.zeros(count)
    heading_deg = np.zeros(count)
    accel_cmd = np.zeros((count, 3))
    thrust_n = np.zeros(count)
    moment_body = np.zeros((count, 3))
    motor_cmd = np.zeros((count, motor_count))
    motor_actual = np.zeros((count, motor_count))
    clamped = np.zeros(count, dtype=int)

    def record(index: int, state: SimState) -> None:
        rotation = quat_to_rotmat(np.asarray(state.attitude_quat, dtype=float))
        t[index] = state.t
        position[index] = state.position
        velocity[index] = state.velocity
        attitude_quat[index] = state.attitude_quat
        tilt_deg[index] = np.degrees(np.arccos(np.clip(rotation[2, 2], -1.0, 1.0)))
        heading_deg[index] = np.degrees(yaw_of(rotation))
        motor_cmd[index] = plant.motor_command
        motor_actual[index] = plant.motor_actual
        clamped[index] = plant.clamped_last_step

    state = zup_state_of(plant)
    record(0, state)
    for step in range(steps):
        if step % controller_period_steps == 0:
            control = controller.compute(state, waypoint, cfg)
            if control.wrench is None:
                raise ValueError(
                    f"{type(controller).__name__} produced a target with no "
                    "wrench. This loop's whole purpose is the actuator path - an "
                    "acceleration demand is not a motor command, and the plant's "
                    "only input is motors. Wrap it in a CascadeController."
                )
            plant.set_motors(motor_commands(airframe, control.wrench, plant))
            accel_cmd[step + 1] = np.asarray(control.accel_cmd, dtype=float).reshape(3)
            thrust_n[step + 1] = float(control.wrench.force_body[2])
            moment_body[step + 1] = np.asarray(control.wrench.moment_body, dtype=float)
        else:
            accel_cmd[step + 1] = accel_cmd[step]
            thrust_n[step + 1] = thrust_n[step]
            moment_body[step + 1] = moment_body[step]
        plant.step(dt)
        state = zup_state_of(plant)
        record(step + 1, state)

    return ActuatorTrace(
        t=t,
        position=position,
        velocity=velocity,
        attitude_quat=attitude_quat,
        tilt_deg=tilt_deg,
        heading_deg=heading_deg,
        accel_cmd=accel_cmd,
        thrust_n=thrust_n,
        moment_body=moment_body,
        motor_cmd=motor_cmd,
        motor_actual=motor_actual,
        clamped=clamped,
        target_position=target,
        steps=count,
    )
