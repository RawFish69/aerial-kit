"""The four-stage cascade, measured against the plant it actually drives.

`controllers/cascade.py` turns an acceleration demand into a wrench and
`dynamics/actuator_loop.py` turns that wrench into motor commands. Three modules
written against each other can agree on a frame that no physical aircraft
shares, and every one of them can be self-consistent while the aircraft flies
backwards - so the load-bearing tests here read the **plant**: its body rates,
its velocity, its achieved wrench. Where a test does restate arithmetic instead
(position/velocity -> attitude -> rate -> moment, all four stages in closed
form) it says so and says why.

Two bugs were found by exactly this file while it was being written, and both
are pinned below as their falsification rather than as a comment:

* **the attitude error was in the world frame.** ``R_d R^T`` instead of
  ``R_d^T R`` - the same rotation, different axes, coinciding only when
  ``R = I``. Every level-attitude test passed. ``test_attitude_error_is_in_the_body_frame``
  asserts the property that fails for every other attitude.
* **the world map was a ``z`` negation.** ``zup_state_of`` applied
  ``(x, y, -z)`` to position and velocity while applying the full NED-to-ENU
  matrix to the attitude. The aircraft's altitude, attitude and heading were all
  correct and it flew away along the diagonal.
  ``test_the_thrust_direction_agrees_with_the_acceleration_it_produces`` flies
  an open-loop demand and reads the velocity the plant actually produced.
"""

from __future__ import annotations

from typing import Any, Mapping

import numpy as np
import pytest

from aerial_kit.airframes.multirotor import MultirotorAirframe
from aerial_kit.controllers import CascadeController, CascadeGains, PIDController
from aerial_kit.controllers.cascade import (
    attitude_error,
    limit_tilt,
    rotation_from_thrust_and_yaw,
    thrust_vector,
)
from aerial_kit.dynamics import (
    ActuatorPlant,
    ActuatorPlantParams,
    fly,
    motor_commands,
    rotmat_to_quat,
    yaw_of,
    zup_state_of,
)
from aerial_kit.dynamics.actuator_loop import NED_TO_ENU
from aerial_kit.dynamics.multirotor_actuator import QUAD_X_SPIN, quad_x_positions
from aerial_kit.dynamics.rotations import quat_to_rotmat
from aerial_kit.interfaces import Controller
from aerial_kit.types import CommandKind, ControlTarget, SimState, Waypoint, Wrench

DT = 0.002  # 500 Hz, the firmware's control rate
G = 9.81
ARM, MASS, YAW_C, TMAX = 0.2, 1.0, 0.02, 6.0
KV_DRAG = 0.1  # the plant's translational drag, N per m/s, world frame


def airframe() -> MultirotorAirframe:
    return MultirotorAirframe(
        arms=4, layout="x", arm_length_m=ARM, mass_kg=MASS, yaw_torque_coeff=YAW_C
    )


def plant(altitude_m: float = 2.0) -> ActuatorPlant:
    """A fresh plant 2 m up, at rest, level, nose north - NED, as it comes."""
    p = ActuatorPlant(
        params=ActuatorPlantParams(
            mass_kg=MASS, max_thrust_per_motor_n=TMAX, yaw_torque_coeff=YAW_C
        ),
        motor_positions=quad_x_positions(ARM),
        spin=QUAD_X_SPIN.copy(),
    )
    p.position[...] = np.array([0.0, 0.0, -altitude_m])
    return p


def gains(**kwargs: Any) -> CascadeGains:
    return CascadeGains(mass_kg=MASS, **kwargs)


class ConstantAccel(Controller):
    """An inner controller that demands one fixed world acceleration.

    The cascade is a wrapper, so the thing under test is easiest to isolate with
    an inner controller that has no dynamics of its own: whatever the aircraft
    does is the cascade's doing.
    """

    def __init__(self, accel: np.ndarray) -> None:
        self.accel = np.asarray(accel, dtype=float).reshape(3)

    def compute(
        self, state: SimState, target_waypoint: Waypoint, cfg: Mapping[str, Any]
    ) -> ControlTarget:
        return ControlTarget(accel_cmd=self.accel.copy())


def level_state(altitude_m: float = 2.0) -> SimState:
    """A z-up state for an aircraft at rest, level, nose north.

    Nose north in z-up ENU is a 90-degree heading, which is the fact
    ``test_a_level_plant_heading_north_reads_ninety_degrees`` records. Built
    here rather than read off a plant so that the unit tests do not depend on
    the mapping they are testing the consumers of.
    """
    return SimState(
        position=np.array([0.0, 0.0, altitude_m]),
        velocity=np.zeros(3),
        attitude_quat=rotmat_to_quat(
            np.array([[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]])
        ),
        body_rates=np.zeros(3),
    )


# ---------------------------------------------------------------------------
# Stage 2: the acceleration demand as a thrust vector
# ---------------------------------------------------------------------------


def test_hover_is_the_weight_on_a_vertical_axis() -> None:
    """``a = 0`` gives exactly ``m*g`` upward. The one input whose answer is
    known before the code is written."""
    thrust_n, axis = thrust_vector(np.zeros(3), gains())
    assert thrust_n == pytest.approx(MASS * G, rel=1e-12)
    assert axis == pytest.approx(np.array([0.0, 0.0, 1.0]), abs=1e-12)


def test_a_sideways_demand_tilts_the_axis_the_way_it_pushes() -> None:
    """The axis leans *against* nothing - it leans the way the aircraft must
    push to accelerate there. +x east is thrust tilted east."""
    thrust_n, axis = thrust_vector(np.array([2.0, 0.0, 0.0]), gains())
    assert axis[0] > 0.0
    assert axis[2] > 0.0
    # The vertical component still carries the weight: the horizontal lean is
    # bought with a larger total, not taken out of the vertical.
    assert thrust_n * axis[2] == pytest.approx(MASS * G, rel=1e-12)


def test_the_floor_is_on_the_vertical_component_so_the_axis_stays_defined() -> None:
    """A demand of ``-g`` or below cannot be answered by taking thrust away,
    and answering it with a horizontal axis would leave the attitude with no
    way to know which way is up."""
    below = np.array([0.0, 0.0, -G * 3.0])
    thrust_n, axis = thrust_vector(below, gains(thrust_floor_frac=0.05))
    assert thrust_n == pytest.approx(MASS * G * 0.05, rel=1e-12)
    assert axis == pytest.approx(np.array([0.0, 0.0, 1.0]), abs=1e-12)


def test_tilt_limit_trades_the_turn_for_altitude() -> None:
    """Clamping the axis back towards vertical makes the vertical component
    *larger*, so an unscaled clamp would climb. The scale holds the vertical
    force at what was asked for and takes the shortfall out of the horizontal."""
    accel = np.array([20.0, 0.0, 0.0])
    thrust_n, axis = thrust_vector(accel, gains())
    clamped, scale = limit_tilt(axis, 35.0)

    assert np.degrees(np.arccos(clamped[2])) == pytest.approx(35.0, abs=1e-9)
    # Same heading of lean, only less of it.
    assert clamped[0] > 0.0
    assert clamped[1] == pytest.approx(0.0, abs=1e-12)
    # The vertical force is unchanged; the horizontal is what was given up.
    assert (thrust_n * scale) * clamped[2] == pytest.approx(thrust_n * axis[2], rel=1e-12)
    assert (thrust_n * scale) * clamped[0] < thrust_n * axis[0]


def test_within_the_limit_nothing_is_touched() -> None:
    _, axis = thrust_vector(np.array([1.0, 0.0, 0.0]), gains())
    clamped, scale = limit_tilt(axis, 35.0)
    assert scale == 1.0
    assert clamped == pytest.approx(axis, abs=1e-15)


# ---------------------------------------------------------------------------
# Stage 3: the attitude error, in the aircraft's own axes
# ---------------------------------------------------------------------------


def test_attitude_error_is_in_the_body_frame() -> None:
    """The property that ``R_d R^T`` fails for every attitude but the identity.

    ``e_R`` is defined by the *body*-frame error, so for ``R = R_d Rx(eps)`` it
    is ``+eps x_hat`` **whatever ``R_d`` is**. The world-frame product
    ``R_d Rx(eps) R_d^T`` has the same angle and the axis ``R_d x_hat``, so it
    answers ``eps x_hat`` only when ``R_d`` fixes x - which is exactly what a
    level-attitude test sets up, and exactly why the bug survived one.
    """
    eps = 0.01
    roll = np.array(
        [
            [1.0, 0.0, 0.0],
            [0.0, np.cos(eps), -np.sin(eps)],
            [0.0, np.sin(eps), np.cos(eps)],
        ]
    )
    # A rotated heading and a tilt, so R_d moves x_hat a long way.
    desired = rotation_from_thrust_and_yaw(
        np.array([0.3, -0.2, 0.9]) / np.linalg.norm(np.array([0.3, -0.2, 0.9])),
        np.radians(37.0),
    )
    assert not np.allclose(desired @ np.array([1.0, 0.0, 0.0]), np.array([1.0, 0.0, 0.0]))

    # Exactly sin(eps), not eps: the skew part of a rotation by eps is
    # sin(eps)*eps_hat, and the small-angle form the module docstring quotes is
    # the first-order statement of the same thing.
    error = attitude_error(desired, desired @ roll)
    assert error == pytest.approx(np.array([np.sin(eps), 0.0, 0.0]), abs=1e-15)

    # The falsification: the world-frame product, which is what the first
    # version computed, gives the same magnitude on a different axis.
    world_frame = desired @ roll @ desired.T
    skew = world_frame - world_frame.T
    world_vee = 0.5 * np.array([skew[2, 1], skew[0, 2], skew[1, 0]])
    assert np.linalg.norm(world_vee) == pytest.approx(np.linalg.norm(error), rel=1e-9)
    assert not np.allclose(world_vee, error, atol=1e-6)


def test_the_rate_demand_sign_is_the_one_that_corrects() -> None:
    """``R = R_d Rx(eps)`` is the aircraft rolled ``+eps`` right of the demand,
    so the rate demand must be ``-k*eps`` about x: left. Getting this backwards
    is not a subtle instability, it is a fly-away at a rate set by ``k_att``."""
    eps = 0.02
    desired = rotation_from_thrust_and_yaw(np.array([0.0, 0.0, 1.0]), np.radians(90.0))
    roll = np.array(
        [
            [1.0, 0.0, 0.0],
            [0.0, np.cos(eps), -np.sin(eps)],
            [0.0, np.sin(eps), np.cos(eps)],
        ]
    )
    error = attitude_error(desired, desired @ roll)
    assert (-4.0 * error)[0] == pytest.approx(-4.0 * np.sin(eps), rel=1e-12)
    assert (-4.0 * error)[0] < 0.0  # left, which is the way back to the demand


def test_the_desired_attitude_is_orthonormal_and_carries_the_heading() -> None:
    """A vertical thrust axis leaves the heading free, so it is carried exactly.
    A tilted one spends a degree of freedom on the tilt, and the heading is what
    gets re-orthogonalised - measured here rather than asserted loosely."""
    level = rotation_from_thrust_and_yaw(np.array([0.0, 0.0, 1.0]), np.radians(20.0))
    assert level @ level.T == pytest.approx(np.eye(3), abs=1e-12)
    assert np.linalg.det(level) == pytest.approx(1.0, rel=1e-12)
    assert yaw_of(level) == pytest.approx(np.radians(20.0), abs=1e-12)

    axis = np.array([0.3, -0.2, 0.9]) / np.linalg.norm(np.array([0.3, -0.2, 0.9]))
    tilt_deg = np.degrees(np.arccos(axis[2]))
    tilted = rotation_from_thrust_and_yaw(axis, np.radians(20.0))
    assert tilted @ tilted.T == pytest.approx(np.eye(3), abs=1e-12)
    assert np.linalg.det(tilted) == pytest.approx(1.0, rel=1e-12)
    assert tilted[:, 2] == pytest.approx(axis, abs=1e-12)
    assert tilted[:, 0] @ tilted[:, 2] == pytest.approx(0.0, abs=1e-12)

    # How far the carried heading can drift, over random tilted axes: bounded
    # by the tilt itself, which is the whole of what the axis-winning costs.
    rng = np.random.default_rng(20260919)
    worst = 0.0
    for _ in range(200):
        raw = rng.normal(size=3)
        raw[2] = abs(raw[2]) + 0.3  # keep it a thrust axis, not a floor
        raw /= np.linalg.norm(raw)
        yaw = float(rng.uniform(-np.pi, np.pi))
        built = rotation_from_thrust_and_yaw(raw, yaw)
        offset = abs(np.arctan2(np.sin(yaw_of(built) - yaw), np.cos(yaw_of(built) - yaw)))
        assert offset < np.arccos(raw[2])
        worst = max(worst, float(np.degrees(offset)))
    assert worst > 1.0  # and it is a real drift, not always zero


# ---------------------------------------------------------------------------
# Stage 4: the rate error as a moment, with no gyroscopic term
# ---------------------------------------------------------------------------


def test_stage_four_is_i_times_k_rate_times_the_rate_error() -> None:
    """An independent restatement of the one line, on a state with a rate on
    every axis so that a transposed or reordered gain vector shows up."""
    inertia = (0.01, 0.02, 0.03)
    controller = CascadeController(
        inner=ConstantAccel([0.0, 0.0, 0.0]),
        gains=CascadeGains(mass_kg=MASS, inertia_kgm2=inertia, k_att=4.0, k_rate=12.0),
    )
    state = level_state()
    state.body_rates = np.array([0.3, -0.2, 0.1])
    controller.compute(state, Waypoint(position=state.position.copy()), {})

    stages = controller.last_stages
    expected = np.asarray(inertia) * 12.0 * (stages["omega_des_rps"] - state.body_rates)
    assert stages["moment_body"] == pytest.approx(expected, rel=1e-12)
    assert controller.compute(
        state, Waypoint(position=state.position.copy()), {}
    ).wrench.moment_body == pytest.approx(expected, rel=1e-12)


def test_the_gyroscopic_term_is_absent() -> None:
    """The plant's own ``_derivatives`` already subtracts ``omega x (I omega)``,
    so a controller that adds it too applies it twice. Many flight controllers
    include it, correctly, because their plant does not - which is why the
    absence is asserted rather than left to a reader to notice.

    The falsification is the term itself: it is large enough at these rates to
    be unmistakable, so "absent" is a measurable claim.
    """
    inertia = np.array([0.01, 0.02, 0.03])
    controller = CascadeController(
        inner=ConstantAccel([0.0, 0.0, 0.0]),
        gains=CascadeGains(mass_kg=MASS, inertia_kgm2=tuple(inertia)),
    )
    state = level_state()
    state.body_rates = np.array([1.5, -1.1, 0.7])
    moment = controller.compute(
        state, Waypoint(position=state.position.copy()), {}
    ).wrench.moment_body

    gyroscopic = np.cross(state.body_rates, inertia * state.body_rates)
    assert np.linalg.norm(gyroscopic) > 1e-2  # not a rounding-scale term
    assert not np.allclose(moment, moment + gyroscopic, atol=1e-6)
    # And the moment is exactly the no-gyroscopic formula, not merely near it.
    assert moment == pytest.approx(
        inertia * 12.0 * (controller.last_stages["omega_des_rps"] - state.body_rates),
        rel=1e-12,
    )


# ---------------------------------------------------------------------------
# The frame map: what the loop closes on, and the 90 degrees it reads
# ---------------------------------------------------------------------------


def test_a_level_plant_heading_north_reads_ninety_degrees() -> None:
    """NED north is ENU ``y``, and the heading is read off the body x axis. The
    aircraft is not rotated by 90 degrees; the axes it is measured against are.
    Recorded here so the next reader does not have to derive it."""
    state = zup_state_of(plant())
    rotation = quat_to_rotmat(state.attitude_quat)
    assert np.degrees(yaw_of(rotation)) == pytest.approx(90.0, abs=1e-9)
    # Level: the up axis is up, and no tilt leaked into the round trip.
    assert rotation[:, 2] == pytest.approx(np.array([0.0, 0.0, 1.0]), abs=1e-12)


def test_position_and_velocity_go_through_the_full_world_map() -> None:
    """``(x, y, -z)`` is not NED-to-ENU, it is ENU-with-the-axes-swapped.

    North is the NED ``x`` and the ENU ``y``. A map that only negates ``z``
    sends a north displacement to the east - and leaves the *attitude* right,
    which is what makes it survive every test that looks at one of them.
    """
    p = plant()
    p.position[...] = np.array([1.0, 2.0, -3.0])  # 1 m north, 2 m east, 3 m up
    p.velocity[...] = np.array([4.0, 5.0, 6.0])

    state = zup_state_of(p)
    # Spelled out, because this is the assertion that matters: north -> y.
    assert state.position == pytest.approx(np.array([2.0, 1.0, 3.0]), abs=1e-12)
    assert state.velocity == pytest.approx(np.array([5.0, 4.0, -6.0]), abs=1e-12)


def test_the_thrust_direction_agrees_with_the_acceleration_it_produces() -> None:
    """The closed loop, read from the plant's own velocity.

    This is the assertion the frame bug failed. A constant +x demand, no
    position feedback to confuse the picture: the aircraft holds its tilt, keeps
    its nose north, and builds speed east - and the plant's velocity, not the
    controller's idea of it, is what says so.

    The plant has translational drag (``kv_drag``, 0.1 N per m/s), so the
    achieved acceleration is the demand **minus** the drag, and a constant
    demand reaches a terminal velocity rather than running away. Expecting the
    naive ``demand * duration`` was the first version of this test and it was
    wrong by a third - the drag is quoted here and integrated, so the assertion
    is exact instead of approximate, and it checks the mass and the drag
    coefficient along with the frame.
    """
    demand = np.array([1.0, 0.0, 0.0])
    controller = CascadeController(inner=ConstantAccel(demand), gains=gains())
    trace = fly(controller, airframe(), plant(), np.zeros(3), steps=3000, dt=DT)

    # The motors start stopped, so the first fraction of a second is the
    # attitude loop catching a falling aircraft. Measure once it has.
    t0, t1 = 1000, 3000
    assert trace.t[t0] == pytest.approx(2.0, abs=1e-9)
    delta_v = trace.velocity[t1] - trace.velocity[t0]
    duration = trace.t[t1] - trace.t[t0]
    window = slice(t0, t1 + 1)

    # Gravity is already inside the demand's arithmetic (stage 2 builds the
    # thrust that produces the demanded acceleration *net* of gravity), so all
    # three axes have the same force balance: demand minus drag.
    for axis in range(3):
        expected = demand[axis] * duration - (KV_DRAG / MASS) * np.trapz(
            trace.velocity[window, axis], trace.t[window]
        )
        assert delta_v[axis] == pytest.approx(expected, abs=1e-6)

    # And the direction, which is the frame claim and does not depend on drag.
    assert delta_v[0] > 2.0  # east, the direction asked for
    assert abs(delta_v[1]) < 1e-9  # and not one metre per second north
    assert np.abs(trace.velocity[t0:t1, 1]).max() < 1e-9
    assert trace.heading_deg[t1] == pytest.approx(90.0, abs=1e-6)
    # The tilt is the one the demand asks for, and it is held, not hunted.
    expected_tilt = np.degrees(np.arctan2(demand[0], G))
    assert trace.tilt_deg[t1] == pytest.approx(expected_tilt, abs=0.01)
    assert np.abs(trace.tilt_deg[t0:t1] - expected_tilt).max() < 0.01


# ---------------------------------------------------------------------------
# The wrench reaching the plant
# ---------------------------------------------------------------------------


def test_hover_motor_commands_match_the_arithmetic_derived_here() -> None:
    """The hover command in the plant's units, from the airframe's own mass:
    ``m*g / 4 / max_thrust``, which on this aircraft is 0.40875 and not 2.4525."""
    hover = Wrench(force_body=np.array([0.0, 0.0, MASS * G]), moment_body=np.zeros(3))
    fraction = MASS * G / 4.0 / TMAX
    commands = motor_commands(airframe(), hover, plant())
    assert commands == pytest.approx(np.full(4, fraction), rel=1e-12)
    assert fraction == pytest.approx(0.40875, abs=1e-12)


def test_a_lateral_demand_rolls_the_aircraft_the_way_the_demand_points() -> None:
    """Sign checked on the plant's body rates, not on a matrix.

    Nose north, demand east. East is 90 degrees to the right of the nose, so a
    pure +x demand is a pure right roll - positive about body x, with pitch and
    yaw left alone. Read off ``plant.body_rates``, which is the aircraft moving.
    """
    p = plant()
    controller = CascadeController(
        inner=ConstantAccel([1.0, 0.0, 0.0]), gains=gains()
    )
    wrench = controller.compute(
        level_state(), Waypoint(position=np.array([0.0, 0.0, 2.0])), {}
    ).wrench
    # The moment the controller asks for, in the z-up frame, is a right roll.
    assert wrench.moment_body[0] > 0.0

    p.set_motors(motor_commands(airframe(), wrench, p))
    for _ in range(100):  # 0.2 s
        p.step(DT)
    assert p.body_rates[0] > 0.5  # rolling right, in the plant's FRD frame
    assert abs(p.body_rates[1]) < 0.05 * p.body_rates[0]
    assert abs(p.body_rates[2]) < 0.05 * p.body_rates[0]


def test_the_body_axis_it_leans_about_is_the_one_the_error_named() -> None:
    """The lateral step's error is a roll, and the aircraft is heard to roll.

    This is the measurement that found the world-frame error: at heading 90 a
    demand of +x is a roll about **body x**, and the buggy version named body
    y - a pitch - which is a different physical motion, not a sign flip.
    """
    controller = CascadeController(inner=ConstantAccel([1.0, 0.0, 0.0]), gains=gains())
    controller.compute(level_state(), Waypoint(position=np.zeros(3)), {})
    error = controller.last_stages["attitude_error_rad"]

    assert error[0] < -0.05  # a roll error about body x
    assert abs(error[1]) < 1e-9  # nothing about body y
    assert abs(error[2]) < 1e-9
    assert controller.last_stages["moment_body"][0] > 0.0  # correcting right


def test_motor_actual_stays_in_range_and_an_infeasible_demand_stays_visible() -> None:
    """The whole reason the plant keeps ``motor_command`` and ``motor_actual``
    apart: a demand the motors cannot meet has to look like one."""
    p = plant()
    af = airframe()
    # 60 m/s^2 up on a 1 kg aircraft: ~70 N wanted, 24 N available.
    infeasible = Wrench(
        force_body=np.array([0.0, 0.0, MASS * (60.0 + G)]), moment_body=np.zeros(3)
    )

    commands = motor_commands(af, infeasible, p)
    assert commands.max() > 1.0  # the seam does not clip, on purpose
    p.set_motors(commands)
    p.step(DT)

    assert p.clamped_last_step == 4
    assert np.all(p.motor_actual <= 1.0) and np.all(p.motor_actual >= 0.0)
    commanded = p.commanded_wrench()
    achieved = p.achieved_wrench()
    assert np.linalg.norm(commanded.force_body - achieved.force_body) > 10.0


def test_a_plant_with_the_other_spin_is_refused() -> None:
    """The sibling's mixer carries the sibling's spin signs. A plant with a
    different table is a different aircraft, and driving it through this seam
    gives a yaw response of the wrong sign or magnitude."""
    p = plant()
    p.spin = -np.asarray(p.spin, dtype=float)
    hover = Wrench(force_body=np.array([0.0, 0.0, MASS * G]), moment_body=np.zeros(3))
    with pytest.raises(ValueError, match="spin"):
        motor_commands(airframe(), hover, p)


def test_rotmat_to_quat_round_trips_every_rotation_it_is_given() -> None:
    """The attitude crosses the frame map as a quaternion and comes back as a
    matrix, so the round trip is load-bearing rather than a convenience.

    Compared by **dot product, not elementwise**: ``rotmat_to_quat`` does not
    canonicalise the sign, and a quaternion and its negative are the same
    rotation. Anything comparing two of these elementwise is comparing an
    arbitrary choice of sign.
    """
    rng = np.random.default_rng(20260919)
    for _ in range(200):
        quat = rng.normal(size=4)
        quat /= np.linalg.norm(quat)
        rotation = quat_to_rotmat(quat)
        recovered = rotmat_to_quat(rotation)
        assert abs(float(recovered @ quat)) == pytest.approx(1.0, abs=1e-9)
        assert quat_to_rotmat(recovered) == pytest.approx(rotation, abs=1e-9)
        # And the frame map in between is a proper rotation, so it round trips.
        mapped = NED_TO_ENU @ rotation @ np.diag([1.0, -1.0, -1.0])
        assert np.linalg.det(mapped) == pytest.approx(1.0, rel=1e-9)
        assert quat_to_rotmat(rotmat_to_quat(mapped)) == pytest.approx(mapped, abs=1e-9)


# ---------------------------------------------------------------------------
# The loop, end to end
# ---------------------------------------------------------------------------


def test_a_waypoint_is_reached_with_the_bank_inside_the_limit() -> None:
    """Done in the sense the ledger asked for: not position error alone, but
    position error *and* the attitude the aircraft used to get there, with the
    motors inside their bounds the whole way."""
    target = np.array([3.0, 0.0, 2.0])
    controller = CascadeController(inner=PIDController(), gains=gains(max_tilt_deg=35.0))
    trace = fly(controller, airframe(), plant(), target, steps=8000, dt=DT)

    assert trace.position_error[-1] < 0.01
    # It starts three metres out and dips a few microns below that while the
    # motors, which start stopped, catch the aircraft - so the maximum is just
    # *above* the starting error, not equal to it.
    assert trace.position_error[0] == pytest.approx(3.0, abs=1e-12)
    assert 3.0 <= trace.position_error.max() < 3.001
    assert trace.tilt_deg.max() < 35.0  # never hit the limit, so never fought it
    assert trace.tilt_deg.max() > 1.0  # and it did have to tilt
    assert np.all(trace.motor_actual >= 0.0) and np.all(trace.motor_actual <= 1.0)
    assert trace.clamped.sum() == 0
    assert np.abs(trace.velocity[-1]).max() < 0.02

    # A pure +x demand from a north-facing aircraft is a pure roll, so the
    # aircraft never leaves the x axis - a symmetry the plant has no reason to
    # respect unless the loop is actually flying it straight.
    assert np.abs(trace.position[:, 1]).max() < 1e-9
    assert trace.heading_deg[-1] == pytest.approx(90.0, abs=1e-6)
    # The recorded entry point: the loop starts with its motors stopped, so the
    # first fraction of a second is free fall and the "settled" time is late.
    assert trace.settled_after(0.05) is not None
    assert trace.settled_after(0.05) > 4.0


def test_a_three_axis_step_converges_and_the_yaw_is_only_damped() -> None:
    """``hold_yaw_deg=None`` means the desired heading is re-read as the current
    heading every step, so a yaw disturbance is damped and never corrected.
    That is a documented limitation, so it is measured rather than hidden: the
    position converges and the heading ends *near* where it started."""
    target = np.array([3.0, 3.0, 2.0])
    controller = CascadeController(inner=PIDController(), gains=gains())
    trace = fly(controller, airframe(), plant(), target, steps=8000, dt=DT)

    assert trace.position_error[-1] < 0.01
    assert trace.tilt_deg.max() < 35.0
    # Damped, not regulated: it moves, and it does not run away.
    assert 90.0 <= trace.heading_deg[-1] < 100.0


def test_a_cascade_rate_far_below_the_motors_is_where_it_stops_flying() -> None:
    """``controller_period_steps`` exists because the cascade is not the same
    kind of thing at every rate, and the rate at which that stops being true is
    a measured number rather than an assertion about "outrunning the lag".

    The knob turns out to be far more forgiving than the first version of this
    test claimed - position converges at every rate down to 5 Hz, and what a
    slower cascade costs is **attitude margin**, not accuracy: the peak tilt
    grows from 15.9 to 27.2 degrees as the rate falls from 500 Hz to 5 Hz. The
    cliff is below that. At 3.3 Hz the rate loop is slower than the attitude
    loop it is supposed to be closing, the sign of the correction no longer
    arrives in time, and the aircraft inverts and lands on its back.

    Both halves are asserted, because the pass is only interesting next to the
    failure - and the failure is the one a reader would otherwise assume starts
    much earlier.
    """
    target = np.array([3.0, 3.0, 2.0])

    def run(period: int):
        controller = CascadeController(inner=PIDController(), gains=gains())
        return fly(
            controller, airframe(), plant(), target, steps=8000, dt=DT,
            controller_period_steps=period,
        )

    slow_but_flying = run(100)  # 5 Hz
    assert slow_but_flying.position_error[-1] < 0.01
    assert slow_but_flying.tilt_deg.max() < 35.0
    assert slow_but_flying.clamped.sum() == 0

    inverted = run(150)  # 3.3 Hz
    assert inverted.tilt_deg.max() > 90.0  # past horizontal and over
    assert inverted.position_error[-1] > 10.0
    assert inverted.clamped.sum() > 1000  # the motors pinned trying to stop it
    assert inverted.position[-1][2] == pytest.approx(0.0, abs=0.01)  # on the ground


# ---------------------------------------------------------------------------
# What the cascade refuses
# ---------------------------------------------------------------------------


def test_the_cascade_presents_as_its_inner_controllers_command_kind() -> None:
    """``runner.py`` refuses a controller whose ``command_kind`` is not the
    airframe's, and a multirotor airframe is ACCEL. The wrapper must therefore
    inherit rather than declare."""
    assert CascadeController(inner=PIDController()).command_kind is CommandKind.ACCEL


def test_the_cascade_refuses_a_state_it_cannot_close_a_loop_on() -> None:
    """A state with no attitude is one only an acceleration-level controller can
    be flown on, and the inner controller would have accepted it happily. That
    difference between the two contracts is why this raises."""
    controller = CascadeController(inner=ConstantAccel(np.zeros(3)), gains=gains())
    flat = SimState(position=np.zeros(3), velocity=np.zeros(3))
    with pytest.raises(ValueError, match="attitude"):
        controller.compute(flat, Waypoint(position=np.zeros(3)), {})

    no_rates = SimState(
        position=np.zeros(3), velocity=np.zeros(3), attitude_quat=level_state().attitude_quat
    )
    with pytest.raises(ValueError, match="body rates"):
        controller.compute(no_rates, Waypoint(position=np.zeros(3)), {})


def test_the_cascade_refuses_to_invent_the_mass() -> None:
    """Stage 2 builds the hover thrust from the mass, so a guessed mass is an
    aircraft that does not hold altitude rather than a loop that flies badly."""
    with pytest.raises(ValueError, match="mass_kg"):
        CascadeGains.from_config({"controller": {"cascade": {}}})
    assert CascadeGains.from_config(
        {"controller": {"cascade": {"mass_kg": 1.3, "k_att": 5.0}}}
    ) == CascadeGains(mass_kg=1.3, k_att=5.0)
    with pytest.raises(ValueError, match="unknown cascade setting"):
        CascadeGains.from_config({"controller": {"cascade": {"mass_kg": 1.0, "kp": 3.0}}})


def test_the_gains_refuse_values_that_are_not_a_controller() -> None:
    for bad in (
        {"mass_kg": 0.0},
        {"mass_kg": -1.0},
        {"mass_kg": MASS, "inertia_kgm2": (0.01, 0.0, 0.02)},
        {"mass_kg": MASS, "k_att": 0.0},
        {"mass_kg": MASS, "k_rate": -3.0},
        {"mass_kg": MASS, "max_tilt_deg": 90.0},
        {"mass_kg": MASS, "thrust_floor_frac": 1.0},
    ):
        with pytest.raises(ValueError):
            CascadeGains(**bad)


def test_fly_refuses_a_controller_that_produces_no_wrench() -> None:
    """An acceleration demand is not a motor command, and the plant's only input
    is motors. The refusal names the fix rather than the symptom."""
    with pytest.raises(ValueError, match="CascadeController"):
        fly(PIDController(), airframe(), plant(), np.zeros(3), steps=10, dt=DT)


def test_fly_refuses_a_run_that_is_not_a_run() -> None:
    controller = CascadeController(inner=PIDController(), gains=gains())
    for kwargs in ({"steps": 0}, {"dt": 0.0}, {"controller_period_steps": 0}):
        with pytest.raises(ValueError):
            fly(controller, airframe(), plant(), np.zeros(3), **kwargs)


def test_the_trace_reports_settling_as_the_time_it_never_leaves_the_band_again() -> None:
    """``settled_after`` answers ``None`` rather than a large number when there
    is no such time, because a large number reads like a slow success."""
    controller = CascadeController(inner=PIDController(), gains=gains())
    trace = fly(controller, airframe(), plant(), np.array([3.0, 0.0, 2.0]), steps=8000, dt=DT)

    settled = trace.settled_after(0.05)
    assert settled is not None
    later = trace.t >= settled
    assert np.all(trace.position_error[later] <= 0.05)
    # A tolerance it never meets at the end is not a settling time.
    assert trace.settled_after(1e-9) is None
