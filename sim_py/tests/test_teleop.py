"""Tests for real-time quadrotor teleoperation.

Everything here runs headless: the input mapping, the fixed-step scheduler and
the physics need no GUI, and the renderer is exercised under the Agg backend.
"""

from __future__ import annotations

from dataclasses import replace

import numpy as np
import pytest

import matplotlib

matplotlib.use("Agg")

from aerial_kit.controllers.fixed_wing import body_axis_pitch_bank
from aerial_kit.types import ControlTarget, SimState
from sim_py.backends.multirotor_backend import YAW_RATE_METADATA_KEY, MultirotorBackend
from sim_py.core.registry import create_backend, register_builtin_components
from sim_py.teleop import input_state as ks
from sim_py.teleop.camera import FOLLOW, WORLD, CameraController
from sim_py.teleop.commands import (
    FixedWingTeleopTuning,
    InputAxes,
    TeleopTuning,
    axes_from_keys,
    command_from_keys,
    fixedwing_actuator_from_axes,
    fixedwing_command_from_keys,
    neutral_command,
    neutral_fixedwing_command,
)
from sim_py.teleop.engine import TeleopEngine
from sim_py.teleop.loop import FixedStepScheduler, resolve_interval_ms
from sim_py.teleop.model import MOTOR_IS_FRONT, QuadGeometry, WingGeometry, quat_to_euler_rpy
from sim_py.teleop.session import ensure_interactive_backend, run_teleop_cli
from sim_py.teleop.world import TeleopWorld

TUNING = TeleopTuning()
FW_TUNING = FixedWingTeleopTuning()
START = np.array([50.0, 50.0, 20.0])
#: Cruise velocity along +x; a fixed wing cannot reset() at rest without
#: stalling on the very first step (zero airspeed -> zero lift).
FW_CRUISE_VELOCITY = np.array([18.0, 0.0, 0.0])


def _world() -> TeleopWorld:
    return TeleopWorld(
        terrain_type="empty",
        space_dim=np.array([120.0, 120.0, 60.0]),
        max_z_allowed=60.0,
        start_position=START.copy(),
        terrain_clearance=0.0,
    )


def _engine(dt: float = 0.01) -> TeleopEngine:
    register_builtin_components()
    world = _world()
    backend = create_backend("multirotor")
    backend.reset(
        initial_state=SimState(position=START.copy(), velocity=np.zeros(3), t=0.0),
        world={},
        cfg={"simulation": {"multirotor": {"mass": 1.2, "kv_drag": 0.12}}},
    )
    return TeleopEngine(backend=backend, world=world, tuning=TUNING, dt=dt)


def _fixedwing_engine(dt: float = 0.02) -> TeleopEngine:
    register_builtin_components()
    world = _world()
    backend = create_backend("fixedwing")
    backend.reset(
        initial_state=SimState(
            position=START.copy(), velocity=FW_CRUISE_VELOCITY.copy(), t=0.0
        ),
        world={},
        cfg={"simulation": {"fixedwing": {}}},
    )
    return TeleopEngine(backend=backend, world=world, tuning=FW_TUNING, dt=dt)


def _command(*keys: str, yaw: float = 0.0, velocity: np.ndarray | None = None):
    keyboard = ks.KeyboardState()
    for key in keys:
        keyboard.press(key)
    return keyboard, command_from_keys(
        keyboard,
        yaw=yaw,
        velocity=np.zeros(3) if velocity is None else velocity,
        tuning=TUNING,
    )


# --------------------------------------------------------------------------
# Input-to-command mapping
# --------------------------------------------------------------------------


@pytest.mark.parametrize("key", ["w", "up"])
def test_forward_keys_produce_forward_command(key: str) -> None:
    _, command = _command(key)

    assert command.axes.forward == pytest.approx(1.0)
    # yaw=0 means body forward is world +X.
    assert command.accel_cmd[0] > 0.0
    assert command.accel_cmd[1] == pytest.approx(0.0)
    assert command.accel_cmd[2] == pytest.approx(0.0)


@pytest.mark.parametrize("key", ["s", "down"])
def test_backward_keys_produce_backward_command(key: str) -> None:
    _, command = _command(key)

    assert command.axes.forward == pytest.approx(-1.0)
    assert command.accel_cmd[0] < 0.0


@pytest.mark.parametrize(
    ("key", "expected_y_sign"),
    [("d", -1.0), ("right", -1.0), ("a", 1.0), ("left", 1.0)],
)
def test_strafe_keys_command_sideways_acceleration(key: str, expected_y_sign: float) -> None:
    # At yaw=0 the vehicle faces world +X, so its right-hand side is world -Y.
    _, command = _command(key)

    assert np.sign(command.accel_cmd[1]) == expected_y_sign
    assert command.accel_cmd[0] == pytest.approx(0.0)


def test_space_produces_climb_and_shift_produces_descent() -> None:
    _, climb = _command("space")
    _, descend = _command("shift")

    assert climb.axes.up == pytest.approx(1.0)
    assert climb.accel_cmd[2] > 0.0
    assert descend.axes.up == pytest.approx(-1.0)
    assert descend.accel_cmd[2] < 0.0
    assert climb.accel_cmd[2] == pytest.approx(-descend.accel_cmd[2])


def test_literal_space_character_is_treated_as_the_space_key() -> None:
    _, command = _command(" ")

    assert command.axes.up == pytest.approx(1.0)


def test_q_and_e_produce_opposite_yaw_rate_commands() -> None:
    _, left = _command("q")
    _, right = _command("e")

    assert left.yaw_rate_cmd > 0.0
    assert right.yaw_rate_cmd < 0.0
    assert left.yaw_rate_cmd == pytest.approx(-right.yaw_rate_cmd)


def test_opposing_keys_cancel() -> None:
    _, command = _command("w", "s", "q", "e")

    assert command.axes.forward == pytest.approx(0.0)
    assert command.yaw_rate_cmd == pytest.approx(0.0)


def test_forward_command_follows_actual_heading() -> None:
    _, command = _command("w", yaw=np.pi / 2.0)

    # Facing world +Y, so a forward demand must accelerate along +Y.
    assert command.accel_cmd[1] > 0.0
    assert command.accel_cmd[0] == pytest.approx(0.0, abs=1e-9)


def test_releasing_keys_returns_command_toward_neutral() -> None:
    keyboard, held = _command("w")
    assert held.accel_cmd[0] > 0.0

    keyboard.release("w")
    released = command_from_keys(
        keyboard, yaw=0.0, velocity=np.zeros(3), tuning=TUNING
    )

    assert keyboard.active_actions() == ()
    assert released.axes.is_neutral
    assert np.allclose(released.accel_cmd, np.zeros(3))


def test_released_keys_damp_existing_velocity() -> None:
    keyboard = ks.KeyboardState()
    command = command_from_keys(
        keyboard, yaw=0.0, velocity=np.array([5.0, 0.0, 0.0]), tuning=TUNING
    )

    # With no pilot demand the only acceleration is opposing the motion.
    assert command.accel_cmd[0] < 0.0


def test_x_clears_held_commands_immediately() -> None:
    keyboard, _ = _command("w", "space", "q")
    assert keyboard.held_keys

    keyboard.press("x")

    assert keyboard.held_keys == set()
    assert keyboard.neutralize_requests == 1
    command = command_from_keys(keyboard, yaw=0.0, velocity=np.zeros(3), tuning=TUNING)
    assert command.axes.is_neutral
    assert np.allclose(command.accel_cmd, np.zeros(3))


def test_focus_loss_clears_held_inputs() -> None:
    keyboard, _ = _command("w", "space")

    keyboard.set_focused(False)

    assert keyboard.held_keys == set()
    assert keyboard.focus_losses == 1
    assert keyboard.focused is False


def test_focus_loss_neutralizes_engine_command_even_with_keys_held() -> None:
    engine = _engine()
    engine.keyboard.press("w")
    assert engine.current_command().accel_cmd[0] > 0.0

    engine.keyboard.set_focused(False)
    engine.keyboard.press("w")  # a stray press while unfocused must not fly

    assert np.allclose(engine.current_command().accel_cmd, np.zeros(3))


def test_pause_holds_commands_neutral() -> None:
    engine = _engine()
    engine.keyboard.press("w")
    engine.keyboard.press("p")

    assert engine.keyboard.paused is True
    assert np.allclose(engine.current_command().accel_cmd, np.zeros(3))

    engine.keyboard.press("p")
    assert engine.keyboard.paused is False
    assert engine.current_command().accel_cmd[0] > 0.0


def test_escape_stops_the_session() -> None:
    keyboard = ks.KeyboardState()
    keyboard.press("escape")

    assert keyboard.running is False


def test_shift_modifier_combination_keeps_both_keys_latched() -> None:
    keyboard = ks.KeyboardState()
    keyboard.press("shift")
    keyboard.press("shift+up")

    assert keyboard.is_active(ks.FORWARD)
    assert keyboard.is_active(ks.DESCEND)

    # Releasing the arrow arrives as the combination; shift is still down.
    keyboard.release("shift+up")

    assert not keyboard.is_active(ks.FORWARD)
    assert keyboard.is_active(ks.DESCEND)

    keyboard.release("shift")
    assert keyboard.held_keys == set()


def test_uppercase_key_implies_shift_is_held() -> None:
    keyboard = ks.KeyboardState()
    keyboard.press("W")

    assert keyboard.is_active(ks.FORWARD)
    assert keyboard.is_active(ks.DESCEND)


def test_both_wasd_and_arrows_are_bound_to_the_same_actions() -> None:
    for letter, arrow in (("w", "up"), ("s", "down"), ("a", "left"), ("d", "right")):
        assert ks.HOLD_BINDINGS[letter] == ks.HOLD_BINDINGS[arrow]


# --------------------------------------------------------------------------
# Fixed-step real-time loop
# --------------------------------------------------------------------------


def test_scheduler_converts_wall_clock_into_fixed_steps() -> None:
    now = [100.0]
    scheduler = FixedStepScheduler(dt=0.01, clock=lambda: now[0])
    scheduler.start()

    now[0] += 0.033
    assert scheduler.tick() == 3
    now[0] += 0.033
    # The 0.003 s remainder accumulates instead of being discarded.
    assert scheduler.tick() == 3
    now[0] += 0.034
    assert scheduler.tick() == 4
    assert scheduler.steps == 10
    assert scheduler.frames == 3


def test_scheduler_bounds_catch_up_after_a_stall() -> None:
    now = [0.0]
    scheduler = FixedStepScheduler(dt=0.01, max_catch_up_s=0.25, clock=lambda: now[0])
    scheduler.start()

    now[0] += 30.0
    steps = scheduler.tick()

    assert steps == scheduler.max_steps_per_tick == 25
    assert scheduler.dropped_steps > 0


def test_scheduler_reports_frame_rate_and_real_time_factor() -> None:
    now = [0.0]
    scheduler = FixedStepScheduler(dt=0.01, clock=lambda: now[0])
    scheduler.start()
    for _ in range(20):
        now[0] += 1.0 / 30.0
        scheduler.tick()

    assert scheduler.frame_rate == pytest.approx(30.0, rel=0.05)
    assert scheduler.real_time_factor == pytest.approx(1.0, rel=0.1)
    assert scheduler.frames == 20


def test_resolve_interval_ms_is_at_least_one() -> None:
    assert resolve_interval_ms(30.0) == 33
    assert resolve_interval_ms(100000.0) == 1


def test_paused_scheduler_does_not_accumulate_catch_up() -> None:
    now = [0.0]
    scheduler = FixedStepScheduler(dt=0.01, clock=lambda: now[0])
    scheduler.start()
    now[0] += 5.0
    scheduler.pause_drift()
    now[0] += 0.02

    assert scheduler.tick() == 2


# --------------------------------------------------------------------------
# Physics: commands must actually move and rotate the vehicle
# --------------------------------------------------------------------------


def test_injected_forward_command_changes_simulated_position() -> None:
    engine = _engine()
    start = engine.state.position.copy()

    engine.keyboard.press("w")
    state = engine.advance(200)  # two seconds

    travelled = float(state.position[0] - start[0])
    assert travelled > 3.0, f"forward travel was only {travelled:.2f} m"
    assert state.velocity[0] > 2.0
    assert abs(state.position[1] - start[1]) < 0.5


def test_space_command_climbs() -> None:
    engine = _engine()
    start_z = float(engine.state.position[2])

    engine.keyboard.press("space")
    state = engine.advance(200)

    assert float(state.position[2]) - start_z > 2.0
    assert state.velocity[2] > 1.0


def test_accelerating_tilts_the_vehicle() -> None:
    engine = _engine()

    engine.keyboard.press("w")
    engine.advance(30)
    _roll, pitch, _yaw = quat_to_euler_rpy(engine.state.attitude_quat)

    # Nose-down pitch is what produces forward thrust in this convention.
    assert abs(pitch) > np.radians(3.0), f"pitch was {np.degrees(pitch):.2f} deg"


def test_yaw_command_changes_the_actual_quaternion() -> None:
    engine = _engine()
    assert quat_to_euler_rpy(engine.state.attitude_quat)[2] == pytest.approx(0.0, abs=1e-6)

    engine.keyboard.press("q")
    left_yaw = quat_to_euler_rpy(engine.advance(100).attitude_quat)[2]

    assert left_yaw > np.radians(30.0), f"left yaw was {np.degrees(left_yaw):.2f} deg"

    engine.keyboard.release("q")
    engine.keyboard.press("e")
    right_yaw = quat_to_euler_rpy(engine.advance(100).attitude_quat)[2]

    assert right_yaw < left_yaw


def test_uncommanded_vehicle_holds_its_heading() -> None:
    """A pure forward input must not spin the airframe.

    Roll/pitch corrections are Euler-angle rates; feeding them straight to the
    body-rate loop leaks into yaw whenever the vehicle is tilted, which used to
    make the quad drift tens of degrees during ordinary flight.
    """
    engine = _engine()

    engine.keyboard.press("w")
    engine.advance(400)
    yaw_while_flying = np.degrees(engine.yaw)

    assert abs(yaw_while_flying) < 3.0, f"yaw drifted to {yaw_while_flying:.1f} deg"


def test_heading_is_held_through_a_mixed_maneuver() -> None:
    engine = _engine()

    engine.keyboard.press("w")
    engine.keyboard.press("d")
    engine.keyboard.press("space")
    engine.advance(300)
    engine.keyboard.neutralize()
    engine.advance(300)

    assert abs(np.degrees(engine.yaw)) < 5.0


def test_yaw_holds_the_new_heading_after_the_key_is_released() -> None:
    engine = _engine()
    engine.keyboard.press("q")
    engine.advance(100)
    engine.keyboard.release("q")
    engine.advance(50)
    settled = np.degrees(engine.yaw)

    engine.advance(400)

    assert abs(np.degrees(engine.yaw) - settled) < 3.0


def test_idle_vehicle_neither_drifts_nor_rotates() -> None:
    engine = _engine()
    start = engine.state.position.copy()

    engine.advance(500)

    assert np.allclose(engine.state.position, start, atol=1e-6)
    assert np.allclose(engine.state.velocity, np.zeros(3), atol=1e-6)
    assert abs(np.degrees(engine.yaw)) < 1e-6


def test_euler_rate_to_body_rate_conversion_is_identity_when_level() -> None:
    from sim_py.backends.multirotor_backend import _euler_rates_to_body_rates

    rates = np.array([0.3, -0.2, 0.5])
    body = _euler_rates_to_body_rates(rates, roll=0.0, pitch=0.0)

    assert np.allclose(body, rates)


def test_euler_rate_to_body_rate_conversion_redistributes_when_tilted() -> None:
    from sim_py.backends.multirotor_backend import _euler_rates_to_body_rates

    # A pure heading change while rolled 90 degrees is a body pitch motion.
    body = _euler_rates_to_body_rates(
        np.array([0.0, 0.0, 1.0]), roll=np.pi / 2.0, pitch=0.0
    )

    assert body[1] == pytest.approx(1.0, abs=1e-9)
    assert body[2] == pytest.approx(0.0, abs=1e-9)


def test_backend_consumes_yaw_rate_metadata() -> None:
    backend = MultirotorBackend()
    backend.reset(
        initial_state=SimState(position=np.array([0.0, 0.0, 10.0]), velocity=np.zeros(3)),
        world={},
        cfg={"simulation": {}},
    )

    for _ in range(100):
        backend.step(
            ControlTarget(accel_cmd=np.zeros(3), metadata={YAW_RATE_METADATA_KEY: 1.0}), 0.01
        )

    yaw = quat_to_euler_rpy(backend.state().attitude_quat)[2]
    assert yaw > np.radians(30.0)


def test_yawed_vehicle_still_accelerates_along_its_own_nose() -> None:
    """Roll/pitch targets must be resolved in the vehicle's heading frame."""
    engine = _engine()
    engine.keyboard.press("q")
    engine.advance(120)
    engine.keyboard.release("q")
    engine.advance(60)  # let the yaw rate settle

    yaw = engine.yaw
    start = engine.state.position.copy()
    engine.keyboard.press("w")
    state = engine.advance(150)

    travel = np.asarray(state.position - start, dtype=float)[:2]
    heading = np.array([np.cos(yaw), np.sin(yaw)])
    assert float(np.linalg.norm(travel)) > 2.0
    # Motion is along the nose, not along the world X axis it started on.
    assert float(travel @ heading) / float(np.linalg.norm(travel)) > 0.9


def test_releasing_controls_stabilizes_the_vehicle() -> None:
    engine = _engine()
    engine.keyboard.press("w")
    engine.keyboard.press("space")
    moving = engine.advance(150)
    assert float(np.linalg.norm(moving.velocity)) > 2.0

    engine.keyboard.neutralize()
    settled = engine.advance(500)

    assert float(np.linalg.norm(settled.velocity)) < 0.5
    roll, pitch, _yaw = quat_to_euler_rpy(settled.attitude_quat)
    assert abs(roll) < np.radians(2.0)
    assert abs(pitch) < np.radians(2.0)


def test_engine_records_telemetry_and_a_flight_trail() -> None:
    engine = _engine()
    engine.keyboard.press("w")
    engine.advance(100)

    trail = engine.telemetry.trail_array()
    assert trail.shape[1] == 3
    assert trail.shape[0] >= 2
    assert engine.telemetry.to_array().shape[0] >= 2


def test_engine_clamps_position_to_world_bounds() -> None:
    engine = _engine()
    engine.keyboard.press("space")
    state = engine.advance(3000)

    assert float(state.position[2]) <= engine.world.max_bounds[2] + 1e-6


# --------------------------------------------------------------------------
# Quadrotor visual model
# --------------------------------------------------------------------------


def test_quad_model_contains_four_motor_positions() -> None:
    motors = QuadGeometry().motor_positions_body()

    assert motors.shape == (4, 3)
    assert len({tuple(np.round(m, 6)) for m in motors}) == 4
    assert sum(1 for m in motors if m[0] > 0) == 2  # two front motors
    assert sum(1 for m in motors if m[0] < 0) == 2  # two rear motors
    assert sum(1 for m in motors if m[1] > 0) == 2  # two left motors


def test_quad_model_has_arms_motors_props_and_a_nose() -> None:
    geometry = QuadGeometry()

    assert len(geometry.arm_segments_body()) == 4
    assert len(geometry.motor_housing_segments_body()) == 4
    assert len(geometry.propeller_polylines_body()) == 4
    assert len(geometry.propeller_blade_segments_body()) == 8
    assert geometry.nose_segments_body()
    assert len(geometry.body_segments_body()) >= 2


def test_wing_model_has_fuselage_wings_and_twin_motors() -> None:
    from sim_py.teleop.model import WingGeometry

    geometry = WingGeometry()
    parts = geometry.segments_body()
    colors, widths = geometry.segment_styles()

    assert len(parts) == 8
    assert len(colors) == len(parts) == len(widths)
    fuse = parts[0]
    assert fuse[1, 0] > fuse[0, 0]
    wing = parts[1]
    assert abs(wing[0, 1] - wing[1, 1]) > abs(fuse[0, 0] - fuse[1, 0]) * 0.8
    even = geometry.segment_styles()[0]
    split = geometry.segment_styles(motor_thrust=np.array([0.15, 0.9]))[0]
    # Disc of each motor (indices 2 and 5) picks up the thrust tint.
    assert even[2] == even[5]
    assert split[2] != split[5]


def test_front_and_rear_motors_are_visually_distinct() -> None:
    colors = QuadGeometry().motor_colors()

    assert len(colors) == 4
    assert MOTOR_IS_FRONT == (True, True, False, False)
    assert len(set(colors)) == 2
    assert colors[0] == colors[1] and colors[2] == colors[3]


def test_hover_thrust_is_even_across_rotors() -> None:
    from sim_py.teleop.model import rotor_thrust_fractions

    hover = rotor_thrust_fractions(collective=1.0)

    assert hover == pytest.approx(np.full(4, 0.5), abs=0.02)


def test_climb_raises_all_rotor_thrusts() -> None:
    from sim_py.teleop.model import rotor_thrust_fractions

    hover = rotor_thrust_fractions(collective=1.0)
    climb = rotor_thrust_fractions(collective=1.6)

    assert np.all(climb > hover)


def test_nose_down_pitch_loads_the_front_rotors() -> None:
    from sim_py.teleop.model import rotor_thrust_fractions

    # FRD +q is nose-down, which a real mixer answers with more front thrust.
    fl, fr, rr, rl = rotor_thrust_fractions(
        collective=1.0, body_rate_cmd=np.array([0.0, 4.0, 0.0]), max_body_rate=4.0
    )

    assert fl > rr and fr > rl


def test_yaw_right_splits_the_opposite_spin_pairs() -> None:
    from sim_py.teleop.model import rotor_thrust_fractions

    fl, fr, rr, rl = rotor_thrust_fractions(
        collective=1.0, body_rate_cmd=np.array([0.0, 0.0, 4.0]), max_body_rate=4.0
    )

    assert fr > fl and rl > rr


def test_thrust_color_runs_from_cool_idle_to_hot_full() -> None:
    from sim_py.teleop.model import thrust_to_color

    idle = thrust_to_color(0.0)
    hover = thrust_to_color(0.5)
    full = thrust_to_color(1.0)

    def rgb(color: str) -> tuple[int, int, int]:
        return int(color[1:3], 16), int(color[3:5], 16), int(color[5:7], 16)

    idle_rgb, hover_rgb, full_rgb = rgb(idle), rgb(hover), rgb(full)
    assert idle != hover != full
    # Grey at zero throttle, then yellow/orange through to red at full.
    assert max(idle_rgb) - min(idle_rgb) < 20
    assert hover_rgb[0] > 200 and hover_rgb[1] > 80 and hover_rgb[2] < 80
    assert full_rgb[0] > 200 and full_rgb[1] < 80
    assert hover_rgb[1] > full_rgb[1]


def test_thrust_color_is_grey_then_yellow_orange_red() -> None:
    from sim_py.teleop.model import thrust_to_color

    def rgb(color: str) -> tuple[int, int, int]:
        return int(color[1:3], 16), int(color[3:5], 16), int(color[5:7], 16)

    grey, yellow, orange, red = (rgb(thrust_to_color(t)) for t in (0.0, 0.22, 0.55, 1.0))
    assert max(grey) - min(grey) < 20
    assert yellow[0] > 200 and yellow[1] > 180 and yellow[2] < 120
    assert orange[0] > 200 and orange[1] < yellow[1] and orange[1] > red[1]
    assert red[0] > 200 and red[1] < 80
    assert len({grey, yellow, orange, red}) == 4


def test_engine_climb_increases_mean_rotor_thrust() -> None:
    hover = _engine()
    hover.advance(30)
    climbing = _engine()
    climbing.keyboard.press("space")
    climbing.advance(30)

    assert float(climbing.motor_thrust_fractions().mean()) > float(hover.motor_thrust_fractions().mean())


def test_nose_indicator_points_along_body_x() -> None:
    tip = QuadGeometry().nose_segments_body()[0][-1]

    assert tip[0] > 0.0
    assert tip[1] == pytest.approx(0.0)


def test_model_rotates_with_yaw() -> None:
    from sim_py.teleop.model import quat_to_rotation_matrix, transform_points, yaw_to_quat

    geometry = QuadGeometry()
    motors = geometry.motor_positions_body()
    rotation = quat_to_rotation_matrix(yaw_to_quat(np.pi / 2.0))
    rotated = transform_points(motors, rotation, np.zeros(3))

    # A 90 degree yaw maps body +X onto world +Y.
    assert rotated[0][1] == pytest.approx(motors[0][0], abs=1e-9)
    assert rotated[0][0] == pytest.approx(-motors[0][1], abs=1e-9)


# --------------------------------------------------------------------------
# Camera
# --------------------------------------------------------------------------


def test_follow_camera_centers_on_the_vehicle() -> None:
    camera = CameraController(
        world_bounds=np.array([[0.0, 0.0, 0.0], [120.0, 120.0, 60.0]]), radius=10.0
    )
    lower, upper = camera.follow_limits(np.array([60.0, 60.0, 30.0]))

    assert lower.tolist() == [50.0, 50.0, 20.0]
    assert upper.tolist() == [70.0, 70.0, 40.0]


def test_follow_camera_slides_instead_of_shrinking_at_a_boundary() -> None:
    camera = CameraController(
        world_bounds=np.array([[0.0, 0.0, 0.0], [120.0, 120.0, 60.0]]), radius=10.0
    )
    lower, upper = camera.follow_limits(np.array([2.0, 60.0, 1.0]))

    assert lower[0] == pytest.approx(0.0)
    assert upper[0] - lower[0] == pytest.approx(20.0)
    assert lower[2] == pytest.approx(0.0)
    assert upper[2] - lower[2] == pytest.approx(20.0)


def test_camera_mode_toggles_between_follow_and_world() -> None:
    camera = CameraController(world_bounds=np.array([[0.0, 0.0, 0.0], [120.0, 120.0, 60.0]]))

    assert camera.mode == FOLLOW
    assert camera.toggle_mode() == WORLD
    limits = camera.limits_for(np.array([10.0, 10.0, 10.0]))
    assert limits[1].tolist() == [120.0, 120.0, 60.0]
    assert camera.toggle_mode() == FOLLOW


def test_world_view_looks_down_steeper_than_follow() -> None:
    """A shallow follow angle hides the aircraft behind terrain in world mode."""
    camera = CameraController(
        world_bounds=np.array([[0.0, 0.0, 0.0], [120.0, 120.0, 60.0]]),
        elevation_deg=22.0,
        world_elevation_deg=34.0,
    )

    class _Ax:
        def set_xlim(self, *a):
            pass

        def set_ylim(self, *a):
            pass

        def set_zlim(self, *a):
            pass

        def set_box_aspect(self, *a, **k):
            pass

        def view_init(self, elev, azim):
            self.elev = elev
            self.azim = azim

    ax = _Ax()
    pos = np.array([10.0, 60.0, 20.0])
    camera.mode = FOLLOW
    camera.apply(ax, pos)
    follow_elev = ax.elev
    camera.mode = WORLD
    camera.apply(ax, pos)

    assert ax.elev > follow_elev
    assert ax.elev == pytest.approx(34.0)


def test_camera_zoom_method_is_not_shadowed_by_the_canvas_fill_field() -> None:
    camera = CameraController(world_bounds=np.array([[0.0, 0.0, 0.0], [120.0, 120.0, 60.0]]))

    assert callable(camera.zoom)
    assert isinstance(camera.canvas_fill, float)


def test_minus_widens_the_view_and_equals_moves_the_camera_in() -> None:
    """`-` must zoom out and `=` must zoom in, as everywhere else."""
    camera = CameraController(world_bounds=np.array([[0.0, 0.0, 0.0], [120.0, 120.0, 60.0]]))
    keys = ks.KeyboardState()
    start = camera.radius

    keys.press("-")
    camera.zoom(keys.zoom_requests)
    widened = camera.radius
    assert widened > start

    keys.zoom_requests = 0
    keys.press("=")
    camera.zoom(keys.zoom_requests)
    assert camera.radius < widened


def test_camera_zoom_is_clamped() -> None:
    camera = CameraController(
        world_bounds=np.array([[0.0, 0.0, 0.0], [120.0, 120.0, 60.0]]),
        radius=14.0,
        min_radius=5.0,
        max_radius=40.0,
    )
    camera.zoom(-50)
    assert camera.radius == pytest.approx(5.0)
    camera.zoom(50)
    assert camera.radius == pytest.approx(40.0)


def test_follow_camera_radius_keeps_local_motion_visible() -> None:
    """A metre of travel must be a meaningful fraction of the view."""
    camera = CameraController(
        world_bounds=np.array([[0.0, 0.0, 0.0], [120.0, 120.0, 60.0]]), radius=14.0
    )
    lower, upper = camera.follow_limits(np.array([60.0, 60.0, 30.0]))
    view_span = float(upper[0] - lower[0])

    assert view_span <= 30.0


def _camera(**overrides) -> CameraController:
    defaults = dict(world_bounds=np.array([[0.0, 0.0, 0.0], [200.0, 200.0, 100.0]]), radius=15.0)
    defaults.update(overrides)
    return CameraController(**defaults)


def test_chase_camera_recomputes_azimuth_from_heading_each_call() -> None:
    """The whole point of chase mode: azimuth must follow the vehicle, not stay fixed."""
    camera = _camera()
    position = np.array([50.0, 50.0, 20.0])

    class _StubAxes:
        def set_xlim(self, *a): pass
        def set_ylim(self, *a): pass
        def set_zlim(self, *a): pass
        def set_box_aspect(self, *a, **kw): pass
        def view_init(self, *, elev, azim): pass

    ax = _StubAxes()
    camera.apply(ax, position, yaw=0.0)
    first = camera.azimuth_deg
    camera.apply(ax, position, yaw=np.radians(90.0))
    second = camera.azimuth_deg

    assert first != second
    assert second - first == pytest.approx(90.0)


def test_chase_camera_offset_puts_it_behind_not_in_front() -> None:
    """heading + offset must land near heading + 180 (behind), not heading (in front).

    Empirically calibrated against the quad's red-front/blue-rear geometry: at
    azim == heading the camera faces the nose head-on; the chase view needs to
    be looking at the tail instead, i.e. roughly opposite.
    """
    camera = _camera()
    behind = (0.0 + camera.chase_azimuth_offset_deg) % 360.0
    assert 135.0 <= behind <= 225.0  # within 45 deg of directly-behind (180)


def test_chase_camera_only_engages_in_follow_mode() -> None:
    camera = _camera(mode=WORLD)
    position = np.array([50.0, 50.0, 20.0])
    camera.azimuth_deg = 12.0

    class _StubAxes:
        def set_xlim(self, *a): pass
        def set_ylim(self, *a): pass
        def set_zlim(self, *a): pass
        def set_box_aspect(self, *a, **kw): pass
        def view_init(self, *, elev, azim): pass

    camera.apply(_StubAxes(), position, yaw=np.radians(45.0))

    assert camera.azimuth_deg == 12.0  # untouched -- world mode doesn't chase


def test_chase_camera_can_be_disabled_for_a_fixed_angle() -> None:
    """A static-still capture script sets azimuth_deg itself and must keep it."""
    camera = _camera(chase=False)
    camera.azimuth_deg = 42.0
    position = np.array([50.0, 50.0, 20.0])

    class _StubAxes:
        def set_xlim(self, *a): pass
        def set_ylim(self, *a): pass
        def set_zlim(self, *a): pass
        def set_box_aspect(self, *a, **kw): pass
        def view_init(self, *, elev, azim): pass

    camera.apply(_StubAxes(), position, yaw=np.radians(90.0))

    assert camera.azimuth_deg == 42.0


def test_chase_camera_defaults_to_enabled() -> None:
    assert CameraController(world_bounds=np.zeros((2, 3))).chase is True


def test_camera_describe_reports_chase_status() -> None:
    on = _camera(chase=True)
    off = _camera(chase=False)

    assert "chase" in on.describe()
    assert "chase" not in off.describe()


def test_renderer_chase_camera_tracks_a_live_turn() -> None:
    """End-to-end: as the vehicle's heading changes across frames, so must the view."""
    from sim_py.teleop.model import yaw_to_quat
    from sim_py.teleop.renderer import HudInfo, TeleopRenderer

    import matplotlib.pyplot as plt

    plt.close("all")
    world = _world()
    camera = CameraController(world_bounds=world.camera_bounds, radius=14.0)
    renderer = TeleopRenderer(
        world=world, camera=camera, geometry=QuadGeometry(), backend_name="multirotor"
    )
    try:
        level = SimState(position=START.copy(), velocity=np.zeros(3), t=0.0)
        renderer.update(level, neutral_command(), np.vstack([START, START + 1.0]), HudInfo())
        azim_level = camera.azimuth_deg

        turned = SimState(
            position=START.copy(),
            velocity=np.zeros(3),
            t=1.0,
            attitude_quat=yaw_to_quat(np.pi / 2.0),
        )
        renderer.update(turned, neutral_command(), np.vstack([START, START + 1.0]), HudInfo())
        azim_turned = camera.azimuth_deg

        assert azim_turned - azim_level == pytest.approx(90.0)
    finally:
        plt.close("all")


# --------------------------------------------------------------------------
# Session assembly and backend guard
# --------------------------------------------------------------------------


def test_non_interactive_backend_fails_with_an_actionable_error() -> None:
    with matplotlib.rc_context():
        matplotlib.use("Agg")
        with pytest.raises(RuntimeError) as excinfo:
            ensure_interactive_backend()

    message = str(excinfo.value)
    assert "interactive" in message.lower()
    assert "MPLBACKEND" in message


def test_session_suppresses_matplotlib_shortcuts_that_collide_with_controls() -> None:
    """Matplotlib binds 'q' to close and 's' to save -- teleop needs them free.

    The suppression must happen in the session itself: a session built through
    ``build_session`` is fully interactive, so relying on the launcher to do it
    would leave 'q' closing the window mid-flight.
    """
    from aerial_kit.sim.teleop import teleop_config
    from sim_py.teleop.session import CONFLICTING_KEYMAPS, build_session

    import matplotlib.pyplot as plt

    with matplotlib.rc_context():
        matplotlib.rcParams["keymap.quit"] = ["q", "ctrl+w"]
        matplotlib.rcParams["keymap.save"] = ["s"]
        session = build_session(teleop_config())
        try:
            assert matplotlib.rcParams["keymap.quit"] == []
            assert matplotlib.rcParams["keymap.save"] == []
            for name in ("q", "s", "p", "c", "h"):
                bound = {
                    key
                    for param in CONFLICTING_KEYMAPS
                    if param in matplotlib.rcParams
                    for key in matplotlib.rcParams[param]
                }
                assert name not in bound
        finally:
            session.stop()
            plt.close("all")

        assert matplotlib.rcParams["keymap.quit"] == ["q", "ctrl+w"]
        assert matplotlib.rcParams["keymap.save"] == ["s"]


def test_launcher_reports_an_unusable_backend_without_a_traceback() -> None:
    """`aerial-kit-teleop` under Agg should exit cleanly, not dump a stack."""
    from aerial_kit.sim.teleop import teleop_config

    with matplotlib.rc_context():
        matplotlib.use("Agg")
        with pytest.raises(SystemExit) as excinfo:
            run_teleop_cli(teleop_config())

    assert "interactive" in str(excinfo.value).lower()


def test_public_launcher_selects_quad_multirotor_and_unlimited_duration() -> None:
    from aerial_kit.sim.teleop import teleop_config

    config = teleop_config()

    assert config.airframe_name == "quad"
    assert config.backend_name == "multirotor"
    assert config.sim_time == 0.0  # 0 means "no time limit"
    assert config.dt > 0.0
    assert "teleop" in config.controller_cfg


def test_teleop_world_attaches_a_planned_path() -> None:
    from aerial_kit.sim.teleop import teleop_config
    from sim_py.teleop.world import build_world

    world = build_world(teleop_config())

    assert world.planned_waypoints is not None
    assert len(world.planned_waypoints) >= 2
    assert world.goal_position is not None
    assert world.planner_type in {"straight", "astar", "rrt", "rrtstar", "dubins"}


def test_teleop_empty_world_when_terrain_is_cleared() -> None:
    from aerial_kit.sim.teleop import teleop_config
    from sim_py.teleop.world import build_world

    cfg = replace(teleop_config(), terrain_override=None, terrain_config_path=None)
    world = build_world(cfg)

    assert world.terrain_type == "empty"
    assert world.obstacles == []
    assert world.terrain is None
    assert world.planned_waypoints is not None
    assert len(world.planned_waypoints) >= 2


def test_teleop_tuning_reads_the_bundled_config() -> None:
    from aerial_kit.sim.teleop import teleop_config

    tuning = TeleopTuning.from_config(teleop_config().controller_cfg)

    assert tuning.accel_xy > 0.0
    assert tuning.yaw_rate > 0.0
    assert tuning.max_speed_xy > tuning.max_speed_z


def test_session_builds_and_ticks_without_keyboard_input() -> None:
    """The HUD frame counter and simulation clock must advance on their own."""
    from aerial_kit.sim.teleop import teleop_config
    from sim_py.teleop.session import build_session

    import matplotlib.pyplot as plt

    session = build_session(teleop_config(), target_fps=60.0)
    try:
        session.start()
        first = session.hud_info()
        for _ in range(5):
            session.tick()
        later = session.hud_info()

        assert later.frames > first.frames
        assert session.engine.state.t > 0.0
        assert later.backend_name == "multirotor"
        assert "follow" in later.camera
        assert later.paused is False
    finally:
        session.stop()
        plt.close("all")

    assert session.keyboard.running is False


def test_session_key_events_reach_the_engine() -> None:
    from aerial_kit.sim.teleop import teleop_config
    from sim_py.teleop.session import build_session

    import matplotlib.pyplot as plt

    session = build_session(teleop_config(), target_fps=60.0)
    try:
        session.start()
        start_x = float(session.engine.state.position[0])
        session.keyboard.press("w")
        session.engine.advance(200)

        assert float(session.engine.state.position[0]) - start_x > 3.0

        session.keyboard.press("c")
        session.tick()
        assert session.camera.mode == WORLD
    finally:
        session.stop()
        plt.close("all")


def test_timer_interval_shrinks_when_the_renderer_cannot_keep_up() -> None:
    from aerial_kit.sim.teleop import teleop_config
    from sim_py.teleop.session import MIN_TIMER_INTERVAL_MS, build_session

    import matplotlib.pyplot as plt

    session = build_session(teleop_config(), target_fps=30.0)
    try:
        nominal = session.timer.interval
        session.scheduler.frames = 20
        session.scheduler.frame_rate = 12.0
        session.retune_timer()

        assert session.timer.interval < nominal

        session.scheduler.frame_rate = 60.0
        for _ in range(12):
            session.retune_timer()

        assert session.timer.interval == nominal
        assert session.timer.interval >= MIN_TIMER_INTERVAL_MS
    finally:
        session.stop()
        plt.close("all")


def test_renderer_draws_every_quad_component() -> None:
    from sim_py.teleop.renderer import HudInfo, TeleopRenderer

    import matplotlib.pyplot as plt

    plt.close("all")
    world = _world()
    camera = CameraController(world_bounds=world.camera_bounds, radius=14.0)
    geometry = QuadGeometry()
    renderer = TeleopRenderer(
        world=world, camera=camera, geometry=geometry, backend_name="multirotor"
    )
    try:
        expected = (
            len(geometry.arm_segments_body())
            + len(geometry.body_segments_body())
            + len(geometry.motor_housing_segments_body())
            + len(geometry.propeller_polylines_body())
            + len(geometry.propeller_blade_segments_body())
            + len(geometry.nose_segments_body())
        )
        assert renderer.quad.segment_count == expected
        assert len(renderer.quad.world_segments) == expected

        state = SimState(position=START.copy(), velocity=np.array([3.0, 0.0, 0.0]), t=1.5)
        renderer.update(state, neutral_command(), np.vstack([START, START + 1.0]), HudInfo())

        assert len(renderer.quad.world_segments) == expected
        motors = renderer.quad.motor_positions_world(START, None)
        assert motors.shape == (4, 3)
        assert len({tuple(np.round(m, 6)) for m in motors}) == 4

        hud = renderer.hud.get_text()

        assert "t     1.50s" in hud
        assert "frame" in hud and "fps" in hud and "real-time" in hud
        assert "pos" in hud and "vel" in hud
        assert "multirotor" in hud
        assert "rpy" in hud  # roll, pitch and yaw
        assert "thr" in hud and "FL" in hud
        assert "collision" in hud
        assert "RUNNING" in hud

        hot = np.array([0.15, 0.95, 0.15, 0.95])
        renderer.update(state, neutral_command(), np.vstack([START, START + 1.0]), HudInfo(motor_thrust=hot))
        cool = renderer.quad.display_colors[renderer.quad._thrust_by_motor[0][0]]
        loaded = renderer.quad.display_colors[renderer.quad._thrust_by_motor[1][0]]
        assert cool != loaded
        # Disc plus two blades per rotor pick up the thrust tint.
        assert len(renderer.quad._thrust_by_motor[1]) >= 3
        assert "FR 95" in renderer.hud.get_text()
    finally:
        plt.close("all")


def test_renderer_does_not_scatter_start_or_goal_blobs() -> None:
    from matplotlib.collections import PathCollection
    from sim_py.teleop.renderer import TeleopRenderer

    import matplotlib.pyplot as plt

    plt.close("all")
    world = _world()
    goal = START + np.array([40.0, 0.0, 0.0])
    world.planned_waypoints = np.vstack([START, START + np.array([20.0, 0.0, 0.0]), goal])
    world.goal_position = goal.copy()
    renderer = TeleopRenderer(
        world=world,
        camera=CameraController(world_bounds=world.camera_bounds, radius=14.0),
        geometry=QuadGeometry(),
        backend_name="multirotor",
    )
    try:
        scatter_count = 0
        for artist in renderer.ax.collections:
            if isinstance(artist, PathCollection) and getattr(artist, "_offsets3d", None) is not None:
                xs = np.asarray(artist._offsets3d[0]).reshape(-1)
                scatter_count += int(xs.size)
        assert scatter_count == 0
    finally:
        plt.close("all")


def test_renderer_transforms_the_model_with_the_attitude_quaternion() -> None:
    from sim_py.teleop.model import yaw_to_quat
    from sim_py.teleop.renderer import HudInfo, TeleopRenderer

    import matplotlib.pyplot as plt

    plt.close("all")
    world = _world()
    renderer = TeleopRenderer(
        world=world,
        camera=CameraController(world_bounds=world.camera_bounds, radius=14.0),
        geometry=QuadGeometry(),
        backend_name="multirotor",
    )
    try:
        trail = np.vstack([START, START])
        level = SimState(position=START.copy(), velocity=np.zeros(3), t=0.0)
        renderer.update(level, neutral_command(), trail, HudInfo())
        level_motors = renderer.quad.motor_positions_world(START, level.attitude_quat)

        yawed = SimState(
            position=START.copy(),
            velocity=np.zeros(3),
            t=0.0,
            attitude_quat=yaw_to_quat(np.pi / 2.0),
        )
        renderer.update(yawed, neutral_command(), trail, HudInfo())
        yawed_motors = renderer.quad.motor_positions_world(START, yawed.attitude_quat)

        assert not np.allclose(level_motors, yawed_motors)
        # Yawing in place must not move the hub away from the vehicle centre.
        assert np.allclose(level_motors.mean(axis=0), yawed_motors.mean(axis=0), atol=1e-9)

        rolled = SimState(
            position=START.copy(),
            velocity=np.zeros(3),
            t=0.0,
            attitude_quat=np.array([np.cos(0.2), np.sin(0.2), 0.0, 0.0]),
        )
        renderer.update(rolled, neutral_command(), trail, HudInfo())
        rolled_motors = renderer.quad.motor_positions_world(START, rolled.attitude_quat)

        # Roll lifts one side of the airframe and drops the other.
        assert rolled_motors[:, 2].max() > level_motors[:, 2].max() + 0.1
        assert rolled_motors[:, 2].min() < level_motors[:, 2].min() - 0.1
    finally:
        plt.close("all")


# --------------------------------------------------------------------------
# Fixed-wing teleop
#
# The multirotor path commands a world-frame acceleration; a fixed wing
# cannot be driven that way (see FixedWingBackend.step's docstring), so it
# gets its own actuator-command mapping, tuning class and idle/neutral
# command. These tests exercise that path in isolation (command mapping),
# through the headless engine (physics), and through the full session
# (renderer + HUD), mirroring the multirotor tests above.
# --------------------------------------------------------------------------


def test_fixedwing_actuator_from_axes_is_neutral_at_rest() -> None:
    neutral = fixedwing_actuator_from_axes(InputAxes(), tuning=FW_TUNING)

    throttle_l, throttle_r, elevon_l, elevon_r = neutral
    assert throttle_l == pytest.approx(FW_TUNING.throttle_trim_n)
    assert throttle_r == pytest.approx(FW_TUNING.throttle_trim_n)
    assert elevon_l == pytest.approx(FW_TUNING.trim_elevon_rad)
    assert elevon_r == pytest.approx(FW_TUNING.trim_elevon_rad)


def test_fixedwing_throttle_axis_moves_both_motors_together() -> None:
    up = fixedwing_actuator_from_axes(InputAxes(up=1.0), tuning=FW_TUNING)
    down = fixedwing_actuator_from_axes(InputAxes(up=-1.0), tuning=FW_TUNING)

    assert up[0] == pytest.approx(up[1])
    assert down[0] == pytest.approx(down[1])
    assert up[0] > FW_TUNING.throttle_trim_n > down[0]
    assert down[0] >= 0.0  # thrust never goes negative


def test_fixedwing_roll_axis_differentiates_the_elevons() -> None:
    # Verified empirically against FixedWingDynamics: a positive
    # elevon_L - elevon_R produces a positive roll moment there, which in
    # this body-FRD convention (y = right) banks the aircraft to the right.
    right = fixedwing_actuator_from_axes(InputAxes(right=1.0), tuning=FW_TUNING)
    left = fixedwing_actuator_from_axes(InputAxes(right=-1.0), tuning=FW_TUNING)

    assert right[2] > right[3]  # elevon_L > elevon_R
    assert left[2] < left[3]
    # Throttle is untouched by a pure roll command.
    assert right[0] == pytest.approx(FW_TUNING.throttle_trim_n)
    assert right[1] == pytest.approx(FW_TUNING.throttle_trim_n)


def test_fixedwing_pitch_axis_moves_both_elevons_together() -> None:
    # Forward stick dives: both elevons deflect the same amount, nose-down.
    forward = fixedwing_actuator_from_axes(InputAxes(forward=1.0), tuning=FW_TUNING)
    backward = fixedwing_actuator_from_axes(InputAxes(forward=-1.0), tuning=FW_TUNING)

    assert forward[2] == pytest.approx(forward[3])
    assert backward[2] == pytest.approx(backward[3])
    assert forward[2] < FW_TUNING.trim_elevon_rad < backward[2]


def test_fixedwing_yaw_axis_biases_thrust_not_elevons() -> None:
    # No rudder on a flying wing -- Q/E nudges differential thrust instead.
    yaw = fixedwing_actuator_from_axes(InputAxes(yaw=1.0), tuning=FW_TUNING)

    assert yaw[0] != pytest.approx(yaw[1])
    assert yaw[2] == pytest.approx(FW_TUNING.trim_elevon_rad)
    assert yaw[3] == pytest.approx(FW_TUNING.trim_elevon_rad)


def test_fixedwing_command_from_keys_round_trips_through_axes() -> None:
    keyboard = ks.KeyboardState()
    keyboard.press("d")
    command = fixedwing_command_from_keys(keyboard, tuning=FW_TUNING)

    assert command.axes.right == 1.0
    assert command.actuator_cmd is not None
    assert command.actuator_cmd[2] > command.actuator_cmd[3]


def test_fixedwing_neutral_command_keeps_cruise_thrust_not_zero() -> None:
    # Unlike the multirotor (thrust off = hover), zero thrust here means fall.
    neutral = neutral_fixedwing_command(FW_TUNING)

    assert neutral.actuator_cmd is not None
    assert neutral.actuator_cmd[0] == pytest.approx(FW_TUNING.throttle_trim_n)
    assert neutral.actuator_cmd[1] == pytest.approx(FW_TUNING.throttle_trim_n)


def test_fixedwing_command_to_control_target_carries_actuator_cmd() -> None:
    command = fixedwing_command_from_keys(ks.KeyboardState(), tuning=FW_TUNING)
    target = command.to_control_target()

    assert target.metadata["actuator_cmd"] is not None
    assert "yaw_rate_cmd" not in target.metadata
    np.testing.assert_array_equal(target.accel_cmd, np.zeros(3))


def test_fixedwing_engine_flies_forward_under_neutral_input() -> None:
    engine = _fixedwing_engine()
    engine.advance(50)

    assert engine.command.actuator_cmd is not None
    assert engine.state.position[0] > START[0]  # moved forward along +x
    assert np.all(np.isfinite(engine.state.position))
    assert np.all(np.isfinite(engine.state.velocity))


def test_fixedwing_engine_right_roll_input_banks_right() -> None:
    engine = _fixedwing_engine()
    engine.keyboard.press("d")
    engine.advance(30)

    _pitch, bank = body_axis_pitch_bank(engine.state.attitude_quat)
    assert bank > np.radians(5.0)


def test_fixedwing_engine_left_roll_input_banks_left() -> None:
    engine = _fixedwing_engine()
    engine.keyboard.press("a")
    engine.advance(30)

    _pitch, bank = body_axis_pitch_bank(engine.state.attitude_quat)
    assert bank < -np.radians(5.0)


def test_fixedwing_engine_forward_input_pitches_nose_down() -> None:
    engine = _fixedwing_engine()
    engine.keyboard.press("w")
    engine.advance(15)

    pitch, _bank = body_axis_pitch_bank(engine.state.attitude_quat)
    assert pitch < -np.radians(5.0)


def test_fixedwing_engine_backward_input_pitches_nose_up() -> None:
    engine = _fixedwing_engine()
    engine.keyboard.press("s")
    engine.advance(15)

    pitch, _bank = body_axis_pitch_bank(engine.state.attitude_quat)
    assert pitch > np.radians(5.0)


def test_fixedwing_engine_neutralizes_when_unfocused() -> None:
    # Losing focus must fall back to trimmed flight, not a missing actuator_cmd.
    engine = _fixedwing_engine()
    engine.keyboard.press("w")
    engine.keyboard.set_focused(False)

    engine.advance(5)  # must not raise FixedWingBackend's missing-actuator_cmd error

    assert engine.command.actuator_cmd is not None
    assert engine.command.actuator_cmd[2] == pytest.approx(FW_TUNING.trim_elevon_rad)


def test_fixedwing_motor_thrust_fractions_are_two_wide_and_bounded() -> None:
    engine = _fixedwing_engine()
    engine.advance(5)

    fractions = engine.motor_thrust_fractions()
    assert fractions.shape == (2,)
    assert np.all(fractions >= 0.0) and np.all(fractions <= 1.0)


class _NaNAfterOneStepBackend:
    """A backend that flies fine once, then diverges -- for testing the
    engine's non-finite-state guard without needing a real dynamics model to
    actually blow up (slow, and dependent on exact aero coefficients)."""

    def __init__(self) -> None:
        self._t = 0.0
        self._steps = 0

    def reset(self, *, initial_state, world, cfg) -> None:
        self._state = initial_state

    def step(self, control_target, dt: float) -> None:
        self._steps += 1
        self._t += dt
        if self._steps >= 2:
            self._state = SimState(
                position=np.full(3, np.nan),
                velocity=np.full(3, np.nan),
                t=self._t,
                attitude_quat=np.full(4, np.nan),
                body_rates=np.full(3, np.nan),
            )
        else:
            self._state = SimState(
                position=self._state.position + 1.0,
                velocity=self._state.velocity,
                t=self._t,
                attitude_quat=self._state.attitude_quat,
                body_rates=self._state.body_rates,
            )

    def apply_constraints(self, **kwargs) -> None:
        pass

    def state(self) -> SimState:
        return self._state


def test_engine_freezes_and_flags_a_crash_instead_of_propagating_nan() -> None:
    """Regression: a diverging fixed-wing model used to hand NaN position to
    the renderer, which crashed the whole GUI on ax.set_xlim(nan, nan)."""
    backend = _NaNAfterOneStepBackend()
    backend.reset(
        initial_state=SimState(
            position=START.copy(),
            velocity=FW_CRUISE_VELOCITY.copy(),
            t=0.0,
            attitude_quat=np.array([1.0, 0.0, 0.0, 0.0]),
            body_rates=np.zeros(3),
        ),
        world={},
        cfg={},
    )
    engine = TeleopEngine(backend=backend, world=_world(), tuning=FW_TUNING, dt=0.02)

    engine.step()  # advances fine
    assert not engine.crashed
    last_good = engine.state.position.copy()

    engine.step()  # backend now returns NaN
    assert engine.crashed
    assert engine.colliding
    np.testing.assert_array_equal(engine.state.position, last_good)  # frozen, not NaN
    assert np.all(np.isfinite(engine.state.position))

    engine.advance(10)  # must not raise, and must stay frozen
    np.testing.assert_array_equal(engine.state.position, last_good)


def test_hud_reports_crashed_distinctly_from_a_normal_collision() -> None:
    """A diverged flight and a bumped obstacle must not read the same to the pilot."""
    from sim_py.teleop.renderer import HudInfo, TeleopRenderer

    import matplotlib.pyplot as plt

    plt.close("all")
    world = _world()
    camera = CameraController(world_bounds=world.camera_bounds, radius=14.0)
    renderer = TeleopRenderer(
        world=world, camera=camera, geometry=QuadGeometry(), backend_name="multirotor"
    )
    try:
        state = SimState(position=START.copy(), velocity=np.zeros(3), t=1.0)
        text = renderer._hud_text(state, neutral_command(), HudInfo(crashed=True))
        assert "CRASHED" in text

        text = renderer._hud_text(state, neutral_command(), HudInfo(colliding=True, crashed=False))
        assert "CRASHED" not in text
    finally:
        plt.close("all")


def test_public_launcher_selects_fixed_wing_when_requested() -> None:
    from aerial_kit.sim.teleop import teleop_config

    config = teleop_config("fixed-wing")

    assert config.airframe_name == "twin_wing"
    assert config.backend_name == "fixedwing"
    assert config.sim_time == 0.0
    assert config.initial_state_cfg.get("velocity_mps") is not None


def test_public_launcher_rejects_an_unknown_airframe() -> None:
    from aerial_kit.sim.teleop import teleop_config

    with pytest.raises(ValueError):
        teleop_config("hexacopter")


def test_renderer_draws_the_wing_for_a_fixedwing_geometry() -> None:
    from sim_py.teleop.renderer import HudInfo, TeleopRenderer

    import matplotlib.pyplot as plt

    plt.close("all")
    world = _world()
    camera = CameraController(world_bounds=world.camera_bounds, radius=25.0)
    geometry = WingGeometry()
    renderer = TeleopRenderer(
        world=world, camera=camera, geometry=geometry, backend_name="fixedwing"
    )
    try:
        expected = len(geometry.segments_body())
        assert renderer.wing.segment_count == expected
        assert renderer.airframe is renderer.wing
        assert not hasattr(renderer, "quad")

        command = fixedwing_command_from_keys(ks.KeyboardState(), tuning=FW_TUNING)
        state = SimState(
            position=START.copy(), velocity=FW_CRUISE_VELOCITY.copy(), t=1.5
        )
        info = HudInfo(motor_thrust=np.array([0.8, 0.3]))
        renderer.update(state, command, np.vstack([START, START + 1.0]), info)

        assert len(renderer.wing.world_segments) == expected
        # HUD text must render without raising for the two-motor / actuator-cmd path.
        text = renderer._hud_text(state, command, info)
        assert "thr  L 80   R 30 %" in text
        assert "thrN" in text  # actuator suffix, not the multirotor's "yr" field
    finally:
        plt.close("all")


def test_session_builds_fixed_wing_and_ticks_without_keyboard_input() -> None:
    from aerial_kit.sim.teleop import teleop_config
    from sim_py.teleop.session import build_session

    import matplotlib.pyplot as plt

    session = build_session(teleop_config("fixed-wing"), target_fps=60.0)
    try:
        session.start()
        first = session.hud_info()
        for _ in range(5):
            session.tick()
        later = session.hud_info()

        assert later.frames > first.frames
        assert session.engine.state.t > 0.0
        assert later.backend_name == "fixedwing"
        assert later.motor_thrust.shape == (2,)
        assert hasattr(session.renderer, "wing")
    finally:
        session.stop()
        plt.close("all")

    assert session.keyboard.running is False


def test_session_fixed_wing_key_events_reach_the_engine_as_actuator_commands() -> None:
    from aerial_kit.sim.teleop import teleop_config
    from sim_py.teleop.session import build_session

    import matplotlib.pyplot as plt

    session = build_session(teleop_config("fixed-wing"), target_fps=60.0)
    try:
        session.start()
        session._on_key_press(type("Event", (), {"key": "d"})())
        session.engine.advance(5)

        assert session.engine.command.actuator_cmd is not None
        left, right = session.engine.command.actuator_cmd[2:]
        assert left > right  # right-roll differential reached the backend
    finally:
        session.stop()
        plt.close("all")
