"""Allocator and shared-controller wiring for the single-motor wing."""

from __future__ import annotations

import math

import numpy as np
import pytest

from aerial_kit.airframes.single_wing import load_single_wing_spec
from aerial_kit.airframes.single_wing_airframe import SingleWingAirframe
from aerial_kit.controllers.single_wing import (
    SingleWingL1TECSController,
    single_wing_airspeed_nav_wrench,
    single_wing_controller_config,
)
from aerial_kit.types import CommandKind, SimState, Waypoint, Wrench


def _state(airspeed_mps: float = 14.0) -> SimState:
    return SimState(
        position=np.array([0.0, 0.0, 100.0]),
        velocity=np.array([airspeed_mps, 0.0, 0.0]),
        airspeed_mps=airspeed_mps,
        body_rates=np.zeros(3),
    )


def test_single_wing_capabilities_and_trim_shape() -> None:
    spec = load_single_wing_spec()
    wing = SingleWingAirframe(spec=spec)
    assert wing.capabilities.n_actuators == 3
    assert wing.capabilities.command_kind == CommandKind.AIRSPEED_NAV
    assert wing.capabilities.can_hover is False
    assert wing.capabilities.min_airspeed_mps == pytest.approx(
        spec.flight_envelope["min_airspeed_mps"])
    assert wing.cruise_airspeed_mps == pytest.approx(
        spec.flight_envelope["cruise_airspeed_mps"])

    trim = wing.trim(_state())
    assert trim.shape == (3,)
    expected = wing.dynamics.compute_trim(14.0)
    np.testing.assert_allclose(trim, expected)
    # Spin +1 makes motor reaction roll left, so the trim rolls right to offset it.
    assert trim[2] > trim[1]


def test_yaw_demand_is_dropped_because_there_is_no_yaw_actuator() -> None:
    wing = SingleWingAirframe()
    state = _state()
    force = np.array([wing.trim(state)[0], 0.0, 0.0])
    without_yaw = wing.allocate(
        Wrench(force_body=force, moment_body=np.array([0.1, 0.2, 0.0])),
        state,
    )
    with_yaw = wing.allocate(
        Wrench(force_body=force, moment_body=np.array([0.1, 0.2, 20.0])),
        state,
    )
    np.testing.assert_allclose(without_yaw, with_yaw)


def test_allocator_is_finite_below_the_operational_speed_envelope() -> None:
    wing = SingleWingAirframe()
    state = _state(airspeed_mps=2.0)
    command = wing.allocate(
        Wrench(
            force_body=np.array([wing.max_thrust_n, 0.0, 0.0]),
            moment_body=np.array([10.0, 10.0, 10.0]),
        ),
        state,
    )
    assert np.all(np.isfinite(command))
    assert np.all(np.abs(command[1:]) <= wing.max_elevon_rad + 1e-12)


def test_pitch_has_priority_over_roll_at_the_elevon_limit() -> None:
    wing = SingleWingAirframe()
    state = _state()
    limit = wing.max_elevon_rad
    trim_common, _, _ = wing.dynamics.trim_moment_bases(14.0, wing.trim(state)[0])
    requested_pitch = 2.0 * limit
    command = wing.allocate(
        Wrench(
            force_body=np.array([wing.trim(state)[0], 0.0, 0.0]),
            moment_body=np.array([10.0, requested_pitch, 0.0]),
        ),
        state,
    )
    common = 0.5 * (command[1] + command[2])
    differential = 0.5 * (command[2] - command[1])
    assert common == math.copysign(limit, trim_common + requested_pitch)
    assert differential == 0.0
    assert abs(command[1]) <= limit + 1e-12
    assert abs(command[2]) <= limit + 1e-12

    command_with_headroom = wing.allocate(
        Wrench(
            force_body=np.array([wing.trim(state)[0], 0.0, 0.0]),
            moment_body=np.array([10.0, 0.10, 0.0]),
        ),
        state,
    )
    common = 0.5 * (command_with_headroom[1] + command_with_headroom[2])
    differential = 0.5 * (
        command_with_headroom[2] - command_with_headroom[1])
    assert common == pytest.approx(trim_common + 0.10)
    assert differential > 0.0
    assert abs(common) + abs(differential) == pytest.approx(limit)


def test_shared_controller_and_airspeed_nav_clear_yaw_demand() -> None:
    wing = SingleWingAirframe()
    state = _state()
    state.attitude_quat = np.array([0.0, 1.0, 0.0, 0.0])
    state.body_rates = np.array([0.0, 0.0, 1.0])
    cfg = single_wing_controller_config(14.0)
    result = SingleWingL1TECSController().compute(
        state,
        Waypoint(position=np.array([200.0, 80.0, 100.0])),
        cfg,
    )
    assert result.wrench is not None
    assert result.wrench.moment_body[2] == 0.0
    assert result.metadata["controller"] == "single_wing_l1_tecs"

    airspeed_nav = single_wing_airspeed_nav_wrench(
        state,
        airspeed_cmd=14.0,
        bank_cmd=math.radians(25.0),
        climb_rate_cmd=0.5,
        cfg=cfg,
    )
    assert airspeed_nav.moment_body[2] == 0.0

