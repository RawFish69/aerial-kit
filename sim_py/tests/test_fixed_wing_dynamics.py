"""Tests for FixedWingDynamics/FixedWingBackend (workstream 04)."""

from __future__ import annotations

import numpy as np
import pytest

from sim_py.backends.fixedwing_backend import FixedWingBackend
from sim_py.core.types import ControlTarget, SimState
from aerial_kit.dynamics.fixed_wing import (
    FixedWingDynamics,
    FixedWingParams,
    level_attitude_quat,
    lift_coefficient,
)


def test_lift_coefficient_increases_with_alpha_pre_stall():
    params = FixedWingParams()
    cl_low = lift_coefficient(np.radians(2.0), params)
    cl_high = lift_coefficient(np.radians(8.0), params)
    assert cl_high > cl_low


def test_lift_coefficient_does_not_blow_up_past_stall():
    params = FixedWingParams()
    cl_at_60_deg = lift_coefficient(np.radians(60.0), params)
    # Linear extrapolation would give an absurd value; the flat-plate blend
    # must bound it well below that.
    cl_linear_naive = params.cl0 + params.cl_alpha * np.radians(60.0)
    assert abs(cl_at_60_deg) < abs(cl_linear_naive)
    assert abs(cl_at_60_deg) < 2.5


def test_lift_coefficient_symmetric_in_sign_of_alpha_past_stall():
    params = FixedWingParams()
    cl_pos = lift_coefficient(np.radians(45.0), params)
    cl_neg = lift_coefficient(np.radians(-45.0), params)
    assert cl_pos == pytest.approx(-cl_neg, abs=1e-6)


def test_compute_trim_balances_lift_against_weight():
    params = FixedWingParams()
    dyn = FixedWingDynamics(params=params)
    cruise = 15.0
    actuator_cmd = dyn.compute_trim(cruise)
    throttle_l, throttle_r, elevon_l, elevon_r = actuator_cmd

    assert throttle_l == pytest.approx(throttle_r)
    assert elevon_l == pytest.approx(elevon_r)
    assert abs(elevon_l) < params.alpha_stall_rad  # sane elevon deflection

    qbar = 0.5 * params.air_density_kg_m3 * cruise**2
    cl_trim = (params.mass_kg * params.gravity_mps2) / (qbar * params.wing_area_m2)
    alpha_trim = (cl_trim - params.cl0) / params.cl_alpha
    # compute_trim() uses the pure-linear CL relation; lift_coefficient() applies
    # the stall blend, which is never exactly zero, so allow its small contribution.
    assert lift_coefficient(alpha_trim, params) == pytest.approx(cl_trim, rel=0.01)


def test_trim_holds_level_flight_over_60_seconds():
    """Plan 04's acceptance test: trim holds altitude +/-2 m, airspeed +/-1 m/s over 60 s."""
    params = FixedWingParams()
    cruise = 15.0
    dyn = FixedWingDynamics(params=params)
    dyn.position = np.array([0.0, 0.0, 100.0])
    dyn.velocity = np.array([cruise, 0.0, 0.0])
    dyn.attitude_quat = level_attitude_quat(heading_rad=0.0)
    actuator_cmd = dyn.compute_trim(cruise)

    dt = 0.01
    steps = int(60.0 / dt)
    for _ in range(steps):
        dyn.step(actuator_cmd, dt)

    altitude_change = abs(dyn.position[2] - 100.0)
    airspeed, _, _ = dyn.airspeed_alpha_beta()
    airspeed_change = abs(airspeed - cruise)

    assert altitude_change < 2.0, f"altitude drifted {altitude_change:.2f} m"
    assert airspeed_change < 1.0, f"airspeed drifted {airspeed_change:.2f} m/s"


def test_backend_requires_actuator_cmd_metadata():
    backend = FixedWingBackend()
    backend.reset(
        initial_state=SimState(position=np.zeros(3), velocity=np.array([15.0, 0.0, 0.0])),
        world={},
        cfg={},
    )
    with pytest.raises(ValueError, match="actuator_cmd"):
        backend.step(ControlTarget(accel_cmd=np.zeros(3)), dt=0.01)


def test_backend_reset_reads_fixedwing_config_block():
    backend = FixedWingBackend()
    backend.reset(
        initial_state=SimState(position=np.array([1.0, 2.0, 3.0]), velocity=np.zeros(3)),
        world={},
        cfg={"simulation": {"fixedwing": {"mass_kg": 2.0, "wind_mps": [1.0, 0.0, 0.0]}}},
    )
    assert backend._dyn is not None
    assert backend._dyn.params.mass_kg == pytest.approx(2.0)
    assert backend._dyn.wind_mps == pytest.approx(np.array([1.0, 0.0, 0.0]))
    state = backend.state()
    assert state.position == pytest.approx(np.array([1.0, 2.0, 3.0]))
    assert state.attitude_quat is not None
    assert state.body_rates is not None


def test_backend_trim_step_via_actuator_cmd_metadata():
    backend = FixedWingBackend()
    cruise = 15.0
    backend.reset(
        initial_state=SimState(position=np.array([0.0, 0.0, 50.0]), velocity=np.array([cruise, 0.0, 0.0])),
        world={},
        cfg={},
    )
    actuator_cmd = backend._dyn.compute_trim(cruise)
    for _ in range(200):
        backend.step(
            ControlTarget(accel_cmd=np.zeros(3), metadata={"actuator_cmd": actuator_cmd}),
            dt=0.01,
        )
    state = backend.state()
    assert abs(state.position[2] - 50.0) < 0.5
