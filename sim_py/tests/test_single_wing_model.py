"""Single-motor flying-wing specification, trim and propulsion tests."""

from __future__ import annotations

import math

import numpy as np
import pytest

from aerial_kit.airframes.single_wing import load_single_wing_spec
from aerial_kit.dynamics.fixed_wing import MAX_BODY_RATE_RAD_S
from aerial_kit.dynamics.rotations import rotmat_to_quat
from aerial_kit.dynamics.single_wing import SingleWingDynamics, SingleWingParams
from sim_py.backends.single_wing_backend import SingleWingBackend
from sim_py.core.types import ControlTarget, SimState


def attitude_for_alpha_heading(alpha_rad: float, heading_rad: float = 0.0) -> np.ndarray:
    """Body-FRD to world-ENU attitude for horizontal air velocity at alpha."""
    c = math.cos(alpha_rad)
    s = math.sin(alpha_rad)
    ch = math.cos(heading_rad)
    sh = math.sin(heading_rad)
    rotation = np.column_stack((
        np.array([c * ch, c * sh, s]),
        np.array([sh, -ch, 0.0]),
        np.array([s * ch, s * sh, -c]),
    ))
    return rotmat_to_quat(rotation)


def trimmed_dynamics(
    airspeed_mps: float = 14.0,
    *,
    spin: int = 1,
) -> tuple[SingleWingDynamics, dict[str, float]]:
    spec = load_single_wing_spec()
    dyn = SingleWingDynamics(spec.dynamics_params(prop_spin=spin))
    trim = dyn.compute_trim_detailed(airspeed_mps)
    dyn.position = np.array([0.0, 0.0, 100.0])
    dyn.velocity = np.array([airspeed_mps, 0.0, 0.0])
    dyn.attitude_quat = attitude_for_alpha_heading(trim["alpha_rad"])
    dyn.thrust_n = trim["thrust_n"]
    return dyn, trim


def test_single_wing_yaml_is_the_parameter_source_of_truth() -> None:
    spec = load_single_wing_spec()
    defaults = SingleWingParams()
    for key, value in spec.dynamics_param_overrides().items():
        np.testing.assert_allclose(
            getattr(defaults, key), value, err_msg=f"dataclass drift: {key}")

    assert spec.planform_area() == pytest.approx(spec.area, abs=1e-12)
    assert spec.area / spec.span == pytest.approx(spec.mean_chord, abs=1e-12)
    assert 1.0 <= spec.total_mass_kg <= 1.2
    assert 1.1 <= spec.span <= 1.3


def test_trim_exists_and_holds_level_flight_for_30_seconds() -> None:
    cruise = 14.0
    dyn, trim = trimmed_dynamics(cruise)
    command = dyn.compute_trim(cruise)
    assert np.all(np.isfinite(command))
    assert 0.0 < trim["thrust_n"] < dyn.params.max_thrust_n
    assert 0.0 < trim["alpha_rad"] < dyn.params.alpha_stall_rad

    dt = 0.01
    for _ in range(int(30.0 / dt)):
        dyn.step(command, dt)

    airspeed, alpha, beta = dyn.airspeed_alpha_beta()
    assert np.all(np.isfinite(dyn.position))
    assert np.all(np.isfinite(dyn.velocity))
    assert np.all(np.isfinite(dyn.attitude_quat))
    assert np.all(np.isfinite(dyn.body_rates))
    assert dyn.position[2] == pytest.approx(100.0, abs=0.1)
    assert airspeed == pytest.approx(cruise, abs=0.1)
    assert alpha == pytest.approx(trim["alpha_rad"], abs=1e-4)
    assert abs(beta) < 1e-6


def test_positive_elevons_pitch_up_and_right_differential_rolls_right() -> None:
    dyn, trim = trimmed_dynamics()
    trim_cmd = dyn.compute_trim(14.0)
    _, baseline = dyn.aero_forces_moments(trim_cmd, trim["thrust_n"])

    pitch_cmd = trim_cmd.copy()
    pitch_cmd[1:] += math.radians(2.0)
    _, pitched = dyn.aero_forces_moments(pitch_cmd, trim["thrust_n"])
    assert pitched[1] > baseline[1]

    roll_cmd = trim_cmd.copy()
    roll_cmd[1] -= math.radians(2.0)
    roll_cmd[2] += math.radians(2.0)
    _, rolled = dyn.aero_forces_moments(roll_cmd, trim["thrust_n"])
    assert rolled[0] > baseline[0]


@pytest.mark.parametrize(
    ("spin", "expected_sign"),
    [(1, -1.0), (-1, 1.0)],
)
def test_motor_reaction_follows_spin_and_is_trimmed_out(
    spin: int, expected_sign: float
) -> None:
    spec = load_single_wing_spec()
    params = spec.dynamics_params(
        prop_spin=spin,
        enable_thrust_offset=False,
        enable_prop_wash=False,
    )
    dyn = SingleWingDynamics(params)
    dyn.attitude_quat = attitude_for_alpha_heading(0.0)
    dyn.velocity = np.array([14.0, 0.0, 0.0])
    _, moment = dyn.aero_forces_moments(
        np.array([2.0, 0.0, 0.0]), thrust_n=2.0)
    assert np.sign(moment[0]) == expected_sign
    assert abs(moment[0]) == pytest.approx(
        2.0 * params.reaction_torque_per_n_nm, rel=1e-12)

    trimmed, trim = trimmed_dynamics(spin=spin)
    _, balanced_moment = trimmed.aero_forces_moments(
        trimmed.compute_trim(14.0), thrust_n=trim["thrust_n"])
    np.testing.assert_allclose(balanced_moment, np.zeros(3), atol=1e-12)


def test_first_order_thrust_lag_and_feature_switches() -> None:
    spec = load_single_wing_spec()
    lagged = SingleWingDynamics(spec.dynamics_params(
        thrust_tau_s=0.25, enable_thrust_lag=True))
    instant = SingleWingDynamics(spec.dynamics_params(
        thrust_tau_s=0.25, enable_thrust_lag=False))
    lagged_value = lagged.update_thrust(4.0, 0.05)
    assert 0.0 < lagged_value < 4.0
    assert instant.update_thrust(4.0, 0.05) == pytest.approx(4.0)

    wash_on = SingleWingDynamics(spec.dynamics_params(
        enable_prop_wash=True, enable_thrust_offset=False))
    wash_off = SingleWingDynamics(spec.dynamics_params(
        enable_prop_wash=False, enable_thrust_offset=False))
    for dyn in (wash_on, wash_off):
        dyn.attitude_quat = attitude_for_alpha_heading(0.0)
        dyn.velocity = np.array([4.0, 0.0, 0.0])
    command = np.array([6.0, math.radians(5.0), math.radians(5.0)])
    _, moment_on = wash_on.aero_forces_moments(command, thrust_n=6.0)
    _, moment_off = wash_off.aero_forces_moments(command, thrust_n=6.0)
    assert moment_on[1] > moment_off[1]

    offset_on = SingleWingDynamics(spec.dynamics_params(
        enable_thrust_offset=True, enable_motor_reaction=False))
    offset_off = SingleWingDynamics(spec.dynamics_params(
        enable_thrust_offset=False, enable_motor_reaction=False))
    for dyn in (offset_on, offset_off):
        dyn.attitude_quat = attitude_for_alpha_heading(0.0)
        dyn.velocity = np.zeros(3)
    zero_control = np.array([3.0, 0.0, 0.0])
    _, with_offset = offset_on.aero_forces_moments(zero_control, thrust_n=3.0)
    _, without_offset = offset_off.aero_forces_moments(
        zero_control, thrust_n=3.0)
    assert with_offset[1] < 0.0
    assert without_offset[1] == pytest.approx(0.0, abs=1e-12)


def test_wind_is_removed_before_airspeed_and_aero_evaluation() -> None:
    spec = load_single_wing_spec()
    wind = np.array([3.0, 1.0, 0.0])
    windy = SingleWingDynamics(spec.dynamics_params(), wind_mps=wind)
    calm = SingleWingDynamics(spec.dynamics_params())
    for dyn in (windy, calm):
        dyn.attitude_quat = attitude_for_alpha_heading(0.0)
        dyn.body_rates = np.zeros(3)
    windy.velocity = np.array([17.0, 1.0, 0.0])
    calm.velocity = np.array([14.0, 0.0, 0.0])

    assert windy.airspeed_alpha_beta()[0] == pytest.approx(14.0)
    assert windy.airspeed_alpha_beta()[1] == pytest.approx(0.0, abs=1e-12)
    command = np.array([1.0, 0.0, 0.0])
    windy_force, windy_moment = windy.aero_forces_moments(command)
    calm_force, calm_moment = calm.aero_forces_moments(command)
    np.testing.assert_allclose(windy_force, calm_force, atol=1e-12)
    np.testing.assert_allclose(windy_moment, calm_moment, atol=1e-12)


def test_full_stick_commands_stay_finite_and_rate_limited() -> None:
    dyn, _ = trimmed_dynamics()
    max_elevon = dyn.params.max_elevon_rad
    commands = (
        np.array([dyn.params.max_thrust_n, max_elevon, max_elevon]),
        np.array([0.0, -max_elevon, max_elevon]),
        np.array([dyn.params.max_thrust_n, max_elevon, -max_elevon]),
        np.array([0.0, max_elevon, max_elevon]),
    )
    for index in range(1200):
        dyn.step(commands[(index // 300) % len(commands)], 0.01)
        assert np.all(np.isfinite(dyn.position))
        assert np.all(np.isfinite(dyn.velocity))
        assert np.all(np.isfinite(dyn.attitude_quat))
        assert np.all(np.isfinite(dyn.body_rates))
        assert np.all(
            np.abs(dyn.body_rates) <= MAX_BODY_RATE_RAD_S + 1e-12)


def test_backend_requires_actuator_metadata_and_uses_configured_wind() -> None:
    backend = SingleWingBackend()
    backend.reset(
        initial_state=SimState(
            position=np.array([0.0, 0.0, 100.0]),
            velocity=np.array([14.0, 0.0, 0.0]),
            airspeed_mps=14.0,
        ),
        world={},
        cfg={
            "simulation": {
                "singlewing": {
                    "wind_mps": [3.0, 1.0, 0.0],
                },
            },
        },
    )
    assert backend._dyn is not None
    np.testing.assert_allclose(backend._dyn.wind_mps, [3.0, 1.0, 0.0])
    with pytest.raises(ValueError, match="actuator_cmd"):
        backend.step(ControlTarget(accel_cmd=np.zeros(3)), dt=0.01)

    backend.step(
        ControlTarget(
            accel_cmd=np.zeros(3),
            metadata={"actuator_cmd": [1.0, 0.0, 0.0]},
        ),
        dt=0.01,
    )
    state = backend.state()
    assert state.airspeed_mps is not None
    assert np.isfinite(state.airspeed_mps)

