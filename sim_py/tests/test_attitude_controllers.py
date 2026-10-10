"""Geometric SE(3) control and thrust/body-rate NMPC, on the actuator plant.

Both controllers return a wrench, so they are flown through
``aerial_kit.dynamics.actuator_loop.fly`` on the same motor-level plant the
cascade tests use (first-order motor lag, per-motor saturation, 500 Hz) rather
than on an ideal-acceleration backend that would hide what attitude control is
for.
"""

from __future__ import annotations

import numpy as np
import pytest

from aerial_kit.airframes.multirotor import MultirotorAirframe
from aerial_kit.controllers import CascadeController, CascadeGains, PIDController
from aerial_kit.controllers.geometric import (
    FlatReference,
    GeometricController,
    GeometricGains,
    desired_attitude,
    flat_body_rates,
    hat,
    vee,
)
from aerial_kit.controllers.nmpc import (
    NMPCController,
    NMPCGains,
    QuadrotorNMPC,
    quad_dynamics,
    rk4_step,
    yaw_quat,
)
from aerial_kit.dynamics import ActuatorPlant, ActuatorPlantParams
from aerial_kit.dynamics.actuator_loop import fly
from aerial_kit.dynamics.multirotor_actuator import QUAD_X_SPIN, quad_x_positions
from aerial_kit.dynamics.rotations import quat_to_rotmat, rotmat_to_quat
from aerial_kit.types import SimState, Waypoint

DT = 0.002
G = 9.81
ARM, MASS, YAW_C, TMAX = 0.2, 1.0, 0.02, 6.0


def airframe() -> MultirotorAirframe:
    return MultirotorAirframe(arms=4, layout="x", arm_length_m=ARM, mass_kg=MASS, yaw_torque_coeff=YAW_C)


def plant(altitude_m: float = 2.0, east_m: float = 0.0, roll_deg: float = 0.0) -> ActuatorPlant:
    """At rest, nose north (NED as the plant comes), optionally rolled."""
    p = ActuatorPlant(
        params=ActuatorPlantParams(mass_kg=MASS, max_thrust_per_motor_n=TMAX, yaw_torque_coeff=YAW_C),
        motor_positions=quad_x_positions(ARM),
        spin=QUAD_X_SPIN.copy(),
    )
    p.position[...] = np.array([0.0, east_m, -altitude_m])
    if roll_deg:
        a = np.radians(roll_deg)
        rx = np.array([[1.0, 0.0, 0.0], [0.0, np.cos(a), -np.sin(a)], [0.0, np.sin(a), np.cos(a)]])
        p.attitude_quat[...] = rotmat_to_quat(quat_to_rotmat(p.attitude_quat) @ rx)
    return p


def geometric(**kw) -> GeometricController:
    return GeometricController(GeometricGains(mass_kg=MASS), **kw)


def nmpc(**kw) -> NMPCController:
    return NMPCController(NMPCGains(mass_kg=MASS), **kw)


def time_to_within(trace, metres: float) -> float:
    inside = trace.position_error < metres
    # first time after which it stays inside
    last_out = np.nonzero(~inside)[0]
    idx = 0 if last_out.size == 0 else last_out[-1] + 1
    return float(trace.t[min(idx, len(trace.t) - 1)])


# A circle of radius 2 m, period 5 s, at 3 m altitude, nose fixed east.
RADIUS, PERIOD, ALT = 2.0, 5.0, 3.0
W = 2.0 * np.pi / PERIOD


def circle(t: float) -> FlatReference:
    c, s = np.cos(W * t), np.sin(W * t)
    return FlatReference(
        position=np.array([RADIUS * c, RADIUS * s, ALT]),
        velocity=np.array([-RADIUS * W * s, RADIUS * W * c, 0.0]),
        acceleration=np.array([-RADIUS * W * W * c, -RADIUS * W * W * s, 0.0]),
        jerk=np.array([RADIUS * W**3 * s, -RADIUS * W**3 * c, 0.0]),
        yaw=0.0,
    )


def circle_error(trace) -> np.ndarray:
    ref = np.array([circle(t).position for t in trace.t])
    return np.linalg.norm(trace.position - ref, axis=1)


# ---------------------------------------------------------------------------
# Geometric: pieces
# ---------------------------------------------------------------------------


def test_hat_and_vee_are_inverse_and_hat_is_the_cross_product():
    v, w = np.array([0.3, -1.2, 2.0]), np.array([1.0, 0.5, -0.4])
    np.testing.assert_allclose(vee(hat(v)), v)
    np.testing.assert_allclose(hat(v) @ w, np.cross(v, w))


@pytest.mark.parametrize("thrust_dir", [[0, 0, 1], [0.3, -0.2, 1.0], [1.0, 0.5, -0.8]])
def test_desired_attitude_is_a_rotation_with_the_thrust_axis_as_b3(thrust_dir):
    """Including a thrust direction below the horizon, which the cascade's
    construction refuses and a large-angle controller needs."""
    R = desired_attitude(np.array(thrust_dir, dtype=float), yaw=0.7)
    np.testing.assert_allclose(R.T @ R, np.eye(3), atol=1e-12)
    assert np.linalg.det(R) == pytest.approx(1.0)
    b3 = np.array(thrust_dir, dtype=float) / np.linalg.norm(thrust_dir)
    np.testing.assert_allclose(R[:, 2], b3, atol=1e-12)
    # b1 is the heading projected onto the plane normal to b3.
    heading = np.array([np.cos(0.7), np.sin(0.7), 0.0])
    proj = heading - (heading @ b3) * b3
    np.testing.assert_allclose(R[:, 0], proj / np.linalg.norm(proj), atol=1e-12)


def test_flatness_body_rates_match_the_derivative_of_the_desired_attitude():
    """All three rates against ``vee(R_d^T dR_d/dt)`` by central differences,
    on a trajectory with a turning heading. The yaw component is the one a
    textbook formula from a different attitude construction gets wrong."""

    def flat(t):
        acc = np.array([-2 * np.sin(t), -6 * np.sin(2 * t), -0.245 * np.sin(0.7 * t)])
        jerk = np.array([-2 * np.cos(t), -12 * np.cos(2 * t), -0.1715 * np.cos(0.7 * t)])
        return acc, jerk, 0.4 * t, 0.4

    def R_d(t):
        acc, _, yaw, _ = flat(t)
        f = acc + G * np.array([0.0, 0.0, 1.0])
        return desired_attitude(f, yaw), float(np.linalg.norm(f))

    h = 1e-5
    for t in (0.3, 1.1, 2.5, 4.0):
        R0, thrust = R_d(t)
        dR = (R_d(t + h)[0] - R_d(t - h)[0]) / (2 * h)
        _, jerk, yaw, yaw_rate = flat(t)
        np.testing.assert_allclose(flat_body_rates(R0, thrust, jerk, yaw, yaw_rate), vee(R0.T @ dR), atol=1e-6)


def test_hover_at_the_target_is_the_weight_and_no_moment():
    state = SimState(
        position=np.array([1.0, 2.0, 3.0]), velocity=np.zeros(3),
        attitude_quat=yaw_quat(0.4), body_rates=np.zeros(3),
    )
    out = geometric().compute(state, Waypoint(position=np.array([1.0, 2.0, 3.0])), {})
    assert out.wrench.force_body[2] == pytest.approx(MASS * G)
    np.testing.assert_allclose(out.wrench.moment_body, 0.0, atol=1e-12)
    np.testing.assert_allclose(out.accel_cmd, 0.0, atol=1e-12)


def test_gyroscopic_term_cancels_the_plants_coupling():
    """``M`` carries ``w x J w`` so that the closed loop is ``J w' = -k_R e_R -
    k_W e_W``; with the term switched off it is exactly that much smaller."""
    state = SimState(
        position=np.zeros(3), velocity=np.zeros(3),
        attitude_quat=yaw_quat(0.0), body_rates=np.array([1.0, -2.0, 3.0]),
    )
    on = GeometricController(GeometricGains(mass_kg=MASS)).compute(state, Waypoint(np.zeros(3)), {})
    off = GeometricController(GeometricGains(mass_kg=MASS, gyroscopic_compensation=False)).compute(
        state, Waypoint(np.zeros(3)), {}
    )
    J = np.diag([0.01, 0.01, 0.02])
    w = state.body_rates
    np.testing.assert_allclose(on.wrench.moment_body - off.wrench.moment_body, np.cross(w, J @ w))


def test_geometric_refuses_what_it_cannot_close_a_loop_on():
    flat_state = SimState(position=np.zeros(3), velocity=np.zeros(3))
    with pytest.raises(ValueError, match="attitude"):
        geometric().compute(flat_state, Waypoint(np.zeros(3)), {})
    with pytest.raises(ValueError, match="mass_kg"):
        GeometricGains.from_config({"controller": {"geometric": {}}})
    with pytest.raises(ValueError, match="unknown"):
        GeometricGains.from_config({"controller": {"geometric": {"mass_kg": 1.0, "k_p": 3}}})
    with pytest.raises(ValueError):
        GeometricGains(mass_kg=1.0, omega_att=0.0)


# ---------------------------------------------------------------------------
# Geometric: closed loop on the actuator plant
# ---------------------------------------------------------------------------


def test_geometric_flies_a_position_step():
    trace = fly(geometric(), airframe(), plant(), np.array([3.0, 3.0, 2.0]), steps=4000, dt=DT)
    assert trace.position_error[-1] < 0.01
    assert time_to_within(trace, 0.1) < 3.0
    assert trace.tilt_deg.max() < 45.0


def test_geometric_recovers_from_150_degrees_of_roll_and_beats_the_cascade():
    """The point of designing on SO(3) instead of around its linearisation."""
    target = np.array([0.0, 0.0, 10.0])
    geo = fly(geometric(), airframe(), plant(10.0, roll_deg=150.0), target, steps=4000, dt=DT)
    cas = fly(
        CascadeController(inner=PIDController(), gains=CascadeGains(mass_kg=MASS)),
        airframe(), plant(10.0, roll_deg=150.0), target, steps=4000, dt=DT,
    )
    assert geo.tilt_deg[0] == pytest.approx(150.0, abs=0.5)
    assert geo.position_error[-1] < 0.01
    assert geo.tilt_deg[-1] < 1.0
    geo_loss = 10.0 - geo.position[:, 2].min()
    cas_loss = 10.0 - cas.position[:, 2].min()
    assert geo_loss < 3.5
    assert geo_loss < 0.5 * cas_loss


def test_geometric_tracks_a_circle_with_feedforward():
    def position_only(t):
        return FlatReference(position=circle(t).position, yaw=0.0)

    full = fly(geometric(reference=circle), airframe(), plant(ALT, east_m=RADIUS), np.zeros(3), steps=6000, dt=DT)
    lagging = fly(geometric(reference=position_only), airframe(), plant(ALT, east_m=RADIUS), np.zeros(3), steps=6000, dt=DT)
    err_full = circle_error(full)[1000:]  # after the first 2 s
    err_lag = circle_error(lagging)[1000:]
    assert np.sqrt(np.mean(err_full**2)) < 0.1
    assert np.sqrt(np.mean(err_lag**2)) > 10.0 * np.sqrt(np.mean(err_full**2))


# ---------------------------------------------------------------------------
# NMPC: model and solver
# ---------------------------------------------------------------------------


def hover_state(yaw=0.0, position=(0.0, 0.0, 2.0)):
    return np.concatenate([position, np.zeros(3), yaw_quat(yaw), np.zeros(3)])


def test_hover_is_an_equilibrium_of_the_model():
    x = hover_state(0.8)
    u = np.array([G, 0.0, 0.0, 0.0])
    np.testing.assert_allclose(quad_dynamics(x, u), 0.0, atol=1e-12)


def test_rk4_keeps_the_quaternion_unit_and_integrates_a_yaw_rate():
    x = hover_state(0.0)
    x[10:13] = [0.0, 0.0, 0.5]  # already at the commanded rate
    u = np.array([G, 0.0, 0.0, 0.5])
    for _ in range(40):  # 2 s
        x = rk4_step(x, u, 0.05)
    assert np.linalg.norm(x[6:10]) == pytest.approx(1.0, abs=1e-12)
    np.testing.assert_allclose(x[6:10], yaw_quat(1.0), atol=1e-9)
    np.testing.assert_allclose(x[0:6], hover_state()[0:6], atol=1e-9)


def test_rate_states_follow_the_command_with_the_loops_time_constant():
    x = hover_state()
    u = np.array([G, 1.0, 0.0, 0.0])
    tau = 1.0 / 15.0
    y = rk4_step(x, u, tau / 10.0, rate_tau=tau)
    for _ in range(9):
        y = rk4_step(y, u, tau / 10.0, rate_tau=tau)
    assert y[10] == pytest.approx(1.0 - np.exp(-1.0), abs=1e-4)


def test_batched_linearisation_matches_central_differences():
    rng = np.random.default_rng(0)
    m = QuadrotorNMPC(horizon=3)
    X = np.tile(hover_state(0.3), (4, 1))
    X[:, 0:6] += rng.normal(scale=0.5, size=(4, 6))
    X[:, 10:13] += rng.normal(scale=0.3, size=(4, 3))
    U = np.tile([G, 0.2, -0.1, 0.05], (3, 1))
    A, B = m.linearize(X, U)
    h = 1e-6
    for k in range(3):
        for i in (0, 4, 7, 11):
            dx = np.zeros(13)
            dx[i] = h
            col = (rk4_step(X[k] + dx, U[k], m.dt, rate_tau=m.rate_tau) - rk4_step(X[k] - dx, U[k], m.dt, rate_tau=m.rate_tau)) / (2 * h)
            np.testing.assert_allclose(A[k][:, i], col, atol=1e-4)
        for j in range(4):
            du = np.zeros(4)
            du[j] = h
            col = (rk4_step(X[k], U[k] + du, m.dt, rate_tau=m.rate_tau) - rk4_step(X[k], U[k] - du, m.dt, rate_tau=m.rate_tau)) / (2 * h)
            np.testing.assert_allclose(B[k][:, j], col, atol=1e-4)


def test_nmpc_at_the_reference_plans_hover():
    m = QuadrotorNMPC()
    x0 = hover_state(1.2)
    sol = m.solve(x0, np.tile(x0, (m.horizon + 1, 1)))
    np.testing.assert_allclose(sol.u0, [G, 0.0, 0.0, 0.0], atol=1e-6)


def test_nmpc_plan_respects_bounds_and_roughly_the_tilt_limit():
    m = QuadrotorNMPC(max_tilt_deg=40.0)
    x0 = hover_state(0.0)
    ref = np.tile(x0, (m.horizon + 1, 1))
    ref[:, 0:3] = [6.0, -4.0, 3.0]
    sol = m.solve(x0, ref)
    assert np.all(sol.inputs >= m.u_lo - 1e-12) and np.all(sol.inputs <= m.u_hi + 1e-12)
    tilt = np.degrees(2.0 * np.arcsin(np.sqrt(np.clip(sol.states[:, 7] ** 2 + sol.states[:, 8] ** 2, 0, 1))))
    assert tilt.max() < 40.0 + 10.0  # a soft limit: small excess for a large error
    assert sol.cost < m.cost(m.rollout(x0, np.tile(m.u_hover, (m.horizon, 1))), np.tile(m.u_hover, (m.horizon, 1)), ref)


def test_nmpc_warm_start_needs_fewer_iterations():
    m = QuadrotorNMPC()
    x0 = hover_state()
    ref = np.tile(x0, (m.horizon + 1, 1))
    ref[:, 0:3] = [3.0, 3.0, 2.0]
    cold = m.solve(x0, ref)
    warm = m.solve(x0, ref)
    assert warm.iterations < cold.iterations


@pytest.mark.parametrize(
    "kwargs",
    [{"dt": 0.0}, {"horizon": 1}, {"thrust_max": 5.0}, {"rate_max_xy": 0.0}, {"rate_tau": 0.0}, {"max_tilt_deg": 200.0}],
)
def test_nmpc_rejects_bad_settings(kwargs):
    with pytest.raises(ValueError):
        QuadrotorNMPC(**kwargs)


def test_nmpc_gains_need_the_mass():
    with pytest.raises(ValueError, match="mass_kg"):
        NMPCGains.from_config({"controller": {"nmpc": {}}})
    assert NMPCGains.from_config({"controller": {"nmpc": {"mass_kg": 1.2, "horizon": 15}}}).horizon == 15


# ---------------------------------------------------------------------------
# NMPC: closed loop on the actuator plant
# ---------------------------------------------------------------------------


def test_nmpc_flies_a_position_step_faster_than_the_geometric_controller():
    target = np.array([3.0, 3.0, 2.0])
    mpc = fly(nmpc(), airframe(), plant(), target, steps=3000, dt=DT)
    geo = fly(geometric(), airframe(), plant(), target, steps=3000, dt=DT)
    assert mpc.position_error[-1] < 0.01
    assert time_to_within(mpc, 0.1) < time_to_within(geo, 0.1)
    assert mpc.tilt_deg.max() < 60.0  # planned at 45; the motors' lag is not in the model
    assert np.abs(mpc.position[:, 2] - 2.0).max() < 0.3


def test_nmpc_tracks_a_circle():
    trace = fly(nmpc(reference=circle), airframe(), plant(ALT, east_m=RADIUS), np.zeros(3), steps=5000, dt=DT)
    err = circle_error(trace)[1000:]
    assert np.sqrt(np.mean(err**2)) < 0.1


def test_nmpc_recovers_from_150_degrees_of_roll():
    trace = fly(nmpc(), airframe(), plant(10.0, roll_deg=150.0), np.array([0.0, 0.0, 10.0]), steps=4000, dt=DT)
    assert trace.position_error[-1] < 0.02
    assert trace.tilt_deg[-1] < 1.0
    assert 10.0 - trace.position[:, 2].min() < 6.0
