"""TrackingCore against a velocity-controlled plant, no ROS required.

The backends this node drives (Gazebo's multicopter velocity controller, the
PX4/CRSF adapters) accept a velocity setpoint and close their own loop on it.
``VelocityPlant`` stands in for that: a first-order lag on velocity with an
acceleration limit, integrated finer than the control rate.
"""

import math

import numpy as np
import pytest

from uav_control.tracking_core import (
    STATE_ACTIVE,
    STATE_COMPLETE,
    STATE_IDLE,
    TrackedWaypoint,
    TrackerConfig,
    TrackingCore,
    clamp_velocity,
    heading_for_velocity,
    parse_sphere_obstacles,
    world_to_body,
    wrap_angle,
)


class VelocityPlant:
    def __init__(self, p0=(0.0, 0.0, 0.0), tau=0.25, max_accel=4.0):
        self.p = np.array(p0, dtype=float)
        self.v = np.zeros(3)
        self.tau = tau
        self.max_accel = max_accel

    def step(self, v_cmd, dt, substeps=10):
        h = dt / substeps
        for _ in range(substeps):
            a = (np.asarray(v_cmd) - self.v) / self.tau
            n = np.linalg.norm(a)
            if n > self.max_accel:
                a *= self.max_accel / n
            self.v = self.v + a * h
            self.p = self.p + self.v * h


def run_mission(core, waypoints, *, p0=(0.0, 0.0, 0.0), duration=60.0, rate_hz=20.0):
    plant = VelocityPlant(p0)
    core.load_mission(waypoints, plant.p.copy())
    dt = 1.0 / rate_hz
    log = {"p": [], "v": [], "cmd": [], "status": [], "out": []}
    t = 0.0
    while t < duration:
        out = core.step(t, plant.p.copy(), plant.v.copy())
        log["p"].append(plant.p.copy())
        log["v"].append(plant.v.copy())
        log["cmd"].append(out.velocity_world.copy())
        log["status"].append(out.status_text)
        log["out"].append(out)
        if out.complete:
            break
        plant.step(out.velocity_world, dt)
        t += dt
    for key in ("p", "v", "cmd"):
        log[key] = np.array(log[key])
    log["t"] = t
    return log


def wp(x, y, z, **kw):
    return TrackedWaypoint(position=np.array([x, y, z], dtype=float), **kw)


SQUARE = [wp(8, 0, 2, acceptance_radius_m=0.4), wp(8, 8, 2, acceptance_radius_m=0.4), wp(0, 8, 3, acceptance_radius_m=0.4)]


@pytest.mark.parametrize("controller", ["mpc", "mppi"])
def test_flies_a_mission_to_completion(controller):
    core = TrackingCore(TrackerConfig(controller=controller))
    log = run_mission(core, SQUARE, p0=(0.0, 0.0, 2.0))
    final = log["out"][-1]
    assert final.complete and final.state == STATE_COMPLETE
    assert np.linalg.norm(log["p"][-1] - SQUARE[-1].position) <= 0.4
    assert log["t"] < 40.0


@pytest.mark.parametrize("controller", ["mpc", "mppi"])
def test_setpoints_respect_the_speed_limits(controller):
    cfg = TrackerConfig(controller=controller, max_xy_speed_mps=1.5, max_z_speed_mps=0.5, cruise_speed_mps=3.0)
    log = run_mission(TrackingCore(cfg), SQUARE, p0=(0.0, 0.0, 0.0))
    xy = np.hypot(log["cmd"][:, 0], log["cmd"][:, 1])
    assert xy.max() <= 1.5 + 1e-9
    assert np.abs(log["cmd"][:, 2]).max() <= 0.5 + 1e-9


def test_intermediate_waypoints_are_flown_through_not_stopped_at():
    """The executor stops at every waypoint; the tracker keeps speed through
    waypoints without a hold, because they are on one leg of one path."""
    core = TrackingCore(TrackerConfig(controller="mpc", cruise_speed_mps=2.0))
    log = run_mission(core, SQUARE, p0=(0.0, 0.0, 2.0))
    corner = SQUARE[0].position
    near = np.linalg.norm(log["p"] - corner, axis=1) < 1.5
    assert near.any()
    assert np.linalg.norm(log["v"][near], axis=1).min() > 0.5


def test_hold_time_stops_and_waits_at_the_waypoint():
    mission = [wp(5, 0, 2, hold_time_sec=2.0, acceptance_radius_m=0.3), wp(5, 5, 2, acceptance_radius_m=0.3)]
    core = TrackingCore(TrackerConfig(controller="mpc"))
    log = run_mission(core, mission, p0=(0.0, 0.0, 2.0))
    assert log["out"][-1].complete
    holding = [i for i, s in enumerate(log["status"]) if s.startswith("holding wp 1/2")]
    assert holding, "never held at the first waypoint"
    held_s = len(holding) / 20.0
    assert held_s >= 2.0 - 0.1
    # While holding it stays inside the acceptance radius.
    held_p = log["p"][holding]
    assert np.linalg.norm(held_p - mission[0].position, axis=1).max() <= 0.3 + 0.05


def test_waypoint_desired_speed_caps_the_cruise_speed():
    mission = [wp(15, 0, 2, desired_speed_mps=0.8)]
    core = TrackingCore(TrackerConfig(controller="mpc", cruise_speed_mps=2.0))
    log = run_mission(core, mission, p0=(0.0, 0.0, 2.0))
    assert log["out"][-1].complete
    assert np.linalg.norm(log["v"], axis=1).max() < 0.8 * 1.25


def test_mppi_keeps_clear_of_an_obstacle_on_the_leg():
    sphere = parse_sphere_obstacles([6.0, 0.0, 2.0, 1.2])
    cfg = TrackerConfig(controller="mppi", obstacle_margin_m=0.6, horizon=25)
    core = TrackingCore(cfg, sphere)
    log = run_mission(core, [wp(12, 0, 2)], p0=(0.0, 0.0, 2.0))
    clearance = np.linalg.norm(log["p"] - np.array(sphere[0].center), axis=1)
    assert clearance.min() > sphere[0].radius
    assert log["out"][-1].complete


def test_a_path_that_doubles_back_still_completes():
    mission = [wp(8, 0, 2, acceptance_radius_m=0.4), wp(0, 0.5, 2, acceptance_radius_m=0.4)]
    core = TrackingCore(TrackerConfig(controller="mpc"))
    log = run_mission(core, mission, p0=(0.0, 0.0, 2.0))
    assert log["out"][-1].complete
    # It went out to the turn before coming back. Intermediate waypoints are
    # flown through, so a hairpin is cut by about a metre at cruise speed.
    assert log["p"][:, 0].max() > 7.0


def test_status_reports_progress_through_the_mission():
    core = TrackingCore(TrackerConfig(controller="mpc"))
    log = run_mission(core, SQUARE, p0=(0.0, 0.0, 2.0))
    indices = [o.active_index for o in log["out"]]
    assert indices == sorted(indices)  # never goes backwards
    assert {o.state for o in log["out"][:-1]} == {STATE_ACTIVE}
    assert all(o.total_waypoints == 3 for o in log["out"])
    assert log["out"][0].predicted_positions.shape == (core.cfg.horizon, 3)


def test_planned_displacement_points_along_the_leg_from_a_standstill():
    """The node steers heading by this, not by the setpoint: with a fast
    backend (small velocity_loop_tau_s) the setpoint from rest is tiny, but the
    plan already goes somewhere."""
    core = TrackingCore(TrackerConfig(controller="mpc", velocity_loop_tau_s=0.05))
    core.load_mission([wp(10, 0, 2)], np.array([0.0, 0.0, 2.0]))
    out = core.step(0.0, np.array([0.0, 0.0, 2.0]), np.zeros(3))
    assert np.linalg.norm(out.velocity_world[:2]) < 0.3
    d = out.planned_displacement_xy
    assert d[0] > 0.3 and abs(d[1]) < 1e-6


def test_after_completion_it_commands_zero():
    core = TrackingCore(TrackerConfig())
    run_mission(core, [wp(2, 0, 0)])
    out = core.step(100.0, np.array([2.0, 0.0, 0.0]), np.zeros(3))
    assert out.complete and not np.any(out.velocity_world)


def test_no_mission_is_idle_and_a_new_mission_replaces_the_old_one():
    core = TrackingCore(TrackerConfig())
    out = core.step(0.0, np.zeros(3), np.zeros(3))
    assert out.state == STATE_IDLE and not np.any(out.velocity_world)

    core.load_mission([wp(20, 0, 0)], np.zeros(3))
    core.step(0.0, np.zeros(3), np.zeros(3))
    core.load_mission([wp(0, -3, 0)], np.zeros(3))
    out = core.step(0.1, np.zeros(3), np.zeros(3))
    assert out.velocity_world[1] < 0.0 and abs(out.velocity_world[0]) < 1e-3

    core.clear()
    assert not core.has_mission
    assert core.step(0.2, np.zeros(3), np.zeros(3)).state == STATE_IDLE


def test_matching_the_backend_velocity_loop_is_what_lets_it_settle():
    """A slow velocity loop (sim_fast's is 0.67 s) and a tracker that assumes a
    fast one: each planned acceleration arrives at a fraction of its size and
    the aircraft creeps. Matching tau is the fix, not more gain."""
    mission = [wp(6, 0, 2, acceptance_radius_m=0.3), wp(6, 6, 2, acceptance_radius_m=0.3)]

    def fly(tau_assumed):
        core = TrackingCore(TrackerConfig(controller="mpc", velocity_loop_tau_s=tau_assumed))
        plant = VelocityPlant((0.0, 0.0, 2.0), tau=1.0 / 1.5, max_accel=2.0)
        core.load_mission(mission, plant.p.copy())
        t = 0.0
        while t < 30.0:
            out = core.step(t, plant.p.copy(), plant.v.copy())
            if out.complete:
                return t
            plant.step(out.velocity_world, 0.05)
            t += 0.05
        return None

    matched = fly(1.0 / 1.5)
    assert matched is not None and matched < 20.0
    mismatched = fly(0.1)
    assert mismatched is None or mismatched > 1.5 * matched


def test_bad_config_is_rejected():
    with pytest.raises(ValueError, match="controller"):
        TrackerConfig(controller="pid")
    with pytest.raises(ValueError):
        TrackerConfig(max_xy_speed_mps=0.0)
    with pytest.raises(ValueError):
        TrackerConfig(cruise_speed_mps=-1.0)
    with pytest.raises(ValueError):
        TrackerConfig(velocity_loop_tau_s=0.0)


# ---------------------------------------------------------------------------
# Helpers the node uses
# ---------------------------------------------------------------------------


def test_clamp_velocity_limits_xy_norm_and_z_separately():
    v = clamp_velocity(np.array([3.0, 4.0, -2.0]), max_xy=2.5, max_z=1.0)
    np.testing.assert_allclose(v, [1.5, 2.0, -1.0])
    np.testing.assert_allclose(clamp_velocity(np.array([0.1, 0.2, 0.3]), 1.0, 1.0), [0.1, 0.2, 0.3])


def test_heading_conventions():
    # Nose +Y: yaw 0 faces +Y; facing +X needs yaw -90 deg.
    assert heading_for_velocity(0.0, 1.0, "+y") == pytest.approx(0.0)
    assert heading_for_velocity(1.0, 0.0, "+y") == pytest.approx(-math.pi / 2)
    # Nose +X: the usual atan2.
    assert heading_for_velocity(0.0, 1.0, "+x") == pytest.approx(math.pi / 2)
    with pytest.raises(ValueError):
        heading_for_velocity(1.0, 0.0, "-z")
    assert wrap_angle(3 * math.pi) == pytest.approx(math.pi)


def test_world_to_body_rotates_and_optionally_drops_lateral():
    v = np.array([1.0, 0.0, 0.5])
    yaw = heading_for_velocity(1.0, 0.0, "+y")  # nose pointed along +X world
    body = world_to_body(v, yaw, "+y", lateral=True)
    np.testing.assert_allclose(body, [0.0, 1.0, 0.5], atol=1e-12)  # all forward

    # Nose along world +X, so body +X (right) is world -Y.
    right = world_to_body(np.array([0.0, -1.0, 0.0]), yaw, "+y", lateral=True)
    np.testing.assert_allclose(right, [1.0, 0.0, 0.0], atol=1e-12)
    dropped = world_to_body(np.array([0.0, 1.0, 0.0]), yaw, "+y", lateral=False)
    np.testing.assert_allclose(dropped, [0.0, 0.0, 0.0], atol=1e-12)


def test_parse_sphere_obstacles():
    spheres = parse_sphere_obstacles([1, 2, 3, 0.5, 4, 5, 6, 1.0])
    assert [s.center for s in spheres] == [(1.0, 2.0, 3.0), (4.0, 5.0, 6.0)]
    assert [s.radius for s in spheres] == [0.5, 1.0]
    with pytest.raises(ValueError):
        parse_sphere_obstacles([1, 2, 3])
