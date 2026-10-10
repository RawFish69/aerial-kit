"""Mission tracking with a predictive controller - the ROS-free half of the node.

Everything that decides *what* the aircraft should do lives here, and nothing
here imports rclpy, so the whole state machine is testable with plain pytest
(the same split ``air_unit.rtl`` uses). ``mpc_tracker_node`` only converts
messages to arrays and back.

How it flies a mission:

* The waypoint list is cut into **legs** that end at a *stop* - a waypoint with
  a hold time, or the last one. Within a leg the aircraft does not stop at the
  intermediate waypoints: a :class:`~aerial_kit.controllers.PathReference`
  marches along the leg at cruise speed and the controller tracks it, so it
  rounds corners instead of braking into each one. The reference decelerates
  into the stop at ``approach_decel_mps2``.
* Each tick the controller (:class:`~aerial_kit.controllers.ConstrainedMPC` or
  :class:`~aerial_kit.controllers.MPPI`) plans accelerations over the horizon.
  The backends this stack drives take **velocity** setpoints and close their
  own velocity loop, so the planned acceleration has to be turned into a
  setpoint *that loop* will turn back into that acceleration. For a
  first-order loop ``v' = (v_sp - v) / tau`` that is ``v_sp = v + a * tau``,
  with ``tau = velocity_loop_tau_s``, clamped to the xy/z speed limits. Get
  ``tau`` wrong by a lot and the aircraft only receives a fraction of every
  planned acceleration - which is how a first version of this, sending
  ``v + a * plan_dt`` (0.1 s) to sim_fast's 0.67 s loop, crept up to waypoints
  it never quite settled on.
* A stop is reached when the aircraft is inside its acceptance radius *and*
  slower than ``settle_speed_mps``. It then holds for ``hold_time_sec`` and
  moves to the next leg, or reports the mission complete.

Frames are the stack's: world ENU, z up, metres and seconds.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Optional, Sequence

import numpy as np

try:  # installed with pip, or importable from the source tree
    from aerial_kit.controllers import MPPI, ConstrainedMPC, PathReference, SphereObstacle
    from aerial_kit.controllers.mppi import BoxObstacle, CylinderObstacle
    from aerial_kit.controllers.reference import constant_reference
except ImportError:  # pragma: no cover - exercised only in a colcon install without aerial_kit
    from uav_algorithms.repo_paths import ensure_repo_root_on_path

    ensure_repo_root_on_path()
    from aerial_kit.controllers import MPPI, ConstrainedMPC, PathReference, SphereObstacle
    from aerial_kit.controllers.mppi import BoxObstacle, CylinderObstacle
    from aerial_kit.controllers.reference import constant_reference


STATE_IDLE = 0
STATE_ACTIVE = 1
STATE_PAUSED = 2
STATE_COMPLETE = 3

CONTROLLERS = ("mpc", "mppi")


@dataclass
class TrackedWaypoint:
    """One mission waypoint, as the tracker needs it."""

    position: np.ndarray
    acceptance_radius_m: float = 0.5
    hold_time_sec: float = 0.0
    desired_speed_mps: float = 0.0  # <= 0: use the tracker's cruise speed


@dataclass
class TrackerConfig:
    controller: str = "mpc"
    plan_dt: float = 0.1
    horizon: int = 20
    cruise_speed_mps: float = 2.0
    max_xy_speed_mps: float = 2.0
    max_z_speed_mps: float = 1.0
    max_accel_xy_mps2: float = 2.5
    max_accel_z_mps2: float = 1.5
    approach_decel_mps2: float = 1.0
    settle_speed_mps: float = 0.3
    velocity_loop_tau_s: float = 0.25  # time constant of the backend's velocity loop
    # MPC weights
    q_pos: float = 8.0
    q_vel: float = 1.0
    r_acc: float = 0.5
    r_delta: float = 2.0
    speed_limit_xy: str = "disc"  # MPC: bound |v_xy| (octagon), or "box" per axis
    # MPPI settings
    mppi_samples: int = 384
    mppi_temperature: float = 0.1  # relative to the batch's cost spread
    mppi_noise_std: float = 1.0
    mppi_seed: Optional[int] = 0
    obstacle_margin_m: float = 0.5
    # Obstacles and blockage. The path ahead is checked out to
    # blocked_lookahead_m; a point closer than blocked_clearance_m to an
    # obstacle's surface blocks it. MPPI flies around what it can and is only
    # called blocked when it stops making progress (stall_*); the MPC cannot
    # avoid anything, so for it a blocked path means stop and ask for a replan.
    blocked_lookahead_m: float = 8.0
    blocked_clearance_m: float = 0.0
    stall_time_s: float = 6.0
    stall_progress_m: float = 0.5
    min_altitude_m: Optional[float] = None

    def __post_init__(self) -> None:
        self.controller = str(self.controller).strip().lower()
        if self.controller not in CONTROLLERS:
            raise ValueError(f"controller must be one of {CONTROLLERS}, got {self.controller!r}")
        if self.max_xy_speed_mps <= 0.0 or self.max_z_speed_mps <= 0.0:
            raise ValueError("speed limits must be positive")
        if self.cruise_speed_mps <= 0.0:
            raise ValueError("cruise speed must be positive")
        self.horizon = max(int(self.horizon), 2)
        if self.velocity_loop_tau_s <= 0.0:
            raise ValueError("velocity_loop_tau_s must be positive")


@dataclass
class TrackerOutput:
    velocity_world: np.ndarray  # (3,) velocity setpoint, world frame
    state: int
    complete: bool
    active_index: int
    total_waypoints: int
    status_text: str
    predicted_positions: np.ndarray = field(default_factory=lambda: np.zeros((0, 3)))
    # Where the plan goes in xy over the horizon (end minus now). Heading is
    # steered by this, not by ``velocity_world``: at a standstill the first
    # setpoint is only ``a * dt``, and a forward-only vehicle that waited for
    # it to grow before turning would never turn - and never move.
    planned_displacement_xy: np.ndarray = field(default_factory=lambda: np.zeros(2))
    controller_info: dict = field(default_factory=dict)
    blocked: bool = False  # the follower cannot get on: a replan is needed
    path_blocked_ahead: bool = False  # the path itself runs through an obstacle
    blocked_reason: str = ""


def build_controller(cfg: TrackerConfig, obstacles: Sequence = ()):
    """The planner the tracker drives, built from its config."""
    if cfg.controller == "mpc":
        return ConstrainedMPC(
            dt=cfg.plan_dt,
            horizon=cfg.horizon,
            q_pos=cfg.q_pos,
            q_vel=cfg.q_vel,
            r_acc=cfg.r_acc,
            r_delta=cfg.r_delta,
            max_accel_xy=cfg.max_accel_xy_mps2,
            max_accel_z=cfg.max_accel_z_mps2,
            max_speed_xy=cfg.max_xy_speed_mps,
            max_speed_z=cfg.max_z_speed_mps,
            # The output is norm-clamped anyway; planning with the same bound
            # keeps the plan the controller follows the one it can fly.
            speed_limit_xy=cfg.speed_limit_xy,
        )
    return MPPI(
        dt=cfg.plan_dt,
        horizon=cfg.horizon,
        samples=cfg.mppi_samples,
        temperature=cfg.mppi_temperature,
        noise_std=cfg.mppi_noise_std,
        q_pos=cfg.q_pos,
        q_vel=cfg.q_vel,
        max_accel_xy=cfg.max_accel_xy_mps2,
        max_accel_z=cfg.max_accel_z_mps2,
        max_speed=math.hypot(cfg.max_xy_speed_mps, cfg.max_z_speed_mps),
        obstacles=obstacles,
        obstacle_margin=cfg.obstacle_margin_m,
        min_altitude=cfg.min_altitude_m,
        seed=cfg.mppi_seed,
    )


def clamp_velocity(v: np.ndarray, max_xy: float, max_z: float) -> np.ndarray:
    """Clamp the xy norm and |z| separately (direction in xy is preserved)."""
    out = np.asarray(v, dtype=float).copy()
    xy = math.hypot(out[0], out[1])
    if xy > max_xy:
        out[:2] *= max_xy / xy
    out[2] = min(max(out[2], -max_z), max_z)
    return out


def wrap_angle(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def heading_for_velocity(vx: float, vy: float, nose_axis: str = "+y") -> float:
    """Yaw that points the nose along ``(vx, vy)``.

    The Gazebo multicopter models in this repo fly nose-forward along body +Y
    (see ``mission_executor_node``), so at yaw ``psi`` the nose points to
    ``(-sin psi, cos psi)`` and the yaw that faces ``(vx, vy)`` is
    ``atan2(-vx, vy)``. A conventional body +X nose gives ``atan2(vy, vx)``.
    """
    if nose_axis == "+x":
        return math.atan2(vy, vx)
    if nose_axis == "+y":
        return math.atan2(-vx, vy)
    raise ValueError("nose_axis must be '+x' or '+y'")


def world_to_body(v: np.ndarray, yaw: float, nose_axis: str = "+y", lateral: bool = True) -> np.ndarray:
    """Rotate a world velocity into the body frame (yaw only; z unchanged).

    With ``lateral=False`` the sideways component is dropped and only the
    nose-axis component kept - the behaviour ``mission_executor_node`` uses for
    the Gazebo velocity controller, which tracks forward speed and yaw rate.
    """
    c, s = math.cos(yaw), math.sin(yaw)
    bx = c * v[0] + s * v[1]  # body +X in world: (cos, sin)
    by = -s * v[0] + c * v[1]  # body +Y in world: (-sin, cos)
    if not lateral:
        if nose_axis == "+y":
            bx = 0.0
        else:
            by = 0.0
    return np.array([bx, by, float(v[2])])


class TrackingCore:
    """Waypoint mission tracker around a predictive controller."""

    def __init__(self, cfg: TrackerConfig, obstacles: Sequence = ()) -> None:
        self.cfg = cfg
        # Fixed obstacles from configuration, plus a set replaced at run time
        # (the node's /terrain/obstacles subscription). The MPPI is handed
        # only the ones within reach of its horizon, per solve.
        self.static_obstacles = list(obstacles)
        self.dynamic_obstacles: list = []
        self.controller = build_controller(cfg)
        self.waypoints: list[TrackedWaypoint] = []
        self._legs: list[tuple[int, int]] = []  # (first waypoint index, stop index)
        self._leg = 0
        self._path: Optional[PathReference] = None
        self._waypoint_s: np.ndarray = np.zeros(0)
        self._s = 0.0
        self._active = 0
        self._hold_started: Optional[float] = None
        self._holding = False
        self.complete = False
        self._progress_mark: Optional[tuple[float, float]] = None  # (t, s)

    # -- obstacles ----------------------------------------------------------

    @property
    def obstacles(self) -> list:
        return self.static_obstacles + self.dynamic_obstacles

    def set_obstacles(self, obstacles: Sequence) -> None:
        """Replace the run-time obstacle set (anything with an ``sdf``)."""
        self.dynamic_obstacles = list(obstacles)

    def nearby_obstacles(self, position: np.ndarray) -> list:
        """Obstacles the plan could reach: within the horizon's travel, plus margin."""
        reach = (
            math.hypot(self.cfg.max_xy_speed_mps, self.cfg.max_z_speed_mps) * self.cfg.horizon * self.cfg.plan_dt
            + self.cfg.obstacle_margin_m
            + 1.0
        )
        p = np.asarray(position, dtype=float).reshape(1, 3)
        return [o for o in self.obstacles if float(o.sdf(p)[0]) < reach]

    def path_blocked_ahead(self) -> tuple[bool, Optional[np.ndarray]]:
        """Whether the current leg, from here to ``blocked_lookahead_m`` on, runs into an obstacle."""
        if self._path is None or not self.obstacles:
            return False, None
        s_end = min(self._s + self.cfg.blocked_lookahead_m, self._path.length)
        if s_end <= self._s:
            return False, None
        s_samples = np.linspace(self._s, s_end, max(int((s_end - self._s) / 0.25) + 1, 2))
        points, _ = self._path._interp(s_samples)
        for o in self.obstacles:
            hit = o.sdf(points) < self.cfg.blocked_clearance_m
            if np.any(hit):
                return True, points[int(np.argmax(hit))]
        return False, None

    # -- mission ------------------------------------------------------------

    @property
    def has_mission(self) -> bool:
        return bool(self.waypoints)

    @property
    def active_index(self) -> int:
        return self._active

    def load_mission(self, waypoints: Sequence[TrackedWaypoint], start_position: Optional[np.ndarray]) -> None:
        """Start a new mission. ``start_position`` anchors the first leg."""
        self.waypoints = [
            TrackedWaypoint(
                position=np.asarray(w.position, dtype=float).reshape(3),
                acceptance_radius_m=max(0.1, float(w.acceptance_radius_m)),
                hold_time_sec=max(0.0, float(w.hold_time_sec)),
                desired_speed_mps=float(w.desired_speed_mps),
            )
            for w in waypoints
        ]
        self._legs = []
        first = 0
        for i, w in enumerate(self.waypoints):
            if w.hold_time_sec > 0.0 or i == len(self.waypoints) - 1:
                self._legs.append((first, i))
                first = i + 1
        self._leg = 0
        self._active = 0
        self._hold_started = None
        self._holding = False
        self.complete = not self.waypoints
        self.controller.reset()
        if self.waypoints:
            self._start_leg(start_position)

    def clear(self) -> None:
        self.load_mission([], None)

    def pause(self) -> None:
        """Forget the controller's warm start (the plan is stale after a pause)."""
        self.controller.reset()

    def _start_leg(self, start_position: Optional[np.ndarray]) -> None:
        first, stop = self._legs[self._leg]
        pts = [w.position for w in self.waypoints[first:stop + 1]]
        anchor = None
        if start_position is not None:
            anchor = np.asarray(start_position, dtype=float).reshape(3)
        elif first > 0:
            anchor = self.waypoints[first - 1].position
        if anchor is not None:
            pts = [anchor] + pts
        self._path = PathReference(np.array(pts), decel_mps2=self.cfg.approach_decel_mps2)
        offset = 1 if anchor is not None else 0
        # Arc length of each waypoint of this leg along the leg's path.
        self._waypoint_s = np.array([self._path.project(p) for p in pts[offset:]])
        self._s = 0.0
        self._active = first
        self._hold_started = None
        self._holding = False
        self._progress_mark = None

    # -- control ------------------------------------------------------------

    def _cruise_speed(self) -> float:
        w = self.waypoints[self._active]
        speed = w.desired_speed_mps if w.desired_speed_mps > 0.0 else self.cfg.cruise_speed_mps
        return max(0.1, min(speed, math.hypot(self.cfg.max_xy_speed_mps, self.cfg.max_z_speed_mps)))

    def step(self, t: float, position: np.ndarray, velocity: np.ndarray) -> TrackerOutput:
        """Advance the mission and compute a world-frame velocity setpoint."""
        p = np.asarray(position, dtype=float).reshape(3)
        v = np.asarray(velocity, dtype=float).reshape(3)
        total = len(self.waypoints)
        if not self.waypoints:
            return self._idle("no mission")
        if self.complete:
            return TrackerOutput(np.zeros(3), STATE_COMPLETE, True, total, total, "mission complete")

        first, stop = self._legs[self._leg]
        stop_wp = self.waypoints[stop]
        dist_stop = float(np.linalg.norm(stop_wp.position - p))
        speed = float(np.linalg.norm(v))

        # Progress along the leg is monotonic: a path that doubles back must not
        # make the reference jump back to an earlier pass.
        self._s = self._path.project(p, s_min=self._s)
        for i in range(self._active, stop):
            w = self.waypoints[i]
            passed = self._s >= self._waypoint_s[i - first] - w.acceptance_radius_m
            near = np.linalg.norm(w.position - p) <= w.acceptance_radius_m
            if passed or near:
                self._active = i + 1
            else:
                break

        arrived = dist_stop <= stop_wp.acceptance_radius_m and speed <= self.cfg.settle_speed_mps
        if arrived or self._holding:
            self._active = stop
            if not self._holding:
                self._holding = True
                self._hold_started = float(t)
            held = float(t) - float(self._hold_started)
            if held >= stop_wp.hold_time_sec:
                if self._leg + 1 >= len(self._legs):
                    self.complete = True
                    self.controller.reset()
                    return TrackerOutput(np.zeros(3), STATE_COMPLETE, True, total, total, "mission complete")
                self._leg += 1
                self._start_leg(stop_wp.position)
                return self.step(t, p, v)
            reference = constant_reference(stop_wp.position, self.cfg.horizon)
            status = f"holding wp {stop + 1}/{total} ({held:.1f}/{stop_wp.hold_time_sec:.1f}s)"
            self._progress_mark = None
        else:
            reference = self._path.sample(
                self._s, cruise_mps=self._cruise_speed(), dt=self.cfg.plan_dt, horizon=self.cfg.horizon
            )
            status = f"tracking wp {self._active + 1}/{total} dist_to_stop={dist_stop:.2f}m"

        blocked_ahead, where = self.path_blocked_ahead() if not self._holding else (False, None)
        blocked, reason = False, ""
        if blocked_ahead and self.cfg.controller == "mpc":
            blocked = True
            reason = f"path blocked at ({where[0]:.1f}, {where[1]:.1f}, {where[2]:.1f})"
        elif not self._holding:
            # MPPI goes around what it can; it is blocked when it stops getting on.
            if self._progress_mark is None or self._s >= self._progress_mark[1] + self.cfg.stall_progress_m:
                self._progress_mark = (float(t), self._s)
            elif float(t) - self._progress_mark[0] > self.cfg.stall_time_s:
                blocked = True
                reason = f"no progress for {float(t) - self._progress_mark[0]:.1f}s"

        if blocked and self.cfg.controller == "mpc":
            # Stop short rather than fly the plan into the obstacle.
            self.controller.reset()
            return TrackerOutput(
                velocity_world=np.zeros(3), state=STATE_PAUSED, complete=False,
                active_index=self._active, total_waypoints=total,
                status_text=f"blocked: {reason}", blocked=True,
                path_blocked_ahead=blocked_ahead, blocked_reason=reason,
            )

        if self.cfg.controller == "mppi":
            solution = self.controller.solve(p, v, reference, obstacles=self.nearby_obstacles(p))
        else:
            solution = self.controller.solve(p, v, reference)
        # Invert the backend's velocity loop (see the module docstring).
        v_cmd = clamp_velocity(
            v + solution.accel * self.cfg.velocity_loop_tau_s,
            self.cfg.max_xy_speed_mps,
            self.cfg.max_z_speed_mps,
        )
        info = {"accel": solution.accel.copy()}
        if hasattr(solution, "converged"):
            info.update(converged=solution.converged, iterations=solution.iterations)
        if hasattr(solution, "effective_samples"):
            info.update(effective_samples=solution.effective_samples, min_cost=solution.min_cost)
        return TrackerOutput(
            velocity_world=v_cmd,
            state=STATE_ACTIVE,
            complete=False,
            active_index=self._active,
            total_waypoints=total,
            status_text=status,
            predicted_positions=solution.predicted_positions.copy(),
            planned_displacement_xy=(solution.predicted_positions[-1, :2] - p[:2]).copy(),
            controller_info=info,
            blocked=blocked,
            path_blocked_ahead=blocked_ahead,
            blocked_reason=reason,
        )

    def _idle(self, text: str) -> TrackerOutput:
        return TrackerOutput(np.zeros(3), STATE_IDLE, False, 0, len(self.waypoints), text)


def obstacles_from_markers(markers) -> list:
    """visualization_msgs Marker list -> obstacles.

    Reads what ``terrain_generator`` publishes: ``CYLINDER`` (scale = diameter,
    diameter, height; pose at mid-height) as trees, ``CUBE`` as axis-aligned
    boxes, ``SPHERE`` as spheres. Duck-typed on the message fields so it is
    testable without ROS. A ``DELETEALL`` marker starts a fresh set.
    """
    CUBE, SPHERE, CYLINDER = 1, 2, 3  # visualization_msgs/Marker type values
    DELETE, DELETEALL = 2, 3  # Marker action values
    out: list = []
    for m in markers:
        action = int(getattr(m, "action", 0))
        if action == DELETEALL:
            out = []
            continue
        if action == DELETE:
            continue
        x, y, z = float(m.pose.position.x), float(m.pose.position.y), float(m.pose.position.z)
        sx, sy, sz = float(m.scale.x), float(m.scale.y), float(m.scale.z)
        kind = int(m.type)
        if kind == CYLINDER:
            out.append(CylinderObstacle((x, y), 0.5 * max(sx, sy), z - 0.5 * sz, z + 0.5 * sz))
        elif kind == CUBE:
            out.append(BoxObstacle((x, y, z), (0.5 * sx, 0.5 * sy, 0.5 * sz)))
        elif kind == SPHERE:
            out.append(SphereObstacle((x, y, z), 0.5 * max(sx, sy, sz)))
    return out


def parse_sphere_obstacles(flat: Sequence[float]) -> list[SphereObstacle]:
    """``[x, y, z, r, x, y, z, r, ...]`` -> spheres (the ROS parameter format)."""
    values = [float(x) for x in flat]
    if len(values) % 4:
        raise ValueError("obstacle_spheres must be a flat list of x, y, z, radius quadruples")
    return [
        SphereObstacle((values[i], values[i + 1], values[i + 2]), values[i + 3])
        for i in range(0, len(values), 4)
    ]


__all__ = [
    "CONTROLLERS",
    "STATE_ACTIVE",
    "STATE_COMPLETE",
    "STATE_IDLE",
    "STATE_PAUSED",
    "TrackedWaypoint",
    "TrackerConfig",
    "TrackerOutput",
    "TrackingCore",
    "build_controller",
    "clamp_velocity",
    "heading_for_velocity",
    "obstacles_from_markers",
    "parse_sphere_obstacles",
    "world_to_body",
    "wrap_angle",
]
