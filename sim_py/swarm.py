"""Formation flight for :class:`sim_py.backends.mujoco_swarm.MujocoSwarm`.

Each drone runs the same velocity law the single-vehicle simulator uses
(``MultirotorPilot.velocity_wrench``: velocity -> attitude -> rotor thrusts,
from its spec). On top, a position loop flies it to its slot in a formation,
and a short-range separation term pushes neighbours apart, so formation
changes do not run drones through each other. Slots are assigned to the
nearest free drone when the formation changes.

    sim = SwarmFormationSim(n=12)
    sim.takeoff(2.0)
    sim.fly_formation(circle_formation(12, radius_m=4.0, z_m=3.0), seconds=10.0)
    sim.land()
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from aerial_kit.controllers.cascade import CascadeGains
from aerial_kit.controllers.multirotor_manual import ManualLimits, MultirotorPilot
from aerial_kit.types import ControlTarget

from .backends.mujoco_swarm import MujocoSwarm, grid_positions


def circle_formation(n: int, radius_m: float, z_m: float, centre=(0.0, 0.0), phase_rad: float = 0.0) -> np.ndarray:
    a = phase_rad + 2.0 * math.pi * np.arange(n) / n
    return np.stack([centre[0] + radius_m * np.cos(a), centre[1] + radius_m * np.sin(a), np.full(n, z_m)], axis=1)


def grid_formation(n: int, spacing_m: float, z_m: float, centre=(0.0, 0.0)) -> np.ndarray:
    p = grid_positions(n, spacing_m, z_m)
    p[:, 0] += centre[0]
    p[:, 1] += centre[1]
    return p


def line_formation(n: int, spacing_m: float, z_m: float, heading_rad: float = 0.0) -> np.ndarray:
    s = (np.arange(n) - (n - 1) / 2.0) * spacing_m
    return np.stack([s * -math.sin(heading_rad), s * math.cos(heading_rad), np.full(n, z_m)], axis=1)


def assign_slots(positions: np.ndarray, slots: np.ndarray) -> np.ndarray:
    """Greedy nearest assignment: ``slots[order[i]]`` is drone ``i``'s slot."""
    n = len(positions)
    cost = np.linalg.norm(positions[:, None, :] - slots[None, :, :], axis=-1)
    order = np.full(n, -1)
    free_d, free_s = set(range(n)), set(range(n))
    for i, j in sorted(((i, j) for i in range(n) for j in range(n)), key=lambda ij: cost[ij]):
        if i in free_d and j in free_s:
            order[i] = j
            free_d.discard(i)
            free_s.discard(j)
    return order


@dataclass
class FormationGains:
    kp_pos: float = 1.0           # (m/s) per m of position error
    max_speed_mps: float = 2.0
    separation_m: float = 1.1     # neighbours closer than this get pushed apart
    k_sep: float = 4.0            # (m/s) per m of intrusion
    max_separation_speed_mps: float = 2.0


def formation_velocity(positions: np.ndarray, targets: np.ndarray, gains: FormationGains) -> np.ndarray:
    """World velocity per drone: P on the slot error plus short-range separation."""
    p = np.asarray(positions, dtype=float)
    n = len(p)
    goal = gains.kp_pos * (np.asarray(targets, dtype=float) - p)
    speed = np.linalg.norm(goal, axis=1, keepdims=True)
    goal *= np.minimum(1.0, gains.max_speed_mps / np.maximum(speed, 1e-9))
    # Separation is added after the goal's speed cap, so a drone cruising at
    # full speed still yields; it has its own cap.
    delta = p[:, None, :] - p[None, :, :]
    dist = np.linalg.norm(delta, axis=-1) + np.eye(n) * 1e9
    push = np.clip(gains.separation_m - dist, 0.0, None) / dist
    sep = gains.k_sep * np.einsum("ij,ijk->ik", push, delta)
    s_mag = np.linalg.norm(sep, axis=1, keepdims=True)
    sep *= np.minimum(1.0, gains.max_separation_speed_mps / np.maximum(s_mag, 1e-9))
    return goal + sep


def transit_targets(targets: np.ndarray, layer_m: float = 0.6, layers: int = 3) -> np.ndarray:
    """The same slots with drone ``i`` in altitude layer ``i % layers`` (centred).

    Fly these first, then the slots themselves: during a reshuffle crossing
    paths are mostly at different heights.
    """
    out = np.asarray(targets, dtype=float).copy()
    offsets = (np.arange(len(out)) % layers - (layers - 1) / 2.0) * layer_m
    out[:, 2] += offsets
    return out


FORMATIONS = ("grid", "circle", "line")


def formation_slots(name: str, n: int, z_m: float, spacing_m: float = 1.5, radius_m: float | None = None,
                    heading_rad: float = 0.0) -> np.ndarray:
    """Slots for a named formation: grid | circle | line."""
    if name == "grid":
        return grid_formation(n, spacing_m, z_m)
    if name == "circle":
        return circle_formation(n, radius_m if radius_m is not None else max(2.0, 0.5 * n), z_m)
    if name == "line":
        return line_formation(n, spacing_m, z_m, heading_rad)
    raise ValueError(f"unknown formation {name!r}; expected one of {FORMATIONS}")


class SwarmFormationSim:
    """A :class:`MujocoSwarm` with one velocity-controlled pilot per drone."""

    def __init__(self, n: int = 4, airframe: str = "swarm_quad", spacing_m: float = 1.5,
                 gains: FormationGains | None = None, rate_hz: float = 100.0, **swarm_kw) -> None:
        self.swarm = MujocoSwarm(n=n, airframe=airframe, spacing_m=spacing_m, **swarm_kw)
        v0 = self.swarm.vehicles[0]
        self.pilots = [MultirotorPilot(CascadeGains(**v0.controller_defaults("cascade")),
                                       ManualLimits.from_spec(self.swarm.spec)) for _ in range(self.swarm.n)]
        for p, s in zip(self.pilots, self.swarm.states()):
            p.reset_yaw(s)
        self.gains = gains or FormationGains()
        self.dt = 1.0 / float(rate_hz)
        self.min_separation_seen = math.inf
        self.contacts_seen: set[tuple[int, int]] = set()
        self.armed = [False] * self.swarm.n

    @property
    def n(self) -> int:
        return self.swarm.n

    def velocity_commands(self, targets: np.ndarray) -> np.ndarray:
        return formation_velocity(self.swarm.positions(), targets, self.gains)

    def step_velocity(self, v_world: np.ndarray) -> None:
        states = self.swarm.states()
        targets = []
        for i, (pilot, s) in enumerate(zip(self.pilots, states)):
            if self.armed[i]:
                targets.append(ControlTarget(accel_cmd=np.zeros(3),
                                             wrench=pilot.velocity_wrench(s, v_world[i], 0.0, self.dt)))
            else:
                targets.append(ControlTarget(accel_cmd=np.zeros(3),
                                             metadata={"rotor_thrust_n": np.zeros(self.swarm.spec.n_rotors)}))
        self.swarm.step(targets, self.dt)
        self.min_separation_seen = min(self.min_separation_seen, self.swarm.min_separation_m())
        self.contacts_seen.update(self.swarm.drone_contacts())

    def fly_to(self, targets: np.ndarray, seconds: float) -> None:
        for _ in range(int(round(seconds / self.dt))):
            self.step_velocity(self.velocity_commands(targets))

    def takeoff(self, z_m: float, seconds: float = 4.0) -> None:
        self.armed = [True] * self.n
        for p, s in zip(self.pilots, self.swarm.states()):
            p.reset_yaw(s)
        p = self.swarm.positions()
        self.fly_to(np.column_stack([p[:, :2], np.full(self.n, z_m)]), seconds)

    def fly_formation(self, slots: np.ndarray, seconds: float) -> np.ndarray:
        """Assign each drone its nearest free slot and fly there; returns the slot per drone."""
        order = assign_slots(self.swarm.positions(), np.asarray(slots, dtype=float))
        targets = np.asarray(slots, dtype=float)[order]
        layered = transit_targets(targets)
        # Climb/descend into the layers in place, cross, then settle into the slots.
        here = self.swarm.positions()
        self.fly_to(np.column_stack([here[:, :2], layered[:, 2]]), 0.2 * seconds)
        self.fly_to(layered, 0.55 * seconds)
        self.fly_to(targets, 0.25 * seconds)
        return targets

    def land(self, seconds: float = 6.0, descent_mps: float = 0.6) -> None:
        for _ in range(int(round(seconds / self.dt))):
            v = self.velocity_commands(self.swarm.positions())   # hold xy, keep apart
            v[:, 2] = -descent_mps
            self.step_velocity(v)
            for i, veh in enumerate(self.swarm.vehicles):
                if self.armed[i] and veh.on_ground():
                    self.armed[i] = False

    def formation_error_m(self, targets: np.ndarray) -> float:
        return float(np.linalg.norm(self.swarm.positions() - targets, axis=1).max())
