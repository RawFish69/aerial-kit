"""N multirotors in one shared MuJoCo scene.

Each vehicle is the single-airframe model from its spec, attached to one world
under the prefix ``d<i>_`` with MuJoCo's model-attach API. Every vehicle keeps
its own propulsion model (motor speed lag, battery sag, ground effect) and
drag through a :class:`MujocoMultirotorBackend` bound to its slice of the
shared ``MjData``. All vehicles advance together, one ``mj_step`` per substep,
so they share time, the ground, and each other: collision geometry is live
between drones as well as with the ground.

    swarm = MujocoSwarm(n=12)                       # swarm_quad, 4 x 3 grid
    swarm.step([ControlTarget(...)] * 12, 0.01)
    states = swarm.states()
"""

from __future__ import annotations

import colorsys
import math
from typing import Any, Mapping, Sequence

import numpy as np

from aerial_kit.airframes.multirotor_specs import load_multirotor_spec
from aerial_kit.airframes.multirotor_specs.mjcf import multirotor_mjcf
from aerial_kit.types import ControlTarget, SimState

from .mujoco_guard import step_guarded
from .mujoco_multirotor_backend import MujocoMultirotorBackend, _import_mujoco

#: Materials repainted per vehicle so each drone is recognisable in the viewer.
ACCENT_MATERIALS = ("front_arm", "prop_tip_front")


def grid_positions(n: int, spacing_m: float, z_m: float = 0.0, columns: int | None = None) -> np.ndarray:
    """Start positions on a near-square grid centred on the origin (ENU)."""
    if n < 1:
        raise ValueError("a swarm needs at least one vehicle")
    cols = columns or int(math.ceil(math.sqrt(n)))
    rows = int(math.ceil(n / cols))
    out = []
    for i in range(n):
        r, c = divmod(i, cols)
        out.append([(c - (cols - 1) / 2.0) * spacing_m, ((rows - 1) / 2.0 - r) * spacing_m, z_m])
    return np.array(out, dtype=float)


def vehicle_prefix(i: int) -> str:
    return f"d{i}_"


def swarm_model(mujoco, spec, positions: np.ndarray, timestep: float):
    """Compile one MjModel holding ``len(positions)`` copies of ``spec``."""
    world = mujoco.MjSpec.from_string(multirotor_mjcf(spec, timestep, ground=True))
    for actuator in list(world.actuators):        # keep sky, ground, light and options
        world.delete(actuator)
    world.delete(world.body("body"))
    world.modelname = f"{spec.name}_swarm_{len(positions)}"
    for i, pos in enumerate(positions):
        child = mujoco.MjSpec.from_string(multirotor_mjcf(spec, timestep, ground=False))
        frame = world.worldbody.add_frame(pos=[float(v) for v in pos])
        frame.attach_body(child.body("body"), vehicle_prefix(i), "")
    model = world.compile()
    for i in range(len(positions)):
        rgba = list(colorsys.hsv_to_rgb((i * 0.618034) % 1.0, 0.75, 0.95)) + [1.0]
        for name in ACCENT_MATERIALS:
            mid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_MATERIAL, vehicle_prefix(i) + name)
            if mid >= 0:
                model.mat_rgba[mid] = rgba
    return model


class MujocoSwarm:
    """A shared-scene swarm; vehicle ``i`` is ``swarm.vehicles[i]``."""

    def __init__(self, n: int = 4, airframe: str = "swarm_quad", spacing_m: float = 1.5,
                 positions: Sequence[Sequence[float]] | None = None, start_z: float | None = None,
                 timestep: float = 0.002, settings: Mapping[str, Any] | None = None) -> None:
        mujoco = _import_mujoco()
        self.mujoco = mujoco
        self.spec = load_multirotor_spec(airframe)
        on_ground = start_z is None
        if positions is None:
            positions = grid_positions(int(n), float(spacing_m))
        positions = np.asarray(positions, dtype=float).reshape(-1, 3)
        self.n = len(positions)
        self.timestep = float(timestep)
        self.model = swarm_model(mujoco, self.spec, np.zeros_like(positions), self.timestep)
        self.data = mujoco.MjData(self.model)
        base = {"airframe": airframe, "timestep": self.timestep,
                "start_motors": "idle" if on_ground else "hover", **dict(settings or {})}
        self.vehicles: list[MujocoMultirotorBackend] = []
        self.rest_z = self._rest_height()       # body height when resting on the gear
        rest_z = self.rest_z if on_ground else float(start_z)
        for i, pos in enumerate(positions):
            v = MujocoMultirotorBackend()
            v.configure(base)
            v.bind(mujoco, self.model, self.data, vehicle_prefix(i))
            z = rest_z + (float(pos[2]) if not on_ground else 0.0)
            v.init_state(SimState(position=np.array([pos[0], pos[1], z]), velocity=np.zeros(3), t=0.0,
                                  attitude_quat=np.array([1.0, 0.0, 0.0, 0.0]), body_rates=np.zeros(3)))
            self.vehicles.append(v)
        self.t = 0.0
        if on_ground:   # let the gear settle
            idle = [ControlTarget(accel_cmd=np.zeros(3), metadata={"rotor_thrust_n": np.zeros(self.spec.n_rotors)})]
            for _ in range(int(round(0.5 / 0.01))):
                self.step(idle * self.n, 0.01)

    def _rest_height(self) -> float:
        """Body-origin height at which the lowest collision geometry touches z = 0."""
        mujoco = self.mujoco
        probe = mujoco.MjModel.from_xml_string(multirotor_mjcf(self.spec, self.timestep, ground=False))
        d = mujoco.MjData(probe)
        mujoco.mj_forward(probe, d)
        low = 0.0
        for g in range(probe.ngeom):
            if probe.geom_contype[g] == 0 and probe.geom_conaffinity[g] == 0:
                continue
            low = min(low, float(d.geom_xpos[g][2]) - float(probe.geom_rbound[g]))
        return -low + 0.002

    def step(self, targets: Sequence[ControlTarget], dt: float) -> None:
        if len(targets) != self.n:
            raise ValueError(f"need {self.n} control targets, got {len(targets)}")
        dt = float(dt)
        if not math.isfinite(dt) or dt <= 0.0:
            raise ValueError(f"dt must be finite and positive, got {dt!r}")
        for v, target in zip(self.vehicles, targets):
            v._apply_thrusts(v.thrusts_for(target))
        n = max(1, int(math.ceil(dt / self.timestep - 1e-12)))
        self.model.opt.timestep = dt / n
        for _ in range(n):
            for v in self.vehicles:
                v.pre_substep(dt / n)
            step_guarded(self.mujoco, self.model, self.data)
        self.t += dt
        for v in self.vehicles:
            v._t = self.t

    def states(self) -> list[SimState]:
        return [v.state() for v in self.vehicles]

    def positions(self) -> np.ndarray:
        return np.array([s.position for s in self.states()])

    def min_separation_m(self) -> float:
        p = self.positions()
        if len(p) < 2:
            return math.inf
        d = np.linalg.norm(p[:, None, :] - p[None, :, :], axis=-1)
        return float(d[np.triu_indices(len(p), 1)].min())

    def drone_contacts(self) -> list[tuple[int, int]]:
        """Pairs of vehicles whose geometry is touching right now."""
        m, d = self.model, self.data
        owner = {}
        for i, v in enumerate(self.vehicles):
            owner[v._body_id] = i
        root_of = m.body_rootid
        pairs = set()
        for k in range(d.ncon):
            c = d.contact[k]
            a = owner.get(int(root_of[m.geom_bodyid[c.geom1]]))
            b = owner.get(int(root_of[m.geom_bodyid[c.geom2]]))
            if a is not None and b is not None and a != b:
                pairs.add((min(a, b), max(a, b)))
        return sorted(pairs)
