"""Shared multirotor airframe descriptions: one YAML per airframe.

Bundled airframes (``available_airframes()``):

    fpv_quad_5in     5-inch FPV freestyle quad, stretched-X carbon frame
    research_hex     ~3.2 kg research hexacopter, round hub, landing gear
    research_octo    ~5.5 kg research octocopter
    micro_lr_quad    3-inch micro long-range quad built from the lr_drone CAD

Frame: body **FLU, z up** (x forward, y left, z up), origin at the composite
CG. That is the frame of :class:`~aerial_kit.types.SimState` for multirotors,
of :class:`~aerial_kit.types.Wrench`, of the cascade controller and of
REP-103 URDFs. (The firmware's actuator plant is FRD; the seam in
``aerial_kit.dynamics.quad_x_seam`` owns that conversion, not this module.)

Rotor ``spin`` is the rotation seen from above: ``ccw`` or ``cw``. A rotor
spinning CCW reacts on the body with a *negative* (CW) yaw torque of
``yaw_torque_coeff_m * thrust``. Every bundled quad uses Betaflight's default
"props-in" pattern, which is also the firmware's quad-X spin table.

Every number in the bundled YAMLs is a research default (or, for the micro
quad, read from its CAD export), not a bench measurement.
"""

from __future__ import annotations

import copy
import math
from dataclasses import dataclass
from functools import lru_cache
from pathlib import Path
from typing import Any, Mapping
from xml.etree import ElementTree

import numpy as np

SPEC_DIR = Path(__file__).resolve().parent
REPO_ROOT = SPEC_DIR.parents[2]

#: CAD (lr_drone) axes are +y forward, +x right, z up. Body FLU = R @ cad.
CAD_TO_FLU = np.array([[0.0, 1.0, 0.0], [-1.0, 0.0, 0.0], [0.0, 0.0, 1.0]])


def available_airframes() -> list[str]:
    return sorted(p.stem for p in SPEC_DIR.glob("*.yaml"))


def _load_yaml(path: Path) -> dict[str, Any]:
    try:
        import yaml
    except Exception as exc:  # pragma: no cover - optional dependency
        raise RuntimeError(
            "PyYAML is required to read multirotor airframe specs "
            "(pip install pyyaml, or the 'sim' extra).") from exc
    with open(path, "r", encoding="utf-8") as handle:
        data = yaml.safe_load(handle)
    if not isinstance(data, dict):
        raise ValueError(f"{path}: expected a mapping at the top level")
    return data


def _spin_sign(spin: str) -> float:
    """+1 for CCW seen from above (angular momentum along body +z)."""
    s = str(spin).lower()
    if s == "ccw":
        return 1.0
    if s == "cw":
        return -1.0
    raise ValueError(f"rotor spin must be 'cw' or 'ccw', got {spin!r}")


def _parallel(mass: float, d: np.ndarray) -> np.ndarray:
    d = np.asarray(d, dtype=float)
    return mass * (float(d @ d) * np.eye(3) - np.outer(d, d))


@dataclass(frozen=True)
class RotorSpec:
    name: str
    position_m: np.ndarray      # hub centre, body FLU, about the CG
    spin: float                 # +1 CCW from above, -1 CW
    max_thrust_n: float
    motor_tau_s: float
    yaw_torque_coeff_m: float

    @property
    def body_yaw_torque_per_n(self) -> float:
        """Body yaw torque (about +z) per newton of this rotor's thrust."""
        return -self.spin * self.yaw_torque_coeff_m


@dataclass(frozen=True)
class BodyMass:
    name: str
    mass_kg: float
    com_m: np.ndarray           # body FLU, about the composite CG
    inertia_kg_m2: np.ndarray   # 3x3 about its own COM, body axes


class BatteryModel:
    """Lumped Li-ion pack: open-circuit voltage from state of charge, IR sag.

    ``V = n_cells * (v_empty + (v_full - v_empty) * soc) - I * R``. Electrical
    power is momentum-theory rotor power ``T^1.5 / sqrt(2 rho A)`` divided by
    the figure of merit and the drive (ESC + motor) efficiency, plus a
    constant avionics load. A research-grade lumped model: no thermal
    effects, no Peukert, linear OCV.
    """

    def __init__(self, cfg: Mapping[str, Any], disk_area_m2: float, rho: float = 1.225):
        self.cells = int(cfg["cells"])
        self.capacity_c = float(cfg["capacity_mah"]) * 3.6        # coulombs
        self.r_ohm = float(cfg["internal_resistance_ohm"])
        self.v_full = float(cfg.get("cell_full_v", 4.2))
        self.v_empty = float(cfg.get("cell_empty_v", 3.5))
        self.fom = float(cfg.get("figure_of_merit", 0.6))
        self.eta = float(cfg.get("drive_efficiency", 0.8))
        self.avionics_w = float(cfg.get("avionics_w", 3.0))
        self.k_power = 1.0 / math.sqrt(2.0 * rho * disk_area_m2)
        if self.cells < 1 or self.capacity_c <= 0.0 or self.r_ohm < 0.0:
            raise ValueError("battery needs cells >= 1, capacity > 0, resistance >= 0")
        self.reset(float(cfg.get("initial_soc", 1.0)))

    @property
    def full_voltage(self) -> float:
        return self.cells * self.v_full

    def open_circuit(self) -> float:
        return self.cells * (self.v_empty + (self.v_full - self.v_empty) * self.soc)

    def reset(self, soc: float = 1.0) -> None:
        self.soc = float(np.clip(soc, 0.0, 1.0))
        self.current = 0.0
        self.voltage = self.open_circuit()

    def power_w(self, thrusts) -> float:
        t = np.clip(np.asarray(thrusts, dtype=float), 0.0, None)
        return float(np.sum(t ** 1.5) * self.k_power / (self.fom * self.eta) + self.avionics_w)

    def step(self, thrusts, dt: float) -> float:
        """Advance by dt at these rotor thrusts; returns the loaded voltage."""
        p = self.power_w(thrusts)
        voc = self.open_circuit()
        # V = Voc - R P / V  ->  V^2 - Voc V + R P = 0 (upper root).
        disc = voc * voc - 4.0 * self.r_ohm * p
        v = 0.5 * (voc + math.sqrt(max(disc, 0.0)))
        self.voltage = max(v, 0.5 * voc)
        self.current = p / self.voltage
        self.soc = max(0.0, self.soc - self.current * dt / self.capacity_c)
        return self.voltage


def ground_effect_factor(height_m: float, rotor_radius_m: float) -> float:
    """Cheeseman-Bennett thrust ratio T_IGE / T_OGE, clamped below h = R/2."""
    h = max(float(height_m), 0.5 * rotor_radius_m)
    return 1.0 / (1.0 - (rotor_radius_m / (4.0 * h)) ** 2)


class MultirotorAllocator:
    """Collective thrust + body moments -> per-rotor thrust, from real geometry.

    The mixer is built from the rotor positions and spins in the spec, so an
    asymmetric frame (the micro quad's CG is not on its rotor centroid) is
    allocated correctly rather than as an ideal polygon.

    Saturation keeps the collective and scales the moment demand down (a
    bisection on one factor) until every rotor is inside ``[min, max]``. If
    even zero moment is infeasible, thrust is clipped. Roll/pitch/yaw share the
    one factor, so the direction of the moment demand is preserved.
    """

    def __init__(self, rotors: list[RotorSpec], min_thrust_frac: float = 0.0):
        self.rotors = rotors
        n = len(rotors)
        mixer = np.zeros((4, n))
        for i, r in enumerate(rotors):
            x, y, _ = r.position_m
            mixer[0, i] = 1.0
            mixer[1, i] = y           # roll  Mx = sum(y T)   (z-up thrust)
            mixer[2, i] = -x          # pitch My = -sum(x T)
            mixer[3, i] = r.body_yaw_torque_per_n
        self.mixer = mixer
        self.mixer_pinv = np.linalg.pinv(mixer)
        self.t_max = np.array([r.max_thrust_n for r in rotors])
        self.t_min = self.t_max * float(min_thrust_frac)

    def wrench_of(self, thrusts: np.ndarray) -> np.ndarray:
        """[collective, Mx, My, Mz] produced by per-rotor thrusts."""
        return self.mixer @ np.asarray(thrusts, dtype=float)

    def allocate(self, collective_n: float, moment_body) -> tuple[np.ndarray, float]:
        """Per-rotor thrust and the moment scale actually achieved (1 = full)."""
        moment = np.asarray(moment_body, dtype=float).reshape(3)
        base = self.mixer_pinv @ np.array([float(collective_n), 0.0, 0.0, 0.0])
        delta = self.mixer_pinv @ np.concatenate([[0.0], moment])

        def feasible(scale: float) -> bool:
            t = base + scale * delta
            return bool(np.all(t >= self.t_min - 1e-9) and np.all(t <= self.t_max + 1e-9))

        if feasible(1.0):
            return base + delta, 1.0
        lo, hi = 0.0, 1.0
        if not feasible(0.0):
            return np.clip(base, self.t_min, self.t_max), 0.0
        for _ in range(30):
            mid = 0.5 * (lo + hi)
            if feasible(mid):
                lo = mid
            else:
                hi = mid
        return np.clip(base + lo * delta, self.t_min, self.t_max), lo


class MultirotorSpec:
    """Typed view over one multirotor YAML plus everything derived from it."""

    def __init__(self, data: Mapping[str, Any], source: str = "<dict>") -> None:
        self.data = copy.deepcopy(dict(data))
        self.source = source
        self.name = str(self.data.get("name", Path(source).stem))
        self._cad = None
        if self.style == "cad_meshes":
            self._cad = _CadAirframe(self.data["cad"], self.data.get("payloads", []))
        self._validate()
        self.rotors = self._build_rotors()
        self.allocator = MultirotorAllocator(
            self.rotors, float(self.propulsion.get("min_thrust_frac", 0.0)))

    # ------------------------------------------------------------------ io
    @classmethod
    def load(cls, name_or_path: str | Path) -> "MultirotorSpec":
        candidate = Path(str(name_or_path)).expanduser()
        if candidate.suffix in (".yaml", ".yml") and candidate.is_file():
            return cls(_load_yaml(candidate), source=str(candidate))
        bundled = SPEC_DIR / f"{name_or_path}.yaml"
        if bundled.is_file():
            return cls(_load_yaml(bundled), source=str(bundled))
        raise ValueError(
            f"unknown multirotor airframe {name_or_path!r}; bundled: "
            f"{', '.join(available_airframes())} (or pass a .yaml path)")

    # ------------------------------------------------------------ sections
    @property
    def role(self) -> str:
        return str(self.data.get("role", "research"))

    @property
    def style(self) -> str:
        return str(self.data.get("style", "research_hub"))

    @property
    def propulsion(self) -> Mapping[str, Any]:
        return self.data["propulsion"]

    @property
    def prop(self) -> Mapping[str, Any]:
        return self.data["prop"]

    @property
    def rotor_cfg(self) -> Mapping[str, Any]:
        return self.data["rotor"]

    @property
    def control(self) -> Mapping[str, Any]:
        return self.data["control"]

    @property
    def geometry(self) -> Mapping[str, Any]:
        return self.data.get("geometry", {}) or {}

    @property
    def materials(self) -> Mapping[str, list]:
        return self.data["materials"]

    @property
    def collision(self) -> Mapping[str, Any]:
        return self.data["collision"]

    @property
    def camera(self) -> Mapping[str, Any] | None:
        return self.data.get("camera")

    # ---------------------------------------------------------- validation
    def _validate(self) -> None:
        for key in ("propulsion", "prop", "rotor", "control", "materials", "collision", "aero"):
            if key not in self.data:
                raise ValueError(f"{self.source}: missing section {key!r}")
        if self.style not in ("fpv_x", "research_hub", "cad_meshes"):
            raise ValueError(f"{self.source}: unknown style {self.style!r}")
        if self._cad is None:
            mp = self.data.get("mass_properties", {})
            if float(mp.get("total_mass_kg", 0.0)) <= 0.0:
                raise ValueError(f"{self.source}: total_mass_kg must be positive")
            if len(self.data.get("rotors", [])) < 3:
                raise ValueError(f"{self.source}: need at least 3 rotors")
        p = self.propulsion
        for key in ("max_thrust_n", "motor_tau_s", "yaw_torque_coeff_m"):
            if float(p[key]) <= 0.0:
                raise ValueError(f"{self.source}: propulsion.{key} must be positive")
        for name, rgba in self.materials.items():
            if len(rgba) != 4:
                raise ValueError(f"material {name!r} must be rgba")

    def _build_rotors(self) -> list[RotorSpec]:
        p = self.propulsion
        raw = self._cad.rotors() if self._cad is not None else self.data["rotors"]
        cg = self._raw_cg()
        rotors = []
        for r in raw:
            rotors.append(RotorSpec(
                name=str(r["name"]),
                position_m=np.asarray(r["position_m"], dtype=float) - cg,
                spin=_spin_sign(r["spin"]),
                max_thrust_n=float(p["max_thrust_n"]),
                motor_tau_s=float(p["motor_tau_s"]),
                yaw_torque_coeff_m=float(p["yaw_torque_coeff_m"]),
            ))
        names = [r.name for r in rotors]
        if len(set(names)) != len(names):
            raise ValueError(f"{self.source}: rotor names must be unique")
        if abs(sum(r.spin for r in rotors)) > 1e-9:
            raise ValueError(f"{self.source}: rotor spins must balance (equal CW and CCW)")
        return rotors

    def _raw_cg(self) -> np.ndarray:
        """CG in the YAML's own coordinates (zero for procedural specs)."""
        if self._cad is not None:
            return self._cad.raw_cg()
        return np.asarray(self.data.get("mass_properties", {}).get("cg_m", [0, 0, 0]), dtype=float)

    # ------------------------------------------------------- mass and size
    @property
    def n_rotors(self) -> int:
        return len(self.rotors)

    @property
    def total_mass_kg(self) -> float:
        if self._cad is not None:
            return self._cad.total_mass()
        return float(self.data["mass_properties"]["total_mass_kg"])

    def rotor_masses(self) -> list[BodyMass]:
        if self._cad is not None:
            return self._cad.rotor_masses(self._raw_cg(), self.rotors)
        r = self.rotor_cfg
        m = float(r["mass_kg"])
        axial = self.rotor_axial_inertia()
        transverse = 0.5 * axial + 1e-9
        return [BodyMass(f"{rot.name}_rotor", m, rot.position_m.copy(),
                         np.diag([transverse, transverse, axial])) for rot in self.rotors]

    def root_mass(self) -> BodyMass:
        """Root-body mass properties so the composite matches the spec."""
        if self._cad is not None:
            return self._cad.root_mass(self._raw_cg())
        rotors = self.rotor_masses()
        m_rot = sum(r.mass_kg for r in rotors)
        m_root = self.total_mass_kg - m_rot
        if m_root <= 0.0:
            raise ValueError(f"{self.source}: rotor masses exceed the total mass")
        com_root = -sum(r.mass_kg * r.com_m for r in rotors) / m_root
        inertia = np.diag(np.asarray(self.data["mass_properties"]["inertia_kg_m2"], dtype=float))
        for r in rotors:
            inertia = inertia - r.inertia_kg_m2 - _parallel(r.mass_kg, r.com_m)
        inertia = inertia - _parallel(m_root, com_root)
        inertia = 0.5 * (inertia + inertia.T)
        eig = np.linalg.eigvalsh(inertia)
        if eig.min() <= 0.0 or eig.max() > eig.sum() - eig.max() + 1e-12:
            raise ValueError(f"{self.source}: root inertia is not physical after removing rotors: {eig}")
        return BodyMass("body", m_root, com_root, inertia)

    def payload_masses(self) -> list[BodyMass]:
        if self._cad is not None:
            return self._cad.payload_masses(self._raw_cg())
        return []

    def composite_inertia(self) -> np.ndarray:
        """Composite inertia about the CG (body axes), all bodies included."""
        total = np.zeros((3, 3))
        for b in [self.root_mass()] + self.rotor_masses() + self.payload_masses():
            total += b.inertia_kg_m2 + _parallel(b.mass_kg, b.com_m)
        return total

    @property
    def inertia_diag(self) -> np.ndarray:
        return np.diag(self.composite_inertia()).copy()

    def rotor_axial_inertia(self) -> float:
        """Simulated shaft inertia.

        With ``preserve_angular_momentum`` the inertia is scaled by
        physical/visual speed so the gyroscopic coupling of the slow display
        shaft matches the real rotor (the twin wing does this). Multirotor
        specs default to the real inertia instead: scaling by 25-35x would
        demand an equally inflated transverse inertia (triangle inequality)
        and distort roll/pitch of a light airframe far more than it helps.
        """
        r = self.rotor_cfg
        inertia = float(r["physical_inertia_kg_m2"])
        if bool(r.get("preserve_angular_momentum", False)):
            inertia *= float(r["physical_max_speed_rad_s"]) / float(r["visual_max_speed_rad_s"])
        return inertia

    @property
    def prop_radius_m(self) -> float:
        return 0.5 * float(self.prop["diameter_m"])

    def arm_radius_m(self) -> float:
        return float(np.mean([np.linalg.norm(r.position_m[:2]) for r in self.rotors]))

    # ------------------------------------------------------------ forces
    @property
    def propulsion_model(self) -> str:
        model = str(self.propulsion.get("model", "lagged_thrust"))
        if model not in ("lagged_thrust", "rpm"):
            raise ValueError(f"propulsion.model must be 'lagged_thrust' or 'rpm', got {model!r}")
        return model

    @property
    def ground_effect(self) -> bool:
        return bool(self.propulsion.get("ground_effect", False))

    def battery_model(self) -> BatteryModel | None:
        cfg = self.data.get("battery")
        if not cfg:
            return None
        area = math.pi * self.prop_radius_m ** 2
        return BatteryModel(cfg, area)

    @property
    def max_thrust_n(self) -> float:
        return float(self.propulsion["max_thrust_n"])

    @property
    def gravity_mps2(self) -> float:
        return float(self.data.get("gravity_mps2", 9.81))

    def hover_thrusts(self) -> np.ndarray:
        t, scale = self.allocator.allocate(self.total_mass_kg * self.gravity_mps2, np.zeros(3))
        return t

    def thrust_to_weight(self) -> float:
        return self.n_rotors * self.max_thrust_n / (self.total_mass_kg * self.gravity_mps2)

    def linear_drag(self) -> np.ndarray:
        drag = self.data["aero"]["linear_drag_n_per_mps"]
        drag = np.asarray(drag, dtype=float)
        return np.full(3, float(drag)) if drag.ndim == 0 else drag.reshape(3)

    def shaft_speed_for_thrust(self, thrust_n: float, rotor: RotorSpec) -> float:
        frac = min(max(float(thrust_n) / rotor.max_thrust_n, 0.0), 1.0)
        return rotor.spin * float(self.rotor_cfg["visual_max_speed_rad_s"]) * math.sqrt(frac)

    def clamp_rotor_thrusts(self, thrusts) -> tuple[np.ndarray, bool]:
        t = np.asarray(thrusts, dtype=float)
        if t.shape != (self.n_rotors,):
            raise ValueError(f"expected {self.n_rotors} rotor thrusts, got shape {t.shape}")
        if not np.all(np.isfinite(t)):
            raise ValueError("rotor thrust command contains a non-finite value")
        out = np.clip(t, 0.0, self.allocator.t_max)
        return out, bool(np.any(out != t))

    def cascade_gains(self, **overrides):
        from aerial_kit.controllers.cascade import CascadeGains

        c = self.control
        values = dict(
            mass_kg=self.total_mass_kg,
            inertia_kgm2=tuple(float(v) for v in self.inertia_diag),
            k_att=float(c["k_att"]),
            k_rate=float(c["k_rate"]),
            max_tilt_deg=float(c["max_tilt_deg"]),
            gravity_mps2=self.gravity_mps2,
        )
        values.update(overrides)
        return CascadeGains(**values)


# ----------------------------------------------------------------- CAD import
class _CadAirframe:
    """Reads mass, inertia and rotor placement from a URDF + alignment YAML.

    The CAD files stay the source of truth for the micro quad; the spec only
    adds what CAD does not know (thrust, motor lag, spin by mesh name, an
    optional battery payload).
    """

    def __init__(self, cad: Mapping[str, Any], payloads):
        self.cad = dict(cad)
        self.urdf_path = (REPO_ROOT / str(cad["urdf"])).resolve()
        self.alignment_path = (REPO_ROOT / str(cad["alignment"])).resolve()
        self.mesh_dir = (REPO_ROOT / str(cad["mesh_dir"])).resolve()
        self.mesh_scale = float(cad.get("mesh_scale", 0.001))
        root = ElementTree.parse(self.urdf_path).getroot()
        self.links: dict[str, tuple[float, np.ndarray, np.ndarray]] = {}
        for link in root.findall("link"):
            inertial = link.find("inertial")
            if inertial is None:
                continue
            origin = inertial.find("origin")
            com = np.array([float(v) for v in origin.attrib["xyz"].split()]) if origin is not None else np.zeros(3)
            mass = float(inertial.find("mass").attrib["value"])
            i = inertial.find("inertia").attrib
            tensor = np.array([
                [float(i["ixx"]), float(i["ixy"]), float(i["ixz"])],
                [float(i["ixy"]), float(i["iyy"]), float(i["iyz"])],
                [float(i["ixz"]), float(i["iyz"]), float(i["izz"])],
            ])
            self.links[link.attrib["name"]] = (mass, com, tensor)
        self.joints = {}
        for joint in root.findall("joint"):
            if joint.attrib.get("type") != "continuous":
                continue
            child = joint.find("child").attrib["link"]
            xyz = np.array([float(v) for v in joint.find("origin").attrib["xyz"].split()])
            self.joints[joint.attrib["name"]] = (child, xyz)
        alignment = _load_yaml(self.alignment_path)
        self.alignment = alignment["rotors"]
        self.frame_link = str(cad.get("frame_link", "frame"))
        self.payloads = [dict(p) for p in (payloads or [])]

    def to_body(self, v_cad) -> np.ndarray:
        return CAD_TO_FLU @ np.asarray(v_cad, dtype=float)

    def _rotor_entries(self):
        out = []
        for joint_name, entry in self.alignment.items():
            child, xyz = self.joints[joint_name]
            spin = "ccw" if "ccw" in child.lower() else "cw"
            out.append((joint_name, child, self.to_body(xyz), spin, entry))
        order = self.cad.get("rotor_order")
        if order:
            by_name = {e[0]: e for e in out}
            out = [by_name[n] for n in order]
        return out

    def rotors(self) -> list[dict]:
        names = self.cad.get("rotor_names", {})
        return [{"name": names.get(j, j), "position_m": pos, "spin": spin}
                for j, _, pos, spin, _ in self._rotor_entries()]

    def _items(self):
        """(name, mass, com_body_raw, inertia_body) for frame, rotors, payloads."""
        items = []
        m, com, tensor = self.links[self.frame_link]
        items.append(("body", m, self.to_body(com), CAD_TO_FLU @ tensor @ CAD_TO_FLU.T))
        for j, child, pos, _, _ in self._rotor_entries():
            m, com, tensor = self.links[child]
            items.append((j, m, pos + self.to_body(com), CAD_TO_FLU @ tensor @ CAD_TO_FLU.T))
        for p in self.payloads:
            size = np.asarray(p["size_m"], dtype=float)
            m = float(p["mass_kg"])
            box = m / 12.0 * np.diag([size[1] ** 2 + size[2] ** 2,
                                      size[0] ** 2 + size[2] ** 2,
                                      size[0] ** 2 + size[1] ** 2])
            items.append((str(p["name"]), m, np.asarray(p["position_m"], dtype=float), box))
        return items

    def total_mass(self) -> float:
        return float(sum(i[1] for i in self._items()))

    def raw_cg(self) -> np.ndarray:
        items = self._items()
        return sum(i[1] * i[2] for i in items) / sum(i[1] for i in items)

    def root_mass(self, cg) -> BodyMass:
        _, m, com, inertia = self._items()[0]
        return BodyMass("body", m, com - cg, inertia)

    def rotor_masses(self, cg, rotors) -> list[BodyMass]:
        items = self._items()[1:1 + len(rotors)]
        return [BodyMass(f"{rot.name}_rotor", m, com - cg, inertia)
                for rot, (_, m, com, inertia) in zip(rotors, items)]

    def payload_masses(self, cg) -> list[BodyMass]:
        n = len(self.alignment)
        return [BodyMass(name, m, com - cg, inertia) for name, m, com, inertia in self._items()[1 + n:]]


@lru_cache(maxsize=None)
def _cached(name: str) -> MultirotorSpec:
    return MultirotorSpec.load(name)


def load_multirotor_spec(name_or_path: str | Path = "fpv_quad_5in") -> MultirotorSpec:
    """Load a bundled airframe by name, or a YAML path. Bundled ones are cached."""
    key = str(name_or_path)
    if (SPEC_DIR / f"{key}.yaml").is_file():
        return _cached(key)
    return MultirotorSpec.load(name_or_path)
