"""Load the shared twin-wing airframe description and derive geometry from it.

``twin_wing.yaml`` next to this module is the single source of truth for the
twin wing's mass, planform, fins, elevons, motors, props, materials and
collision simplification. Everything that needs those numbers -- the MuJoCo
model, the Gazebo visual, :class:`FixedWingParams` construction in the ROS
adapter -- goes through :class:`TwinWingSpec` rather than restating them.

Frame: body FRD (x forward, y right, z down), origin at the reference CG.
"""

from __future__ import annotations

import copy
import math
from dataclasses import dataclass
from functools import lru_cache
from pathlib import Path
from typing import Any, Mapping

import numpy as np

DEFAULT_SPEC_PATH = Path(__file__).with_name("twin_wing.yaml")

SIDES = ("left", "right")


def _side_sign(side: str) -> float:
    if side == "left":
        return -1.0
    if side == "right":
        return 1.0
    raise ValueError(f"side must be 'left' or 'right', got {side!r}")


def _load_yaml(path: Path) -> dict[str, Any]:
    try:
        import yaml
    except Exception as exc:  # pragma: no cover - optional dependency
        raise RuntimeError(
            "PyYAML is required to read the twin-wing airframe spec "
            "(pip install pyyaml, or the 'sim' extra).") from exc
    with open(path, "r", encoding="utf-8") as handle:
        data = yaml.safe_load(handle)
    if not isinstance(data, dict):
        raise ValueError(f"{path}: expected a mapping at the top level")
    return data


@dataclass(frozen=True)
class ChildMass:
    """Mass properties of one articulated child body, in body FRD about the CG frame."""

    name: str
    mass_kg: float
    com_m: np.ndarray          # child COM in the root frame
    inertia_kg_m2: np.ndarray  # 3x3 about the child COM, root-frame axes


@dataclass(frozen=True)
class RootMass:
    mass_kg: float
    com_m: np.ndarray
    inertia_kg_m2: np.ndarray  # 3x3 about the root COM


class TwinWingSpec:
    """Typed view over ``twin_wing.yaml`` plus the geometry derived from it."""

    def __init__(self, data: Mapping[str, Any], source: str = "<dict>") -> None:
        self.data = copy.deepcopy(dict(data))
        self.source = source
        self._validate()

    # ------------------------------------------------------------------ io
    @classmethod
    def load(cls, path: str | Path | None = None) -> "TwinWingSpec":
        path = Path(path) if path is not None else DEFAULT_SPEC_PATH
        return cls(_load_yaml(path), source=str(path))

    # ------------------------------------------------------------ sections
    @property
    def mass(self) -> Mapping[str, Any]:
        return self.data["mass_properties"]

    @property
    def aero(self) -> Mapping[str, Any]:
        return self.data["aero_reference"]

    @property
    def wing(self) -> Mapping[str, Any]:
        return self.data["wing"]

    @property
    def body(self) -> Mapping[str, Any]:
        return self.data["center_body"]

    @property
    def fins(self) -> Mapping[str, Any]:
        return self.data["fins"]

    @property
    def elevons(self) -> Mapping[str, Any]:
        return self.data["elevons"]

    @property
    def propulsion(self) -> Mapping[str, Any]:
        return self.data["propulsion"]

    @property
    def materials(self) -> Mapping[str, list[float]]:
        return self.data["materials"]

    @property
    def collision(self) -> Mapping[str, Any]:
        return self.data["collision"]

    # ---------------------------------------------------------- validation
    def _validate(self) -> None:
        for key in (
            "mass_properties", "aero_reference", "wing", "center_body", "fins",
            "elevons", "propulsion", "materials", "collision",
        ):
            if key not in self.data:
                raise ValueError(f"{self.source}: missing section {key!r}")
        if float(self.mass["total_mass_kg"]) <= 0.0:
            raise ValueError("total_mass_kg must be positive")
        inertia = np.asarray(self.mass["inertia_kg_m2"], dtype=float)
        if inertia.shape != (3,) or np.any(inertia <= 0.0):
            raise ValueError("inertia_kg_m2 must be three positive values")
        e = self.elevons
        if not 0.0 < float(e["inboard_y_m"]) < float(e["outboard_y_m"]) < self.half_span:
            raise ValueError("elevon span must lie inside the half span")
        if not 0.05 < float(e["chord_fraction"]) < 0.6:
            raise ValueError("elevon chord_fraction must be in (0.05, 0.6)")
        if float(e["mechanical_limit_deg"]) < float(e["travel_deg"]):
            raise ValueError("elevon mechanical limit must not be below its travel")
        idx = e["command_index"]
        if sorted((int(idx["left"]), int(idx["right"]))) != [2, 3]:
            raise ValueError("elevon command_index must map left/right onto 2 and 3")
        for side in SIDES:
            spin = int(self.propulsion["motors"][side]["spin"])
            if spin not in (-1, 1):
                raise ValueError(f"{side} motor spin must be +1 or -1")
        rotor = self.propulsion["rotor"]
        if float(rotor["visual_max_speed_rad_s"]) <= 0.0:
            raise ValueError("visual_max_speed_rad_s must be positive")
        for name, rgba in self.materials.items():
            if len(rgba) != 4:
                raise ValueError(f"material {name!r} must be rgba")

    # ---------------------------------------------------- reference values
    @property
    def total_mass_kg(self) -> float:
        return float(self.mass["total_mass_kg"])

    @property
    def inertia_diag(self) -> np.ndarray:
        return np.asarray(self.mass["inertia_kg_m2"], dtype=float)

    @property
    def span(self) -> float:
        return float(self.aero["wingspan_m"])

    @property
    def half_span(self) -> float:
        return 0.5 * self.span

    @property
    def planform_area(self) -> float:
        w = self.wing
        return self.span * 0.5 * (float(w["root_chord_m"]) + float(w["tip_chord_m"]))

    @property
    def motor_separation_m(self) -> float:
        motors = self.propulsion["motors"]
        return abs(float(motors["right"]["y_m"]) - float(motors["left"]["y_m"]))

    @property
    def max_elevon_rad(self) -> float:
        return math.radians(float(self.elevons["travel_deg"]))

    @property
    def elevon_limit_rad(self) -> float:
        return math.radians(float(self.elevons["mechanical_limit_deg"]))

    # ------------------------------------------------------------ planform
    def chord(self, y: float) -> float:
        w = self.wing
        eta = min(abs(float(y)) / self.half_span, 1.0)
        return float(w["root_chord_m"]) + eta * (float(w["tip_chord_m"]) - float(w["root_chord_m"]))

    def le_x(self, y: float) -> float:
        w = self.wing
        return float(w["root_le_x_m"]) - abs(float(y)) * math.tan(math.radians(float(w["le_sweep_deg"])))

    def te_x(self, y: float) -> float:
        return self.le_x(y) - self.chord(y)

    def wing_z(self, y: float) -> float:
        # Dihedral raises the tips: -z is up in FRD.
        return -abs(float(y)) * math.tan(math.radians(float(self.wing["dihedral_deg"])))

    def hinge_x(self, y: float) -> float:
        return self.te_x(y) + float(self.elevons["chord_fraction"]) * self.chord(y)

    def thickness_ratio(self, y: float) -> float:
        a = self.wing["airfoil"]
        eta = min(abs(float(y)) / self.half_span, 1.0)
        return float(a["root_thickness"]) + eta * (float(a["tip_thickness"]) - float(a["root_thickness"]))

    def mean_aero_chord(self) -> tuple[float, float, float]:
        """(MAC length, its spanwise station, its LE x)."""
        w = self.wing
        cr, ct = float(w["root_chord_m"]), float(w["tip_chord_m"])
        lam = ct / cr
        mac = 2.0 / 3.0 * cr * (1.0 + lam + lam * lam) / (1.0 + lam)
        y_mac = self.span / 6.0 * (1.0 + 2.0 * lam) / (1.0 + lam)
        return mac, y_mac, self.le_x(y_mac)

    def static_margin(self) -> float:
        """Quarter-MAC neutral-point estimate minus CG, in MAC (geometric only)."""
        mac, _, x_le = self.mean_aero_chord()
        x_np = x_le - 0.25 * mac
        return (0.0 - x_np) / mac

    # ------------------------------------------------------------- elevons
    def elevon_hinge(self, side: str) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """(inboard point, outboard point, unit axis) of a hinge line.

        The axis is oriented so that a positive rotation about it moves the
        trailing edge up (-z in FRD) on both sides.
        """
        s = _side_sign(side)
        e = self.elevons
        y0, y1 = float(e["inboard_y_m"]), float(e["outboard_y_m"])
        p0 = np.array([self.hinge_x(y0), s * y0, self.wing_z(y0)])
        p1 = np.array([self.hinge_x(y1), s * y1, self.wing_z(y1)])
        direction = (p1 - p0) * s          # points toward +y on both sides
        direction /= np.linalg.norm(direction)
        # Rotation about +y moves a point behind the hinge (-x) to +z (down),
        # so trailing-edge-up positive needs the hinge direction negated.
        axis = -direction
        return p0, p1, axis

    def elevon_command_index(self, side: str) -> int:
        return int(self.elevons["command_index"][side])

    # ----------------------------------------------------------- propulsion
    def motor_y(self, side: str) -> float:
        return float(self.propulsion["motors"][side]["y_m"])

    def motor_spin(self, side: str) -> int:
        return int(self.propulsion["motors"][side]["spin"])

    def prop_position(self, side: str) -> np.ndarray:
        p = self.propulsion
        y = self.motor_y(side)
        x = self.le_x(y) + float(p["prop_plane_ahead_of_le_m"])
        return np.array([x, y, float(p["thrust_line_z_m"])])

    def rotor_axial_inertia(self) -> float:
        """Simulated shaft inertia, scaled to keep real angular momentum."""
        r = self.propulsion["rotor"]
        return float(r["physical_inertia_kg_m2"]) * (
            float(r["physical_max_speed_rad_s"]) / float(r["visual_max_speed_rad_s"]))

    def shaft_speed_for_thrust(self, thrust_n: float, side: str) -> float:
        """Signed simulated shaft speed for a thrust (static thrust ~ omega^2)."""
        max_t = float(self.propulsion["max_thrust_n"])
        frac = min(max(float(thrust_n) / max_t, 0.0), 1.0)
        omega = float(self.propulsion["rotor"]["visual_max_speed_rad_s"]) * math.sqrt(frac)
        return self.motor_spin(side) * omega

    def clamp_actuator_command(
        self, command, max_thrust_n: float | None = None
    ) -> tuple[np.ndarray, bool]:
        """Clamp ``[thrust_L, thrust_R, elevon_L, elevon_R]`` to the airframe's limits.

        Thrust goes to ``[0, max_thrust]`` (the smaller of the spec limit and
        ``max_thrust_n``), elevons to the commanded servo travel. Returns the
        clamped vector and whether anything was clipped. A non-finite or
        wrongly-shaped command is refused with ``ValueError`` rather than
        clamped: NaN has no safe nearest value.
        """
        cmd = np.asarray(command, dtype=float)
        if cmd.shape != (4,):
            raise ValueError(f"actuator command must have 4 values, got shape {cmd.shape}")
        if not np.all(np.isfinite(cmd)):
            raise ValueError("actuator command contains a non-finite value")
        limit = float(self.propulsion["max_thrust_n"])
        if max_thrust_n is not None:
            limit = min(limit, float(max_thrust_n))
        travel = self.max_elevon_rad
        out = cmd.copy()
        out[:2] = np.clip(out[:2], 0.0, limit)
        out[2:] = np.clip(out[2:], -travel, travel)
        return out, bool(np.any(out != cmd))

    # --------------------------------------------------------- mass split
    def elevon_mass(self, side: str) -> ChildMass:
        e = self.elevons
        m = float(e["mass_kg_each"])
        y0, y1 = float(e["inboard_y_m"]), float(e["outboard_y_m"])
        ym = 0.5 * (y0 + y1)
        c = float(e["chord_fraction"]) * self.chord(ym)
        span = y1 - y0
        s = _side_sign(side)
        com = np.array([self.hinge_x(ym) - 0.42 * c, s * ym, self.wing_z(ym)])
        # Plate: span along y, chord along x, ~8 mm thick.
        t = 0.008
        inertia = np.diag([
            m * (span ** 2 + t ** 2) / 12.0,
            m * (c ** 2 + t ** 2) / 12.0,
            m * (span ** 2 + c ** 2) / 12.0,
        ])
        return ChildMass(f"{side}_elevon", m, com, inertia)

    def rotor_mass(self, side: str) -> ChildMass:
        r = self.propulsion["rotor"]
        m = float(r["mass_kg"])
        axial = self.rotor_axial_inertia()
        # Triangle inequality: transverse >= axial / 2 for any real body.
        transverse = 0.5 * axial + 1e-7
        com = self.prop_position(side) + np.array([-0.012, 0.0, 0.0])
        return ChildMass(f"{side}_prop", m, com, np.diag([axial, transverse, transverse]))

    def children_mass(self) -> list[ChildMass]:
        return [self.elevon_mass(s) for s in SIDES] + [self.rotor_mass(s) for s in SIDES]

    def root_mass(self) -> RootMass:
        """Root-body mass properties so the composite equals the spec totals.

        The composite CG lands on the frame origin and the composite inertia
        about it equals ``diag(inertia_kg_m2)`` at neutral elevons.
        """
        children = self.children_mass()
        m_children = sum(c.mass_kg for c in children)
        m_root = self.total_mass_kg - m_children
        if m_root <= 0.0:
            raise ValueError("child masses exceed the total airframe mass")
        first_moment = sum(c.mass_kg * c.com_m for c in children)
        com_root = -first_moment / m_root
        target = np.diag(self.inertia_diag)

        def parallel(mass: float, d: np.ndarray) -> np.ndarray:
            return mass * (float(d @ d) * np.eye(3) - np.outer(d, d))

        inertia = target.copy()
        for c in children:
            inertia -= c.inertia_kg_m2 + parallel(c.mass_kg, c.com_m)
        inertia -= parallel(m_root, com_root)
        inertia = 0.5 * (inertia + inertia.T)
        eig = np.linalg.eigvalsh(inertia)
        if eig.min() <= 0.0 or eig.max() > eig.sum() - eig.max():
            raise ValueError(
                "root inertia after removing child bodies is not physical: "
                f"eigenvalues {eig}")
        return RootMass(m_root, com_root, inertia)

    # --------------------------------------------- aero model construction
    def fixed_wing_param_overrides(self) -> dict[str, Any]:
        """The FixedWingParams fields this spec owns."""
        return {
            "mass_kg": self.total_mass_kg,
            "wing_area_m2": float(self.aero["wing_area_m2"]),
            "wingspan_m": self.span,
            "mean_chord_m": float(self.aero["mean_chord_m"]),
            "inertia_kg_m2": self.inertia_diag.copy(),
            "motor_separation_m": self.motor_separation_m,
        }

    def fixed_wing_params(self, **overrides: Any):
        from aerial_kit.dynamics.fixed_wing import FixedWingParams

        values = self.fixed_wing_param_overrides()
        values.update(overrides)
        return FixedWingParams(**values)


@lru_cache(maxsize=1)
def _default_spec() -> TwinWingSpec:
    return TwinWingSpec.load()


def load_twin_wing_spec(path: str | Path | None = None) -> TwinWingSpec:
    """Load a spec; the bundled default is cached (treat it as read-only)."""
    if path is None or str(path) == "":
        return _default_spec()
    return TwinWingSpec.load(path)
