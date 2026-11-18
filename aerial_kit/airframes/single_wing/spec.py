"""Load and validate the single-motor flying-wing research specification."""

from __future__ import annotations

import copy
import math
from functools import lru_cache
from pathlib import Path
from typing import Any, Mapping

import numpy as np

from aerial_kit.airframes.twin_wing.spec import _load_yaml, _side_sign

DEFAULT_SPEC_PATH = Path(__file__).with_name("single_wing.yaml")
SIDES = ("left", "right")


class SingleWingSpec:
    """Typed view over ``single_wing.yaml``; all values are research defaults."""

    def __init__(self, data: Mapping[str, Any], source: str = "<dict>") -> None:
        self.data = copy.deepcopy(dict(data))
        self.source = source
        self._validate()

    @classmethod
    def load(cls, path: str | Path | None = None) -> "SingleWingSpec":
        path = Path(path) if path is not None else DEFAULT_SPEC_PATH
        return cls(_load_yaml(path), source=str(path))

    @property
    def mass(self) -> Mapping[str, Any]:
        return self.data["mass_properties"]

    @property
    def aero(self) -> Mapping[str, Any]:
        return self.data["aero_reference"]

    @property
    def coefficients(self) -> Mapping[str, Any]:
        return self.data["aero_coefficients"]

    @property
    def wing(self) -> Mapping[str, Any]:
        return self.data["wing"]

    @property
    def elevons(self) -> Mapping[str, Any]:
        return self.data["elevons"]

    @property
    def propulsion(self) -> Mapping[str, Any]:
        return self.data["propulsion"]

    @property
    def features(self) -> Mapping[str, Any]:
        return self.data["features"]

    @property
    def flight_envelope(self) -> Mapping[str, Any]:
        return self.data["flight_envelope"]

    def _validate(self) -> None:
        required = (
            "mass_properties", "aero_reference", "wing", "center_body",
            "fins", "elevons", "aero_coefficients", "propulsion",
            "features", "materials", "collision", "flight_envelope",
        )
        for key in required:
            if key not in self.data:
                raise ValueError(f"{self.source}: missing section {key!r}")
        if self.total_mass_kg <= 0.0:
            raise ValueError("total_mass_kg must be positive")
        inertia = self.inertia_diag
        if inertia.shape != (3,) or np.any(inertia <= 0.0):
            raise ValueError("inertia_kg_m2 must be three positive values")
        cg = np.asarray(self.mass["cg_m"], dtype=float)
        if cg.shape != (3,):
            raise ValueError("cg_m must be a three-vector")
        if not 0.0 < self.elevons["inboard_y_m"] < self.elevons["outboard_y_m"] < self.half_span:
            raise ValueError("elevon span must lie inside the half span")
        if self.elevon_limit_rad < self.max_elevon_rad:
            raise ValueError("mechanical elevon limit must cover commanded travel")
        if self.prop_spin not in (-1, 1):
            raise ValueError("motor spin must be +1 or -1")
        axis = np.asarray(self.thrust_axis, dtype=float)
        if axis.shape != (3,) or np.linalg.norm(axis) < 1e-9:
            raise ValueError("motor thrust_axis must be a nonzero 3-vector")
        if axis[0] <= 1e-9:
            raise ValueError("single-wing thrust axis must have a forward component")
        if self.max_thrust_n <= 0.0 or self.thrust_tau_s < 0.0:
            raise ValueError("max thrust must be positive and thrust_tau_s non-negative")
        if self.coefficients["cl_alpha_per_rad"] <= 0.0:
            raise ValueError("cl_alpha_per_rad must be positive")
        if self.reaction_torque_per_n < 0.0 or self.prop_wash_gain < 0.0:
            raise ValueError("propulsion torque and wash gains must be non-negative")
        if self.prop_inertia_kg_m2 <= 0.0 or self.motor_inertia_kg_m2 <= 0.0:
            raise ValueError("propeller and motor inertias must be positive")
        envelope = self.flight_envelope
        if not (
            0.0 < float(envelope["min_airspeed_mps"])
            < float(envelope["cruise_airspeed_mps"])
            < float(envelope["max_airspeed_mps"])
        ):
            raise ValueError("flight-envelope airspeeds must be strictly increasing")
        if not 0.0 < float(envelope["max_bank_deg"]) < 90.0:
            raise ValueError("max_bank_deg must be in (0, 90)")

    @property
    def total_mass_kg(self) -> float:
        return float(self.mass["total_mass_kg"])

    @property
    def inertia_diag(self) -> np.ndarray:
        return np.asarray(self.mass["inertia_kg_m2"], dtype=float)

    @property
    def cg_m(self) -> np.ndarray:
        return np.asarray(self.mass["cg_m"], dtype=float)

    @property
    def span(self) -> float:
        return float(self.aero["wingspan_m"])

    @property
    def half_span(self) -> float:
        return 0.5 * self.span

    @property
    def area(self) -> float:
        return float(self.aero["wing_area_m2"])

    @property
    def mean_chord(self) -> float:
        return float(self.aero["mean_chord_m"])

    @property
    def max_elevon_rad(self) -> float:
        return math.radians(float(self.elevons["travel_deg"]))

    @property
    def elevon_limit_rad(self) -> float:
        return math.radians(float(self.elevons["mechanical_limit_deg"]))

    @property
    def max_thrust_n(self) -> float:
        return float(self.propulsion["max_thrust_n"])

    @property
    def thrust_tau_s(self) -> float:
        return float(self.propulsion["thrust_tau_s"])

    @property
    def motor_position(self) -> np.ndarray:
        return np.asarray(self.propulsion["motor"]["position_m"], dtype=float)

    @property
    def thrust_axis(self) -> np.ndarray:
        axis = np.asarray(self.propulsion["motor"]["thrust_axis"], dtype=float)
        return axis / np.linalg.norm(axis)

    @property
    def prop_spin(self) -> int:
        return int(self.propulsion["motor"]["spin"])

    @property
    def reaction_torque_per_n(self) -> float:
        return float(self.propulsion["motor"]["reaction_torque_nm_per_n"])

    @property
    def prop_inertia_kg_m2(self) -> float:
        return float(self.propulsion["propeller"]["inertia_kg_m2"])

    @property
    def motor_inertia_kg_m2(self) -> float:
        return float(self.propulsion["motor"]["rotor_inertia_kg_m2"])

    @property
    def prop_wash_gain(self) -> float:
        return float(self.propulsion["prop_wash"]["authority_gain"])

    def planform_area(self) -> float:
        return self.span * 0.5 * (
            float(self.wing["root_chord_m"]) + float(self.wing["tip_chord_m"]))

    def chord(self, y: float) -> float:
        eta = min(abs(float(y)) / self.half_span, 1.0)
        return float(self.wing["root_chord_m"]) + eta * (
            float(self.wing["tip_chord_m"]) - float(self.wing["root_chord_m"]))

    def le_x(self, y: float) -> float:
        return float(self.wing["root_le_x_m"]) - abs(float(y)) * math.tan(
            math.radians(float(self.wing["le_sweep_deg"])))

    def te_x(self, y: float) -> float:
        return self.le_x(y) - self.chord(y)

    def wing_z(self, y: float) -> float:
        return -abs(float(y)) * math.tan(
            math.radians(float(self.wing["dihedral_deg"])))

    def elevon_hinge(self, side: str) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        s = _side_sign(side)
        y0 = float(self.elevons["inboard_y_m"])
        y1 = float(self.elevons["outboard_y_m"])
        fraction = float(self.elevons["chord_fraction"])
        p0 = np.array([
            self.te_x(y0) + fraction * self.chord(y0), s * y0, self.wing_z(y0)])
        p1 = np.array([
            self.te_x(y1) + fraction * self.chord(y1), s * y1, self.wing_z(y1)])
        direction = (p1 - p0) * s
        direction /= np.linalg.norm(direction)
        return p0, p1, -direction

    def dynamics_param_overrides(self) -> dict[str, Any]:
        c = self.coefficients
        p = self.propulsion
        f = self.features
        return {
            "mass_kg": self.total_mass_kg,
            "wing_area_m2": self.area,
            "wingspan_m": self.span,
            "mean_chord_m": self.mean_chord,
            "inertia_kg_m2": self.inertia_diag.copy(),
            "air_density_kg_m3": float(c["air_density_kg_m3"]),
            "gravity_mps2": float(c["gravity_mps2"]),
            "cl0": float(c["cl0"]),
            "cl_alpha": float(c["cl_alpha_per_rad"]),
            "alpha_stall_rad": math.radians(float(c["alpha_stall_deg"])),
            "stall_blend_rate": float(c["stall_blend_rate"]),
            "cd0": float(c["cd0"]),
            "k_induced": float(c["induced_drag_k"]),
            "cm0": float(c["cm0"]),
            "cm_alpha": float(c["cm_alpha"]),
            "cm_q": float(c["cm_q"]),
            "cm_delta_e": float(c["cm_delta_e"]),
            "cl_beta": float(c["cl_beta"]),
            "cl_p": float(c["cl_p"]),
            "cl_delta_a": float(c["cl_delta_a"]),
            "cn_beta": float(c["cn_beta"]),
            "cn_r": float(c["cn_r"]),
            "max_thrust_n": float(p["max_thrust_n"]),
            "thrust_tau_s": float(p["thrust_tau_s"]),
            "max_elevon_rad": self.max_elevon_rad,
            "thrust_axis": self.thrust_axis.copy(),
            "motor_position_m": self.motor_position,
            "prop_spin": self.prop_spin,
            "reaction_torque_per_n_nm": self.reaction_torque_per_n,
            "prop_inertia_kg_m2": self.prop_inertia_kg_m2,
            "motor_inertia_kg_m2": self.motor_inertia_kg_m2,
            "prop_wash_gain": self.prop_wash_gain,
            "enable_motor_reaction": bool(f["motor_reaction_torque"]),
            "enable_thrust_lag": bool(f["thrust_lag"]),
            "enable_prop_wash": bool(f["prop_wash"]),
            "enable_thrust_offset": bool(f["thrust_offset_moment"]),
        }

    def dynamics_params(self, **overrides: Any):
        from aerial_kit.dynamics.single_wing import SingleWingParams

        values = self.dynamics_param_overrides()
        values.update(overrides)
        return SingleWingParams(**values)


@lru_cache(maxsize=1)
def _default_spec() -> SingleWingSpec:
    return SingleWingSpec.load()


def load_single_wing_spec(path: str | Path | None = None) -> SingleWingSpec:
    if path is None or str(path) == "":
        return _default_spec()
    return SingleWingSpec.load(path)
