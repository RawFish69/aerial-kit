"""L1/TECS wiring for the single-motor flying wing.

The longitudinal/lateral guidance and attitude law are shared with the twin
wing. The only single-wing-specific behaviour is that yaw moment is discarded:
there is no differential thrust or rudder actuator.
"""

from __future__ import annotations

from typing import Any, Mapping

import numpy as np

from aerial_kit.airframes.single_wing import SingleWingSpec, load_single_wing_spec
from aerial_kit.controllers.fixed_wing import (
    FixedWingL1TECSController,
    airspeed_nav_wrench,
)
from aerial_kit.dynamics.single_wing import SingleWingDynamics
from aerial_kit.types import CommandKind, ControlTarget, SimState, Waypoint, Wrench


class SingleWingL1TECSController(FixedWingL1TECSController):
    """Shared L1/TECS controller with the unsupported yaw demand cleared."""

    command_kind = CommandKind.AIRSPEED_NAV

    def compute(
        self,
        state: SimState,
        target_waypoint: Waypoint,
        cfg: Mapping[str, Any],
    ) -> ControlTarget:
        result = super().compute(state, target_waypoint, cfg)
        if result.wrench is not None:
            result.wrench.moment_body[2] = 0.0
        result.metadata = dict(result.metadata)
        result.metadata["controller"] = "single_wing_l1_tecs"
        return result


def single_wing_airspeed_nav_wrench(
    state: SimState,
    airspeed_cmd: float,
    bank_cmd: float,
    climb_rate_cmd: float,
    cfg: Mapping[str, Any] | None = None,
) -> Wrench:
    """Shared AirspeedNav-to-Wrench conversion with no yaw actuator demand."""
    wrench = airspeed_nav_wrench(
        state, airspeed_cmd, bank_cmd, climb_rate_cmd, cfg)
    wrench.moment_body[2] = 0.0
    return wrench


def single_wing_controller_config(
    cruise_airspeed_mps: float | None = None,
    spec: SingleWingSpec | None = None,
) -> dict[str, Any]:
    """Controller block whose neutral thrust comes from the shared spec."""
    spec = spec or load_single_wing_spec()
    if cruise_airspeed_mps is None:
        cruise_airspeed_mps = float(
            spec.flight_envelope["cruise_airspeed_mps"])
    trim = SingleWingDynamics(spec.dynamics_params()).compute_trim_detailed(
        cruise_airspeed_mps)
    return {
        "controller": {
            "l1_tecs": {
                "cruise_airspeed_mps": float(cruise_airspeed_mps),
                "l1_distance_m": 35.0,
                "max_bank_rad": float(np.radians(42.0)),
                "trim_throttle_n": float(trim["thrust_n"]),
                "max_thrust_n": spec.max_thrust_n,
                "tecs_kp_throttle": 0.20,
                "tecs_kp_pitch": 0.08,
                "climb_rate_kp": 0.25,
                "kp_roll": 0.70,
                "kd_roll": 0.18,
                "kp_pitch": 0.90,
                "kd_pitch": 0.22,
                "kp_yaw_rate": 1.50,
            }
        },
        "teleop_fixedwing": {
            "throttle_trim_n": float(trim["thrust_n"]),
            "throttle_range_n": 3.0,
            "elevon_max_rad": float(np.radians(10.0)),
            "yaw_diff_n": 0.0,
            "trim_elevon_rad": float(
                trim["elevon_common_rad"]),
        },
    }
