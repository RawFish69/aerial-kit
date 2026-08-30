"""AIRSPEED_NAV controller for the twin wing: L1 lateral + TECS-lite + attitude PID.

Produces a body ``Wrench`` (in ``ControlTarget.metadata["wrench"]``, since the
shared ``ControlTarget.accel_cmd`` field is an ACCEL-kind concept this
controller doesn't use) for ``TwinWingAirframe.allocate()`` to turn into
actuator commands. Three loops, outer to inner, per plan 04:

1. L1 picks a bank command from the vehicle's position/velocity toward the
   target waypoint (see ``aerial_kit.guidance.l1`` for why this is the
   point-target variant, not full path-segment L1).
2. TECS-lite picks throttle + pitch command from airspeed/altitude error (see
   ``aerial_kit.guidance.tecs``).
3. An attitude PID converts bank/pitch *commands* into roll/pitch *moment*
   demands using the vehicle's actual attitude, closing the loop.

Attitude feedback avoids extracting classical Euler angles from
``attitude_quat`` (its body-FRD/world-ENU convention is not the one those
formulas assume -- see ``aerial_kit/dynamics/fixed_wing.py``'s docstring). Instead
it uses body axes projected onto the world frame directly: pitch is the
elevation of the body-forward axis above the horizon, bank is (approximately,
for small pitch) the depression of the body-right axis below the horizon.
This sidesteps Euler-order/sign ambiguity entirely and is exact at zero pitch,
which covers the cruise-flight regime this whole hand-rolled model targets.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Mapping

import numpy as np

from ..dynamics.fixed_wing import quat_to_rotmat
from ..guidance.l1 import l1_bank_command
from ..guidance.tecs import TecsGains, tecs_command
from ..interfaces import Controller
from ..types import CommandKind, ControlTarget, SimState, Waypoint, Wrench


@dataclass
class AttitudeGains:
    kp_roll: float = 0.6
    kd_roll: float = 0.15
    kp_pitch: float = 0.8
    kd_pitch: float = 0.2
    kp_yaw_rate: float = 2.0


def body_axis_pitch_bank(attitude_quat: np.ndarray) -> tuple[float, float]:
    """(pitch_rad, bank_rad) from body axes projected onto the world frame."""
    rot = quat_to_rotmat(attitude_quat)
    body_x_world = rot @ np.array([1.0, 0.0, 0.0])
    body_y_world = rot @ np.array([0.0, 1.0, 0.0])
    pitch = float(np.arcsin(np.clip(body_x_world[2], -1.0, 1.0)))
    bank = float(-np.arcsin(np.clip(body_y_world[2], -1.0, 1.0)))
    return pitch, bank


class FixedWingL1TECSController(Controller):
    """L1 + TECS-lite + attitude PID, emitting a Wrench for TwinWingAirframe."""

    command_kind = CommandKind.AIRSPEED_NAV

    def compute(
        self,
        state: SimState,
        target_waypoint: Waypoint,
        cfg: Mapping[str, Any],
    ) -> ControlTarget:
        ctrl_cfg = dict(cfg.get("controller", {}) or {})
        fw_cfg = dict(ctrl_cfg.get("l1_tecs", {}) or {})

        cruise_airspeed_mps = float(fw_cfg.get("cruise_airspeed_mps", 15.0))
        l1_distance_m = float(fw_cfg.get("l1_distance_m", 40.0))
        max_bank_rad = float(fw_cfg.get("max_bank_rad", np.radians(45.0)))
        gravity_mps2 = float(fw_cfg.get("gravity_mps2", 9.81))

        tecs_gains = TecsGains(
            kp_throttle=float(fw_cfg.get("tecs_kp_throttle", 0.15)),
            kp_pitch=float(fw_cfg.get("tecs_kp_pitch", 0.08)),
            trim_throttle_n=float(fw_cfg.get("trim_throttle_n", 3.0)),
            max_thrust_n=float(fw_cfg.get("max_thrust_n", 10.0)),
            max_pitch_rad=float(fw_cfg.get("max_pitch_rad", 0.35)),
        )
        att_gains = AttitudeGains(
            kp_roll=float(fw_cfg.get("kp_roll", 0.6)),
            kd_roll=float(fw_cfg.get("kd_roll", 0.15)),
            kp_pitch=float(fw_cfg.get("kp_pitch", 0.8)),
            kd_pitch=float(fw_cfg.get("kd_pitch", 0.2)),
            kp_yaw_rate=float(fw_cfg.get("kp_yaw_rate", 2.0)),
        )

        position = np.asarray(state.position, dtype=float)
        velocity = np.asarray(state.velocity, dtype=float)
        airspeed = float(np.linalg.norm(velocity[:2]))

        bank_cmd = l1_bank_command(
            position_xy=position[:2],
            velocity_xy=velocity[:2],
            target_xy=np.asarray(target_waypoint.position, dtype=float)[:2],
            airspeed_mps=max(airspeed, 1.0),
            l1_distance_m=l1_distance_m,
            max_bank_rad=max_bank_rad,
            gravity_mps2=gravity_mps2,
        )
        throttle_cmd_n, pitch_cmd = tecs_command(
            airspeed_mps=airspeed,
            airspeed_cmd_mps=cruise_airspeed_mps,
            altitude_m=float(position[2]),
            altitude_cmd_m=float(target_waypoint.position[2]),
            gains=tecs_gains,
            gravity_mps2=gravity_mps2,
        )

        if state.attitude_quat is None or state.body_rates is None:
            pitch, bank = 0.0, 0.0
            p_rate = q_rate = r_rate = 0.0
        else:
            pitch, bank = body_axis_pitch_bank(np.asarray(state.attitude_quat, dtype=float))
            p_rate, q_rate, r_rate = np.asarray(state.body_rates, dtype=float)

        roll_m = att_gains.kp_roll * (bank_cmd - bank) - att_gains.kd_roll * p_rate
        pitch_m = att_gains.kp_pitch * (pitch_cmd - pitch) - att_gains.kd_pitch * q_rate

        # Coordinated turn: command the yaw rate a level turn at this bank and
        # airspeed actually needs (r_cmd = g*tan(bank)/V), rather than only
        # damping yaw rate toward zero -- banking with no yaw-rate command
        # produces sideslip, not a clean turn, since this model has no rudder
        # and only weak weathervaning aero to close that gap on its own.
        turn_airspeed = max(airspeed, 1.0)
        r_cmd = gravity_mps2 * np.tan(bank) / turn_airspeed
        yaw_m = att_gains.kp_yaw_rate * (r_cmd - r_rate)

        wrench = Wrench(
            force_body=np.array([throttle_cmd_n, 0.0, 0.0], dtype=float),
            moment_body=np.array([roll_m, pitch_m, yaw_m], dtype=float),
        )
        return ControlTarget(
            accel_cmd=np.zeros(3, dtype=float),
            metadata={
                "controller": "l1_tecs",
                "wrench": wrench,
                "bank_cmd": bank_cmd,
                "pitch_cmd": pitch_cmd,
                "throttle_cmd_n": throttle_cmd_n,
            },
        )
