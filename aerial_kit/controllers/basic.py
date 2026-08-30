"""Built-in controller implementations for the simulation framework."""

from __future__ import annotations

from typing import Any, Mapping

import numpy as np

from ..interfaces import Controller
from ..types import ControlTarget, SimState, Waypoint
from .position import lqr_position_control, mpc_position_control, pid_position_control


class PIDController(Controller):
    """PD position controller returning acceleration commands."""

    def compute(
        self,
        state: SimState,
        target_waypoint: Waypoint,
        cfg: Mapping[str, Any],
    ) -> ControlTarget:
        ctrl_cfg = dict(cfg.get("controller", {}) or {})
        pid_cfg = dict(ctrl_cfg.get("pid", {}) or {})
        kp = float(pid_cfg.get("kp", 0.8))
        kd = float(pid_cfg.get("kd", 1.2))

        acc_cmd = pid_position_control(
            state.position,
            state.velocity,
            target_waypoint.position,
            kp=kp,
            kd=kd,
        )
        return ControlTarget(accel_cmd=np.asarray(acc_cmd, dtype=float), metadata={"controller": "pid"})


class LQRController(Controller):
    """LQR position controller returning acceleration commands."""

    def compute(
        self,
        state: SimState,
        target_waypoint: Waypoint,
        cfg: Mapping[str, Any],
    ) -> ControlTarget:
        ctrl_cfg = dict(cfg.get("controller", {}) or {})
        lqr_cfg = dict(ctrl_cfg.get("lqr", {}) or {})
        acc_cmd = lqr_position_control(
            state.position,
            state.velocity,
            target_waypoint.position,
            q_pos=float(lqr_cfg.get("q_pos", 10.0)),
            q_vel=float(lqr_cfg.get("q_vel", 2.0)),
            r_acc=float(lqr_cfg.get("r_acc", 1.0)),
        )
        return ControlTarget(accel_cmd=np.asarray(acc_cmd, dtype=float), metadata={"controller": "lqr"})


class MPCController(Controller):
    """MPC-like position controller returning acceleration commands."""

    def compute(
        self,
        state: SimState,
        target_waypoint: Waypoint,
        cfg: Mapping[str, Any],
    ) -> ControlTarget:
        ctrl_cfg = dict(cfg.get("controller", {}) or {})
        mpc_cfg = dict(ctrl_cfg.get("mpc", {}) or {})
        acc_cmd = mpc_position_control(
            state.position,
            state.velocity,
            target_waypoint.position,
            q_pos=float(mpc_cfg.get("q_pos", 8.0)),
            q_vel=float(mpc_cfg.get("q_vel", 2.0)),
            r_acc=float(mpc_cfg.get("r_acc", 10.0)),
        )
        return ControlTarget(accel_cmd=np.asarray(acc_cmd, dtype=float), metadata={"controller": "mpc"})
