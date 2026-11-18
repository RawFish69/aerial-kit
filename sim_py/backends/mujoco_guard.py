"""Divergence guard shared by the MuJoCo backends.

When MuJoCo meets a non-finite or huge acceleration it logs a warning and
*resets the whole simulation state* (``mj_resetData``): the aircraft silently
reappears at the model origin with zero velocity. For a flight simulator that
is the worst possible failure, because everything downstream (estimator,
mission, ROS telemetry) sees a plausible-looking teleport home.

:func:`step_guarded` steps once and, if MuJoCo flagged a bad acceleration or
the state became non-finite, restores the last finite state and raises
:class:`SimulationDiverged` so the caller can stop and say so.
"""

from __future__ import annotations

import numpy as np


class SimulationDiverged(RuntimeError):
    """The MuJoCo integration blew up; the state was restored to the last finite step."""


def step_guarded(mujoco, model, data) -> None:
    bad = mujoco.mjtWarning.mjWARN_BADQACC
    before = int(data.warning[bad].number)
    qpos, qvel, act = data.qpos.copy(), data.qvel.copy(), data.act.copy()
    t = float(data.time)
    mujoco.mj_step(model, data)
    if int(data.warning[bad].number) == before and np.all(np.isfinite(data.qpos)) \
            and np.all(np.isfinite(data.qvel)):
        return
    rates = qvel[3:6] if qvel.shape[0] >= 6 else qvel
    data.qpos[:], data.qvel[:], data.act[:] = qpos, qvel, act
    data.time = t
    mujoco.mj_forward(model, data)
    raise SimulationDiverged(
        f"MuJoCo integration diverged at t={t:.3f} s (body rates before the step "
        f"{np.round(rates, 1).tolist()} rad/s); state restored to the last finite step. "
        "Usually a crash or tumble beyond what the integrator step can follow.")
