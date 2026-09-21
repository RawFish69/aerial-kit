"""Core datatypes for the standalone simulator framework."""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum, auto
from typing import Any

import numpy as np


class CommandKind(Enum):
    """What kind of control target an airframe's inner loop accepts."""

    ACCEL = auto()  # multirotor / point mass -- today's behavior
    AIRSPEED_NAV = auto()  # fixed wing: (airspeed, bank_or_course_rate, climb_rate)
    WRENCH = auto()  # low-level: body thrust + moment


@dataclass(frozen=True)
class Capabilities:
    """What an airframe can physically do -- used to validate controller/planner fit."""

    can_hover: bool
    min_airspeed_mps: float | None  # None for hovering craft
    max_airspeed_mps: float
    max_climb_rate_mps: float
    max_bank_deg: float
    min_turn_radius_m: float | None  # None if it can turn in place
    n_actuators: int
    command_kind: CommandKind


class ControlMode(Enum):
    """Which inner-loop path produced a step, stated rather than implied.

    Both are legitimate and they are not the same experiment. Only one of them
    is a plant.

    ``IDEAL_ACCEL``
        The controller's acceleration command *is* the aircraft's acceleration.
        The backend integrates it directly, so there is no motor lag, no thrust
        saturation, no allocation error and no attitude dynamics between the
        command and the motion. Fast, and the right tool for guidance and
        planning work, where the inner loop is not the question.

    ``ACTUATOR``
        The controller's output goes through an airframe's allocator and the
        backend integrates what the actuators could actually produce. Slower,
        and the only mode in which a claim about an aircraft's *dynamics* means
        anything.

    This exists because the first of those used to be an ``else`` branch. A run
    in ideal-acceleration mode and a run in actuator mode produced trajectories
    that looked alike and were not comparable, and nothing in the output said
    which one you were reading.
    """

    IDEAL_ACCEL = auto()
    ACTUATOR = auto()


@dataclass
class Wrench:
    """Body-frame thrust + moment, the common output of an inner-loop controller.

    **The frame, because it is not the obvious one.** This repository's models
    are z-up (ENU-aligned body axes): ``force_body[2]`` is *upward* thrust, and
    ``moment_body`` follows from it with ``Mx = +sum(y*T)``, ``My = -sum(x*T)``.
    That is not universal and it is not the firmware's convention - the
    AerialKit firmware is FRD, where z points down and a rotor's thrust is a
    *negative* z force - so the same physical hover is ``+9.81`` here and
    ``-9.81`` there. Two frame conventions that are never adjacent cannot
    disagree visibly; putting this one next to the other is what made it
    visible.

    **And the units at the far end.** ``MultirotorAirframe.allocate`` returns
    motor commands in **newtons** (they sum to ``force_body[2]``, and ``trim()``
    returns ``mass*g/4`` per motor), whereas ``UAVDynamics`` takes a
    **hover-normalised fraction** where ``hover_thrust=1.0`` is hover. Feeding
    one into the other is a factor of 2.4525 on a 1 kg quadrotor.
    """

    force_body: np.ndarray  # 3D body-frame force [N], +z up in this repository
    moment_body: np.ndarray  # 3D body-frame moment [N*m]


@dataclass
class SimState:
    """Canonical simulator state used across controllers and backends.

    Quaternion convention is ``[w, x, y, z]`` when present.
    """

    position: np.ndarray
    velocity: np.ndarray
    t: float = 0.0
    attitude_quat: np.ndarray | None = None
    body_rates: np.ndarray | None = None


@dataclass
class ControlTarget:
    """Canonical high-level control target produced by controllers.

    Two ways to say what the aircraft should do, and they are not alternatives
    to each other - one is a demand, the other is a realisation.

    ``accel_cmd``
        A world-frame acceleration demand, z-up. This is the contract the four
        acceleration-level backends consume (point-mass, multirotor, rotorpy,
        MuJoCo) and the contract ``controllers/basic.py``'s four controllers
        produce. In that path the demand *is* the aircraft's acceleration, and
        there is no motor lag, no saturation, no allocation error and no
        attitude dynamics in the loop.

    ``wrench``
        A body-frame force and moment in this repository's z-up body frame
        (:class:`Wrench`), set by a controller that has closed an attitude and
        rate loop of its own: the fixed wing's L1/TECS, and
        :class:`~aerial_kit.controllers.cascade.CascadeController`. It exists
        because an acceleration demand is not something an actuator-level plant
        can be *handed* - it has to become a moment demand first, or the
        aircraft has no moment at all and tumbles.

    Why both fields are filled by a wrench-producing controller, rather than
    ``accel_cmd`` being left empty or made optional: ``accel_cmd`` is what the
    controller was *asked for* and ``wrench`` is what it decided to *do about
    it*. Keeping the demand on the record is what makes an ideal-acceleration
    run and an actuator-level run of the same controller comparable - same
    number going in, different realisation - which is the only way the two
    control modes can be told apart by reading their logs.

    A controller whose ``command_kind`` is not ``ACCEL`` has no acceleration
    concept at all (the wing's is ``AIRSPEED_NAV``), and fills ``accel_cmd``
    with zeros as the statement of that rather than as a value to read.

    Until this field existed, the wing carried its wrench in
    ``metadata["wrench"]``: an untyped key with a ``KeyError`` for an error
    message, and a second channel that a reader had no way to discover from the
    type. There is one channel now, and it is this one.
    """

    accel_cmd: np.ndarray
    metadata: dict[str, Any] = field(default_factory=dict)
    wrench: Wrench | None = None


@dataclass
class Waypoint:
    """Single waypoint in 3D ENU coordinates."""

    position: np.ndarray


@dataclass
class TrajectoryLog:
    """Legacy simulation result type.

    New code should use :class:`aerial_kit.sim.SimulationResult`. This class is
    retained so existing integrations importing ``aerial_kit.types`` do not break.
    """

    trajectory: np.ndarray
    planned_waypoints: np.ndarray
    obstacles: list[Any]
    space_dim: np.ndarray
    terrain_type: str
    visual_cfg: dict[str, Any]
    goal_position: np.ndarray
    planner_type: str
    final_time: float
    final_waypoint_index: int
    final_waypoint_count: int
    distance_to_goal: float
    collisions_detected: int
    attitude_quats: np.ndarray | None = None
    backend_name: str = ""
