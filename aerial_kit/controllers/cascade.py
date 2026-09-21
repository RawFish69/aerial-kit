"""The four-stage cascade: an acceleration demand turned into a wrench.

Every multirotor controller in this repository asks for an *acceleration* and
stops there (``controllers/basic.py``, all four of them). That is the right
contract for the ideal-acceleration backends and it is useless to an
actuator-level plant: a quadrotor cannot be handed "accelerate north at 2 m/s^2"
as a motor command, and if nothing turns that into a moment demand the aircraft
has no moment at all and tumbles. This module is the missing three stages.

    stage 1  position   -> acceleration demand      the inner controller, any of them
    stage 2  acceleration -> attitude + collective  the thrust vector, tilted
    stage 3  attitude   -> body rate demand         the attitude error, as a rate
    stage 4  body rate  -> moment                   the rate error, through the inertia

Everything is z-up, and every number in it is SI
------------------------------------------------
This is the repository's own convention, not the firmware's: world axes ENU,
body axes with ``x`` forward, ``y`` left, ``z`` up, which is what :class:`Wrench`
documents. So the thrust is ``force_body = [0, 0, +T]`` - the *opposite* sign to
the FRD actuator plant, where a rotor pushes along negative z. That sign is
``aerial_kit/dynamics/quad_x_seam.py``'s business and never this module's; a
cascade that flipped it here as well would flip it twice.

Gains are physical, not tuned-to-taste:

* ``k_att`` is in 1/s and is the attitude loop's bandwidth - an attitude error
  of ``e`` radians asks for a body rate of ``k_att * e``.
* ``k_rate`` is in 1/s and is the rate loop's bandwidth. The moment is
  ``I * k_rate * (omega_des - omega)``, which is the diagonal-inertia rigid-body
  law ``M = I * alpha`` with ``alpha = k_rate * omega_error``. Because that is
  what it is, the loop is *stable by bandwidth* rather than by luck: the plant
  has a first-order motor lag with a 30 ms time constant, a 33 rad/s corner, and
  a rate loop at 12 rad/s sits a comfortable factor under it. Raise ``k_rate``
  towards the motor's corner and the loop will ring, which is a fact about the
  motor rather than about the gains.

**The gyroscopic term is deliberately absent.** Euler's equations for a rotating
body are ``I * omega_dot + omega x (I omega) = M``, and the plant integrates
them in exactly that form (``multirotor_actuator.py:_derivatives``). A
controller that also fed the ``omega x (I omega)`` term into its moment demand
would be cancelling the plant's own arithmetic and doubling it in the other
direction. This is a real fork in the road - many flight controllers *do*
include the term, correctly, because their plant does not - and the only safe
rule is that it appears once, on whichever side of the wire the integrator is.

Limits it states about itself
-----------------------------
* **The attitude error is the small-angle form.** ``e_R`` is the skew-symmetric
  part of the rotation error, which is proportional to the error angle only up
  to about 90 degrees and is exactly zero at 180, where a stabilising loop has
  no direction to push. The tilt limit below is what keeps the loop inside its
  basin rather than a hope that demand stays modest.
* **Yaw is damped, not regulated, unless a heading is given.** The desired
  heading defaults to the heading the aircraft already has, so a yaw
  disturbance is damped back to no rotation but a *persistent* heading offset is
  never removed. Pass ``hold_yaw_deg`` to regulate to a fixed heading instead.
* **There is no integral term anywhere.** A steady disturbance produces a steady
  offset, and there is no wind in the plant to produce one. Adding an integrator
  to a loop with a rate limit and a lag is how a flight controller gets a
  wind-up mode, and it is not a thing to do without a measured need.
* **It closes on a state, not on a sensor.** The attitude and rates it uses come
  from whatever ``SimState`` it is handed. Closing this on the actuator plant's
  ``sense()`` needs an estimator, which this repository has on the firmware side
  and not here; see ``aerial_kit/dynamics/actuator_loop.py`` for which of the
  two a given trace was closed on.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Mapping

import numpy as np

from ..dynamics.rotations import quat_to_rotmat, yaw_of
from ..interfaces import Controller
from ..types import CommandKind, ControlTarget, SimState, Waypoint, Wrench
from .basic import PIDController

VERTICAL = np.array([0.0, 0.0, 1.0])


@dataclass(frozen=True)
class CascadeGains:
    """The cascade's physical parameters. ``mass_kg`` has no default, on purpose.

    A wrong mass is not a tuning annoyance: stage 2 builds the hover thrust out
    of it, so a mass that is off by a factor of two is an aircraft that either
    climbs at half a g or falls out of the sky, and it does it while every other
    number in the loop looks reasonable. The plant knows its own mass; this
    refuses to guess it and says so.
    """

    mass_kg: float
    inertia_kgm2: tuple[float, float, float] = (0.01, 0.01, 0.02)
    k_att: float = 4.0
    k_rate: float = 12.0
    max_tilt_deg: float = 35.0
    thrust_floor_frac: float = 0.05
    gravity_mps2: float = 9.81
    hold_yaw_deg: float | None = None

    def __post_init__(self) -> None:
        if self.mass_kg <= 0.0:
            raise ValueError(f"mass_kg must be > 0, got {self.mass_kg}")
        if np.any(np.asarray(self.inertia_kgm2, dtype=float) <= 0.0):
            raise ValueError("every inertia component must be > 0")
        if self.k_att <= 0.0 or self.k_rate <= 0.0:
            raise ValueError(
                f"k_att and k_rate are bandwidths in 1/s and must be > 0, got "
                f"k_att={self.k_att}, k_rate={self.k_rate}"
            )
        # 85 rather than 90: at 90 degrees the thrust axis is horizontal and
        # `rotation_from_thrust_and_yaw` has to cross it with a vector that is
        # parallel to it, so the desired attitude stops being defined rather
        # than merely being extreme.
        if not 0.0 < self.max_tilt_deg <= 85.0:
            raise ValueError(
                f"max_tilt_deg must be in (0, 85], got {self.max_tilt_deg}"
            )
        if not 0.0 <= self.thrust_floor_frac < 1.0:
            raise ValueError(
                f"thrust_floor_frac must be in [0, 1), got {self.thrust_floor_frac}"
            )

    @classmethod
    def from_config(cls, cfg: Mapping[str, Any]) -> "CascadeGains":
        """Read the gains out of a runner ``cfg``, refusing to invent the mass."""
        cascade_cfg = dict((cfg.get("controller", {}) or {}).get("cascade", {}) or {})
        if "mass_kg" not in cascade_cfg:
            raise ValueError(
                "the cascade needs `controller.cascade.mass_kg` and will not "
                "guess it: stage 2 builds the hover thrust from it, so a wrong "
                "mass is an aircraft that does not hold altitude rather than a "
                "loop that flies badly. It must be the plant's own mass_kg."
            )
        known = {f for f in cls.__dataclass_fields__}
        unknown = sorted(set(cascade_cfg) - known)
        if unknown:
            raise ValueError(
                f"unknown cascade setting(s) {unknown}; known are {sorted(known)}"
            )
        inertia = cascade_cfg.get("inertia_kgm2")
        return cls(
            mass_kg=float(cascade_cfg["mass_kg"]),
            **({} if inertia is None else {"inertia_kgm2": tuple(float(i) for i in inertia)}),
            **{
                key: cascade_cfg[key]
                for key in known - {"mass_kg", "inertia_kgm2"}
                if key in cascade_cfg
            },
        )


# ---------------------------------------------------------------------------
# Stage 2: an acceleration demand as a thrust vector
# ---------------------------------------------------------------------------


def thrust_vector(accel_cmd: np.ndarray, gains: CascadeGains) -> tuple[float, np.ndarray]:
    """The total thrust and the body axis it must point along, world frame.

    A rotorcraft's only non-gravitational force is its thrust, so the force that
    produces a demanded acceleration is ``m * (a - g)`` with ``g`` the gravity
    vector. Hover falls straight out of it: ``a = 0`` gives ``m * g`` upwards,
    which is a level axis at exactly the weight. That is the check to run first
    on any change here, because it is the one input whose answer is known before
    the code is written.
    """
    a = np.asarray(accel_cmd, dtype=float).reshape(3)

    # No multirotor can accelerate downwards faster than gravity: it can only
    # take thrust away, and thrust is the thing pushing it up. A demand below
    # that is answered with the floor, and the floor is on the *vertical
    # component* rather than on the resulting magnitude so that the axis stays
    # defined - a horizontal axis has no way to say which way is up, and
    # `rotation_from_thrust_and_yaw` would have nothing to build from.
    a_z_min = -gains.gravity_mps2 * (1.0 - gains.thrust_floor_frac)
    a = np.array([a[0], a[1], max(float(a[2]), a_z_min)], dtype=float)

    force = gains.mass_kg * (a + gains.gravity_mps2 * VERTICAL)
    magnitude = float(np.linalg.norm(force))
    if magnitude <= 0.0:  # unreachable while thrust_floor_frac < 1, kept anyway
        raise ValueError(f"an acceleration demand of {accel_cmd} produced no thrust")
    return magnitude, force / magnitude


def limit_tilt(
    z_body: np.ndarray, max_tilt_deg: float
) -> tuple[np.ndarray, float]:
    """Clamp the thrust axis to a cone about vertical.

    Returns the clamped axis and the factor to apply to the thrust magnitude.

    **The factor is the interesting half.** Left at 1.0, clamping the axis makes
    the aircraft climb harder than it was asked to: the thrust magnitude was
    chosen for the *unclamped* axis's vertical component, and tilting the axis
    back towards vertical increases exactly that component. So the magnitude is
    scaled by ``z_old[2] / z_new[2]``, which holds the vertical force at the
    value the demand asked for and takes the whole of the shortfall out of the
    horizontal acceleration. Altitude is preserved and the turn is what gets
    given up, which is the right way round: a position loop that cannot make its
    turn is off course, and one that cannot make its altitude is in the ground.
    """
    z_body = np.asarray(z_body, dtype=float).reshape(3)
    if not 0.0 < max_tilt_deg <= 85.0:
        raise ValueError(f"max_tilt_deg must be in (0, 85], got {max_tilt_deg}")
    cos_limit = float(np.cos(np.radians(max_tilt_deg)))
    if z_body[2] >= cos_limit:
        return z_body, 1.0

    horizontal = z_body[:2]
    spread = float(np.linalg.norm(horizontal))
    if spread <= 0.0:  # z_body = (0, 0, -1): pointing straight down
        raise ValueError(
            "the thrust axis points straight down, which is not a tilt that can "
            "be clamped - a demand like this is a sign error upstream, not a "
            "demand to limit"
        )
    sin_limit = float(np.sin(np.radians(max_tilt_deg)))
    clamped = np.array(
        [horizontal[0] * sin_limit / spread, horizontal[1] * sin_limit / spread, cos_limit]
    )
    return clamped, float(z_body[2] / cos_limit)


def rotation_from_thrust_and_yaw(z_body: np.ndarray, yaw_rad: float) -> np.ndarray:
    """The desired body-to-world rotation: thrust axis plus a heading.

    The thrust axis fixes two of the three axes and a heading fixes the third,
    which is all an aircraft with four rotors has to spend. The heading is
    carried as a world-frame horizontal direction and re-orthogonalised against
    the thrust axis, so the two constraints cannot fight: the axis wins and the
    heading is what gets tilted.
    """
    z_body = np.asarray(z_body, dtype=float).reshape(3)
    z_body = z_body / np.linalg.norm(z_body)
    heading = np.array([np.cos(yaw_rad), np.sin(yaw_rad), 0.0], dtype=float)

    y_body = np.cross(z_body, heading)
    norm = float(np.linalg.norm(y_body))
    if norm < 1e-9:
        # Thrust axis within 1e-9 of horizontal along the heading: no heading
        # information survives. The tilt limit makes this unreachable, and it
        # raises rather than picking an arbitrary roll.
        raise ValueError(
            f"the thrust axis {z_body} is parallel to the heading, so there is "
            f"no attitude that satisfies both (max_tilt_deg must be < 90)"
        )
    y_body = y_body / norm
    x_body = np.cross(y_body, z_body)
    return np.column_stack([x_body, y_body, z_body])


# ---------------------------------------------------------------------------
# Stage 3: an attitude error as a body rate demand
# ---------------------------------------------------------------------------


def attitude_error(desired: np.ndarray, current: np.ndarray) -> np.ndarray:
    """``e_R = 0.5 * vee(R_d^T R - R^T R_d)``, in the body frame, in radians.

    The sign, derived rather than remembered. Take the aircraft to be at the
    desired attitude rotated by a small ``+eps`` about body x, so
    ``R = R_d Rx(eps)``. Then ``R_d^T R = Rx(eps)`` and ``R^T R_d = Rx(-eps)``,
    whose difference has rotation vector ``2 sin(eps) x_hat``, so
    ``e_R = +sin(eps) x_hat`` - ``+eps x_hat`` to first order, and ``sin``
    rather than ``eps`` exactly, because the skew part of a rotation is its
    sine. Correcting that error means rotating by ``-eps`` about x, so the rate
    demand is ``omega_des = -k_att * e_R`` -- the minus sign is doing real work,
    and getting it wrong is not a subtle instability but an aircraft that flies
    away from its setpoint at a rate set by ``k_att``.

    Written as the skew part rather than as ``log(R_d^T R)`` so that it needs no
    branch near pi and no matrix logarithm; the price is stated in the module
    docstring - it is the small-angle form.

    **``R_d^T R`` and not ``R_d R^T``, and the difference is not a sign.** The
    two are conjugate - ``R_d R^T = R_d (R_d^T R) R_d^T`` - so they are the same
    rotation with the same angle and *different axes*: ``R_d^T R`` is the error
    in the aircraft's own body axes, which is what a rate demand is written in,
    and ``R_d R^T`` is the same error in world axes. Their rotation vectors
    coincide only when ``R = I``, which is precisely the attitude a test is
    most likely to set up - so getting this wrong passes a level-at-heading-zero
    test and produces a roll demand on a pitch axis for an aircraft facing any
    other way. This function had exactly that bug, and the lateral step in
    `test_cascade.py` is what found it.
    """
    error = np.asarray(desired, dtype=float).T @ np.asarray(current, dtype=float)
    skew = error - error.T
    return 0.5 * np.array([skew[2, 1], skew[0, 2], skew[1, 0]], dtype=float)


# ---------------------------------------------------------------------------
# The controller
# ---------------------------------------------------------------------------


class CascadeController(Controller):
    """An accel-level controller and three more loops underneath it.

    ``inner`` is any controller that returns an ``accel_cmd`` - all four of
    ``controllers/basic.py``, unchanged - so which outer loop gets flown is a
    choice of what to wrap rather than a fifth controller to write. Its
    ``command_kind`` defaults to the inner controller's, because the *airframe*
    contract is the inner controller's: this wrapper is handed the same state,
    the same waypoint and the same config, and it is what the airframe's
    allocator ultimately gets a wrench from.

    The ``ControlTarget`` it returns carries both halves: ``accel_cmd`` is the
    inner controller's demand, untouched, and ``wrench`` is what this loop
    decided to do about it. See ``ControlTarget``'s docstring for why both.
    """

    def __init__(
        self,
        inner: Controller | None = None,
        gains: CascadeGains | None = None,
    ) -> None:
        self.inner = PIDController() if inner is None else inner
        self.gains = gains
        self.command_kind: CommandKind = self.inner.command_kind
        # The last step's intermediate values, for a trace to read. Kept as a
        # plain dict so that a caller who wants them does not have to recompute
        # the stages and risk recomputing them differently.
        self.last_stages: dict[str, Any] = {}

    def compute(
        self,
        state: SimState,
        target_waypoint: Waypoint,
        cfg: Mapping[str, Any],
    ) -> ControlTarget:
        gains = self.gains if self.gains is not None else CascadeGains.from_config(cfg)
        accel_cmd = np.asarray(
            self.inner.compute(state, target_waypoint, cfg).accel_cmd, dtype=float
        ).reshape(3)

        rotation = self._attitude_of(state)
        rates = self._rates_of(state)

        thrust_n, z_body = thrust_vector(accel_cmd, gains)
        z_body, scale = limit_tilt(z_body, gains.max_tilt_deg)
        thrust_n *= scale

        yaw = (
            np.radians(gains.hold_yaw_deg)
            if gains.hold_yaw_deg is not None
            else yaw_of(rotation)
        )
        desired = rotation_from_thrust_and_yaw(z_body, yaw)

        error = attitude_error(desired, rotation)
        omega_des = -gains.k_att * error
        moment = (
            np.asarray(gains.inertia_kgm2, dtype=float)
            * gains.k_rate
            * (omega_des - rates)
        )

        self.last_stages = {
            "accel_cmd": accel_cmd,
            "thrust_n": thrust_n,
            "tilt_deg": float(np.degrees(np.arccos(np.clip(z_body[2], -1.0, 1.0)))),
            "tilt_limited": scale < 1.0,
            "yaw_rad": yaw,
            "attitude_error_rad": error,
            "omega_des_rps": omega_des,
            "moment_body": moment,
        }
        return ControlTarget(
            accel_cmd=accel_cmd,
            wrench=Wrench(
                force_body=np.array([0.0, 0.0, thrust_n], dtype=float),
                moment_body=moment,
            ),
            metadata={
                "controller": "cascade",
                "inner": type(self.inner).__name__,
                "thrust_n": thrust_n,
                "tilt_deg": self.last_stages["tilt_deg"],
                "attitude_error_rad": float(np.linalg.norm(error)),
            },
        )

    # -- the state it needs, refused rather than defaulted ------------------
    def _attitude_of(self, state: SimState) -> np.ndarray:
        if state.attitude_quat is None:
            raise ValueError(
                "the cascade needs an attitude: it closes an attitude and rate "
                "loop, and a state with attitude_quat=None is one that only an "
                "acceleration-level controller can be flown on. The inner "
                "controller would have accepted this state happily - that is "
                "the difference between the two contracts, and it is why this "
                "raises instead of assuming level flight."
            )
        quat = np.asarray(state.attitude_quat, dtype=float).reshape(4)
        norm = float(np.linalg.norm(quat))
        if not np.isfinite(norm) or norm < 1e-6:
            raise ValueError(f"attitude_quat {state.attitude_quat} is not a rotation")
        return quat_to_rotmat(quat / norm)

    @staticmethod
    def _rates_of(state: SimState) -> np.ndarray:
        if state.body_rates is None:
            raise ValueError(
                "the cascade needs body rates: stage 4 damps the rate error, "
                "and with no rate to damp from there is no stability margin in "
                "the attitude loop at all. Pass zeros if the aircraft is known "
                "to be at rest - do not pass None and let this guess."
            )
        return np.asarray(state.body_rates, dtype=float).reshape(3)
