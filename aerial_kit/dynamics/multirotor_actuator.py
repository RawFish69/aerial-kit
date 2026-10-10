"""Actuator-input multirotor plant: motor commands in, motion and samples out.

**The one thing to know before reading further: this plant is NED/FRD.** Every
other dynamics model in this package is ENU (`multirotor.py` says so in its
state docstring, `fixed_wing.py` likewise). This one is not, on purpose. Its
consumer is the firmware's control core, whose estimator, mixer and rate loops
are NED/FRD, and the single most expensive mistake in this whole integration
was a frame convention that both sides documented and neither checked (see
`hosts/nas/agents/04-traps.md` §67). So the plant states its frame in its own
name, natively speaks the firmware's, and leaves the conversion to whoever needs
ENU, where `AERIAL-KIT-GOAL-PROGRESS.md` §C1/C2 already pins it.

Independent by construction
---------------------------
The assessment's finding F8 is that the existing multirotor path computes a
wrench, calls `airframe.allocate`, **discards the return**, and advances the
backend with an acceleration. A hex/octo mission therefore proves nothing about
six or eight actuators. F2 is the same complaint one layer down: a plant that
reuses the controller's own equations agrees with the controller for reasons
that are not about the aircraft.

This module is the answer to both. Its only inputs are bounded per-motor
commands in the firmware's own units (`0..1`, the same array that crosses the
ABI as `akc_outputs_t.motor[]`), and its only forces and moments arise from
those motors. Nothing here is derived from a controller equation, and the
rotation maths is written locally rather than imported from the package under
test, so the two implementations share no code even though they must agree.

What is modelled, and what is not
---------------------------------
Modelled: first-order motor lag, per-motor saturation with the request kept so
an infeasible demand stays visible, translational drag on the velocity relative
to the air, wind (a steady part plus first-order Gauss-Markov gusts from their
own seeded generator), full quaternion kinematics with a general body-rate
transformation, additive gyro and accelerometer noise from a seeded generator,
and a sample delay of an integer number of steps.

Not modelled: blade flapping, inflow, ground effect, propeller thrust curves
that vary with airspeed, ESC timing, or any structural dynamics. Wind acts only
through the drag term: no moment, and no change in rotor thrust. The inertia is
diagonal and constant. This is a *validation* plant for the
actuator path, not a flight-dynamics package.

The two sign conventions that matter
------------------------------------
1. **Thrust is along body -z.** FRD means z points down, so a rotor pushes the
   aircraft *up* with a force along negative z.
2. **The accelerometer field is the specific force negated.** A real inertial
   unit reports `(0, 0, -1)` g at rest and level. The firmware's `accel[3]`
   reports `(0, 0, +1)` - `docs/31-contract.md` C1 is the normative sentence
   ("z reads +1 with the board level and the right way up"). `sense()` applies
   that negation, and `test_multirotor_actuator.py` asserts the level case
   before it asserts anything dynamic, because that is the cheapest possible
   input and the one that was wrong last time.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass, field

import numpy as np

from ..types import Wrench

# The firmware's own ordering for its quad-X mixer, and its spin directions.
#
# `ak_mixer_quad_x` in the firmware numbers its motors rear-right, front-right,
# rear-left, front-left, with yaw coefficients -1, +1, +1, -1. The positions and
# spins below are the geometry those coefficients describe, and
# `test_multirotor_actuator.py` asserts that this plant reproduces all three of
# the mixer's signs rather than trusting the comment.
QUAD_X_SPIN = np.array([-1.0, 1.0, 1.0, -1.0])


def quad_x_positions(arm_length_m: float) -> np.ndarray:
    """Rotor (x, y) in body FRD metres. x forward, y right, so the first row is
    the rear-right motor - the firmware's row 0."""
    d = float(arm_length_m) / np.sqrt(2.0)
    return np.array(
        [
            [-d, +d],  # 0 rear right
            [+d, +d],  # 1 front right
            [-d, -d],  # 2 rear left
            [+d, -d],  # 3 front left
        ],
        dtype=float,
    )


@dataclass
class ActuatorPlantParams:
    """Everything the plant needs that is not geometry."""

    mass_kg: float = 1.0
    # Diagonal inertia in FRD: Ixx (roll), Iyy (pitch), Izz (yaw), kg m^2.
    inertia_kgm2: tuple[float, float, float] = (0.01, 0.01, 0.02)
    # Thrust of one motor at command 1.0, newtons. Hover for a 1 kg quad is
    # 9.81/4 = 2.45 N per motor, so 6.0 leaves real authority above hover.
    max_thrust_per_motor_n: float = 6.0
    # Yaw moment per newton of thrust. Reaction torque, so its sign is the
    # rotor's spin direction - see `spin`.
    yaw_torque_coeff: float = 0.02
    # Command bounds. A quadrotor's mixer has its own idle floor and passes
    # that through; this is the hard bound at the motor.
    motor_min: float = 0.0
    motor_max: float = 1.0
    # First-order lag from commanded to actual motor command. 30 ms is a
    # plausible small outrunner plus prop; it is a round number chosen so the
    # lag is observable at 500 Hz, not a measured value.
    motor_tau_s: float = 0.03
    # Translational drag, world frame, newtons per m/s of airspeed.
    kv_drag: float = 0.1
    # Wind, NED m/s: the air moves at `wind_ned_mps` plus a gust. Each gust
    # axis is a first-order Gauss-Markov process with standard deviation
    # `gust_sigma_mps` and correlation time `gust_tau_s`, drawn once per step
    # from a generator of its own, so turning gusts on does not change the
    # sensor noise sequence. Zero by default, for the same reason the sensor
    # noise is. Wind enters through `kv_drag` only: with no drag it does
    # nothing.
    wind_ned_mps: tuple[float, float, float] = (0.0, 0.0, 0.0)
    gust_sigma_mps: float = 0.0
    gust_tau_s: float = 2.0
    gravity_mps2: float = 9.81
    # Sensor noise, one sigma. Zero by default: a plant that is noisy when you
    # did not ask is a plant whose failures are unreproducible.
    gyro_noise_dps: float = 0.0
    accel_noise_g: float = 0.0
    # Whole steps of delay between the state and the sample handed out.
    sample_delay_steps: int = 0
    seed: int = 0
    # A unilateral ground plane at the NED origin: the surface supplies whatever
    # normal force it must to stop the aircraft descending through it.
    #
    # This is not scenery. A multirotor with its motors stopped *falls*, and a
    # body in free fall has no specific force for an accelerometer to measure -
    # the reading goes to zero and the attitude it implies is unobservable. A
    # real quadrotor on a bench is held up by a table, which is why its
    # accelerometer reads +1 g and why its estimator converges before arming.
    # Without this the plant reproduces the falling case and no arming check
    # can pass, which is a fact about the ground and not about the firmware.
    ground_enabled: bool = True


@dataclass
class ActuatorSample:
    """What the plant would put on the wire, in the firmware's units.

    `sequence` counts samples produced, so a consumer can do what the ABI's
    boundary rules require and refuse a stale one. `t_ms` is plant time, not
    wall time.
    """

    gyro_rps: np.ndarray  # rad/s, body FRD
    accel_g: np.ndarray  # g, firmware convention: +1 in z at level
    body_rates_rps: np.ndarray
    attitude_quat: np.ndarray  # w-first, body-to-NED
    t_ms: float
    sequence: int
    delayed_steps: int  # how many steps old this sample actually is


@dataclass(eq=False)  # numpy fields: the generated __eq__ would raise on them
class ActuatorPlant:
    """A multirotor whose only inputs are bounded motor commands.

    Usage is `step()` then `sense()`; `achieved_wrench()` and
    `commanded_wrench()` are the pair F8 asks for, so a test can assert that an
    infeasible demand was *not* silently satisfied.
    """

    params: ActuatorPlantParams = field(default_factory=ActuatorPlantParams)
    motor_positions: np.ndarray | None = None
    spin: np.ndarray | None = None

    # ---- state, NED/FRD -------------------------------------------------
    position: np.ndarray = field(init=False)
    velocity: np.ndarray = field(init=False)
    attitude_quat: np.ndarray = field(init=False)
    body_rates: np.ndarray = field(init=False)
    motor_actual: np.ndarray = field(init=False)

    # ---- commands and bookkeeping --------------------------------------
    motor_command: np.ndarray = field(init=False)
    # Plant time and the last dt, both kept because a sample has to be able to
    # say *when* it was taken - the ABI's staleness rule is a subtraction
    # against this number and nothing else.
    time_s: float = field(init=False, default=0.0)
    last_dt_s: float = field(init=False, default=0.0)
    steps: int = field(init=False, default=0)
    clamped_last_step: int = field(init=False, default=0)
    # The gust part of the wind, NED m/s, held constant across one step.
    gust: np.ndarray = field(init=False)
    _rng: np.random.Generator = field(init=False, repr=False)
    _gust_rng: np.random.Generator = field(init=False, repr=False)
    _delay: deque = field(init=False, repr=False)

    def __post_init__(self) -> None:
        p = self.params
        if self.motor_positions is None:
            arms = len(QUAD_X_SPIN)
            # 0.2 m is the sibling multirotor's default arm; the plant does not
            # guess from `arms` because row order is airframe-specific.
            self.motor_positions = quad_x_positions(0.2)
        else:
            self.motor_positions = np.asarray(self.motor_positions, dtype=float)
        arms = self.motor_positions.shape[0]
        if self.spin is None:
            if arms != len(QUAD_X_SPIN):
                raise ValueError(
                    f"spin is required for {arms} motors: the default is the "
                    "firmware's quad-X, and inventing a spin order for another "
                    "airframe would be a guess about which props turn which way"
                )
            self.spin = QUAD_X_SPIN.copy()
        else:
            self.spin = np.asarray(self.spin, dtype=float)
        if self.spin.shape != (arms,):
            raise ValueError(f"spin must be {arms}-long, got {self.spin.shape}")
        if p.motor_tau_s <= 0.0:
            raise ValueError("motor_tau_s must be > 0")
        if p.mass_kg <= 0.0:
            raise ValueError("mass_kg must be > 0")
        if p.sample_delay_steps < 0:
            raise ValueError("sample_delay_steps must be >= 0")
        if np.any(np.asarray(p.inertia_kgm2, dtype=float) <= 0.0):
            raise ValueError("every inertia component must be > 0")
        if np.asarray(p.wind_ned_mps, dtype=float).shape != (3,):
            raise ValueError("wind_ned_mps must have 3 components")
        if p.gust_sigma_mps < 0.0:
            raise ValueError("gust_sigma_mps must be >= 0")
        if p.gust_tau_s <= 0.0:
            raise ValueError("gust_tau_s must be > 0")

        self.position = np.zeros(3, dtype=float)
        self.velocity = np.zeros(3, dtype=float)
        self.attitude_quat = np.array([1.0, 0.0, 0.0, 0.0], dtype=float)
        self.body_rates = np.zeros(3, dtype=float)
        # A fresh plant starts with its motors stopped and its command stopped,
        # so the first step is not a step from an arbitrary state.
        self.motor_actual = np.full(arms, p.motor_min, dtype=float)
        self.motor_command = np.full(arms, p.motor_min, dtype=float)
        self._rng = np.random.default_rng(p.seed)
        self._gust_rng = np.random.default_rng([p.seed, 1])
        # Start the gust from its stationary distribution rather than from
        # calm, so the first seconds are not a different experiment.
        self.gust = (
            self._gust_rng.normal(0.0, p.gust_sigma_mps, size=3)
            if p.gust_sigma_mps
            else np.zeros(3, dtype=float)
        )
        self._delay: deque = deque(maxlen=max(1, p.sample_delay_steps + 1))

    # ---- inputs ---------------------------------------------------------
    def set_motors(self, command: np.ndarray) -> None:
        """Set the per-motor command, in the firmware's `0..1` units.

        The command is stored *unclamped*: saturation happens at the motor, and
        `clamped_last_step` reports how many motors could not do what they were
        asked. Clamping at the API instead would make an infeasible request
        indistinguishable from a feasible one, which is F8's whole complaint.
        """
        command = np.asarray(command, dtype=float)
        if command.shape != self.motor_command.shape:
            raise ValueError(
                f"expected {self.motor_command.shape[0]} motor commands, "
                f"got shape {command.shape}"
            )
        if not np.all(np.isfinite(command)):
            raise ValueError("motor command must be finite")
        self.motor_command = command.copy()

    def set_hover(self, fraction: float = 1.0) -> None:
        """Command every motor to the thrust that holds the aircraft up."""
        p = self.params
        per_motor = self.motor_command.shape[0]
        hover_n = p.mass_kg * p.gravity_mps2 / per_motor
        self.set_motors(np.full(per_motor, float(fraction) * hover_n / p.max_thrust_per_motor_n))

    # ---- integration ----------------------------------------------------
    def step(self, dt: float) -> None:
        """Advance by `dt` seconds with RK4 over the full state.

        The motor lag is part of the integrated state rather than a separate
        Euler update, so the lag and the motion are consistent to the same order
        as everything else.
        """
        dt = float(dt)
        if dt <= 0.0:
            raise ValueError("dt must be > 0")

        # Explicit RK4 on the motor lag y' = (u - y)/tau is stable only while
        # dt/tau stays under about 2.785. Past that the lag does not integrate
        # poorly - it diverges, and it does so into a state that looks like a
        # modelling result rather than a numerical one. A caller who sets a
        # 1 ms motor time constant and steps at 500 Hz gets dt/tau = 2 and is
        # fine; a caller who sets 1e-9 gets a NaN aircraft a few steps later,
        # and this is the only place that can say why. See trap 74.
        max_ratio = 2.785
        if dt > max_ratio * self.params.motor_tau_s:
            raise ValueError(
                f"dt={dt:g} s is too large for motor_tau_s="
                f"{self.params.motor_tau_s:g} s: explicit RK4 on the motor lag "
                f"is stable only for dt <= {max_ratio} * motor_tau_s = "
                f"{max_ratio * self.params.motor_tau_s:g} s. Raise the time "
                f"constant or shorten the step."
            )

        # First, before doing any work: refuse a state that is already not
        # finite. The `dt` guard above catches the divergence it knows about,
        # but a nan can arrive another way - a caller assigning one, an
        # unbounded parameter - and would otherwise propagate into the wrench,
        # the sensors and every number downstream while the aircraft went on
        # reporting itself. `_rotmat`'s `n <= 0.0` check does not catch it:
        # `nan <= 0.0` is false.
        self._assert_finite("before", dt)

        p = self.params
        u_cmd = np.clip(self.motor_command, p.motor_min, p.motor_max)
        self.clamped_last_step = int(np.count_nonzero(self.motor_command != u_cmd))

        s = (self.position, self.velocity, self.attitude_quat, self.body_rates, self.motor_actual)

        k1 = self._derivatives(*s, u_cmd)
        s2 = self._advance(s, k1, 0.5 * dt)
        k2 = self._derivatives(*s2, u_cmd)
        s3 = self._advance(s, k2, 0.5 * dt)
        k3 = self._derivatives(*s3, u_cmd)
        s4 = self._advance(s, k3, dt)
        k4 = self._derivatives(*s4, u_cmd)

        # Every k is a fresh array (see `_derivatives`), so writing the five
        # state vectors in place is safe and keeps the arrays the rest of the
        # class holds references to.
        for i in range(len(s)):
            total = (k1[i] + 2.0 * k2[i] + 2.0 * k3[i] + k4[i]) / 6.0
            s[i][...] = s[i] + dt * total

        self.attitude_quat[...] = self.attitude_quat / np.linalg.norm(self.attitude_quat)
        self.motor_actual[...] = np.clip(self.motor_actual, p.motor_min, p.motor_max)

        # The other half of the unilateral constraint: the force above stops the
        # aircraft accelerating into the ground, and this stops it arriving
        # through it. Touchdown is inelastic - a landing gear that bounced would
        # be a spring, and this is a surface.
        if p.ground_enabled and self.position[2] > 0.0:
            self.position[2] = 0.0
            if self.velocity[2] > 0.0:
                self.velocity[2] = 0.0

        self._assert_finite("after", dt)
        self._advance_gust(dt)
        self.steps += 1
        self.time_s += dt
        self.last_dt_s = dt

    def wind(self) -> np.ndarray:
        """The air's velocity now, NED m/s: the steady wind plus the gust."""
        return np.asarray(self.params.wind_ned_mps, dtype=float) + self.gust

    def _advance_gust(self, dt: float) -> None:
        """Exact discretisation of the Gauss-Markov gust over one step."""
        p = self.params
        if not p.gust_sigma_mps:
            return
        decay = np.exp(-dt / p.gust_tau_s)
        self.gust[...] = decay * self.gust + p.gust_sigma_mps * np.sqrt(1.0 - decay * decay) * \
            self._gust_rng.normal(0.0, 1.0, size=3)

    def _assert_finite(self, when: str, dt: float) -> None:
        """Refuse to integrate a state that is not finite.

        Checked at both ends of a step and not only at the end: a nan assigned
        by a caller is caught before a wasted RK4 pass, and one produced *by*
        the pass is caught before it is handed back.
        """
        for name in ("position", "velocity", "attitude_quat", "body_rates",
                     "motor_actual"):
            if not np.all(np.isfinite(getattr(self, name))):
                if when == "after":
                    # Produced by the step that just ran, so the integrator is
                    # the suspect and the fastest state is what sets its limit.
                    hint = (
                        " The usual cause is a step too long for the motor time "
                        "constant (motor_tau_s=%g s), which diverges rather "
                        "than integrating badly; check that before reading "
                        "anything the plant reported." % self.params.motor_tau_s
                    )
                else:
                    # Assigned from outside: the integrator has not run yet, so
                    # naming a time constant here would send the reader to the
                    # wrong place.
                    hint = (
                        " It was already not finite on entry, so this is a "
                        "state that was assigned rather than one the integrator "
                        "produced."
                    )
                raise ValueError(
                    "the plant's %s is not finite %s %d steps at dt=%g s.%s"
                    % (name, when, self.steps, dt, hint)
                )

    @staticmethod
    def _advance(state: tuple, k: tuple, dt: float) -> tuple:
        return tuple(np.asarray(state[i]) + dt * np.asarray(k[i]) for i in range(len(state)))

    def _derivatives(self, position, velocity, quat, omega, motor, u_cmd):
        """Time derivatives of the whole integrated state."""
        p = self.params
        rotor = self._rotor_wrench(motor)
        drag = -p.kv_drag * (np.asarray(velocity, dtype=float) - self.wind())
        non_contact = self._rotmat(quat) @ rotor.force_body + drag
        accel_world = (non_contact + self._ground_force(position, non_contact)) \
            / p.mass_kg + np.array([0.0, 0.0, p.gravity_mps2])

        inertia = np.asarray(p.inertia_kgm2, dtype=float)
        omega = np.asarray(omega, dtype=float)
        # Euler's equations, diagonal inertia: I wdot + w x (I w) = M
        alpha = (rotor.moment_body - np.cross(omega, inertia * omega)) / inertia

        w, x, y, z = quat
        wx, wy, wz = omega
        qdot = 0.5 * np.array(
            [
                -x * wx - y * wy - z * wz,
                w * wx + y * wz - z * wy,
                w * wy - x * wz + z * wx,
                w * wz + x * wy - y * wx,
            ]
        )
        motor_dot = (u_cmd - np.asarray(motor, dtype=float)) / p.motor_tau_s
        # Copies, not views: `dpos/dt` is numerically the velocity vector, and
        # handing back the *same array* would let the in-place state update in
        # `step()` rewrite a k it is still summing.
        return (
            np.array(velocity, dtype=float),
            accel_world,
            qdot,
            alpha,
            motor_dot,
        )

    def _ground_force(self, position, non_contact_world) -> np.ndarray:
        """The normal force the surface supplies, in world NED axes.

        A unilateral constraint rather than a stiff spring: while the aircraft
        is at or below the plane and would otherwise accelerate further into it,
        the surface supplies exactly the force that makes its vertical
        acceleration zero. Exact instead of tuned, and it costs no stiffness
        parameter to get wrong.

        It is a real force, so it appears in the accelerometer exactly the way
        the table under a real quadrotor does - which is the whole point, and
        the reason this is a force here rather than a position clamp somewhere
        downstream of the sensors.
        """
        p = self.params
        if not p.ground_enabled:
            return np.zeros(3)
        # `position[2]` is down, so "on the ground" is z >= 0 and "resting"
        # needs the weight to be pushing into it.
        if position[2] < 0.0:
            return np.zeros(3)
        weight_z = p.mass_kg * p.gravity_mps2
        # z is *down*, so the upward force that holds the aircraft is the
        # negative one. `normal` is that component: it comes out negative when
        # the surface must push (motors off: -(0 + m*g)), and positive when
        # holding the aircraft down would be required - which is thrust already
        # exceeding the weight, and a surface cannot pull.
        normal = -(non_contact_world[2] + weight_z)
        if normal > 0.0:
            return np.zeros(3)
        return np.array([0.0, 0.0, normal])

    # ---- the wrench the actuators actually produce ----------------------
    def _rotor_wrench(self, motor) -> Wrench:
        """Wrench from *actual* motor states. The only source of force here."""
        p = self.params
        thrust = np.asarray(motor, dtype=float) * p.max_thrust_per_motor_n
        total = float(np.sum(thrust))

        pos = self.motor_positions
        # Thrust is along body -z; a rotor at (x, y) contributes r x F with
        # F = (0, 0, -T), which is (-y T, x T, 0).
        mx = float(np.sum(-pos[:, 1] * thrust))
        my = float(np.sum(pos[:, 0] * thrust))
        mz = float(np.sum(self.spin * p.yaw_torque_coeff * thrust))

        return Wrench(
            force_body=np.array([0.0, 0.0, -total]),
            moment_body=np.array([mx, my, mz]),
        )

    def achieved_wrench(self) -> Wrench:
        """What the motors are really producing, after lag and saturation."""
        return self._rotor_wrench(self.motor_actual)

    def commanded_wrench(self) -> Wrench:
        """What the *request* would produce if the aircraft could deliver it.

        Deliberately **not** clipped and **not** lagged: this is the demand,
        including the part the aircraft has no authority for. The pair is what
        makes an infeasible request visible - when these two differ, the
        allocator asked for something that does not exist, and the difference is
        the size of the lie. Clipping here would make the two agree and hide it,
        which is exactly the failure F8 describes.
        """
        return self._rotor_wrench(self.motor_command)

    # ---- outputs --------------------------------------------------------
    def sense(self) -> ActuatorSample:
        """Sample the sensors, apply noise, and apply the sample delay."""
        p = self.params
        quat = self.attitude_quat.copy()
        omega = self.body_rates.copy()

        rot = self._rotmat(quat)
        rotor = self._rotor_wrench(self.motor_actual)
        drag = -p.kv_drag * (self.velocity - self.wind())
        # Specific force: what a real unit measures is (a_world - g), which for
        # a body whose only non-gravitational forces are thrust, drag and the
        # ground is exactly those forces over mass. Then the firmware's
        # negation.
        non_contact = rot @ rotor.force_body + drag
        normal = self._ground_force(self.position, non_contact)
        specific_world = (non_contact + normal) / p.mass_kg
        accel_g = -(rot.T @ specific_world) / p.gravity_mps2

        if p.gyro_noise_dps:
            omega = omega + np.radians(
                self._rng.normal(0.0, p.gyro_noise_dps, size=3)
            )
        if p.accel_noise_g:
            accel_g = accel_g + self._rng.normal(0.0, p.accel_noise_g, size=3)

        # The noise is drawn before the delay, so a delayed sample carries the
        # noise of the moment it was taken rather than the noise of now.
        self._delay.append(
            (omega.copy(), accel_g.copy(), quat.copy(), 1000.0 * self.time_s, self.steps)
        )
        held = self._delay[0]
        age = max(0, self.steps - int(held[4]))

        return ActuatorSample(
            gyro_rps=held[0],
            accel_g=held[1],
            body_rates_rps=omega,
            attitude_quat=held[2],
            t_ms=held[3],
            sequence=self.steps,
            delayed_steps=age,
        )

    # ---- helpers --------------------------------------------------------
    @staticmethod
    def _rotmat(quat) -> np.ndarray:
        """Body-to-NED rotation matrix, w-first quaternion. Written here rather
        than imported so this plant shares no code with the estimator it is
        meant to be independent of."""
        w, x, y, z = np.asarray(quat, dtype=float)
        n = w * w + x * x + y * y + z * z
        if n <= 0.0:
            raise ValueError("quaternion has zero norm")
        s = 2.0 / n
        return np.array(
            [
                [1 - s * (y * y + z * z), s * (x * y - z * w), s * (x * z + y * w)],
                [s * (x * y + z * w), 1 - s * (x * x + z * z), s * (y * z - x * w)],
                [s * (x * z - y * w), s * (y * z + x * w), 1 - s * (x * x + y * y)],
            ]
        )
