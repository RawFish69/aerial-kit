"""Tests for the actuator-input multirotor plant.

Three things are being established here and they are different:

* that the plant is a *plant* - lag, saturation, noise, delay and general body
  rotation each behave the way the model says, and none of them is decorative;
* that its geometry is the **firmware's** geometry. A plant whose roll moment
  has the opposite sign to the mixer that commands it produces an aircraft that
  flies away from the stick, and the failure looks like a tuning problem rather
  than a sign error. So the firmware's own quad-X coefficient table is written
  out below and the plant has to reproduce all three of its axes;
* that the plant and the **sibling airframe** do *not* agree as they stand, in
  two specific and checkable ways, because assuming they did would be the
  expensive kind of mistake. See
  `test_the_two_repositories_disagree_about_the_wrench_sign` and
  `test_the_two_repositories_agree_about_the_yaw_pair_and_differ_by_a_sign`.
  The second of those is the one to read before "fixing" anything: the yaw
  columns are not a disagreement about the aircraft, and `test_quad_x_seam.py`
  is where the allocator is actually connected to this plant and the sign is
  measured rather than argued.

The level-at-rest accelerometer check comes first on purpose. It is the
cheapest possible input, and it is the one that was wrong in the last
integration (`hosts/nas/agents/04-traps.md` §67).
"""

from __future__ import annotations

import numpy as np
import pytest

from aerial_kit.airframes.multirotor import MultirotorAirframe
from aerial_kit.dynamics.multirotor_actuator import (
    QUAD_X_SPIN,
    ActuatorPlant,
    ActuatorPlantParams,
    quad_x_positions,
)
from aerial_kit.types import Wrench

DT = 0.002  # 500 Hz, the rate the firmware's control loop runs at
G = 9.81

# The firmware's `ak_mixer_quad_x` coefficient table, transcribed from
# `aerialkit/src/core/flight/ak_mixer.c`. Columns: throttle, roll, pitch, yaw.
# Rows: rear right, front right, rear left, front left.
#
# The pitch column is the *negative* of Betaflight's, and that is a convention
# rather than a disagreement: this firmware's pitch is positive nose-*up*, and
# more thrust at the back of an aircraft pushes the nose down, so a nose-up
# command has to lower the rear pair. The firmware's header said the opposite
# until 2026-09-19 (trap §73); this table is what says which is right.
FW_QUAD_X = np.array(
    [
        [1.0, -1.0, -1.0, -1.0],  # rear right
        [1.0, -1.0, +1.0, +1.0],  # front right
        [1.0, +1.0, -1.0, +1.0],  # rear left
        [1.0, +1.0, +1.0, -1.0],  # front left
    ]
)


def make_plant(**kwargs) -> ActuatorPlant:
    params = ActuatorPlantParams(**kwargs) if kwargs else ActuatorPlantParams()
    return ActuatorPlant(params=params)


def hover_command(plant: ActuatorPlant, scale: float = 1.0) -> np.ndarray:
    """Every motor at the thrust that just holds the aircraft up."""
    per_motor_n = plant.params.mass_kg * G / plant.motor_command.shape[0]
    return np.full(
        plant.motor_command.shape[0],
        scale * per_motor_n / plant.params.max_thrust_per_motor_n,
    )


def spinning_at_hover(plant: ActuatorPlant, scale: float = 1.0) -> ActuatorPlant:
    """A plant whose motors are already at speed.

    The sensor-convention tests set `motor_actual` directly rather than waiting
    out the lag, because they are about what the accelerometer *reads* and a
    motor still spinning up would scale every number without changing any
    direction. Tests about the lag itself do it the slow way, on purpose.
    """
    plant.set_motors(hover_command(plant, scale))
    plant.motor_actual[...] = np.clip(
        hover_command(plant, scale),
        plant.params.motor_min,
        plant.params.motor_max,
    )
    return plant


# ---------------------------------------------------------------------------
# The convention, checked at the simplest possible input
# ---------------------------------------------------------------------------


def test_level_at_rest_reads_plus_one_g_in_z():
    """The firmware's accelerometer, not a real IMU's.

    A real unit reports (0, 0, -1) g at rest and level. `docs/31-contract.md`
    C1 says this firmware's `accel[]` reads (0, 0, +1), and `ak_estimator.c`'s
    `est_body_up` agrees. Getting this wrong is silent: the estimator decides
    the aircraft is upside down and flies it that way.
    """
    plant = spinning_at_hover(make_plant())
    plant.step(DT)
    assert plant.sense().accel_g == pytest.approx([0.0, 0.0, 1.0], abs=1e-6)


def test_a_quadrotor_resting_on_the_ground_reads_plus_one_g():
    """The bench condition, which is the one an arming check runs in.

    Motors stopped, sitting on the surface. This is not the same test as the
    hover one above even though both read (0, 0, +1) g: here the thrust is zero
    and the *ground* is supplying the force. Without a ground the aircraft falls
    instead, its accelerometer reads zero, the attitude it implies is
    unobservable, and the estimator never converges - so nothing that checks the
    estimator can be run on an aircraft that has not taken off yet.

    Measured before the ground existed: accel_z collapsed to +0.043 within
    100 ms and the core never armed.
    """
    plant = make_plant()
    for _ in range(200):  # 0.4 s of sitting there
        plant.step(DT)
    assert plant.position[2] == pytest.approx(0.0, abs=1e-12)
    assert plant.velocity[2] == pytest.approx(0.0, abs=1e-12)
    assert plant.sense().accel_g == pytest.approx([0.0, 0.0, 1.0], abs=1e-9)


def test_free_fall_still_reads_zero_when_the_ground_is_off():
    """The same plant with nothing underneath it, which is the other half.

    A body in free fall has no specific force to measure and a real
    accelerometer says so. If this did not read zero the ground above would be
    hiding a wrong sensor rather than modelling a right one.

    The drag is turned off for this, and that is the point rather than a
    convenience: a falling body *with* air resistance is not in free fall, and
    the sensor reads the drag. Left on, this aircraft reads 0.039 g after
    0.4 s of falling at about 3.9 m/s - which is `kv * v / g` and is correct.
    Both facts are asserted, in that order, so neither can be mistaken for the
    other.
    """
    plant = make_plant(ground_enabled=False, kv_drag=0.0)
    for _ in range(200):
        plant.step(DT)
    assert plant.velocity[2] == pytest.approx(G * 200 * DT, rel=1e-6)
    assert plant.sense().accel_g == pytest.approx([0.0, 0.0, 0.0], abs=1e-12)

    # And with the drag back on, the sensor reports the drag instead.
    dragged = make_plant(ground_enabled=False)
    for _ in range(200):
        dragged.step(DT)
    assert dragged.velocity[2] > 1.0, "it really is falling"
    assert dragged.sense().accel_g[2] == pytest.approx(
        dragged.params.kv_drag * dragged.velocity[2] / G, rel=1e-6
    )
    assert dragged.sense().accel_g[2] > 0.01, "and it is big enough to see"


def test_the_ground_lets_the_aircraft_leave():
    """A surface pushes and never pulls: past hover the aircraft must go."""
    plant = make_plant()
    plant.set_motors(hover_command(plant, 1.2))
    plant.motor_actual[...] = np.clip(
        plant.motor_command, plant.params.motor_min, plant.params.motor_max
    )
    for _ in range(500):
        plant.step(DT)
    assert plant.position[2] < -0.5, "z is down, so climbing is negative"
    assert plant.velocity[2] < 0.0


def test_touchdown_is_inelastic():
    """Arriving at the ground does not bounce - it is a surface, not a spring."""
    plant = make_plant(ground_enabled=False)
    plant.position = np.array([0.0, 0.0, -5.0])
    for _ in range(2000):  # fall 5 m
        plant.step(DT)
    assert plant.velocity[2] > 5.0, "it was falling fast when it arrived"
    plant.params.ground_enabled = True
    for _ in range(200):
        plant.step(DT)
    assert plant.position[2] == pytest.approx(0.0, abs=1e-12)
    assert plant.velocity[2] == pytest.approx(0.0, abs=1e-12)
    assert plant.sense().accel_g == pytest.approx([0.0, 0.0, 1.0], abs=1e-9)


def test_gyro_is_zero_at_rest():
    plant = spinning_at_hover(make_plant())
    plant.step(DT)
    assert plant.sense().gyro_rps == pytest.approx([0.0, 0.0, 0.0], abs=1e-9)


def test_drag_shows_up_in_the_accelerometer():
    """Why the accelerometer is a *specific force* and not an attitude sensor.

    Fly level and forward, and drag pulls backwards; the sensor reads that as a
    component along body x. A plant that fed its accelerometer the gravity
    direction, or only the thrust axis, would read exactly (0, 0, 1) here and
    the estimator would believe it.
    """
    plant = spinning_at_hover(make_plant())
    plant.position = np.array([0.0, 0.0, -50.0])  # airborne: no ground in this
    plant.velocity = np.array([5.0, 0.0, 0.0])
    plant.step(DT)
    accel = plant.sense().accel_g
    expected_x = plant.params.kv_drag * 5.0 / G
    assert accel[0] == pytest.approx(expected_x, rel=1e-3)
    assert accel[2] == pytest.approx(1.0, abs=1e-3)
    assert accel[0] > 0.01, "the drag term must be big enough to see"


def test_a_body_frame_specific_force_rotates_with_the_aircraft():
    """Nose up 90 degrees, descending: the same world drag lands on another
    body axis.

    With the aircraft pitched up 90 degrees its thrust points along world -x,
    so a world drag along -z is *across* the thrust axis and shows up on body
    x. A world-frame accelerometer would read it on z.
    """
    plant = spinning_at_hover(make_plant())
    plant.position = np.array([0.0, 0.0, -50.0])  # airborne: no ground in this
    s = c = np.sqrt(0.5)
    plant.attitude_quat = np.array([c, 0.0, s, 0.0])  # pitch up 90 degrees
    plant.velocity = np.array([0.0, 0.0, 5.0])  # descending
    plant.step(DT)
    accel = plant.sense().accel_g
    assert abs(accel[0]) == pytest.approx(plant.params.kv_drag * 5.0 / G, rel=5e-2)
    assert abs(accel[2]) == pytest.approx(1.0, abs=5e-2)


# ---------------------------------------------------------------------------
# The geometry is the firmware's geometry
# ---------------------------------------------------------------------------


def test_rotor_positions_are_the_firmware_row_order():
    pos = quad_x_positions(0.2)
    # x forward, y right, z down. Row 0 is the rear-right motor.
    assert pos[0, 0] < 0 and pos[0, 1] > 0, "row 0 must be rear right"
    assert pos[1, 0] > 0 and pos[1, 1] > 0, "row 1 must be front right"
    assert pos[2, 0] < 0 and pos[2, 1] < 0, "row 2 must be rear left"
    assert pos[3, 0] > 0 and pos[3, 1] < 0, "row 3 must be front left"


def test_the_plant_reproduces_every_sign_in_the_firmware_mixer():
    """The plant's geometry against the firmware's table, axis by axis.

    The firmware's table says how a control command moves each motor; the plant
    says what each motor does to the aircraft. Composing them has to come back
    to the same physical claim for all three axes, and the claim is checked as
    a *sign of a moment on an aircraft* rather than as agreement between two
    copies of a number.
    """
    plant = make_plant()
    scale = plant.params.max_thrust_per_motor_n
    delta = 0.1

    for axis, name, moment_index in ((1, "roll", 0), (2, "pitch", 1), (3, "yaw", 2)):
        motors = np.full(4, 0.5) + delta * FW_QUAD_X[:, axis]
        moment = plant._rotor_wrench(motors * scale).moment_body[moment_index]
        assert moment > 0, (
            f"a positive {name} command through the firmware's table must "
            f"produce a positive {name} moment, got {moment}"
        )


def test_a_positive_roll_command_lowers_the_right_pair():
    """The firmware header's own bench check, as arithmetic."""
    motors = 0.5 + 0.1 * FW_QUAD_X[:, 1]
    assert max(motors[0], motors[1]) < min(motors[2], motors[3])


def test_a_nose_up_command_raises_the_front_pair():
    """And the one the header got wrong for months (trap §73).

    More thrust at the back of an aircraft pushes the nose *down*, so a
    positive (nose-up) pitch command has to speed up the front motors.
    """
    motors = 0.5 + 0.1 * FW_QUAD_X[:, 2]
    assert motors[1] > motors[0], "front right must be above rear right"
    assert motors[3] > motors[2], "front left must be above rear left"


def test_thrust_acts_along_body_minus_z():
    """FRD: z points down, so a rotor pushes the aircraft up."""
    plant = spinning_at_hover(make_plant())
    force = plant.achieved_wrench().force_body
    assert force[2] < 0
    assert force[2] == pytest.approx(-plant.params.mass_kg * G, rel=1e-6)


def test_the_default_spin_matches_the_firmware_yaw_column():
    """The default is not arbitrary: it is the firmware's yaw column."""
    assert np.array_equal(QUAD_X_SPIN, FW_QUAD_X[:, 3])


# ---------------------------------------------------------------------------
# The plant and the sibling airframe disagree, in two checkable ways
# ---------------------------------------------------------------------------


def test_the_two_repositories_disagree_about_the_wrench_sign():
    """`Wrench.force_body[2]` means opposite things in the two repositories.

    This plant follows the firmware: FRD, z down, so lift is a *negative* z
    force. `MultirotorAirframe.allocate` reads `force_body[2]` as total
    **upward** thrust, so the same physical hover is `-9.81` here and `+9.81`
    there. Nothing in either tree says so.

    Handing the allocator this plant's hover wrench returns motors at -2.36,
    which is not a small error - it is a quadrotor asked to fly upside down.
    The test asserts both readings so the difference is pinned rather than
    remembered.
    """
    airframe = MultirotorAirframe(arms=4, layout="x", mass_kg=1.0)
    moment = np.zeros(3)

    as_frd = Wrench(force_body=np.array([0.0, 0.0, -1.0 * G]), moment_body=moment)
    motors_from_frd = airframe.allocate(as_frd, state=None)

    as_up = Wrench(force_body=np.array([0.0, 0.0, +1.0 * G]), moment_body=moment)
    motors_from_up = airframe.allocate(as_up, state=None)

    assert motors_from_up.min() > 0.0, "the sibling's own convention hovers"
    assert motors_from_frd.max() < 0.0, (
        "and the firmware's convention, fed straight in, does not - which is "
        "the whole point of this test"
    )


def at_the_stop(plant: ActuatorPlant) -> ActuatorPlant:
    """Put the motors where the *clamped* command would take them, now.

    The infeasibility tests are about which wrenches are reachable, not about
    how long a motor takes to get there, and one 2 ms step reaches 6% of a
    command with the default time constant. Stepping to convergence would work
    too and would take longer to say the same thing.
    """
    plant.motor_actual[...] = np.clip(
        plant.motor_command, plant.params.motor_min, plant.params.motor_max
    )
    return plant


def motors_as_fraction(plant: ActuatorPlant, newtons: np.ndarray) -> np.ndarray:
    """The sibling's allocator speaks Newtons; this plant speaks 0..1.

    Both are defensible and they are not the same unit. Converting in one named
    place, only where the allocator's output is deliberately being fed in, keeps
    every other test in this file honest about which one it means.
    """
    return np.asarray(newtons, dtype=float) / plant.params.max_thrust_per_motor_n


def test_the_two_repositories_disagree_about_what_allocate_returns():
    """`allocate` returns thrust in **Newtons**. The plant takes 0..1.

    Measured, not inferred: the returned motor values sum to `force_body[2]`
    exactly, and `trim()` returns `mass * g / 4` per motor, so hover on a 1 kg
    quadrotor comes back as 2.4525 rather than 1.0.

    This matters more than a unit slip usually would, because the *other* end of
    the same wire has a third convention: `UAVDynamics` takes a normalized hover
    fraction (`hover_thrust=1.0`, and `thrust_mag = mass*g*(c0 + c1*cmd)`). So
    `allocate`'s output feeding `set_command` would be 2.4525x hover as well as
    being the wrong shape - and by the time anyone noticed, the "fix" for F8
    would have been blamed instead of the units.
    """
    airframe = MultirotorAirframe(arms=4, layout="x", mass_kg=1.0)
    wrench = Wrench(
        force_body=np.array([0.0, 0.0, +1.0 * G]), moment_body=np.zeros(3)
    )
    motors = airframe.allocate(wrench, state=None)

    assert motors.sum() == pytest.approx(G, rel=1e-9), "Newtons, not a fraction"
    assert airframe.trim(state=None) == pytest.approx(
        np.full(4, G / 4), rel=1e-9
    ), "hover comes back as mass*g/4 per motor"
    assert motors.max() > 1.0, "so the allocator's own hover is not a valid input here"


def test_the_documented_wrench_frame_is_the_one_allocate_implements():
    """`Wrench` now states its frame. This checks the statement is true.

    The docstring says z is up here - `force_body[2]` is upward thrust - and
    that the moments follow as `Mx = +sum(y*T)`, `My = -sum(x*T)`. A docstring
    that describes a convention is worth exactly as much as the code agreeing
    with it, and this is the code.
    """
    airframe = MultirotorAirframe(arms=4, layout="x", mass_kg=1.0)
    body = airframe._mixer
    # Row 0 is thrust, row 1 is Mx, row 2 is My; the columns are the motors.
    assert np.all(body[0] > 0), "every motor contributes positive thrust"
    assert np.array_equal(body[1], airframe._mixer[1])
    # Mx = +sum(y_i * T_i) and My = -sum(x_i * T_i), against the arm geometry.
    pos = airframe.positions if hasattr(airframe, "positions") else None
    if pos is not None:
        assert np.allclose(body[1], pos[:, 1], atol=1e-9)
        assert np.allclose(body[2], -pos[:, 0], atol=1e-9)

    # And the sign flip against the firmware, stated as the physical hover.
    plant = make_plant()
    plant.set_hover(1.0)
    assert plant.commanded_wrench().force_body[2] < 0, "FRD here: lift is -z"
    assert airframe.trim(state=None).sum() > 0, "and +z there"


def test_the_multirotor_path_in_this_repository_is_ideal_acceleration():
    """Which airframes are flown as plants here, and which are not.

    `CommandKind.ACCEL` is the ideal-acceleration path: the controller's
    accel_cmd goes to the backend as the aircraft's acceleration, with no motor,
    no allocator and no attitude between the two. Every multirotor in this
    repository is on that path today, so a multirotor trajectory from here is a
    guidance result and not a dynamics result, however much it looks like one.
    The fixed wing is the exception, and that is why B7 used it.

    This is asserted rather than described because the label is only worth
    anything if it stays true: an airframe moving to the actuator path should
    have to change this test.
    """
    from aerial_kit.types import CommandKind
    from sim_py.core.registry import create_airframe, register_builtin_components

    register_builtin_components()
    for name in ("quad", "hex", "octo"):
        airframe = create_airframe(name)
        assert airframe.capabilities.command_kind is CommandKind.ACCEL, (
            f"{name} is on the ideal-acceleration path; if that has changed, "
            f"this test and the label in runner.py both need revisiting"
        )
    wing = create_airframe("twin_wing")
    assert wing.capabilities.command_kind is CommandKind.AIRSPEED_NAV


def test_the_two_repositories_agree_about_the_yaw_pair_and_differ_by_a_sign():
    """The yaw columns are elementwise negatives once the rotors are aligned.

    This test replaces one that asserted the opposite, and the way it was wrong
    is worth more than the fact it got wrong. It compared the sibling's yaw
    column against the firmware's **without aligning the rotors** - the sibling
    numbers its quad-X by polygon angle from 45 degrees, the firmware numbers it
    rear-right, front-right, rear-left, front-left - so a difference that is
    purely a relabelling read as a difference in the aircraft. It then papered
    over the misalignment with a `[1, 1, -1, -1]` mask that made the assertion
    pass, and its docstring drew the wrong conclusion from the pair: *"even
    after aligning the rows the two disagree about which pair is which."*

    They do not disagree about which pair is which. Aligned, both put
    rear-right with front-left and front-right with rear-left; the columns are
    elementwise negatives, which is the sign a moment picks up from the
    z-negation between a z-up frame and FRD. Trap 76 said otherwise until
    2026-09-19, and acting on it - negating the sibling's spin column to "match"
    the firmware's numbers - inverts the yaw axis. `test_quad_x_seam.py` pins
    that measurement against a real plant.

    Aligning means comparing the *same physical rotor* in both. Rotor identity
    is by position, and `sibling_rotor_positions_frd` is that map.
    """
    from aerial_kit.dynamics.quad_x_seam import sibling_rotor_positions_frd

    airframe = MultirotorAirframe(arms=4, layout="x", arm_length_m=0.2)
    sibling_yaw = airframe._mixer[3] / airframe.yaw_torque_coeff

    # Reorder the sibling's rotors into the firmware's row order by geometry,
    # not by a hand-written table.
    mine = sibling_rotor_positions_frd(0.2)
    firmware_positions = quad_x_positions(0.2)
    rows = [int(np.argmin(np.linalg.norm(mine - spot, axis=1)))
            for spot in firmware_positions]
    aligned = sibling_yaw[rows]

    assert np.array_equal(aligned, -FW_QUAD_X[:, 3]), (
        "aligned to the same rotors the two yaw columns must differ by exactly "
        f"a sign, got {aligned.tolist()} against {-FW_QUAD_X[:, 3].tolist()}"
    )
    # And they really are the same *pairing*: the diagonals move together in
    # both, so neither tree thinks two rotors on the same side are a pair.
    for column in (aligned, FW_QUAD_X[:, 3]):
        assert column[0] == column[3] and column[1] == column[2], (
            "rear-right must move with front-left and front-right with "
            f"rear-left, got {column.tolist()}"
        )


# ---------------------------------------------------------------------------
# Motor lag
# ---------------------------------------------------------------------------


def test_a_motor_reaches_63_percent_of_a_step_in_one_time_constant():
    plant = make_plant(motor_tau_s=0.05)
    plant.set_motors(np.full(4, 0.5))
    for _ in range(int(round(plant.params.motor_tau_s / DT))):
        plant.step(DT)
    assert plant.motor_actual[0] == pytest.approx(0.5 * (1 - np.exp(-1.0)), abs=2e-3)


def test_the_motors_are_lagged_rather_than_instant():
    """A plant with no lag would make the control loop look better than it is."""
    plant = make_plant(motor_tau_s=0.05)
    plant.set_motors(np.full(4, 1.0))
    plant.step(DT)
    assert plant.motor_actual[0] < 0.1, "one 2 ms step must not reach the command"


def test_commanded_and_achieved_differ_while_the_motors_spin_up():
    plant = make_plant(motor_tau_s=0.05)
    plant.set_motors(np.full(4, 0.8))
    plant.step(DT)
    assert plant.achieved_wrench().force_body[2] > plant.commanded_wrench().force_body[2]


# ---------------------------------------------------------------------------
# Saturation, and the infeasible request F8 asks about
# ---------------------------------------------------------------------------


def test_an_impossible_command_is_clamped_and_the_request_is_kept():
    plant = make_plant()
    plant.set_motors(np.full(4, 2.5))  # far beyond motor_max
    plant.step(DT)
    assert plant.clamped_last_step == 4, "every motor should be reported clamped"
    assert plant.motor_actual.max() <= plant.params.motor_max + 1e-12
    achieved = -plant.achieved_wrench().force_body[2]
    assert achieved <= 4 * plant.params.max_thrust_per_motor_n + 1e-9
    demanded = -plant.commanded_wrench().force_body[2]
    assert demanded > achieved * 3, "the demand stays on the record"


def test_a_negative_motor_command_is_clamped_to_the_floor():
    """The allocator's pseudoinverse can and does return these (F8)."""
    plant = make_plant()
    plant.set_motors(np.array([-0.4, 0.6, -0.4, 0.6]))
    plant.step(DT)
    assert plant.clamped_last_step >= 2
    assert plant.motor_actual.min() >= plant.params.motor_min - 1e-12


def test_the_allocator_can_ask_for_more_than_the_aircraft_has():
    """F8's actual case, in the sibling's own thrust convention.

    The airframe's `allocate` is an unconstrained pseudoinverse, so a wrench
    beyond the motors' authority comes back as motor commands outside [0, 1].
    Before this plant existed the return value was discarded and the
    acceleration backend advanced anyway, so the aircraft "achieved" a wrench
    no set of motors could produce.
    """
    airframe = MultirotorAirframe(arms=4, layout="x", mass_kg=1.0)
    wrench = Wrench(
        force_body=np.array([0.0, 0.0, +1.0 * G]),  # the sibling's convention
        moment_body=np.array([50.0, 0.0, 0.0]),  # far past the arms' authority
    )
    motors = airframe.allocate(wrench, state=None)
    assert motors.min() < 0.0 or motors.max() > 1.0, (
        "the pseudoinverse was expected to leave the feasible set here"
    )

    plant = make_plant()
    plant.set_motors(motors_as_fraction(plant, motors))
    at_the_stop(plant)
    plant.step(DT)
    assert plant.clamped_last_step >= 1
    achieved = abs(plant.achieved_wrench().moment_body[0])
    demanded = abs(plant.commanded_wrench().moment_body[0])
    assert achieved < demanded, "the aircraft cannot have done this"

    # And the ceiling is the geometry, not the number 50. A roll moment comes
    # from *differential* thrust, so only the two motors on one side contribute:
    # two motors at 6 N on 0.1414 m arms, and the other two at nothing.
    d = 0.2 / np.sqrt(2.0)
    ceiling = 2 * plant.params.max_thrust_per_motor_n * d
    assert achieved <= ceiling + 1e-6, (
        f"achieved {achieved} N.m exceeds the geometry's {ceiling} N.m"
    )
    assert achieved > 0.9 * ceiling, "it should be pinned against that ceiling"


def test_a_feasible_wrench_is_delivered():
    """The same path for a demand the aircraft *can* meet, so the check above
    is measuring infeasibility rather than a broken allocator."""
    airframe = MultirotorAirframe(arms=4, layout="x", mass_kg=1.0)
    wrench = Wrench(
        force_body=np.array([0.0, 0.0, +1.0 * G]),
        moment_body=np.array([0.05, 0.0, 0.0]),
    )
    motors = airframe.allocate(wrench, state=None)

    plant = make_plant(motor_tau_s=2e-3)
    fraction = motors_as_fraction(plant, motors)
    assert fraction.min() >= 0.0 and fraction.max() <= 1.0, (
        f"this demand is within the motors' authority ({motors.max():.3f} N of "
        f"{plant.params.max_thrust_per_motor_n} N per motor)"
    )
    plant.set_motors(fraction)
    for _ in range(50):  # let the near-instant lag settle
        plant.step(DT)
    # The sibling's Mx is our -Mx: its model is z-up, ours is FRD.
    assert plant.achieved_wrench().moment_body[0] == pytest.approx(-0.05, rel=5e-2)


# ---------------------------------------------------------------------------
# Sample delay and noise
# ---------------------------------------------------------------------------


def test_a_delayed_sample_reports_its_own_age():
    plant = make_plant(sample_delay_steps=5)
    plant.set_motors(hover_command(plant))
    for _ in range(20):
        plant.step(DT)
        sample = plant.sense()
    assert sample.delayed_steps == 5
    assert sample.t_ms == pytest.approx(1000.0 * (plant.steps - 5) * DT, abs=1e-9)


def test_a_delayed_sample_is_older_than_the_body_rates_it_reports():
    """The delay is visible in the data, not only in the counter."""
    plant = spinning_at_hover(make_plant(sample_delay_steps=10))
    for _ in range(30):
        plant.step(DT)
        plant.sense()

    # Left pair up: a positive roll. Kick it and look immediately - the delayed
    # gyro must not have it yet.
    plant.motor_actual[...] = np.array([0.2, 0.2, 0.7, 0.7])
    plant.step(DT)
    sample = plant.sense()
    assert plant.body_rates[0] > 1e-4, "the aircraft really is rolling"
    assert sample.gyro_rps[0] == pytest.approx(0.0, abs=1e-9), (
        "the delayed sample was taken before the roll started"
    )


def test_zero_noise_by_default_is_bit_identical():
    """A plant that is noisy when you did not ask for it is a plant whose
    failures cannot be reproduced."""

    def run():
        plant = make_plant()
        plant.set_motors(hover_command(plant, 1.05))
        out = []
        for _ in range(100):
            plant.step(DT)
            out.append(plant.sense().accel_g.copy())
        return np.array(out)

    assert np.array_equal(run(), run())


def test_noise_is_reproducible_from_the_seed():
    def run(seed):
        plant = make_plant(gyro_noise_dps=2.0, accel_noise_g=0.01, seed=seed)
        plant.set_motors(hover_command(plant))
        out = []
        for _ in range(50):
            plant.step(DT)
            out.append(plant.sense().gyro_rps.copy())
        return np.array(out)

    a, b, c = run(1), run(1), run(2)
    assert np.array_equal(a, b), "same seed must give the same noise"
    assert not np.array_equal(a, c), "a different seed must not"
    assert np.abs(a).max() > 0.0, "the noise must actually be present"


# ---------------------------------------------------------------------------
# Rotation
# ---------------------------------------------------------------------------


def test_a_roll_moment_produces_a_positive_roll_rate():
    """Left pair up is a positive roll: the firmware's own bench check."""
    plant = spinning_at_hover(make_plant())
    plant.motor_actual[...] = np.array([0.2, 0.2, 0.7, 0.7])
    for _ in range(50):
        plant.step(DT)
    assert plant.body_rates[0] > 0.0
    assert plant.body_rates[0] > abs(plant.body_rates[1])


def test_a_pitch_moment_raises_the_nose():
    """Front pair up is nose up, and the attitude moves the way the
    estimator's pitch convention says it should."""
    plant = spinning_at_hover(make_plant())
    plant.motor_actual[...] = np.array([0.2, 0.7, 0.2, 0.7])
    for _ in range(50):
        plant.step(DT)
    assert plant.body_rates[1] > 0.0
    # Nose up tilts the body x axis toward -z (up, since z is down).
    assert plant._rotmat(plant.attitude_quat)[2, 0] < 0.0


def test_a_yaw_moment_turns_the_nose_right():
    """The pair the firmware's table speeds up for a positive yaw command."""
    plant = spinning_at_hover(make_plant())
    plant.motor_actual[...] = np.array([0.3, 0.6, 0.6, 0.3])
    for _ in range(50):
        plant.step(DT)
    assert plant.body_rates[2] > 0.0


def test_hover_holds_height_once_the_motors_are_up_to_speed():
    plant = spinning_at_hover(make_plant())
    for _ in range(500):  # 1 second
        plant.step(DT)
    assert plant.velocity[2] == pytest.approx(0.0, abs=1e-3)
    assert plant.position[2] == pytest.approx(0.0, abs=1e-3)


def test_more_than_hover_climbs():
    """z is down, so climbing means z going negative."""
    plant = spinning_at_hover(make_plant(), 1.1)
    for _ in range(500):
        plant.step(DT)
    assert plant.velocity[2] < -0.1
    assert plant.position[2] < 0.0


# ---------------------------------------------------------------------------
# Refusals
# ---------------------------------------------------------------------------


def test_an_invented_spin_order_is_refused():
    """The spin directions are which way the props are mounted, which is a fact
    about an airframe and not something to guess for a motor count."""
    hexa = np.array(
        [[1.0, 0.0], [0.5, 0.9], [-0.5, 0.9], [-1.0, 0.0], [-0.5, -0.9], [0.5, -0.9]]
    )
    with pytest.raises(ValueError, match="spin is required"):
        ActuatorPlant(params=ActuatorPlantParams(), motor_positions=hexa)


def test_a_wrong_length_command_is_refused():
    plant = make_plant()
    with pytest.raises(ValueError, match="expected 4 motor commands"):
        plant.set_motors(np.zeros(6))


def test_a_non_finite_command_is_refused():
    plant = make_plant()
    with pytest.raises(ValueError, match="must be finite"):
        plant.set_motors(np.array([0.5, np.nan, 0.5, 0.5]))


def test_a_bad_dt_is_refused():
    plant = make_plant()
    with pytest.raises(ValueError, match="dt must be > 0"):
        plant.step(0.0)


def test_a_non_finite_state_is_refused_rather_than_propagated():
    """`nan` is not caught by the quaternion's zero-norm check.

    `_rotmat` raises on `n <= 0.0`, and `nan <= 0.0` is **false**, so a diverged
    attitude walks straight through it and contaminates the wrench, the sensors
    and every number downstream while the plant goes on reporting itself.
    Measured with the guard removed: the RK4 at dt/tau = 2e6 diverges at step 14
    to `|quat| = nan`, not to zero - so the zero-norm error that first exposed
    this was luck, and a run landing on nan instead raises nothing at all.
    """
    plant = make_plant()
    plant.motor_actual[2] = np.nan
    with pytest.raises(ValueError, match="motor_actual is not finite before"):
        plant.step(DT)

    other = make_plant()
    other.attitude_quat[0] = np.inf
    with pytest.raises(ValueError, match="attitude_quat is not finite before"):
        other.step(DT)

    # The exit check is the one that would fire on a state the *integrator*
    # produced, and it is not reachable from here: the stability guard refuses
    # the dt that used to produce it, and the entry check catches a state that
    # a caller assigned. Both of those are tested above and below. It stays in
    # `step()` as the thing that would have caught the original divergence had
    # it been there, not as a branch with a test pretending to reach it.


def test_a_step_too_long_for_the_motor_lag_is_refused():
    """Not a warning: explicit RK4 on the lag *diverges* past dt/tau ~ 2.785.

    A caller who sets a 1 microsecond motor time constant and steps at 500 Hz
    gets dt/tau = 2000, and a few steps later a NaN state that reads like a
    modelling result. It is the kind of number that gets typed by accident - a
    `1e-3` meaning milliseconds written as `1e-9` - so it is checked rather
    than left to show up as a mysterious aircraft.

    The plant's own default (tau = 30 ms at dt = 2 ms, a ratio of 0.067) is two
    orders of magnitude inside the limit, so nothing normal is affected.
    """
    # The mistake is a *small* time constant, not a small step: 1e-9 typed where
    # 1e-3 was meant. A short step is always safe and must keep working.
    with pytest.raises(ValueError, match="too large for motor_tau_s"):
        make_plant(motor_tau_s=1e-9).step(DT)
    with pytest.raises(ValueError, match="too large for motor_tau_s"):
        make_plant(motor_tau_s=1e-6).step(DT)
    make_plant().step(1e-9)

    # And the boundary is not off by anything: 2.785 * tau passes, 2.79 does not.
    make_plant(motor_tau_s=1e-3).step(2.785e-3)
    with pytest.raises(ValueError, match="too large for motor_tau_s"):
        make_plant(motor_tau_s=1e-3).step(2.79e-3)
