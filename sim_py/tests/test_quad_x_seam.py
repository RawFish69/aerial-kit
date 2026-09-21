"""The allocator driving the actuator plant, measured through a real plant.

Assessment task 8 asks to "ensure allocator outputs actually drive the
multirotor". Before this file that sentence had no subject: `allocate` returned
newtons in a z-up frame for a motor order of its own, the plant took fractions
in FRD for the firmware's order, and nothing joined them. Two tests in
`test_multirotor_actuator.py` documented the gap; one of them documented it
wrongly, and the correction is the interesting part.

The three claims here, in the order they matter:

* **The composition is correct as it stands.** A wrench in the sibling's own
  frame, through `allocate`, through the frame rotation and the rotor
  permutation, reaches the plant's motors and comes out as the same physical
  moment - every axis, to 1e-12. Nothing needs "fixing" for it to work.
* **The yaw sign comes out right *because* the spin table is left alone.** Trap
  76 used to call the two yaw columns a disagreement about which diagonal turns
  which way, and the obvious repair was to negate one to match the other.
  `test_negating_the_sibling_spin_column_inverts_yaw` measures what that repair
  does: it inverts the yaw axis, by exactly twice the demand. A test that
  asserts a sign is only worth as much as the failure it would catch, so the
  failure is a test too.
* **The direction the aircraft actually turns is the direction the sibling's
  own convention says.** Not inferred from a matrix: the plant is flown
  open-loop and its body rates are read.

Everything is against `ActuatorPlant`, never against a second copy of the
arithmetic in this file - a seam tested against its own restatement proves only
that the restatement was typed twice.
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
from aerial_kit.dynamics.quad_x_seam import (
    motor_commands,
    plant_rows_of,
    sibling_rotor_positions_frd,
    z_up_wrench_to_frd,
)
from aerial_kit.types import Wrench

DT = 0.002  # 500 Hz, the firmware's control rate
G = 9.81
ARM, MASS, YAW_C, TMAX = 0.2, 1.0, 0.02, 6.0


def airframe() -> MultirotorAirframe:
    return MultirotorAirframe(
        arms=4, layout="x", arm_length_m=ARM, mass_kg=MASS, yaw_torque_coeff=YAW_C
    )


def plant(spin: np.ndarray | None = None) -> ActuatorPlant:
    return ActuatorPlant(
        params=ActuatorPlantParams(
            mass_kg=MASS, max_thrust_per_motor_n=TMAX, yaw_torque_coeff=YAW_C
        ),
        motor_positions=quad_x_positions(ARM),
        spin=QUAD_X_SPIN.copy() if spin is None else spin,
    )


def at_the_command(
    p: ActuatorPlant, wrench: Wrench, af: MultirotorAirframe | None = None
) -> ActuatorPlant:
    """Drive the plant to the commanded motors *now*, skipping the lag.

    The static claims below are about which wrenches the allocator asks for,
    not about how long a motor takes to get there, and one 2 ms step reaches 6%
    of a command at the default time constant. The lag itself is tested in
    `test_multirotor_actuator.py`.

    Goes through `motor_commands`, the real seam, so a test that mutates the
    airframe is still measuring the path a flight would take.
    """
    p.set_motors(motor_commands(af or airframe(), wrench, p))
    p.motor_actual[...] = np.clip(
        p.motor_command, p.params.motor_min, p.params.motor_max
    )
    return p


def hover(moment=(0.0, 0.0, 0.0)) -> Wrench:
    """A demand in the **sibling's** frame: +z force is upward thrust."""
    return Wrench(
        force_body=np.array([0.0, 0.0, MASS * G]),
        moment_body=np.asarray(moment, dtype=float),
    )


CASES = {
    "hover": (0.0, 0.0, 0.0),
    "roll right": (+0.20, 0.0, 0.0),
    "roll left": (-0.20, 0.0, 0.0),
    "nose up": (0.0, +0.20, 0.0),
    "nose down": (0.0, -0.20, 0.0),
    "yaw left": (0.0, 0.0, +0.02),
    "yaw right": (0.0, 0.0, -0.02),
}


# ---------------------------------------------------------------------------
# The two conversions, on their own
# ---------------------------------------------------------------------------


def test_the_frame_map_is_a_proper_rotation_and_is_its_own_inverse():
    """`(x, y, z) -> (x, -y, -z)` is a half turn about x, not a reflection.

    This is the fact the whole seam rests on. A reflection would flip the sign
    of a moment on top of the force's, and the yaw axis would come out
    inverted - so the map has to be checked as a rotation, not just as "the
    numbers look right for hover".
    """
    from aerial_kit.dynamics.quad_x_seam import z_up_wrench_to_frd as f

    rotation = np.diag([1.0, -1.0, -1.0])
    assert np.linalg.det(rotation) == pytest.approx(+1.0), "must be proper"

    once = f(hover((0.1, 0.2, 0.3)))
    assert np.allclose(once.force_body, [0.0, 0.0, -MASS * G])
    assert np.allclose(once.moment_body, [0.1, -0.2, -0.3])

    twice = f(once)
    assert np.allclose(twice.force_body, hover((0.1, 0.2, 0.3)).force_body)
    assert np.allclose(twice.moment_body, [0.1, 0.2, 0.3])


def test_the_rotor_map_is_derived_by_position_not_tabulated():
    """The measured map, and that it is derived rather than remembered.

    The first version of this seam's probe wrote the map in the wrong direction
    by hand. That produced a front/back mirror which preserved roll and
    silently inverted pitch and yaw - the kind of error that survives a quick
    look. Matching rotor *positions* cannot be written backwards: the direction
    is fixed by which list is being indexed.
    """
    rows = plant_rows_of(airframe(), plant())
    assert rows.tolist() == [2, 3, 1, 0], (
        "sibling rotors are numbered by polygon angle from 45 degrees "
        "(front-left, rear-left, rear-right, front-right); the firmware is "
        "rear-right, front-right, rear-left, front-left"
    )
    assert sorted(rows.tolist()) == [0, 1, 2, 3], "must be a bijection"


def test_the_rotor_map_refuses_a_geometry_it_does_not_match():
    """A plant flying a different airframe is refused, not rounded to nearest."""
    other = ActuatorPlant(
        params=ActuatorPlantParams(),
        motor_positions=quad_x_positions(0.3),  # a 0.3 m quad, not this 0.2 m one
        spin=QUAD_X_SPIN.copy(),
    )
    with pytest.raises(ValueError, match="no multirotor_4_x rotor at"):
        plant_rows_of(airframe(), other)


# ---------------------------------------------------------------------------
# The composition, through the plant
# ---------------------------------------------------------------------------


@pytest.mark.parametrize("name", sorted(CASES))
def test_the_allocator_drives_the_plant_to_the_demand_it_was_given(name):
    """The claim task 8 asks for, axis by axis, against a real plant."""
    demand = hover(CASES[name])
    p = at_the_command(plant(), demand)
    achieved = p.achieved_wrench()
    wanted = z_up_wrench_to_frd(demand)

    assert np.allclose(achieved.force_body, wanted.force_body, atol=1e-9), (
        f"{name}: thrust came back as {achieved.force_body}"
    )
    assert np.allclose(achieved.moment_body, wanted.moment_body, atol=1e-9), (
        f"{name}: moment came back as {achieved.moment_body}, "
        f"wanted {wanted.moment_body}"
    )


def test_negating_the_sibling_spin_column_inverts_yaw():
    """The falsification of trap 76's old advice, measured.

    Trap 76 read the sibling's yaw column and the firmware's as a disagreement
    about which diagonal turns which way, and the repair that follows from that
    reading is to negate one of them. Negating the sibling's does not align
    anything - it inverts the yaw response, and this is the number: a 0.02 N*m
    demand comes back as -0.02, an error of twice the demand.

    Roll and pitch are untouched by it, which is what makes the mistake
    plausible: a bench check that only rolls the aircraft passes.
    """
    af = airframe()
    af._mixer[3, :] *= -1.0
    af._mixer_pinv = np.linalg.pinv(af._mixer)

    p = at_the_command(plant(), hover((0.0, 0.0, -0.02)), af)
    yaw = p.achieved_wrench().moment_body[2]

    assert yaw == pytest.approx(-0.02, abs=1e-9), (
        "the yaw response is inverted by the repair trap 76 recommended; if "
        f"this now reads +0.02 the recommendation changed, got {yaw}"
    )


def test_dropping_the_frame_transform_is_not_a_small_error():
    """Hover read in the wrong frame is not a scale slip, it is a different aircraft.

    `allocate` reads `force_body[2]` as upward thrust; the plant's own z is
    down. Comparing the plant's output to the un-rotated demand leaves an error
    of twice the weight - the aircraft is being asked to hover upside down.
    """
    demand = hover()
    achieved = at_the_command(plant(), demand).achieved_wrench()
    assert achieved.force_body[2] == pytest.approx(-MASS * G, rel=1e-9)
    assert achieved.force_body[2] - demand.force_body[2] == pytest.approx(
        -2.0 * MASS * G, rel=1e-9
    )


def test_a_yaw_demand_moves_the_rotors_the_firmware_table_says():
    """Anchored to `ak_mixer.c`, not to the seam's own arithmetic.

    The firmware's quad-X table is the external authority for what a yaw
    command does to the motors, and its physical claim is checkable on a bench
    with the props off: yaw right speeds up front-right and rear-left. This
    reads the seam's actual output in the firmware's row order and asserts that
    claim, so a seam that was self-consistently wrong could not pass it.
    """
    p = plant()
    # +z in the sibling's frame is yaw left, so yaw *right* is a negative demand.
    commands = motor_commands(airframe(), hover((0.0, 0.0, -0.02)), p)
    # rows are the firmware's: rear right, front right, rear left, front left
    rr, fr, rl, fl = commands
    assert fr > rr, "front right must be above rear right for yaw right"
    assert rl > fl, "rear left must be above front left for yaw right"


# ---------------------------------------------------------------------------
# What the aircraft does, flown
# ---------------------------------------------------------------------------


@pytest.mark.parametrize(
    "name,axis,sign,physical",
    [
        ("roll right", 0, +1, "the right side goes down"),
        ("nose up", 1, -1, "the nose rises"),
        ("yaw left", 2, -1, "the nose goes left"),
    ],
)
def test_the_aircraft_turns_the_way_the_sibling_convention_says(
    name, axis, sign, physical
):
    """Open-loop, through the integrator: the sign the aircraft actually turns.

    The sibling's own convention is that +Mx rolls right, +My pitches nose-up
    and +Mz yaws left. In the plant's FRD frame those are a positive rate about
    x, a negative rate about y and a negative rate about z - because FRD's y
    points right and its z points down, and a right-handed rotation about
    forward takes right toward down. So this is a statement about the physical
    aircraft, not about which matrix is which.
    """
    p = plant()
    p.set_motors(np.full(4, 1.0))  # hover first, so the motors are near trim
    for _ in range(200):
        p.step(DT)

    moment = [0.0, 0.0, 0.0]
    moment[axis] = 0.02 if axis == 2 else 0.20
    for _ in range(200):
        p.set_motors(motor_commands(airframe(), hover(moment), p))
        p.step(DT)

    rate = p.body_rates[axis]
    assert np.sign(rate) == sign, (
        f"a positive {name} demand must give a body rate of sign {sign} about "
        f"FRD axis {axis} ({physical}), got {rate}"
    )


# ---------------------------------------------------------------------------
# The two ways this seam could be quietly wrong
# ---------------------------------------------------------------------------


def test_a_plant_with_other_props_is_refused_rather_than_flown():
    """The spin table is not a free parameter at this boundary.

    The sibling's mixer has the sibling's spin baked into its yaw row and the
    frame rotation accounts for exactly one sign flip. A plant whose props turn
    differently is a different aircraft, and the yaw response through this seam
    would have the wrong sign or magnitude depending on how it differs - so it
    is refused by name. Inventing a mapping here would be a guess about which
    props are on which arm.
    """
    with pytest.raises(ValueError, match="quad-X spin"):
        motor_commands(airframe(), hover(), plant(spin=np.array([1.0, 1.0, 1.0, 1.0])))


def test_an_infeasible_demand_stays_visible_instead_of_being_clipped():
    """A demand the motors cannot meet must not look like one they met.

    A hard roll at hover asks some motors for negative thrust. `allocate` does
    not clip and neither does the seam; the plant clamps at the motor and
    counts how many. If any layer clipped on the way in, `commanded_wrench()`
    and `achieved_wrench()` would agree and the aircraft's real limit would be
    invisible - which is exactly what F8 exists to prevent.
    """
    p = plant()
    p.set_motors(motor_commands(airframe(), hover((3.0, 0.0, 0.0)), p))

    assert p.motor_command.min() < 0.0, (
        "a 3 N*m roll demand on a 0.2 m quad is not reachable and must be "
        "allowed to ask for negative thrust rather than be quietly clipped"
    )
    gap = np.abs(
        p.commanded_wrench().moment_body - p.achieved_wrench().moment_body
    ).max()
    assert gap > 1e-6, "the unreachable part must show up as a gap"
