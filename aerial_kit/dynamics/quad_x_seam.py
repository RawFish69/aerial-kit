"""Driving the firmware's quad-X actuator plant from the sibling allocator.

The two halves of this repository do not share a convention, and until this
module existed there was no path at all from `MultirotorAirframe.allocate` to
`ActuatorPlant` - which made B8's "ensure allocator outputs actually drive the
multirotor" a claim about two things that had never met. This is the meeting
point, and it is deliberately small: a frame rotation, a rotor permutation
derived from geometry, and a division by the motor's full-scale thrust.

What the seam has to bridge
---------------------------
1. **The frame.** `Wrench` is z-up (``force_body[2]`` is *upward* thrust,
   ``Mx = +sum(y*T)``); the plant is FRD, where a rotor pushes along
   **negative** z and the same physical hover is a negative z force. The map is
   ``(x, y, z) -> (x, -y, -z)`` - a rotation by pi about x, so it is proper and
   the moment pseudovector transforms exactly like the force.

2. **The motor order.** The sibling numbers its rotors by polygon angle
   starting at 45 degrees for a quad-X; the firmware numbers them
   rear-right, front-right, rear-left, front-left. Neither order is wrong and
   neither is derivable from the other by inspection, so :func:`plant_rows_of`
   **matches rotor positions** rather than hard-coding a permutation. A
   hand-written table is a sign error waiting to happen: the first version of
   this seam's own probe wrote the map in the wrong direction and produced a
   front/back mirror that preserved roll and silently inverted pitch and yaw.

3. **The units.** ``allocate`` returns **newtons** per motor; the plant takes a
   fraction of ``max_thrust_per_motor_n``. On a 1 kg quadrotor at 6 N per motor
   the hover command is 9.81/4/6 = 0.409, not 2.45.

What the seam must *not* do - the correction this module exists to record
------------------------------------------------------------------------
Trap 76 used to say the two trees disagree about the yaw column: the firmware's
``[-1, +1, +1, -1]`` against the sibling's ``[+1, -1, +1, -1]``, read as a
disagreement about which diagonal turns which way. **That reading was wrong,
and acting on it inverts the yaw axis.** The two lists were written in
different motor orders, which manufactures an asymmetry that is not there.
Aligned to the same rotors, the two columns are elementwise negatives of each
other: the *pairing* is identical in both (rear-right with front-left, and
front-right with rear-left), and the only difference is a global sign - which
is exactly the sign a pseudovector picks up from the z-negation in point 1
above.

So the two sign flips cancel, and the composition is correct with the sibling's
spin table left alone. Measured, on the sibling allocator feeding this plant:

    demand             frame transform only     after "fixing" the spin col
    roll  +/-0.20 N*m  err 0.0                  err 0.0
    pitch +/-0.20 N*m  err 0.0                  err 0.0
    yaw   +/-0.02 N*m  err 0.0                  err -/+0.04  (2x the demand)

Every axis agrees to 1e-12 in the first column. That measurement is pinned in
`sim_py/tests/test_quad_x_seam.py`, including the falsification that negating
the spin column - the change trap 76 used to recommend - breaks yaw.

The spin table is therefore left **unparameterised on purpose**: it is not a
free choice at this boundary. Whoever mounts the props fixes it, and the
plant's own default already encodes the firmware's. A plant carrying a
different spin is refused below rather than silently trusted, because the
sibling's mixer has the sibling's spin baked into its yaw row and only the
matching table is consistent with it.
"""

from __future__ import annotations

import numpy as np

from ..types import Wrench
from .multirotor_actuator import QUAD_X_SPIN, ActuatorPlant


def sibling_rotor_positions_frd(
    arm_length_m: float, arms: int = 4, layout: str = "x"
) -> np.ndarray:
    """The sibling's rotor positions, re-expressed in the plant's FRD frame.

    Derived from the same polygon `MultirotorAirframe._build_mixer` walks, so
    the two cannot drift apart silently - if that formula changes, the
    permutation this feeds stops matching and :func:`plant_rows_of` raises
    rather than returning a plausible wrong answer.
    """
    if layout not in {"x", "+"}:
        raise ValueError(f"layout must be 'x' or '+', got {layout!r}")
    offset = np.pi / arms if layout == "x" else 0.0
    out = np.zeros((arms, 2), dtype=float)
    for i in range(arms):
        theta = 2.0 * np.pi * i / arms + offset
        # The sibling's body axes are z-up, so y points *left*; FRD has y to
        # the right and z down. Only y flips in the horizontal plane.
        out[i, 0] = arm_length_m * np.cos(theta)
        out[i, 1] = -arm_length_m * np.sin(theta)
    return out


def plant_rows_of(airframe, plant: ActuatorPlant) -> np.ndarray:
    """`rows[i]` is the sibling rotor index the plant's rotor `i` is.

    Used as ``thrust_plant = thrust_sibling[rows]``.

    The mapping is *matched by position*, not tabulated: a rotor is the same
    rotor if it is in the same place. Two rotors closer together than a
    thousandth of the arm length would make the matching ambiguous and this
    raises instead of picking one.
    """
    positions = np.asarray(plant.motor_positions, dtype=float)
    if positions.ndim != 2 or positions.shape[1] != 2:
        raise ValueError(
            f"plant motor_positions must be (n, 2), got {positions.shape}"
        )
    mine = sibling_rotor_positions_frd(
        airframe.arm_length_m, arms=airframe.arms, layout=airframe.layout
    )
    if positions.shape[0] != mine.shape[0]:
        raise ValueError(
            f"the plant has {positions.shape[0]} rotors and the airframe "
            f"{mine.shape[0]}"
        )

    rows = []
    for index, spot in enumerate(positions):
        distance = np.linalg.norm(mine - spot, axis=1)
        nearest = int(np.argmin(distance))
        # An exact match or nothing. `approx` would be the wrong instinct
        # here: a near miss means the two geometries disagree, and rounding
        # that to the nearest rotor is how a mirrored airframe gets flown.
        if distance[nearest] > 1e-9:
            raise ValueError(
                f"no {airframe.name} rotor at the plant's rotor {index} "
                f"{np.round(spot, 6)} (nearest is {distance[nearest]:g} m away)"
            )
        rows.append(nearest)
    rows = np.asarray(rows, dtype=int)
    if len(set(rows.tolist())) != len(rows):
        raise ValueError(f"rotor positions are not distinct: map is {rows}")
    return rows


def z_up_wrench_to_frd(wrench: Wrench) -> Wrench:
    """The sibling's z-up body wrench as the plant's FRD one.

    ``(x, y, z) -> (x, -y, -z)``, applied to the force and to the moment. The
    moment is a pseudovector and this is a proper rotation (a half turn about
    x, determinant +1), so it needs no extra sign of its own - which is the
    fact that makes the yaw cancellation in the module docstring work.
    """
    flip = np.array([1.0, -1.0, -1.0])
    return Wrench(
        force_body=np.asarray(wrench.force_body, dtype=float) * flip,
        moment_body=np.asarray(wrench.moment_body, dtype=float) * flip,
    )


def motor_commands(
    airframe, wrench_z_up: Wrench, plant: ActuatorPlant
) -> np.ndarray:
    """Motor commands for ``plant.set_motors()``, from a sibling-frame demand.

    `wrench_z_up` is in the *sibling's* convention - the one `allocate` and
    every controller in this repository speak - and the result is in the
    plant's order and units, ready to hand straight to `set_motors`.

    The result is **not clipped**, for the reason `set_motors` gives: a demand
    the motors cannot meet has to stay visible as `commanded_wrench()` minus
    `achieved_wrench()`, and clipping here would make an infeasible request
    look like a satisfied one. A large roll demand legitimately produces
    negative per-motor thrusts, and what happens to those is the motor's
    business, not this function's.
    """
    _require_firmware_spin(plant)
    thrust_n = np.asarray(airframe.allocate(wrench_z_up, None), dtype=float)
    ordered = thrust_n[plant_rows_of(airframe, plant)]
    return ordered / float(plant.params.max_thrust_per_motor_n)


def _require_firmware_spin(plant: ActuatorPlant) -> None:
    """Refuse a plant whose props turn the other way.

    The sibling's mixer carries the sibling's spin signs in its yaw row, and
    the frame rotation above accounts for them exactly once. A plant with any
    other spin table is a different aircraft, and driving it through this seam
    would give a yaw response of the wrong sign *or* the wrong magnitude
    depending on how it differs - so it is refused by name rather than flown.
    """
    spin = np.asarray(plant.spin, dtype=float)
    if spin.shape != QUAD_X_SPIN.shape or not np.array_equal(spin, QUAD_X_SPIN):
        raise ValueError(
            f"this seam drives the firmware's quad-X spin {QUAD_X_SPIN.tolist()}; "
            f"the plant has {spin.tolist()}. The sibling allocator's yaw row "
            "assumes the matching table - see this module's docstring."
        )
