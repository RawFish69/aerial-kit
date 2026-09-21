"""Quaternion and rotation-matrix conversions, w-first, body-to-world.

These three lived apart until something needed all of them: `quat_to_rotmat` in
the fixed-wing dynamics, `rotmat_to_quat` and a heading read-out in the two
modules that close a loop over the actuator plant. Three copies of the same
subject in three packages is how a sign convention acquires a second version, so
they are here instead - and `dynamics.fixed_wing` still re-exports
`quat_to_rotmat` under its old name, because every existing import site meant
that one and none of them should have to move for this.

**Convention, once.** A quaternion is ``[w, x, y, z]`` and is the rotation that
takes a vector's *body* coordinates to its *world* coordinates. The matrix
``quat_to_rotmat(q)`` is that same rotation as a matrix, so
``quat_to_rotmat(q) @ v_body == v_world``. Nothing here knows what the world
frame is or which way is up, which is why the same functions serve a NED plant
and a z-up controller without either of them being wrong.
"""

from __future__ import annotations

import numpy as np


def quat_to_rotmat(quat: np.ndarray) -> np.ndarray:
    """Body-to-world rotation matrix from a w-first quaternion.

    Assumes a unit quaternion. A rotation matrix built from a non-unit one is
    scaled rather than wrong in an obvious way, so callers that cannot vouch for
    the norm should normalise first - `CascadeController._attitude_of` does.
    """
    w, x, y, z = np.asarray(quat, dtype=float)
    return np.array(
        [
            [1 - 2 * (y**2 + z**2), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x**2 + z**2), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x**2 + y**2)],
        ]
    )


def rotmat_to_quat(rotation: np.ndarray) -> np.ndarray:
    """w-first quaternion from a rotation matrix, by Shepperd's method.

    Branches on the largest of the four quantities so that the divisor is never
    small: the naive ``w = sqrt(1 + trace)/2`` form loses all its precision at a
    180-degree rotation, which is exactly the attitude a tumbling aircraft is
    most likely to be found in. `test_cascade.py` checks this against
    `quat_to_rotmat` on random rotations rather than against a second copy of
    the same formula, so the two cannot agree on a mistake.
    """
    rotation = np.asarray(rotation, dtype=float).reshape(3, 3)
    trace = float(rotation[0, 0] + rotation[1, 1] + rotation[2, 2])
    if trace > 0.0:
        s = np.sqrt(trace + 1.0) * 2.0
        quat = np.array(
            [
                0.25 * s,
                (rotation[2, 1] - rotation[1, 2]) / s,
                (rotation[0, 2] - rotation[2, 0]) / s,
                (rotation[1, 0] - rotation[0, 1]) / s,
            ]
        )
    elif rotation[0, 0] > rotation[1, 1] and rotation[0, 0] > rotation[2, 2]:
        s = np.sqrt(1.0 + rotation[0, 0] - rotation[1, 1] - rotation[2, 2]) * 2.0
        quat = np.array(
            [
                (rotation[2, 1] - rotation[1, 2]) / s,
                0.25 * s,
                (rotation[0, 1] + rotation[1, 0]) / s,
                (rotation[0, 2] + rotation[2, 0]) / s,
            ]
        )
    elif rotation[1, 1] > rotation[2, 2]:
        s = np.sqrt(1.0 + rotation[1, 1] - rotation[0, 0] - rotation[2, 2]) * 2.0
        quat = np.array(
            [
                (rotation[0, 2] - rotation[2, 0]) / s,
                (rotation[0, 1] + rotation[1, 0]) / s,
                0.25 * s,
                (rotation[1, 2] + rotation[2, 1]) / s,
            ]
        )
    else:
        s = np.sqrt(1.0 + rotation[2, 2] - rotation[0, 0] - rotation[1, 1]) * 2.0
        quat = np.array(
            [
                (rotation[1, 0] - rotation[0, 1]) / s,
                (rotation[0, 2] + rotation[2, 0]) / s,
                (rotation[1, 2] + rotation[2, 1]) / s,
                0.25 * s,
            ]
        )
    norm = float(np.linalg.norm(quat))
    if not np.isfinite(norm) or norm <= 0.0:
        raise ValueError(f"{rotation} is not a rotation matrix")
    return quat / norm


def yaw_of(rotation: np.ndarray) -> float:
    """Heading of a body-to-world rotation, in radians.

    Read off the body x axis, which is what a heading *is*: the direction the
    nose points. Taken from the matrix rather than from a quaternion so that a
    quaternion sign flip cannot come out as a 360-degree heading change.

    Which way is "zero" is the caller's frame's business, and the two frames in
    this repository do not agree - a level aircraft heading north reads 0 in NED
    and 90 degrees in ENU. See `aerial_kit/dynamics/actuator_loop.py`.
    """
    x_body = np.asarray(rotation, dtype=float).reshape(3, 3)[:, 0]
    return float(np.arctan2(x_body[1], x_body[0]))
