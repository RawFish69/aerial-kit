"""Multirotor airframe specs (FPV quad, research hex/octo, micro LR quad).

See ``spec.py`` and ``docs/MULTIROTOR_MUJOCO.md``.
"""

from .spec import (
    MultirotorAllocator,
    MultirotorSpec,
    RotorSpec,
    available_airframes,
    load_multirotor_spec,
)

__all__ = [
    "MultirotorAllocator",
    "MultirotorSpec",
    "RotorSpec",
    "available_airframes",
    "load_multirotor_spec",
]
