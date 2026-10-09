"""Matplotlib version workarounds shared by the renderers."""

from __future__ import annotations

from typing import Sequence

import numpy as np


def add_line_collection3d(ax, collection, segments: Sequence[np.ndarray]):
    """``ax.add_collection3d(collection)`` with the autoscale done here.

    Matplotlib 3.10 autoscales a ``Line3DCollection`` with
    ``np.array(col._segments3d)``, which numpy 2 refuses when the segments
    have different lengths ("inhomogeneous shape"). 3.11 fixed it, but needs
    Python 3.11, so on Python 3.10 every renderer here that mixes a 2-point
    arm with a 13-point rotor ring would raise. Stacking the points instead
    gives the same limits on every version.
    """
    had_data = ax.has_data()
    ax.add_collection3d(collection, autolim=False)
    points = [np.asarray(s, dtype=float).reshape(-1, 3) for s in segments]
    if points:
        xyz = np.vstack(points)
        ax.auto_scale_xyz(xyz[:, 0], xyz[:, 1], xyz[:, 2], had_data=had_data)
    return collection
