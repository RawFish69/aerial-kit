"""Airframe profiles: capabilities, allocation, and trim per vehicle family."""

from __future__ import annotations

from .base import Airframe
from .fixed_wing import TwinWingAirframe
from .multirotor import MultirotorAirframe

__all__ = ["Airframe", "MultirotorAirframe", "TwinWingAirframe"]
