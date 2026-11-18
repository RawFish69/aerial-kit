"""Airframe profiles: capabilities, allocation, and trim per vehicle family."""

from __future__ import annotations

from .base import Airframe
from .fixed_wing import TwinWingAirframe
from .multirotor import MultirotorAirframe
from .single_wing_airframe import SingleWingAirframe

__all__ = ["Airframe", "MultirotorAirframe", "SingleWingAirframe", "TwinWingAirframe"]
