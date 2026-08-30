"""Abstract airframe interface: capabilities, allocation, trim."""

from __future__ import annotations

from abc import ABC, abstractmethod

import numpy as np

from ..types import Capabilities, SimState, Wrench


class Airframe(ABC):
    """A vehicle profile: what it can do, and how a wrench becomes actuator commands."""

    name: str
    capabilities: Capabilities

    @abstractmethod
    def allocate(self, wrench: Wrench, state: SimState) -> np.ndarray:
        """Body thrust+moment -> per-actuator commands."""

    @abstractmethod
    def trim(self, state: SimState) -> np.ndarray:
        """Actuator vector holding steady flight (hover for multirotor, level
        cruise for a wing)."""
