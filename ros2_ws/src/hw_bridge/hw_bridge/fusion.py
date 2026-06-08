"""Altitude fusion: barometer-primary complementary filter with slow GPS-alt correction."""

from typing import Optional


class AltitudeFilter:
    """Complementary filter.

    z = alpha*(z_prev + vz*dt) + (1-alpha)*baro_alt, with an optional slow GPS-alt nudge.
    Baro is zeroed externally by the caller (subtract home baro before passing in) if a
    relative altitude is desired.
    """

    def __init__(self, alpha: float = 0.98, gps_alt_gain: float = 0.01) -> None:
        self.alpha = float(alpha)
        self.gps_alt_gain = float(gps_alt_gain)
        self._z: Optional[float] = None

    def reset(self) -> None:
        self._z = None

    def update(self, baro_alt: float, vz: float, dt: float, gps_alt: Optional[float] = None) -> float:
        if self._z is None:
            self._z = float(baro_alt)
            return self._z
        predicted = self._z + float(vz) * float(dt)
        self._z = self.alpha * predicted + (1.0 - self.alpha) * float(baro_alt)
        if gps_alt is not None:
            self._z += self.gps_alt_gain * (float(gps_alt) - self._z)
        return self._z

    @property
    def altitude(self) -> Optional[float]:
        return self._z
