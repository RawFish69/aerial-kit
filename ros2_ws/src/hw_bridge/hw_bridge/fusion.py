"""Altitude fusion: barometer-primary filters with slow GPS-alt correction."""

import math
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


class BaroVerticalFilter:
    """Second-order tracking filter on barometric altitude: estimates z *and* vz.

    ``AltitudeFilter`` needs a vertical velocity from somewhere, and the
    estimator used to hand it a constant 0. With ``vz = 0`` that filter is a
    plain low-pass on the baro - at alpha 0.98 and 30 Hz, a 1.7 s lag - and the
    odometry it fed reported a climb rate of exactly zero. The hover hold damps
    on that climb rate, so in SITL the aircraft oscillated through 40 m.

    This is the steady-state filter of a constant-velocity Kalman model,
    written as a critically damped loop with natural frequency ``2*pi*hz``:

        e  = baro - (z + vz*dt)
        z  = z + vz*dt + 2*zeta*w*dt * e
        vz = vz + w**2*dt * e

    It needs no IMU, so it does not depend on what frame or gravity convention
    an IMU source publishes. ``hz`` trades noise for lag: the velocity estimate
    settles in roughly ``1 / hz`` seconds.
    """

    def __init__(self, hz: float = 1.0, zeta: float = 1.0, gps_alt_gain: float = 0.01) -> None:
        if hz <= 0.0:
            raise ValueError("hz must be positive")
        self.omega = 2.0 * math.pi * float(hz)
        self.zeta = float(zeta)
        self.gps_alt_gain = float(gps_alt_gain)
        self._z: Optional[float] = None
        self._vz = 0.0

    def reset(self) -> None:
        self._z = None
        self._vz = 0.0

    def update(self, baro_alt: float, dt: float, gps_alt: Optional[float] = None) -> tuple[float, float]:
        """Fold in one baro sample; returns ``(z, vz)``."""
        if self._z is None:
            self._z = float(baro_alt)
            self._vz = 0.0
            return self._z, self._vz
        dt = max(float(dt), 0.0)
        # Keep the discrete loop stable even if a caller's dt jumps.
        k1 = min(2.0 * self.zeta * self.omega * dt, 1.0)
        k2 = (self.omega ** 2) * dt
        predicted = self._z + self._vz * dt
        err = float(baro_alt) - predicted
        self._z = predicted + k1 * err
        self._vz += k2 * err
        if gps_alt is not None:
            self._z += self.gps_alt_gain * (float(gps_alt) - self._z)
        return self._z, self._vz

    @property
    def altitude(self) -> Optional[float]:
        return self._z

    @property
    def climb_rate(self) -> float:
        return self._vz


class GpsVelocity:
    """Horizontal velocity from successive GPS fixes, differenced per *fix*.

    The estimator used to difference position on every 30 Hz tick, whatever
    the GPS rate. With a 10 Hz receiver that reads 0, 0, then a 3x spike - a
    velocity no feedback loop can use. This differences only when a new fix
    arrives, over the time between fixes, and low-passes the result.
    """

    def __init__(self, smoothing: float = 0.5, max_gap_s: float = 1.0) -> None:
        if not 0.0 <= smoothing < 1.0:
            raise ValueError("smoothing must be in [0, 1)")
        self.smoothing = float(smoothing)
        self.max_gap_s = float(max_gap_s)
        self.reset()

    def reset(self) -> None:
        self._last: Optional[tuple[float, float, float]] = None
        self.vx = 0.0
        self.vy = 0.0

    def update(self, east: float, north: float, t: float) -> tuple[float, float]:
        """Fold in a fix at time ``t`` [s]; a repeat of the last ``t`` is ignored."""
        if self._last is not None:
            e0, n0, t0 = self._last
            dt = float(t) - t0
            if dt <= 0.0:
                return self.vx, self.vy  # same fix again
            if dt <= self.max_gap_s:
                a = self.smoothing
                self.vx = a * self.vx + (1.0 - a) * (float(east) - e0) / dt
                self.vy = a * self.vy + (1.0 - a) * (float(north) - n0) / dt
            else:  # after a dropout, a difference across the gap is meaningless
                self.vx = self.vy = 0.0
        self._last = (float(east), float(north), float(t))
        return self.vx, self.vy


class EkfFusion:
    """The estimator node's EKF path, without rclpy: home, frames, and timing.

    Wraps :class:`aerial_kit.estimation.InsEkf` with what the node knows and the
    filter does not: positions are relative to the home captured at arming
    (GPS through ``geo.lla_to_enu``, GPS and baro altitude relative to their
    values at home), and nothing is estimated before there is a home.
    """

    def __init__(self, config) -> None:
        from aerial_kit.estimation import InsEkf

        self._make = lambda: InsEkf(config)
        self.ekf = self._make()
        self.home = False
        self.home_gps_alt: Optional[float] = None
        self.home_baro: Optional[float] = None

    def capture_home(self, t: float, gps_alt: Optional[float], baro: Optional[float]) -> None:
        self.ekf = self._make()
        # Home is the origin and the baro is zeroed there, so both are known;
        # without GPS altitude their split is not observable later, and a
        # loose prior on either would decide it.
        self.ekf.reset(position=[0.0, 0.0, 0.0], t=t, position_sigma=0.1, baro_bias_sigma=0.1)
        self.home_gps_alt = gps_alt
        self.home_baro = baro
        self.home = True

    def on_imu(self, t: float, accel, attitude_quat) -> None:
        if self.home:
            self.ekf.predict(t, accel, attitude_quat)

    def on_gps(self, t: float, east: float, north: float, altitude: Optional[float]) -> bool:
        if not self.home:
            return False
        self.ekf.predict(t)
        if altitude is None or self.home_gps_alt is None:
            return self.ekf.update_gps([east, north, 0.0], use_altitude=False).accepted
        up = float(altitude) - self.home_gps_alt
        return self.ekf.update_gps([east, north, up]).accepted

    def on_baro(self, t: float, baro: float) -> bool:
        if not self.home or self.home_baro is None:
            return False
        self.ekf.predict(t)
        return self.ekf.update_baro(float(baro) - self.home_baro).accepted

    def state_at(self, t: float):
        """``(position, velocity)`` propagated to ``t``; zeros before home."""
        if not self.home:
            return [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]
        self.ekf.predict(t)
        return self.ekf.position.tolist(), self.ekf.velocity.tolist()
