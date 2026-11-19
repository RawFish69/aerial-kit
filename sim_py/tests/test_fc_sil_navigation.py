"""Firmware navigation regressions that need a C compiler, not MuJoCo."""

from __future__ import annotations

import math
import shutil
import unittest

HAVE_CC = (shutil.which("cc") or shutil.which("gcc")) is not None
requires_cc = unittest.skipUnless(HAVE_CC, "needs a C compiler")


@requires_cc
class TestFirmwareNavigation(unittest.TestCase):
    def _core(self):
        from sim_py.fc_sil import AkFlightCore

        fc = AkFlightCore()
        fc.set_velocity_channel(7)
        return fc

    def test_heading_alignment_requires_a_converged_straight_track(self) -> None:
        fc = self._core()
        now_ms = 0

        def step(seconds: float) -> None:
            nonlocal now_ms
            channels = fc.channels_from_sticks(
                roll=0.0, pitch=0.0, yaw=0.0, throttle=0.0,
                arm=False, angle_mode=True, velocity=True,
            )
            count = int(round(seconds * 50.0))
            for _ in range(count):
                now_ms += 20
                fc.step_nav(
                    (0.0, 0.0, 0.0), (0.0, 0.0, 1.0),
                    channels, True, now_ms, now_ms * 1000, now_ms,
                    True, 0.0, 5.0, 100.0,
                )

        try:
            step(2.0)
            self.assertFalse(fc.track_aligned)
            step(10.0)
            self.assertTrue(fc.track_aligned)
            self.assertAlmostEqual(fc.attitude()[2], math.pi / 2.0, delta=math.radians(12.0))
        finally:
            fc.close()

    def test_zero_climb_demand_can_trim_a_non_hover_cruise_value(self) -> None:
        fc = self._core()
        now_ms = 0

        def channels(throttle: float, arm: bool) -> list[int]:
            return fc.channels_from_sticks(
                roll=0.0, pitch=0.0, yaw=0.0, throttle=throttle,
                arm=arm, angle_mode=True, velocity=True,
            )

        def step(seconds: float, throttle: float, arm: bool, climb_mps: float) -> None:
            nonlocal now_ms
            ch = channels(throttle, arm)
            for _ in range(int(round(seconds * 50.0))):
                now_ms += 20
                altitude = 100.0 + climb_mps * now_ms / 1000.0
                fc.step_nav(
                    (0.0, 0.0, 0.0), (0.0, 0.0, 1.0),
                    ch, True, now_ms, now_ms * 1000, now_ms,
                    True, 0.0, 0.0, altitude,
                )

        try:
            fc.set_nav_cruise(0.55)
            step(2.0, 0.0, False, 0.0)
            step(5.0, 0.0, True, 0.0)
            self.assertNotEqual(fc.state, "disarmed")
            # Feed a climb the proportional law cannot trim on its own, with a
            # zero climb demand and the configured cruise above the real hover.
            step(2.0, 0.5, True, 2.6)
            self.assertLess(fc.nav_climb_integral, -0.10)
        finally:
            fc.close()


if __name__ == "__main__":
    unittest.main()
