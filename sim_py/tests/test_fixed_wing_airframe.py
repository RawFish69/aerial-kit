from __future__ import annotations

import unittest

import numpy as np

from aerial_kit.airframes.fixed_wing import TwinWingAirframe
from aerial_kit.types import CommandKind, SimState, Wrench
from sim_py.core.registry import create_airframe, register_builtin_components


def _state() -> SimState:
    return SimState(position=np.zeros(3), velocity=np.zeros(3))


class TestTwinWingAirframe(unittest.TestCase):
    def test_capabilities(self) -> None:
        wing = TwinWingAirframe(
            cruise_airspeed_mps=15.0, max_bank_deg=45.0, gravity_mps2=9.81
        )
        caps = wing.capabilities
        self.assertFalse(caps.can_hover)
        self.assertEqual(caps.n_actuators, 4)
        self.assertEqual(caps.command_kind, CommandKind.AIRSPEED_NAV)
        self.assertIsNotNone(caps.min_turn_radius_m)
        expected_radius = 15.0**2 / (9.81 * np.tan(np.radians(45.0)))
        self.assertAlmostEqual(caps.min_turn_radius_m, expected_radius, places=6)

    def test_trim_returns_symmetric_actuators(self) -> None:
        wing = TwinWingAirframe(trim_throttle_n=3.0, trim_elevon_rad=0.05)
        actuators = wing.trim(_state())
        np.testing.assert_allclose(actuators, [3.0, 3.0, 0.05, 0.05])

    def test_allocate_zero_wrench_matches_trim_shape(self) -> None:
        wing = TwinWingAirframe(max_thrust_n=10.0)
        wrench = Wrench(force_body=np.array([4.0, 0.0, 0.0]), moment_body=np.zeros(3))
        actuators = wing.allocate(wrench, _state())
        np.testing.assert_allclose(actuators, [4.0, 4.0, 0.0, 0.0], atol=1e-9)

    def test_pitch_preserved_over_roll_when_elevons_saturate(self) -> None:
        wing = TwinWingAirframe(max_elevon_rad=np.radians(25.0))
        max_rad = np.radians(25.0)
        # Command pitch at the limit and a large roll demand that alone would
        # blow through the elevon limit -- pitch must come through exactly,
        # roll must be clipped to whatever headroom remains.
        wrench = Wrench(
            force_body=np.array([1.0, 0.0, 0.0]),
            moment_body=np.array([np.radians(100.0), max_rad, 0.0]),
        )
        throttle_l, throttle_r, elevon_l, elevon_r = wing.allocate(wrench, _state())
        # Pitch component is the average of the two elevons (roll cancels).
        self.assertAlmostEqual((elevon_l + elevon_r) / 2.0, max_rad, places=9)
        self.assertLessEqual(abs(elevon_l), max_rad + 1e-9)
        self.assertLessEqual(abs(elevon_r), max_rad + 1e-9)
        # No roll headroom left, since pitch alone consumed the whole limit.
        self.assertAlmostEqual(elevon_l, elevon_r, places=9)

    def test_yaw_preserved_last_when_throttle_saturates(self) -> None:
        wing = TwinWingAirframe(max_thrust_n=10.0, motor_separation_m=0.5)
        wrench = Wrench(
            force_body=np.array([10.0, 0.0, 0.0]),  # T_c already at the ceiling
            moment_body=np.array([0.0, 0.0, 5.0]),  # large yaw demand
        )
        throttle_l, throttle_r, _, _ = wing.allocate(wrench, _state())
        self.assertLessEqual(throttle_l, 10.0 + 1e-9)
        self.assertLessEqual(throttle_r, 10.0 + 1e-9)
        self.assertGreaterEqual(throttle_r, 0.0 - 1e-9)
        # T_c was already saturated, so there's no headroom for yaw at all.
        self.assertAlmostEqual(throttle_l, throttle_r, places=9)

    def test_rejects_invalid_params(self) -> None:
        with self.assertRaises(ValueError):
            TwinWingAirframe(mass_kg=0.0)
        with self.assertRaises(ValueError):
            TwinWingAirframe(motor_separation_m=0.0)
        with self.assertRaises(ValueError):
            TwinWingAirframe(max_thrust_n=0.0)
        with self.assertRaises(ValueError):
            TwinWingAirframe(max_bank_deg=0.0)
        with self.assertRaises(ValueError):
            TwinWingAirframe(max_bank_deg=90.0)


class TestTwinWingRegistry(unittest.TestCase):
    def setUp(self) -> None:
        register_builtin_components()

    def test_twin_wing_resolves_from_registry(self) -> None:
        wing = create_airframe("twin_wing")
        self.assertEqual(wing.name, "twin_wing")
        self.assertEqual(wing.capabilities.command_kind, CommandKind.AIRSPEED_NAV)


if __name__ == "__main__":
    unittest.main()
