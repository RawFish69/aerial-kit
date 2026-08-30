from __future__ import annotations

import unittest

import numpy as np

from aerial_kit.airframes.multirotor import MultirotorAirframe
from aerial_kit.types import CommandKind, SimState, Wrench
from sim_py.core.registry import create_airframe, register_builtin_components


def _state() -> SimState:
    return SimState(position=np.zeros(3), velocity=np.zeros(3))


class TestMultirotorAirframe(unittest.TestCase):
    def test_quad_capabilities(self) -> None:
        quad = MultirotorAirframe(arms=4, layout="x")
        self.assertTrue(quad.capabilities.can_hover)
        self.assertIsNone(quad.capabilities.min_airspeed_mps)
        self.assertIsNone(quad.capabilities.min_turn_radius_m)
        self.assertEqual(quad.capabilities.n_actuators, 4)
        self.assertEqual(quad.capabilities.command_kind, CommandKind.ACCEL)

    def test_trim_produces_equal_hover_thrust_per_rotor(self) -> None:
        quad = MultirotorAirframe(arms=4, layout="x", mass_kg=2.0, gravity_mps2=9.81)
        actuators = quad.trim(_state())
        self.assertEqual(actuators.shape, (4,))
        expected_each = 2.0 * 9.81 / 4.0
        np.testing.assert_allclose(actuators, expected_each, atol=1e-9)

    def test_trim_scales_with_arm_count(self) -> None:
        for arms in (4, 6, 8):
            airframe = MultirotorAirframe(arms=arms, layout="x", mass_kg=1.0, gravity_mps2=9.81)
            actuators = airframe.trim(_state())
            self.assertEqual(actuators.shape, (arms,))
            self.assertAlmostEqual(float(np.sum(actuators)), 9.81, places=6)

    def test_allocate_pure_thrust_matches_trim(self) -> None:
        quad = MultirotorAirframe(arms=4, layout="x", mass_kg=1.0, gravity_mps2=9.81)
        wrench = Wrench(force_body=np.array([0.0, 0.0, 9.81]), moment_body=np.zeros(3))
        np.testing.assert_allclose(quad.allocate(wrench, _state()), quad.trim(_state()), atol=1e-9)

    def test_allocate_recovers_commanded_wrench(self) -> None:
        quad = MultirotorAirframe(arms=4, layout="x")
        wrench = Wrench(force_body=np.array([0.0, 0.0, 12.0]), moment_body=np.array([0.3, -0.2, 0.05]))
        actuators = quad.allocate(wrench, _state())
        reconstructed = quad._mixer @ actuators
        np.testing.assert_allclose(
            reconstructed, [12.0, 0.3, -0.2, 0.05], atol=1e-9
        )

    def test_pure_yaw_moment_yields_zero_net_roll_pitch(self) -> None:
        quad = MultirotorAirframe(arms=4, layout="x")
        wrench = Wrench(force_body=np.array([0.0, 0.0, 9.81]), moment_body=np.array([0.0, 0.0, 0.5]))
        actuators = quad.allocate(wrench, _state())
        reconstructed = quad._mixer @ actuators
        self.assertAlmostEqual(reconstructed[1], 0.0, places=9)
        self.assertAlmostEqual(reconstructed[2], 0.0, places=9)
        self.assertAlmostEqual(reconstructed[3], 0.5, places=9)

    def test_rejects_too_few_arms(self) -> None:
        with self.assertRaises(ValueError):
            MultirotorAirframe(arms=2)

    def test_rejects_unknown_layout(self) -> None:
        with self.assertRaises(ValueError):
            MultirotorAirframe(arms=4, layout="y")


class TestAirframeRegistry(unittest.TestCase):
    def setUp(self) -> None:
        register_builtin_components()

    def test_builtin_airframes_resolve(self) -> None:
        quad = create_airframe("quad")
        hex_ = create_airframe("hex")
        octo = create_airframe("octo")
        self.assertEqual(quad.capabilities.n_actuators, 4)
        self.assertEqual(hex_.capabilities.n_actuators, 6)
        self.assertEqual(octo.capabilities.n_actuators, 8)

    def test_unknown_airframe_raises_helpful_error(self) -> None:
        with self.assertRaises(ValueError) as e:
            create_airframe("does-not-exist")
        self.assertIn("Unknown airframe", str(e.exception))


if __name__ == "__main__":
    unittest.main()
