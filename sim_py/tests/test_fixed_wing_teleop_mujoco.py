"""Keyboard -> actuator command -> articulated MuJoCo twin wing.

Exercises the same shared key mapping the ROS teleop node uses
(``sim_py.teleop``) against the MuJoCo backend, so the signs are checked on
the physical hinges and the resulting motion, not just on numbers.
"""

from __future__ import annotations

import importlib.util
import math
import unittest

import numpy as np

from aerial_kit.airframes.twin_wing import load_twin_wing_spec
from aerial_kit.dynamics.fixed_wing import FixedWingDynamics, level_attitude_quat
from aerial_kit.types import ControlTarget, SimState
from sim_py.teleop.commands import (
    FixedWingTeleopTuning,
    fixedwing_command_from_keys,
    neutral_fixedwing_command,
)
from sim_py.teleop.input_state import KeyboardState

requires_mujoco = unittest.skipUnless(
    importlib.util.find_spec("mujoco") is not None, "mujoco is not installed")


def _tuning() -> FixedWingTeleopTuning:
    spec = load_twin_wing_spec()
    trim = FixedWingDynamics(spec.fixed_wing_params()).compute_trim(15.0)
    return FixedWingTeleopTuning(
        throttle_trim_n=float(trim[0]), throttle_range_n=3.0,
        elevon_max_rad=math.radians(10.0), yaw_diff_n=0.6,
        trim_elevon_rad=float(trim[2]))


class TestActuatorClamp(unittest.TestCase):
    def setUp(self) -> None:
        self.spec = load_twin_wing_spec()

    def test_in_range_command_passes_unchanged(self) -> None:
        cmd = np.array([1.0, 2.0, 0.1, -0.1])
        out, clipped = self.spec.clamp_actuator_command(cmd)
        np.testing.assert_array_equal(out, cmd)
        self.assertFalse(clipped)

    def test_limits_come_from_the_spec(self) -> None:
        travel = self.spec.max_elevon_rad
        out, clipped = self.spec.clamp_actuator_command([-3.0, 50.0, 2.0, -2.0])
        self.assertTrue(clipped)
        np.testing.assert_allclose(
            out, [0.0, self.spec.propulsion["max_thrust_n"], travel, -travel])
        out, _ = self.spec.clamp_actuator_command([9.0, 9.0, 0.0, 0.0], max_thrust_n=4.0)
        np.testing.assert_allclose(out[:2], [4.0, 4.0])

    def test_non_finite_and_malformed_commands_are_refused(self) -> None:
        for bad in ([math.nan, 1.0, 0.0, 0.0], [1.0, 1.0, math.inf, 0.0],
                    [1.0, 1.0, 0.0]):
            with self.subTest(bad=bad), self.assertRaises(ValueError):
                self.spec.clamp_actuator_command(bad)


@requires_mujoco
class TestKeyboardFliesTheArticulatedModel(unittest.TestCase):
    def _backend(self):
        from sim_py.backends.mujoco_fixedwing_backend import MujocoFixedWingBackend

        backend = MujocoFixedWingBackend()
        backend.reset(
            SimState(position=np.array([0.0, 0.0, 60.0]), velocity=np.array([15.0, 0.0, 0.0]),
                     t=0.0, attitude_quat=level_attitude_quat(), body_rates=np.zeros(3)),
            {}, {"simulation": {"mujoco": {"timestep": 0.002}}})
        return backend

    def _fly(self, backend, keys, seconds):
        tuning = _tuning()
        keyboard = KeyboardState()
        for key in keys:
            keyboard.press(key)
        cmd = fixedwing_command_from_keys(keyboard, tuning=tuning).actuator_cmd
        for _ in range(int(seconds / 0.01)):
            backend.step(ControlTarget(accel_cmd=np.zeros(3),
                                       metadata={"actuator_cmd": cmd}), 0.01)
        return cmd

    def test_right_bank_key_raises_the_right_trailing_edge_and_rolls_right(self) -> None:
        backend = self._backend()
        cmd = self._fly(backend, ["d"], 0.4)
        self.assertGreater(cmd[3], cmd[2])
        hinges = backend.actuator_state()
        self.assertGreater(hinges["right_elevon_rad"], hinges["left_elevon_rad"] + 0.1)
        # Positive body roll rate p is right-wing-down in FRD.
        self.assertGreater(backend.state().body_rates[0], 0.2)

    def test_pitch_up_key_raises_both_trailing_edges_and_pitches_up(self) -> None:
        backend = self._backend()
        cmd = self._fly(backend, ["s"], 0.3)
        self.assertAlmostEqual(cmd[2], cmd[3])
        hinges = backend.actuator_state()
        self.assertAlmostEqual(hinges["left_elevon_rad"], hinges["right_elevon_rad"], places=4)
        self.assertGreater(hinges["left_elevon_rad"], 0.1)
        self.assertGreater(backend.state().body_rates[1], 0.1)

    def test_throttle_key_spins_both_props_faster(self) -> None:
        cruise = self._backend()
        self._fly(cruise, [], 0.5)
        boosted = self._backend()
        self._fly(boosted, ["space"], 0.5)
        slow, fast = cruise.actuator_state(), boosted.actuator_state()
        self.assertGreater(abs(fast["left_prop_rad_s"]), abs(slow["left_prop_rad_s"]) + 20.0)
        self.assertGreater(abs(fast["right_prop_rad_s"]), abs(slow["right_prop_rad_s"]) + 20.0)
        self.assertLess(fast["left_prop_rad_s"], 0.0)
        self.assertGreater(fast["right_prop_rad_s"], 0.0)

    def test_neutral_trim_holds_flight_for_a_few_seconds(self) -> None:
        backend = self._backend()
        cmd = neutral_fixedwing_command(_tuning()).actuator_cmd
        for _ in range(500):
            backend.step(ControlTarget(accel_cmd=np.zeros(3),
                                       metadata={"actuator_cmd": cmd}), 0.01)
        state = backend.state()
        self.assertTrue(np.all(np.isfinite(backend.data.qpos)))
        self.assertLess(abs(state.position[2] - 60.0), 8.0)
        self.assertGreater(state.airspeed_mps, 10.0)


if __name__ == "__main__":
    unittest.main()
