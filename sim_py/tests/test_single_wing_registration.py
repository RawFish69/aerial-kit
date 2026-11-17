"""Single wing through the shared registries and the generic runner."""

from __future__ import annotations

import unittest
from pathlib import Path

import yaml

from aerial_kit.airframes import SingleWingAirframe
from aerial_kit.sim import run_simulation

CONFIG = Path(__file__).resolve().parents[2] / "examples" / "single_wing" / "config.yaml"


class TestSingleWingRegistration(unittest.TestCase):
    def test_exported(self) -> None:
        self.assertEqual(SingleWingAirframe().capabilities.n_actuators, 3)

    def test_generic_runner_flies_the_example_mission(self) -> None:
        cfg = yaml.safe_load(CONFIG.read_text(encoding="utf-8"))
        result = run_simulation(cfg)
        self.assertEqual(result.backend_name, "singlewing")
        self.assertEqual(result.collisions_detected, 0)
        self.assertEqual(result.final_waypoint_index, result.final_waypoint_count)
        self.assertLess(result.distance_to_goal, cfg["path"]["waypoint_acceptance_m"])

    def test_backend_offers_spec_trim_to_the_controller(self) -> None:
        from sim_py.backends.single_wing_backend import SingleWingBackend
        from sim_py.core.types import SimState
        import numpy as np

        backend = SingleWingBackend()
        backend.reset(SimState(position=np.zeros(3), velocity=np.array([14.0, 0, 0]), t=0.0), {},
                      {"simulation": {"singlewing": {}}})
        d = backend.controller_defaults("l1_tecs")
        self.assertGreater(d["trim_throttle_n"], 0.3)
        self.assertLess(d["trim_throttle_n"], d["max_thrust_n"])
        self.assertEqual(backend.controller_defaults("cascade"), {})


if __name__ == "__main__":
    unittest.main()
