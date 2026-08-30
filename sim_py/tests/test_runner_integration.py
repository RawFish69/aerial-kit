from __future__ import annotations

import unittest
from pathlib import Path

import numpy as np

from aerial_kit.types import CommandKind
from aerial_kit.controllers.basic import PIDController
from sim_py.core.config import NormalizedSimConfig
from sim_py.core.registry import register_builtin_components, register_controller
from sim_py.core.runner import run_simulation


class TestRunnerIntegration(unittest.TestCase):
    def _base_cfg(self, controller_name: str, airframe_name: str = "quad") -> NormalizedSimConfig:
        return NormalizedSimConfig(
            sim_config_path=Path("sim_py/sim_config.yaml"),
            terrain_override="plains",
            terrain_config_path=None,
            controller_name=controller_name,
            backend_name="pointmass",
            dt=0.05,
            sim_time=0.3,
            path_cfg={
                "planner_type": "straight",
                "start_relative_x": 0.1,
                "start_relative_y": 0.1,
                "start_relative_z": 0.1,
                "end_relative_x": 0.2,
                "end_relative_y": 0.2,
                "end_relative_z": 0.2,
                "start_xy_offset_range": 0.0,
                "end_xy_offset_range": 0.0,
                "collision_inflation": 0.0,
                "terrain_clearance": 0.0,
            },
            controller_cfg={
                "acc_max": 20.0,
                "pid": {"kp": 0.8, "kd": 1.2},
                "lqr": {"q_pos": 10.0, "q_vel": 2.0, "r_acc": 1.0},
                "mpc": {"q_pos": 8.0, "q_vel": 2.0, "r_acc": 10.0},
            },
            visual_cfg={
                "forest_density_scale": 1.0,
                "tree_height_scale": 1.0,
                "height_ratio": 1.2,
            },
            simulation_cfg={"backend": "pointmass", "seed": 123},
            raw_cfg={},
            seed=123,
            airframe_name=airframe_name,
        )

    def test_default_run_with_pointmass(self) -> None:
        sim_log = run_simulation(self._base_cfg("pid"))
        self.assertGreaterEqual(sim_log.trajectory.shape[0], 2)
        self.assertEqual(sim_log.trajectory.shape[1], 3)

    def test_lqr_run_with_pointmass(self) -> None:
        sim_log = run_simulation(self._base_cfg("lqr"))
        self.assertGreaterEqual(sim_log.trajectory.shape[0], 2)
        self.assertEqual(sim_log.trajectory.shape[1], 3)

    def test_hex_and_octo_fly_the_same_mission_as_quad(self) -> None:
        """Workstream 02 phase 2 verify: --airframe hex/octo completes the quad mission.

        PointMassBackend has no actuator concept, so it ignores which airframe is
        selected -- the trajectories are expected to be identical. What this test
        actually exercises is that create_airframe()/allocate() run without error
        for hex and octo across every step of a real mission.
        """
        quad_log = run_simulation(self._base_cfg("pid", airframe_name="quad"))
        hex_log = run_simulation(self._base_cfg("pid", airframe_name="hex"))
        octo_log = run_simulation(self._base_cfg("pid", airframe_name="octo"))

        np.testing.assert_array_equal(quad_log.trajectory, hex_log.trajectory)
        np.testing.assert_array_equal(quad_log.trajectory, octo_log.trajectory)

    def test_unknown_airframe_raises_helpful_error(self) -> None:
        with self.assertRaises(ValueError) as e:
            run_simulation(self._base_cfg("pid", airframe_name="does-not-exist"))
        self.assertIn("Unknown airframe", str(e.exception))

    def test_pid_controller_rejects_twin_wing_airframe(self) -> None:
        """Real-world exercise of the phase-3 compatibility check: pid emits
        ACCEL, twin_wing declares AIRSPEED_NAV, so this must fail readably --
        no synthetic test double needed, both sides already exist."""
        with self.assertRaises(ValueError) as e:
            run_simulation(self._base_cfg("pid", airframe_name="twin_wing"))
        msg = str(e.exception)
        self.assertIn("ACCEL", msg)
        self.assertIn("AIRSPEED_NAV", msg)
        self.assertIn("twin_wing", msg)

    def test_mismatched_controller_airframe_fails_at_startup(self) -> None:
        """Workstream 02 phase 3 verify: a mismatched pair fails fast, readably."""

        class _WrenchOnlyPIDController(PIDController):
            command_kind = CommandKind.WRENCH

        register_builtin_components()
        register_controller("_test_wrench_only_pid", _WrenchOnlyPIDController)
        try:
            with self.assertRaises(ValueError) as e:
                run_simulation(self._base_cfg("_test_wrench_only_pid", airframe_name="quad"))
            msg = str(e.exception)
            self.assertIn("WRENCH", msg)
            self.assertIn("ACCEL", msg)
            self.assertIn("quad", msg)
        finally:
            from sim_py.core.registry import CONTROLLERS

            CONTROLLERS.pop("_test_wrench_only_pid", None)

    def test_l1_tecs_twin_wing_fixedwing_dispatch_runs_end_to_end(self) -> None:
        """CommandKind.AIRSPEED_NAV dispatch: runner.py must route the l1_tecs
        controller's Wrench through TwinWingAirframe.allocate() into
        FixedWingBackend's actuator_cmd path, not the ACCEL/accel_cmd path.
        Not a flight-quality check (run_simulation() always starts at zero
        velocity, so there is no runway/launch model) -- purely a wiring
        smoke test that the new dispatch branch executes without error."""
        cfg = self._base_cfg("l1_tecs", airframe_name="twin_wing")
        cfg = NormalizedSimConfig(
            **{
                **vars(cfg),
                "backend_name": "fixedwing",
                "dt": 0.02,
                "sim_time": 1.0,
                "controller_cfg": {**cfg.controller_cfg, "l1_tecs": {"cruise_airspeed_mps": 15.0}},
                "simulation_cfg": {**cfg.simulation_cfg, "fixedwing": {}},
            }
        )
        sim_log = run_simulation(cfg)
        self.assertEqual(sim_log.backend_name, "fixedwing")
        self.assertTrue(np.all(np.isfinite(sim_log.trajectory)))

    def test_dubins_planner_reads_turn_radius_from_twin_wing_capabilities(self) -> None:
        """runner.py creates the airframe before planning specifically so a
        curvature-aware planner can read Capabilities.min_turn_radius_m --
        select twin_wing + the dubins planner and confirm it actually runs
        (not a flight-quality check; see the dedicated waypoint-loop test in
        test_fixed_wing_waypoint_loop.py for that)."""
        cfg = self._base_cfg("l1_tecs", airframe_name="twin_wing")
        cfg = NormalizedSimConfig(
            **{
                **vars(cfg),
                "backend_name": "fixedwing",
                "dt": 0.02,
                "sim_time": 1.0,
                "path_cfg": {**cfg.path_cfg, "planner_type": "dubins"},
                "controller_cfg": {**cfg.controller_cfg, "l1_tecs": {"cruise_airspeed_mps": 15.0}},
                "simulation_cfg": {**cfg.simulation_cfg, "fixedwing": {}},
            }
        )
        sim_log = run_simulation(cfg)
        self.assertEqual(sim_log.planner_type, "dubins")
        self.assertTrue(np.all(np.isfinite(sim_log.trajectory)))


if __name__ == "__main__":
    unittest.main()
