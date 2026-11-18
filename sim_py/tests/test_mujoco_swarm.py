"""N multirotors in one MuJoCo scene, and formation flight on top."""

from __future__ import annotations

import importlib.util
import unittest

import numpy as np

from aerial_kit.types import ControlTarget, SimState

requires = unittest.skipUnless(importlib.util.find_spec("mujoco") is not None, "needs mujoco")


def _thrust(sw, scale):
    return ControlTarget(accel_cmd=np.zeros(3), metadata={"rotor_thrust_n": sw.spec.hover_thrusts() * scale})


class TestFormations(unittest.TestCase):
    def test_slot_assignment_is_a_permutation_and_prefers_near_slots(self) -> None:
        from sim_py.swarm import assign_slots

        drones = np.array([[0.0, 0, 0], [10.0, 0, 0], [20.0, 0, 0]])
        slots = np.array([[21.0, 0, 0], [1.0, 0, 0], [11.0, 0, 0]])
        order = assign_slots(drones, slots)
        self.assertEqual(sorted(order.tolist()), [0, 1, 2])
        np.testing.assert_array_equal(order, [1, 2, 0])

    def test_formation_shapes(self) -> None:
        from sim_py.swarm import circle_formation, grid_formation, line_formation

        c = circle_formation(12, 4.0, 3.0)
        np.testing.assert_allclose(np.linalg.norm(c[:, :2], axis=1), 4.0)
        self.assertEqual(grid_formation(16, 1.5, 2.0).shape, (16, 3))
        line = line_formation(5, 1.0, 2.0)
        np.testing.assert_allclose(np.diff(line[:, 1]), 1.0)


@requires
class TestSharedScene(unittest.TestCase):
    def test_n_vehicles_compile_into_one_model_with_their_own_names(self) -> None:
        from sim_py.backends.mujoco_swarm import MujocoSwarm

        for n in (4, 12, 16):
            with self.subTest(n=n):
                sw = MujocoSwarm(n=n)
                self.assertEqual(sw.n, n)
                self.assertEqual(sw.model.nu, n * 2 * sw.spec.n_rotors)       # thrust + spin per rotor
                names = {sw.model.body(i).name for i in range(sw.model.nbody)}
                self.assertTrue({f"d{i}_body" for i in range(n)} <= names)
                p = sw.positions()
                self.assertGreater(sw.min_separation_m(), 1.0)
                self.assertTrue(np.all(p[:, 2] < 0.2))                        # resting on the ground

    def test_vehicles_are_independent(self) -> None:
        from sim_py.backends.mujoco_swarm import MujocoSwarm

        sw = MujocoSwarm(n=4, start_z=3.0)
        cmds = [_thrust(sw, 1.0), _thrust(sw, 1.3), _thrust(sw, 1.0), _thrust(sw, 0.7)]
        for _ in range(100):
            sw.step(cmds, 0.01)
        z = sw.positions()[:, 2]
        self.assertAlmostEqual(z[0], 3.0, delta=0.05)
        self.assertAlmostEqual(z[2], 3.0, delta=0.05)
        self.assertGreater(z[1], 3.5)
        self.assertLess(z[3], 2.5)
        # Each vehicle reports its own state, clock shared.
        self.assertTrue(all(s.t == sw.t for s in sw.states()))

    def test_drones_collide_with_each_other(self) -> None:
        from sim_py.backends.mujoco_swarm import MujocoSwarm

        sw = MujocoSwarm(n=2, positions=[[0.0, 0.0, 0.0], [0.6, 0.0, 0.0]], start_z=2.0)
        v0 = sw.vehicles[0]
        v0.init_state(SimState(position=np.array([0.0, 0.0, 2.0]), velocity=np.array([3.0, 0.0, 0.0]), t=0.0,
                               attitude_quat=np.array([1.0, 0.0, 0.0, 0.0]), body_rates=np.zeros(3)))
        touched = False
        for _ in range(60):
            sw.step([_thrust(sw, 1.0)] * 2, 0.005)
            touched |= sw.drone_contacts() == [(0, 1)]
        self.assertTrue(touched)
        self.assertGreater(sw.states()[1].velocity[0], 0.3)                    # momentum transferred
        self.assertTrue(np.all(np.isfinite(sw.data.qpos)))


@requires
class TestFormationFlight(unittest.TestCase):
    def _mission(self, n):
        from sim_py.swarm import SwarmFormationSim, circle_formation, grid_formation, line_formation

        sim = SwarmFormationSim(n=n)
        sim.takeoff(2.0, 4.0)
        np.testing.assert_allclose(sim.swarm.positions()[:, 2], 2.0, atol=0.15)
        circle = sim.fly_formation(circle_formation(n, radius_m=max(2.0, 0.5 * n), z_m=3.0), 10.0)
        self.assertLess(sim.formation_error_m(circle), 0.2)
        line = sim.fly_formation(line_formation(n, 1.2, 2.5, heading_rad=0.5), 12.0)
        self.assertLess(sim.formation_error_m(line), 0.2)
        # line -> grid swaps neighbours head-on: the case that came within 0.26 m
        # before separation was applied after the speed cap and transits were layered.
        grid = sim.fly_formation(grid_formation(n, 1.5, 2.0), 10.0)
        self.assertLess(sim.formation_error_m(grid), 0.2)
        sim.land(8.0)
        self.assertFalse(any(sim.armed))
        self.assertLess(sim.swarm.positions()[:, 2].max(), 0.2)
        self.assertGreater(sim.min_separation_seen, 0.45)   # prop tips touch at ~0.28 m
        self.assertEqual(sim.contacts_seen, set())
        self.assertTrue(np.all(np.isfinite(sim.swarm.data.qpos)))

    def test_four_drones(self) -> None:
        self._mission(4)

    def test_twelve_drones(self) -> None:
        self._mission(12)


if __name__ == "__main__":
    unittest.main()
