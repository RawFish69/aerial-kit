"""MuJoCo-integrated single-motor flying wing."""

from __future__ import annotations

import importlib.util
import math
import unittest

import numpy as np

from aerial_kit.airframes.single_wing import load_single_wing_spec
from aerial_kit.dynamics.fixed_wing import level_attitude_quat
from aerial_kit.dynamics.single_wing import SingleWingDynamics
from aerial_kit.types import ControlTarget, SimState

requires_mujoco = unittest.skipUnless(importlib.util.find_spec("mujoco") is not None,
                                      "mujoco is not installed")


def _state(speed=14.0, z=80.0):
    return SimState(position=np.array([0.0, 0.0, z]), velocity=np.array([speed, 0.0, 0.0]), t=0.0,
                    attitude_quat=level_attitude_quat(0.0), body_rates=np.zeros(3))


def _cmd(values):
    return ControlTarget(accel_cmd=np.zeros(3), metadata={"actuator_cmd": np.asarray(values, float)})


@requires_mujoco
class TestMujocoSingleWing(unittest.TestCase):
    def _backend(self):
        from sim_py.backends.mujoco_single_wing_backend import MujocoSingleWingBackend

        b = MujocoSingleWingBackend()
        b.reset(_state(), {}, {"simulation": {"singlewing": {}}})
        return b

    def test_model_hierarchy_and_mass(self) -> None:
        import mujoco

        b = self._backend()
        m, d = b.model, b.data
        spec = load_single_wing_spec()
        root = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "body")
        self.assertAlmostEqual(m.body_subtreemass[root], spec.total_mass_kg, places=6)
        np.testing.assert_allclose(d.subtree_com[root], d.xpos[root], atol=1e-6)
        for name in ("left_elevon", "right_elevon", "prop"):
            bid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, name)
            self.assertEqual(m.body_parentid[bid], root, name)
        prop = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, "prop_joint")
        np.testing.assert_allclose(m.jnt_axis[prop], [1, 0, 0])
        # Pusher: prop behind the CG on the centreline.
        self.assertLess(m.body_pos[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "prop")][0], -0.05)
        meshes = [g for g in range(m.ngeom) if m.geom_type[g] == mujoco.mjtGeom.mjGEOM_MESH]
        self.assertTrue(meshes and all(m.geom_contype[g] == 0 for g in meshes))

    def test_roll_right_raises_the_right_trailing_edge(self) -> None:
        import mujoco

        b = self._backend()
        for _ in range(30):
            b.step(_cmd([2.0, -0.15, 0.15]), 0.01)
        s = b.actuator_state()
        self.assertAlmostEqual(s["right_elevon_rad"], 0.15, delta=0.02)
        self.assertAlmostEqual(s["left_elevon_rad"], -0.15, delta=0.02)
        sid = mujoco.mj_name2id(b.model, mujoco.mjtObj.mjOBJ_SITE, "right_elevon_te_marker")
        bid = mujoco.mj_name2id(b.model, mujoco.mjtObj.mjOBJ_BODY, "body")
        local = b.data.xmat[bid].reshape(3, 3).T @ (b.data.site_xpos[sid] - b.data.xpos[bid])
        self.assertLess(local[2], -0.005)        # FRD: up is -z
        self.assertGreater(b.state().body_rates[0], 0.2)

    def test_prop_spins_with_thrust_in_the_spec_direction(self) -> None:
        b = self._backend()
        for _ in range(40):
            b.step(_cmd([5.0, 0.0, 0.0]), 0.01)
        s = b.actuator_state()
        self.assertEqual(np.sign(s["prop_rad_s"]), load_single_wing_spec().prop_spin)
        self.assertGreater(abs(s["prop_rad_s"]), 50.0)
        self.assertGreater(s["thrust_n"], 4.5)

    def test_trim_flight_matches_the_analytic_plant(self) -> None:
        from sim_py.backends.single_wing_backend import SingleWingBackend

        trim = SingleWingDynamics(load_single_wing_spec().dynamics_params()).compute_trim(14.0)
        mj = self._backend()
        an = SingleWingBackend()
        an.reset(_state(), {}, {"simulation": {"singlewing": {}}})
        for _ in range(300):
            mj.step(_cmd(trim), 0.01)
            an.step(_cmd(trim), 0.01)
        a, b = mj.state(), an.state()
        self.assertTrue(np.all(np.isfinite(mj.data.qpos)))
        self.assertLess(abs(a.position[2] - b.position[2]), 1.0)
        self.assertLess(np.linalg.norm(a.position[:2] - b.position[:2]), 2.0)
        self.assertLess(abs(a.airspeed_mps - b.airspeed_mps), 0.5)

    def test_controller_defaults_carry_the_spec_trim(self) -> None:
        d = self._backend().controller_defaults("l1_tecs")
        self.assertGreater(d["trim_throttle_n"], 0.3)
        self.assertLess(d["trim_throttle_n"], d["max_thrust_n"])


if __name__ == "__main__":
    unittest.main()
