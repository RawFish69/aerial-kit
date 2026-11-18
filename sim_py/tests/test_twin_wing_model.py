"""Shared twin-wing spec, procedural geometry and the articulated MuJoCo model."""

from __future__ import annotations

import importlib.util
import math
import unittest
from collections import Counter
from pathlib import Path

import numpy as np

from aerial_kit.airframes.fixed_wing import TwinWingAirframe
from aerial_kit.airframes.twin_wing import load_twin_wing_spec
from aerial_kit.airframes.twin_wing.geometry import TwinWingGeometry
from aerial_kit.controllers.fixed_wing import airspeed_nav_wrench
from aerial_kit.dynamics.fixed_wing import (
    FixedWingDynamics,
    FixedWingParams,
    level_attitude_quat,
)
from aerial_kit.types import ControlTarget, SimState

REPO = Path(__file__).resolve().parents[2]
HAS_MUJOCO = importlib.util.find_spec("mujoco") is not None
requires_mujoco = unittest.skipUnless(HAS_MUJOCO, "mujoco is not installed")


class TestSpecIsTheSingleSource(unittest.TestCase):
    def setUp(self) -> None:
        self.spec = load_twin_wing_spec()

    def test_planform_matches_the_aero_reference(self) -> None:
        self.assertAlmostEqual(self.spec.planform_area, self.spec.aero["wing_area_m2"], places=9)
        self.assertAlmostEqual(
            self.spec.aero["wing_area_m2"] / self.spec.span,
            self.spec.aero["mean_chord_m"], places=9)

    def test_geometric_static_margin_is_positive_and_modest(self) -> None:
        margin = self.spec.static_margin()
        self.assertGreater(margin, 0.03)
        self.assertLess(margin, 0.25)

    def test_dataclass_defaults_do_not_drift_from_the_yaml(self) -> None:
        defaults = FixedWingParams()
        for key, value in self.spec.fixed_wing_param_overrides().items():
            np.testing.assert_allclose(getattr(defaults, key), value, err_msg=key)

    def test_airframe_defaults_do_not_drift_from_the_yaml(self) -> None:
        wing = TwinWingAirframe()
        self.assertAlmostEqual(wing.mass_kg, self.spec.total_mass_kg)
        self.assertAlmostEqual(wing.motor_separation_m, self.spec.motor_separation_m)
        self.assertAlmostEqual(wing.max_thrust_n, self.spec.propulsion["max_thrust_n"])
        self.assertAlmostEqual(wing.max_elevon_rad, self.spec.max_elevon_rad)

    def test_bundled_sim_configs_agree_with_the_yaml(self) -> None:
        import yaml

        for path in (REPO / "aerial_kit" / "sim" / "defaults" / "fixed_wing.yaml",
                     REPO / "examples" / "fixed_wing" / "mujoco.yaml"):
            cfg = yaml.safe_load(path.read_text(encoding="utf-8"))
            fw = cfg["simulation"].get("fixedwing", {})
            for key, value in self.spec.fixed_wing_param_overrides().items():
                if key in fw:
                    np.testing.assert_allclose(fw[key], value, err_msg=f"{path.name}:{key}")

    def test_child_bodies_leave_a_physical_root(self) -> None:
        root = self.spec.root_mass()
        children = self.spec.children_mass()
        self.assertAlmostEqual(
            root.mass_kg + sum(c.mass_kg for c in children), self.spec.total_mass_kg)
        self.assertGreater(root.mass_kg, 0.8 * self.spec.total_mass_kg)
        eig = np.linalg.eigvalsh(root.inertia_kg_m2)
        self.assertGreater(eig.min(), 0.0)

    def test_hinge_axes_raise_the_trailing_edge_for_positive_rotation(self) -> None:
        for side in ("left", "right"):
            p0, p1, axis = self.spec.elevon_hinge(side)
            # Hinge lies along the swept line, nearly spanwise.
            self.assertGreater(abs(axis[1]), 0.9)
            self.assertAlmostEqual(float(np.linalg.norm(axis)), 1.0)
            # A point aft of the hinge, rotated by a small +angle about the
            # axis, must move toward -z (up in FRD).
            arm = np.array([-0.05, 0.0, 0.0])
            moved = arm + 0.1 * np.cross(axis, arm)
            self.assertLess(moved[2], 0.0, side)

    def test_props_counter_rotate(self) -> None:
        self.assertEqual(self.spec.motor_spin("left"), -self.spec.motor_spin("right"))
        self.assertLess(self.spec.shaft_speed_for_thrust(3.0, "left"), 0.0)
        self.assertGreater(self.spec.shaft_speed_for_thrust(3.0, "right"), 0.0)
        self.assertEqual(self.spec.shaft_speed_for_thrust(0.0, "right"), 0.0)


class TestProceduralGeometry(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.spec = load_twin_wing_spec()
        cls.parts = TwinWingGeometry(cls.spec).parts()

    def test_every_mesh_is_closed_and_outward(self) -> None:
        for part in self.parts:
            with self.subTest(part=part.name):
                edges = Counter()
                for a, b, c in part.mesh.faces:
                    for u, v in ((a, b), (b, c), (c, a)):
                        edges[(min(u, v), max(u, v))] += 1
                # Coincident loft points may merge some edges, but no edge
                # may be open (count 1).
                self.assertNotIn(1, set(edges.values()))
                self.assertGreater(part.mesh.signed_volume(), 0.0)

    def test_required_parts_exist_on_the_right_bodies(self) -> None:
        by_name = {p.name: p.body for p in self.parts}
        for side in ("left", "right"):
            self.assertEqual(by_name[f"{side}_elevon"], f"{side}_elevon")
            self.assertEqual(by_name[f"{side}_prop_blades"], f"{side}_prop")
            self.assertEqual(by_name[f"{side}_spinner"], f"{side}_prop")
            self.assertEqual(by_name[f"{side}_nacelle"], "body")
            self.assertEqual(by_name[f"{side}_wingtip_fin"], "body")
        self.assertEqual(by_name["center_body"], "body")

    def test_span_and_prop_diameter_match_the_spec(self) -> None:
        verts = np.vstack([p.mesh.vertices for p in self.parts if p.body == "body"])
        self.assertAlmostEqual(verts[:, 1].max() - verts[:, 1].min(), self.spec.span, delta=0.04)
        blades = next(p for p in self.parts if p.name == "right_prop_tips").mesh.vertices
        radius = np.linalg.norm(blades[:, 1:], axis=1).max()
        self.assertAlmostEqual(2 * radius, self.spec.propulsion["prop"]["diameter_m"], delta=0.005)

    def test_fins_stand_up_from_the_wingtips(self) -> None:
        for side, sgn in (("left", -1.0), ("right", 1.0)):
            fin = next(p for p in self.parts if p.name == f"{side}_wingtip_fin").mesh.vertices
            self.assertLess(fin[:, 2].min(), -0.1)       # up is -z in FRD
            self.assertAlmostEqual(float(np.mean(fin[:, 1])), sgn * self.spec.half_span, delta=0.01)


@requires_mujoco
class TestArticulatedMjcf(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        import mujoco

        from sim_py.backends.mujoco_fixedwing_backend import twin_wing_mjcf

        cls.mj = mujoco
        cls.spec = load_twin_wing_spec()
        cls.model = mujoco.MjModel.from_xml_string(twin_wing_mjcf())
        cls.data = mujoco.MjData(cls.model)
        mujoco.mj_forward(cls.model, cls.data)

    def _id(self, kind, name):
        return self.mj.mj_name2id(self.model, kind, name)

    def test_body_tree(self) -> None:
        root = self._id(self.mj.mjtObj.mjOBJ_BODY, "body")
        self.assertGreater(root, 0)
        for name in ("left_elevon", "right_elevon", "left_prop", "right_prop"):
            bid = self._id(self.mj.mjtObj.mjOBJ_BODY, name)
            self.assertGreater(bid, 0, name)
            self.assertEqual(self.model.body_parentid[bid], root, name)

    def test_joints_and_actuators(self) -> None:
        hinge = self.mj.mjtJoint.mjJNT_HINGE
        for side in ("left", "right"):
            elevon = self._id(self.mj.mjtObj.mjOBJ_JOINT, f"{side}_elevon_joint")
            prop = self._id(self.mj.mjtObj.mjOBJ_JOINT, f"{side}_prop_joint")
            self.assertEqual(self.model.jnt_type[elevon], hinge)
            self.assertEqual(self.model.jnt_type[prop], hinge)
            self.assertTrue(self.model.jnt_limited[elevon])
            np.testing.assert_allclose(
                self.model.jnt_range[elevon],
                [-self.spec.elevon_limit_rad, self.spec.elevon_limit_rad], atol=1e-5)
            np.testing.assert_allclose(
                self.model.jnt_axis[elevon], self.spec.elevon_hinge(side)[2], atol=1e-6)
            np.testing.assert_allclose(self.model.jnt_axis[prop], [1.0, 0.0, 0.0])
            self.assertGreater(self.model.dof_damping[self.model.jnt_dofadr[elevon]], 0.0)
            for act in (f"{side}_elevon_servo", f"{side}_prop_motor"):
                self.assertGreaterEqual(self._id(self.mj.mjtObj.mjOBJ_ACTUATOR, act), 0, act)

    def test_visual_geoms_never_collide_and_collision_is_simplified(self) -> None:
        visual = [g for g in range(self.model.ngeom)
                  if self.model.geom_type[g] == self.mj.mjtGeom.mjGEOM_MESH]
        self.assertGreater(len(visual), 20)
        for g in visual:
            self.assertEqual(self.model.geom_contype[g], 0)
            self.assertEqual(self.model.geom_conaffinity[g], 0)
        colliders = [g for g in range(self.model.ngeom)
                     if self.model.geom_contype[g] and self.model.geom_bodyid[g] != 0]
        self.assertTrue(colliders)
        self.assertTrue(all(self.model.geom_type[g] != self.mj.mjtGeom.mjGEOM_MESH
                            for g in colliders))

    def test_required_named_geoms(self) -> None:
        for name in ("center_body", "left_wing_top", "right_wing_bottom",
                     "left_wingtip_fin", "right_wingtip_fin", "left_elevon",
                     "right_elevon", "left_prop_blades", "right_prop_blades",
                     "left_spinner", "right_nacelle"):
            self.assertGreaterEqual(self._id(self.mj.mjtObj.mjOBJ_GEOM, name), 0, name)

    def test_total_mass_and_composite_inertia(self) -> None:
        root = self._id(self.mj.mjtObj.mjOBJ_BODY, "body")
        self.assertAlmostEqual(self.model.body_subtreemass[root], 1.5, places=9)
        np.testing.assert_allclose(self.data.subtree_com[root], 0.0, atol=1e-7)
        composite = np.zeros((3, 3))
        for b in range(1, self.model.nbody):
            rot = self.data.ximat[b].reshape(3, 3)
            r = self.data.xipos[b] - self.data.subtree_com[root]
            composite += rot @ np.diag(self.model.body_inertia[b]) @ rot.T
            composite += self.model.body_mass[b] * (r @ r * np.eye(3) - np.outer(r, r))
        np.testing.assert_allclose(composite, np.diag(self.spec.inertia_diag), atol=1e-7)


@requires_mujoco
class TestMujocoArticulation(unittest.TestCase):
    def _backend(self, wind=(0.0, 0.0, 0.0)):
        from sim_py.backends.mujoco_fixedwing_backend import MujocoFixedWingBackend

        backend = MujocoFixedWingBackend()
        backend.reset(
            SimState(position=np.array([0.0, 0.0, 55.0]), velocity=np.array([15.0, 0.0, 0.0]),
                     t=0.0, attitude_quat=level_attitude_quat(), body_rates=np.zeros(3)),
            {}, {"simulation": {"fixedwing": {"wind_mps": list(wind)},
                                "mujoco": {"timestep": 0.002}}})
        return backend

    @staticmethod
    def _cmd(command):
        return ControlTarget(accel_cmd=np.zeros(3), metadata={"actuator_cmd": np.asarray(command, float)})

    def _te_height(self, backend, side):
        import mujoco

        sid = mujoco.mj_name2id(backend.model, mujoco.mjtObj.mjOBJ_SITE, f"{side}_elevon_te_marker")
        bid = mujoco.mj_name2id(backend.model, mujoco.mjtObj.mjOBJ_BODY, "body")
        # Height of the trailing edge above the wing plane, in the body frame
        # (FRD, so "up" is -z), independent of how the airframe has moved.
        rot = backend.data.xmat[bid].reshape(3, 3)
        local = rot.T @ (backend.data.site_xpos[sid] - backend.data.xpos[bid])
        return float(-local[2])

    def test_pitch_command_moves_both_hinges_trailing_edge_up(self) -> None:
        backend = self._backend()
        neutral = {s: self._te_height(backend, s) for s in ("left", "right")}
        for _ in range(30):
            backend.step(self._cmd([2.0, 2.0, 0.2, 0.2]), 0.01)
        state = backend.actuator_state()
        self.assertAlmostEqual(state["left_elevon_rad"], 0.2, delta=0.02)
        self.assertAlmostEqual(state["right_elevon_rad"], 0.2, delta=0.02)
        for side in ("left", "right"):
            self.assertGreater(self._te_height(backend, side), neutral[side] + 0.008, side)

    def test_roll_command_is_mixed_onto_the_physically_correct_surfaces(self) -> None:
        # Aerial Kit's convention: elevon_R - elevon_L > 0 rolls right.
        backend = self._backend()
        neutral = {s: self._te_height(backend, s) for s in ("left", "right")}
        for _ in range(30):
            backend.step(self._cmd([2.0, 2.0, -0.15, 0.15]), 0.01)
        state = backend.actuator_state()
        # Right wing must lose lift (TE up), left wing gain it (TE down).
        self.assertAlmostEqual(state["right_elevon_rad"], 0.15, delta=0.02)
        self.assertAlmostEqual(state["left_elevon_rad"], -0.15, delta=0.02)
        self.assertGreater(self._te_height(backend, "right"), neutral["right"] + 0.005)
        self.assertLess(self._te_height(backend, "left"), neutral["left"] - 0.005)
        # And the aero model, fed the measured hinges, rolls right.
        _, moment = backend._aero.aero_forces_moments(backend.measured_actuator_command())
        self.assertGreater(moment[0], 0.0)

    def test_hinges_respect_travel_and_servo_rate(self) -> None:
        backend = self._backend()
        spec = backend.spec
        backend.step(self._cmd([2.0, 2.0, 1.0, 1.0]), 0.01)
        rate = spec.elevons["servo"]["max_rate_rad_s"]
        self.assertLessEqual(backend.actuator_state()["left_elevon_rad"], rate * 0.01 + 1e-3)
        for _ in range(80):
            backend.step(self._cmd([2.0, 2.0, -1.0, 1.0]), 0.01)
        state = backend.actuator_state()
        self.assertAlmostEqual(state["right_elevon_rad"], spec.max_elevon_rad, delta=0.01)
        self.assertAlmostEqual(state["left_elevon_rad"], -spec.max_elevon_rad, delta=0.01)
        self.assertLessEqual(abs(state["left_elevon_rad"]), spec.elevon_limit_rad)

    def test_props_spin_up_from_positive_thrust_in_opposite_directions(self) -> None:
        backend = self._backend()
        for _ in range(40):
            backend.step(self._cmd([4.0, 4.0, 0.0, 0.0]), 0.01)
        state = backend.actuator_state()
        spec = backend.spec
        self.assertLess(state["left_prop_rad_s"], 0.0)
        self.assertGreater(state["right_prop_rad_s"], 0.0)
        expected = spec.shaft_speed_for_thrust(4.0, "right")
        self.assertAlmostEqual(state["right_prop_rad_s"], expected, delta=0.05 * expected)
        self.assertAlmostEqual(state["left_prop_rad_s"], -expected, delta=0.05 * expected)
        # Shaft angle really advances (the visual blades turn).
        jid = backend._qpos_adr["right_prop_joint"]
        before = float(backend.data.qpos[jid])
        backend.step(self._cmd([4.0, 4.0, 0.0, 0.0]), 0.01)
        self.assertGreater(float(backend.data.qpos[jid]) - before, 0.0)

    def test_zero_thrust_spins_down(self) -> None:
        backend = self._backend()
        for _ in range(30):
            backend.step(self._cmd([4.0, 4.0, 0.0, 0.0]), 0.01)
        for _ in range(60):
            backend.step(self._cmd([0.0, 0.0, 0.0, 0.0]), 0.01)
        self.assertLess(abs(backend.actuator_state()["right_prop_rad_s"]), 5.0)

    def _closed_loop(self, backend, seconds, z_ref=55.0):
        params = load_twin_wing_spec().fixed_wing_params()
        trim = FixedWingDynamics(params).compute_trim(15.0)
        airframe = TwinWingAirframe(mass_kg=params.mass_kg)
        cfg = {"controller": {"l1_tecs": {"cruise_airspeed_mps": 15.0,
                                          "trim_throttle_n": float(trim[0])}}}
        zs, vs = [], []
        for k in range(int(seconds / 0.01)):
            state = backend.state()
            climb = float(np.clip(0.4 * (z_ref - state.position[2]), -2.0, 2.0))
            bank = math.radians(25.0) if 300 <= k < 900 else 0.0
            wrench = airspeed_nav_wrench(state, 15.0, bank, climb, cfg)
            wrench.moment_body[1] += float(trim[2])
            backend.step(self._cmd(airframe.allocate(wrench, state)), 0.01)
            s = backend.state()
            zs.append(s.position[2])
            vs.append(s.airspeed_mps)
        return np.asarray(zs), np.asarray(vs)

    def test_trim_flight_holds_altitude_and_airspeed(self) -> None:
        backend = self._backend()
        zs, vs = self._closed_loop(backend, 20.0)
        self.assertTrue(np.all(np.isfinite(zs)) and np.all(np.isfinite(vs)))
        self.assertTrue(np.all(np.isfinite(backend.data.qpos)))
        self.assertLess(np.max(np.abs(zs - 55.0)), 3.0)
        self.assertLess(np.max(np.abs(vs[200:] - 15.0)), 2.0)
        self.assertLess(abs(zs[-1] - 55.0), 1.0)

    def test_trim_flight_in_wind_stays_finite_and_bounded(self) -> None:
        backend = self._backend(wind=(3.0, 1.0, 0.0))
        zs, vs = self._closed_loop(backend, 20.0)
        self.assertTrue(np.all(np.isfinite(zs)))
        self.assertLess(np.max(np.abs(zs - 55.0)), 5.0)
        self.assertLess(np.max(np.abs(vs[300:] - 15.0)), 2.5)

    def test_mujoco_still_flies_like_the_analytic_plant(self) -> None:
        from sim_py.backends.fixedwing_backend import FixedWingBackend

        mj = self._backend()
        analytic = FixedWingBackend()
        analytic.reset(
            SimState(position=np.array([0.0, 0.0, 55.0]), velocity=np.array([15.0, 0.0, 0.0]),
                     t=0.0, attitude_quat=level_attitude_quat(), body_rates=np.zeros(3)),
            {}, {"simulation": {"fixedwing": {}}})
        z_mj, _ = self._closed_loop(mj, 12.0)
        z_an, _ = self._closed_loop(analytic, 12.0)
        self.assertLess(np.max(np.abs(z_mj - z_an)), 1.0)
        np.testing.assert_allclose(mj.state().position[:2], analytic.state().position[:2], atol=2.0)


if __name__ == "__main__":
    unittest.main()
