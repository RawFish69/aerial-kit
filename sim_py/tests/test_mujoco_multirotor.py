"""Multirotor airframe specs and the MuJoCo multirotor plant."""

from __future__ import annotations

import copy
import importlib.util
import math
import unittest

import numpy as np

from aerial_kit.airframes.multirotor_specs import (
    MultirotorSpec,
    available_airframes,
    load_multirotor_spec,
)
from aerial_kit.dynamics.multirotor_actuator import QUAD_X_SPIN
from aerial_kit.types import ControlTarget, SimState, Waypoint, Wrench

HAS_MUJOCO = importlib.util.find_spec("mujoco") is not None
requires_mujoco = unittest.skipUnless(HAS_MUJOCO, "mujoco is not installed")
AIRFRAMES = ("fpv_quad_5in", "micro_lr_quad", "research_hex", "research_octo")


def _state(z=5.0):
    return SimState(position=np.array([0.0, 0.0, z]), velocity=np.zeros(3), t=0.0,
                    attitude_quat=np.array([1.0, 0.0, 0.0, 0.0]), body_rates=np.zeros(3))


class TestSpecs(unittest.TestCase):
    def test_bundled_airframes(self) -> None:
        self.assertTrue(set(AIRFRAMES) <= set(available_airframes()))
        counts = {name: load_multirotor_spec(name).n_rotors for name in AIRFRAMES}
        self.assertEqual(counts, {"fpv_quad_5in": 4, "micro_lr_quad": 4,
                                  "research_hex": 6, "research_octo": 8})

    def test_quads_use_the_firmware_spin_table_and_order(self) -> None:
        # Firmware rows: rear-right, front-right, rear-left, front-left.
        for name in ("fpv_quad_5in", "micro_lr_quad"):
            spec = load_multirotor_spec(name)
            np.testing.assert_array_equal([r.spin for r in spec.rotors], QUAD_X_SPIN, err_msg=name)
            pos = np.array([r.position_m for r in spec.rotors])
            self.assertTrue(pos[0, 0] < 0 and pos[0, 1] < 0, name)   # rear right
            self.assertTrue(pos[1, 0] > 0 and pos[1, 1] < 0, name)   # front right
            self.assertTrue(pos[2, 0] < 0 and pos[2, 1] > 0, name)   # rear left
            self.assertTrue(pos[3, 0] > 0 and pos[3, 1] > 0, name)   # front left

    def test_props_fit_without_overlap(self) -> None:
        for name in AIRFRAMES:
            spec = load_multirotor_spec(name)
            pos = np.array([r.position_m[:2] for r in spec.rotors])
            gaps = [np.linalg.norm(a - b) for i, a in enumerate(pos) for b in pos[i + 1:]]
            self.assertGreater(min(gaps), 2 * spec.prop_radius_m, name)

    def test_hover_is_feasible_with_margin(self) -> None:
        for name in AIRFRAMES:
            spec = load_multirotor_spec(name)
            hover = spec.hover_thrusts()
            np.testing.assert_allclose(spec.allocator.wrench_of(hover),
                                       [spec.total_mass_kg * 9.81, 0, 0, 0], atol=1e-9)
            self.assertLess(hover.max(), 0.5 * spec.max_thrust_n, name)
            self.assertGreater(spec.thrust_to_weight(), 2.0, name)

    def test_allocator_round_trip_and_priority(self) -> None:
        spec = load_multirotor_spec("research_hex")
        weight = spec.total_mass_kg * 9.81
        demand = np.array([0.4, -0.3, 0.05])
        thrusts, scale = spec.allocator.allocate(weight, demand)
        self.assertEqual(scale, 1.0)
        np.testing.assert_allclose(spec.allocator.wrench_of(thrusts), [weight, *demand], atol=1e-9)
        # An impossible moment keeps the collective and the moment direction.
        thrusts, scale = spec.allocator.allocate(weight, 100.0 * demand)
        self.assertLess(scale, 1.0)
        achieved = spec.allocator.wrench_of(thrusts)
        self.assertAlmostEqual(achieved[0], weight, places=6)
        np.testing.assert_allclose(achieved[1:] / np.linalg.norm(achieved[1:]),
                                   demand / np.linalg.norm(demand), atol=1e-6)
        self.assertTrue(np.all(thrusts >= -1e-9) and np.all(thrusts <= spec.max_thrust_n + 1e-9))

    def test_micro_quad_reads_its_cad(self) -> None:
        spec = load_multirotor_spec("micro_lr_quad")
        # URDF frame + four props + the declared battery.
        self.assertAlmostEqual(spec.total_mass_kg, 0.23217587 + 2 * 0.0125963 + 2 * 0.0125968 + 0.095, places=5)
        front = [r for r in spec.rotors if r.position_m[0] > 0]
        self.assertEqual({r.name for r in front}, {"front_right", "front_left"})
        # The assumed battery balances the CG over the rotor centroid.
        centroid = np.mean([r.position_m for r in spec.rotors], axis=0)
        self.assertLess(np.linalg.norm(centroid[:2]), 0.002)

    def test_thrust_command_validation(self) -> None:
        spec = load_multirotor_spec("fpv_quad_5in")
        out, clipped = spec.clamp_rotor_thrusts([-1.0, 99.0, 1.0, 1.0])
        self.assertTrue(clipped)
        np.testing.assert_allclose(out, [0.0, spec.max_thrust_n, 1.0, 1.0])
        for bad in ([math.nan, 1, 1, 1], [1, 1, 1]):
            with self.subTest(bad=bad), self.assertRaises(ValueError):
                spec.clamp_rotor_thrusts(bad)


@requires_mujoco
class TestMjcf(unittest.TestCase):
    def test_hierarchy_mass_and_inertia(self) -> None:
        import mujoco

        from aerial_kit.airframes.multirotor_specs.mjcf import multirotor_mjcf

        for name in AIRFRAMES:
            with self.subTest(airframe=name):
                spec = load_multirotor_spec(name)
                model = mujoco.MjModel.from_xml_string(multirotor_mjcf(spec))
                data = mujoco.MjData(model)
                mujoco.mj_forward(model, data)
                root = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "body")
                self.assertAlmostEqual(model.body_subtreemass[root], spec.total_mass_kg, places=9)
                np.testing.assert_allclose(data.subtree_com[root], data.xpos[root], atol=1e-7)
                composite = np.zeros((3, 3))
                for b in range(1, model.nbody):
                    rot = data.ximat[b].reshape(3, 3)
                    r = data.xipos[b] - data.subtree_com[root]
                    composite += rot @ np.diag(model.body_inertia[b]) @ rot.T
                    composite += model.body_mass[b] * (r @ r * np.eye(3) - np.outer(r, r))
                np.testing.assert_allclose(composite, spec.composite_inertia(), atol=1e-8)
                for rotor in spec.rotors:
                    bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, f"{rotor.name}_rotor")
                    self.assertEqual(model.body_parentid[bid], root)
                    jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, f"{rotor.name}_rotor_joint")
                    np.testing.assert_allclose(model.jnt_axis[jid], [0, 0, 1])
                    act = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, f"{rotor.name}_thrust")
                    np.testing.assert_allclose(model.actuator_gear[act],
                                               [0, 0, 1, 0, 0, rotor.body_yaw_torque_per_n], atol=1e-9)
                    # lagged_thrust: MuJoCo's filter is the motor lag. rpm: the
                    # backend integrates motor speed and sets the thrust.
                    expected = (mujoco.mjtDyn.mjDYN_NONE if spec.propulsion_model == "rpm"
                                else mujoco.mjtDyn.mjDYN_FILTER)
                    self.assertEqual(model.actuator_dyntype[act], expected)
                    # Thrust acts on the frame, never on the free rotor shaft.
                    site = model.actuator_trnid[act, 0]
                    self.assertEqual(model.site_bodyid[site], root)
                meshes = [g for g in range(model.ngeom) if model.geom_type[g] == mujoco.mjtGeom.mjGEOM_MESH]
                self.assertTrue(meshes)
                self.assertTrue(all(model.geom_contype[g] == 0 for g in meshes))
                colliders = [g for g in range(model.ngeom)
                             if model.geom_contype[g] and model.geom_bodyid[g] == root]
                self.assertTrue(colliders)


@requires_mujoco
class TestMujocoMultirotorPlant(unittest.TestCase):
    def _backend(self, name="fpv_quad_5in", z=5.0, start="hover", wind=(0.0, 0.0, 0.0), **extra):
        from sim_py.backends.mujoco_multirotor_backend import MujocoMultirotorBackend

        backend = MujocoMultirotorBackend()
        backend.reset(_state(z), {}, {"simulation": {"mujoco_multirotor": {
            "airframe": name, "start_motors": start, "wind_mps": list(wind), **extra}}})
        return backend

    @staticmethod
    def _thrusts(values):
        return ControlTarget(accel_cmd=np.zeros(3), metadata={"rotor_thrust_n": np.asarray(values, float)})

    def test_open_loop_hover_holds(self) -> None:
        for name in AIRFRAMES:
            with self.subTest(airframe=name):
                backend = self._backend(name)
                hover = backend.spec.hover_thrusts()
                for _ in range(200):
                    backend.step(self._thrusts(hover), 0.01)
                state = backend.state()
                self.assertLess(abs(state.position[2] - 5.0), 0.02)
                self.assertLess(np.linalg.norm(state.position[:2]), 0.05)

    def test_motor_lag_matches_the_spec(self) -> None:
        # The lagged-thrust model's first-order lag is on thrust itself.
        backend = self._backend(propulsion_model="lagged_thrust", ground_effect=False)
        spec = backend.spec
        hover = spec.hover_thrusts()
        target = hover + 4.0
        backend.step(self._thrusts(target), spec.rotors[0].motor_tau_s)
        actual = backend.rotor_state()["actual_n"]
        np.testing.assert_allclose((actual - hover) / 4.0, 1 - math.exp(-1), atol=0.02)

    def test_props_spin_in_their_spec_directions(self) -> None:
        for name in AIRFRAMES:
            with self.subTest(airframe=name):
                backend = self._backend(name)
                hover = backend.spec.hover_thrusts()
                for _ in range(30):
                    backend.step(self._thrusts(hover), 0.01)
                speeds = backend.rotor_state()["shaft_rad_s"]
                spins = np.array([r.spin for r in backend.spec.rotors])
                np.testing.assert_array_equal(np.sign(speeds), spins)

    def test_moment_signs(self) -> None:
        for name in AIRFRAMES:
            spec = load_multirotor_spec(name)
            weight = spec.total_mass_kg * 9.81
            for axis, label in ((0, "roll"), (1, "pitch"), (2, "yaw")):
                with self.subTest(airframe=name, axis=label):
                    backend = self._backend(name)
                    moment = np.zeros(3)
                    moment[axis] = 0.05 * spec.total_mass_kg * spec.arm_radius_m()
                    for _ in range(20):
                        backend.step(ControlTarget(accel_cmd=np.zeros(3), wrench=Wrench(
                            force_body=np.array([0.0, 0.0, weight]), moment_body=moment)), 0.01)
                    rates = backend.state().body_rates
                    self.assertGreater(rates[axis], 0.0)
                    # Dominant axis. The micro quad's CAD has products of
                    # inertia, so some coupling into the other axes is real.
                    self.assertGreater(abs(rates[axis]), 2 * np.abs(np.delete(rates, axis)).max())

    def test_ccw_rotors_react_clockwise(self) -> None:
        backend = self._backend("fpv_quad_5in")
        spec = backend.spec
        thrusts = spec.hover_thrusts().copy()
        for k, r in enumerate(spec.rotors):
            thrusts[k] += 0.3 if r.spin > 0 else -0.3
        for _ in range(30):
            backend.step(self._thrusts(thrusts), 0.01)
        self.assertLess(backend.state().body_rates[2], -0.1)

    def test_idle_on_the_ground_rests_and_full_thrust_takes_off(self) -> None:
        for name in AIRFRAMES:
            with self.subTest(airframe=name):
                backend = self._backend(name, z=0.4, start="idle")
                zero = np.zeros(backend.spec.n_rotors)
                for _ in range(150):
                    backend.step(self._thrusts(zero), 0.01)
                rest = backend.state()
                self.assertLess(np.linalg.norm(rest.velocity), 0.05)
                self.assertTrue(np.all(np.isfinite(backend.data.qpos)))
                climb = 0.7 * np.full(backend.spec.n_rotors, backend.spec.max_thrust_n)
                for _ in range(100):
                    backend.step(self._thrusts(climb), 0.01)
                self.assertGreater(backend.state().position[2], rest.position[2] + 0.5)

    def test_cascade_flies_every_airframe_to_a_waypoint(self) -> None:
        from aerial_kit.controllers.basic import PIDController
        from aerial_kit.controllers.cascade import CascadeController, CascadeGains

        for name in AIRFRAMES:
            with self.subTest(airframe=name):
                backend = self._backend(name, wind=(1.5, -1.0, 0.0))
                ctrl = CascadeController(PIDController(), CascadeGains(**backend.controller_defaults("cascade")))
                wp = Waypoint(position=np.array([5.0, 0.0, 8.0]))
                for _ in range(1000):
                    backend.step(ctrl.compute(backend.state(), wp, {"controller": {}}), 0.01)
                state = backend.state()
                self.assertLess(np.linalg.norm(state.position - wp.position), 0.3)
                self.assertTrue(np.all(np.isfinite(backend.data.qpos)))

    def test_matches_the_firmware_actuator_plant(self) -> None:
        """Same airframe, same motor commands: MuJoCo vs the FRD ActuatorPlant."""
        from aerial_kit.dynamics.actuator_loop import FRD_TO_ZUP_BODY, NED_TO_ENU, zup_state_of
        from aerial_kit.dynamics.rotations import rotmat_to_quat
        from aerial_kit.dynamics.multirotor_actuator import ActuatorPlant, ActuatorPlantParams
        from sim_py.backends.mujoco_multirotor_backend import MujocoMultirotorBackend

        base = load_multirotor_spec("fpv_quad_5in")
        data = copy.deepcopy(base.data)
        data["aero"]["linear_drag_n_per_mps"] = 0.1   # the plant's drag is isotropic
        data["propulsion"]["model"] = "lagged_thrust"
        data["propulsion"]["ground_effect"] = False
        spec = MultirotorSpec(data, source="fpv_isotropic_drag")
        rotors = spec.rotors
        plant = ActuatorPlant(
            params=ActuatorPlantParams(
                mass_kg=spec.total_mass_kg,
                inertia_kgm2=tuple(spec.inertia_diag),
                max_thrust_per_motor_n=spec.max_thrust_n,
                yaw_torque_coeff=rotors[0].yaw_torque_coeff_m,
                motor_tau_s=rotors[0].motor_tau_s,
                kv_drag=0.1, ground_enabled=False),
            motor_positions=np.array([[r.position_m[0], -r.position_m[1]] for r in rotors]),
            spin=np.array([r.spin for r in rotors]),
        )
        # Same pose as MuJoCo's identity (facing ENU east, level), mapped
        # into the plant's NED/FRD exactly as ActuatorBackend.reset does.
        plant.position[...] = NED_TO_ENU @ np.array([0.0, 0.0, 5.0])
        plant.attitude_quat[...] = rotmat_to_quat(NED_TO_ENU @ np.eye(3) @ FRD_TO_ZUP_BODY)
        hover = spec.hover_thrusts()
        plant.set_motors(hover / spec.max_thrust_n)
        plant.motor_actual[...] = plant.motor_command

        backend = MujocoMultirotorBackend()
        # The ActuatorPlant is a lagged-thrust plant with no battery or ground.
        backend.reset(_state(), {}, {"simulation": {"mujoco_multirotor": {
            "airframe": "fpv_quad_5in", "propulsion_model": "lagged_thrust", "ground_effect": False}}})
        # Swap in the isotropic-drag spec (same geometry and mass).
        backend._spec, backend._drag = spec, spec.linear_drag()

        # Short opposing pulses on each axis: large enough to excite roll,
        # pitch and yaw, small enough that both models stay well airborne.
        spins = np.array([r.spin for r in rotors])
        roll = np.array([-0.15, -0.15, 0.15, 0.15])
        pitch = np.array([0.15, -0.15, 0.15, -0.15])
        pulses = {range(10, 20): roll, range(20, 30): -roll,
                  range(50, 60): pitch, range(60, 70): -pitch,
                  range(90, 110): 0.4 * spins, range(110, 130): -0.4 * spins}
        schedule = []
        for k in range(150):
            t = hover.copy()
            for window, delta in pulses.items():
                if k in window:
                    t = t + delta
            schedule.append(t)
        for t in schedule:
            plant.set_motors(t / spec.max_thrust_n)
            for _ in range(5):
                plant.step(0.002)
            backend.step(self._thrusts(t), 0.01)
        ref = zup_state_of(plant)
        got = backend.state()
        self.assertGreater(ref.position[2], 4.0)          # stayed airborne
        np.testing.assert_allclose(got.position, ref.position, atol=0.02)
        np.testing.assert_allclose(got.velocity, ref.velocity, atol=0.03)
        np.testing.assert_allclose(got.body_rates, ref.body_rates, atol=0.03)
        self.assertGreater(abs(float(np.dot(got.attitude_quat, ref.attitude_quat))), 0.999)


if __name__ == "__main__":
    unittest.main()


@requires_mujoco
class TestRunnerIntegration(unittest.TestCase):
    def test_example_mission_flies_on_mujoco_rotors(self) -> None:
        from pathlib import Path

        from aerial_kit.sim import run_simulation

        import yaml

        path = Path(__file__).resolve().parents[2] / "examples" / "quadrotor" / "mujoco.yaml"
        raw = yaml.safe_load(path.read_text(encoding="utf-8"))
        raw["simulation"]["mujoco_multirotor"]["airframe"] = "research_hex"
        result = run_simulation(raw)
        self.assertEqual(result.backend_name, "mujoco_multirotor")
        self.assertTrue(np.all(np.isfinite(result.trajectory)))
        self.assertEqual(result.collisions_detected, 0)
        self.assertLess(result.distance_to_goal, 3.0)

    def test_controller_defaults_come_from_the_spec_and_config_wins(self) -> None:
        from sim_py.backends.mujoco_multirotor_backend import MujocoMultirotorBackend

        backend = MujocoMultirotorBackend()
        backend.reset(_state(), {}, {"simulation": {"mujoco_multirotor": {"airframe": "research_octo"}}})
        spec = backend.spec
        d = backend.controller_defaults("cascade")
        self.assertEqual(d["mass_kg"], spec.total_mass_kg)
        np.testing.assert_allclose(d["inertia_kgm2"], spec.inertia_diag)
        self.assertEqual(d["k_rate"], spec.control["k_rate"])
        self.assertNotIn("k_rate", backend.controller_defaults("geometric"))


@requires_mujoco
class TestPilotModes(unittest.TestCase):
    def _fly(self, name, mode, sticks, seconds):
        from aerial_kit.controllers.cascade import CascadeGains
        from aerial_kit.controllers.multirotor_manual import ManualLimits, MultirotorPilot
        from sim_py.backends.mujoco_multirotor_backend import MujocoMultirotorBackend

        backend = MujocoMultirotorBackend()
        backend.reset(_state(10.0), {}, {"simulation": {"mujoco_multirotor": {"airframe": name}}})
        pilot = MultirotorPilot(CascadeGains(**backend.controller_defaults("cascade")),
                                ManualLimits.from_spec(backend.spec))
        for _ in range(int(seconds / 0.01)):
            wrench = pilot.stick_wrench(backend.state(), mode, sticks, 0.01)
            backend.step(ControlTarget(accel_cmd=np.zeros(3), wrench=wrench), 0.01)
        return backend

    def test_angle_mode_banks_to_the_limit_and_holds_altitude(self) -> None:
        from aerial_kit.controllers.multirotor_manual import StickInput
        from aerial_kit.dynamics.rotations import quat_to_rotmat

        for name in ("fpv_quad_5in", "research_hex"):
            with self.subTest(airframe=name):
                b = self._fly(name, "angle", StickInput(roll=1.0), 1.5)
                r = quat_to_rotmat(b.state().attitude_quat)
                roll = math.degrees(math.atan2(r[2, 1], r[2, 2]))
                self.assertAlmostEqual(roll, b.spec.control["manual"]["max_angle_deg"], delta=1.5)
                self.assertLess(b.state().velocity[1], -1.0)        # flies right (-y)
                self.assertLess(abs(b.state().position[2] - 10.0), 1.0)

    def test_angle_mode_throttle_is_a_bounded_climb_rate(self) -> None:
        from aerial_kit.controllers.multirotor_manual import StickInput

        b = self._fly("fpv_quad_5in", "angle", StickInput(throttle=1.0), 3.0)
        target = b.spec.control["max_climb_mps"]
        # A P loop against vertical drag settles slightly short; what matters is
        # that full throttle is a bounded climb rate, not an unbounded climb.
        self.assertGreater(b.state().velocity[2], 0.9 * target)
        self.assertLessEqual(b.state().velocity[2], target + 0.2)

    def test_velocity_mode_tracks_stick_speed(self) -> None:
        from aerial_kit.controllers.multirotor_manual import StickInput

        b = self._fly("research_octo", "velocity", StickInput(pitch=1.0), 6.0)
        self.assertAlmostEqual(b.state().velocity[0], b.spec.control["max_speed_mps"], delta=0.5)
        self.assertLess(abs(b.state().position[2] - 10.0), 0.5)

    def test_acro_mode_commands_body_rates(self) -> None:
        from aerial_kit.controllers.multirotor_manual import StickInput

        b = self._fly("fpv_quad_5in", "acro", StickInput(yaw=1.0), 1.0)
        expected = -math.radians(b.spec.control["manual"]["yaw_rate_deg_s"])
        self.assertAlmostEqual(b.state().body_rates[2], expected, delta=0.3)


@requires_mujoco
class TestPropulsionModel(unittest.TestCase):
    def _backend(self, name="fpv_quad_5in", z=5.0, **extra):
        from sim_py.backends.mujoco_multirotor_backend import MujocoMultirotorBackend

        b = MujocoMultirotorBackend()
        b.reset(_state(z), {}, {"simulation": {"mujoco_multirotor": {"airframe": name, **extra}}})
        return b

    @staticmethod
    def _cmd(t):
        return ControlTarget(accel_cmd=np.zeros(3), metadata={"rotor_thrust_n": np.asarray(t, float)})

    def test_ground_effect_factor(self) -> None:
        from aerial_kit.airframes.multirotor_specs.spec import ground_effect_factor

        self.assertAlmostEqual(ground_effect_factor(100.0, 0.1), 1.0, places=5)
        self.assertAlmostEqual(ground_effect_factor(0.05, 0.1), 1.0 / 0.75)   # clamped at R/2
        self.assertGreater(ground_effect_factor(0.1, 0.1), ground_effect_factor(0.3, 0.1))

    def test_rpm_spin_up_and_down_are_asymmetric_in_thrust(self) -> None:
        b = self._backend(battery_soc=1.0)
        spec = b.spec
        hover = spec.hover_thrusts()
        high = np.full(spec.n_rotors, 0.8 * spec.max_thrust_n)
        tau = spec.rotors[0].motor_tau_s
        b.step(self._cmd(high), tau)
        up = (b.rotor_state()["actual_n"] - hover) / (high - hover)
        b2 = self._backend(battery_soc=1.0)
        for _ in range(int(0.3 / 0.01)):
            b2.step(self._cmd(high), 0.01)
        start = b2.rotor_state()["actual_n"].copy()
        b2.step(self._cmd(hover), tau)
        down = (start - b2.rotor_state()["actual_n"]) / (start - hover)
        # Speed lags first-order, thrust ~ speed^2: rising thrust is slower to
        # arrive than a first-order thrust lag would be, falling is faster.
        self.assertLess(float(up.mean()), 1 - math.exp(-1) - 0.03)
        self.assertGreater(float(down.mean()), 1 - math.exp(-1) + 0.03)

    def test_voltage_sag_lowers_the_thrust_ceiling(self) -> None:
        full = self._backend(battery_soc=1.0)
        empty = self._backend(battery_soc=0.1)
        top = np.full(full.spec.n_rotors, full.spec.max_thrust_n)
        for _ in range(30):
            full.step(self._cmd(top), 0.01)
            empty.step(self._cmd(top), 0.01)
        f, e = full.rotor_state(), empty.rotor_state()
        self.assertLess(f["battery_v"], full._battery.full_voltage - 0.5)      # IR sag under load
        self.assertLess(float(e["actual_n"].mean()), 0.85 * float(f["actual_n"].mean()))
        self.assertLess(e["battery_soc"], 0.1)

    def test_compensation_holds_commanded_thrust_and_off_shows_droop(self) -> None:
        on = self._backend(battery_soc=0.5)
        off = self._backend(battery_soc=0.5, voltage_compensation=False)
        hover = on.spec.hover_thrusts()
        for _ in range(100):
            on.step(self._cmd(hover), 0.01)
            off.step(self._cmd(hover), 0.01)
        np.testing.assert_allclose(on.rotor_state()["actual_n"], hover, rtol=0.01)
        self.assertLess(float(off.rotor_state()["actual_n"].mean()), 0.95 * float(hover.mean()))
        self.assertLess(off.state().position[2], on.state().position[2] - 0.2)

    def test_ground_effect_lifts_near_the_ground(self) -> None:
        from aerial_kit.airframes.multirotor_specs.spec import ground_effect_factor

        # Hub 0.36 m up (just above the hex's 0.28 m resting height on its gear).
        high = self._backend("research_hex", z=10.0)
        low = self._backend("research_hex", z=0.30)
        hover = high.spec.hover_thrusts()
        # A hover start spins up to produce exactly the start thrust, ground
        # effect included; after a few motor time constants the same command
        # settles to the in-ground-effect thrust.
        for b in (high, low):
            for _ in range(20):
                b.step(self._cmd(hover), 0.01)
        ratio = low.rotor_state()["actual_n"].mean() / high.rotor_state()["actual_n"].mean()
        rotor_z = low.data.site_xpos[low._site[0]][2]
        expected = ground_effect_factor(rotor_z, high.spec.prop_radius_m)
        self.assertGreater(ratio, 1.015)
        self.assertAlmostEqual(ratio, expected, delta=0.01)

    def test_hover_power_and_endurance_are_plausible(self) -> None:
        # Research-default sanity bounds, not a fit: hover current and an
        # endurance (80 % of capacity) in the range such airframes fly.
        bounds = {"fpv_quad_5in": (3, 20), "micro_lr_quad": (3, 20),
                  "research_hex": (10, 45), "research_octo": (10, 45)}
        for name, (lo, hi) in bounds.items():
            with self.subTest(airframe=name):
                b = self._backend(name)
                hover = b.spec.hover_thrusts()
                for _ in range(50):
                    b.step(self._cmd(hover), 0.01)
                amps = b.rotor_state()["battery_a"]
                minutes = 0.8 * float(b.spec.data["battery"]["capacity_mah"]) / 1000.0 / amps * 60.0
                self.assertTrue(lo <= minutes <= hi, f"{name}: {minutes:.1f} min at {amps:.1f} A")
