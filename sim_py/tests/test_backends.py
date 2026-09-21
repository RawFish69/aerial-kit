from __future__ import annotations

import importlib.util
import unittest

import numpy as np

from sim_py.backends.pointmass_backend import PointMassBackend
from sim_py.backends.mujoco_backend import MujocoBackend
from sim_py.backends.rotorpy_backend import RotorPyBackend
from sim_py.core.types import ControlTarget, SimState


class TestBackends(unittest.TestCase):
    def test_pointmass_backend_interface(self) -> None:
        backend = PointMassBackend()
        backend.reset(
            initial_state=SimState(
                position=np.array([0.0, 0.0, 1.0], dtype=float),
                velocity=np.zeros(3, dtype=float),
                t=0.0,
            ),
            world={},
            cfg={"simulation": {}},
        )
        backend.step(ControlTarget(accel_cmd=np.array([0.1, 0.0, 0.0], dtype=float)), dt=0.1)
        s = backend.state()
        self.assertGreater(s.t, 0.0)
        self.assertEqual(s.position.shape, (3,))
        self.assertEqual(s.velocity.shape, (3,))

    def test_rotorpy_backend_import_guard_or_step(self) -> None:
        backend = RotorPyBackend()
        has_rotorpy = importlib.util.find_spec("rotorpy") is not None

        if not has_rotorpy:
            with self.assertRaises(RuntimeError) as e:
                backend.reset(
                    initial_state=SimState(
                        position=np.array([0.0, 0.0, 1.0], dtype=float),
                        velocity=np.zeros(3, dtype=float),
                        t=0.0,
                    ),
                    world={},
                    cfg={"simulation": {}},
                )
            self.assertIn("requirements-rotorpy.txt", str(e.exception))
            return

        backend.reset(
            initial_state=SimState(
                position=np.array([0.0, 0.0, 1.0], dtype=float),
                velocity=np.zeros(3, dtype=float),
                t=0.0,
            ),
            world={},
            cfg={"simulation": {}},
        )
        backend.step(ControlTarget(accel_cmd=np.zeros(3, dtype=float)), dt=0.01)
        s = backend.state()
        self.assertGreaterEqual(s.t, 0.01)
        self.assertEqual(s.position.shape, (3,))

    def test_mujoco_backend_import_guard_or_step(self) -> None:
        backend = MujocoBackend()
        has_mujoco = importlib.util.find_spec("mujoco") is not None

        if not has_mujoco:
            with self.assertRaises(RuntimeError) as e:
                backend.reset(
                    initial_state=SimState(
                        position=np.array([0.0, 0.0, 1.0], dtype=float),
                        velocity=np.zeros(3, dtype=float),
                        t=0.0,
                    ),
                    world={},
                    cfg={"simulation": {}},
                )
            self.assertIn("requirements-mujoco.txt", str(e.exception))
            return

        backend.reset(
            initial_state=SimState(
                position=np.array([0.0, 0.0, 1.0], dtype=float),
                velocity=np.zeros(3, dtype=float),
                t=0.0,
            ),
            world={},
            cfg={"simulation": {}},
        )
        backend.step(ControlTarget(accel_cmd=np.zeros(3, dtype=float)), dt=0.01)
        s = backend.state()
        self.assertGreaterEqual(s.t, 0.01)
        self.assertEqual(s.position.shape, (3,))


requires_mujoco = unittest.skipUnless(
    importlib.util.find_spec("mujoco") is not None, "mujoco is not installed"
)


class TestMujocoBackendTimestep(unittest.TestCase):
    """``mj_step`` integrates ``model.opt.timestep``, not the ``dt`` it is given.

    Stepping once and advancing the clock by ``dt`` therefore ran the physics and
    the clock at different rates: with MuJoCo's 2 ms default and a 10 ms caller,
    the simulated interval was a fifth of the reported one. These tests measure
    the state after a known acceleration, which is the only way to see it - the
    clock alone always looked right.
    """

    def _backend(self, cfg=None):
        backend = MujocoBackend()
        backend.reset(
            initial_state=SimState(
                position=np.zeros(3, dtype=float),
                velocity=np.zeros(3, dtype=float),
                t=0.0,
            ),
            world={},
            cfg={"simulation": {"mujoco": cfg or {"mass": 1.0}}},
        )
        return backend

    @requires_mujoco
    def test_simulated_interval_equals_the_requested_dt(self) -> None:
        # A constant 1 m/s^2 for T seconds gives v = T exactly, so the velocity
        # is a direct readout of how long the physics actually ran for.
        for dt, steps in ((0.002, 100), (0.01, 100), (0.003, 100), (0.1, 10), (0.02, 50)):
            with self.subTest(dt=dt, steps=steps):
                backend = self._backend()
                target = ControlTarget(accel_cmd=np.array([1.0, 0.0, 0.0]))
                for _ in range(steps):
                    backend.step(target, dt)
                expected = steps * dt
                self.assertAlmostEqual(backend.state().t, expected, places=12)
                self.assertAlmostEqual(backend.state().velocity[0], expected, places=6)

    @requires_mujoco
    def test_varying_dt_keeps_the_clock_and_the_physics_together(self) -> None:
        backend = self._backend()
        target = ControlTarget(accel_cmd=np.array([1.0, 0.0, 0.0]))
        total = 0.0
        for i in range(100):
            dt = 0.005 + 0.005 * (i % 3)
            backend.step(target, dt)
            total += dt
        state = backend.state()
        self.assertAlmostEqual(state.t, total, places=12)
        self.assertAlmostEqual(state.velocity[0], total, places=6)

    @requires_mujoco
    def test_substep_timestep_never_exceeds_the_nominal(self) -> None:
        backend = self._backend({"mass": 1.0, "timestep": 0.002})
        target = ControlTarget(accel_cmd=np.zeros(3))
        backend.step(target, 0.003)
        # 0.003 / 0.002 rounds up to two substeps, so each is dt / 2 and the
        # product is dt exactly rather than approximately.
        self.assertAlmostEqual(backend._model.opt.timestep, 0.0015, places=15)
        self.assertAlmostEqual(backend._model.opt.timestep * 2, 0.003, places=15)

    @requires_mujoco
    def test_refuses_a_dt_that_is_not_a_duration(self) -> None:
        backend = self._backend()
        target = ControlTarget(accel_cmd=np.zeros(3))
        for bad in (0.0, -0.01, float("nan"), float("inf")):
            with self.subTest(dt=bad):
                with self.assertRaises(ValueError):
                    backend.step(target, bad)

    @requires_mujoco
    def test_refuses_a_nonsense_configured_timestep(self) -> None:
        for bad in (0.0, -0.002, float("nan")):
            with self.subTest(timestep=bad):
                with self.assertRaises(ValueError):
                    self._backend({"mass": 1.0, "timestep": bad})


if __name__ == "__main__":
    unittest.main()
