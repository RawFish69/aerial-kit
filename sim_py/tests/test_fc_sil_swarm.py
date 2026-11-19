"""A swarm in which every drone runs its own AK firmware."""

from __future__ import annotations

import importlib.util
import math
import shutil
import unittest

import numpy as np

HAVE = (importlib.util.find_spec("mujoco") is not None
        and (shutil.which("cc") or shutil.which("gcc")) is not None)
requires = unittest.skipUnless(HAVE, "needs mujoco and a C compiler")


class TestCompanionLaw(unittest.TestCase):
    def test_sticks_point_the_right_way_and_compensate_tilt(self) -> None:
        from sim_py.fc_sil.swarm import velocity_to_angle_sticks

        fwd = velocity_to_angle_sticks([0, 0, 0], [2.0, 0, 0], 0.0, 0.4, 35.0)
        self.assertGreater(fwd["pitch"], 0.0)                 # stick forward = nose down
        self.assertAlmostEqual(fwd["roll"], 0.0)
        right = velocity_to_angle_sticks([0, 0, 0], [0, -2.0, 0], 0.0, 0.4, 35.0)   # -y is right in ENU at yaw 0
        self.assertGreater(right["roll"], 0.0)
        hover = velocity_to_angle_sticks([0, 0, 0], [0, 0, 0], 0.0, 0.4, 35.0)
        self.assertAlmostEqual(hover["throttle"], 0.4)
        self.assertGreater(fwd["throttle"], 0.4)              # tilted: more throttle for the same lift
        turned = velocity_to_angle_sticks([0, 0, 0], [0, 2.0, 0], math.pi / 2, 0.4, 35.0)
        self.assertGreater(turned["pitch"], 0.0)              # facing +y, +y is forward


def _climbed(accel_model, n):
    from sim_py.fc_sil.swarm import FirmwareSwarm
    from sim_py.swarm import FormationGains, formation_velocity

    fs = FirmwareSwarm(n=n, accel_model=accel_model)
    fs.arm()
    assert fs.fc_states == ["armed"] * n, fs.fc_states
    p0 = fs.swarm.positions()
    up = np.column_stack([p0[:, :2], np.full(n, 2.0)])
    fs.fly_velocity(lambda p: formation_velocity(p, up, FormationGains()), 5.0)
    return fs


def _circle(fs, seconds=10.0):
    from sim_py.swarm import FormationGains, assign_slots, formation_slots, formation_velocity

    slots = formation_slots("circle", fs.n, 3.0)
    tg = slots[assign_slots(fs.swarm.positions(), slots)]
    fs.fly_velocity(lambda p: formation_velocity(p, tg, FormationGains()), seconds)
    return float(np.linalg.norm(fs.swarm.positions() - tg, axis=1).max())


@requires
class TestFirmwareSwarm(unittest.TestCase):
    def test_every_drone_arms_its_own_firmware_and_takes_off(self) -> None:
        fs = _climbed("physical", 4)
        np.testing.assert_allclose(fs.swarm.positions()[:, 2], 2.0, atol=0.3)
        self.assertEqual(len({id(fc) for fc in fs.fcs}), 4)
        self.assertEqual(fs.contacts_seen, set())

    def test_radio_loss_on_one_drone_fails_only_that_drone(self) -> None:
        fs = _climbed("physical", 3)
        fs.rc_alive[1] = False
        fs.step(0.6)
        self.assertEqual(fs.fc_states, ["armed", "failsafe", "armed"])
        np.testing.assert_array_equal(fs.motors[1], 0.0)

    def test_formation_with_a_gravity_only_accelerometer(self) -> None:
        # Isolates the bridge and the companion law from the estimator.
        fs = _climbed("gravity", 4)
        self.assertLess(_circle(fs), 0.4)
        self.assertGreater(fs.min_separation_seen, 0.6)

    @unittest.expectedFailure
    def test_known_gap_formation_with_a_physical_accelerometer(self) -> None:
        # KNOWN GAP (2026-11-18): below the firmware's 7 deg gate the quad
        # estimator still levels on the accelerometer, which reads thrust while
        # accelerating: commanded ~4 deg, flown ~7 deg, estimated ~2 deg. A
        # 1 m/s velocity step overshoots to 1.32 m/s and the 4-drone circle is
        # 3.3 m out after 10 s (0.22 m with a gravity-only accelerometer).
        # See docs/evidence/fc-sil-swarm-small-tilt-finding-2026-11-18.md.
        fs = _climbed("physical", 4)
        self.assertLess(_circle(fs), 0.4)


if __name__ == "__main__":
    unittest.main()
