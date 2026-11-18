"""Firmware-in-the-loop: the unmodified AK flight core flying MuJoCo airframes."""

from __future__ import annotations

import importlib.util
import math
import shutil
import unittest

import numpy as np

from aerial_kit.dynamics.rotations import quat_to_rotmat

HAVE = (importlib.util.find_spec("mujoco") is not None
        and (shutil.which("cc") or shutil.which("gcc")) is not None)
requires = unittest.skipUnless(HAVE, "needs mujoco and a C compiler")


def _roll_pitch_yaw_deg(quat):
    r = quat_to_rotmat(quat)
    return np.degrees([math.atan2(r[2, 1], r[2, 2]), -math.asin(r[2, 0]), math.atan2(r[1, 0], r[0, 0])])


def _armed_and_airborne(airframe="fpv_quad_5in", **kw):
    from sim_py.fc_sil import FirmwareInTheLoop

    sim = FirmwareInTheLoop(airframe, **kw)
    sim.set_sticks(arm=False, throttle=0.0)
    sim.step(0.5)
    sim.set_sticks(arm=True)
    sim.step(1.0)
    assert sim.fc.state == "armed", sim.fc.state
    sim.set_sticks(throttle=0.45)
    sim.step(1.2)
    sim.set_sticks(throttle=0.337)
    sim.step(0.5)
    return sim


@requires
class TestFlightCoreOnHost(unittest.TestCase):
    def test_library_builds_and_layout_matches_the_firmware(self) -> None:
        from sim_py.fc_sil import AkFlightCore

        fc = AkFlightCore()
        L = fc.layout
        # firmware/src/core/flight: ak_types.h + ak_rc_default_config
        self.assertEqual((L.roll, L.pitch, L.throttle, L.yaw, L.mode, L.arm), (0, 1, 2, 3, 4, 5))
        self.assertEqual((L.min, L.mid, L.max), (172, 992, 1811))
        self.assertEqual(fc.n_motors, 4)
        fc.close()

    def test_imu_conversion_reads_plus_one_g_level_and_firmware_signs(self) -> None:
        from sim_py.fc_sil import flu_to_firmware_imu

        gyro, accel = flu_to_firmware_imu([0.1, 0.2, 0.3], np.zeros(3), np.eye(3))
        np.testing.assert_allclose(accel, [0.0, 0.0, 1.0])
        np.testing.assert_allclose(gyro, [0.1, -0.2, -0.3])          # FLU -> FRD
        # Nose up 10 deg (FLU rotation -10 deg about y): firmware accel x = -sin.
        a = math.radians(10.0)
        rot = np.array([[math.cos(a), 0, -math.sin(a)], [0, 1, 0], [math.sin(a), 0, math.cos(a)]])
        _, accel = flu_to_firmware_imu(np.zeros(3), np.zeros(3), rot)
        self.assertAlmostEqual(accel[0], -math.sin(a), places=6)


@requires
class TestFirmwareFliesMujoco(unittest.TestCase):
    def test_arming_follows_the_firmware_rules(self) -> None:
        from sim_py.fc_sil import FirmwareInTheLoop

        sim = FirmwareInTheLoop("fpv_quad_5in")
        sim.set_sticks(arm=True)                      # switch already on at power-up
        sim.step(1.0)
        self.assertEqual(sim.fc.state, "disarmed")
        np.testing.assert_array_equal(sim.motors, 0.0)
        sim.set_sticks(arm=False)
        sim.step(0.3)
        sim.set_sticks(arm=True, throttle=0.5)        # throttle not low
        sim.step(1.0)
        self.assertEqual(sim.fc.state, "disarmed")
        sim.set_sticks(arm=False, throttle=0.0)
        sim.step(0.3)
        sim.set_sticks(arm=True)
        sim.step(1.0)
        self.assertEqual(sim.fc.state, "armed")
        self.assertTrue(np.all(sim.motors > 0.0))      # idle spin
        self.assertLess(sim.state.position[2], 0.1)    # still on the ground

    def test_takes_off_and_holds_level(self) -> None:
        sim = _armed_and_airborne()
        sim.step(1.0)
        s = sim.state
        self.assertGreater(s.position[2], 5.0)
        self.assertLess(np.abs(_roll_pitch_yaw_deg(s.attitude_quat)[:2]).max(), 1.0)
        self.assertTrue(np.all(np.isfinite(sim.backend.data.qpos)))

    def test_stick_signs(self) -> None:
        cases = {"roll": (0, +1), "pitch": (1, +1), "yaw": (2, -1)}
        for stick, (axis, sign) in cases.items():
            with self.subTest(stick=stick):
                sim = _armed_and_airborne()
                sim.set_sticks(**{stick: 0.5})
                sim.step(0.4)
                # FLU: +roll = right side down; +pitch (FLU about y) = nose down;
                # yaw stick right = clockwise from above = negative FLU yaw rate.
                rate = sim.state.body_rates[axis]
                self.assertGreater(sign * rate, 0.2, f"{stick}: rate {rate}")

    def test_acro_commands_a_rate(self) -> None:
        sim = _armed_and_airborne()
        sim.set_sticks(angle_mode=False, roll=0.3)
        sim.step(0.2)
        self.assertGreater(sim.state.body_rates[0], 1.0)

    def test_radio_loss_cuts_the_motors_and_latches(self) -> None:
        # The firmware's documented quad failsafe: motors off, latched.
        sim = _armed_and_airborne()
        sim.rc_alive = False
        sim.step(0.6)
        self.assertEqual(sim.fc.state, "failsafe")
        np.testing.assert_array_equal(sim.motors, 0.0)
        sim.rc_alive = True
        sim.step(0.3)
        self.assertEqual(sim.fc.state, "failsafe")

    def test_angle_mode_holds_the_angle_with_a_gravity_only_accelerometer(self) -> None:
        # Control experiment: with an accelerometer that reads gravity only,
        # the firmware's estimate equals the truth and angle mode holds.
        sim = _armed_and_airborne()
        orig = sim.imu

        def ideal():
            (gyro, _), v = orig()
            g = quat_to_rotmat(sim.state.attitude_quat).T @ np.array([0.0, 0.0, 9.81])
            return (gyro, np.array([-g[0], g[1], g[2]]) / 9.81), v

        sim.imu = ideal
        sim.set_sticks(roll=0.5, throttle=0.36)
        sim.step(2.0)
        true_roll = _roll_pitch_yaw_deg(sim.state.attitude_quat)[0]
        self.assertAlmostEqual(true_roll, math.degrees(sim.fc.attitude()[0]), delta=1.0)
        self.assertAlmostEqual(true_roll, 17.5, delta=2.0)

    @unittest.expectedFailure
    def test_known_gap_estimator_tracks_sustained_tilt_with_a_physical_accelerometer(self) -> None:
        # KNOWN GAP (2026-11-17): ak_estimator trusts the accelerometer with a
        # 0.5 s time constant whenever |a| ~ 1 g. On a multirotor the
        # accelerometer sees thrust, not gravity, so a sustained tilt is
        # under-read (truth 37 deg vs estimate 13 deg after 1 s) and angle
        # mode keeps rolling. This sim has no rotor drag, which would make the
        # real error smaller but not remove it. Remove expectedFailure when
        # the firmware estimator is fixed.
        sim = _armed_and_airborne()
        sim.set_sticks(roll=0.5, throttle=0.36)
        sim.step(1.0)
        true_roll = _roll_pitch_yaw_deg(sim.state.attitude_quat)[0]
        self.assertAlmostEqual(true_roll, math.degrees(sim.fc.attitude()[0]), delta=5.0)

    def test_refuses_airframes_the_firmware_cannot_drive(self) -> None:
        from sim_py.fc_sil import FirmwareInTheLoop

        with self.assertRaises(ValueError):
            FirmwareInTheLoop("research_hex")


class TestFirmwareRadio(unittest.TestCase):
    def test_arm_sequence_holds_throttle_low_then_passes_sticks(self) -> None:
        from sim_py.fc_sil import FirmwareRadio

        radio = FirmwareRadio(hover_throttle=0.34, arm_hold_s=0.8)
        self.assertFalse(radio.sticks(0.0, arm=False)["arm"])
        first = radio.sticks(1.0, arm=True, roll=0.7, throttle=0.9)
        self.assertTrue(first["arm"])
        self.assertEqual((first["throttle"], first["roll"]), (0.0, 0.0))
        held = radio.sticks(1.79, arm=True, roll=0.7, throttle=0.9)
        self.assertEqual(held["throttle"], 0.0)
        live = radio.sticks(1.81, arm=True, roll=0.7, throttle=0.9)
        self.assertEqual(live["roll"], 0.7)
        self.assertGreater(live["throttle"], 0.34)
        self.assertFalse(radio.sticks(2.0, arm=False)["arm"])
        self.assertEqual(radio.sticks(2.1, arm=True)["throttle"], 0.0)   # re-arm holds again

    def test_throttle_centre_is_hover_and_modes(self) -> None:
        from sim_py.fc_sil import FirmwareRadio

        radio = FirmwareRadio(hover_throttle=0.34, arm_hold_s=0.0)
        self.assertAlmostEqual(radio.firmware_throttle(0.5), 0.34)
        self.assertEqual(radio.firmware_throttle(0.0), 0.0)
        self.assertAlmostEqual(radio.firmware_throttle(1.0), 0.34 * math.sqrt(2.0))   # 2x hover thrust
        self.assertAlmostEqual(radio.firmware_throttle(0.25), 0.34 * math.sqrt(0.5))
        self.assertEqual(FirmwareRadio(0.8, max_thrust_ratio=4.0).firmware_throttle(1.0), 1.0)
        self.assertFalse(radio.sticks(0.0, arm=True, mode="acro")["angle_mode"])
        self.assertTrue(radio.sticks(0.0, arm=True, mode="velocity")["angle_mode"])


if __name__ == "__main__":
    unittest.main()
