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

    def test_estimator_tracks_sustained_tilt_with_a_physical_accelerometer(self) -> None:
        # Was a known gap (2026-11-17): with tau 0.5 s accelerometer trust the
        # estimate stayed near 13 deg while the quad rolled to 64 deg. Fixed in
        # the firmware estimator (lib/estimator-and-api); now the bank holds.
        sim = _armed_and_airborne()
        sim.set_sticks(roll=0.5, throttle=0.36)
        sim.step(1.0)
        roll_1s = _roll_pitch_yaw_deg(sim.state.attitude_quat)[0]
        self.assertAlmostEqual(roll_1s, math.degrees(sim.fc.attitude()[0]), delta=5.0)
        sim.step(1.0)
        roll_2s = _roll_pitch_yaw_deg(sim.state.attitude_quat)[0]
        self.assertAlmostEqual(roll_2s, roll_1s, delta=2.0)          # holds, does not keep rolling
        self.assertAlmostEqual(roll_2s, 17.5, delta=5.0)              # half of the 35 deg limit (20.3 measured)
        # Codex's acceptance figure (change-requests.md, 2026-11-18): within 3 deg (2.78 measured).
        self.assertAlmostEqual(roll_2s, math.degrees(sim.fc.attitude()[0]), delta=3.0)

    def test_refuses_airframes_the_firmware_cannot_drive(self) -> None:
        from sim_py.fc_sil import FirmwareInTheLoop

        with self.assertRaises(ValueError):
            FirmwareInTheLoop("research_hex")


class TestPackagedWithoutFirmware(unittest.TestCase):
    def test_missing_firmware_sources_raise_a_clear_error(self) -> None:
        # An installed wheel has sim_py but not firmware/: say so, do not hand
        # the compiler a missing file (clean-install CI checks the same).
        import tempfile
        from pathlib import Path
        from unittest import mock

        import importlib

        fc_build = importlib.import_module("sim_py.fc_sil.build")

        with tempfile.TemporaryDirectory() as empty, mock.patch.object(fc_build, "CORE", Path(empty) / "core"):
            with self.assertRaises(RuntimeError) as ctx:
                fc_build.build()
        message = str(ctx.exception).lower()
        self.assertIn("firmware", message)
        self.assertIn("source", message)


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


def _wing_eul(quat):
    """(roll right-wing-down +, pitch nose-up +, heading) in degrees for a body-FRD wing."""
    r = quat_to_rotmat(quat)
    x, y, z = r[:, 0], r[:, 1], r[:, 2]
    return (math.degrees(math.atan2(-y[2], -z[2])), math.degrees(math.asin(max(-1.0, min(1.0, x[2])))),
            math.degrees(math.atan2(x[1], x[0])))


def _launched_wing(airframe, accel_model="physical", throttle=0.6):
    from sim_py.fc_sil import WingFirmwareInTheLoop

    w = WingFirmwareInTheLoop(airframe, accel_model=accel_model)
    w.set_sticks(arm=False, throttle=0.0)
    w.step(0.5)
    w.set_sticks(arm=True)
    w.step(1.0)
    assert w.fc.state == "armed", w.fc.state
    w.set_sticks(throttle=throttle)
    w.step(0.3)
    w.launch()
    return w


@requires
class TestFirmwareFliesTheWings(unittest.TestCase):
    WINGS = ("twin_wing", "single_wing")

    def test_arms_in_the_hand_and_not_before_the_switch_cycles(self) -> None:
        from sim_py.fc_sil import WingFirmwareInTheLoop

        for airframe in self.WINGS:
            with self.subTest(airframe=airframe):
                w = WingFirmwareInTheLoop(airframe)
                w.set_sticks(arm=True)
                w.step(1.0)
                self.assertEqual(w.fc.state, "disarmed")       # switch on at power-up
                self.assertEqual(w.state.position[2], 60.0)      # still in the hand
                w.set_sticks(arm=False)
                w.step(0.3)
                w.set_sticks(arm=True)
                w.step(1.0)
                self.assertEqual(w.fc.state, "armed")

    def test_angle_mode_holds_level_with_a_gravity_only_accelerometer(self) -> None:
        # Isolates the bridge (IMU frames, mixer rows, servo and thrust scaling)
        # from the estimator: with an accelerometer that reads gravity only the
        # firmware flies both wings level.
        for airframe in self.WINGS:
            with self.subTest(airframe=airframe):
                w = _launched_wing(airframe, "gravity")
                w.step(6.0)
                roll, pitch, _ = _wing_eul(w.state.attitude_quat)
                self.assertLess(abs(roll), 2.0)
                self.assertLess(abs(pitch), 3.0)
                self.assertAlmostEqual(w.state.position[2], 60.0, delta=4.0)
                self.assertAlmostEqual(pitch, math.degrees(w.fc.attitude()[1]), delta=1.0)

    def test_stick_signs(self) -> None:
        # RC convention, as on the quads: roll right -> right wing down; pitch
        # stick forward (positive) -> nose down.
        for airframe in self.WINGS:
            for stick, check in (("roll", lambda r, p: r > 10.0), ("pitch", lambda r, p: p < -5.0)):
                with self.subTest(airframe=airframe, stick=stick):
                    w = _launched_wing(airframe, "gravity")
                    w.step(1.0)
                    w.set_sticks(**{stick: 0.5})
                    w.step(1.0)
                    roll, pitch, _ = _wing_eul(w.state.attitude_quat)
                    self.assertTrue(check(roll, pitch), (airframe, stick, roll, pitch))

    def test_radio_loss_flies_the_descending_circle_not_a_motor_cut(self) -> None:
        # The firmware's wing failsafe: bank ~30 deg, low throttle, circle down.
        w = _launched_wing("twin_wing", "gravity")
        w.step(2.0)
        w.rc_alive = False
        w.step(1.0)
        self.assertEqual(w.fc.state, "circling down")      # AK_FLIGHT_DESCEND, the wing's failsafe
        headings = []
        for _ in range(8):
            w.step(1.0)
            headings.append(_wing_eul(w.state.attitude_quat)[2])
        roll = _wing_eul(w.state.attitude_quat)[0]
        self.assertGreater(abs(roll), 15.0)
        self.assertGreater(float(w.motors.sum()), 0.1)                # still powered: a wing must keep flying
        self.assertAlmostEqual(abs(roll), 30.0, delta=5.0)            # wing_descend_bank_deg
        turned = np.unwrap(np.radians(headings))
        self.assertGreater(abs(math.degrees(turned[-1] - turned[0])), 90.0)

    def test_wing_holds_level_with_a_physical_accelerometer(self) -> None:
        # Was a known gap (2026-11-18): launch thrust tilted the measured
        # specific force ~27 deg, the estimator trusted it in flight and angle
        # mode flew the wing into the ground (truth -29 deg, estimate +1 deg).
        # Fixed in the firmware (gyro-carried attitude in powered flight,
        # PR #26); now both wings hold level on the real accelerometer.
        for airframe in self.WINGS:
            with self.subTest(airframe=airframe):
                w = _launched_wing(airframe, "physical")
                w.step(6.0)
                _, pitch, _ = _wing_eul(w.state.attitude_quat)
                self.assertLess(abs(pitch), 3.0)
                self.assertAlmostEqual(pitch, math.degrees(w.fc.attitude()[1]), delta=1.0)
                self.assertAlmostEqual(w.state.position[2], 60.0, delta=4.0)


class TestWingStickInversion(unittest.TestCase):
    def test_mix_round_trip(self) -> None:
        from sim_py.fc_sil import wing_sticks_from_actuators

        t = 0.4                                           # rad travel
        # Firmware mix: left = -roll + pitch_cmd, right = roll + pitch_cmd (servo units).
        roll, pitch_cmd = 0.3, 0.2                        # pitch_cmd + = nose up = stick back
        left, right = (-roll + pitch_cmd) * t, (roll + pitch_cmd) * t
        s = wing_sticks_from_actuators([4.0, 2.0, left, right], 2, 10.0, t)
        self.assertAlmostEqual(s["roll"], roll)
        self.assertAlmostEqual(s["pitch"], -pitch_cmd)    # RC stick: forward positive
        self.assertAlmostEqual(s["yaw"], (4.0 - 2.0) / 6.0)
        self.assertAlmostEqual(s["throttle"], math.sqrt(6.0 / 20.0))
        single = wing_sticks_from_actuators([8.0, 0.0, 0.0], 1, 8.0, t)
        self.assertEqual((single["throttle"], single["yaw"]), (1.0, 0.0))


if __name__ == "__main__":
    unittest.main()
