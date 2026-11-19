"""End to end without hardware: host radio code -> CRSF -> lossy ELRS link -> real AK firmware -> MuJoCo.

Every stage is the code that would run for real except the link and the
plant: ``BackendArmSequencer`` and ``velocity_to_rc`` (host), CRSF framing
and parsing (``aerial_kit.comms.crsf``), and the unmodified AK flight core
(``sim_py.fc_sil``) reading channel counts plus frame arrival times, exactly
as it reads an ELRS receiver on its UART. The firmware alone decides arming,
flight mode and radio-loss failsafe.
"""

from __future__ import annotations

import importlib.util
import math
import random
import shutil
from collections import deque

import numpy as np
import pytest

pytestmark = pytest.mark.skipif(
    importlib.util.find_spec('mujoco') is None or not (shutil.which('cc') or shutil.which('gcc')),
    reason='needs mujoco and a C compiler (builds the AK flight core)',
)

FRAME_HZ = 150.0
LATENCY_S = 0.008


class RadioFirmwareLoop:
    """The host radio backend flying the real firmware over a simulated link."""

    def __init__(self, airframe='fpv_quad_5in', loss=0.0, seed=7, start_on_ground=True, mode='angle'):
        """mode 'velocity': the FC's velocity hold flies it (AUX7, as bringup sets up);
        'angle': the host closes velocity over angle sticks (the older path)."""
        from aerial_kit.airframes.multirotor_specs import load_multirotor_spec
        from aerial_kit.comms.crsf import CrsfParser, make_rc_channels_frame
        from hw_bridge.crsf_radio_mapping import BackendArmSequencer, CrsfChannelMap
        from hw_bridge.rc_mapping import params_from_airframe
        from sim_py.fc_sil import FirmwareInTheLoop, radio_stick_settings

        if mode not in ('angle', 'velocity'):
            raise ValueError(mode)
        self.mode = mode
        self.sim = FirmwareInTheLoop(airframe, velocity_channel=7 if mode == 'velocity' else None)
        # The same settings bringup gives the radio backend in firmware mode:
        # the FC's tilt limit, this airframe's hover throttle, a thrust-scaled kz.
        rs = radio_stick_settings(load_multirotor_spec(airframe), self.sim.fc)
        base = params_from_airframe(airframe, hover_throttle=rs['hover_throttle'], kz=rs['kz'],
                                    kv_xy=rs['kv_xy'], throttle_min=rs['throttle_min'],
                                    throttle_max=rs['throttle_max'])
        self.params = type(base)(**{**base.__dict__, 'max_tilt_deg': rs['max_tilt_deg'],
                                    'tilt_compensation': rs['tilt_compensation']})
        # Optional lag on the host's velocity measurement (s), like telemetry + baro filter.
        self.measure_lag_s = 0.0
        self._v_meas = np.zeros(3)
        self.sequencer = BackendArmSequencer(arm_hold_sec=1.0)
        self.channel_map = CrsfChannelMap(velocity=7 if mode == 'velocity' else 0)
        self.make_frame = make_rc_channels_frame
        self.parser = CrsfParser()
        self.rng = random.Random(seed)
        self.loss = loss
        self.in_flight: deque[tuple[float, bytes]] = deque()
        self.next_frame = 0.0
        self.link_up = True
        self.enabled = False
        self.demand = (0.0, 0.0, 0.0)   # world ENU velocity, m/s
        self.sent = self.delivered = 0

    def _host_frame(self):
        from hw_bridge.crsf_radio_mapping import rotate_world_velocity_to_body, velocity_to_manual_sticks
        from hw_bridge.rc_mapping import velocity_to_rc

        if self.mode == 'velocity':
            # The host knows the FC's heading from its attitude telemetry (the FC's
            # own estimate, NED, clockwise from north) and sends velocity sticks in it.
            yaw_enu = math.pi / 2.0 - float(self.sim.fc.attitude()[2])
            right, forward = rotate_world_velocity_to_body(*self.demand[:2], yaw_enu)
            sticks = velocity_to_manual_sticks(right, forward, self.demand[2], 0.0, max_speed_mps=6.0,
                                               max_climb_mps=3.0, max_yaw_rate_rps=1.5)
            sticks, arm = self.sequencer.update(self.enabled, sticks, self.sim.t)
            return self.make_frame(self.channel_map.encode(
                sticks, enabled=arm, velocity_hold=self.sequencer.armed))

        s = self.sim.state
        if self.measure_lag_s > 0.0:
            alpha = (1.0 / FRAME_HZ) / (self.measure_lag_s + 1.0 / FRAME_HZ)
            self._v_meas += alpha * (np.asarray(s.velocity) - self._v_meas)
        else:
            self._v_meas = np.asarray(s.velocity, dtype=float).copy()
        r = np.asarray(s.attitude_quat)
        yaw = math.atan2(2 * (r[0] * r[3] + r[1] * r[2]), 1 - 2 * (r[2] ** 2 + r[3] ** 2))
        vx, vy = rotate_world_velocity_to_body(*self.demand[:2], yaw)
        mvx, mvy = rotate_world_velocity_to_body(self._v_meas[0], self._v_meas[1], yaw)
        sticks = velocity_to_rc(self.params, vx=vx, vy=vy, vz=self.demand[2], wz=0.0,
                                measured_vxy=(mvx, mvy), measured_vz=float(self._v_meas[2]))
        sticks, arm = self.sequencer.update(self.enabled, sticks, self.sim.t)
        return self.make_frame(self.channel_map.encode(sticks, enabled=arm))

    def run(self, seconds: float) -> None:
        from aerial_kit.comms.crsf import CRSF_FRAMETYPE_RC_CHANNELS_PACKED, unpack_channels

        end = self.sim.t + seconds
        while self.sim.t < end - 1e-9:
            now = self.sim.t
            if now >= self.next_frame:
                self.next_frame += 1.0 / FRAME_HZ
                frame = self._host_frame()
                self.sent += 1
                if self.link_up and self.rng.random() >= self.loss:
                    self.in_flight.append((now + LATENCY_S, frame))
            while self.in_flight and self.in_flight[0][0] <= now:
                _, raw = self.in_flight.popleft()
                for f in self.parser.feed(raw):
                    if f.frame_type == CRSF_FRAMETYPE_RC_CHANNELS_PACKED:
                        self.sim.receive_frame(unpack_channels(f.payload))
                        self.delivered += 1
            self.sim.step(0.001)

    def close(self):
        self.sim.close()


def _tilt_deg(quat):
    w, x, y, z = quat
    return math.degrees(math.acos(max(-1.0, min(1.0, 1 - 2 * (x * x + y * y)))))


def test_radio_arms_the_firmware_and_its_velocity_hold_flies_the_demand():
    # The host sends world velocity on AUX7; the FC closes the loops on its own
    # sensors (firmware velocity hold), not over the link.
    loop = RadioFirmwareLoop(loss=0.10, mode='velocity')
    try:
        loop.run(0.5)
        assert loop.sim.fc.state == 'disarmed'
        loop.enabled = True
        loop.run(1.2)                                  # 1.0 s host arm hold > 0.5 s FW hold
        assert loop.sim.fc.state in ('armed', 'on autopilot')
        assert loop.sim.state.position[2] < 0.1        # throttle held low while arming
        loop.demand = (0.0, 0.0, 1.0)
        loop.run(3.0)
        s = loop.sim.state
        assert loop.sim.fc.state == 'on autopilot' and loop.sim.fc.velocity_mode
        assert abs(s.velocity[2] - 1.0) < 0.2, s.velocity
        loop.demand = (3.0, 0.0, 0.0)
        loop.run(10.0)
        s = loop.sim.state
        assert loop.sim.fc.track_aligned
        assert abs(np.linalg.norm(s.velocity[:2]) - 3.0) < 0.6, s.velocity      # the speed asked for
        assert s.velocity[0] > 1.0, s.velocity       # roughly the right way: heading comes from the track
        assert abs(s.velocity[2]) < 0.3, s.velocity
        assert 0.8 < loop.delivered / loop.sent < 0.97
        assert np.all(np.isfinite(loop.sim.backend.data.qpos))
    finally:
        loop.close()


def test_angle_sticks_still_fly_the_demand_direction():
    # The older path (host closes velocity over angle sticks). With the
    # estimator fixed it no longer overshoots: 3 m/s asked, ~1.3 m/s reached in
    # 4 s at the conservative 0.2/s gain. It must move the right way and stay level.
    loop = RadioFirmwareLoop(loss=0.10)
    try:
        loop.run(0.5)
        loop.enabled = True
        loop.run(1.2)
        assert loop.sim.fc.state == 'armed'
        loop.demand = (0.0, 0.0, 2.0)
        loop.run(2.5)
        assert 3.0 < loop.sim.state.position[2] < 8.0
        loop.demand = (3.0, 0.0, 0.0)
        loop.run(4.0)
        s = loop.sim.state
        assert s.velocity[0] > 1.0 and abs(s.velocity[1]) < 0.5, s.velocity
        assert _tilt_deg(s.attitude_quat) < 25.0
    finally:
        loop.close()


@pytest.mark.parametrize('mode', ['angle', 'velocity'])
def test_link_cut_lets_the_firmware_fail_safe_on_its_own(mode):
    loop = RadioFirmwareLoop(mode=mode)
    try:
        loop.run(0.3)
        loop.enabled = True
        loop.run(1.2)
        loop.demand = (0.0, 0.0, 2.0)
        loop.run(2.0)
        flying = 'on autopilot' if mode == 'velocity' else 'armed'     # velocity hold is MANAGED
        assert loop.sim.fc.state == flying
        loop.link_up = False                           # TX keeps sending, nothing arrives
        loop.run(0.1)
        assert loop.sim.fc.state == flying             # inside the FC's receiver timeout
        loop.run(0.6)
        assert loop.sim.fc.state == 'failsafe'
        assert np.all(loop.sim.motors == 0.0)
        loop.link_up = True                            # link back: latched until re-armed
        loop.run(0.5)
        assert loop.sim.fc.state == 'failsafe'
    finally:
        loop.close()


def test_disable_disarms_the_firmware():
    loop = RadioFirmwareLoop()
    try:
        loop.run(0.3)
        loop.enabled = True
        loop.run(1.5)
        assert loop.sim.fc.state == 'armed'
        loop.enabled = False
        loop.run(0.2)
        assert loop.sim.fc.state == 'disarmed'
        assert np.all(loop.sim.motors == 0.0)
    finally:
        loop.close()


def _fly_with_lagging_estimate(loop, seconds=8.0):
    loop.measure_lag_s = 1.0           # CRSF telemetry + the host's 1 Hz baro/GPS filter
    loop.run(0.3)
    loop.enabled = True
    loop.run(1.2)
    loop.demand = (0.0, 0.0, 1.0)
    loop.run(3.0)
    loop.demand = (1.1, 1.1, 0.0)      # the first leg of the bringup demo mission
    worst, z_min = 0.0, 1e9
    for _ in range(int(seconds / 0.1)):
        loop.run(0.1)
        worst = max(worst, _tilt_deg(loop.sim.state.attitude_quat))
        z_min = min(z_min, loop.sim.state.position[2])
    return worst, z_min


def test_survives_a_one_second_lag_in_the_velocity_estimate():
    # Regression (2026-11-18, bringup flight_controller:=firmware): the
    # simulator-mode gains (kz 0.2, kv_xy 0.5) first ran away upward to 1.5 km,
    # then flipped the quad on the first horizontal leg (original estimator).
    loop = RadioFirmwareLoop()
    try:
        worst, z_min = _fly_with_lagging_estimate(loop)
        assert worst < 25.0, worst            # observed ~12 deg
        assert z_min > 1.5, z_min             # climbed to ~3 m; observed >= 2.5 m
        assert loop.sim.fc.state == 'armed'
    finally:
        loop.close()


def test_the_old_gains_saturate_the_tilt_limit():
    # Shows the test above is sensitive: same loop, simulator-mode gains. On the
    # original firmware estimator they flipped the quad; on the fixed one they
    # still bank it into the FC's 35 deg limit.
    import dataclasses

    loop = RadioFirmwareLoop()
    loop.params = dataclasses.replace(loop.params, kv_xy=0.5, kz=0.2, throttle_min=0.05,
                                      throttle_max=0.95, tilt_compensation=False)
    try:
        worst, _ = _fly_with_lagging_estimate(loop)
        assert worst > 35.0, worst
    finally:
        loop.close()
