"""Swarm plant and formation commander, driven in-process with a fake clock."""

import importlib.util

import numpy as np
import pytest

pytest.importorskip('rclpy')
if importlib.util.find_spec('mujoco') is None:
    pytest.skip('needs mujoco', allow_module_level=True)

import rclpy  # noqa: E402
from geometry_msgs.msg import Twist  # noqa: E402
from std_msgs.msg import Bool  # noqa: E402

from sim_bridge import swarm_backend_adapter_node as adapter  # noqa: E402


@pytest.fixture
def swarm(monkeypatch):
    clock = {'t': 500.0}
    monkeypatch.setattr(adapter.time, 'monotonic', lambda: clock['t'])
    rclpy.init(args=['--ros-args', '-p', 'n:=3'])
    node = adapter.SwarmBackendAdapterNode()
    yield node, clock
    node.destroy_node()
    rclpy.shutdown()


def _run(node, clock, seconds, commands=None):
    for _ in range(int(seconds / node.period)):
        clock['t'] += node.period
        for i, (en, vz) in (commands or {}).items():
            node._on_enable(i, Bool(data=en))
            if vz is not None:
                t = Twist()
                t.linear.z = vz
                node._on_twist(i, t)
        node._tick()


def test_each_drone_follows_its_own_topics(swarm):
    node, clock = swarm
    assert [node.namespace_of(i) for i in range(3)] == ['/uav0', '/uav1', '/uav2']
    _run(node, clock, 2.0, {0: (True, 1.0), 1: (True, 0.5), 2: (False, None)})
    z = node.swarm.positions()[:, 2]
    assert z[0] > 1.2 and 0.5 < z[1] < 1.0 and z[2] < 0.2, z   # 1 m/s, 0.5 m/s from rest, idle
    assert [d.source for d in node.drones] == ['cmd_twist', 'cmd_twist', 'idle']


def test_stale_commands_hold_and_disarm_lands(swarm):
    node, clock = swarm
    _run(node, clock, 2.0, {0: (True, 1.0), 1: (True, 1.0), 2: (True, 1.0)})
    _run(node, clock, 1.0, {0: (True, None), 1: (False, None)})   # drone 0: no twist; 1: disarmed
    assert node.drones[0].source == 'hold'
    assert node.drones[1].source == 'auto_land'
    assert abs(node.swarm.states()[0].velocity[2]) < 0.4
    _run(node, clock, 5.0, {1: (False, None)})
    assert node.drones[1].source == 'idle'
    assert node.swarm.positions()[1, 2] < 0.2
    assert np.all(np.isfinite(node.swarm.data.qpos))


def test_rejects_bad_sizes():
    rclpy.init(args=['--ros-args', '-p', 'n:=0'])
    try:
        with pytest.raises(ValueError, match='n must be'):
            adapter.SwarmBackendAdapterNode()
    finally:
        rclpy.shutdown()
