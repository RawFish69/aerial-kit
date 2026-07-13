import math

from mavlink_bridge.frame_transforms import (
    euler_to_quaternion_ned,
    ned_body_to_enu_quaternion,
    ned_to_enu_position,
    ned_to_enu_velocity,
    yaw_from_quaternion_enu,
)


def test_position_axis_mapping():
    east, north, up = ned_to_enu_position(1.0, 2.0, -3.0)
    assert (east, north, up) == (2.0, 1.0, 3.0)


def test_velocity_axis_mapping():
    ve, vn, vu = ned_to_enu_velocity(0.5, -1.5, 2.0)
    assert (ve, vn, vu) == (-1.5, 0.5, -2.0)


def test_nose_north_in_ned_is_yaw_90_in_enu():
    w, x, y, z = euler_to_quaternion_ned(0.0, 0.0, 0.0)
    qx, qy, qz, qw = ned_body_to_enu_quaternion(w, x, y, z)
    yaw = yaw_from_quaternion_enu(qx, qy, qz, qw)
    assert math.isclose(yaw, math.pi / 2.0, abs_tol=1e-6)


def test_nose_east_in_ned_is_yaw_0_in_enu():
    w, x, y, z = euler_to_quaternion_ned(0.0, 0.0, math.pi / 2.0)
    qx, qy, qz, qw = ned_body_to_enu_quaternion(w, x, y, z)
    yaw = yaw_from_quaternion_enu(qx, qy, qz, qw)
    assert math.isclose(yaw, 0.0, abs_tol=1e-6)


def test_nose_south_in_ned_is_yaw_negative_90_in_enu():
    w, x, y, z = euler_to_quaternion_ned(0.0, 0.0, math.pi)
    qx, qy, qz, qw = ned_body_to_enu_quaternion(w, x, y, z)
    yaw = yaw_from_quaternion_enu(qx, qy, qz, qw)
    assert math.isclose(abs(yaw), math.pi / 2.0, abs_tol=1e-6)


def test_quaternion_stays_normalized():
    w, x, y, z = euler_to_quaternion_ned(0.3, -0.2, 1.1)
    qx, qy, qz, qw = ned_body_to_enu_quaternion(w, x, y, z)
    norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    assert math.isclose(norm, 1.0, abs_tol=1e-9)
