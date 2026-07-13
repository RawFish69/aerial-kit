"""NED (MAVLink/aerospace) <-> ENU (ROS) conversions.

MAVLink reports position/velocity in NED (x=North, y=East, z=Down) and
attitude in the aircraft body frame (FRD: x=Forward, y=Right, z=Down).
ROS convention is ENU (x=East, y=North, z=Up) with FLU body frames
(x=Forward, y=Left, z=Up). These helpers implement the standard fixed
rotations used by MAVROS to bridge the two (see MAVROS `frame_tf.cpp`:
NED_ENU_Q is a 180-degree rotation about the roll axis after a 90-degree
yaw, and AIRCRAFT_BASELINK_Q is a 180-degree roll rotation).
"""
import math

# Fixed quaternion NED -> ENU (rotation of the reference frame itself).
# Equivalent to a 90 degree rotation about Z followed by a 180 degree
# rotation about the new X axis. Quaternions are (w, x, y, z).
_NED_ENU_Q = (0.0, math.sqrt(2.0) / 2.0, math.sqrt(2.0) / 2.0, 0.0)

# Fixed quaternion FRD (aircraft body) -> FLU (ROS baselink): 180 degree
# rotation about the body X (forward) axis.
_AIRCRAFT_BASELINK_Q = (0.0, 1.0, 0.0, 0.0)


def ned_to_enu_position(x_ned, y_ned, z_ned):
    """(North, East, Down) -> (East, North, Up)."""
    return (y_ned, x_ned, -z_ned)


def ned_to_enu_velocity(vx_ned, vy_ned, vz_ned):
    """Same axis mapping as position; velocity has no translation part."""
    return ned_to_enu_position(vx_ned, vy_ned, vz_ned)


def _quat_mul(a, b):
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    )


def _quat_conj(q):
    w, x, y, z = q
    return (w, -x, -y, -z)


def ned_body_to_enu_quaternion(w, x, y, z):
    """Convert a body-to-NED attitude quaternion (MAVLink ATTITUDE_QUATERNION,
    aircraft FRD body in NED world) into a body-to-ENU attitude quaternion
    (ROS FLU body in ENU world): q_enu = NED_ENU_Q * q_ned * AIRCRAFT_BASELINK_Q.

    Returns (x, y, z, w) to match ROS geometry_msgs/Quaternion field order.
    """
    q_ned = (w, x, y, z)
    q_enu = _quat_mul(_quat_mul(_NED_ENU_Q, q_ned), _AIRCRAFT_BASELINK_Q)
    ew, ex, ey, ez = q_enu
    return (ex, ey, ez, ew)


def yaw_from_quaternion_enu(x, y, z, w):
    """ROS ENU quaternion (x, y, z, w) -> yaw in radians about +Z."""
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def euler_to_quaternion_ned(roll, pitch, yaw):
    """Aircraft-frame Euler (radians, NED convention) -> quaternion (w, x, y, z),
    used only by tests to build reference ATTITUDE_QUATERNION-style inputs."""
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    return (w, x, y, z)
