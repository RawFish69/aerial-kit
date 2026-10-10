"""Pure RTL guidance: climb to safe alt, cruise home, signal arrival (caller then lands)."""

import math
from dataclasses import dataclass

RTL_CLIMB = "climb"
RTL_CRUISE = "cruise"
RTL_ARRIVED = "arrived"


@dataclass(frozen=True)
class RtlParams:
    altitude_m: float
    cruise_speed_mps: float
    arrival_radius_m: float
    kp: float


def rtl_command(params: RtlParams, pos, home):
    """Return (vx, vy, vz, phase) in world frame. pos=(x,y,z), home=(x,y)."""
    x, y, z = pos
    hx, hy = home
    dx, dy = hx - x, hy - y
    dist = math.hypot(dx, dy)

    if z < params.altitude_m - 0.5:
        vz = min(params.cruise_speed_mps, params.altitude_m - z)
        return 0.0, 0.0, vz, RTL_CLIMB

    if dist <= params.arrival_radius_m:
        return 0.0, 0.0, 0.0, RTL_ARRIVED

    vx = params.kp * dx
    vy = params.kp * dy
    speed = math.hypot(vx, vy)
    if speed > params.cruise_speed_mps:
        s = params.cruise_speed_mps / speed
        vx, vy = vx * s, vy * s
    return vx, vy, 0.0, RTL_CRUISE


def resolve_mode_request(requested: int, rtl_landing: bool, *, rtl_mode: int, land_mode: int):
    """Apply a ground-station mode request; returns ``(mode, rtl_landing)``.

    RTL ends by handing over to LAND. Ground stations re-send their mode
    request continuously (the command manager times out without it), so a
    plain ``mode = request`` turned the next RTL request back into RTL - whose
    first phase climbs back to the RTL altitude. The aircraft hovered around
    14.5 m indefinitely in SITL. While ``rtl_landing`` is latched, an RTL
    request keeps it landing; any other request clears the latch.
    """
    if requested == rtl_mode and rtl_landing:
        return land_mode, True
    return requested, False
