"""TECS-lite: total-energy throttle, energy-distribution pitch.

Full TECS (Lambregts, 1983) tracks total-energy *rate* and energy-distribution
*rate* through separate PI loops with several tuning knobs. This is
deliberately the minimal version plan 04 asks for: proportional control on the
energy *errors* themselves (not their rates), sized to be a first working
longitudinal loop rather than a tuned autopilot -- it is what stops the
classic failure mode where a naive altitude controller pitches up into a
stall while airspeed is already low, by coupling the two through one shared
error decomposition instead of controlling them independently.
"""

from __future__ import annotations

from dataclasses import dataclass


@dataclass
class TecsGains:
    kp_throttle: float = 0.15  # N per meter of specific-energy error
    kp_pitch: float = 0.08  # rad per meter of energy-balance error
    trim_throttle_n: float = 3.0
    max_thrust_n: float = 10.0
    max_pitch_rad: float = 0.35


def tecs_command(
    airspeed_mps: float,
    airspeed_cmd_mps: float,
    altitude_m: float,
    altitude_cmd_m: float,
    gains: TecsGains,
    gravity_mps2: float = 9.81,
) -> tuple[float, float]:
    """Returns ``(throttle_cmd_n, pitch_cmd_rad)``.

    ``energy_error`` is positive when the aircraft has too little total
    (kinetic + potential) energy for the commanded state -- throttle up.
    ``balance_error`` is positive when energy is misallocated toward speed
    and away from altitude -- pitch up trades some speed for climb.
    """
    kinetic_error_m = (airspeed_cmd_mps**2 - airspeed_mps**2) / (2.0 * gravity_mps2)
    potential_error_m = altitude_cmd_m - altitude_m

    energy_error = kinetic_error_m + potential_error_m
    balance_error = potential_error_m - kinetic_error_m

    throttle_cmd = gains.trim_throttle_n + gains.kp_throttle * energy_error
    pitch_cmd = gains.kp_pitch * balance_error

    throttle_cmd = float(min(max(throttle_cmd, 0.0), gains.max_thrust_n))
    pitch_cmd = float(min(max(pitch_cmd, -gains.max_pitch_rad), gains.max_pitch_rad))
    return throttle_cmd, pitch_cmd
