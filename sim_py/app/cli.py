"""CLI argument parsing for the standalone simulator."""

from __future__ import annotations

import argparse


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Standalone UAV simulator (no ROS2).")
    parser.add_argument(
        "--terrain",
        type=str,
        choices=["forest", "mountains", "plains"],
        default=None,
        help="Override terrain type from YAML.",
    )
    parser.add_argument(
        "--terrain-config",
        type=str,
        default=None,
        help="Path to terrain_params.yaml (defaults to ROS2 config).",
    )
    parser.add_argument(
        "--controller",
        type=str,
        choices=["pid", "lqr", "mpc", "teleop", "l1_tecs"],
        default="pid",
        help="Controller type to use. l1_tecs pairs with --airframe twin_wing "
        "--backend fixedwing (AIRSPEED_NAV) -- see sim_py/INFO.md.",
    )
    parser.add_argument(
        "--backend",
        type=str,
        # fixedwing missions always start at zero velocity (run_simulation() has no
        # runway/launch model), so a CLI run belly-flops before gaining lift rather
        # than demonstrating cruise flight -- exercise it directly in tests instead
        # (sim_py/tests/test_fixed_wing_guidance.py starts at cruise trim).
        choices=["pointmass", "rotorpy", "fixedwing"],
        default=None,
        help="Dynamics backend to use (default: pointmass or simulation.backend).",
    )
    parser.add_argument(
        "--airframe",
        type=str,
        choices=["quad", "hex", "octo", "twin_wing"],
        default=None,
        help="Airframe profile to use (default: quad or vehicle.airframe).",
    )
    parser.add_argument(
        "--sim-time",
        type=float,
        default=None,
        help="Total simulation time [s] (overrides sim_config.yaml).",
    )
    parser.add_argument(
        "--dt",
        type=float,
        default=None,
        help="Simulation time step [s] (overrides sim_config.yaml).",
    )
    parser.add_argument(
        "--sim-config",
        type=str,
        default="sim_py/sim_config.yaml",
        help="Path to sim_config.yaml.",
    )
    return parser.parse_args()
