"""No-argument launcher for interactive teleoperation.

``aerial-kit-teleop`` (and ``python -m aerial_kit.sim.teleop``) resolves to
:func:`main`, which opens the bundled quadrotor scenario with no time limit.
Pass ``--airframe fixed-wing`` for the twin-wing scenario instead.
"""

from __future__ import annotations

import logging
from dataclasses import replace
from importlib.resources import as_file, files

from .api import SimulationConfig, load_config

#: Bundled scenario flown by the no-argument launcher, keyed by --airframe.
TELEOP_SCENARIOS = {"quad": "quadrotor.yaml", "fixed-wing": "fixed_wing.yaml"}


def teleop_config(airframe: str = "quad") -> SimulationConfig:
    """Config used by :func:`main`: the bundled scenario with no duration limit.

    ``sim_time=0.0`` means "fly until the pilot exits"; the teleop session only
    enforces a limit when it is positive.
    """
    if airframe not in TELEOP_SCENARIOS:
        raise ValueError(
            f"Unknown teleop airframe '{airframe}'; choose one of {sorted(TELEOP_SCENARIOS)}."
        )
    resource = files("aerial_kit.sim.defaults").joinpath(TELEOP_SCENARIOS[airframe])
    with as_file(resource) as config_path:
        config = load_config(config_path)
    return replace(config, sim_time=0.0)


def main() -> None:
    """Launch a bundled scenario in the interactive viewer."""
    import argparse

    parser = argparse.ArgumentParser(description="Interactive aerial-kit teleoperation.")
    parser.add_argument(
        "--airframe",
        choices=sorted(TELEOP_SCENARIOS),
        default="quad",
        help="Bundled scenario to fly (default: quad).",
    )
    args = parser.parse_args()

    logging.basicConfig(level=logging.INFO, format="%(message)s")

    from sim_py.teleop.session import run_teleop_cli

    run_teleop_cli(teleop_config(args.airframe))


if __name__ == "__main__":
    main()
