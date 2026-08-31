"""No-argument launcher for interactive quadrotor teleoperation.

``aerial-kit-teleop`` (and ``python -m aerial_kit.sim.teleop``) resolves to
:func:`main`, which opens the bundled quadrotor scenario with no time limit.
"""

from __future__ import annotations

import logging
from dataclasses import replace
from importlib.resources import as_file, files

from .api import SimulationConfig, load_config

#: Bundled scenario flown by the no-argument launcher.
TELEOP_SCENARIO = "quadrotor.yaml"


def teleop_config() -> SimulationConfig:
    """Config used by :func:`main`: the quad scenario with no duration limit.

    ``sim_time=0.0`` means "fly until the pilot exits"; the teleop session only
    enforces a limit when it is positive.
    """
    resource = files("aerial_kit.sim.defaults").joinpath(TELEOP_SCENARIO)
    with as_file(resource) as config_path:
        config = load_config(config_path)
    return replace(config, sim_time=0.0)


def main() -> None:
    """Launch the bundled quadrotor scenario in the interactive viewer."""
    logging.basicConfig(level=logging.INFO, format="%(message)s")

    from sim_py.teleop.session import run_teleop_cli

    run_teleop_cli(teleop_config())


if __name__ == "__main__":
    main()
