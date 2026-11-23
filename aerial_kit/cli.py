"""Canonical ``ak`` command dispatcher.

The protocol families are implemented by :mod:`aerial_kit.comms.cli`; the
simulator and teleoperation commands retain their original argument parsers.
"""

from __future__ import annotations

import argparse
import sys
from collections.abc import Sequence


def build_parser(prog: str = "ak") -> argparse.ArgumentParser:
    """Return the top-level command-family help.

    Subcommand parsers stay with their implementations. This parser exists so
    ``ak --help`` advertises every family without duplicating hundreds of
    protocol and simulation options in the dispatcher.
    """
    parser = argparse.ArgumentParser(
        prog=prog,
        description=(
            "Aerial Kit radios, flight controllers, MAVLink vehicles, "
            "simulation, and teleoperation."
        ),
        epilog=(
            "Run '<command> --help' for command-specific options. "
            "Radio and flight-controller writes require --write; arm-capable "
            "commands add their own explicit confirmation."
        ),
    )
    commands = parser.add_subparsers(dest="command", metavar="COMMAND")
    commands.add_parser(
        "radio",
        help="ELRS/CRSF radio status, identity, parameters, bench and teleop",
    )
    commands.add_parser(
        "fc",
        help="Aerial Kit AKP or Betaflight MSP flight-controller access",
    )
    commands.add_parser(
        "mavlink",
        help="PX4/ArduPilot MAVLink vehicle access",
    )
    commands.add_parser(
        "doctor",
        help="list serial endpoints and optionally identify a connected device",
    )
    commands.add_parser(
        "sim",
        help="run the Python simulation",
    )
    commands.add_parser(
        "teleop",
        help="fly the simulated vehicle with the keyboard",
    )
    return parser


def _exit_code(value: int | None) -> int:
    return 0 if value is None else value


def main(argv: Sequence[str] | None = None) -> int:
    args = list(sys.argv[1:] if argv is None else argv)
    if not args:
        build_parser().print_help(sys.stderr)
        return 2
    if args[0] in {"-h", "--help", "help"}:
        build_parser().print_help()
        return 0
    if args[0] == "doctor":
        from .doctor import main as doctor_main

        return _exit_code(doctor_main(args[1:]))
    if args and args[0] == "sim":
        from .sim.cli import main as sim_main

        return _exit_code(sim_main(args[1:]))
    if args and args[0] == "teleop":
        from .sim.teleop import main as teleop_main

        return _exit_code(teleop_main(args[1:]))
    from .comms.cli import main as comms_main

    return comms_main(args, prog="ak")


if __name__ == "__main__":
    raise SystemExit(main())
