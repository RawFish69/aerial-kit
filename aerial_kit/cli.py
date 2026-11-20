"""Canonical ``ak`` command dispatcher.

The protocol families are implemented by :mod:`aerial_kit.comms.cli`; the
simulator and teleoperation commands retain their original argument parsers.
"""

from __future__ import annotations

import sys
from collections.abc import Sequence


def main(argv: Sequence[str] | None = None) -> int:
    args = list(sys.argv[1:] if argv is None else argv)
    if args and args[0] == "sim":
        from .sim.cli import main as sim_main

        sim_main(args[1:])
        return 0
    if args and args[0] == "teleop":
        from .sim.teleop import main as teleop_main

        teleop_main(args[1:])
        return 0
    from .comms.cli import main as comms_main

    return comms_main(args, prog="ak")


if __name__ == "__main__":
    raise SystemExit(main())
