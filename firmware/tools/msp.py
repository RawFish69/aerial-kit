#!/usr/bin/env python3
"""Compatibility entry point for the MSP client in Aerial Kit."""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from aerial_kit.comms.msp import *  # noqa: F401,F403
from aerial_kit.comms.msp import main


if __name__ == "__main__":
    sys.exit(main())
