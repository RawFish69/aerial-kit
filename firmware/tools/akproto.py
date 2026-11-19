#!/usr/bin/env python3
"""Compatibility entry point for the protocol client in Aerial Kit."""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from aerial_kit.comms.akp.codec import *  # noqa: F403
from aerial_kit.comms.akp.codec import main


if __name__ == "__main__":
    sys.exit(main())
