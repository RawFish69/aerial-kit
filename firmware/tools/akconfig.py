#!/usr/bin/env python3
"""Compatibility entry point for the Aerial Kit configurator."""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from aerial_kit.comms.akp.config_cli import (
    Configurator,
    Link,
    Pipe,
    Socket,
    Window,
    _field,
    detect,
    main,
)


if __name__ == "__main__":
    sys.exit(main())
