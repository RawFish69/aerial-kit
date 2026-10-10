"""Make ``uav_control`` and ``aerial_kit`` importable without a colcon build."""

import sys
from pathlib import Path

_PKG = Path(__file__).resolve().parents[1]
_REPO = Path(__file__).resolve().parents[4]
for path in (_PKG, _REPO):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))
