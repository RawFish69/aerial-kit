"""Click Run in an IDE (or ``python examples/quadrotor/teleop.py``) to fly.

Controls
    W/S or Up/Down      forward / backward
    A/D or Left/Right   strafe left / right
    Space / Shift       climb / descend
    Q/E                 yaw left / right
    X                   neutralize all commands
    P                   pause
    C                   toggle follow / world camera
    -/=                 zoom the follow camera out / in
    Esc                 exit
"""

from __future__ import annotations

import sys
from pathlib import Path

# Works from a fresh clone even when aerial-kit has not been pip-installed,
# which is the usual case behind an IDE's Run button.
REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from aerial_kit.sim.teleop import main  # noqa: E402

if __name__ == "__main__":
    main()
