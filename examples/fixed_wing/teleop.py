"""Click Run in an IDE (or ``python examples/fixed_wing/teleop.py``) to fly.

RC-plane-style controls -- not the quadrotor's drone-style ones. The twin wing
has no rudder, so Q/E biases differential thrust rather than yawing directly.

Controls
    W/S or Up/Down      pitch (dive / climb)
    A/D or Left/Right   bank (turn right / left)
    Space / Shift       throttle up / down
    Q/E                 differential-thrust yaw nudge
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

from aerial_kit.sim.teleop import teleop_config  # noqa: E402
from sim_py.teleop.session import run_teleop_cli  # noqa: E402

if __name__ == "__main__":
    run_teleop_cli(teleop_config("fixed-wing"))
