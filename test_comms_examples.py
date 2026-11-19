from __future__ import annotations

import subprocess
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parent


def test_protocol_examples_run_without_hardware() -> None:
    examples = [
        ROOT / "examples/comms/crsf_channels.py",
        ROOT / "examples/comms/akp_hello.py",
        ROOT / "examples/comms/mavlink_status.py",
        ROOT / "examples/comms/msp_info.py",
    ]
    for example in examples:
        result = subprocess.run(
            [sys.executable, str(example), "--demo"],
            cwd=ROOT,
            check=True,
            capture_output=True,
            text=True,
        )
        assert result.stdout.strip()
