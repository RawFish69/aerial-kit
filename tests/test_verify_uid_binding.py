from __future__ import annotations

import importlib.util
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "firmware" / "tools" / "verify_uid_binding.py"


def load_tool():
    spec = importlib.util.spec_from_file_location("verify_uid_binding", SCRIPT)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_uid_pair_tool_refuses_writes_without_confirmation() -> None:
    tool = load_tool()
    with pytest.raises(PermissionError, match="phrase, --write and --confirm"):
        tool.main([
            "--tx", "/dev/ttyACM0", "--rx", "/dev/ttyACM1",
            "--phrase", "test", "--write",
        ])


def test_uid_pair_tool_refuses_same_endpoint_twice() -> None:
    tool = load_tool()
    with pytest.raises(ValueError, match="distinct"):
        tool.main(["--tx", "/dev/ttyACM0", "--rx", "/dev/ttyACM0"])
