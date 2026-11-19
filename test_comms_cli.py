from __future__ import annotations

from argparse import Namespace

import pytest

from aerial_kit.comms.cli import _require_write, build_parser, main


def test_cli_has_all_protocol_command_families() -> None:
    parser = build_parser()
    radio = parser.parse_args(["--port", "/dev/null", "radio", "status"])
    fc = parser.parse_args(["--port", "/dev/null", "fc", "hello"])
    mav = parser.parse_args(["--host", "tcp://127.0.0.1:5760", "mavlink", "status"])
    assert radio.command == "radio"
    assert fc.command == "fc"
    assert mav.command == "mavlink"
    assert radio.radio_action == "status"
    assert fc.fc_action == "hello"
    assert mav.mav_action == "status"


def test_cli_writes_require_explicit_flag() -> None:
    with pytest.raises(PermissionError):
        _require_write(Namespace(write=False))
    _require_write(Namespace(write=True))


def test_cli_reports_a_missing_transport_without_opening_one() -> None:
    assert main(["fc", "hello"]) == 1


def test_radio_parameter_mutation_parses_without_side_effects() -> None:
    parser = build_parser()
    args = parser.parse_args(
        ["--port", "/dev/null", "--write", "radio", "set", "7", "12"]
    )
    assert args.field == 7
    assert args.value == 12
    assert args.write is True
