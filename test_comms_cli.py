from __future__ import annotations

from argparse import Namespace

import pytest

from aerial_kit.comms.cli import _radio_link, _require_write, build_parser, main
from aerial_kit.comms.akp import AkError
from aerial_kit.comms import msp
from aerial_kit.comms.crsf import CRSF_BAUD


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


def test_transport_flags_are_mutually_exclusive() -> None:
    parser = build_parser()
    with pytest.raises(SystemExit):
        parser.parse_args(
            ["--port", "/dev/a", "--host", "tcp://127.0.0.1:5760", "fc", "hello"]
        )


def test_radio_link_defaults_to_crsf_baud(monkeypatch) -> None:
    captured = {}

    class FakeSerial:
        def __init__(self, port, *, baudrate, timeout):
            self.port = port
            captured.update(port=port, baudrate=baudrate, timeout=timeout)

    monkeypatch.setattr("aerial_kit.comms.cli.SerialTransport", FakeSerial)
    args = Namespace(
        radio=None,
        port="/dev/radio",
        host=None,
        baud=None,
        timeout=2.0,
    )
    link = _radio_link(args)
    assert captured["baudrate"] == CRSF_BAUD
    assert link.transport.port == "/dev/radio"


def test_missing_write_arguments_fail_before_opening_a_transport(monkeypatch) -> None:
    def fail(_args):
        pytest.fail("transport was opened before argument validation")

    monkeypatch.setattr("aerial_kit.comms.cli._akp_client", fail)
    assert main(["--port", "/dev/fake", "fc", "get"]) == 1
    assert main(["--port", "/dev/fake", "fc", "set", "--index", "1"]) == 1


def test_write_flag_is_accepted_after_the_subcommand() -> None:
    parser = build_parser()
    args = parser.parse_args(
        ["--port", "/dev/null", "fc", "set", "--index", "1", "--value", "2", "--write"]
    )
    assert args.write is True


def test_protocol_failures_are_printed_without_a_traceback(monkeypatch, capsys) -> None:
    def fail(_args):
        raise AkError("board refused the query")

    monkeypatch.setattr("aerial_kit.comms.cli._akp_client", fail)
    assert main(["--port", "/dev/fake", "fc", "hello"]) == 1
    captured = capsys.readouterr()
    assert "board refused the query" in captured.err
    assert "Traceback" not in captured.err


def test_write_capable_msp_client_is_private() -> None:
    assert hasattr(msp, "Msp")
    assert not hasattr(msp, "MspClient")
