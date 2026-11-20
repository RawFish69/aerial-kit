from __future__ import annotations

from argparse import Namespace
import json
from types import SimpleNamespace

import pytest

from aerial_kit.comms.cli import _radio_link, _require_write, build_parser, main
from aerial_kit.comms.akp import AkError
from aerial_kit.comms import msp
from aerial_kit.comms.crsf import CRSF_BAUD
from aerial_kit.comms.crsf import CrsfTimeout


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
    assert args.field == "7"
    assert args.value == "12"
    assert args.write is True


def test_radio_uid_and_identity_commands_parse() -> None:
    parser = build_parser()
    uid = parser.parse_args(["--port", "/dev/null", "radio", "uid", "--role", "rx"])
    set_uid = parser.parse_args([
        "--port", "/dev/null", "radio", "set-uid", "aa:bb:cc:dd:ee:ff",
        "--write",
    ])
    phrase = parser.parse_args([
        "--port", "/dev/null", "radio", "bind-phrase", "test", "--write",
    ])
    get = parser.parse_args(["--port", "/dev/null", "radio", "get", "7"])
    pair = parser.parse_args([
        "radio", "pair", "--tx", "/dev/a", "--rx", "/dev/b",
        "--phrase", "test", "--write",
    ])
    assert uid.radio_action == "uid"
    assert set_uid.field == "aa:bb:cc:dd:ee:ff"
    assert phrase.field == "test"
    assert get.field == "7"
    assert pair.radio_action == "pair"


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


def test_radio_bench_parser_supports_documented_forms() -> None:
    parser = build_parser()
    monitor = parser.parse_args(
        [
            "radio",
            "monitor",
            "--port",
            "/dev/ttyACM0",
            "--baud",
            "400000",
            "--duration",
            "5",
            "--json",
            "out.json",
        ]
    )
    assert monitor.radio_action == "monitor"
    assert monitor.port == "/dev/ttyACM0"
    assert monitor.duration == 5

    drive = parser.parse_args(
        [
            "radio",
            "drive",
            "--port",
            "/dev/ttyACM0",
            "--hz",
            "150",
            "--duration",
            "10",
            "--pattern",
            "arm-test",
            "--write",
        ]
    )
    assert drive.pattern == "arm-test"
    assert drive.write is True

    bench = parser.parse_args(
        [
            "radio",
            "bench",
            "--tx",
            "/dev/ttyACM0",
            "--rx",
            "/dev/ttyACM1",
            "--write",
        ]
    )
    assert (bench.tx, bench.rx) == ("/dev/ttyACM0", "/dev/ttyACM1")


def test_radio_mutation_is_refused_before_opening_a_port(monkeypatch) -> None:
    def fail(_args):
        pytest.fail("transport was opened before write authorization")

    monkeypatch.setattr("aerial_kit.comms.cli._radio_link", fail)
    assert main(["radio", "drive", "--port", "/dev/fake"]) == 1
    assert main(["radio", "inject", "--port", "/dev/fake", "--file", "x"]) == 1
    assert main(
        ["radio", "bench", "--tx", "/dev/a", "--rx", "/dev/b"]
    ) == 1
    assert main(
        [
            "radio", "set-uid", "aa:bb:cc:dd:ee:ff",
            "--port", "/dev/fake",
        ]
    ) == 1
    assert main(
        [
            "radio", "bind-phrase", "test",
            "--port", "/dev/fake",
        ]
    ) == 1


def test_radio_identity_write_requires_readback_after_reconnect(
    monkeypatch: pytest.MonkeyPatch,
    capsys: pytest.CaptureFixture[str],
) -> None:
    class FakeLink:
        def close(self) -> None:
            pass

    written: list[bytes] = []

    class FakeClient:
        destination = 0xEE

        def write_uid(self, value: bytes, *, allow_write: bool) -> bytes:
            assert allow_write
            written.append(value)
            return value

    read_client = SimpleNamespace(destination=0xF2)
    expected = bytes.fromhex("aabbccddeeff")
    responses = iter([
        (read_client, bytes(6)),
        (read_client, expected),
    ])
    monkeypatch.setattr("aerial_kit.comms.cli._radio_link", lambda _args: FakeLink())
    monkeypatch.setattr(
        "aerial_kit.comms.cli._read_uid_for_link",
        lambda *_args, **_kwargs: next(responses),
    )
    args = Namespace(
        port="/dev/fake",
        host=None,
        radio=None,
        baud=None,
        timeout=0.1,
        role="tx",
        write=True,
        confirm=True,
        uid="aa:bb:cc:dd:ee:ff",
        phrase=None,
        reboot_wait=0.0,
    )

    class Client(FakeClient):
        pass

    responses = iter([(Client(), bytes(6)), (SimpleNamespace(destination=0xEE), expected)])
    assert main([
        "radio", "set-uid", "aa:bb:cc:dd:ee:ff",
        "--port", "/dev/fake", "--write", "--reboot-wait", "0",
    ]) == 0
    assert written == [expected]
    output = capsys.readouterr().out
    assert "write_sent=true" in output
    assert f"verified role=tx uid={expected.hex(':')}" in output


def test_radio_pair_verifies_both_readbacks_before_the_bench(
    monkeypatch: pytest.MonkeyPatch,
    capsys: pytest.CaptureFixture[str],
) -> None:
    expected = bytes.fromhex("4f04fd822155")
    writes: list[tuple[str, str]] = []
    bench_started: list[bool] = []

    def fake_write(path, *, role, phrase, timeout, reboot_wait):
        writes.append((path, role))
        return bytes(6), expected

    class FakeTransport:
        def close(self) -> None:
            pass

    def fake_bench(*_args, **_kwargs):
        bench_started.append(True)
        return SimpleNamespace(
            overall="PASS",
            requirements={
                "rx_rc_capture": "PASS",
                "link_statistics": "PASS",
            },
            to_dict=lambda: {
                "overall": "PASS",
                "requirements": {
                    "rx_rc_capture": "PASS",
                    "link_statistics": "PASS",
                },
            },
        )

    monkeypatch.setattr("aerial_kit.comms.cli._pair_write_phrase", fake_write)
    monkeypatch.setattr("aerial_kit.comms.cli.SerialTransport", lambda *_a, **_k: FakeTransport())
    monkeypatch.setattr("aerial_kit.comms.cli.run_two_board_bench", fake_bench)
    assert main([
        "radio", "pair", "--tx", "/dev/tx", "--rx", "/dev/rx",
        "--phrase", "test", "--duration", "1", "--write",
    ]) == 0
    assert writes == [("/dev/tx", "tx"), ("/dev/rx", "rx")]
    assert bench_started == [True]
    captured = capsys.readouterr().out
    assert "uid_binding=verified uid=4f:04:fd:82:21:55" in captured
    assert "overall=PASS" in captured


def test_radio_status_reports_device_info_and_no_answer_fails(
    monkeypatch: pytest.MonkeyPatch,
    capsys: pytest.CaptureFixture[str],
) -> None:
    class FakeSerial:
        def __init__(self, port: str, *, baudrate: int, timeout: float) -> None:
            self.port = port
            self.baudrate = baudrate
            self.timeout = timeout
            self.usb_serial_number = "SER123"

        def close(self) -> None:
            pass

    monkeypatch.setattr("aerial_kit.comms.cli.SerialTransport", FakeSerial)
    monkeypatch.setattr(
        "aerial_kit.comms.cli.radio_status",
        lambda *args, **kwargs: SimpleNamespace(
            path="/dev/radio",
            usb_serial="SER123",
            logical_baud=400_000,
            role="tx",
            product="Aerial Kit Rev A3",
            origin=0xEE,
            destination=0xEA,
            firmware_string="4.1.0",
            firmware_version=0x00040100,
            parameter_count=12,
            parameter_version=0,
            to_dict=lambda: {"product": "Aerial Kit Rev A3"},
        ),
    )
    assert main(["radio", "status", "--port", "/dev/radio", "--baud", "400000"]) == 0
    captured = capsys.readouterr()
    assert "usb_serial=SER123" in captured.out
    assert "product=Aerial Kit Rev A3" in captured.out

    monkeypatch.setattr(
        "aerial_kit.comms.cli.radio_status",
        lambda *args, **kwargs: (_ for _ in ()).throw(CrsfTimeout("silent")),
    )
    assert main(["radio", "status", "--port", "/dev/radio"]) == 1


def test_bench_actions_write_stable_json(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path,
) -> None:
    class FakeTransport:
        def close(self) -> None:
            pass

    fake_link = SimpleNamespace(transport=FakeTransport(), close=lambda: None)
    monkeypatch.setattr("aerial_kit.comms.cli._radio_link", lambda _args: fake_link)
    monkeypatch.setattr(
        "aerial_kit.comms.cli.monitor_link",
        lambda *_args, **_kwargs: SimpleNamespace(
            status="valid_crsf",
            duration_sec=1.0,
            bytes_read=26,
            frames=1,
            crc_errors=0,
            resyncs=0,
            framing_errors=0,
            frames_by_type={"rc_channels": 1},
            last_seen_seconds={"rc_channels": 0.0},
            passed=True,
            to_dict=lambda: {"status": "valid_crsf", "frames": 1},
        ),
    )
    output = tmp_path / "monitor.json"
    assert main(
        [
            "radio",
            "monitor",
            "--port",
            "/dev/radio",
            "--duration",
            "1",
            "--json",
            str(output),
        ]
    ) == 0
    assert json.loads(output.read_text()) == {"status": "valid_crsf", "frames": 1}


def test_no_capture_frames_is_failure(monkeypatch) -> None:
    class FakeTransport:
        def close(self) -> None:
            pass

    fake_link = SimpleNamespace(transport=FakeTransport(), close=lambda: None)
    monkeypatch.setattr("aerial_kit.comms.cli._radio_link", lambda _args: fake_link)
    monkeypatch.setattr(
        "aerial_kit.comms.cli.capture_link",
        lambda *_args, **_kwargs: SimpleNamespace(
            passed=False,
            status="no_bytes",
            duration_sec=1.0,
            frames=0,
            rc_frames=0,
            rc_cadence_hz=None,
            missed_intervals=0,
            crc_errors=0,
            resyncs=0,
            framing_errors=0,
            channels=None,
            to_dict=lambda: {"status": "no_bytes"},
        ),
    )
    assert main(["radio", "capture", "--port", "/dev/radio", "--duration", "1"]) == 1


def test_write_capable_msp_client_is_private() -> None:
    assert hasattr(msp, "Msp")
    assert not hasattr(msp, "MspClient")
