from __future__ import annotations

import json
from pathlib import Path

import pytest

from aerial_kit import doctor
from aerial_kit.comms.crsf import CrsfError


def _report(*, passed: bool = True) -> doctor.DoctorReport:
    return doctor.DoctorReport(
        environment=doctor.DoctorEnvironment(
            python="3.12.0",
            platform="Linux",
            machine="x86_64",
            dependencies={"pymavlink": False, "serial": True},
        ),
        devices=(
            doctor.SerialDevice(
                path="/dev/ttyACM0",
                aliases=("/dev/serial/by-id/example",),
                readable=True,
                writable=True,
            ),
        ),
        probe=doctor.ProbeResult(
            path="/dev/ttyACM0",
            kind="betaflight" if passed else "unknown",
            product="Betaflight 2026.6.1" if passed else None,
            firmware="2026.6.1" if passed else None,
            details={},
            attempts=(
                doctor.ProbeAttempt(
                    protocol="crsf",
                    ok=False,
                    detail="timed out",
                ),
                doctor.ProbeAttempt(
                    protocol="akp",
                    ok=False,
                    detail="timed out",
                ),
                doctor.ProbeAttempt(
                    protocol="betaflight",
                    ok=passed,
                    detail="betaflight" if passed else "timed out",
                ),
            ),
        ),
        notes=("device listing is read-only",),
    )


def test_serial_device_discovery_uses_by_id_without_opening(tmp_path: Path) -> None:
    port = tmp_path / "ttyACM0"
    port.touch()
    by_id = tmp_path / "serial" / "by-id"
    by_id.mkdir(parents=True)
    alias = by_id / "usb-Aerial_Kit_123"
    alias.symlink_to(port)

    devices = doctor.serial_devices(tmp_path)

    assert len(devices) == 1
    assert devices[0].path == str(port)
    assert devices[0].aliases == (str(alias),)
    assert devices[0].readable
    assert devices[0].writable


def test_probe_tries_read_only_protocols_in_order(monkeypatch) -> None:
    calls: list[str] = []

    def fail_radio(*_args):
        calls.append("crsf")
        raise CrsfError("not CRSF")

    def find_akp(*_args):
        calls.append("akp")
        return ("akp", "aerialkit-f405", "1", {"parameters": 42})

    def unexpected_betaflight(*_args):
        pytest.fail("probe continued after a successful identification")

    monkeypatch.setattr(doctor, "_probe_radio", fail_radio)
    monkeypatch.setattr(doctor, "_probe_akp", find_akp)
    monkeypatch.setattr(doctor, "_probe_betaflight", unexpected_betaflight)

    result = doctor.probe_path("/dev/fake", timeout=0.1)

    assert result.passed
    assert result.kind == "akp"
    assert result.product == "aerialkit-f405"
    assert calls == ["crsf", "akp"]
    assert [(attempt.protocol, attempt.ok) for attempt in result.attempts] == [
        ("crsf", False),
        ("akp", True),
    ]


def test_probe_reports_unknown_after_all_protocols_fail(monkeypatch) -> None:
    def fail(*_args):
        raise CrsfError("no reply")

    monkeypatch.setattr(doctor, "_probe_radio", fail)
    monkeypatch.setattr(doctor, "_probe_akp", fail)
    monkeypatch.setattr(doctor, "_probe_betaflight", fail)

    result = doctor.probe_path("/dev/fake", timeout=0.1)

    assert not result.passed
    assert result.kind == "unknown"
    assert len(result.attempts) == 3
    assert all(not attempt.ok for attempt in result.attempts)


def test_default_report_never_probes(monkeypatch) -> None:
    def unexpected(*_args, **_kwargs):
        pytest.fail("default doctor report attempted to open a port")

    monkeypatch.setattr(doctor, "probe_path", unexpected)
    report = doctor.build_report()
    assert report.probe is None
    assert report.passed


def test_doctor_json_stdout_is_machine_readable(
    monkeypatch,
    capsys,
) -> None:
    monkeypatch.setattr(doctor, "build_report", lambda **_kwargs: _report())
    assert doctor.main(["--json", "-"]) == 0
    output = json.loads(capsys.readouterr().out)
    assert output["passed"] is True
    assert output["devices"][0]["aliases"] == ["/dev/serial/by-id/example"]
    assert output["probe"]["kind"] == "betaflight"


def test_doctor_probe_failure_returns_nonzero(
    monkeypatch,
    capsys,
) -> None:
    monkeypatch.setattr(
        doctor,
        "build_report",
        lambda **_kwargs: _report(passed=False),
    )
    assert doctor.main(["--probe", "/dev/fake"]) == 1
    output = capsys.readouterr().out
    assert "probe=unknown" in output
    assert "attempt=crsf ok=false" in output
