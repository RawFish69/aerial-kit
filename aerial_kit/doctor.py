"""Read-only environment and serial-device diagnostics for ``ak doctor``."""

from __future__ import annotations

import argparse
from collections.abc import Callable
from dataclasses import asdict, dataclass, field
import glob
import importlib.util
import json
import os
from pathlib import Path
import platform
import sys

from .comms.akp import AkClient, AkError
from .comms.crsf import CRSF_BAUD, CrsfError, radio_status
from .comms.msp import Error as MspError, MspClient, describe as describe_msp, identify
from .comms.transport import SerialTransport, Transport

TransportFactory = Callable[[str, int, float], Transport]


@dataclass(frozen=True)
class SerialDevice:
    """One serial endpoint, including its stable `/dev/serial/by-id` aliases."""

    path: str
    aliases: tuple[str, ...]
    readable: bool
    writable: bool

    def to_dict(self) -> dict[str, object]:
        return asdict(self)


@dataclass(frozen=True)
class DoctorEnvironment:
    python: str
    platform: str
    machine: str
    dependencies: dict[str, bool]

    def to_dict(self) -> dict[str, object]:
        return asdict(self)


@dataclass(frozen=True)
class ProbeAttempt:
    protocol: str
    ok: bool
    detail: str

    def to_dict(self) -> dict[str, object]:
        return asdict(self)


@dataclass(frozen=True)
class ProbeResult:
    path: str
    kind: str
    product: str | None
    firmware: str | None
    details: dict[str, object]
    attempts: tuple[ProbeAttempt, ...]

    @property
    def passed(self) -> bool:
        return self.kind != "unknown"

    def to_dict(self) -> dict[str, object]:
        return {
            "path": self.path,
            "kind": self.kind,
            "product": self.product,
            "firmware": self.firmware,
            "details": self.details,
            "attempts": [attempt.to_dict() for attempt in self.attempts],
            "passed": self.passed,
        }


@dataclass(frozen=True)
class DoctorReport:
    environment: DoctorEnvironment
    devices: tuple[SerialDevice, ...]
    probe: ProbeResult | None = None
    notes: tuple[str, ...] = field(default_factory=tuple)

    @property
    def passed(self) -> bool:
        return self.probe is None or self.probe.passed

    def to_dict(self) -> dict[str, object]:
        return {
            "passed": self.passed,
            "environment": self.environment.to_dict(),
            "devices": [device.to_dict() for device in self.devices],
            "probe": None if self.probe is None else self.probe.to_dict(),
            "notes": list(self.notes),
        }


def environment() -> DoctorEnvironment:
    """Report runtime details and optional protocol dependencies without importing them."""
    return DoctorEnvironment(
        python=platform.python_version(),
        platform=platform.system(),
        machine=platform.machine(),
        dependencies={
            "serial": importlib.util.find_spec("serial") is not None,
            "pymavlink": importlib.util.find_spec("pymavlink") is not None,
        },
    )


def serial_devices(dev_root: Path = Path("/dev")) -> tuple[SerialDevice, ...]:
    """List serial ports and by-id aliases without opening any device."""
    aliases: dict[Path, set[str]] = {}
    by_id = dev_root / "serial" / "by-id"
    if by_id.is_dir():
        for alias in sorted(by_id.iterdir()):
            try:
                target = alias.resolve(strict=False)
            except OSError:
                continue
            aliases.setdefault(target, set()).add(str(alias))

    candidates: set[Path] = set(aliases)
    for pattern in ("ttyACM*", "ttyUSB*"):
        candidates.update(Path(path) for path in glob.glob(str(dev_root / pattern)))

    devices = []
    for path in sorted(candidates, key=str):
        devices.append(
            SerialDevice(
                path=str(path),
                aliases=tuple(sorted(aliases.get(path, set()))),
                readable=os.access(path, os.R_OK),
                writable=os.access(path, os.W_OK),
            )
        )
    return tuple(devices)


def _factory(
    path: str,
    baud: int,
    timeout: float,
) -> Transport:
    return SerialTransport(path, baudrate=baud, timeout=timeout)


def _probe_radio(
    path: str,
    timeout: float,
    factory: TransportFactory,
) -> tuple[str, str | None, str | None, dict[str, object]]:
    transport = factory(path, CRSF_BAUD, timeout)
    try:
        report = radio_status(
            transport,
            path=path,
            logical_baud=CRSF_BAUD,
            role="auto",
            timeout=timeout,
        )
    finally:
        transport.close()
    return (
        f"radio-{report.role}",
        report.product,
        report.firmware_string,
        {
            "origin": report.origin,
            "destination": report.destination,
            "parameter_count": report.parameter_count,
        },
    )


def _probe_akp(
    path: str,
    timeout: float,
    factory: TransportFactory,
) -> tuple[str, str | None, str | None, dict[str, object]]:
    transport = factory(path, 115_200, timeout)
    try:
        hello = AkClient(transport, timeout=timeout).hello()
    finally:
        transport.close()
    return (
        "akp",
        hello.product,
        str(hello.version),
        {"parameters": hello.parameter_count, "features": hello.feature_names},
    )


def _probe_betaflight(
    path: str,
    timeout: float,
    factory: TransportFactory,
) -> tuple[str, str | None, str | None, dict[str, object]]:
    transport = factory(path, 115_200, timeout)
    try:
        client = MspClient(transport.read, transport.write, timeout=timeout)
        identity = identify(client)
    finally:
        transport.close()
    return (
        "betaflight",
        describe_msp(identity),
        identity["version"]["text"],
        {"variant": identity["variant"], "api": identity["api"]},
    )


def probe_path(
    path: str,
    *,
    timeout: float = 1.0,
    factory: TransportFactory | None = None,
) -> ProbeResult:
    """Try read-only radio, AKP and Betaflight identification in that order.

    Each attempt receives a fresh transport because a failed parser may have
    consumed bytes. The caller must explicitly request a probe; listing devices
    never opens a port.
    """
    if timeout <= 0:
        raise ValueError("probe timeout must be positive")
    make_transport = factory or _factory
    attempts: list[ProbeAttempt] = []
    probes: tuple[
        tuple[str, Callable[[str, float, TransportFactory], tuple[str, str | None, str | None, dict[str, object]]]],
        ...,
    ] = (
        ("crsf", _probe_radio),
        ("akp", _probe_akp),
        ("betaflight", _probe_betaflight),
    )
    for protocol, probe in probes:
        try:
            kind, product, firmware, details = probe(path, timeout, make_transport)
        except (AkError, CrsfError, MspError, OSError, TimeoutError, ValueError) as exc:
            attempts.append(
                ProbeAttempt(
                    protocol=protocol,
                    ok=False,
                    detail=f"{type(exc).__name__}: {exc}",
                )
            )
            continue
        attempts.append(
            ProbeAttempt(protocol=protocol, ok=True, detail=kind)
        )
        return ProbeResult(
            path=path,
            kind=kind,
            product=product,
            firmware=firmware,
            details=details,
            attempts=tuple(attempts),
        )
    return ProbeResult(
        path=path,
        kind="unknown",
        product=None,
        firmware=None,
        details={},
        attempts=tuple(attempts),
    )


def build_report(
    *,
    dev_root: Path = Path("/dev"),
    probe: str | None = None,
    timeout: float = 1.0,
    factory: TransportFactory | None = None,
) -> DoctorReport:
    devices = serial_devices(dev_root)
    result = (
        probe_path(probe, timeout=timeout, factory=factory)
        if probe is not None
        else None
    )
    notes = [
        "device listing is read-only; ports are opened only by --probe",
    ]
    if not importlib.util.find_spec("serial"):
        notes.append(
            'pyserial is not installed; install with: pip install "aerial-kit[radio]"'
        )
    return DoctorReport(environment=environment(), devices=devices, probe=result, notes=tuple(notes))


def _write_json(path: str, report: DoctorReport) -> None:
    text = json.dumps(report.to_dict(), indent=2, sort_keys=True) + "\n"
    if path == "-":
        sys.stdout.write(text)
    else:
        Path(path).write_text(text, encoding="utf-8")


def _print_report(report: DoctorReport) -> None:
    env = report.environment
    deps = " ".join(
        f"{name}={'yes' if present else 'no'}"
        for name, present in sorted(env.dependencies.items())
    )
    print(f"python={env.python} platform={env.platform} machine={env.machine}")
    print(f"dependencies {deps}")
    print(f"serial_devices={len(report.devices)}")
    for device in report.devices:
        permissions = (
            f"read={'yes' if device.readable else 'no'} "
            f"write={'yes' if device.writable else 'no'}"
        )
        print(f"device={device.path} {permissions}")
        for alias in device.aliases:
            print(f"  by_id={alias}")
    if report.probe is not None:
        print(
            f"probe={report.probe.kind} product={report.probe.product or 'unknown'} "
            f"firmware={report.probe.firmware or 'unknown'}"
        )
        for attempt in report.probe.attempts:
            print(
                f"  attempt={attempt.protocol} ok={str(attempt.ok).lower()} "
                f"detail={attempt.detail}"
            )
    for note in report.notes:
        print(f"note={note}")


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(prog="ak doctor")
    parser.add_argument(
        "--probe",
        metavar="PORT",
        help="open PORT and try read-only CRSF, AKP and Betaflight identification",
    )
    parser.add_argument("--timeout", type=float, default=1.0)
    parser.add_argument("--json", metavar="PATH")
    args = parser.parse_args(argv)
    report = build_report(probe=args.probe, timeout=args.timeout)
    if args.json:
        _write_json(args.json, report)
    if args.json != "-":
        _print_report(report)
    return 0 if report.passed else 1


if __name__ == "__main__":
    raise SystemExit(main())


__all__ = [
    "DoctorEnvironment",
    "DoctorReport",
    "ProbeAttempt",
    "ProbeResult",
    "SerialDevice",
    "TransportFactory",
    "build_report",
    "environment",
    "main",
    "probe_path",
    "serial_devices",
]
