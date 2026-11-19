"""Command-line access to the Aerial Kit communication clients.

The console never opens a device until a subcommand runs. Mutating commands
also require the explicit ``--write`` flag; parsing or ``--help`` has no side
effects.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from urllib.parse import urlsplit

from .akp import AkClient, AkError
from .crsf import (
    CRSF_BAUD,
    CRSF_RX_FC_BAUD,
    CrsfAkpTunnel,
    CrsfLink,
    CrsfParameter,
    CrsfParameterClient,
    CrsfError,
    SerialTransport,
    capture_link,
    drive_link,
    inject_frames,
    load_crsf_frames_file,
    monitor_link,
    radio_status,
    run_two_board_bench,
    unpack_channels,
)
from .mavlink import MavlinkError, MavlinkVehicle
from .msp import MspError
from .transport import TcpTransport, UdpTransport
from .transport import Transport


def _transport_from_args(args: argparse.Namespace) -> Transport:
    if args.port:
        return SerialTransport(
            args.port,
            baudrate=args.baud if args.baud is not None else 115_200,
            timeout=args.timeout,
        )
    if not args.host:
        raise ValueError("one of --port, --host or --radio is required")
    parsed = urlsplit(args.host if "://" in args.host else f"tcp://{args.host}")
    if not parsed.hostname or not parsed.port:
        raise ValueError("--host must include host and port, e.g. tcp://127.0.0.1:5760")
    if parsed.scheme == "udp":
        return UdpTransport.connect(
            parsed.hostname, parsed.port, timeout=args.timeout
        )
    if parsed.scheme == "tcp":
        return TcpTransport.connect(
            parsed.hostname, parsed.port, timeout=args.timeout
        )
    raise ValueError(f"unsupported transport scheme {parsed.scheme!r}")


def _radio_link(args: argparse.Namespace) -> CrsfLink:
    if args.radio:
        raise ValueError("--radio is for the fc command, not the radio command")
    if args.port and args.host:
        raise ValueError("choose only one of --port or --host")
    if args.port and args.baud is None:
        args.baud = (
            CRSF_RX_FC_BAUD
            if getattr(args, "radio_action", None) in {"capture", "inject"}
            else CRSF_BAUD
        )
    if not args.port and not args.host:
        raise ValueError("radio requires --port or --host")
    transport = _transport_from_args(args)
    return CrsfLink(transport)


def _write_json(path: str | None, value: object) -> None:
    text = json.dumps(value, indent=2, sort_keys=True) + "\n"
    if path == "-":
        sys.stdout.write(text)
    elif path:
        Path(path).write_text(text, encoding="utf-8")


def _print_monitor(report) -> None:
    print(
        f"status={report.status} duration={report.duration_sec:.3f}s "
        f"bytes={report.bytes_read} frames={report.frames} "
        f"crc_errors={report.crc_errors} resyncs={report.resyncs} "
        f"framing_errors={report.framing_errors}"
    )
    for name, count in report.frames_by_type.items():
        age = report.last_seen_seconds.get(name)
        print(f"{name:20s} {count:8d} last_seen={age:.3f}s")


def _print_capture(report) -> None:
    channels = (
        " ".join(str(value) for value in report.channels)
        if report.channels is not None
        else "none"
    )
    print(
        f"status={report.status} duration={report.duration_sec:.3f}s "
        f"frames={report.frames} rc_frames={report.rc_frames} "
        f"cadence={report.rc_cadence_hz or 0:.3f}Hz "
        f"missed={report.missed_intervals} crc_errors={report.crc_errors} "
        f"resyncs={report.resyncs} framing_errors={report.framing_errors}"
    )
    print(f"channels={channels}")


def _print_drive(report) -> None:
    print(
        f"pattern={report.pattern} sent={report.frames_sent} "
        f"rate={report.actual_hz:.3f}Hz requested={report.requested_hz:.3f}Hz "
        f"arm_hold_completed={str(report.arm_hold_completed).lower()} "
        f"disarmed_at_end={str(report.disarmed_at_end).lower()} "
        f"link_statistics={report.link_statistics_frames} "
        f"telemetry={','.join(sorted(report.telemetry)) or 'none'}"
    )
    if report.error:
        print(f"error={report.error}")


def _radio_status(args: argparse.Namespace) -> int:
    if not args.port:
        raise ValueError("radio status currently requires --port")
    baud = args.baud if args.baud is not None else CRSF_BAUD
    transport = SerialTransport(args.port, baudrate=baud, timeout=args.timeout)
    try:
        report = radio_status(
            transport,
            path=args.port,
            logical_baud=baud,
            role=args.role,
            timeout=args.timeout,
        )
    finally:
        transport.close()
    if args.json:
        _write_json(args.json, report.to_dict())
    if args.json != "-":
        print(
            f"path={report.path} usb_serial={report.usb_serial or 'unknown'} "
            f"logical_baud={report.logical_baud} role={report.role}"
        )
        print(
            f"product={report.product} origin=0x{report.origin:02x} "
            f"destination=0x{report.destination:02x} "
            f"firmware={report.firmware_string} "
            f"firmware_raw=0x{report.firmware_version:08x} "
            f"parameters={report.parameter_count} "
            f"parameter_version={report.parameter_version}"
        )
    return 0


def _radio_bench(args: argparse.Namespace) -> int:
    _require_write(args)
    if not args.tx or not args.rx:
        raise ValueError("radio bench requires --tx and --rx")
    tx_transport = SerialTransport(args.tx, baudrate=CRSF_BAUD, timeout=0.05)
    rx_transport = SerialTransport(args.rx, baudrate=CRSF_RX_FC_BAUD, timeout=0.05)
    try:
        report = run_two_board_bench(
            tx_transport,
            rx_transport,
            tx_path=args.tx,
            rx_path=args.rx,
            duration=args.duration,
            hz=args.hz,
        )
    finally:
        tx_transport.close()
        rx_transport.close()
    if args.json:
        _write_json(args.json, report.to_dict())
    if args.json != "-":
        for name, status in report.requirements.items():
            print(f"{name}={status}")
        print(f"overall={report.overall}")
    return 0 if report.overall == "PASS" else 1


def _require_write(args: argparse.Namespace) -> None:
    if not args.write:
        raise PermissionError("this command changes a device; pass --write explicitly")


def _print_parameter(parameter: CrsfParameter) -> None:
    value = "" if parameter.value is None else parameter.value
    print(
        f"{parameter.id:3d} {parameter.name:24s} {str(value):16s} "
        f"{parameter.type}"
    )


def _radio(args: argparse.Namespace) -> int:
    if args.radio_action == "set" and (
        args.field is None or args.value is None
    ):
        raise ValueError("radio set requires FIELD and VALUE")
    if args.radio_action in {"set", "bind", "drive", "inject", "bench"}:
        _require_write(args)
    if args.radio_action == "bench":
        return _radio_bench(args)
    if args.radio_action == "status":
        return _radio_status(args)
    link = _radio_link(args)
    transport = link.transport
    try:
        client = CrsfParameterClient(link, timeout=args.timeout)
        if args.radio_action == "params":
            for parameter in client.read_parameter_tree().values():
                _print_parameter(parameter)
        elif args.radio_action == "set":
            result = client.write_parameter(args.field, args.value, allow_write=True)
            if result is None:
                print("parameter write sent")
            else:
                _print_parameter(result)
        elif args.radio_action == "bind":
            client.bind(allow_write=True)
            print("bind command sent")
        elif args.radio_action == "channels":
            frames = link.poll(512, timeout=args.timeout)
            for frame in frames:
                if frame.frame_type == 0x16 and len(frame.payload) == 22:
                    print(" ".join(map(str, unpack_channels(frame.payload))))
                    return
            raise TimeoutError("no RC_CHANNELS_PACKED frame arrived")
        elif args.radio_action == "monitor":
            report = monitor_link(transport, duration=args.duration)
            if args.json:
                _write_json(args.json, report.to_dict())
            if args.json != "-":
                _print_monitor(report)
            return 0 if report.passed else 1
        elif args.radio_action == "drive":
            report = drive_link(
                transport,
                hz=args.hz,
                duration=args.duration,
                pattern=args.pattern,
                arm_hold_sec=args.arm_hold,
                allow_write=True,
            )
            if args.json:
                _write_json(args.json, report.to_dict())
            if args.json != "-":
                _print_drive(report)
            return 0 if report.passed else 1
        elif args.radio_action == "capture":
            report = capture_link(transport, duration=args.duration)
            if args.json:
                _write_json(args.json, report.to_dict())
            if args.json != "-":
                _print_capture(report)
            return 0 if report.passed else 1
        elif args.radio_action == "inject":
            if not args.file:
                raise ValueError("radio inject requires --file")
            frames = load_crsf_frames_file(args.file)
            report = inject_frames(transport, frames, allow_write=True)
            if args.json:
                _write_json(args.json, report.to_dict())
            if args.json != "-":
                print(
                    f"injected={report.frames} duration={report.duration_sec:.3f}s "
                    f"types={','.join(f'0x{value:02x}' for value in report.frame_types)}"
                )
            return 0 if report.passed else 1
    finally:
        link.close()
    return 0


def _akp_client(args: argparse.Namespace) -> AkClient:
    if args.radio:
        link = CrsfLink(
            SerialTransport(args.radio, baudrate=CRSF_BAUD, timeout=args.timeout)
        )
        return AkClient(CrsfAkpTunnel(link), timeout=args.timeout)
    return AkClient(_transport_from_args(args), timeout=args.timeout)


def _fc(args: argparse.Namespace) -> None:
    if args.fc_action == "get" and args.index is None:
        raise ValueError("fc get requires --index")
    if args.fc_action == "set" and (
        args.index is None or args.value is None
    ):
        raise ValueError("fc set requires --index and --value")
    client = _akp_client(args)
    try:
        if args.fc_action == "hello":
            hello = client.hello()
            print(
                f"{hello.product} protocol={hello.version} "
                f"parameters={hello.parameter_count} changed={hello.changed}"
            )
        elif args.fc_action == "params":
            for item in client.parameter_info():
                print(
                    f"{item.index:3d} {item.name or '':24s} "
                    f"{item.type or ''} {item.group_name or ''}"
                )
        elif args.fc_action == "get":
            value = client.get_parameter(args.index)
            print(f"{value.name}={value.value} status={value.status}")
        elif args.fc_action == "set":
            _require_write(args)
            result = client.set_parameter(
                args.index, str(args.value), allow_write=True
            )
            print(f"status={result.status} {result.message}")
        elif args.fc_action == "save":
            _require_write(args)
            result = client.save_parameters(allow_write=True)
            print(f"status={result.status} {result.message}")
        elif args.fc_action == "telemetry":
            rate = client.telemetry_subscribe(args.hz)
            print(f"telemetry_rate={rate}")
            if rate:
                telemetry = client.telemetry_next()
                print(
                    f"uptime_ms={telemetry.uptime_ms} "
                    f"flight_state={telemetry.status.flight_state} "
                    f"link_live={telemetry.status.link_live}"
                )
        elif args.fc_action == "logs":
            print(f"records={client.log_info().count}")
            if args.index is not None:
                print(client.log_record(args.index))
    finally:
        client.close()


def _mavlink(args: argparse.Namespace) -> None:
    vehicle = MavlinkVehicle(_transport_from_args(args), timeout=args.timeout)
    try:
        # A vehicle streams once it has heard a ground station heartbeat.
        vehicle.send_heartbeat()
        if args.mav_action == "status":
            identity = vehicle.identify()
            print(identity)
        elif args.mav_action == "params":
            for parameter in vehicle.parameters():
                print(
                    f"{parameter.index:3d} {parameter.name:24s} "
                    f"{parameter.value}"
                )
        elif args.mav_action == "mission":
            for item in vehicle.mission_download():
                print(item)
    finally:
        vehicle.close()


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(prog="aerial-kit-link")
    transport = parser.add_mutually_exclusive_group()
    transport.add_argument("--port", help="serial port")
    transport.add_argument("--host", help="tcp://host:port or udp://host:port")
    transport.add_argument(
        "--radio",
        metavar="PORT",
        help="serial port connected to an ELRS transmitter module",
    )
    parser.add_argument("--baud", type=int, default=None)
    parser.add_argument("--timeout", type=float, default=2.0)
    parser.add_argument("--write", action="store_true", help="allow a device mutation")

    commands = parser.add_subparsers(dest="command", required=True)
    radio = commands.add_parser("radio", help="query an ELRS/CRSF radio")
    radio.add_argument(
        "radio_action",
        choices=(
            "status",
            "params",
            "set",
            "bind",
            "channels",
            "monitor",
            "drive",
            "capture",
            "inject",
            "bench",
        ),
    )
    radio.add_argument("field", nargs="?", type=int)
    radio.add_argument("value", nargs="?", type=int)
    radio.add_argument(
        "--write", action="store_true", default=argparse.SUPPRESS,
        help="allow a device mutation",
    )
    radio.add_argument("--port", default=argparse.SUPPRESS)
    radio.add_argument("--baud", type=int, default=argparse.SUPPRESS)
    radio.add_argument("--duration", type=float, default=10.0)
    radio.add_argument("--hz", type=float, default=150.0)
    radio.add_argument(
        "--pattern",
        choices=("disarmed", "arm-test", "channel-test"),
        default="disarmed",
    )
    radio.add_argument("--arm-hold", type=float, default=0.75)
    radio.add_argument("--role", choices=("auto", "tx", "rx"), default="auto")
    radio.add_argument("--file")
    radio.add_argument("--tx", metavar="PORT")
    radio.add_argument("--rx", metavar="PORT")
    radio.add_argument("--json")
    radio.set_defaults(handler=_radio)

    fc = commands.add_parser("fc", help="query an Aerial Kit flight controller")
    fc.add_argument(
        "fc_action",
        choices=("hello", "params", "get", "set", "save", "telemetry", "logs"),
    )
    fc.add_argument("--index", type=int)
    fc.add_argument("--value", type=float)
    fc.add_argument("--hz", type=int, default=0)
    fc.add_argument(
        "--write", action="store_true", default=argparse.SUPPRESS,
        help="allow a device mutation",
    )
    fc.set_defaults(handler=_fc)

    mav = commands.add_parser("mavlink", help="query a PX4 or ArduPilot vehicle")
    mav.add_argument("mav_action", choices=("status", "params", "mission"))
    mav.set_defaults(handler=_mavlink)
    return parser


def main(argv: list[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    try:
        result = args.handler(args)
    except KeyboardInterrupt:
        print("aerial-kit-link: interrupted; outputs were left disarmed", file=sys.stderr)
        return 130
    except (
        OSError,
        TimeoutError,
        ValueError,
        PermissionError,
        AkError,
        CrsfError,
        MavlinkError,
        MspError,
    ) as exc:
        print(f"aerial-kit-link: {exc}", file=sys.stderr)
        return 1
    return 0 if result is None else int(result)


if __name__ == "__main__":
    raise SystemExit(main())
