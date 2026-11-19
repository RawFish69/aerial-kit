"""Command-line access to the Aerial Kit communication clients.

The console never opens a device until a subcommand runs. Mutating commands
also require the explicit ``--write`` flag; parsing or ``--help`` has no side
effects.
"""

from __future__ import annotations

import argparse
import sys
from urllib.parse import urlsplit

from .akp import AkClient
from .crsf import (
    CRSF_BAUD,
    CrsfAkpTunnel,
    CrsfLink,
    CrsfParameter,
    CrsfParameterClient,
    SerialTransport,
    unpack_channels,
)
from .mavlink import MavlinkVehicle
from .transport import TcpTransport, UdpTransport
from .transport import Transport


def _transport_from_args(args: argparse.Namespace) -> Transport:
    if args.port:
        return SerialTransport(
            args.port,
            baudrate=args.baud,
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
    transport = _transport_from_args(args)
    return CrsfLink(transport)


def _require_write(args: argparse.Namespace) -> None:
    if not args.write:
        raise PermissionError("this command changes a device; pass --write explicitly")


def _print_parameter(parameter: CrsfParameter) -> None:
    value = "" if parameter.value is None else parameter.value
    print(
        f"{parameter.id:3d} {parameter.name:24s} {str(value):16s} "
        f"{parameter.type}"
    )


def _radio(args: argparse.Namespace) -> None:
    link = _radio_link(args)
    try:
        client = CrsfParameterClient(link, timeout=args.timeout)
        if args.radio_action == "status":
            info = client.ping()
            print(
                f"{info.name} origin=0x{info.origin:02x} "
                f"firmware=0x{info.software_version:08x} "
                f"parameters={info.field_count}"
            )
        elif args.radio_action == "params":
            for parameter in client.read_parameter_tree().values():
                _print_parameter(parameter)
        elif args.radio_action == "set":
            _require_write(args)
            result = client.write_parameter(args.field, args.value, allow_write=True)
            if result is None:
                print("parameter write sent")
            else:
                _print_parameter(result)
        elif args.radio_action == "bind":
            _require_write(args)
            client.bind(allow_write=True)
            print("bind command sent")
        elif args.radio_action == "channels":
            frames = link.poll(512, timeout=args.timeout)
            for frame in frames:
                if frame.frame_type == 0x16 and len(frame.payload) == 22:
                    print(" ".join(map(str, unpack_channels(frame.payload))))
                    return
            raise TimeoutError("no RC_CHANNELS_PACKED frame arrived")
    finally:
        link.close()


def _akp_client(args: argparse.Namespace) -> AkClient:
    if args.radio:
        link = CrsfLink(
            SerialTransport(args.radio, baudrate=CRSF_BAUD, timeout=args.timeout)
        )
        return AkClient(CrsfAkpTunnel(link), timeout=args.timeout)
    return AkClient(_transport_from_args(args), timeout=args.timeout)


def _fc(args: argparse.Namespace) -> None:
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
                args.index, args.value, allow_write=True
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
    parser.add_argument("--port", help="serial port")
    parser.add_argument("--host", help="tcp://host:port or udp://host:port")
    parser.add_argument(
        "--radio",
        metavar="PORT",
        help="serial port connected to an ELRS transmitter module",
    )
    parser.add_argument("--baud", type=int, default=115_200)
    parser.add_argument("--timeout", type=float, default=2.0)
    parser.add_argument("--write", action="store_true", help="allow a device mutation")

    commands = parser.add_subparsers(dest="command", required=True)
    radio = commands.add_parser("radio", help="query an ELRS/CRSF radio")
    radio.add_argument(
        "radio_action", choices=("status", "params", "set", "bind", "channels")
    )
    radio.add_argument("field", nargs="?", type=int)
    radio.add_argument("value", nargs="?", type=int)
    radio.set_defaults(handler=_radio)

    fc = commands.add_parser("fc", help="query an Aerial Kit flight controller")
    fc.add_argument(
        "fc_action",
        choices=("hello", "params", "get", "set", "save", "telemetry", "logs"),
    )
    fc.add_argument("--index", type=int)
    fc.add_argument("--value", type=float)
    fc.add_argument("--hz", type=int, default=0)
    fc.set_defaults(handler=_fc)

    mav = commands.add_parser("mavlink", help="query a PX4 or ArduPilot vehicle")
    mav.add_argument("mav_action", choices=("status", "params", "mission"))
    mav.set_defaults(handler=_mavlink)
    return parser


def main(argv: list[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    try:
        args.handler(args)
    except (OSError, TimeoutError, ValueError, PermissionError) as exc:
        print(f"aerial-kit-link: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
