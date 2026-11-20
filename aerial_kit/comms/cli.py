"""Command-line access to the Aerial Kit communication clients.

The console never opens a device until a subcommand runs. Mutating commands
also require the explicit ``--write`` flag; parsing or ``--help`` has no side
effects.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from datetime import datetime, timezone
import json
import re
import sys
import time
from pathlib import Path
from typing import cast
from urllib.parse import urlsplit

from .akp import AkClient, AkError, AkHello
from .crsf import (
    CRSF_BAUD,
    CRSF_ADDRESS_BROADCAST,
    CRSF_ADDRESS_CRSF_TRANSMITTER,
    CRSF_ADDRESS_ELRS_LUA,
    CRSF_ADDRESS_RADIO_TRANSMITTER,
    CRSF_ADDRESS_RECEIVER,
    CRSF_RX_FC_BAUD,
    CrsfAkpTunnel,
    CaptureReport,
    CrsfChannelMap,
    DriveReport,
    CrsfLink,
    CrsfParameter,
    CrsfParameterClient,
    CrsfError,
    CrsfProtocolError,
    MonitorReport,
    RcSticks,
    SerialTransport,
    capture_link,
    drive_link,
    inject_frames,
    load_crsf_frames_file,
    monitor_link,
    radio_status,
    run_two_board_bench,
    binding_uid,
    format_uid,
    parse_uid,
    unpack_channels,
)
from .mavlink import MavlinkError, MavlinkVehicle
from .msp import (
    MspClient,
    MspError,
    identify as identify_msp,
    describe as describe_msp,
    state as msp_state,
)
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
            if (
                getattr(args, "radio_action", None) in {"capture", "inject"}
                or (
                    getattr(args, "radio_action", None)
                    in {"uid", "set-uid", "bind-phrase"}
                    and getattr(args, "role", "auto") == "rx"
                )
            )
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


def _print_monitor(report: MonitorReport) -> None:
    print(
        f"status={report.status} duration={report.duration_sec:.3f}s "
        f"bytes={report.bytes_read} frames={report.frames} "
        f"crc_errors={report.crc_errors} resyncs={report.resyncs} "
        f"framing_errors={report.framing_errors}"
    )
    for name, count in report.frames_by_type.items():
        age = report.last_seen_seconds.get(name)
        print(f"{name:20s} {count:8d} last_seen={age:.3f}s")


def _print_capture(report: CaptureReport) -> None:
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


def _print_drive(report: DriveReport) -> None:
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


def _uid_client(
    link: CrsfLink,
    *,
    role: str,
    timeout: float,
) -> CrsfParameterClient:
    if role == "auto":
        probe = CrsfParameterClient(
            link,
            destination=CRSF_ADDRESS_BROADCAST,
            origin=CRSF_ADDRESS_ELRS_LUA,
            timeout=timeout,
        )
        info = probe.ping(timeout=max(3.0, timeout))
        destination = (
            CRSF_ADDRESS_RECEIVER
            if info.origin == CRSF_ADDRESS_RECEIVER
            else CRSF_ADDRESS_CRSF_TRANSMITTER
        )
    else:
        destination = (
            CRSF_ADDRESS_RECEIVER
            if role == "rx"
            else CRSF_ADDRESS_CRSF_TRANSMITTER
        )
    return CrsfParameterClient(
        link,
        destination=destination,
        origin=CRSF_ADDRESS_ELRS_LUA,
        timeout=timeout,
    )


def _parameter_client(
    link: CrsfLink,
    *,
    role: str,
    timeout: float,
) -> CrsfParameterClient:
    if role == "rx":
        destination = CRSF_ADDRESS_RECEIVER
    elif role == "tx":
        destination = CRSF_ADDRESS_CRSF_TRANSMITTER
    else:
        info = CrsfParameterClient(
            link,
            destination=CRSF_ADDRESS_BROADCAST,
            origin=CRSF_ADDRESS_RADIO_TRANSMITTER,
            timeout=timeout,
        ).ping(timeout=max(3.0, timeout))
        destination = (
            CRSF_ADDRESS_RECEIVER
            if info.origin == CRSF_ADDRESS_RECEIVER
            else CRSF_ADDRESS_CRSF_TRANSMITTER
        )
    return CrsfParameterClient(
        link,
        destination=destination,
        # The RX OTA connector already claims the normal handset address
        # (0xEA), which would steer parameter replies onto RF instead of USB.
        # ELRS Lua is the host-management origin and resolves to the USB
        # connector on both roles.
        origin=CRSF_ADDRESS_ELRS_LUA,
        timeout=timeout,
    )


def _role_for_uid(client: CrsfParameterClient) -> str:
    return "rx" if client.destination == CRSF_ADDRESS_RECEIVER else "tx"


def _read_uid_for_link(
    link: CrsfLink,
    *,
    role: str,
    timeout: float,
) -> tuple[CrsfParameterClient, bytes]:
    client = _uid_client(link, role=role, timeout=timeout)
    return client, client.read_uid(attempts=2, timeout=max(4.0, timeout))


def _radio_uid(args: argparse.Namespace) -> int:
    link = _radio_link(args)
    try:
        client, uid = _read_uid_for_link(
            link,
            role=args.role,
            timeout=args.timeout,
        )
        print(
            f"role={_role_for_uid(client)} uid={format_uid(uid)} "
            f"unbound={str(uid == bytes(6)).lower()}"
        )
    finally:
        link.close()
    return 0


def _radio_diag(args: argparse.Namespace) -> int:
    if not args.port:
        raise ValueError("radio diag requires --port")
    baud = args.baud
    if baud is None:
        baud = CRSF_RX_FC_BAUD if args.role == "rx" else CRSF_BAUD
    transport = SerialTransport(
        args.port,
        baudrate=baud,
        timeout=0.1,
        dtr=False,
        rts=False,
    )
    deadline = time.monotonic() + max(0.5, args.duration)
    raw = bytearray()
    try:
        while (remaining := deadline - time.monotonic()) > 0:
            raw.extend(transport.read(4096, min(0.2, remaining)))
            text = raw.decode("ascii", "ignore")
            if "AK-BOOT " in text and not getattr(args, "watch_rf", False):
                break
    finally:
        transport.close()

    text = raw.decode("ascii", "ignore")
    boot_line = next(
        (line for line in text.splitlines() if line.startswith("AK-BOOT ")),
        None,
    )
    if boot_line is None:
        raise TimeoutError(
            f"no AK-BOOT diagnostic line arrived from {args.port} "
            f"within {max(0.5, args.duration):g}s"
        )
    fields = {
        key: value
        for key, value in re.findall(r"([A-Za-z0-9_]+)=([^\s]+)", boot_line)
    }
    rf_lines = [
        line for line in text.splitlines() if line.startswith("AK-RF ")
    ]
    result = {
        "path": args.port,
        "logical_baud": baud,
        "line": boot_line,
        "fields": fields,
        "rf_lines": rf_lines,
        "last_rf_fields": (
            {
                key: value
                for key, value in re.findall(
                    r"([A-Za-z0-9_]+)=([^\s]+)", rf_lines[-1]
                )
            }
            if rf_lines
            else None
        ),
    }
    if args.json:
        _write_json(args.json, result)
    if args.json != "-":
        print(boot_line)
    return 0


def _radio_identity_write(
    args: argparse.Namespace,
    *,
    phrase: str | None,
    uid_text: str | None,
) -> int:
    _require_write(args)
    if phrase is None and uid_text is None:
        raise ValueError("radio set-uid requires a UID")
    expected = binding_uid(phrase) if phrase is not None else parse_uid(uid_text or "")
    first = _radio_link(args)
    try:
        client, before = _read_uid_for_link(
            first,
            role=args.role,
            timeout=args.timeout,
        )
        role = _role_for_uid(client)
        print(
            f"before role={role} uid={format_uid(before)} "
            f"unbound={str(before == bytes(6)).lower()}"
        )
        if phrase is None:
            client.write_uid(expected, allow_write=True)
            operation = "set-uid"
        else:
            client.write_binding_phrase(phrase, allow_write=True)
            operation = "bind-phrase"
        print(f"operation={operation} write_sent=true expected_uid={format_uid(expected)}")
    finally:
        first.close()

    time.sleep(max(0.0, args.reboot_wait))
    second = _radio_link(args)
    try:
        client, after = _read_uid_for_link(
            second,
            role=role,
            timeout=args.timeout,
        )
        if after != expected:
            raise CrsfProtocolError(
                f"UID readback mismatch after {operation}: "
                f"expected {format_uid(expected)}, got {format_uid(after)}"
            )
        print(f"verified role={_role_for_uid(client)} uid={format_uid(after)}")
    finally:
        second.close()
    return 0


def _pair_uid(
    path: str,
    *,
    role: str,
    timeout: float,
) -> bytes:
    transport = SerialTransport(
        path,
        baudrate=CRSF_BAUD if role == "tx" else CRSF_RX_FC_BAUD,
        timeout=timeout,
    )
    link = CrsfLink(transport)
    try:
        _, uid = _read_uid_for_link(link, role=role, timeout=timeout)
        return uid
    finally:
        link.close()


def _pair_write_phrase(
    path: str,
    *,
    role: str,
    phrase: str,
    timeout: float,
    reboot_wait: float,
) -> tuple[bytes, bytes]:
    transport = SerialTransport(
        path,
        baudrate=CRSF_BAUD if role == "tx" else CRSF_RX_FC_BAUD,
        timeout=timeout,
    )
    link = CrsfLink(transport)
    try:
        client, before = _read_uid_for_link(link, role=role, timeout=timeout)
        client.write_binding_phrase(phrase, allow_write=True)
    finally:
        link.close()
    time.sleep(max(0.0, reboot_wait))
    after = _pair_uid(path, role=role, timeout=timeout)
    return before, after


def _radio_pair(args: argparse.Namespace) -> int:
    _require_write(args)
    if not args.tx or not args.rx:
        raise ValueError("radio pair requires --tx and --rx")
    if args.tx == args.rx:
        raise ValueError("--tx and --rx must be distinct USB endpoints")
    phrase = getattr(args, "phrase", None)
    if phrase is None and getattr(args, "field", None) is not None:
        phrase = str(args.field)
    if phrase is None:
        raise ValueError("radio pair requires a binding phrase")

    started_at = datetime.now(timezone.utc).isoformat()
    expected = binding_uid(phrase)
    tx_before, tx_after = _pair_write_phrase(
        args.tx,
        role="tx",
        phrase=phrase,
        timeout=args.timeout,
        reboot_wait=args.reboot_wait,
    )
    rx_before, rx_after = _pair_write_phrase(
        args.rx,
        role="rx",
        phrase=phrase,
        timeout=args.timeout,
        reboot_wait=args.reboot_wait,
    )
    if tx_after != expected or rx_after != expected or tx_after != rx_after:
        raise CrsfProtocolError(
            "pair readback mismatch: "
            f"expected {format_uid(expected)}, TX {format_uid(tx_after)}, "
            f"RX {format_uid(rx_after)}"
        )
    if tx_after == bytes(6):
        raise CrsfProtocolError("pair readback is all zero; the endpoints are unbound")

    tx_transport = SerialTransport(args.tx, baudrate=CRSF_BAUD, timeout=0.05)
    rx_transport = SerialTransport(args.rx, baudrate=CRSF_RX_FC_BAUD, timeout=0.05)
    try:
        bench = run_two_board_bench(
            tx_transport,
            rx_transport,
            tx_path=args.tx,
            rx_path=args.rx,
            duration=args.duration,
            hz=args.hz,
            channel_map=_radio_channel_map(args),
        )
    finally:
        tx_transport.close()
        rx_transport.close()

    result = {
        "started_at": started_at,
        "ended_at": datetime.now(timezone.utc).isoformat(),
        "tx_path": args.tx,
        "rx_path": args.rx,
        "expected_uid": format_uid(expected),
        "tx_before_uid": format_uid(tx_before),
        "rx_before_uid": format_uid(rx_before),
        "tx_uid": format_uid(tx_after),
        "rx_uid": format_uid(rx_after),
        "uids_match": True,
        "bench": bench.to_dict(),
        "overall": bench.overall,
    }
    if args.json:
        _write_json(args.json, result)
    if args.json != "-":
        print(f"uid_binding=verified uid={format_uid(tx_after)}")
        for name, status in bench.requirements.items():
            print(f"{name}={status}")
        print(f"overall={bench.overall}")
    return 0 if bench.overall == "PASS" else 1


def _radio_bench(args: argparse.Namespace) -> int:
    _require_write(args)
    if not args.tx or not args.rx:
        raise ValueError("radio bench requires --tx and --rx")
    tx_transport = SerialTransport(args.tx, baudrate=CRSF_BAUD, timeout=0.05)
    rx_transport = SerialTransport(args.rx, baudrate=CRSF_RX_FC_BAUD, timeout=0.05)
    try:
        tx_link = CrsfLink(tx_transport)
        rx_link = CrsfLink(rx_transport)
        _, tx_uid = _read_uid_for_link(
            tx_link,
            role="tx",
            timeout=args.timeout,
        )
        _, rx_uid = _read_uid_for_link(
            rx_link,
            role="rx",
            timeout=args.timeout,
        )
        if tx_uid != rx_uid:
            raise CrsfProtocolError(
                "refusing the two-board RF test: TX UID "
                f"{format_uid(tx_uid)} does not match RX UID {format_uid(rx_uid)}"
            )
        if tx_uid == bytes(6):
            raise CrsfProtocolError(
                "refusing the two-board RF test: both endpoints are unbound"
            )
        print(f"binding=verified uid={format_uid(tx_uid)}")
        report = run_two_board_bench(
            tx_transport,
            rx_transport,
            tx_path=args.tx,
            rx_path=args.rx,
            duration=args.duration,
            hz=args.hz,
            channel_map=_radio_channel_map(args),
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


def _radio_channel_map(args: argparse.Namespace) -> CrsfChannelMap:
    profile = getattr(args, "profile", "aerial-kit")
    defaults = (
        {"arm": 5, "mode": 7}
        if profile == "betaflight"
        else {"arm": 6, "mode": 5}
    )
    arm = getattr(args, "arm_channel", None)
    mode = getattr(args, "mode_channel", None)
    velocity = getattr(args, "velocity_channel", None)
    return CrsfChannelMap(
        arm=int(defaults["arm"] if arm is None else arm),
        mode=int(defaults["mode"] if mode is None else mode),
        velocity=int(0 if velocity is None else velocity),
    )


def _radio_sticks(text: str | None) -> RcSticks | None:
    if text is None:
        return None
    parts = text.split(",")
    if len(parts) != 4:
        raise ValueError("--sticks requires roll,pitch,yaw,throttle")
    try:
        roll, pitch, yaw, throttle = (float(part) for part in parts)
    except ValueError as exc:
        raise ValueError("--sticks values must be numeric") from exc
    if any(not -1.0 <= value <= 1.0 for value in (roll, pitch, yaw)):
        raise ValueError("roll, pitch and yaw sticks must be in -1..1")
    if not 0.0 <= throttle <= 1.0:
        raise ValueError("throttle stick must be in 0..1")
    return RcSticks(roll=roll, pitch=pitch, yaw=yaw, throttle=throttle)


def _print_parameter(parameter: CrsfParameter) -> None:
    value = "" if parameter.value is None else parameter.value
    print(
        f"{parameter.id:3d} {parameter.name:24s} {str(value):16s} "
        f"{parameter.type}"
    )


def _radio(args: argparse.Namespace) -> int:
    if args.radio_action in {"get", "set"}:
        if args.field is None:
            raise ValueError(f"radio {args.radio_action} requires FIELD")
        try:
            args.field = int(args.field)
        except (TypeError, ValueError) as exc:
            raise ValueError("radio field must be an integer") from exc
    if args.radio_action == "set":
        if args.value is None:
            raise ValueError("radio set requires FIELD and VALUE")
        try:
            args.value = int(args.value)
        except (TypeError, ValueError) as exc:
            raise ValueError("radio value must be an integer") from exc
    if args.radio_action in {"set", "bind", "drive", "inject", "bench"}:
        _require_write(args)
    requested_sticks = _radio_sticks(getattr(args, "sticks", None))
    if requested_sticks is not None and not getattr(args, "confirm_arm", False):
        raise PermissionError(
            "a custom stick command can arm; pass --confirm-arm explicitly"
        )
    if args.radio_action == "bench":
        return _radio_bench(args)
    if args.radio_action == "pair":
        return _radio_pair(args)
    if args.radio_action == "status":
        return _radio_status(args)
    if args.radio_action == "uid":
        return _radio_uid(args)
    if args.radio_action == "diag":
        return _radio_diag(args)
    if args.radio_action == "set-uid":
        value = args.uid if getattr(args, "uid", None) is not None else args.field
        return _radio_identity_write(args, phrase=None, uid_text=value)
    if args.radio_action == "bind-phrase":
        value = args.phrase if getattr(args, "phrase", None) is not None else args.field
        if value is None:
            raise ValueError("radio bind-phrase requires a phrase")
        return _radio_identity_write(args, phrase=str(value), uid_text=None)
    link = _radio_link(args)
    transport = link.transport
    try:
        client = (
            _parameter_client(link, role=args.role, timeout=args.timeout)
            if args.radio_action in {"params", "get", "set"}
            else CrsfParameterClient(link, timeout=args.timeout)
        )
        if args.radio_action == "params":
            def parameter_progress(parameter: CrsfParameter) -> None:
                print(
                    f"field={parameter.id} parent={parameter.parent} "
                    f"type={parameter.type} name={parameter.name}",
                    file=sys.stderr,
                    flush=True,
                )

            for parameter in client.read_parameter_tree(
                on_parameter=parameter_progress
            ).values():
                _print_parameter(parameter)
        elif args.radio_action == "get":
            _print_parameter(client.read_parameter(args.field))
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
                    return 0
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
                channel_map=_radio_channel_map(args),
                sticks=requested_sticks,
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


@dataclass
class _FcSession:
    protocol: str
    client: AkClient | MspClient
    transport: Transport
    identity: AkHello | dict[str, object]


def _connect_fc(args: argparse.Namespace) -> _FcSession:
    requested = getattr(args, "protocol", "auto")
    if args.radio:
        if requested == "betaflight":
            raise ValueError("MSP/Betaflight is not carried over the AKP radio tunnel")
        link = CrsfLink(
            SerialTransport(args.radio, baudrate=CRSF_BAUD, timeout=args.timeout)
        )
        client = AkClient(CrsfAkpTunnel(link), timeout=args.timeout)
        return _FcSession("akp", client, link.transport, client.hello())

    transport = _transport_from_args(args)
    if requested == "akp":
        client = AkClient(transport, timeout=args.timeout)
        return _FcSession("akp", client, transport, client.hello())

    msp_client = MspClient(
        transport.read,
        transport.write,
        timeout=args.timeout,
    )
    if requested == "betaflight":
        return _FcSession("betaflight", msp_client, transport, identify_msp(msp_client))

    akp_client = AkClient(transport, timeout=args.timeout)
    try:
        return _FcSession("akp", akp_client, transport, akp_client.hello())
    except (AkError, OSError, TimeoutError) as akp_error:
        try:
            identity = identify_msp(msp_client)
        except (MspError, OSError, TimeoutError) as msp_error:
            transport.close()
            raise TimeoutError(
                "no Aerial Kit or Betaflight reply; "
                f"AKP: {akp_error}; MSP: {msp_error}"
            ) from msp_error
        return _FcSession("betaflight", msp_client, transport, identity)


def _fc_identity_line(session: _FcSession) -> str:
    if session.protocol == "akp":
        assert isinstance(session.identity, AkHello)
        hello = session.identity
        return (
            f"protocol=akp product={hello.product} firmware={hello.version} "
            f"parameters={hello.parameter_count} changed={hello.changed}"
        )
    identity = session.identity
    assert isinstance(identity, dict)
    return f"protocol=betaflight {describe_msp(identity)}"


def _fc_state(session: _FcSession) -> dict[str, object]:
    if session.protocol == "akp":
        assert isinstance(session.client, AkClient)
        status = session.client.status()
        return {
            "armed": int(status.flight_state),
            "link_live": status.link_live,
            "roll_deg": status.roll_deg,
            "pitch_deg": status.pitch_deg,
            "yaw_deg": status.yaw_deg,
            "gps_fix_type": status.gps_fix_type,
            "gps_satellites": status.gps_satellites,
            "latitude": status.latitude,
            "longitude": status.longitude,
            "motors": list(status.motors),
        }
    assert isinstance(session.client, MspClient)
    state = dict(msp_state(session.client))
    state["armed"] = state.get("flight_state")
    return state


def _print_fc_state(state: dict[str, object]) -> None:
    fields = [
        f"armed={_as_int(state.get('armed', 0))}",
        f"roll_deg={_as_float(state.get('roll_deg', 0.0)):.1f}",
        f"pitch_deg={_as_float(state.get('pitch_deg', 0.0)):.1f}",
        f"yaw_deg={_as_float(state.get('yaw_deg', 0.0)):.1f}",
    ]
    if state.get("vbat_v") is not None:
        fields.append(f"vbat_v={_as_float(state['vbat_v']):.2f}")
    if state.get("mah") is not None:
        fields.append(f"mah={_as_int(state['mah'])}")
    if state.get("gps_fix_type") is not None:
        fields.append(f"gps_fix={state['gps_fix_type']}")
        fields.append(f"sats={state.get('gps_satellites')}")
    print(" ".join(fields))


def _as_int(value: object) -> int:
    if isinstance(value, (bool, int, float, str, bytes, bytearray)):
        return int(value)
    raise ValueError(f"{value!r} is not an integer value")


def _as_float(value: object) -> float:
    if isinstance(value, (bool, int, float, str, bytes, bytearray)):
        return float(value)
    raise ValueError(f"{value!r} is not a numeric value")


def _require_fc_disarmed(session: _FcSession) -> None:
    state = _fc_state(session)
    armed = state.get("armed")
    if armed is None:
        raise PermissionError("the flight controller did not report an armed state")
    if _as_int(armed) != 0:
        raise PermissionError("the flight controller is armed; refusing the operation")


def _fc_argument_value(args: argparse.Namespace) -> object | None:
    positional = cast(object | None, getattr(args, "fc_value", None))
    if positional is not None:
        return positional
    return cast(object | None, args.value)


def _fc_selected_name(args: argparse.Namespace) -> str | None:
    named = cast(object, args.name)
    if isinstance(named, list) and named:
        return str(named[0])
    if isinstance(named, str) and named:
        return named
    field = cast(object, getattr(args, "field", None))
    return field if isinstance(field, str) and field else None


def _akp_index_for_name(client: AkClient, name: str) -> int:
    for item in client.parameter_info():
        if item.name == name:
            return int(item.index)
    raise ValueError(f"Aerial Kit has no parameter named {name!r}")


def _fc_index(args: argparse.Namespace, client: AkClient) -> int:
    if args.index is not None:
        return int(args.index)
    name = _fc_selected_name(args)
    if not name:
        raise ValueError("fc get/set requires --index or a parameter/setting name")
    if name.isdigit():
        return int(name)
    return _akp_index_for_name(client, name)


def _fc_msp_setting_name(args: argparse.Namespace) -> str:
    name = _fc_selected_name(args)
    if not name:
        raise ValueError("Betaflight settings are selected by name, not by index")
    return name


def _fc(session: _FcSession, args: argparse.Namespace) -> int:
    action = args.fc_action
    if action in {"hello", "info"}:
        print(_fc_identity_line(session))
        return 0

    if action == "status":
        print(_fc_identity_line(session))
        state = _fc_state(session)
        _print_fc_state(state)
        if args.json:
            _write_json(args.json, {"identity": _fc_identity_line(session), "state": state})
        return 0

    if action == "params":
        if session.protocol == "akp":
            assert isinstance(session.client, AkClient)
            for item in session.client.parameter_info():
                print(
                    f"{item.index:3d} {item.name or '':24s} "
                    f"{item.type if item.type is not None else ''} "
                    f"{item.group_name or ''}"
                )
            return 0
        assert isinstance(session.client, MspClient)
        names = args.name or []
        if not names:
            print(
                "Betaflight does not enumerate settings; use "
                "`--name NAME` (repeatable) or `ak fc get NAME`."
            )
            return 0
        for name in names:
            print(session.client.setting(name))
        return 0

    if action == "get":
        if session.protocol == "akp":
            assert isinstance(session.client, AkClient)
            value = session.client.get_parameter(_fc_index(args, session.client))
            print(f"{value.name}={value.value} status={value.status}")
        else:
            assert isinstance(session.client, MspClient)
            print(session.client.setting(_fc_msp_setting_name(args)))
        return 0

    if action == "set":
        _require_write(args)
        value = _fc_argument_value(args)
        if value is None:
            raise ValueError("fc set requires a value")
        _require_fc_disarmed(session)
        if session.protocol == "akp":
            assert isinstance(session.client, AkClient)
            result = session.client.set_parameter(
                _fc_index(args, session.client),
                str(value),
                allow_write=True,
            )
            print(f"status={result.status} {result.message}")
        else:
            assert isinstance(session.client, MspClient)
            print(session.client.set_setting(
                _fc_msp_setting_name(args),
                value,
                allow_write=True,
            ))
        return 0

    if action == "save":
        _require_write(args)
        _require_fc_disarmed(session)
        if session.protocol == "akp":
            assert isinstance(session.client, AkClient)
            result = session.client.save_parameters(allow_write=True)
            print(f"status={result.status} {result.message}")
        else:
            assert isinstance(session.client, MspClient)
            session.client.save(allow_write=True)
            print("save acknowledged by the flight controller")
        return 0

    if action in {"telemetry", "monitor"}:
        samples = 1 if action == "telemetry" else max(
            1, int(round(args.duration * args.hz))
        )
        interval = 0.0 if action == "telemetry" else 1.0 / max(1.0, args.hz)
        rows: list[dict[str, object]] = []
        for number in range(samples):
            state = _fc_state(session)
            rows.append({"time": time.time(), "state": state})
            _print_fc_state(state)
            if number + 1 < samples:
                time.sleep(interval)
        if args.json:
            _write_json(args.json, rows)
        return 0

    if action == "rc":
        if session.protocol == "akp":
            assert isinstance(session.client, AkClient)
            rc = session.client.rc_channels()
            print(
                f"status={rc.status_name} protocol={rc.protocol_name or ''} "
                f"link={'link' in rc.flag_names} "
                f"channels={','.join(map(str, rc.channels)) or 'none'}"
            )
        else:
            assert isinstance(session.client, MspClient)
            print("channels=" + ",".join(map(str, session.client.rc_channels())))
        return 0

    if action == "preflight":
        if session.protocol == "akp":
            assert isinstance(session.client, AkClient)
            for line in session.client.preflight():
                print(f"{line.name} verdict={line.verdict} {line.detail}")
        else:
            state = _fc_state(session)
            print(
                f"armed={_as_int(state.get('armed', 0))} "
                f"arming_disable_flags={state.get('arming_disable_flags', 'unknown')}"
            )
        return 0

    if action == "logs":
        if session.protocol != "akp":
            raise ValueError("MSP/Betaflight does not expose the Aerial Kit log store")
        assert isinstance(session.client, AkClient)
        print(f"records={session.client.log_info().count}")
        if args.index is not None:
            print(session.client.log_record(args.index))
        return 0

    raise ValueError(f"unknown flight-controller action {action!r}")


def _fc_command(args: argparse.Namespace) -> int:
    if args.fc_action in {"set", "save"}:
        _require_write(args)
    if args.fc_action in {"get", "set"} and (
        args.index is None and _fc_selected_name(args) is None
    ):
        raise ValueError("fc get/set requires --index or a parameter/setting name")
    if args.fc_action == "set" and _fc_argument_value(args) is None:
        raise ValueError("fc set requires a value")
    if args.duration < 0:
        raise ValueError("fc duration must not be negative")
    if args.hz <= 0:
        raise ValueError("fc hz must be positive")
    session = _connect_fc(args)
    try:
        return _fc(session, args)
    finally:
        session.transport.close()


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


def build_parser(prog: str = "aerial-kit-link") -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(prog=prog)
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
            "uid",
            "diag",
            "params",
            "get",
            "set",
            "set-uid",
            "bind-phrase",
            "bind",
            "channels",
            "monitor",
            "drive",
            "capture",
            "inject",
            "bench",
            "pair",
        ),
    )
    radio.add_argument("field", nargs="?")
    radio.add_argument("value", nargs="?")
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
    radio.add_argument(
        "--sticks",
        metavar="ROLL,PITCH,YAW,THROTTLE",
        help="hold one normalised stick command after the arm sequence",
    )
    radio.add_argument(
        "--confirm-arm",
        action="store_true",
        default=argparse.SUPPRESS,
        help="required together with --write for --sticks",
    )
    radio.add_argument(
        "--profile",
        choices=("aerial-kit", "betaflight"),
        default="aerial-kit",
        help="switch defaults: Aerial Kit arm CH6/mode CH5; Betaflight arm CH5/mode CH7",
    )
    radio.add_argument("--arm-channel", type=int)
    radio.add_argument("--mode-channel", type=int)
    radio.add_argument(
        "--velocity-channel",
        type=int,
        default=0,
        help="Aerial Kit velocity-hold AUX channel; 0 disables it",
    )
    radio.add_argument("--role", choices=("auto", "tx", "rx"), default="auto")
    radio.add_argument("--file")
    radio.add_argument("--uid")
    radio.add_argument("--phrase")
    radio.add_argument(
        "--reboot-wait",
        type=float,
        default=1.0,
        help="seconds to wait before reconnecting after an identity write",
    )
    radio.add_argument("--tx", metavar="PORT")
    radio.add_argument("--rx", metavar="PORT")
    radio.add_argument("--json")
    radio.add_argument(
        "--watch-rf",
        action="store_true",
        default=argparse.SUPPRESS,
        help="keep reading after AK-BOOT and collect periodic AK-RF diagnostics",
    )
    radio.set_defaults(handler=_radio)

    fc = commands.add_parser("fc", help="query Aerial Kit or Betaflight")
    fc_transport = fc.add_mutually_exclusive_group()
    fc_transport.add_argument("--port", default=argparse.SUPPRESS)
    fc_transport.add_argument("--host", default=argparse.SUPPRESS)
    fc_transport.add_argument(
        "--radio",
        metavar="PORT",
        default=argparse.SUPPRESS,
        help="carry Aerial Kit AKP over an ELRS transmitter module",
    )
    fc.add_argument("--baud", type=int, default=argparse.SUPPRESS)
    fc.add_argument("--timeout", type=float, default=argparse.SUPPRESS)
    fc.add_argument(
        "fc_action",
        choices=(
            "hello",
            "info",
            "status",
            "params",
            "get",
            "set",
            "save",
            "telemetry",
            "monitor",
            "rc",
            "preflight",
            "logs",
        ),
    )
    fc.add_argument(
        "field",
        nargs="?",
        help="parameter index or Betaflight setting name",
    )
    fc.add_argument("fc_value", nargs="?")
    fc.add_argument(
        "--protocol",
        choices=("auto", "akp", "betaflight"),
        default="auto",
    )
    fc.add_argument("--index", type=int)
    fc.add_argument(
        "--name",
        action="append",
        help="Betaflight setting name; repeatable for `fc params`",
    )
    fc.add_argument("--value", type=float)
    fc.add_argument("--hz", type=float, default=5.0)
    fc.add_argument("--duration", type=float, default=1.0)
    fc.add_argument("--json")
    fc.add_argument(
        "--write", action="store_true", default=argparse.SUPPRESS,
        help="allow a device mutation",
    )
    fc.set_defaults(handler=_fc_command)

    mav = commands.add_parser("mavlink", help="query a PX4 or ArduPilot vehicle")
    mav.add_argument("mav_action", choices=("status", "params", "mission"))
    mav.set_defaults(handler=_mavlink)
    return parser


def main(argv: list[str] | None = None, *, prog: str = "aerial-kit-link") -> int:
    parser = build_parser(prog=prog)
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
