from __future__ import annotations

import importlib.util
import math
import socket
import subprocess
import sys
import time
from pathlib import Path
from types import ModuleType
from typing import Any

import pytest

from aerial_kit.comms.mavlink import (
    MavlinkCommandRejected,
    MavlinkMissionRejected,
    MavlinkParameter,
    MavlinkVehicle,
    MavlinkWriteRefused,
    PymavlinkDialect,
    euler_to_quaternion_ned,
    ned_body_to_enu_quaternion,
    ned_to_enu_position,
    yaw_from_quaternion_enu,
)
from aerial_kit.comms.mavlink import codec
from aerial_kit.comms.transport import MemoryTransport, UdpTransport


class MavReplyTransport(MemoryTransport):
    def __init__(self, ack_result: int = 0) -> None:
        super().__init__(timeout=0)
        self.outgoing: list[bytes] = []
        self.ack_result = ack_result

    def _write(self, data: bytes) -> int:
        self.outgoing.append(bytes(data))
        name, fields, header = codec.parse(data)
        if name == "COMMAND_LONG":
            self.feed(codec.build(
                "COMMAND_ACK",
                {"command": fields["command"], "result": self.ack_result,
                 "target_system": 1, "target_component": 1},
                sequence=0, system=1, component=1,
            ))
        return len(data)


def test_transforms_are_the_library_source() -> None:
    assert ned_to_enu_position(1.0, 2.0, -3.0) == (2.0, 1.0, 3.0)
    q = euler_to_quaternion_ned(0.0, 0.0, 0.0)
    enu = ned_body_to_enu_quaternion(*q)
    assert yaw_from_quaternion_enu(*enu) == pytest.approx(math.pi / 2.0)


def test_vehicle_writes_require_opt_in_and_arm_sends_command() -> None:
    transport = MavReplyTransport()
    vehicle = MavlinkVehicle(transport)
    with pytest.raises(MavlinkWriteRefused):
        vehicle.arm()
    assert transport.outgoing == []

    vehicle.allow_write = True
    vehicle.mav.vehicle = (1, 1)
    ack = vehicle.arm()
    assert ack["fields"]["command"] == 400
    assert len(transport.outgoing) == 1
    name, fields, _ = codec.parse(transport.outgoing[0])
    assert name == "COMMAND_LONG"
    assert fields["param1"] == 1.0


def test_parameter_and_position_write_frames_are_explicit() -> None:
    transport = MavReplyTransport()
    vehicle = MavlinkVehicle(transport, allow_write=True)
    vehicle.mav.vehicle = (1, 1)
    vehicle.set_velocity_target(1.0, -2.0, 0.5, 0.1)
    name, fields, _ = codec.parse(transport.outgoing[-1])
    assert name == "SET_POSITION_TARGET_LOCAL_NED"
    assert fields["vx"] == pytest.approx(1.0)
    assert fields["vy"] == pytest.approx(-2.0)
    assert fields["yaw_rate"] == pytest.approx(0.1)

    vehicle.set_position_target(47.0, -122.0, 100.0)
    name, fields, _ = codec.parse(transport.outgoing[-1])
    assert name == "SET_POSITION_TARGET_GLOBAL_INT"
    assert fields["lat_int"] == 470_000_000
    assert fields["lon_int"] == -1_220_000_000
    assert fields["type_mask"] == 0x0DF8

    vehicle.set_velocity_target(1.0, -2.0, 0.5, 0.1)
    _name, fields, _ = codec.parse(transport.outgoing[-1])
    assert fields["type_mask"] == 0x05C7


def test_parameter_set_uses_param_set_wire_layout() -> None:
    transport = MavReplyTransport()
    vehicle = MavlinkVehicle(transport, allow_write=True)
    vehicle.mav.vehicle = (1, 1)

    vehicle.parameters = lambda **_kwargs: (
        MavlinkParameter("RATE_RLL_P", 0.2, 9, 0, 1),
    )  # type: ignore[method-assign]
    vehicle.mav.timeout = 0.01
    # A vehicle that does not echo a write returns None, which is the documented
    # unresolved outcome rather than a fabricated success.
    assert vehicle.set_parameter("RATE_RLL_P", 0.3) is None
    name, fields, _ = codec.parse(transport.outgoing[-1])
    assert name == "PARAM_SET"
    assert fields["param_id"] == "RATE_RLL_P"
    assert fields["param_value"] == pytest.approx(0.3)


def test_command_rejection_is_typed() -> None:
    transport = MavReplyTransport(ack_result=4)
    vehicle = MavlinkVehicle(transport, allow_write=True)
    vehicle.mav.vehicle = (1, 1)
    with pytest.raises(MavlinkCommandRejected) as caught:
        vehicle.set_mode(4)
    assert caught.value.command == 176
    assert caught.value.result == 4


class ParameterTransport(MemoryTransport):
    def __init__(self) -> None:
        super().__init__(timeout=0)
        self.outgoing: list[bytes] = []

    def _write(self, data: bytes) -> int:
        self.outgoing.append(bytes(data))
        name, _fields, _header = codec.parse(data)
        if name == "PARAM_REQUEST_LIST":
            for index, param_name in ((7, "SECOND"), (3, "FIRST")):
                self.feed(codec.build(
                    "PARAM_VALUE",
                    {
                        "param_value": float(index),
                        "param_count": 2,
                        "param_index": index,
                        "param_id": param_name,
                        "param_type": 9,
                    },
                    sequence=index,
                    system=1,
                    component=1,
                ))
        return len(data)


class ParameterWriteTransport(ParameterTransport):
    def _write(self, data: bytes) -> int:
        self.outgoing.append(bytes(data))
        name, fields, _header = codec.parse(data)
        if name == "PARAM_REQUEST_LIST":
            for index, param_name in ((7, "SECOND"), (3, "FIRST")):
                self.feed(codec.build(
                    "PARAM_VALUE",
                    {
                        "param_value": float(index),
                        "param_count": 2,
                        "param_index": index,
                        "param_id": param_name,
                        "param_type": 9,
                    },
                    sequence=index,
                    system=1,
                    component=1,
                ))
        if name == "PARAM_SET":
            self.feed(codec.build(
                "PARAM_VALUE",
                {
                    "param_value": fields["param_value"],
                    "param_count": 2,
                    "param_index": 3,
                    "param_id": fields["param_id"],
                    "param_type": fields["param_type"],
                },
                system=1,
                component=1,
            ))
        return len(data)


def test_parameter_list_preserves_wire_indices() -> None:
    transport = ParameterTransport()
    vehicle = MavlinkVehicle(transport)
    vehicle.mav.vehicle = (1, 1)
    parameters = vehicle.parameters(timeout=0.05)
    assert [(item.index, item.name) for item in parameters] == [
        (3, "FIRST"),
        (7, "SECOND"),
    ]


def test_parameter_read_and_successful_write_are_typed() -> None:
    transport = ParameterWriteTransport()
    vehicle = MavlinkVehicle(transport, allow_write=True)
    vehicle.mav.vehicle = (1, 1)
    parameter = vehicle.parameter("FIRST", timeout=0.05)
    assert parameter.index == 3
    assert parameter.value == 3.0

    written = vehicle.set_parameter("FIRST", 2.5)
    assert written is not None
    assert written.name == "FIRST"
    assert written.value == pytest.approx(2.5)
    name, fields, _header = codec.parse(transport.outgoing[-1])
    assert name == "PARAM_SET"
    assert fields["param_value"] == pytest.approx(2.5)
    assert fields["target_system"] == 1
    assert fields["target_component"] == 1


def telemetry_frame(name: str, fields: dict[str, Any], sequence: int) -> bytes:
    return codec.build(
        name, fields, sequence=sequence, system=1, component=1
    )


def test_telemetry_collects_attitude_position_velocity_battery_gps_and_text() -> None:
    transport = MemoryTransport(timeout=0)
    vehicle = MavlinkVehicle(transport)
    frames = [
        telemetry_frame("HEARTBEAT", {
            "type": 2, "autopilot": 3, "base_mode": 128,
            "custom_mode": 4, "system_status": 3, "mavlink_version": 3,
        }, 0),
        telemetry_frame("ATTITUDE", {
            "time_boot_ms": 1000, "roll": math.radians(10),
            "pitch": math.radians(-5), "yaw": math.radians(270),
            "rollspeed": 0.0, "pitchspeed": 0.0, "yawspeed": 0.1,
        }, 1),
        telemetry_frame("GLOBAL_POSITION_INT", {
            "time_boot_ms": 1000, "lat": 473_977_000, "lon": -1_227_654_321,
            "alt": 50_000, "relative_alt": 25_000,
            "vx": 125, "vy": -250, "vz": 75, "hdg": 27_000,
        }, 2),
        telemetry_frame("GPS_RAW_INT", {
            "time_usec": 1, "lat": 473_977_000, "lon": -1_227_654_321,
            "alt": 50_000, "eph": 90, "epv": 120, "vel": 300,
            "cog": 27_000, "fix_type": 3, "satellites_visible": 14,
            "alt_ellipsoid": 0, "h_acc": 0, "v_acc": 0,
            "vel_acc": 0, "hdg_acc": 0, "yaw": 0,
        }, 3),
        telemetry_frame("SYS_STATUS", {
            "onboard_control_sensors_present": 0,
            "onboard_control_sensors_enabled": 0,
            "onboard_control_sensors_health": 0, "load": 100,
            "voltage_battery": 16_800, "current_battery": 1_200,
            "drop_rate_comm": 0, "errors_comm": 0, "errors_count1": 0,
            "errors_count2": 0, "errors_count3": 0, "errors_count4": 0,
            "battery_remaining": 88,
        }, 4),
        telemetry_frame("SERVO_OUTPUT_RAW", {
            "time_usec": 1, "servo1_raw": 1100, "servo2_raw": 1200,
            "servo3_raw": 1300, "servo4_raw": 1400,
            "servo5_raw": 0, "servo6_raw": 0, "servo7_raw": 0,
            "servo8_raw": 0, "port": 0, "servo9_raw": 0,
            "servo10_raw": 0, "servo11_raw": 0, "servo12_raw": 0,
            "servo13_raw": 0, "servo14_raw": 0, "servo15_raw": 0,
            "servo16_raw": 0,
        }, 5),
        telemetry_frame("RC_CHANNELS", {
            "time_boot_ms": 1, **{f"chan{i}_raw": 1000 + i for i in range(1, 19)},
            "chancount": 8, "rssi": 90,
        }, 6),
        telemetry_frame("STATUSTEXT", {
            "severity": 6, "text": "EKF3 IMU0 is using GPS",
            "id": 7, "chunk_seq": 0,
        }, 7),
    ]
    for frame in frames:
        transport.feed(frame)

    telemetry = vehicle.telemetry(timeout=0.2)
    assert telemetry.flight_state == 1
    assert telemetry.roll_deg == pytest.approx(10.0)
    assert telemetry.pitch_deg == pytest.approx(-5.0)
    assert telemetry.yaw_deg == pytest.approx(270.0)
    assert (telemetry.latitude, telemetry.longitude) == pytest.approx(
        (47.3977, -122.7654321)
    )
    assert telemetry.altitude_m == pytest.approx(25.0)
    assert (
        telemetry.vx_north_mps,
        telemetry.vy_east_mps,
        telemetry.vz_down_mps,
    ) == pytest.approx((1.25, -2.5, 0.75))
    assert telemetry.battery_v == pytest.approx(16.8)
    assert telemetry.battery_remaining == 88
    assert telemetry.gps_fix_type == 3
    assert telemetry.gps_satellites == 14
    assert telemetry.motors == (1100, 1200, 1300, 1400)
    assert telemetry.rc[:4] == (1001, 1002, 1003, 1004)
    assert telemetry.status_text == "EKF3 IMU0 is using GPS"


def mission_item(sequence: int) -> dict[str, Any]:
    return {
        "target_system": 1, "target_component": 1, "seq": sequence,
        "frame": 6, "command": 16, "current": int(sequence == 0),
        "autocontinue": 1, "param1": float(sequence + 1),
        "param2": 0.0, "param3": 0.0, "param4": 0.0,
        "x": 473_977_000 + sequence, "y": -1_227_654_321 + sequence,
        "z": 25.0 + sequence, "mission_type": 0,
    }


class MissionTransport(MemoryTransport):
    def __init__(self, ack_type: int = 0) -> None:
        super().__init__(timeout=0)
        self.outgoing: list[bytes] = []
        self.uploaded: list[int] = []
        self.ack_type = ack_type

    def _write(self, data: bytes) -> int:
        self.outgoing.append(bytes(data))
        name, fields, _header = codec.parse(data)
        if name == "MISSION_REQUEST_LIST":
            self.feed(codec.build(
                "MISSION_COUNT",
                {"count": 2, "target_system": 255,
                 "target_component": 0, "mission_type": 0},
                system=1, component=1,
            ))
        elif name == "MISSION_REQUEST_INT":
            sequence = int(fields["seq"])
            self.feed(codec.build(
                "MISSION_ITEM_INT", mission_item(sequence),
                system=1, component=1,
            ))
        elif name == "MISSION_COUNT":
            for sequence in range(int(fields["count"])):
                self.feed(codec.build(
                    "MISSION_REQUEST_INT",
                    {"seq": sequence, "target_system": 255,
                     "target_component": 0, "mission_type": 0},
                    system=1, component=1,
                ))
        elif name == "MISSION_ITEM_INT":
            self.uploaded.append(int(fields["seq"]))
            if len(self.uploaded) == 2:
                self.feed(codec.build(
                    "MISSION_ACK",
                    {"target_system": 255, "target_component": 0,
                     "type": self.ack_type, "mission_type": 0},
                    system=1, component=1,
                ))
        return len(data)


def test_mission_download_is_read_only_and_upload_is_opt_in() -> None:
    transport = MissionTransport()
    vehicle = MavlinkVehicle(transport)
    vehicle.mav.vehicle = (1, 1)
    downloaded = vehicle.mission_download(timeout=0.1)
    assert [item.sequence for item in downloaded] == [0, 1]

    refused = MissionTransport()
    unwritable = MavlinkVehicle(refused)
    with pytest.raises(MavlinkWriteRefused):
        unwritable.mission_upload([mission_item(0), mission_item(1)])
    assert refused.outgoing == []

    ack = vehicle.mission_upload(
        [mission_item(0), mission_item(1)], allow_write=True, timeout=0.1
    )
    assert ack["type"] == 0
    assert transport.uploaded == [0, 1]

    rejected = MissionTransport(ack_type=2)
    rejecting_vehicle = MavlinkVehicle(rejected)
    rejecting_vehicle.mav.vehicle = (1, 1)
    with pytest.raises(MavlinkMissionRejected) as caught:
        rejecting_vehicle.mission_upload(
            [mission_item(0), mission_item(1)],
            allow_write=True,
            timeout=0.1,
        )
    assert caught.value.result == 2


@pytest.mark.parametrize("operation", [
    lambda vehicle: vehicle.arm(),
    lambda vehicle: vehicle.send_command_long(400, param1=1),
    lambda vehicle: vehicle.disarm(),
    lambda vehicle: vehicle.set_mode(4),
    lambda vehicle: vehicle.takeoff(20),
    lambda vehicle: vehicle.land(),
    lambda vehicle: vehicle.set_parameter("RATE_RLL_P", 1.0),
    lambda vehicle: vehicle.set_velocity_target(1, 0, 0),
    lambda vehicle: vehicle.set_position_target(47, -122, 20),
    lambda vehicle: vehicle.mission_upload([mission_item(0)]),
])
def test_every_mutation_is_refused_before_writing(operation) -> None:
    transport = MavReplyTransport()
    vehicle = MavlinkVehicle(transport)
    with pytest.raises(MavlinkWriteRefused):
        operation(vehicle)
    assert transport.outgoing == []


def representative_fields(name: str) -> dict[str, Any]:
    _msg_id, _extra, fmt, names = codec.MESSAGES[name]
    body = fmt[1:] if fmt[:1] in "<>=@!" else fmt
    shapes: list[tuple[str, int]] = []
    count = ""
    for char in body:
        if char.isdigit():
            count += char
            continue
        shapes.append((char, int(count) if count else 1))
        count = ""
    fields: dict[str, Any] = {}
    for index, field in enumerate(names):
        char, length = shapes[index]
        if field in ("param_id", "text"):
            fields[field] = (field.encode() + b"_test")[:length]
        elif length > 1:
            fields[field] = list(range(1, length + 1))
        elif char in "fd":
            fields[field] = 1.25 + index
        else:
            fields[field] = index + 1
    return fields


def same_frame(mine: bytes, theirs: bytes) -> bool:
    head = 10 if mine[0] == codec.MAGIC_V2 else 6
    if mine[0] != theirs[0] or mine[2:head] != theirs[2:head]:
        return False
    return (
        mine[head:head + mine[1]].rstrip(b"\x00")
        == theirs[head:head + theirs[1]].rstrip(b"\x00")
    )


def normalize_fields(fields: dict[str, Any]) -> dict[str, Any]:
    output = {}
    for key, value in fields.items():
        if key == "mavpackettype":
            continue
        if isinstance(value, bytes):
            value = value.split(b"\x00", 1)[0].decode("utf-8", "replace")
        elif isinstance(value, tuple):
            value = list(value)
        output[key] = value
    return output


def test_codec_matches_pymavlink_for_every_supported_message_both_ways() -> None:
    common = pytest.importorskip("pymavlink.dialects.v20.common")
    mav = common.MAVLink(None)
    mav.srcSystem = codec.GCS_SYSTEM
    mav.srcComponent = codec.GCS_COMPONENT
    mav.robust_parsing = True

    failures: list[str] = []
    for name in sorted(codec.MESSAGES):
        cls = getattr(common, f"MAVLink_{name.lower()}_message")
        fields = representative_fields(name)
        ours = codec.build(name, fields)
        theirs = cls(**fields).pack(mav)
        _ours_name, ours_fields, _header = codec.parse(theirs)
        packed = mav.parse_buffer(ours)
        if not same_frame(ours, theirs):
            failures.append(f"{name}: frames differ")
            continue
        if normalize_fields(ours_fields) != normalize_fields(
            packed[0].to_dict()
        ):
            failures.append(f"{name}: decoded fields differ")
        if cls.id != codec.MESSAGES[name][0]:
            failures.append(f"{name}: id")
        if cls.crc_extra != codec.MESSAGES[name][1]:
            failures.append(f"{name}: crc_extra")
        if cls.unpacker.format != codec.MESSAGES[name][2]:
            failures.append(f"{name}: format")
        if tuple(cls.ordered_fieldnames) != tuple(codec.MESSAGES[name][3]):
            failures.append(f"{name}: field order")
    assert failures == []


def test_optional_full_dialect_encodes_messages_outside_the_core_set() -> None:
    pytest.importorskip("pymavlink")
    dialect = PymavlinkDialect()
    frame = dialect.encode(
        "BATTERY_STATUS",
        {
            "id": 1, "battery_function": 0, "type": 0,
            "temperature": 300, "voltages": [3700, 3700, 3700, 0,
                                             0, 0, 0, 0, 0, 0],
            "current_battery": 1200, "current_consumed": 100,
            "energy_consumed": 0, "battery_remaining": 80,
            "time_remaining": 0, "charge_state": 0,
            "voltages_ext": [0, 0, 0, 0], "mode": 0, "fault_bitmask": 0,
        },
    )
    name, fields, header = dialect.decode(frame)
    assert name == "BATTERY_STATUS"
    assert fields["battery_remaining"] == 80
    assert header["version"] == 2
    assert "BATTERY_STATUS" in dialect.message_types


FAKE_VEHICLE = Path(__file__).parent / "firmware" / "tools" / "mavlink_fake_vehicle.py"


def load_fake_vehicle() -> ModuleType:
    pytest.importorskip("pymavlink")
    spec = importlib.util.spec_from_file_location(
        "aerial_kit_test_mavlink_fake_vehicle", FAKE_VEHICLE
    )
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class FakeMemoryTransport(MemoryTransport):
    def __init__(self, fake: Any, parser: Any) -> None:
        super().__init__(timeout=0)
        self.fake = fake
        self.parser = parser

    def _write(self, data: bytes) -> int:
        for message in self.parser.parse_buffer(data):
            self.fake.handle(message)
        self.fake.tick(time.time())
        return len(data)


def test_fake_vehicle_over_memory() -> None:
    fake_module = load_fake_vehicle()
    common = pytest.importorskip("pymavlink.dialects.v20.common")
    parser = common.MAVLink(None)
    parser.robust_parsing = True
    transport = FakeMemoryTransport(None, parser)  # type: ignore[arg-type]
    fake = fake_module.Vehicle(output=transport.feed)
    transport.fake = fake
    vehicle = MavlinkVehicle(transport, timeout=0.1)

    vehicle.send_heartbeat()
    identity = vehicle.identify()
    assert identity.autopilot == 3
    assert identity.system_id == 1
    assert identity.component_id == 1

    parameters = vehicle.parameters(timeout=0.5)
    assert len(parameters) == 40
    assert parameters[0].name == "RATE_RLL_P"

    vehicle.request("ATTITUDE", 50)
    telemetry = vehicle.telemetry(timeout=0.2)
    assert telemetry.link_live == 1
    assert telemetry.roll_deg is not None
    assert telemetry.gps_satellites == 11


def free_udp_port() -> int:
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("127.0.0.1", 0))
    port = int(sock.getsockname()[1])
    sock.close()
    return port


def test_fake_vehicle_over_udp() -> None:
    pytest.importorskip("pymavlink")
    port = free_udp_port()
    process = subprocess.Popen(
        [sys.executable, str(FAKE_VEHICLE), "--udp-port", str(port)],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )
    try:
        assert process.stdout is not None
        assert process.stdout.readline().startswith(b"ready ")
        transport = UdpTransport.connect("127.0.0.1", port, timeout=0.05)
        vehicle = MavlinkVehicle(transport, timeout=0.2)
        vehicle.send_heartbeat()
        identity = vehicle.identify()
        assert identity.autopilot == 3
        vehicle.request("ATTITUDE", 50)
        telemetry = vehicle.telemetry(timeout=1.0)
        assert telemetry.link_live == 1
        assert telemetry.roll_deg is not None
    finally:
        process.terminate()
        try:
            process.wait(timeout=2)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait(timeout=2)
