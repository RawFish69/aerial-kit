"""Typed, opt-in MAVLink vehicle client."""

from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Any, Mapping, Sequence, cast

from ..transport import Transport
from . import codec


class MavlinkError(RuntimeError):
    """Base class for MAVLink client failures."""


class MavlinkWriteRefused(MavlinkError):
    """A mutation was not explicitly enabled."""


class MavlinkTimeout(codec.Timeout, MavlinkError):
    """The vehicle did not answer a request in time."""


class MavlinkCommandRejected(MavlinkError):
    """The vehicle explicitly refused a MAVLink command."""

    def __init__(self, command: int, result: int, fields: Mapping[str, Any]) -> None:
        self.command = int(command)
        self.result = int(result)
        self.fields = dict(fields)
        super().__init__(
            f"vehicle rejected command {self.command} with result {self.result}"
        )


class MavlinkMissionRejected(MavlinkError):
    """The vehicle refused a complete mission upload."""

    def __init__(self, result: int, fields: Mapping[str, Any]) -> None:
        self.result = int(result)
        self.fields = dict(fields)
        super().__init__(f"mission rejected with MAV_MISSION_RESULT {self.result}")

@dataclass(frozen=True)
class MavlinkIdentity:
    autopilot: int
    vehicle_type: int
    system_id: int
    component_id: int
    system_status: int
    version: Mapping[str, Any] | None


@dataclass(frozen=True)
class MavlinkParameter:
    name: str
    value: float
    param_type: int
    index: int
    count: int


@dataclass(frozen=True)
class MavlinkMissionItem:
    target_system: int
    target_component: int
    sequence: int
    frame: int
    command: int
    current: int
    autocontinue: int
    param1: float
    param2: float
    param3: float
    param4: float
    x: int
    y: int
    z: float
    mission_type: int

    @classmethod
    def from_wire(
        cls, fields: Mapping[str, Any]
    ) -> "MavlinkMissionItem":
        return cls(
            target_system=int(fields["target_system"]),
            target_component=int(fields["target_component"]),
            sequence=int(fields["seq"]),
            frame=int(fields["frame"]),
            command=int(fields["command"]),
            current=int(fields["current"]),
            autocontinue=int(fields["autocontinue"]),
            param1=float(fields["param1"]),
            param2=float(fields["param2"]),
            param3=float(fields["param3"]),
            param4=float(fields["param4"]),
            x=int(fields["x"]),
            y=int(fields["y"]),
            z=float(fields["z"]),
            mission_type=int(fields["mission_type"]),
        )


@dataclass(frozen=True)
class MavlinkTelemetry:
    flight_state: int | None
    link_live: int
    gps_fix_type: int | None
    gps_satellites: int | None
    latitude: float | None
    longitude: float | None
    altitude_m: float | None
    roll_deg: float | None
    pitch_deg: float | None
    yaw_deg: float | None
    battery_v: float | None
    battery_remaining: int | None
    vx_north_mps: float | None
    vy_east_mps: float | None
    vz_down_mps: float | None
    status_text: str | None
    motors: tuple[int, ...]
    rc: tuple[int, ...]


class MavlinkVehicle:
    """MAVLink v1/v2 client with explicit mutation permission."""

    def __init__(
        self,
        transport: Transport,
        *,
        allow_write: bool = False,
        timeout: float = 2.0,
        system: int = codec.GCS_SYSTEM,
        component: int = codec.GCS_COMPONENT,
    ) -> None:
        self.transport = transport
        self.allow_write = bool(allow_write)
        self.mav = codec.Mavlink(
            lambda size, wait: transport.read(size, wait),
            transport.write,
            system=system,
            component=component,
            timeout=timeout,
        )

    def close(self) -> None:
        self.transport.close()

    def identify(self) -> MavlinkIdentity:
        data = self.mav.identify()
        return MavlinkIdentity(
            autopilot=data["autopilot"],
            vehicle_type=data["type"],
            system_id=data["system"],
            component_id=data["component"],
            system_status=data["system_status"],
            version=data["version"],
        )

    def request(self, message: str, hz: float) -> None:
        self.mav.request(message, hz)

    def parameters(self, timeout: float = 30.0) -> tuple[MavlinkParameter, ...]:
        data = self.mav.parameters(timeout=timeout)
        return tuple(
            MavlinkParameter(
                name=str(name),
                value=float(value),
                param_type=int(param_type),
                index=index,
                count=int(data["count"] or 0),
            )
            for index, name, value, param_type in data["indexed"]
        )

    def parameter(
        self, name: str, timeout: float = 30.0
    ) -> MavlinkParameter:
        """Read one named parameter from the vehicle's current table."""
        for item in self.parameters(timeout=timeout):
            if item.name == name:
                return item
        raise KeyError(f"vehicle does not expose parameter {name!r}")

    def telemetry(self, timeout: float = 2.0) -> MavlinkTelemetry:
        data = codec.state(self.mav, timeout=timeout)
        return MavlinkTelemetry(
            flight_state=data["flight_state"],
            link_live=data["link_live"],
            gps_fix_type=data["gps_fix_type"],
            gps_satellites=data["gps_satellites"],
            latitude=data["lat"],
            longitude=data["lon"],
            altitude_m=data["alt_m"],
            roll_deg=data["roll_deg"],
            pitch_deg=data["pitch_deg"],
            yaw_deg=data["yaw_deg"],
            battery_v=data["vbat_v"],
            battery_remaining=data.get("battery_remaining"),
            vx_north_mps=data["vx_north_mps"],
            vy_east_mps=data["vy_east_mps"],
            vz_down_mps=data["vz_down_mps"],
            status_text=data["status_text"],
            motors=tuple(data["motors"]),
            rc=tuple(data["rc"]),
        )

    def _require_write(self) -> None:
        if not self.allow_write:
            raise MavlinkWriteRefused("MAVLink mutations require allow_write=True")

    def command_long(
        self,
        command: int,
        *,
        param1: float = 0.0,
        param2: float = 0.0,
        param3: float = 0.0,
        param4: float = 0.0,
        param5: float = 0.0,
        param6: float = 0.0,
        param7: float = 0.0,
        allow_write: bool | None = None,
    ) -> Mapping[str, Any]:
        if allow_write is None:
            allow_write = self.allow_write
        if not allow_write:
            raise MavlinkWriteRefused("MAVLink COMMAND_LONG requires allow_write=True")
        self.send_command_long(
            command,
            param1=param1,
            param2=param2,
            param3=param3,
            param4=param4,
            param5=param5,
            param6=param6,
            param7=param7,
            allow_write=True,
        )
        deadline = time.monotonic() + self.mav.timeout
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise MavlinkTimeout(f"no COMMAND_ACK for command {command}")
            name, fields, header = self.mav.wait(
                "COMMAND_ACK", remaining,
                predicate=lambda values: values["command"] == command,
            )
            result = int(fields["result"])
            if result == 5:  # MAV_RESULT_IN_PROGRESS
                continue
            if result != 0:
                raise MavlinkCommandRejected(command, result, fields)
            break
        return {"name": name, "fields": fields, "header": header}

    def send_command_long(
        self,
        command: int,
        *,
        param1: float = 0.0,
        param2: float = 0.0,
        param3: float = 0.0,
        param4: float = 0.0,
        param5: float = 0.0,
        param6: float = 0.0,
        param7: float = 0.0,
        allow_write: bool | None = None,
    ) -> None:
        """Send a command without consuming its asynchronous acknowledgement."""
        if allow_write is None:
            allow_write = self.allow_write
        if not allow_write:
            raise MavlinkWriteRefused("MAVLink COMMAND_LONG requires allow_write=True")
        if self.mav.vehicle is None:
            self.identify()
        system, component = self.mav.vehicle
        self.mav.send("COMMAND_LONG", {
            "param1": param1, "param2": param2, "param3": param3,
            "param4": param4, "param5": param5, "param6": param6,
            "param7": param7, "command": command,
            "target_system": system, "target_component": component,
            "confirmation": 0,
        })

    def set_parameter(
        self,
        name: str,
        value: float,
        *,
        param_type: int = 9,
        allow_write: bool | None = None,
    ) -> MavlinkParameter | None:
        if allow_write is None:
            allow_write = self.allow_write
        if not allow_write:
            raise MavlinkWriteRefused("MAVLink parameter writes require allow_write=True")
        if self.mav.vehicle is None:
            self.identify()
        system, component = self.mav.vehicle
        known = {item.name for item in self.parameters()}
        if name not in known:
            raise MavlinkWriteRefused(f"vehicle does not expose parameter {name!r}")
        self.mav.send("PARAM_SET", {
            "param_value": float(value),
            "target_system": system,
            "target_component": component,
            "param_id": name.encode("ascii")[:16],
            "param_type": param_type,
        })
        try:
            _name, fields, _header = self.mav.wait(
                "PARAM_VALUE", self.mav.timeout,
                predicate=lambda item: item["param_id"] == name,
            )
        except codec.Timeout:
            return None
        return MavlinkParameter(
            name=fields["param_id"],
            value=fields["param_value"],
            param_type=fields["param_type"],
            index=fields["param_index"],
            count=fields["param_count"],
        )

    def arm(self, *, allow_write: bool | None = None) -> Mapping[str, Any]:
        return self.command_long(400, param1=1.0, allow_write=allow_write)

    def disarm(self, *, allow_write: bool | None = None) -> Mapping[str, Any]:
        return self.command_long(400, param1=0.0, allow_write=allow_write)

    def set_mode(self, mode: int, *, allow_write: bool | None = None) -> Mapping[str, Any]:
        return self.command_long(176, param1=float(mode), allow_write=allow_write)

    def takeoff(self, altitude_m: float, *, allow_write: bool | None = None) -> Mapping[str, Any]:
        return self.command_long(22, param7=altitude_m, allow_write=allow_write)

    def land(self, *, allow_write: bool | None = None) -> Mapping[str, Any]:
        return self.command_long(21, allow_write=allow_write)

    def set_velocity_target(
        self,
        vx: float,
        vy: float,
        vz: float,
        yaw_rate: float = 0.0,
        *,
        coordinate_frame: int = 1,
        allow_write: bool | None = None,
    ) -> None:
        if allow_write is None:
            allow_write = self.allow_write
        if not allow_write:
            raise MavlinkWriteRefused("MAVLink setpoints require allow_write=True")
        if self.mav.vehicle is None:
            self.identify()
        system, component = self.mav.vehicle
        # Ignore position, acceleration and yaw angle; keep velocity and yaw rate.
        type_mask = 0x05C7
        self.mav.send("SET_POSITION_TARGET_LOCAL_NED", {
            "time_boot_ms": 0, "target_system": system,
            "target_component": component,
            "coordinate_frame": int(coordinate_frame),
            "type_mask": type_mask, "vx": float(vx), "vy": float(vy),
            "vz": float(vz), "yaw_rate": float(yaw_rate),
        })

    def set_position_target(
        self,
        latitude_deg: float,
        longitude_deg: float,
        altitude_m: float,
        *,
        coordinate_frame: int = 6,
        allow_write: bool | None = None,
    ) -> None:
        if allow_write is None:
            allow_write = self.allow_write
        if not allow_write:
            raise MavlinkWriteRefused("MAVLink setpoints require allow_write=True")
        if self.mav.vehicle is None:
            self.identify()
        system, component = self.mav.vehicle
        self.mav.send("SET_POSITION_TARGET_GLOBAL_INT", {
            "time_boot_ms": 0, "target_system": system,
            "target_component": component,
            "coordinate_frame": int(coordinate_frame),
            "type_mask": 0x0DF8,
            "lat_int": round(latitude_deg * 1e7),
            "lon_int": round(longitude_deg * 1e7),
            "alt": float(altitude_m),
        })

    def mission_download(
        self, *, timeout: float = 10.0
    ) -> tuple[MavlinkMissionItem, ...]:
        if self.mav.vehicle is None:
            self.identify()
        system, component = self.mav.vehicle
        self.mav.send("MISSION_REQUEST_LIST", {
            "target_system": system, "target_component": component,
            "mission_type": 0,
        })
        _name, count_fields, _ = self.mav.wait("MISSION_COUNT", timeout)
        count = int(count_fields["count"])
        items: list[MavlinkMissionItem] = []
        for sequence in range(count):
            self.mav.send("MISSION_REQUEST_INT", {
                "seq": sequence, "target_system": system,
                "target_component": component, "mission_type": 0,
            })
            _name, fields, _ = self.mav.wait("MISSION_ITEM_INT", timeout)
            items.append(MavlinkMissionItem.from_wire(fields))
        return tuple(items)

    def mission_upload(
        self,
        items: Sequence[Mapping[str, Any]],
        *,
        allow_write: bool = False,
        timeout: float = 10.0,
    ) -> Mapping[str, Any]:
        if not allow_write:
            raise MavlinkWriteRefused("mission operations require allow_write=True")
        if self.mav.vehicle is None:
            self.identify()
        system, component = self.mav.vehicle
        count = len(items)
        self.mav.send("MISSION_COUNT", {
            "count": count, "target_system": system,
            "target_component": component, "mission_type": 0,
        })
        for sequence in range(count):
            _name, request, _ = self.mav.wait("MISSION_REQUEST_INT", timeout)
            item = dict(items[sequence])
            item.update({
                "target_system": system, "target_component": component,
                "seq": int(request["seq"]), "mission_type": 0,
            })
            self.mav.send("MISSION_ITEM_INT", item)
        _name, ack, _ = self.mav.wait("MISSION_ACK", timeout)
        if int(ack["type"]) != 0:
            raise MavlinkMissionRejected(int(ack["type"]), ack)
        return cast(Mapping[str, Any], ack)

    def send_heartbeat(self) -> None:
        self.mav.send_heartbeat()
