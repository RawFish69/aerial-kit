"""Pure-Python CRSF framing, RC channels, telemetry and transport helpers.

The codec deliberately has no ROS or hardware dependency. ``SerialTransport``
imports pyserial only when constructed, so importing this module is safe on a
machine with no serial devices.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable, Sequence

import math
import struct

from ..transport import Transport

CRSF_BAUD = 400_000
CRSF_RX_FC_BAUD = 420_000
CRSF_MAX_FRAME_LEN = 64
CRSF_CHANNEL_COUNT = 16
CRSF_CHANNEL_MIN = 172
CRSF_CHANNEL_MID = 992
CRSF_CHANNEL_MAX = 1811

CRSF_ADDRESS_BROADCAST = 0x00
CRSF_ADDRESS_FLIGHT_CONTROLLER = 0xC8
CRSF_ADDRESS_RADIO_TRANSMITTER = 0xEA
CRSF_ADDRESS_RECEIVER = 0xEC

CRSF_FRAMETYPE_GPS = 0x02
CRSF_FRAMETYPE_BARO_ALTITUDE = 0x09
CRSF_FRAMETYPE_BATTERY_SENSOR = 0x08
CRSF_FRAMETYPE_LINK_STATISTICS = 0x14
CRSF_FRAMETYPE_RC_CHANNELS_PACKED = 0x16
CRSF_FRAMETYPE_ATTITUDE = 0x1E
CRSF_FRAMETYPE_FLIGHT_MODE = 0x21

_VALID_ADDRESSES = {
    CRSF_ADDRESS_BROADCAST,
    CRSF_ADDRESS_FLIGHT_CONTROLLER,
    CRSF_ADDRESS_RADIO_TRANSMITTER,
    CRSF_ADDRESS_RECEIVER,
}


def crc8_dvb_s2(data: bytes | bytearray | memoryview) -> int:
    """CRC-8/DVB-S2: polynomial 0xD5, initial value zero."""
    crc = 0
    for value in data:
        crc ^= value
        for _ in range(8):
            crc = ((crc << 1) ^ 0xD5) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc


def _clamp_channel(value: int) -> int:
    return max(CRSF_CHANNEL_MIN, min(CRSF_CHANNEL_MAX, int(value)))


def pack_channels(channels: Sequence[int]) -> bytes:
    """Pack exactly 16 CRSF 11-bit channels into 22 payload bytes."""
    if len(channels) != CRSF_CHANNEL_COUNT:
        raise ValueError(f"CRSF requires {CRSF_CHANNEL_COUNT} channels, got {len(channels)}")

    output = bytearray(22)
    accumulator = 0
    bits = 0
    offset = 0
    for channel in channels:
        accumulator |= _clamp_channel(channel) << bits
        bits += 11
        while bits >= 8:
            output[offset] = accumulator & 0xFF
            offset += 1
            accumulator >>= 8
            bits -= 8
    return bytes(output)


def unpack_channels(payload: bytes | bytearray | memoryview) -> tuple[int, ...]:
    """Unpack the 22-byte RC payload into 16 unsigned 11-bit channels."""
    if len(payload) != 22:
        raise ValueError(f"CRSF RC payload must be 22 bytes, got {len(payload)}")

    channels: list[int] = []
    accumulator = 0
    bits = 0
    offset = 0
    for _ in range(CRSF_CHANNEL_COUNT):
        while bits < 11:
            accumulator |= int(payload[offset]) << bits
            offset += 1
            bits += 8
        channels.append(accumulator & 0x7FF)
        accumulator >>= 11
        bits -= 11
    return tuple(channels)


@dataclass(frozen=True)
class CrsfFrame:
    address: int
    frame_type: int
    payload: bytes

    def encode(self) -> bytes:
        body = bytes((self.frame_type,)) + self.payload
        return bytes((self.address, len(body) + 1)) + body + bytes((crc8_dvb_s2(body),))

    @property
    def raw(self) -> bytes:
        return self.encode()


def make_frame(
    frame_type: int,
    payload: bytes,
    *,
    address: int = CRSF_ADDRESS_FLIGHT_CONTROLLER,
) -> bytes:
    return CrsfFrame(address=address, frame_type=frame_type, payload=bytes(payload)).encode()


def make_rc_channels_frame(
    channels: Sequence[int],
    *,
    address: int = CRSF_ADDRESS_FLIGHT_CONTROLLER,
) -> bytes:
    return make_frame(CRSF_FRAMETYPE_RC_CHANNELS_PACKED, pack_channels(channels), address=address)


class CrsfParser:
    """Incremental byte-stream parser that survives partial data and bad CRCs."""

    def __init__(self) -> None:
        self._buffer = bytearray()

    def reset(self) -> None:
        self._buffer.clear()

    def feed(self, data: bytes | bytearray | memoryview) -> list[CrsfFrame]:
        self._buffer.extend(data)
        frames: list[CrsfFrame] = []

        while len(self._buffer) >= 2:
            if self._buffer[0] not in _VALID_ADDRESSES:
                del self._buffer[0]
                continue

            length = self._buffer[1]
            total = length + 2
            if length < 2 or total > CRSF_MAX_FRAME_LEN:
                del self._buffer[0]
                continue
            if len(self._buffer) < total:
                break

            raw = bytes(self._buffer[:total])
            body = raw[2:-1]
            if crc8_dvb_s2(body) != raw[-1]:
                del self._buffer[0]
                continue

            frames.append(CrsfFrame(address=raw[0], frame_type=raw[2], payload=raw[3:-1]))
            del self._buffer[:total]

        return frames


@dataclass(frozen=True)
class LinkStatisticsTelemetry:
    uplink_rssi_1_dbm: int
    uplink_rssi_2_dbm: int
    uplink_link_quality: int
    uplink_snr_db: int
    active_antenna: int
    rf_mode: int
    uplink_tx_power: int
    downlink_rssi_dbm: int
    downlink_link_quality: int
    downlink_snr_db: int

    @property
    def rssi_dbm(self) -> int:
        return max(self.uplink_rssi_1_dbm, self.uplink_rssi_2_dbm)


@dataclass(frozen=True)
class AttitudeTelemetry:
    roll_rad: float
    pitch_rad: float
    yaw_rad: float


@dataclass(frozen=True)
class BatteryTelemetry:
    voltage_v: float
    current_a: float
    capacity_mah: int
    remaining_percent: int


@dataclass(frozen=True)
class BarometerTelemetry:
    """Barometric altitude and vertical speed."""

    altitude_m: float
    vertical_speed_mps: float


@dataclass(frozen=True)
class GpsTelemetry:
    latitude_deg: float
    longitude_deg: float
    groundspeed_mps: float
    heading_deg: float
    altitude_m: float
    satellites: int


@dataclass(frozen=True)
class FlightModeTelemetry:
    mode: str


def make_link_statistics_frame(stats: LinkStatisticsTelemetry) -> bytes:
    payload = bytes(
        (
            max(0, min(255, -stats.uplink_rssi_1_dbm)),
            max(0, min(255, -stats.uplink_rssi_2_dbm)),
            max(0, min(100, stats.uplink_link_quality)),
            stats.uplink_snr_db & 0xFF,
            max(0, min(3, stats.active_antenna)),
            max(0, min(255, stats.rf_mode)),
            max(0, min(255, stats.uplink_tx_power)),
            max(0, min(255, -stats.downlink_rssi_dbm)),
            max(0, min(100, stats.downlink_link_quality)),
            stats.downlink_snr_db & 0xFF,
        )
    )
    return make_frame(CRSF_FRAMETYPE_LINK_STATISTICS, payload, address=CRSF_ADDRESS_RADIO_TRANSMITTER)


def make_attitude_frame(attitude: AttitudeTelemetry) -> bytes:
    payload = struct.pack(
        ">hhh",
        round(attitude.pitch_rad * 10_000.0),
        round(attitude.roll_rad * 10_000.0),
        round(attitude.yaw_rad * 10_000.0),
    )
    return make_frame(CRSF_FRAMETYPE_ATTITUDE, payload, address=CRSF_ADDRESS_RADIO_TRANSMITTER)


def make_battery_frame(battery: BatteryTelemetry) -> bytes:
    capacity = max(0, min(0xFFFFFF, int(battery.capacity_mah)))
    payload = struct.pack(
        ">HH",
        max(0, min(0xFFFF, round(battery.voltage_v * 10.0))),
        max(0, min(0xFFFF, round(battery.current_a * 10.0))),
    )
    payload += capacity.to_bytes(3, "big")
    payload += bytes((max(0, min(100, battery.remaining_percent)),))
    return make_frame(CRSF_FRAMETYPE_BATTERY_SENSOR, payload, address=CRSF_ADDRESS_RADIO_TRANSMITTER)


def make_barometer_frame(barometer: BarometerTelemetry) -> bytes:
    """Encode altitude in decimetres and vertical speed in centimetres/second."""
    altitude_dm = round(barometer.altitude_m * 10.0) + 10_000
    if 0 <= altitude_dm <= 0xFFFF:
        altitude = altitude_dm
    else:
        altitude = 0x8000 | max(0, min(0x7FFF, round(barometer.altitude_m)))
    vertical_cms = max(-0x8000, min(0x7FFF, round(barometer.vertical_speed_mps * 100.0)))
    payload = struct.pack(">Hh", altitude, vertical_cms)
    return make_frame(CRSF_FRAMETYPE_BARO_ALTITUDE, payload, address=CRSF_ADDRESS_RADIO_TRANSMITTER)


def make_gps_frame(gps: GpsTelemetry) -> bytes:
    payload = struct.pack(
        ">iiHHHB",
        round(gps.latitude_deg * 1e7),
        round(gps.longitude_deg * 1e7),
        max(0, min(0xFFFF, round(gps.groundspeed_mps * 36.0))),
        max(0, min(0xFFFF, round(gps.heading_deg * 100.0) % 36000)),
        max(0, min(0xFFFF, round(gps.altitude_m + 1000.0))),
        max(0, min(255, gps.satellites)),
    )
    return make_frame(CRSF_FRAMETYPE_GPS, payload, address=CRSF_ADDRESS_RADIO_TRANSMITTER)


def make_flight_mode_frame(flight_mode: FlightModeTelemetry) -> bytes:
    return make_frame(
        CRSF_FRAMETYPE_FLIGHT_MODE,
        flight_mode.mode.encode("ascii", "replace") + b"\0",
        address=CRSF_ADDRESS_RADIO_TRANSMITTER,
    )


def decode_telemetry(
    frame: CrsfFrame,
) -> (
    LinkStatisticsTelemetry
    | AttitudeTelemetry
    | BarometerTelemetry
    | BatteryTelemetry
    | GpsTelemetry
    | FlightModeTelemetry
    | None
):
    """Decode a telemetry frame, returning ``None`` for unrelated frame types."""
    payload = frame.payload

    if frame.frame_type == CRSF_FRAMETYPE_LINK_STATISTICS:
        if len(payload) < 10:
            raise ValueError("short CRSF link-statistics frame")
        return LinkStatisticsTelemetry(
            uplink_rssi_1_dbm=-payload[0],
            uplink_rssi_2_dbm=-payload[1],
            uplink_link_quality=payload[2],
            uplink_snr_db=struct.unpack("b", payload[3:4])[0],
            active_antenna=payload[4],
            rf_mode=payload[5],
            uplink_tx_power=payload[6],
            downlink_rssi_dbm=-payload[7],
            downlink_link_quality=payload[8],
            downlink_snr_db=struct.unpack("b", payload[9:10])[0],
        )

    if frame.frame_type == CRSF_FRAMETYPE_ATTITUDE:
        if len(payload) < 6:
            raise ValueError("short CRSF attitude frame")
        pitch, roll, yaw = struct.unpack(">hhh", payload[:6])
        return AttitudeTelemetry(roll / 10_000.0, pitch / 10_000.0, yaw / 10_000.0)

    if frame.frame_type == CRSF_FRAMETYPE_BATTERY_SENSOR:
        if len(payload) < 8:
            raise ValueError("short CRSF battery frame")
        voltage_dv, current_da = struct.unpack(">HH", payload[:4])
        capacity = int.from_bytes(payload[4:7], "big")
        remaining = payload[7]
        return BatteryTelemetry(
            voltage_v=voltage_dv / 10.0,
            current_a=current_da / 10.0,
            capacity_mah=capacity,
            remaining_percent=remaining,
        )

    if frame.frame_type == CRSF_FRAMETYPE_BARO_ALTITUDE:
        if len(payload) < 4:
            raise ValueError("short CRSF barometer frame")
        altitude, vertical_cms = struct.unpack(">Hh", payload[:4])
        altitude_m = float(altitude & 0x7FFF) if altitude & 0x8000 else (altitude - 10_000) / 10.0
        return BarometerTelemetry(
            altitude_m=altitude_m,
            vertical_speed_mps=vertical_cms / 100.0,
        )

    if frame.frame_type == CRSF_FRAMETYPE_GPS:
        if len(payload) < 15:
            raise ValueError("short CRSF GPS frame")
        lat, lon, speed_kmh_x10, heading_x100, altitude_offset, satellites = struct.unpack(
            ">iiHHHB", payload[:15]
        )
        return GpsTelemetry(
            latitude_deg=lat / 1e7,
            longitude_deg=lon / 1e7,
            groundspeed_mps=speed_kmh_x10 / 36.0,
            heading_deg=(heading_x100 / 100.0) % 360.0,
            altitude_m=float(altitude_offset) - 1000.0,
            satellites=satellites,
        )

    if frame.frame_type == CRSF_FRAMETYPE_FLIGHT_MODE:
        return FlightModeTelemetry(payload.split(b"\0", 1)[0].decode("ascii", "replace"))

    return None


class CrsfLink:
    """Frame-oriented helper over any transport implementing ``Transport``."""

    def __init__(self, transport: Transport) -> None:
        self.transport = transport
        self.parser = CrsfParser()

    def send_channels(self, channels: Sequence[int]) -> None:
        self.transport.write(make_rc_channels_frame(channels))

    def send_frame(self, frame: CrsfFrame) -> None:
        self.transport.write(frame.encode())

    def poll(self, size: int = 512) -> list[CrsfFrame]:
        return self.parser.feed(self.transport.read(size))

    def poll_telemetry(self, size: int = 512) -> list[object]:
        output: list[object] = []
        for frame in self.poll(size):
            decoded = decode_telemetry(frame)
            if decoded is not None:
                output.append(decoded)
        return output

    def close(self) -> None:
        self.transport.close()

    @property
    def available(self) -> int:
        return getattr(self.transport, "available", 0)


def channels_from_normalized(values: Iterable[float]) -> tuple[int, ...]:
    """Map normalized controls in [-1, 1] to CRSF counts."""
    channels: list[int] = []
    for value in values:
        value = max(-1.0, min(1.0, float(value)))
        channels.append(round(CRSF_CHANNEL_MID + value * (CRSF_CHANNEL_MAX - CRSF_CHANNEL_MID)))
    if len(channels) != CRSF_CHANNEL_COUNT:
        raise ValueError(f"expected {CRSF_CHANNEL_COUNT} controls, got {len(channels)}")
    return tuple(channels)
