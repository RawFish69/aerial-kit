import math

import pytest

from aerial_kit.comms.crsf import (
    AttitudeTelemetry,
    BatteryTelemetry,
    CrsfFrame,
    CrsfLink,
    CrsfParser,
    FlightModeTelemetry,
    GpsTelemetry,
    LinkStatisticsTelemetry,
    MemoryTransport,
    crc8_dvb_s2,
    decode_telemetry,
    make_attitude_frame,
    make_battery_frame,
    make_flight_mode_frame,
    make_gps_frame,
    make_link_statistics_frame,
    make_rc_channels_frame,
    pack_channels,
    unpack_channels,
)


def test_pack_unpack_known_channel_sequence():
    channels = (172, 1811, 992, 1000, 1100, 1200, 1300, 1400,
                1500, 1600, 1700, 1800, 172, 511, 1023, 1811)
    payload = pack_channels(channels)
    assert len(payload) == 22
    assert unpack_channels(payload) == channels


def test_known_rc_frame_layout_and_crc():
    channels = (992,) * 16
    raw = make_rc_channels_frame(channels)
    assert raw[0] == 0xC8
    assert raw[1] == 24  # type + 22 payload + CRC
    assert raw[2] == 0x16
    assert len(raw) == 26
    assert raw[-1] == crc8_dvb_s2(raw[2:-1])


def test_parser_handles_arbitrary_partial_chunks_and_garbage():
    channels = tuple(172 + index * 100 for index in range(16))
    raw = b"\x55\x01" + make_rc_channels_frame(channels) + b"\xff"
    parser = CrsfParser()
    frames = []
    for byte in raw:
        frames.extend(parser.feed(bytes((byte,))))
    assert len(frames) == 1
    assert frames[0].frame_type == 0x16
    assert unpack_channels(frames[0].payload) == channels


def test_parser_rejects_bad_crc_and_resynchronises():
    good = make_rc_channels_frame((992,) * 16)
    bad = bytearray(good)
    bad[-1] ^= 0x01
    parser = CrsfParser()
    frames = parser.feed(bytes(bad) + good)
    assert len(frames) == 1
    assert unpack_channels(frames[0].payload) == (992,) * 16


def test_parser_waits_for_short_frame():
    raw = make_rc_channels_frame((992,) * 16)
    parser = CrsfParser()
    assert parser.feed(raw[:8]) == []
    assert len(parser.feed(raw[8:])) == 1


def test_telemetry_round_trips():
    cases = [
        LinkStatisticsTelemetry(-90, -92, 88, 7, 1, 2, 3, -80, 70, 4),
        AttitudeTelemetry(math.radians(12.5), math.radians(-3.0), math.radians(179.0)),
        BatteryTelemetry(12.6, 8.4, 123456, 73),
        GpsTelemetry(37.1234567, -122.7654321, 4.5, 271.25, 123.4, 17),
        FlightModeTelemetry("ANGLE"),
    ]
    encoders = [
        make_link_statistics_frame,
        make_attitude_frame,
        make_battery_frame,
        make_gps_frame,
        make_flight_mode_frame,
    ]
    for value, encoder in zip(cases, encoders):
        frame = CrsfParser().feed(encoder(value))[0]
        decoded = decode_telemetry(frame)
        assert isinstance(decoded, type(value))
        if isinstance(value, AttitudeTelemetry):
            assert decoded.roll_rad == pytest.approx(value.roll_rad, abs=1e-3)
            assert decoded.pitch_rad == pytest.approx(value.pitch_rad, abs=1e-3)
            assert decoded.yaw_rad == pytest.approx(value.yaw_rad, abs=1e-3)
        elif isinstance(value, BatteryTelemetry):
            assert decoded.voltage_v == pytest.approx(value.voltage_v, abs=0.05)
            assert decoded.current_a == pytest.approx(value.current_a, abs=0.05)
            assert decoded.capacity_mah == value.capacity_mah
            assert decoded.remaining_percent == value.remaining_percent
        elif isinstance(value, GpsTelemetry):
            assert decoded.latitude_deg == pytest.approx(value.latitude_deg, abs=1e-7)
            assert decoded.longitude_deg == pytest.approx(value.longitude_deg, abs=1e-7)
            assert decoded.groundspeed_mps == pytest.approx(value.groundspeed_mps, abs=0.01)
            assert decoded.heading_deg == pytest.approx(value.heading_deg, abs=0.01)
            assert decoded.altitude_m == pytest.approx(value.altitude_m, abs=0.6)
            assert decoded.satellites == value.satellites
        else:
            assert decoded == value


def test_memory_link_round_trip():
    sender = MemoryTransport()
    receiver = MemoryTransport()
    sender_to_receiver = CrsfLink(sender)
    receiving_link = CrsfLink(receiver)

    channels = tuple(992 + index for index in range(16))
    sender_to_receiver.send_channels(channels)
    receiver.feed(sender.drain_tx())
    frames = receiving_link.poll()
    assert len(frames) == 1
    assert unpack_channels(frames[0].payload) == channels


def test_decode_rejects_short_telemetry_payload():
    frame = CrsfFrame(0xEA, 0x14, b"\x01")
    with pytest.raises(ValueError):
        decode_telemetry(frame)
