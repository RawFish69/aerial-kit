from __future__ import annotations

import json
import threading
from pathlib import Path

import pytest

from aerial_kit.comms.crsf import (
    CRSF_ADDRESS_CRSF_TRANSMITTER,
    CRSF_ADDRESS_RADIO_TRANSMITTER,
    CRSF_CHANNEL_MAX,
    CRSF_CHANNEL_MID,
    CRSF_CHANNEL_MIN,
    CRSF_FRAMETYPE_DEVICE_INFO,
    CRSF_FRAMETYPE_DEVICE_PING,
    CRSF_FRAMETYPE_LINK_STATISTICS,
    CrsfChannelMap,
    CrsfParser,
    MemoryTransport,
    capture_link,
    default_telemetry_frames,
    drive_link,
    inject_frames,
    load_crsf_frames_file,
    make_frame,
    make_link_statistics_frame,
    make_rc_channels_frame,
    monitor_link,
    radio_status,
    run_two_board_bench,
    unpack_channels,
)
from aerial_kit.comms.crsf.codec import LinkStatisticsTelemetry


def _device_info(
    origin: int,
    destination: int,
    name: str = "Aerial Kit Rev A3",
) -> bytes:
    payload = bytes((destination, origin)) + name.encode() + b"\0"
    payload += b"ELRS"
    payload += (0).to_bytes(4, "big")
    payload += 0x00040100.to_bytes(4, "big")
    payload += bytes((11, 0))
    return make_frame(CRSF_FRAMETYPE_DEVICE_INFO, payload)


def _link_stats() -> bytes:
    return make_link_statistics_frame(
        LinkStatisticsTelemetry(
            uplink_rssi_1_dbm=-90,
            uplink_rssi_2_dbm=-92,
            uplink_link_quality=88,
            uplink_snr_db=7,
            active_antenna=1,
            rf_mode=2,
            uplink_tx_power=3,
            downlink_rssi_dbm=-80,
            downlink_link_quality=70,
            downlink_snr_db=4,
        )
    )


def test_device_ping_has_exact_extended_layout() -> None:
    transport = MemoryTransport()

    class AnsweringTransport(MemoryTransport):
        def _write(self, data: bytes) -> int:
            count = super()._write(data)
            frame = CrsfParser().feed(data)[0]
            assert frame.frame_type == CRSF_FRAMETYPE_DEVICE_PING
            assert frame.payload == bytes(
                (CRSF_ADDRESS_CRSF_TRANSMITTER, CRSF_ADDRESS_RADIO_TRANSMITTER)
            )
            self.feed(
                _device_info(
                    CRSF_ADDRESS_CRSF_TRANSMITTER,
                    CRSF_ADDRESS_RADIO_TRANSMITTER,
                )
            )
            return count

    status = radio_status(
        AnsweringTransport(),
        path="/dev/ttyACM0",
        logical_baud=400_000,
        role="tx",
        timeout=0.05,
    )
    assert status.product == "Aerial Kit Rev A3"
    assert status.role == "tx"
    assert status.origin == 0xEE
    assert status.destination == 0xEA
    assert status.parameter_count == 11
    assert status.firmware_string == "4.1.0"


def test_rc_drive_frame_is_exactly_26_bytes() -> None:
    frame = make_rc_channels_frame((992,) * 16)
    assert len(frame) == 26
    parsed = CrsfParser().feed(frame)
    assert len(parsed) == 1
    assert parsed[0].frame_type == 0x16
    assert unpack_channels(parsed[0].payload) == (992,) * 16


def test_monitor_counts_frames_crc_and_resync() -> None:
    transport = MemoryTransport(timeout=0.0)
    bad = bytearray(make_rc_channels_frame((992,) * 16))
    bad[-1] ^= 0xFF
    transport.feed(b"not-crsf")
    transport.feed(bytes(bad))
    transport.feed(make_rc_channels_frame((992,) * 16))
    report = monitor_link(transport, duration=0.03)
    assert report.passed
    assert report.frames == 1
    assert report.frames_by_type["rc_channels"] == 1
    assert report.crc_errors == 1
    assert report.resyncs >= 1
    assert json.loads(report.to_json())["status"] == "valid_crsf"


def test_capture_requires_valid_rc_and_reports_cadence() -> None:
    transport = MemoryTransport(timeout=0.0)
    transport.feed(make_rc_channels_frame(tuple(range(172, 188))))
    transport.feed(make_rc_channels_frame(tuple(range(180, 196))))
    transport.feed(make_rc_channels_frame(tuple(range(190, 206))))
    report = capture_link(transport, duration=0.03)
    assert report.passed
    assert report.rc_frames == 3
    assert report.channels == tuple(range(190, 206))
    assert report.status == "valid_rc"
    assert report.rc_cadence_hz is not None

    silent = capture_link(MemoryTransport(timeout=0.0), duration=0.02)
    assert not silent.passed
    assert silent.status == "no_bytes"


def test_drive_arm_hold_and_safe_disarm_sequence() -> None:
    transport = MemoryTransport(timeout=0.0)
    report = drive_link(
        transport,
        hz=100,
        duration=0.80,
        pattern="channel-test",
        arm_hold_sec=0.55,
        allow_write=True,
    )
    assert report.passed
    assert report.arm_hold_completed
    assert report.disarmed_at_end
    frames = [
        frame
        for frame in CrsfParser().feed(transport.drain_tx())
        if frame.frame_type == 0x16
    ]
    assert len(frames) >= 5
    decoded = [unpack_channels(frame.payload) for frame in frames]
    channel_map = CrsfChannelMap()
    assert decoded[0][channel_map.arm - 1] == CRSF_CHANNEL_MAX
    assert decoded[0][channel_map.throttle - 1] == CRSF_CHANNEL_MIN
    assert any(
        channels[channel_map.roll - 1] != CRSF_CHANNEL_MID
        for channels in decoded
    )
    assert decoded[-2][channel_map.arm - 1] >= channel_map.arm_threshold
    assert decoded[-2][channel_map.throttle - 1] == CRSF_CHANNEL_MIN
    assert decoded[-1][channel_map.arm - 1] < channel_map.arm_threshold
    assert decoded[-1][channel_map.throttle - 1] == CRSF_CHANNEL_MIN


def test_drive_requires_write_and_disarms_when_cancelled() -> None:
    transport = MemoryTransport(timeout=0.0)
    with pytest.raises(PermissionError):
        drive_link(transport, duration=0.1, allow_write=False)

    cancelled = threading.Event()
    cancelled.set()
    report = drive_link(
        transport,
        duration=0.2,
        pattern="disarmed",
        allow_write=True,
        cancel_event=cancelled,
    )
    assert report.disarmed_at_end
    final = CrsfParser().feed(transport.drain_tx())[-1]
    channels = unpack_channels(final.payload)
    assert channels[CrsfChannelMap().arm - 1] < CrsfChannelMap().arm_threshold

    armed_transport = MemoryTransport(timeout=0.0)
    armed_cancel = threading.Event()
    result: list[object] = []
    thread = threading.Thread(
        target=lambda: result.append(
            drive_link(
                armed_transport,
                duration=2.0,
                pattern="arm-test",
                arm_hold_sec=0.55,
                allow_write=True,
                cancel_event=armed_cancel,
            )
        )
    )
    thread.start()
    threading.Event().wait(0.08)
    armed_cancel.set()
    thread.join(1.0)
    assert result
    final = CrsfParser().feed(armed_transport.drain_tx())[-1]
    final_channels = unpack_channels(final.payload)
    assert final_channels[CrsfChannelMap().arm - 1] < CrsfChannelMap().arm_threshold


def test_inject_validates_and_sends_telemetry_only(tmp_path: Path) -> None:
    frames = default_telemetry_frames()
    fixture = tmp_path / "telemetry.json"
    fixture.write_text(
        json.dumps({"frames": [{"raw_hex": frame.hex()} for frame in frames]}),
        encoding="utf-8",
    )
    loaded = load_crsf_frames_file(fixture)
    assert loaded == frames
    transport = MemoryTransport(timeout=0.0)
    report = inject_frames(transport, loaded, allow_write=True, interval_sec=0)
    assert report.passed
    assert report.frames == len(frames)
    assert transport.drain_tx() == b"".join(frames)

    wrong = MemoryTransport(timeout=0.0)
    with pytest.raises(PermissionError):
        inject_frames(wrong, loaded, allow_write=False)
    rc_fixture = tmp_path / "rc.json"
    rc_fixture.write_text(
        json.dumps({"raw_hex": make_rc_channels_frame((992,) * 16).hex()}),
        encoding="utf-8",
    )
    with pytest.raises(ValueError, match="telemetry frames only"):
        load_crsf_frames_file(rc_fixture)

    shared = Path(__file__).with_name("protocols") / "vectors" / "crsf.json"
    vector_frames = load_crsf_frames_file(shared)
    assert [CrsfParser().feed(frame)[0].frame_type for frame in vector_frames] == [
        0x1E,
        0x08,
        0x02,
        0x09,
        0x14,
    ]


class _BenchTransport(MemoryTransport):
    def __init__(self, role: str, other: _BenchTransport | None = None) -> None:
        super().__init__(timeout=0.0)
        self.role = role
        self.other = other

    def _write(self, data: bytes) -> int:
        count = super()._write(data)
        frames = CrsfParser().feed(data)
        for frame in frames:
            if frame.frame_type == CRSF_FRAMETYPE_DEVICE_PING:
                if self.role == "tx":
                    self.feed(_device_info(0xEE, 0xEA, "AK TX"))
                else:
                    self.feed(_device_info(0xEC, 0xC8, "AK RX"))
            elif frame.frame_type == 0x16 and self.other is not None:
                self.other.feed(data)
                self.feed(_link_stats())
                self.other.feed(_link_stats())
            elif frame.frame_type in {0x02, 0x08, 0x09, 0x1E} and self.other is not None:
                self.other.feed(data)
        return count


def test_two_board_bench_requires_observed_link_both_ways() -> None:
    tx = _BenchTransport("tx")
    rx = _BenchTransport("rx")
    tx.other = rx
    rx.other = tx
    report = run_two_board_bench(
        tx,
        rx,
        tx_path="/dev/ttyACM0",
        rx_path="/dev/ttyACM1",
        duration=1.0,
        hz=100,
    )
    assert report.tx_status is not None
    assert report.rx_status is not None
    assert report.requirements["rx_rc_capture"] == "PASS"
    assert report.requirements["tx_link_statistics"] == "PASS"
    assert report.requirements["rx_link_statistics"] == "PASS"
    assert report.requirements["telemetry_round_trip"] == "PASS"
    assert report.requirements["rf_link"] == "PASS"
    assert report.overall == "PASS"
    payload = json.loads(report.to_json())
    assert payload["overall"] == "PASS"
