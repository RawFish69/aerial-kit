#!/usr/bin/env python3
"""Native-USB CRSF bench probe for the Rev A3 dual-role firmware.

TX role: streams RC_CHANNELS_PACKED at a fixed rate, sends device pings, and
reports every CRSF frame and AK-* diagnostic line returned on the same USB
stream. RX role: listens for RC frames and can inject battery telemetry.

The ESP32-S3 USB Serial/JTAG resets the chip while RTS is asserted and DTR is
not. Linux asserts both lines on open(); pyserial then clears DTR before RTS,
which briefly produces exactly that reset state. This tool clears RTS first,
then DTR, so opening the port does not reboot the board (pass --reset to
reboot deliberately).
"""

from __future__ import annotations

import argparse
import collections
import dataclasses
import sys
import threading
import time

CRSF_SYNC_BYTES = (0xC8, 0xEA, 0xEC, 0xEE)
CRSF_MAX_FRAME = 64

FRAME_NAMES = {
    0x08: "BATTERY_SENSOR",
    0x14: "LINK_STATISTICS",
    0x16: "RC_CHANNELS_PACKED",
    0x1C: "LINK_RX_ID",
    0x1D: "LINK_TX_ID",
    0x28: "DEVICE_PING",
    0x29: "DEVICE_INFO",
    0x2B: "PARAMETER_SETTINGS_ENTRY",
    0x2D: "PARAMETER_WRITE",
    0x3A: "RADIO_ID",
}


def crc8_dvb_s2(data: bytes) -> int:
    crc = 0
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = ((crc << 1) ^ 0xD5) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc


def build_frame(sync: int, frame_type: int, payload: bytes) -> bytes:
    body = bytes([frame_type]) + payload
    return bytes([sync, len(body) + 1]) + body + bytes([crc8_dvb_s2(body)])


def pack_channels(channels: list[int]) -> bytes:
    if len(channels) != 16:
        raise ValueError("CRSF carries exactly 16 channels")
    bits = 0
    for index, value in enumerate(channels):
        if not 0 <= value <= 0x7FF:
            raise ValueError(f"channel {index} out of range: {value}")
        bits |= value << (11 * index)
    return bits.to_bytes(22, "little")


def rc_frame(channels: list[int] | None = None) -> bytes:
    values = channels if channels is not None else [992] * 16
    return build_frame(0xC8, 0x16, pack_channels(values))


def device_ping(dest: int = 0x00, origin: int = 0xEA) -> bytes:
    return build_frame(0xC8, 0x28, bytes([dest, origin]))


def led_selftest_frame(dest: int, origin: int = 0xEA) -> bytes:
    """Start the firmware LED self-test (pure red, green, blue; 1 s each).

    dest is the board's own CRSF address: 0xEE on TX, 0xEC on RX.
    """
    return build_frame(0xC8, 0x7F, bytes([dest, origin]) + b"AKLT")


def led_permutation_frame(dest: int, origin: int = 0xEA) -> bytes:
    """Start the six-permutation physical LED channel-order diagnostic.

    Each permutation sends logical red, green and blue for one second, then
    half a second off. The USB stream labels each phase with
    ``AK-LEDPERM order=... logical=...``.
    """
    return build_frame(0xC8, 0x7F, bytes([dest, origin]) + b"AKLP")


def battery_frame(decivolts: int = 168, deciamps: int = 0,
                  capacity_mah: int = 0, remaining: int = 100) -> bytes:
    payload = (
        decivolts.to_bytes(2, "big")
        + deciamps.to_bytes(2, "big")
        + capacity_mah.to_bytes(3, "big")
        + bytes([remaining])
    )
    return build_frame(0xC8, 0x08, payload)


@dataclasses.dataclass
class Frame:
    sync: int
    type: int
    payload: bytes
    raw: bytes

    @property
    def name(self) -> str:
        return FRAME_NAMES.get(self.type, f"0x{self.type:02X}")


class CrsfStreamParser:
    """Incremental parser that resynchronises on CRSF sync bytes.

    Bytes that cannot start a frame are collected as text so ASCII diagnostic
    lines (ROM banner, AK-BOOT, AK-FAULT) interleaved on the stream are kept.
    """

    def __init__(self) -> None:
        self.buffer = bytearray()
        self.text = bytearray()
        self.bad_crc = 0
        self.lines: list[str] = []

    def _take_text(self, byte: int) -> None:
        if byte in (0x0A, 0x0D):
            if self.text:
                self.lines.append(self.text.decode("ascii", "replace"))
                self.text.clear()
        elif 0x20 <= byte < 0x7F:
            self.text.append(byte)

    def feed(self, data: bytes) -> list[Frame]:
        self.buffer.extend(data)
        frames: list[Frame] = []
        while self.buffer:
            if self.buffer[0] not in CRSF_SYNC_BYTES:
                self._take_text(self.buffer.pop(0))
                continue
            if len(self.buffer) < 2:
                break
            length = self.buffer[1]
            if length < 2 or length > CRSF_MAX_FRAME - 2:
                self._take_text(self.buffer.pop(0))
                continue
            total = length + 2
            if len(self.buffer) < total:
                break
            raw = bytes(self.buffer[:total])
            if crc8_dvb_s2(raw[2:-1]) != raw[-1]:
                self.bad_crc += 1
                self._take_text(self.buffer.pop(0))
                continue
            del self.buffer[:total]
            frames.append(Frame(raw[0], raw[2], raw[3:-1], raw))
        return frames


def decode_device_info(frame: Frame) -> dict[str, object]:
    payload = frame.payload
    dest, origin = payload[0], payload[1]
    name_end = payload.index(0, 2)
    name = payload[2:name_end].decode("ascii", "replace")
    rest = payload[name_end + 1:]
    return {
        "dest": dest,
        "origin": origin,
        "name": name,
        "serial": rest[0:4].hex() if len(rest) >= 4 else "",
        "parameters": rest[12] if len(rest) >= 13 else None,
    }


def open_port(port: str, reset: bool, baud: int = 400000):
    import serial  # imported here so the pure helpers test without pyserial

    handle = serial.Serial()
    handle.port = port
    handle.baudrate = baud
    handle.timeout = 0.02
    handle.write_timeout = 0.05
    # Leave both lines at their open() state (asserted) and clear them in the
    # order that never presents RTS=1/DTR=0 to the USB Serial/JTAG.
    handle.dtr = True
    handle.rts = True
    handle.open()
    handle.rts = False
    handle.dtr = False
    if reset:
        handle.rts = True   # RTS=1, DTR=0: chip reset
        time.sleep(0.1)
        handle.rts = False
    return handle


def run(args: argparse.Namespace) -> int:
    import serial

    handle = open_port(args.port, args.reset, args.baud)
    parser = CrsfStreamParser()
    counts: collections.Counter[str] = collections.Counter()
    infos: list[dict[str, object]] = []
    stop = threading.Event()
    received_bytes = 0
    first_rx: float | None = None
    start = time.monotonic()

    def reader() -> None:
        nonlocal received_bytes, first_rx
        while not stop.is_set():
            try:
                data = handle.read(4096)
            except serial.SerialException as error:
                print(f"[{time.monotonic() - start:7.3f}] read error: {error}")
                return
            if not data:
                continue
            received_bytes += len(data)
            if first_rx is None:
                first_rx = time.monotonic() - start
            for frame in parser.feed(data):
                counts[frame.name] += 1
                if frame.type == 0x29:
                    infos.append(decode_device_info(frame))
                if args.verbose:
                    print(f"[{time.monotonic() - start:7.3f}] {frame.name} {frame.raw.hex(' ')}")
            while parser.lines:
                print(f"[{time.monotonic() - start:7.3f}] text: {parser.lines.pop(0)}")

    thread = threading.Thread(target=reader, daemon=True)
    thread.start()
    time.sleep(args.settle)

    sent = 0
    stalls = 0
    first_stall: float | None = None
    worst_write = 0.0
    period = 1.0 / args.rate if args.rate > 0 else 0.0
    next_send = time.monotonic()
    next_ping = time.monotonic()
    deadline = time.monotonic() + args.duration
    if args.led_selftest is not None:
        handle.write(led_selftest_frame(args.led_selftest))
    if args.led_permutation is not None:
        handle.write(led_permutation_frame(args.led_permutation))
    frame = rc_frame()
    while time.monotonic() < deadline:
        now = time.monotonic()
        try:
            if args.ping_interval > 0 and now >= next_ping:
                handle.write(device_ping())
                next_ping = now + args.ping_interval
            if args.telemetry and sent % 50 == 0:
                handle.write(battery_frame())
            if args.rate > 0:
                began = time.monotonic()
                handle.write(frame)
                worst_write = max(worst_write, time.monotonic() - began)
                sent += 1
        except serial.SerialTimeoutException:
            stalls += 1
            if first_stall is None:
                first_stall = time.monotonic() - start
                print(f"[{first_stall:7.3f}] first write stall after {sent} RC frames")
        except serial.SerialException as error:
            print(f"[{time.monotonic() - start:7.3f}] write error after {sent} RC frames: {error}")
            break
        if period:
            next_send += period
            delay = next_send - time.monotonic()
            if delay > 0:
                time.sleep(delay)
        else:
            time.sleep(0.01)

    time.sleep(0.5)
    stop.set()
    thread.join(timeout=1)
    handle.close()

    print("--- summary ---")
    print(f"rc_sent={sent} write_stalls={stalls} first_stall={first_stall} "
          f"worst_write_ms={worst_write * 1000:.1f}")
    print(f"rx_bytes={received_bytes} first_rx_s={first_rx} bad_crc={parser.bad_crc}")
    for name, count in sorted(counts.items()):
        print(f"  {name}: {count}")
    for info in infos[:3]:
        print(f"  device_info: {info}")
    return 0 if stalls == 0 else 1


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("port")
    parser.add_argument("--baud", type=int, default=400000,
                        help="logical CRSF rate (TX 400000, RX 420000)")
    parser.add_argument("--rate", type=float, default=150.0, help="RC frames/s (0 = listen only)")
    parser.add_argument("--duration", type=float, default=5.0)
    parser.add_argument("--settle", type=float, default=0.5, help="listen before sending (s)")
    parser.add_argument("--ping-interval", type=float, default=1.0)
    parser.add_argument("--telemetry", action="store_true", help="also send battery telemetry")
    parser.add_argument("--reset", action="store_true", help="reset the board after opening")
    parser.add_argument("--led-selftest", type=lambda v: int(v, 0), metavar="ADDR",
                        help="send the LED self-test to 0xEE (TX) or 0xEC (RX) first")
    parser.add_argument("--led-permutation", type=lambda v: int(v, 0), metavar="ADDR",
                        help="test all six LED wire orders on 0xEE (TX) or 0xEC (RX)")
    parser.add_argument("--verbose", action="store_true")
    parser.add_argument("--write", action="store_true",
                        help="allow RC, telemetry, self-test or reset writes")
    args = parser.parse_args(argv)
    if (args.rate > 0 or args.telemetry or args.reset or
            args.led_selftest is not None or
            args.led_permutation is not None) and not args.write:
        parser.error("--write is required for RC, telemetry, self-test or reset operations")
    return run(args)


if __name__ == "__main__":
    sys.exit(main())
