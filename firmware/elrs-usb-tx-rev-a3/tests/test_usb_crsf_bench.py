"""Host-side tests for tools/usb_crsf_bench.py (no hardware)."""

from __future__ import annotations

from pathlib import Path
import sys
import types
import unittest
from unittest import mock

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))

import usb_crsf_bench as bench


class FrameTest(unittest.TestCase):
    def test_valid_device_ping_bytes(self) -> None:
        self.assertEqual(bench.device_ping(), bytes.fromhex("C8 04 28 00 EA 54"))
        self.assertEqual(bench.device_ping(0xEE), bytes.fromhex("C8 04 28 EE EA 97"))

    def test_led_selftest_frame(self) -> None:
        frame = bench.led_selftest_frame(0xEC)
        self.assertEqual(frame[:9], bytes([0xC8, 8, 0x7F, 0xEC, 0xEA]) + b"AKLT")
        self.assertEqual(bench.crc8_dvb_s2(frame[2:-1]), frame[-1])

    def test_rc_frame_is_26_valid_bytes(self) -> None:
        frame = bench.rc_frame()
        self.assertEqual(len(frame), 26)
        self.assertEqual(frame[:3], bytes([0xC8, 24, 0x16]))
        self.assertEqual(bench.crc8_dvb_s2(frame[2:-1]), frame[-1])

    def test_channel_packing_round_trip(self) -> None:
        values = [172, 992, 1811, 0, 2047] + [992] * 11
        packed = int.from_bytes(bench.pack_channels(values), "little")
        self.assertEqual(
            [(packed >> (11 * index)) & 0x7FF for index in range(16)], values
        )
        with self.assertRaises(ValueError):
            bench.pack_channels([2048] + [992] * 15)


class ParserTest(unittest.TestCase):
    def test_resynchronises_across_text_and_garbage(self) -> None:
        parser = bench.CrsfStreamParser()
        rc = bench.rc_frame()
        stream = (
            b"SPIWP:0xee\r\nAK-BOOT role=tx slot=tx_0 gpio4=low\r\n"
            + b"\x00\xff"
            + rc[:10]
        )
        frames = parser.feed(stream)
        frames += parser.feed(rc[10:] + bench.device_ping() + b"\r\n")
        self.assertEqual([frame.name for frame in frames],
                         ["RC_CHANNELS_PACKED", "DEVICE_PING"])
        self.assertIn("AK-BOOT role=tx slot=tx_0 gpio4=low", parser.lines)

    def test_rejects_bad_crc_and_recovers(self) -> None:
        parser = bench.CrsfStreamParser()
        bad = bytearray(bench.rc_frame())
        bad[-1] ^= 0xFF
        frames = parser.feed(bytes(bad) + bench.rc_frame())
        self.assertEqual(len(frames), 1)
        self.assertGreaterEqual(parser.bad_crc, 1)

    def test_accepts_module_and_receiver_sync_bytes(self) -> None:
        parser = bench.CrsfStreamParser()
        stats = bench.build_frame(0xEA, 0x14, bytes(10))
        frames = parser.feed(stats)
        self.assertEqual([frame.name for frame in frames], ["LINK_STATISTICS"])

    def test_decodes_device_info(self) -> None:
        payload = bytes([0xEA, 0xEE]) + b"AK USB TX A3\x00" + b"ELRS" + bytes(8) + bytes([42, 0])
        frame = bench.CrsfStreamParser().feed(bench.build_frame(0xEA, 0x29, payload))[0]
        info = bench.decode_device_info(frame)
        self.assertEqual(info["name"], "AK USB TX A3")
        self.assertEqual(info["origin"], 0xEE)
        self.assertEqual(info["parameters"], 42)


class PortOpenTest(unittest.TestCase):
    def _fake_serial(self, log: list[tuple[str, bool]]):
        class FakeSerial:
            def __init__(self) -> None:
                self.is_open = False
                self._dtr = True
                self._rts = True

            def open(self) -> None:
                self.is_open = True
                log.append(("open", True))

            @property
            def dtr(self) -> bool:
                return self._dtr

            @dtr.setter
            def dtr(self, value: bool) -> None:
                self._dtr = value
                if self.is_open:
                    log.append(("dtr", value))
                    self._check()

            @property
            def rts(self) -> bool:
                return self._rts

            @rts.setter
            def rts(self, value: bool) -> None:
                self._rts = value
                if self.is_open:
                    log.append(("rts", value))
                    self._check()

            def _check(self) -> None:
                # RTS asserted with DTR released resets the ESP32-S3.
                if self._rts and not self._dtr:
                    log.append(("reset", True))

        return types.SimpleNamespace(Serial=FakeSerial)

    def test_open_never_presents_the_reset_state(self) -> None:
        log: list[tuple[str, bool]] = []
        with mock.patch.dict(sys.modules, {"serial": self._fake_serial(log)}):
            handle = bench.open_port("/dev/null", reset=False)
        self.assertNotIn(("reset", True), log)
        self.assertEqual(log[1:], [("rts", False), ("dtr", False)])
        self.assertFalse(handle.dtr)
        self.assertFalse(handle.rts)

    def test_explicit_reset_pulses_rts(self) -> None:
        log: list[tuple[str, bool]] = []
        with mock.patch.dict(sys.modules, {"serial": self._fake_serial(log)}), \
                mock.patch.object(bench.time, "sleep"):
            bench.open_port("/dev/null", reset=True)
        self.assertEqual(log.count(("reset", True)), 1)


if __name__ == "__main__":
    unittest.main()
