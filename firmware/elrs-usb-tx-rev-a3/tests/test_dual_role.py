from __future__ import annotations

import hashlib
import json
from pathlib import Path
import struct
import sys
import unittest


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))

import assemble_dual_role as assemble


EXPECTED_OFFSETS = {
    "tx_0": (0x10000, 0x1A0000),
    "tx_1": (0x2B0000, 0x1A0000),
    "rx_0": (0x450000, 0x140000),
    "rx_1": (0x590000, 0x140000),
    "role_state": (0x6D0000, 0x2000),
    "spiffs": (0x6D2000, 0x120000),
}


def parse_partition_table(data: bytes, offset: int) -> dict[str, tuple[int, int]]:
    result: dict[str, tuple[int, int]] = {}
    while offset + 32 <= len(data):
        magic, _kind, _subtype, address, size, label, _flags = struct.unpack(
            "<HBBII16sI", data[offset : offset + 32]
        )
        if magic != 0x50AA:
            break
        name = label.split(b"\0", 1)[0].decode("ascii")
        result[name] = (address, size)
        offset += 32
    return result


def parse_role_records(data: bytes, offset: int) -> list[tuple[int, int, int, int]]:
    records = []
    for sector in range(2):
        raw = data[offset + sector * 0x1000 : offset + sector * 0x1000 + 16]
        magic, sequence, tx_slot, rx_slot, _reserved, crc = struct.unpack(
            "<IIBBHI", raw
        )
        expected_crc = assemble.role_record_crc(raw[:12])
        records.append((magic, sequence, tx_slot, rx_slot))
        if magic != assemble.ROLE_RECORD_MAGIC or crc != expected_crc:
            raise AssertionError("invalid role_state record")
    return records


def selected_slot(
    image: bytearray,
    partitions: dict[str, tuple[int, int]],
    role: str,
    configured_slot: int,
) -> int:
    primary = 0 if role == "tx" else 2
    fallback = 1 if role == "tx" else 3

    def valid(slot: int) -> bool:
        name = ("tx_" if role == "tx" else "rx_") + str(slot % 2)
        offset, _size = partitions[name]
        return image[offset] == 0xE9

    for slot in (configured_slot, fallback, primary):
        if valid(slot):
            return slot
    return -99


class DualRoleArtifactTests(unittest.TestCase):
    def test_source_partition_layout(self) -> None:
        rows = assemble.read_partitions(ROOT / "partitions" / "dual-role.csv")
        for name, expected in EXPECTED_OFFSETS.items():
            self.assertEqual(
                (int(rows[name]["offset"]), int(rows[name]["size"])),
                expected,
            )

    def test_role_state_records_are_duplicated_and_valid(self) -> None:
        state = assemble.make_role_state(0x2000)
        records = parse_role_records(state, 0)
        self.assertEqual(
            records,
            [
                (assemble.ROLE_RECORD_MAGIC, 1, 0, 2),
                (assemble.ROLE_RECORD_MAGIC, 1, 0, 2),
            ],
        )

    def test_native_usb_and_led_patch_wiring(self) -> None:
        patch = (
            ROOT / "patches" / "expresslrs-4.1.0-native-usb.patch"
        ).read_text(encoding="utf-8")
        for required in (
            "UsbCrsfConnected",
            "CRSF_FRAMETYPE_RC_CHANNELS_PACKED",
            "SerialUSB_device",
            "new SerialCRSF(USBSerial, USBSerial)",
            "aerialkitRoleHeartbeat",
            "two blue pulses identify RX",
            "one green pulse identifies TX",
        ):
            self.assertIn(required, patch)

    def test_usb_crsf_channel_frame_is_26_bytes(self) -> None:
        from aerial_kit.comms.crsf import CrsfParser, make_rc_channels_frame

        channels = [172, 992, 992, 172, 992, 172] + [992] * 10
        frame = make_rc_channels_frame(channels, address=0xC8)
        self.assertEqual(len(frame), 26)
        parsed = CrsfParser().feed(frame)
        self.assertEqual(len(parsed), 1)
        self.assertEqual(parsed[0].frame_type, 0x16)
        self.assertEqual(len(parsed[0].payload), 22)

    def test_generated_artifact_layout(self) -> None:
        artifact = ROOT / "dist" / "aerialkit-rev-a3-elrs-dual-role.bin"
        manifest_path = ROOT / "dist" / "aerialkit-rev-a3-elrs-dual-role.json"
        if not artifact.is_file() or not manifest_path.is_file():
            self.skipTest("generated factory artifact is not present")

        data = bytearray(artifact.read_bytes())
        manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
        self.assertEqual(
            hashlib.sha256(data).hexdigest(),
            manifest["sha256"],
        )

        partitions = parse_partition_table(data, assemble.PARTITION_TABLE_OFFSET)
        for name, expected in EXPECTED_OFFSETS.items():
            self.assertEqual(partitions[name], expected)

        self.assertEqual(data[0x10000], 0xE9)
        self.assertEqual(data[0x450000], 0xE9)
        self.assertTrue(all(byte == 0xFF for byte in data[0x2B0000:0x2B0010]))
        self.assertTrue(all(byte == 0xFF for byte in data[0x590000:0x590010]))
        self.assertEqual(parse_role_records(data, 0x6D0000)[0][2:], (0, 2))
        self.assertIn(b"littlefs", data[0x6D2000 + 8 : 0x6D2000 + 24])

        self.assertEqual(
            selected_slot(data, partitions, "tx", 0),
            0,
        )
        self.assertEqual(
            selected_slot(data, partitions, "rx", 2),
            2,
        )
        data[0x10000] = 0xFF
        data[0x2B0000] = 0xFF
        self.assertEqual(
            selected_slot(data, partitions, "tx", 0),
            -99,
        )

        data[0x2B0000] = 0xE9
        data[0x10000] = 0xFF
        self.assertEqual(
            selected_slot(data, partitions, "tx", 0),
            1,
        )
        self.assertEqual(
            selected_slot(data, partitions, "rx", 2),
            2,
        )


if __name__ == "__main__":
    unittest.main()
