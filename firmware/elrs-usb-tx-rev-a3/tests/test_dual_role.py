from __future__ import annotations

import hashlib
import json
from pathlib import Path
import struct
import sys
import tempfile
import unittest
from unittest import mock


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


def app_trailer(data: bytes, offset: int) -> tuple[str, dict[str, object]]:
    product = (
        data[offset : offset + assemble.APPENDED_PRODUCT_SIZE]
        .split(b"\0", 1)[0]
        .decode("utf-8")
    )
    hardware_offset = (
        offset
        + assemble.APPENDED_PRODUCT_SIZE
        + assemble.APPENDED_DEVICE_SIZE
        + assemble.APPENDED_OPTIONS_SIZE
    )
    hardware = json.loads(
        data[hardware_offset : hardware_offset + assemble.APPENDED_HARDWARE_SIZE]
        .split(b"\0", 1)[0]
        .decode("utf-8")
    )
    return product, hardware


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
        ):
            self.assertIn(required, patch)
        self.assertNotIn("aerialkitRoleHeartbeat", patch)

        led_patch = (
            ROOT / "patches" / "expresslrs-4.1.0-led-state.patch"
        ).read_text(encoding="utf-8")
        for required in (
            "return aerialkitLedTimeout()",
            "aerialkitLedMarkSerialInput",
        ):
            self.assertIn(required, led_patch)
        self.assertIn(
            "return DURATION_IMMEDIATELY;",
            led_patch,
        )
        led_state = (
            ROOT
            / "expresslrs-overlay"
            / "include"
            / "aerialkit_led_state_machine.h"
        ).read_text(encoding="utf-8")
        for required in ("SlowBreath", "FastBreath", "AmberTriple"):
            self.assertIn(required, led_state)

    def test_rxtx_identity_writes_commit_before_reboot(self) -> None:
        patch = (
            ROOT / "patches" / "expresslrs-4.1.0-rxtx-persistence.patch"
        ).read_text(encoding="utf-8")
        self.assertEqual(patch.count("+                config.Commit();"), 1)
        self.assertEqual(patch.count("+            config.Commit();"), 1)
        self.assertLess(
            patch.index("config.SetUID(mspPayload);"),
            patch.index("config.Commit();", patch.index("config.SetUID(mspPayload);")),
        )
        self.assertLess(
            patch.index("config.SetBindPhrase(mspPayload, payloadLen);"),
            patch.index(
                "config.Commit();",
                patch.index("config.SetBindPhrase(mspPayload, payloadLen);"),
            ),
        )
        bootstrap = (ROOT / "scripts" / "bootstrap.sh").read_text(encoding="utf-8")
        self.assertIn(
            'git -C "$express" apply "$rxtx_persistence_patch_file"',
            bootstrap,
        )

    def test_rx_search_starts_on_the_tx_default_rate(self) -> None:
        sys.path.insert(0, str(ROOT / "tools"))
        import apply_rf_defaults

        with tempfile.TemporaryDirectory() as temporary:
            source = Path(temporary) / "rx_main.cpp"
            source.write_text(
                "void setupRadio() {\n"
                "    scanIndex = config.GetRateInitialIdx();\n"
                "    for (;;) {}\n"
                "}\n",
                encoding="utf-8",
            )
            apply_rf_defaults.apply(source)
            text = source.read_text(encoding="utf-8")
        self.assertIn(
            "scanIndex = enumRatetoIndexSafe(RATE_LORA_2G4_250HZ);",
            text,
        )
        self.assertIn(apply_rf_defaults.MARKER, text)
        bootstrap = (ROOT / "scripts" / "bootstrap.sh").read_text(encoding="utf-8")
        self.assertIn(
            'python3 "$root/tools/apply_rf_defaults.py"',
            bootstrap,
        )

    def test_rf_probe_is_wired_into_both_role_loops(self) -> None:
        sys.path.insert(0, str(ROOT / "tools"))
        import apply_rf_probe

        with tempfile.TemporaryDirectory() as temporary:
            source = Path(temporary)
            (source / "tx_main.cpp").write_text(
                "void loop() {\n  uint32_t now = millis();\n}\n",
                encoding="utf-8",
            )
            (source / "rx_main.cpp").write_text(
                "void loop() {\n    unsigned long now = millis();\n}\n",
                encoding="utf-8",
            )
            apply_rf_probe.apply_tree(source)
            tx = (source / "tx_main.cpp").read_text(encoding="utf-8")
            rx = (source / "rx_main.cpp").read_text(encoding="utf-8")
        self.assertIn(apply_rf_probe.MARKER, tx)
        self.assertIn(apply_rf_probe.MARKER, rx)
        bootstrap = (ROOT / "scripts" / "bootstrap.sh").read_text(encoding="utf-8")
        self.assertIn('python3 "$root/tools/apply_rf_probe.py"', bootstrap)
        led = (
            ROOT / "expresslrs-overlay" / "src" / "aerialkit_led_state.cpp"
        ).read_text(encoding="utf-8")
        self.assertIn("aerialkitRfNoteUsbFrame", led)
        diagnostics = (
            ROOT
            / "expresslrs-overlay"
            / "src"
            / "aerialkit_boot_diagnostics.cpp"
        ).read_text(encoding="utf-8")
        for field in ("usb_frames=", "usb_rc=", "last_usb_type=", "linkmode="):
            self.assertIn(field, diagnostics)
        for field in ("usb_avail_max=", "usb_reads=", "usb_bytes="):
            self.assertIn(field, diagnostics)

    def test_native_usb_read_probe_is_wired(self) -> None:
        sys.path.insert(0, str(ROOT / "tools"))
        import apply_usb_rx_probe

        with tempfile.TemporaryDirectory() as temporary:
            source = Path(temporary) / "tx_main.cpp"
            source.write_text(
                "void HandleUARTin() {\n"
                "    TxUSB->readBytes(buf, size);\n"
                "}\n",
                encoding="utf-8",
            )
            apply_usb_rx_probe.apply(source)
            text = source.read_text(encoding="utf-8")
        self.assertIn(apply_usb_rx_probe.MARKER, text)
        self.assertIn("const auto usbAvailable = TxUSB->available();", text)
        self.assertIn(
            "const auto usbRead = TxUSB->readBytes(buf, size);",
            text,
        )
        self.assertIn(
            "static_cast<uint16_t>(usbRead));",
            text,
        )
        self.assertIn("#else\n    TxUSB->readBytes(buf, size);\n#endif", text)
        bootstrap = (ROOT / "scripts" / "bootstrap.sh").read_text(encoding="utf-8")
        self.assertIn(
            'python3 "$root/tools/apply_usb_rx_probe.py"',
            bootstrap,
        )
        self.assertIn('"$root/tools"', bootstrap)
        self.assertIn("grep -zv '/__pycache__/'", bootstrap)
        self.assertIn("grep -zv '\\.pyc$'", bootstrap)

    def test_gpio4_high_is_tx_and_low_is_rx_everywhere(self) -> None:
        selector = (ROOT / "bootloader" / "selector.c").read_text(
            encoding="utf-8"
        )
        self.assertIn(
            "(AERIALKIT_GPIO_IN_REG & (1U << 4)) ? ROLE_TX : ROLE_RX",
            selector,
        )
        boot_logic = (
            ROOT
            / "expresslrs-overlay"
            / "include"
            / "aerialkit_boot_logic.h"
        ).read_text(encoding="utf-8")
        self.assertIn("return role_pin_high != rx_role;", boot_logic)
        upstream_patch = (
            ROOT / "patches" / "expresslrs-4.1.0-aerialkit.patch"
        ).read_text(encoding="utf-8")
        self.assertIn(
            "aerialkit_role_matches(roleHigh, false)",
            upstream_patch,
        )
        self.assertIn(
            "aerialkit_role_matches(roleHigh, true)",
            upstream_patch,
        )
        self.assertNotIn("const bool matches = !roleHigh;", upstream_patch)
        self.assertNotIn("const bool matches = roleHigh;", upstream_patch)
        diagnostics = (
            ROOT
            / "expresslrs-overlay"
            / "src"
            / "aerialkit_boot_diagnostics.cpp"
        ).read_text(encoding="utf-8")
        self.assertIn('RX_ROLE ? "low" : "high"', diagnostics)

    def test_selector_linker_script_is_parameterized(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            output = Path(temporary) / "selector.ld"
            assemble.render_selector_linker_script(
                ROOT / "bootloader" / "selector.ld",
                output,
                select_function=0x403CD9C8,
                mmap_function=0x403CD384,
                munmap_function=0x403CD3EC,
            )
            rendered = output.read_text(encoding="utf-8")
        self.assertIn(".trampoline 0x403cd9c8", rendered)
        self.assertIn("bootloader_mmap = 0x403cd384;", rendered)
        self.assertIn("bootloader_munmap = 0x403cd3ec;", rendered)
        self.assertNotIn("@BOOTLOADER_", rendered)

    def test_sdk_memory_type_selects_matching_bootloader(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            map_path = Path(temporary) / "firmware.map"
            map_path.write_text(
                "/toolchain/esp32s3/qio_qspi/libspi_flash.a\n"
                "/toolchain/esp32s3/qio_qspi/libbootloader_support.a\n",
                encoding="utf-8",
            )
            memory_type = assemble.sdk_memory_type_from_map(map_path)
        self.assertEqual(memory_type, "qio_qspi")
        self.assertEqual(
            assemble.bootloader_name_for_memory_type(memory_type),
            "bootloader_qio_80m.elf",
        )

    def test_bootloader_source_mismatch_is_fatal(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            temp = Path(temporary)
            expected = temp / "bootloader.bin"
            output = temp / "regenerated.bin"
            expected.write_bytes(b"qio bootloader")

            def write_wrong_image(command: list[str], **_: object) -> None:
                self.assertEqual(command[command.index("--flash_mode") + 1], "dio")
                output.write_bytes(b"dio bootloader")

            with mock.patch.object(assemble, "run", side_effect=write_wrong_image):
                with self.assertRaisesRegex(RuntimeError, "flash mode mismatch"):
                    assemble.verify_bootloader_source(
                        bootloader_elf=temp / "bootloader_qio_80m.elf",
                        build_bootloader=expected,
                        flash_mode="dio",
                        esptool=temp / "esptool.py",
                        output=output,
                    )

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
            manifest["roles"],
            {"gpio4_low": "rx", "gpio4_high": "tx"},
        )
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
            manifest["bootloader"]["elf"], "bootloader_qio_80m.elf"
        )
        self.assertEqual(
            manifest["bootloader"]["sdk_memory_type"], "qio_qspi"
        )
        self.assertEqual(manifest["bootloader"]["app_flash_mode"], "dio")
        self.assertEqual(manifest["bootloader"]["flash_mode"], "dio")

        targets = (
            ROOT
            / ".vendor"
            / "ExpressLRS"
            / "src"
            / "hardware"
        )
        for role, slot, product in (
            (
                "tx",
                "tx_0",
                "Aerial Kit ELRS USB TX Rev A3 2.4GHz TX",
            ),
            (
                "rx",
                "rx_0",
                "Aerial Kit ELRS USB TX Rev A3 2.4GHz RX",
            ),
        ):
            app_offset = partitions[slot][0]
            image_end = manifest["applications"][role]["image_size"]
            product_name, hardware = app_trailer(data, app_offset + image_end)
            expected_hardware = json.loads(
                (
                    targets
                    / role.upper()
                    / "AerialKit ELRS USB TX Rev A3.json"
                ).read_text(encoding="utf-8")
            )
            self.assertEqual(product_name, product)
            self.assertEqual(hardware, expected_hardware)
            self.assertEqual(
                image_end + assemble.APPENDED_BLOCK_SIZE,
                manifest["applications"][role]["size"],
            )

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
