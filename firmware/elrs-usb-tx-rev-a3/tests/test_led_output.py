"""Dual-role LED output tests.

* Host-compiles tests/native_led/test_led_output.cpp against the overlay LED
  header (exact WS2812 words, RMT bit timing, no unrelated hue, boot pulse
  counts, breathing periods).
* Checks the RMT LED patch wiring: upstream devRGB.cpp/esp32rgb.cpp are
  compiled out of dual-role builds.
* When build outputs exist, checks the linked ELFs: no I2S LED driver and no
  upstream rainbow/HSV/fade routine can be reached.
"""

from __future__ import annotations

from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
BUILD = ROOT / ".vendor" / "ExpressLRS" / "src" / ".pio" / "build"
ENVS = (
    "AERIALKIT_ELRS_USB_TX_REV_A3_via_UART",
    "AERIALKIT_ELRS_USB_TX_REV_A3_RX_via_UART",
)
FORBIDDEN_SYMBOLS = (
    "HsvToRgb",
    "blinkyUpdate",
    "hueFadeLED",
    "brightnessFadeLED",
    "flashLED",
    "ESP32LedDriver",
    "WS281Binit",
    "i2s_write",
    "i2s_driver_install",
)
# neopixelWrite/rmtWriteBlocking are linked by the Arduino core's
# digitalWrite()/pinMode(), which only route the virtual pin RGB_BUILTIN
# (SOC_GPIO_PIN_COUNT + 48 = 97) to them. No dual-role code uses pin 97; the
# overlay source checks above forbid any direct use.


def find_nm() -> str | None:
    candidates = sorted(
        Path.home().glob(
            "Projects/elrs-usb-tx/toolchain/toolchain-xtensa-esp32s3/bin/xtensa-esp32s3-elf-nm"
        )
    ) + sorted(
        Path.home().glob(".platformio/packages/toolchain-xtensa-esp32s3/bin/xtensa-esp32s3-elf-nm")
    )
    for candidate in candidates:
        if candidate.is_file():
            return str(candidate)
    return shutil.which("xtensa-esp32s3-elf-nm")


class LedHostTest(unittest.TestCase):
    def test_led_output_host_program(self) -> None:
        compiler = shutil.which("g++") or shutil.which("c++")
        if compiler is None:
            self.skipTest("no host C++ compiler")
        with tempfile.TemporaryDirectory() as temporary:
            binary = Path(temporary) / "test_led_output"
            subprocess.run(
                [
                    compiler,
                    "-std=c++17",
                    "-Wall",
                    "-Wextra",
                    "-Werror",
                    "-I",
                    str(ROOT / "expresslrs-overlay" / "include"),
                    str(ROOT / "tests" / "native_led" / "test_led_output.cpp"),
                    "-o",
                    str(binary),
                ],
                check=True,
            )
            result = subprocess.run([str(binary)], capture_output=True, text=True)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertIn("OK", result.stdout)


class LedPatchWiringTest(unittest.TestCase):
    def test_upstream_led_sources_are_compiled_out(self) -> None:
        patch = (ROOT / "patches" / "expresslrs-4.1.0-rmt-led.patch").read_text()
        self.assertIn("+#if !defined(AERIALKIT_DUAL_ROLE)", patch)
        self.assertIn("+#endif // !AERIALKIT_DUAL_ROLE", patch)
        self.assertIn(
            "+#if defined(PLATFORM_ESP32) && !defined(AERIALKIT_DUAL_ROLE)", patch
        )
        self.assertEqual(patch.count("+    aerialkitBootReport();") +
                         patch.count("+  aerialkitBootReport();"), 2)
        self.assertEqual(patch.count("+        aerialkitLedObserveFrame(message);"), 2)
        bootstrap = (ROOT / "scripts" / "bootstrap.sh").read_text()
        self.assertIn('git -C "$express" apply "$rmt_led_patch_file"', bootstrap)

    def test_rmt_led_device_replaces_rgb_device(self) -> None:
        source = (ROOT / "expresslrs-overlay" / "src" / "aerialkit_led_state.cpp").read_text()
        for required in (
            "device_t RGB_device",
            "aerialkitWs2812Show(color)",
            "aerialkit_led_selftest_frame",
            "void setButtonColors(uint8_t, uint8_t)",
        ):
            self.assertIn(required, source)
        driver = (ROOT / "expresslrs-overlay" / "src" / "aerialkit_ws2812.cpp").read_text()
        for required in (
            "rmt_write_items(LED_CHANNEL, items, AERIALKIT_WS2812_FRAME_ITEMS, false)",
            "rmt_wait_tx_done(LED_CHANNEL, TX_TIMEOUT)",
            "rmt_get_clk_div(LED_CHANNEL, &divider)",
            "config.clk_div = AERIALKIT_WS2812_RMT_CLK_DIV",
        ):
            self.assertIn(required, driver)
        boot = (ROOT / "expresslrs-overlay" / "src" / "aerialkit_boot_diagnostics.cpp").read_text()
        self.assertIn("aerialkitWs2812ShowCapped", boot)
        for text in (source, driver, boot):
            for forbidden in ("i2s_", "HsvToRgb", "ESP32LedDriver", "WS281BsetLED",
                              "rmtWriteBlocking", "rmtInit(", "neopixelWrite"):
                self.assertNotIn(forbidden, text)

    def test_dual_build_uses_one_platformio_invocation(self) -> None:
        build = (ROOT / "scripts" / "build.sh").read_text()
        dual = build.split("    dual)", 1)[1].split("    *)", 1)[0]
        self.assertEqual(dual.count('"$pio" run'), 1)
        self.assertIn("-e AERIALKIT_ELRS_USB_TX_REV_A3_via_UART", dual)
        self.assertIn("-e AERIALKIT_ELRS_USB_TX_REV_A3_RX_via_UART", dual)


class LinkedFirmwareTest(unittest.TestCase):
    def test_linked_elfs_have_no_upstream_led_paths(self) -> None:
        nm = find_nm()
        elfs = [BUILD / env / "firmware.elf" for env in ENVS]
        if nm is None or not all(elf.is_file() for elf in elfs):
            self.skipTest("dual-role build outputs or xtensa nm not available")
        for elf in elfs:
            symbols = subprocess.run(
                [nm, "-C", str(elf)], check=True, capture_output=True, text=True
            ).stdout
            for forbidden in FORBIDDEN_SYMBOLS:
                self.assertNotIn(forbidden, symbols, f"{elf.parent.name}: {forbidden}")
            for required in ("RGB_device", "aerialkitLedTimeout", "aerialkitWs2812Show",
                             "rmt_write_items", "aerialkitBootReport"):
                self.assertIn(required, symbols, f"{elf.parent.name}: {required}")


if __name__ == "__main__":
    unittest.main()
