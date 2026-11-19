#!/usr/bin/env python3
"""Build and verify the single Rev A3 ELRS TX/RX flash image."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile


BOOTLOADER_ADDR = 0x0000
PARTITION_ADDR = 0x8000
OTA_DATA_ADDR = 0xE000
TX_APP_ADDR = 0x10000
RX_APP_ADDR = 0x1F0000
TX_SLOT_SIZE = 0x1E0000
RX_SLOT_SIZE = 0x1E0000

BOOTLOADER_SELECTOR_ADDR = 0x403CC000
BOOTLOADER_SELECTOR_FN_ADDR = 0x403CC010
BOOTLOADER_SELECT_FN_ADDR = 0x403CD868
BOOTLOADER_SELECT_SEGMENT_ADDR = 0x403CC700
BOOTLOADER_CAVE_END = 0x403CC700

FLASH_SIZE = 8 * 1024 * 1024


def run(command: list[str], *, cwd: Path | None = None) -> None:
    print("+", " ".join(command))
    subprocess.run(command, cwd=cwd, check=True)


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def import_esptool(vendor: Path):
    esptool_dir = vendor / "ExpressLRS" / "src" / "python" / "external" / "esptool"
    sys.path.insert(0, str(esptool_dir))
    from esptool.bin_image import ESP32S3FirmwareImage

    return ESP32S3FirmwareImage


def compile_bootloader_selector(
    cache: Path,
    source_dir: Path,
    build_dir: Path,
) -> tuple[bytes, bytes]:
    toolchain = cache / "toolchain-xtensa-esp32s3" / "bin"
    prefix = toolchain / "xtensa-esp32s3-elf-"
    gcc = str(prefix) + "gcc"
    objcopy = str(prefix) + "objcopy"
    if not Path(gcc).is_file():
        raise FileNotFoundError(f"ESP32-S3 compiler not found: {gcc}")

    selector_object = build_dir / "selector.o"
    selector_elf = build_dir / "selector.elf"
    selector_bin = build_dir / "selector.bin"
    jump_object = build_dir / "selector_jump.o"
    jump_elf = build_dir / "selector_jump.elf"
    jump_bin = build_dir / "selector_jump.bin"

    run(
        [
            gcc,
            "-Os",
            "-mlongcalls",
            "-fno-jump-tables",
            "-fno-tree-switch-conversion",
            "-ffunction-sections",
            "-fdata-sections",
            "-fno-stack-protector",
            "-c",
            str(source_dir / "selector.c"),
            "-o",
            str(selector_object),
        ]
    )
    run(
        [
            gcc,
            "-nostdlib",
            "-Wl,--gc-sections",
            f"-Wl,-Ttext=0x{BOOTLOADER_SELECTOR_ADDR:x}",
            "-Wl,-e,aerialkit_role_partition",
            str(selector_object),
            "-o",
            str(selector_elf),
        ]
    )
    run(
        [
            objcopy,
            "-O",
            "binary",
            "-j",
            ".iram.text.aerialkit_selector.literal",
            "-j",
            ".iram.text.aerialkit_selector",
            str(selector_elf),
            str(selector_bin),
        ]
    )
    nm_output = subprocess.check_output(
        [str(prefix) + "nm", "-n", str(selector_elf)],
        text=True,
    )
    selector_symbol = next(
        (
            line.split()
            for line in nm_output.splitlines()
            if line.split() and line.split()[-1] == "aerialkit_role_partition"
        ),
        None,
    )
    if selector_symbol is None or int(selector_symbol[0], 16) != BOOTLOADER_SELECTOR_FN_ADDR:
        raise RuntimeError("selector function moved from its reserved bootloader address")

    run(
        [
            gcc,
            "-c",
            "-mlongcalls",
            str(source_dir / "selector_jump.S"),
            "-o",
            str(jump_object),
        ]
    )
    run(
        [
            gcc,
            "-nostdlib",
            f"-Wl,-Ttext=0x{BOOTLOADER_SELECT_FN_ADDR:x}",
            str(jump_object),
            "-o",
            str(jump_elf),
        ]
    )
    run(
        [
            objcopy,
            "-O",
            "binary",
            "-j",
            ".jump",
            str(jump_elf),
            str(jump_bin),
        ]
    )

    selector = selector_bin.read_bytes()
    jump = jump_bin.read_bytes()
    if not selector or len(selector) > 0x700:
        raise RuntimeError("selector compilation produced an unexpected image")
    if jump != bytes.fromhex("06e9f9"):
        raise RuntimeError(f"selector jump compilation produced {jump.hex()}, expected 06e9f9")
    if BOOTLOADER_SELECTOR_ADDR + len(selector) > BOOTLOADER_CAVE_END:
        raise RuntimeError("selector does not fit in the bootloader IRAM code cave")
    return selector, jump


def patch_bootloader(
    bootloader_elf: Path,
    selector: bytes,
    jump: bytes,
    output_path: Path,
    *,
    toolchain: Path,
    esptool: Path,
) -> None:
    objcopy = toolchain / "xtensa-esp32s3-elf-objcopy"
    loader_section = output_path.with_suffix(".loader.bin")
    run(
        [
            str(objcopy),
            "--dump-section",
            f".iram_loader.text={loader_section}",
            str(bootloader_elf),
        ]
    )
    loader = bytearray(loader_section.read_bytes())
    select_offset = BOOTLOADER_SELECT_FN_ADDR - BOOTLOADER_SELECT_SEGMENT_ADDR
    if select_offset < 0 or select_offset + len(jump) > len(loader):
        raise RuntimeError("bootloader selector function is outside its IRAM section")
    if loader[select_offset : select_offset + len(jump)] == jump:
        raise RuntimeError("bootloader is already dual-role patched")
    if loader[select_offset : select_offset + 3] != bytes.fromhex("36c100"):
        raise RuntimeError(
            "pinned bootloader selector entry has changed; refusing to patch an unknown build"
        )
    loader[select_offset : select_offset + len(jump)] = jump
    loader_section.write_bytes(loader)

    patched_elf = output_path.with_suffix(".elf")
    selector_file = output_path.with_suffix(".selector.bin")
    selector_file.write_bytes(selector)
    run(
        [
            str(objcopy),
            "--update-section",
            f".iram_loader.text={loader_section}",
            "--add-section",
            f".aerialkit_selector={selector_file}",
            "--set-section-flags",
            ".aerialkit_selector=alloc,load,code,contents",
            "--change-section-vma",
            f".aerialkit_selector=0x{BOOTLOADER_SELECTOR_ADDR:x}",
            str(bootloader_elf),
            str(patched_elf),
        ]
    )
    run(
        [
            sys.executable,
            str(esptool),
            "--chip",
            "esp32s3",
            "elf2image",
            "--flash_mode",
            "dio",
            "--flash_freq",
            "80m",
            "--flash_size",
            "8MB",
            "-o",
            str(output_path),
            str(patched_elf),
        ]
    )
    loader_section.unlink(missing_ok=True)
    patched_elf.unlink(missing_ok=True)
    selector_file.unlink(missing_ok=True)


def merge_image(
    bootloader: Path,
    partitions: Path,
    ota_data: Path,
    tx_app: Path,
    rx_app: Path,
    output: Path,
) -> tuple[int, dict[str, int]]:
    pieces = {
        BOOTLOADER_ADDR: bootloader,
        PARTITION_ADDR: partitions,
        OTA_DATA_ADDR: ota_data,
        TX_APP_ADDR: tx_app,
        RX_APP_ADDR: rx_app,
    }
    sizes = {address: path.stat().st_size for address, path in pieces.items()}
    if sizes[TX_APP_ADDR] > TX_SLOT_SIZE:
        raise RuntimeError("TX application overflows its app0 slot")
    if sizes[RX_APP_ADDR] > RX_SLOT_SIZE:
        raise RuntimeError("RX application overflows its app1 slot")

    image_size = max(address + size for address, size in sizes.items())
    merged = bytearray(b"\xff") * image_size
    for address, path in pieces.items():
        merged[address : address + sizes[address]] = path.read_bytes()
    output.write_bytes(merged)
    return image_size, sizes


def verify_image(
    output: Path,
    sizes: dict[str, int],
    *,
    vendor: Path,
) -> None:
    esptool = vendor / "ExpressLRS" / "src" / "python" / "external" / "esptool" / "esptool.py"
    run(
        [
            sys.executable,
            str(esptool),
            "--chip",
            "esp32s3",
            "image_info",
            str(output),
        ]
    )
    expected = {
        "bootloader": sizes[BOOTLOADER_ADDR],
        "partitions": sizes[PARTITION_ADDR],
        "ota_data": sizes[OTA_DATA_ADDR],
        "tx": sizes[TX_APP_ADDR],
        "rx": sizes[RX_APP_ADDR],
    }
    if not all(size > 0 for size in expected.values()):
        raise RuntimeError(f"empty image component: {expected}")


def verify_app(esptool: Path, path: Path, label: str) -> None:
    run(
        [
            sys.executable,
            str(esptool),
            "--chip",
            "esp32s3",
            "image_info",
            str(path),
        ]
    )
    if path.read_bytes()[:1] != b"\xe9":
        raise RuntimeError(f"{label} is not an ESP32-S3 application image")


def verify_patched_bootloader(
    bootloader: Path,
    selector: bytes,
    *,
    esptool: Path,
    image_class,
) -> None:
    with bootloader.open("rb") as stream:
        image = image_class(stream)
    selector_segment = next(
        (segment for segment in image.segments if segment.addr == BOOTLOADER_SELECTOR_ADDR),
        None,
    )
    loader_segment = next(
        (segment for segment in image.segments if segment.addr == BOOTLOADER_SELECT_SEGMENT_ADDR),
        None,
    )
    if selector_segment is None or loader_segment is None:
        raise RuntimeError("patched bootloader has an unexpected segment map")
    if selector_segment.data[: len(selector)] != selector:
        raise RuntimeError("patched bootloader does not contain the selector")
    offset = BOOTLOADER_SELECT_FN_ADDR - loader_segment.addr
    if loader_segment.data[offset : offset + 3] != bytes.fromhex("06e9f9"):
        raise RuntimeError("patched bootloader does not contain the role jump")
    run(
        [
            sys.executable,
            str(esptool),
            "--chip",
            "esp32s3",
            "image_info",
            str(bootloader),
        ]
    )


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--root", type=Path, required=True)
    parser.add_argument("--cache", type=Path, required=True)
    parser.add_argument("--vendor", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()

    root = args.root.resolve()
    cache = args.cache.resolve()
    vendor = args.vendor.resolve()
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)

    expresslrs = vendor / "ExpressLRS"
    image_class = import_esptool(vendor)
    tx_build = expresslrs / "src" / ".pio" / "build" / "AERIALKIT_ELRS_USB_TX_REV_A3_via_UART"
    rx_build = (
        expresslrs / "src" / ".pio" / "build" / "AERIALKIT_ELRS_USB_TX_REV_A3_RX_via_UART"
    )
    for build in (tx_build, rx_build):
        for name in ("firmware.bin", "partitions.bin", "bootloader.bin"):
            if not (build / name).is_file():
                raise FileNotFoundError(build / name)
    if (tx_build / "partitions.bin").read_bytes() != (rx_build / "partitions.bin").read_bytes():
        raise RuntimeError("TX and RX builds have different partition tables")
    if (tx_build / "bootloader.bin").read_bytes() != (rx_build / "bootloader.bin").read_bytes():
        raise RuntimeError("TX and RX builds have different bootloaders")

    with tempfile.TemporaryDirectory(prefix="aerialkit-dual-role-") as temporary:
        temporary_dir = Path(temporary)
        selector, jump = compile_bootloader_selector(
            cache,
            root / "bootloader",
            temporary_dir,
        )
        bootloader_elf = (
            cache
            / "framework-arduinoespressif32"
            / "tools"
            / "sdk"
            / "esp32s3"
            / "bin"
            / "bootloader_dio_80m.elf"
        )
        patched_bootloader = temporary_dir / "bootloader.bin"
        patch_bootloader(
            bootloader_elf,
            selector,
            jump,
            patched_bootloader,
            toolchain=cache / "toolchain-xtensa-esp32s3" / "bin",
            esptool=expresslrs / "src" / "python" / "external" / "esptool" / "esptool.py",
        )
        parts = temporary_dir / "parts"
        parts.mkdir()
        shutil.copy2(patched_bootloader, parts / "bootloader.bin")
        shutil.copy2(tx_build / "partitions.bin", parts / "partitions.bin")
        shutil.copy2(
            cache
            / "framework-arduinoespressif32"
            / "tools"
            / "partitions"
            / "boot_app0.bin",
            parts / "ota_data.bin",
        )
        shutil.copy2(tx_build / "firmware.bin", parts / "tx.bin")
        shutil.copy2(rx_build / "firmware.bin", parts / "rx.bin")

        merged_name = "aerialkit-rev-a3-elrs-dual-role.bin"
        merged = output / merged_name
        image_size, sizes = merge_image(
            parts / "bootloader.bin",
            parts / "partitions.bin",
            parts / "ota_data.bin",
            parts / "tx.bin",
            parts / "rx.bin",
            merged,
        )
        verify_app(
            expresslrs / "src" / "python" / "external" / "esptool" / "esptool.py",
            parts / "tx.bin",
            "TX",
        )
        verify_app(
            expresslrs / "src" / "python" / "external" / "esptool" / "esptool.py",
            parts / "rx.bin",
            "RX",
        )
        verify_patched_bootloader(
            parts / "bootloader.bin",
            selector,
            esptool=expresslrs / "src" / "python" / "external" / "esptool" / "esptool.py",
            image_class=image_class,
        )
        verify_image(merged, sizes, vendor=vendor)

        merged_bytes = merged.read_bytes()
        for offset, label in (
            (TX_APP_ADDR, "TX"),
            (RX_APP_ADDR, "RX"),
        ):
            if merged_bytes[offset : offset + 1] != b"\xe9":
                raise RuntimeError(f"merged image has no valid {label} application")

        manifest = {
            "artifact": merged_name,
            "flash_offset": hex(BOOTLOADER_ADDR),
            "flash_size": FLASH_SIZE,
            "image_size": image_size,
            "sha256": sha256(merged),
            "roles": {"gpio4_low": "tx", "gpio4_high": "rx"},
            "partitions": {
                "tx_app": hex(TX_APP_ADDR),
                "rx_app": hex(RX_APP_ADDR),
            },
            "applications": {
                "tx": {"size": sizes[TX_APP_ADDR], "sha256": sha256(parts / "tx.bin")},
                "rx": {"size": sizes[RX_APP_ADDR], "sha256": sha256(parts / "rx.bin")},
            },
            "bootloader": {
                "size": sizes[BOOTLOADER_ADDR],
                "sha256": sha256(parts / "bootloader.bin"),
            },
        }
        (output / "aerialkit-rev-a3-elrs-dual-role.json").write_text(
            json.dumps(manifest, indent=2) + "\n",
            encoding="utf-8",
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
