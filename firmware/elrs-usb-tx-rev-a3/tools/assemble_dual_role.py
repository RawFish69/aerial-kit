#!/usr/bin/env python3
"""Build and verify the four-slot Rev A3 ELRS factory image."""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import os
from pathlib import Path
import re
import struct
import subprocess
import sys
import tempfile


FLASH_SIZE = 8 * 1024 * 1024
BOOTLOADER_OFFSET = 0x0000
PARTITION_TABLE_OFFSET = 0x8000
OTA_DATA_OFFSET = 0xE000

BOOTLOADER_SELECTOR_ADDRESS = 0x403CC000

ROLE_RECORD_MAGIC = 0x414B5253
ROLE_STATE_SECTOR_SIZE = 0x1000
APPENDED_PRODUCT_SIZE = 128
APPENDED_DEVICE_SIZE = 16
APPENDED_OPTIONS_SIZE = 512
APPENDED_HARDWARE_SIZE = 2048
APPENDED_BLOCK_SIZE = (
    APPENDED_PRODUCT_SIZE
    + APPENDED_DEVICE_SIZE
    + APPENDED_OPTIONS_SIZE
    + APPENDED_HARDWARE_SIZE
)

SDK_MEMORY_TYPES = {
    "dio_qspi",
    "opi_opi",
    "opi_qspi",
    "qio_qspi",
}
FLASH_MODE_NAMES = {
    0: "qio",
    1: "qout",
    2: "dio",
    3: "dout",
}


def run(command: list[str], *, cwd: Path | None = None) -> None:
    print("+", " ".join(command))
    subprocess.run(command, cwd=cwd, check=True)


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def read_partitions(path: Path) -> dict[str, dict[str, int | str]]:
    rows: list[dict[str, int | str]] = []
    with path.open(newline="", encoding="utf-8") as source:
        reader = csv.reader(
            (line for line in source if not line.lstrip().startswith("#")),
            skipinitialspace=True,
        )
        for row in reader:
            if not row:
                continue
            rows.append(
                {
                    "name": row[0].strip(),
                    "type": row[1].strip(),
                    "subtype": row[2].strip(),
                    "offset": int(row[3], 0),
                    "size": int(row[4], 0),
                }
            )

    partitions = {str(row["name"]): row for row in rows}
    previous_end = 0
    for row in rows:
        offset = int(row["offset"])
        size = int(row["size"])
        if offset % ROLE_STATE_SECTOR_SIZE or size % ROLE_STATE_SECTOR_SIZE:
            raise RuntimeError(f"partition {row['name']} is not sector aligned")
        if offset < previous_end:
            raise RuntimeError(f"partition {row['name']} overlaps the previous row")
        previous_end = offset + size
    if previous_end > FLASH_SIZE:
        raise RuntimeError("partition table exceeds the 8 MB flash")
    return partitions


def verify_binary_partitions(
    path: Path,
    expected: dict[str, dict[str, int | str]],
) -> None:
    data = path.read_bytes()
    actual: dict[str, tuple[int, int]] = {}
    for offset in range(0, len(data), 32):
        entry = data[offset : offset + 32]
        if len(entry) < 32:
            break
        magic, _type, _subtype, address, size, label, _flags = struct.unpack(
            "<HBBII16sI", entry
        )
        if magic != 0x50AA:
            break
        name = label.split(b"\0", 1)[0].decode("ascii")
        actual[name] = (address, size)

    for name, row in expected.items():
        if actual.get(name) != (int(row["offset"]), int(row["size"])):
            raise RuntimeError(
                f"generated partition table disagrees with CSV for {name}: "
                f"{actual.get(name)} != ({row['offset']}, {row['size']})"
            )


def import_esptool(esptool: Path):
    sys.path.insert(0, str(esptool.resolve().parent))
    from esptool.bin_image import ESP32S3FirmwareImage

    return ESP32S3FirmwareImage


def default_esptool(vendor: Path) -> Path:
    configured = os.environ.get("ESPTOOLPY")
    if configured:
        return Path(configured)
    platformio = (
        Path.home()
        / ".platformio"
        / "packages"
        / "tool-esptoolpy"
        / "esptool.py"
    )
    if platformio.is_file():
        return platformio
    return (
        vendor
        / "ExpressLRS"
        / "src"
        / "python"
        / "external"
        / "esptool"
        / "esptool.py"
    )


def sdk_memory_type_from_map(map_path: Path) -> str:
    matches = set(
        re.findall(
            r"(?:^|[/\\])esp32s3[/\\]([a-z]+_[a-z]+)[/\\]",
            map_path.read_text(encoding="utf-8", errors="replace"),
        )
    ) & SDK_MEMORY_TYPES
    if len(matches) != 1:
        raise RuntimeError(
            f"{map_path} references SDK memory types {sorted(matches)}; expected one"
        )
    return matches.pop()


def bootloader_name_for_memory_type(memory_type: str) -> str:
    prefix = memory_type.split("_", 1)[0]
    if prefix not in {"dio", "opi", "qio"}:
        raise RuntimeError(f"unsupported ESP32-S3 SDK memory type: {memory_type}")
    return f"bootloader_{prefix}_80m.elf"


def bootloader_symbols(nm: Path, bootloader_elf: Path) -> dict[str, int]:
    output = subprocess.check_output([str(nm), "-n", str(bootloader_elf)], text=True)
    symbols: dict[str, int] = {}
    for line in output.splitlines():
        parts = line.split()
        if len(parts) >= 3:
            try:
                symbols[parts[-1]] = int(parts[0], 16)
            except ValueError:
                continue
    return symbols


def render_selector_linker_script(
    template: Path,
    output: Path,
    *,
    select_function: int,
    mmap_function: int,
    munmap_function: int,
) -> None:
    replacements = {
        "@BOOTLOADER_SELECT_FUNCTION@": f"0x{select_function:08x}",
        "@BOOTLOADER_MMAP@": f"0x{mmap_function:08x}",
        "@BOOTLOADER_MUNMAP@": f"0x{munmap_function:08x}",
    }
    rendered = template.read_text(encoding="utf-8")
    for token, value in replacements.items():
        if token not in rendered:
            raise RuntimeError(f"selector linker template is missing {token}")
        rendered = rendered.replace(token, value)
    output.write_text(rendered, encoding="utf-8")


def app_flash_mode(image_class, path: Path) -> str:
    with path.open("rb") as stream:
        image = image_class(stream)
    try:
        return FLASH_MODE_NAMES[image.flash_mode]
    except KeyError as exc:
        raise RuntimeError(
            f"{path} has unknown ESP image flash mode {image.flash_mode}"
        ) from exc


def verify_bootloader_source(
    *,
    bootloader_elf: Path,
    build_bootloader: Path,
    flash_mode: str,
    esptool: Path,
    output: Path,
) -> None:
    run(
        [
            sys.executable,
            str(esptool),
            "--chip",
            "esp32s3",
            "elf2image",
            "--flash_mode",
            flash_mode,
            "--flash_freq",
            "80m",
            "--flash_size",
            "8MB",
            "-o",
            str(output),
            str(bootloader_elf),
        ]
    )
    if output.read_bytes() != build_bootloader.read_bytes():
        raise RuntimeError(
            "bootloader/app flash mode mismatch: regenerating "
            f"{bootloader_elf.name} does not match {build_bootloader}"
        )


def compile_selector(
    cache: Path,
    source_dir: Path,
    build_dir: Path,
    *,
    linker_script: Path,
    select_function: int,
    select_segment: int,
) -> tuple[bytes, bytes]:
    toolchain = cache / "toolchain-xtensa-esp32s3" / "bin"
    prefix = toolchain / "xtensa-esp32s3-elf-"
    gcc = str(prefix) + "gcc"
    objcopy = str(prefix) + "objcopy"
    if not Path(gcc).is_file():
        raise FileNotFoundError(f"ESP32-S3 compiler not found: {gcc}")

    selector_elf = build_dir / "selector.elf"
    run(
        [
            gcc,
            "-Os",
            "-nostdlib",
            "-ffreestanding",
            "-fno-builtin",
            "-fno-stack-protector",
            "-fno-exceptions",
            "-mlongcalls",
            "-mtext-section-literals",
            "-Wl,--build-id=none",
            f"-Wl,-T,{linker_script}",
            "-Wl,--gc-sections",
            "-o",
            str(selector_elf),
            str(source_dir / "selector_jump.S"),
            str(source_dir / "selector.c"),
        ]
    )

    trampoline_path = build_dir / "trampoline.bin"
    selector_path = build_dir / "selector.bin"
    run(
        [
            objcopy,
            "--dump-section",
            f".trampoline={trampoline_path}",
            str(selector_elf),
        ]
    )
    run(
        [
            objcopy,
            "--dump-section",
            f".selector={selector_path}",
            str(selector_elf),
        ]
    )
    trampoline = trampoline_path.read_bytes()
    selector = selector_path.read_bytes()
    nm_output = subprocess.check_output(
        [str(prefix) + "nm", "-n", str(selector_elf)],
        text=True,
    ).splitlines()
    symbols = {
        parts[-1]: int(parts[0], 16)
        for line in nm_output
        if (parts := line.split())
        and len(parts) >= 3
    }
    if symbols.get("bootloader_utility_get_selected_boot_partition") != select_function:
        raise RuntimeError("bootloader trampoline moved outside its reserved address")
    selector_function = symbols.get("aerialkit_select_boot_partition")
    if selector_function is None or not (
        BOOTLOADER_SELECTOR_ADDRESS
        <= selector_function
        < BOOTLOADER_SELECTOR_ADDRESS + len(selector)
    ):
        raise RuntimeError("selector function moved outside its reserved segment")
    if len(trampoline) != 3:
        raise RuntimeError(f"selector trampoline is {len(trampoline)} bytes")
    if not selector:
        raise RuntimeError("selector image is empty")
    if (
        BOOTLOADER_SELECTOR_ADDRESS + len(selector)
        > select_segment
    ):
        raise RuntimeError("selector does not fit in the bootloader IRAM gap")
    return trampoline, selector


def patch_bootloader(
    bootloader_elf: Path,
    trampoline: bytes,
    selector: bytes,
    output_path: Path,
    *,
    toolchain: Path,
    esptool: Path,
    select_function: int,
    select_segment: int,
    flash_mode: str,
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
    function_offset = select_function - select_segment
    if function_offset < 0 or function_offset + len(trampoline) > len(loader):
        raise RuntimeError("bootloader selector function is outside its IRAM section")
    if loader[function_offset : function_offset + len(trampoline)] == trampoline:
        raise RuntimeError("bootloader is already dual-role patched")
    if loader[function_offset : function_offset + 3] != bytes.fromhex("36c100"):
        raise RuntimeError(
            "pinned bootloader selector entry changed; refusing an unknown build"
        )
    loader[function_offset : function_offset + len(trampoline)] = trampoline
    loader_section.write_bytes(loader)

    selector_file = output_path.with_suffix(".selector.bin")
    selector_file.write_bytes(selector)
    patched_elf = output_path.with_suffix(".elf")
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
            f".aerialkit_selector=0x{BOOTLOADER_SELECTOR_ADDRESS:x}",
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
            flash_mode,
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
    selector_file.unlink(missing_ok=True)
    patched_elf.unlink(missing_ok=True)


def role_record_crc(prefix: bytes) -> int:
    value = 2166136261
    for byte in prefix:
        value ^= byte
        value = (value * 16777619) & 0xFFFFFFFF
    return value


def make_role_state(size: int) -> bytes:
    prefix = struct.pack("<IIBBH", ROLE_RECORD_MAGIC, 1, 0, 2, 0)
    record = prefix + struct.pack("<I", role_record_crc(prefix))
    state = bytearray(b"\xff" * size)
    state[0 : len(record)] = record
    state[ROLE_STATE_SECTOR_SIZE : ROLE_STATE_SECTOR_SIZE + len(record)] = record
    return bytes(state)


def make_littlefs(mklittlefs: Path, size: int, build_dir: Path) -> Path:
    if not mklittlefs.is_file():
        raise FileNotFoundError(f"mklittlefs not found: {mklittlefs}")
    empty_dir = build_dir / "littlefs-empty"
    empty_dir.mkdir()
    image = build_dir / "littlefs.bin"
    run(
        [
            str(mklittlefs),
            "-c",
            str(empty_dir),
            "-b",
            "4096",
            "-p",
            "256",
            "-s",
            str(size),
            str(image),
        ]
    )
    if image.stat().st_size != size:
        raise RuntimeError(
            f"LittleFS image is {image.stat().st_size} bytes, expected {size}"
        )
    return image


def merge_image(
    bootloader: Path,
    partitions: Path,
    ota_data: Path,
    tx: Path,
    rx: Path,
    role_state: bytes,
    littlefs: Path,
    partition_rows: dict[str, dict[str, int | str]],
    output: Path,
) -> tuple[int, dict[int, int]]:
    if tx.stat().st_size > int(partition_rows["tx_0"]["size"]):
        raise RuntimeError("TX application overflows tx_0")
    if rx.stat().st_size > int(partition_rows["rx_0"]["size"]):
        raise RuntimeError("RX application overflows rx_0")
    if len(role_state) != int(partition_rows["role_state"]["size"]):
        raise RuntimeError("role_state image has the wrong size")
    if littlefs.stat().st_size != int(partition_rows["spiffs"]["size"]):
        raise RuntimeError("LittleFS image has the wrong size")

    pieces: dict[int, Path | bytes] = {
        BOOTLOADER_OFFSET: bootloader,
        PARTITION_TABLE_OFFSET: partitions,
        OTA_DATA_OFFSET: ota_data,
        int(partition_rows["tx_0"]["offset"]): tx,
        int(partition_rows["rx_0"]["offset"]): rx,
        int(partition_rows["role_state"]["offset"]): role_state,
        int(partition_rows["spiffs"]["offset"]): littlefs,
    }
    image_end = (
        int(partition_rows["spiffs"]["offset"])
        + int(partition_rows["spiffs"]["size"])
    )
    merged = bytearray(b"\xff" * image_end)
    sizes: dict[int, int] = {}
    for offset, piece in pieces.items():
        data = piece.read_bytes() if isinstance(piece, Path) else piece
        merged[offset : offset + len(data)] = data
        sizes[offset] = len(data)

    output.write_bytes(merged)
    return image_end, sizes


def verify_patched_bootloader(
    path: Path,
    trampoline: bytes,
    selector: bytes,
    *,
    esptool: Path,
    image_class,
    select_function: int,
    select_segment: int,
) -> None:
    with path.open("rb") as stream:
        image = image_class(stream)
    selector_segment = next(
        (segment for segment in image.segments if segment.addr == BOOTLOADER_SELECTOR_ADDRESS),
        None,
    )
    loader_segment = next(
        (segment for segment in image.segments if segment.addr == select_segment),
        None,
    )
    if selector_segment is None or loader_segment is None:
        raise RuntimeError("patched bootloader has an unexpected segment map")
    if selector_segment.data[: len(selector)] != selector:
        raise RuntimeError("bootloader does not contain the four-slot selector")
    function_offset = select_function - loader_segment.addr
    if loader_segment.data[
        function_offset : function_offset + len(trampoline)
    ] != trampoline:
        raise RuntimeError("bootloader selector function was not replaced")
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


def verify_app(esptool: Path, path: Path, label: str) -> None:
    if path.read_bytes()[:1] != b"\xe9":
        raise RuntimeError(f"{label} is not an ESP application image")
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


def firmware_application_end(path: Path) -> int:
    data = path.read_bytes()
    if len(data) < 24 or data[0] != 0xE9:
        raise RuntimeError(f"{path} is not an ESP application image")
    position = 24
    for _ in range(data[1]):
        if position + 8 > len(data):
            raise RuntimeError(f"{path} has a truncated segment table")
        _address, size = struct.unpack_from("<II", data, position)
        position += 8 + size
        if position > len(data):
            raise RuntimeError(f"{path} has a truncated image segment")
    return ((position + 16) & ~15) + 32


def verify_app_trailer(
    path: Path,
    *,
    product_name: str,
    hardware_layout: dict[str, object],
) -> int:
    data = path.read_bytes()
    image_end = firmware_application_end(path)
    if image_end + APPENDED_BLOCK_SIZE != len(data):
        raise RuntimeError(
            f"{path} trailer is {len(data) - image_end} bytes, "
            f"expected {APPENDED_BLOCK_SIZE}"
        )
    product = (
        data[image_end : image_end + APPENDED_PRODUCT_SIZE]
        .split(b"\0", 1)[0]
        .decode("utf-8")
    )
    hardware_offset = (
        image_end
        + APPENDED_PRODUCT_SIZE
        + APPENDED_DEVICE_SIZE
        + APPENDED_OPTIONS_SIZE
    )
    try:
        hardware = json.loads(
            data[
                hardware_offset : hardware_offset + APPENDED_HARDWARE_SIZE
            ]
            .split(b"\0", 1)[0]
            .decode("utf-8")
        )
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise RuntimeError(f"{path} has an invalid hardware layout trailer") from exc
    if product != product_name:
        raise RuntimeError(
            f"{path} product trailer is {product!r}, expected {product_name!r}"
        )
    if hardware != hardware_layout:
        raise RuntimeError(f"{path} hardware layout trailer disagrees with target")
    return image_end


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--root", type=Path, required=True)
    parser.add_argument("--cache", type=Path, required=True)
    parser.add_argument("--vendor", type=Path, required=True)
    parser.add_argument("--mklittlefs", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--esptool", type=Path)
    args = parser.parse_args()

    root = args.root.resolve()
    cache = args.cache.resolve()
    vendor = args.vendor.resolve()
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)

    partition_rows = read_partitions(root / "partitions" / "dual-role.csv")
    expresslrs = vendor / "ExpressLRS"
    tx_build = expresslrs / "src" / ".pio" / "build" / "AERIALKIT_ELRS_USB_TX_REV_A3_via_UART"
    rx_build = (
        expresslrs / "src" / ".pio" / "build" / "AERIALKIT_ELRS_USB_TX_REV_A3_RX_via_UART"
    )
    for build in (tx_build, rx_build):
        for name in ("firmware.bin", "firmware.map", "partitions.bin", "bootloader.bin"):
            if not (build / name).is_file():
                raise FileNotFoundError(build / name)
    if (tx_build / "partitions.bin").read_bytes() != (
        rx_build / "partitions.bin"
    ).read_bytes():
        raise RuntimeError("TX and RX builds have different partition tables")
    if (tx_build / "bootloader.bin").read_bytes() != (
        rx_build / "bootloader.bin"
    ).read_bytes():
        raise RuntimeError("TX and RX builds have different bootloaders")
    verify_binary_partitions(tx_build / "partitions.bin", partition_rows)

    esptool = (args.esptool or default_esptool(vendor)).resolve()
    if not esptool.is_file():
        raise FileNotFoundError(esptool)
    image_class = import_esptool(esptool)
    sdk_memory_types = {
        sdk_memory_type_from_map(tx_build / "firmware.map"),
        sdk_memory_type_from_map(rx_build / "firmware.map"),
    }
    if len(sdk_memory_types) != 1:
        raise RuntimeError(
            f"TX and RX use different SDK memory types: {sorted(sdk_memory_types)}"
        )
    sdk_memory_type = sdk_memory_types.pop()
    bootloader_name = bootloader_name_for_memory_type(sdk_memory_type)
    bootloader_elf = (
        cache
        / "framework-arduinoespressif32"
        / "tools"
        / "sdk"
        / "esp32s3"
        / "bin"
        / bootloader_name
    )
    if not bootloader_elf.is_file():
        raise FileNotFoundError(bootloader_elf)

    tx_flash_mode = app_flash_mode(image_class, tx_build / "firmware.bin")
    rx_flash_mode = app_flash_mode(image_class, rx_build / "firmware.bin")
    if tx_flash_mode != rx_flash_mode:
        raise RuntimeError(
            f"TX and RX application flash modes differ: "
            f"{tx_flash_mode!r} != {rx_flash_mode!r}"
        )
    source_bootloader_flash_mode = tx_flash_mode
    toolchain_prefix = cache / "toolchain-xtensa-esp32s3" / "bin" / "xtensa-esp32s3-elf-"
    symbols = bootloader_symbols(Path(str(toolchain_prefix) + "nm"), bootloader_elf)
    required_symbols = {
        "bootloader_utility_get_selected_boot_partition",
        "bootloader_mmap",
        "bootloader_munmap",
        "bootloader_iram_loader_seg_start",
    }
    missing_symbols = required_symbols - symbols.keys()
    if missing_symbols:
        raise RuntimeError(
            f"{bootloader_name} is missing symbols {sorted(missing_symbols)}"
        )
    select_function = symbols["bootloader_utility_get_selected_boot_partition"]
    select_segment = symbols["bootloader_iram_loader_seg_start"]
    mmap_function = symbols["bootloader_mmap"]
    munmap_function = symbols["bootloader_munmap"]

    with tempfile.TemporaryDirectory(prefix="aerialkit-four-slot-") as temporary:
        temp = Path(temporary)
        verify_bootloader_source(
            bootloader_elf=bootloader_elf,
            build_bootloader=tx_build / "bootloader.bin",
            flash_mode=source_bootloader_flash_mode,
            esptool=esptool,
            output=temp / "unpatched-bootloader.bin",
        )
        linker_script = temp / "selector.ld"
        render_selector_linker_script(
            root / "bootloader" / "selector.ld",
            linker_script,
            select_function=select_function,
            mmap_function=mmap_function,
            munmap_function=munmap_function,
        )
        trampoline, selector = compile_selector(
            cache,
            root / "bootloader",
            temp,
            linker_script=linker_script,
            select_function=select_function,
            select_segment=select_segment,
        )
        patched_bootloader = temp / "bootloader.bin"
        patch_bootloader(
            bootloader_elf,
            trampoline,
            selector,
            patched_bootloader,
            toolchain=cache / "toolchain-xtensa-esp32s3" / "bin",
            esptool=esptool,
            select_function=select_function,
            select_segment=select_segment,
            flash_mode=source_bootloader_flash_mode,
        )
        verify_patched_bootloader(
            patched_bootloader,
            trampoline,
            selector,
            esptool=esptool,
            image_class=image_class,
            select_function=select_function,
            select_segment=select_segment,
        )

        role_state = make_role_state(int(partition_rows["role_state"]["size"]))
        littlefs = make_littlefs(
            args.mklittlefs.resolve(),
            int(partition_rows["spiffs"]["size"]),
            temp,
        )
        tx = tx_build / "firmware.bin"
        rx = rx_build / "firmware.bin"
        verify_app(esptool, tx, "TX")
        verify_app(esptool, rx, "RX")
        targets = expresslrs / "src" / "hardware"
        target_products = {
            "tx": "Aerial Kit ELRS USB TX Rev A3 2.4GHz TX",
            "rx": "Aerial Kit ELRS USB TX Rev A3 2.4GHz RX",
        }
        target_layouts = {
            role: json.loads(
                (
                    targets
                    / role.upper()
                    / "AerialKit ELRS USB TX Rev A3.json"
                ).read_text(encoding="utf-8")
            )
            for role in target_products
        }
        app_image_ends = {
            "tx": verify_app_trailer(
                tx,
                product_name=target_products["tx"],
                hardware_layout=target_layouts["tx"],
            ),
            "rx": verify_app_trailer(
                rx,
                product_name=target_products["rx"],
                hardware_layout=target_layouts["rx"],
            ),
        }

        merged = output / "aerialkit-rev-a3-elrs-dual-role.bin"
        image_size, sizes = merge_image(
            patched_bootloader,
            tx_build / "partitions.bin",
            cache
            / "framework-arduinoespressif32"
            / "tools"
            / "partitions"
            / "boot_app0.bin",
            tx,
            rx,
            role_state,
            littlefs,
            partition_rows,
            merged,
        )
        merged_bytes = merged.read_bytes()
        if merged_bytes[int(partition_rows["tx_0"]["offset"]) :][:1] != b"\xe9":
            raise RuntimeError("tx_0 does not contain the TX image")
        if merged_bytes[int(partition_rows["rx_0"]["offset"]) :][:1] != b"\xe9":
            raise RuntimeError("rx_0 does not contain the RX image")

        manifest = {
            "artifact": merged.name,
            "flash_offset": hex(BOOTLOADER_OFFSET),
            "flash_size": FLASH_SIZE,
            "image_size": image_size,
            "sha256": sha256(merged),
            "roles": {"gpio4_low": "rx", "gpio4_high": "tx"},
            "partitions": {
                name: {
                    "offset": hex(int(row["offset"])),
                    "size": int(row["size"]),
                }
                for name, row in partition_rows.items()
            },
            "applications": {
                "tx": {
                    "slot": "tx_0",
                    "size": sizes[int(partition_rows["tx_0"]["offset"])],
                    "image_size": app_image_ends["tx"],
                    "sha256": sha256(tx),
                },
                "rx": {
                    "slot": "rx_0",
                    "size": sizes[int(partition_rows["rx_0"]["offset"])],
                    "image_size": app_image_ends["rx"],
                    "sha256": sha256(rx),
                },
            },
            "bootloader": {
                "elf": bootloader_name,
                "sdk_memory_type": sdk_memory_type,
                "app_flash_mode": tx_flash_mode,
                "flash_mode": source_bootloader_flash_mode,
                "size": sizes[BOOTLOADER_OFFSET],
                "sha256": sha256(patched_bootloader),
            },
            "littlefs": {
                "offset": partition_rows["spiffs"]["offset"],
                "size": int(partition_rows["spiffs"]["size"]),
                "sha256": sha256(littlefs),
            },
            "role_state": {
                "offset": partition_rows["role_state"]["offset"],
                "initial_tx_slot": "tx_0",
                "initial_rx_slot": "rx_0",
            },
        }
        (output / "aerialkit-rev-a3-elrs-dual-role.json").write_text(
            json.dumps(manifest, indent=2) + "\n",
            encoding="utf-8",
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
