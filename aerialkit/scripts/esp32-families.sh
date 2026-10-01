#!/usr/bin/env bash
#
# esp32-families.sh - build AerialKit for every chip in the ESP32 family.
#
# The port has one IDF project and one set of portable sources; what changes per
# chip is the board file (its pins, its LED, how many outputs the chip can
# drive) and the sdkconfig fragment that names the target. This script is that
# mapping in one place, because the alternative is a person remembering four
# command lines with four different build directories.
#
#   scripts/esp32-families.sh            # every board that has a file
#   scripts/esp32-families.sh ESP32S3DEV # one of them
#
# A chip whose toolchain is not installed is *skipped with a line* rather than
# failing: this is a build of four different architectures, and a machine that
# can do three of them is a machine that can do three of them.
#
set -euo pipefail

root="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")/.." && pwd)"
idf="${IDF_PATH:-$HOME/esp-idf}"

[ -f "$idf/export.sh" ] || {
    echo "esp32-families: no ESP-IDF at $idf (set IDF_PATH)" >&2
    exit 1
}

# shellcheck disable=SC1090
. "$idf/export.sh" >/dev/null

# board, the IDF target, the sdkconfig fragment, and the QEMU machine this
# machine's QEMU has for it (empty means "builds here, cannot be run here").
#
# The toolchain column is the compiler IDF needs: the S2 and S3 are Xtensa like
# the ESP32, the C3 is RISC-V, and the RISC-V toolchain is not installed on this
# NAS - which is why the C3 is the one that reports "skipped" here while the
# other three build. See docs/17-esp32-port.md for what each chip costs.
boards=(
    "ESP32DEV:esp32:sdkconfig.defaults:esp32:xtensa-esp32-elf-gcc"
    "ESP32S3DEV:esp32s3:sdkconfig.s3:esp32s3:xtensa-esp32-elf-gcc"
    "ESP32S2DEV:esp32s2:sdkconfig.s2::xtensa-esp32-elf-gcc"
    "ESP32C3DEV:esp32c3:sdkconfig.c3::riscv32-esp-elf-gcc"
)

want="${1:-}"

for row in "${boards[@]}"; do
    IFS=: read -r board target defaults machine cc <<<"$row"
    [ -z "$want" ] || [ "$want" = "$board" ] || continue

    if [ ! -f "$root/src/boards/$board/board.c" ]; then
        echo "== $board: no board file yet - skipped"
        continue
    fi

    if ! command -v "$cc" >/dev/null; then
        # `export.sh` puts the toolchains for the *default* target on PATH -
        # which is Xtensa - so the RISC-V compiler can be installed and not
        # exported at all. Look for it where IDF keeps it, the way this
        # workspace's own scripts find their ARM toolchain.
        tool_bin=$(find "$HOME/.espressif/tools/${cc%-gcc}" -type d -name bin \
                        2>/dev/null | head -1)
        [ -n "$tool_bin" ] && PATH="$tool_bin:$PATH"
    fi

    if ! command -v "$cc" >/dev/null; then
        echo "== $board: no $cc on PATH - skipped (this chip is built elsewhere)"
        continue
    fi

    dir="ports/esp32/build-${target}"
    echo "== $board ($target): building"
    # SDKCONFIG is relative to the *project* directory, which -C changes into;
    # -B is relative to where this script was run. Getting that backwards
    # produced "ports/esp32/ports/esp32/build-.../sdkconfig" the first time.
    idf.py -C ports/esp32 -B "$dir" -DSDKCONFIG="build-${target}/sdkconfig" \
        -DAK_BOARD="$board" -DSDKCONFIG_DEFAULTS="sdkconfig.defaults;$defaults" \
        build >/dev/null

    bin="$dir/aerialkit-esp32.bin"
    echo "   $(stat -c%s "$bin") bytes"

    if [ -z "$machine" ]; then
        echo "   and this machine's QEMU has no $target machine: built, not run"
        continue
    fi

    seconds="${AK_QEMU_SECONDS:-8}"
    echo "   running it under QEMU for ${seconds}s"
    timeout --signal=INT "$seconds" idf.py -C ports/esp32 -B "$dir" \
        -DSDKCONFIG="build-${target}/sdkconfig" qemu 2>&1 | \
        sed -n '/AerialKit Firmware/,/^$/p' | head -12 || true
done

echo "esp32-families: done"
