#!/usr/bin/env bash
#
# check-portable.sh - compile the portable core for another architecture.
#
#   scripts/check-portable.sh <toolchain-root> [output-dir] [source ...]
#
# src/core, src/core/flight and src/core/sensors are supposed to be free of the
# MCU: no arch header, no libc, no assumption about the instruction set. The
# cheapest way to keep that true is to compile them for another architecture
# now and then, and the goal's second target is an ESP32 - so the architectures
# to try are the two Espressif ones:
#
#   xtensa-esp-elf    ESP32, ESP32-S3
#   riscv32-esp-elf   ESP32-C3, ESP32-C6, ESP32-P4
#
# No link, no startup, no arch directory: this is a source-level check. It says
# nothing about whether anything runs on an ESP32, and it is exactly the check
# that catches a stray arch include in the flight core.
#
set -euo pipefail

root="${1:?usage: check-portable.sh <toolchain-root> [output-dir] [source ...]}"
out="${2:-build-port}"
shift 2 || true

if [ "$#" -gt 0 ]; then
    sources=("$@")
else
    sources=(src/core/*.c src/core/flight/*.c src/core/sensors/*.c)
fi

default_cflags="-std=c11 -O2 -ffreestanding -Wall -Wextra -Wshadow -Wconversion
                -Wdouble-promotion -DAK_BOARD=portable -DAK_PRODUCT=aerialkit-f405
                -Isrc/core -Isrc/core/flight -Isrc/core/sensors"
cflags="${PORT_CFLAGS:-$default_cflags}"

arches=(
    "xtensa-esp-elf:xtensa-esp32-elf-gcc:-mlongcalls"
    "riscv32-esp-elf:riscv32-esp32-elf-gcc:-march=rv32imc -mabi=ilp32"
)

# The riscv compiler is named riscv32-esp-elf-gcc, not riscv32-esp32-elf-gcc.
arches[1]="riscv32-esp-elf:riscv32-esp-elf-gcc:-march=rv32imc -mabi=ilp32"

failed_total=0
for entry in "${arches[@]}"; do
    IFS=: read -r dir cc flags <<< "$entry"
    compiler="$root/$dir/bin/$cc"
    if [ ! -x "$compiler" ]; then
        echo "port: $compiler not found; skipping $dir"
        continue
    fi

    mkdir -p "$out/$dir"
    count=0
    failed=0
    warned=0
    for src in "${sources[@]}"; do
        obj="$out/$dir/$(echo "$src" | tr / _).o"
        if ! $compiler $flags $cflags -c "$src" -o "$obj" 2>"$obj.log"; then
            failed=$((failed + 1))
            echo "  failed: $src"
            sed 's/^/      /' "$obj.log"
        elif [ -s "$obj.log" ]; then
            # A warning here is a warning nobody would otherwise see: the build
            # machines never compile this code for these architectures.
            warned=$((warned + 1))
            echo "  warning: $src"
            sed 's/^/      /' "$obj.log"
            rm -f "$obj.log"
        else
            rm -f "$obj.log"
        fi
        count=$((count + 1))
    done
    echo "port: $cc compiled $count files for $dir, $failed failed, $warned with warnings"
    failed_total=$((failed_total + failed))
done

exit $((failed_total > 0 ? 1 : 0))
